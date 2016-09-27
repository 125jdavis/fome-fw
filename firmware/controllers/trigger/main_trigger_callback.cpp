/**
 * @file    main_trigger_callback.cpp
 * @brief   Main logic is here!
 *
 * See http://rusefi.com/docs/html/
 *
 * @date Feb 7, 2013
 * @author Andrey Belomutskiy, (c) 2012-2016
 *
 * This file is part of rusEfi - see http://rusefi.com
 *
 * rusEfi is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * rusEfi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "main.h"
#if EFI_PROD_CODE || defined(__DOXYGEN__)
#include <nvic.h>
#endif

#if (!EFI_PROD_CODE && !EFI_SIMULATOR) || defined(__DOXYGEN__)

#define chThdSelf() 0
#define getRemainingStack(x) (999999)

#endif

#if EFI_ENGINE_CONTROL || defined(__DOXYGEN__)

#include "main_trigger_callback.h"
#include "efiGpio.h"
#include "engine_math.h"
#include "trigger_central.h"
#include "spark_logic.h"
#include "rpm_calculator.h"
#include "signal_executor.h"
#include "engine_configuration.h"
#include "interpolation.h"
#include "advance_map.h"
#include "allsensors.h"
#include "cyclic_buffer.h"
#include "histogram.h"
#include "fuel_math.h"
#include "histogram.h"
#include "efiGpio.h"
#if EFI_PROD_CODE || defined(__DOXYGEN__)
#include "rfiutil.h"
#endif /* EFI_HISTOGRAMS */
#include "LocalVersionHolder.h"
#include "event_queue.h"
#include "engine.h"
#include "efilib2.h"

EXTERN_ENGINE
;
extern bool hasFirmwareErrorFlag;

static LocalVersionHolder triggerVersion;
static const char *prevOutputName = NULL;

static Logging *logger;

// todo: figure out if this even helps?
//#if defined __GNUC__
//#define RAM_METHOD_PREFIX __attribute__((section(".ram")))
//#else
//#define RAM_METHOD_PREFIX
//#endif

static void startSimultaniousInjection(Engine *engine) {
	for (int i = 0; i < engine->engineConfiguration->specs.cylindersCount; i++) {
		turnPinHigh(&enginePins.injectors[i]);
	}
}

static void endSimultaniousInjection(Engine *engine) {
	for (int i = 0; i < engine->engineConfiguration->specs.cylindersCount; i++) {
		turnPinLow(&enginePins.injectors[i]);
	}
}

static void scheduleFuelInjection(int rpm, int injEventIndex, OutputSignal *signal, efitimeus_t nowUs, floatus_t delayUs, floatus_t durationUs, InjectorOutputPin *output DECLARE_ENGINE_PARAMETER_S) {
	if (durationUs < 0) {
		warning(CUSTOM_OBD_3, "duration cannot be negative: %d", durationUs);
		return;
	}
	if (cisnan(durationUs)) {
		warning(CUSTOM_OBD_4, "NaN in scheduleFuelInjection", durationUs);
		return;
	}
#if EFI_PRINTF_FUEL_DETAILS || defined(__DOXYGEN__)
	printf("fuelout %s duration %d total=%d\t\n", output->name, (int)durationUs,
			(int)MS2US(getCrankshaftRevolutionTimeMs(rpm)));
#endif /*EFI_PRINTF_FUEL_DETAILS */

	efiAssertVoid(signal!=NULL, "signal is NULL");
	int index = getRevolutionCounter() % 2;
	scheduling_s * sUp = &signal->signalTimerUp[index];
	scheduling_s * sDown = &signal->signalTimerDown[index];

	efitimeus_t turnOnTime = nowUs + (int) delayUs;
	bool isSecondaryOverlapping = turnOnTime < output->overlappingScheduleOffTime;

	if (isSecondaryOverlapping) {
		output->cancelNextTurningInjectorOff = true;
#if EFI_UNIT_TEST || EFI_SIMULATOR || defined(__DOXYGEN__)
	printf("please cancel %s %d %d\r\n", output->name, (int)getTimeNowUs(), output->overlappingCounter);
#endif /* EFI_UNIT_TEST || EFI_SIMULATOR */
	} else {
		seScheduleByTime("out up", sUp, turnOnTime, (schfunc_t) &seTurnPinHigh, output);
	}
	efitimeus_t turnOffTime = nowUs + (int) (delayUs + durationUs);
	seScheduleByTime("out down", sDown, turnOffTime, (schfunc_t) &seTurnPinLow, output);
}

static ALWAYS_INLINE void handleFuelInjectionEvent(int injEventIndex, InjectionEvent *event,
		int rpm DECLARE_ENGINE_PARAMETER_S) {

	/**
	 * todo: this is a bit tricky with batched injection. is it? Does the same
	 * wetting coefficient works the same way for any injection mode, or is something
	 * x2 or /2?
	 */
	const floatms_t injectionDuration = ENGINE(wallFuel).adjust(event->injectorIndex, ENGINE(fuelMs) PASS_ENGINE_PARAMETER);
#if EFI_PRINTF_FUEL_DETAILS || defined(__DOXYGEN__)
	printf("fuel fuelMs=%f adjusted=%f\t\n", ENGINE(fuelMs), injectionDuration);
#endif /*EFI_PRINTF_FUEL_DETAILS */

	if (injectionDuration > getCrankshaftRevolutionTimeMs(rpm)) {
		warning(CUSTOM_OBD_50, "Too long fuel injection");
	}

	// todo: pre-calculate 'numberOfInjections'
	floatms_t totalPerCycle = injectionDuration * getNumberOfInjections(engineConfiguration->injectionMode PASS_ENGINE_PARAMETER);
	floatus_t engineCycleDuration = engine->rpmCalculator.oneDegreeUs * engine->engineCycle;
	if (MS2US(totalPerCycle) > engineCycleDuration) {
		warning(CUSTOM_OBD_26, "injector duty cycle too high %fms @ %d", totalPerCycle,
				getRevolutionCounter());
	}

	ENGINE(actualLastInjection) = injectionDuration;
	if (cisnan(injectionDuration)) {
		warning(CUSTOM_OBD_NAN_INJECTION, "NaN injection pulse");
		return;
	}
	if (injectionDuration < 0) {
		warning(CUSTOM_OBD_NEG_INJECTION, "Negative injection pulse %f", injectionDuration);
		return;
	}

#if FUEL_MATH_EXTREME_LOGGING || defined(__DOXYGEN__)
	scheduleMsg(logger, "handleFuel totalPerCycle=%f", totalPerCycle);
	scheduleMsg(logger, "handleFuel engineCycleDuration=%f", engineCycleDuration);
#endif /* FUEL_MATH_EXTREME_LOGGING */

	floatus_t injectionStartDelayUs = ENGINE(rpmCalculator.oneDegreeUs) * event->injectionStart.angleOffset;

#if EFI_DEFAILED_LOGGING || defined(__DOXYGEN__)
	scheduleMsg(logger, "handleFuel pin=%s eventIndex %d duration=%fms %d", event->output->name,
			eventIndex,
			injectionDuration,
			getRevolutionCounter());
	scheduleMsg(logger, "handleFuel pin=%s delay=%f %d", event->output->name, injectionStartDelayUs,
			getRevolutionCounter());
#endif /* EFI_DEFAILED_LOGGING */

	OutputSignal *signal = &ENGINE(engineConfiguration2)->fuelActuators[injEventIndex];

	engine->engineConfiguration2->wasOverlapping[injEventIndex] = event->isOverlapping;

	if (event->isSimultanious) {
		/**
		 * this is pretty much copy-paste of 'scheduleOutput'
		 * 'scheduleOutput' is currently only used for injection, so maybe it should be
		 * changed into 'scheduleInjection' and unified? todo: think about it.
		 */
		efiAssertVoid(signal!=NULL, "signal is NULL");
		int index = getRevolutionCounter() % 2;
		scheduling_s * sUp = &signal->signalTimerUp[index];
		scheduling_s * sDown = &signal->signalTimerDown[index];

		scheduleTask("out up", sUp, (int) injectionStartDelayUs, (schfunc_t) &startSimultaniousInjection, engine);
		scheduleTask("out down", sDown, (int) injectionStartDelayUs + MS2US(injectionDuration),
					(schfunc_t) &endSimultaniousInjection, engine);

	} else {
#if EFI_UNIT_TEST || defined(__DOXYGEN__)
		printf("scheduling injection angle=%f/delay=%f injectionDuration=%f\r\n", event->injectionStart.angleOffset, injectionStartDelayUs, injectionDuration);
#endif

		// we are in this branch of code only in case of NOT IM_SIMULTANEOUS injection
		// we are ignoring low RPM in order not to handle "engine was stopped to engine now running" transition
		if (rpm > 2 * engineConfiguration->cranking.rpm) {
			const char *outputName = event->output->name;
			if (prevOutputName == outputName) {
				warning(CUSTOM_OBD_SKIPPED_FUEL, "looks like skipped fuel event %d %s", getRevolutionCounter(), outputName);
			}
			prevOutputName = outputName;
		}

		scheduleFuelInjection(rpm, injEventIndex, signal, getTimeNowUs(), injectionStartDelayUs, MS2US(injectionDuration), event->output PASS_ENGINE_PARAMETER);
	}
}

static void handleFuelScheduleOverlap(InjectionEventList *injectionEvents DECLARE_ENGINE_PARAMETER_S) {
	/**
	 * here we need to avoid a fuel miss due to changes between previous and current fuel schedule
	 * see https://sourceforge.net/p/rusefi/tickets/299/
	 * see testFuelSchedulerBug299smallAndLarge unit test
	 */
	//
	for (int injEventIndex = 0; injEventIndex < injectionEvents->size; injEventIndex++) {
		InjectionEvent *event = &injectionEvents->elements[injEventIndex];
		if (!engine->engineConfiguration2->wasOverlapping[injEventIndex] && event->isOverlapping) {
			// we are here if new fuel schedule is crossing engine cycle boundary with this event

			InjectorOutputPin *output = &enginePins.injectors[event->injectorIndex];

			// todo: recalc fuel? account for wetting?
			floatms_t injectionDuration = ENGINE(fuelMs);

			scheduling_s * sUp = &ENGINE(engineConfiguration2)->overlappingFuelActuatorTimerUp[injEventIndex];
			scheduling_s * sDown = &ENGINE(engineConfiguration2)->overlappingFuelActuatorTimerDown[injEventIndex];

			efitimeus_t nowUs = getTimeNowUs();

			output->overlappingScheduleOffTime = nowUs + MS2US(injectionDuration);

			scheduleOutput2(sUp, sDown, nowUs, 0, MS2US(injectionDuration), output);
		}
	}
}

static ALWAYS_INLINE void handleFuel(const bool limitedFuel, uint32_t trgEventIndex, int rpm DECLARE_ENGINE_PARAMETER_S) {
	efiAssertVoid(getRemainingStack(chThdSelf()) > 128, "lowstck#3");
	efiAssertVoid(trgEventIndex < ENGINE(triggerShape.getLength()), "handleFuel/event index");

	if (!isInjectionEnabled(engineConfiguration) || limitedFuel) {
		return;
	}
	if (engine->isCylinderCleanupMode) {
		return;
	}


	/**
	 * Ignition events are defined by addFuelEvents() according to selected
	 * fueling strategy
	 */
	FuelSchedule *fs = engine->fuelScheduleForThisEngineCycle;
	InjectionEventList *injectionEvents = &fs->injectionEvents;

	if (trgEventIndex == 0) {
		handleFuelScheduleOverlap(injectionEvents PASS_ENGINE_PARAMETER);
	}

	if (!fs->hasEvents[trgEventIndex]) {
		// that's a performance optimization
		return;
	}

#if FUEL_MATH_EXTREME_LOGGING || defined(__DOXYGEN__)
	scheduleMsg(logger, "handleFuel ind=%d %d", trgEventIndex, getRevolutionCounter());
#endif /* FUEL_MATH_EXTREME_LOGGING */

	ENGINE(tpsAccelEnrichment.onNewValue(getTPS(PASS_ENGINE_PARAMETER_F) PASS_ENGINE_PARAMETER));
	ENGINE(engineLoadAccelEnrichment.onEngineCycle(PASS_ENGINE_PARAMETER_F));

	ENGINE(fuelMs) = getInjectionDuration(rpm PASS_ENGINE_PARAMETER) * CONFIG(globalFuelCorrection);

	for (int injEventIndex = 0; injEventIndex < injectionEvents->size; injEventIndex++) {
		InjectionEvent *event = &injectionEvents->elements[injEventIndex];
		uint32_t eventIndex = event->injectionStart.eventIndex;
// right after trigger change we are still using old & invalid fuel schedule. good news is we do not change trigger on the fly in real life
//		efiAssertVoid(eventIndex < ENGINE(triggerShape.getLength()), "handleFuel/event sch index");
		if (eventIndex != trgEventIndex) {
			continue;
		}
		handleFuelInjectionEvent(injEventIndex, event, rpm PASS_ENGINE_PARAMETER);
	}
}

static histogram_s mainLoopHisto;

void showMainHistogram(void) {
#if EFI_PROD_CODE || defined(__DOXYGEN__)
	printHistogram(logger, &mainLoopHisto);
#endif
}

// todo: the method name is not correct any more - no calc is done here anymore
static ALWAYS_INLINE void ignitionMathCalc(int rpm DECLARE_ENGINE_PARAMETER_S) {
	/**
	 * Within one engine cycle all cylinders are fired with same timing advance.
	 * todo: one day we can control cylinders individually?
	 */
	float dwellMs = ENGINE(engineState.sparkDwell);

	if (cisnan(dwellMs) || dwellMs < 0) {
		firmwareError("invalid dwell: %f at %d", dwellMs, rpm);
		return;
	}
}

#if EFI_PROD_CODE || defined(__DOXYGEN__)
/**
 * this field is used as an Expression in IAR debugger
 */
uint32_t *cyccnt = (uint32_t*) &DWT->CYCCNT;
#endif

/**
 * This is the main trigger event handler.
 * Both injection and ignition are controlled from this method.
 */
void mainTriggerCallback(trigger_event_e ckpSignalType, uint32_t trgEventIndex DECLARE_ENGINE_PARAMETER_S) {
	(void) ckpSignalType;

	ENGINE(m.beforeMainTrigger) = GET_TIMESTAMP();
	if (hasFirmwareError()) {
		/**
		 * In case on a major error we should not process any more events.
		 * TODO: add 'pin shutdown' invocation somewhere - coils might be still open here!
		 */
		return;
	}
	efiAssertVoid(getRemainingStack(chThdSelf()) > 128, "lowstck#2");

	if (trgEventIndex >= ENGINE(triggerShape.getLength())) {
		/**
		 * this could happen in case of a trigger error, just exit silently since the trigger error is supposed to be handled already
		 * todo: should this check be somewhere higher so that no trigger listeners are invoked with noise?
		 */
		return;
	}

	int rpm = ENGINE(rpmCalculator.rpmValue);
	if (rpm == 0) {
		// this happens while we just start cranking
		// todo: check for 'trigger->is_synchnonized?'
		// TODO: add 'pin shutdown' invocation somewhere - coils might be still open here!
		return;
	}
	if (rpm == NOISY_RPM) {
		warning(OBD_Camshaft_Position_Sensor_Circuit_Range_Performance, "noisy trigger");
		// TODO: add 'pin shutdown' invocation somewhere - coils might be still open here!
		return;
	}
	bool limitedSpark = rpm > CONFIG(rpmHardLimit);
	bool limitedFuel = rpm > CONFIG(rpmHardLimit);

	if (CONFIG(boostCutPressure) !=0) {
		if (getMap() > CONFIG(boostCutPressure)) {
			limitedSpark = true;
			limitedFuel = true;
		}
	}

	if (limitedSpark || limitedFuel) {
		// todo: this is not really a warning
		warning(CUSTOM_OBD_34, "skipping stroke due to rpm=%d", rpm);
	}

#if (EFI_HISTOGRAMS && EFI_PROD_CODE) || defined(__DOXYGEN__)
	int beforeCallback = hal_lld_get_counter_value();
#endif

	int revolutionIndex = ENGINE(rpmCalculator).getRevolutionCounter() % 2;

	if (trgEventIndex == 0) {
		// these two statements should be atomic, but in reality we should be fine, right?
		engine->fuelScheduleForThisEngineCycle = ENGINE(engineConfiguration2)->injectionEvents;
		engine->fuelScheduleForThisEngineCycle->usedAtEngineCycle = ENGINE(rpmCalculator).getRevolutionCounter();

		if (triggerVersion.isOld()) {
			// todo: move 'triggerIndexByAngle' change into trigger initialization, why is it invoked from here if it's only about trigger shape & optimization?
			prepareOutputSignals(PASS_ENGINE_PARAMETER_F);
			// we need this to apply new 'triggerIndexByAngle' values
			engine->periodicFastCallback(PASS_ENGINE_PARAMETER_F);
		}
	}

	efiAssertVoid(!CONFIG(useOnlyRisingEdgeForTrigger) || CONFIG(ignMathCalculateAtIndex) % 2 == 0, "invalid ignMathCalculateAtIndex");

	if (trgEventIndex == CONFIG(ignMathCalculateAtIndex)) {
		if (CONFIG(externalKnockSenseAdc) != EFI_ADC_NONE) {
			float externalKnockValue = getVoltageDivided("knock", engineConfiguration->externalKnockSenseAdc);
			engine->knockLogic(externalKnockValue);
		}

		ENGINE(m.beforeIgnitionMath) = GET_TIMESTAMP();
		ignitionMathCalc(rpm PASS_ENGINE_PARAMETER);
		ENGINE(m.ignitionMathTime) = GET_TIMESTAMP() - ENGINE(m.beforeIgnitionMath);
	}


	/**
	 * For fuel we schedule start of injection based on trigger angle, and then inject for
	 * specified duration of time
	 */
	handleFuel(limitedFuel, trgEventIndex, rpm PASS_ENGINE_PARAMETER);
	/**
	 * For spark we schedule both start of coil charge and actual spark based on trigger angle
	 */
	handleSpark(revolutionIndex, limitedSpark, trgEventIndex, rpm,
			&engine->engineConfiguration2->ignitionEvents[revolutionIndex] PASS_ENGINE_PARAMETER);
#if (EFI_HISTOGRAMS && EFI_PROD_CODE) || defined(__DOXYGEN__)
	int diff = hal_lld_get_counter_value() - beforeCallback;
	if (diff > 0)
	hsAdd(&mainLoopHisto, diff);
#endif /* EFI_HISTOGRAMS */

	if (trgEventIndex == 0) {
		ENGINE(m.mainTriggerCallbackTime) = GET_TIMESTAMP() - ENGINE(m.beforeMainTrigger);
	}
}

#if EFI_ENGINE_SNIFFER || defined(__DOXYGEN__)
#include "engine_sniffer.h"
#endif

static void showTriggerHistogram(void) {
	printAllCallbacksHistogram();
	showMainHistogram();
#if EFI_ENGINE_SNIFFER || defined(__DOXYGEN__)
	showWaveChartHistogram();
#endif
}

static void showMainInfo(Engine *engine) {
#if EFI_PROD_CODE || defined(__DOXYGEN__)
	int rpm = engine->rpmCalculator.getRpm(PASS_ENGINE_PARAMETER_F);
	float el = getEngineLoadT(PASS_ENGINE_PARAMETER_F);
	scheduleMsg(logger, "rpm %d engine_load %f", rpm, el);
	scheduleMsg(logger, "fuel %fms timing %f", getInjectionDuration(rpm PASS_ENGINE_PARAMETER), engine->engineState.timingAdvance);
#endif
}

void initMainEventListener(Logging *sharedLogger, Engine *engine) {
	logger = sharedLogger;
	efiAssertVoid(engine!=NULL, "null engine");
	initSparkLogic(logger);

#if EFI_PROD_CODE || defined(__DOXYGEN__)
	addConsoleAction("performanceinfo", showTriggerHistogram);
	addConsoleActionP("maininfo", (VoidPtr) showMainInfo, engine);

	printMsg(logger, "initMainLoop: %d", currentTimeMillis());
	if (!isInjectionEnabled(engine->engineConfiguration))
		printMsg(logger, "!!!!!!!!!!!!!!!!!!! injection disabled");
#endif

#if EFI_HISTOGRAMS || defined(__DOXYGEN__)
	initHistogram(&mainLoopHisto, "main callback");
#endif /* EFI_HISTOGRAMS */

	addTriggerEventListener(mainTriggerCallback, "main loop", engine);
}

#endif /* EFI_ENGINE_CONTROL */
