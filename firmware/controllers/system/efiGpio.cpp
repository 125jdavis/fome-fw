/**
 * @file	efiGpio.cpp
 *
 * @date Sep 26, 2014
 * @author Andrey Belomutskiy, (c) 2012-2016
 */

#include "main.h"
#if EFI_GPIO
#include "efiGpio.h"
#include "io_pins.h"
#if EFI_PROD_CODE || defined(__DOXYGEN__)
#include "gpio_helper.h"
#endif

pin_output_mode_e OUTPUT_MODE_DEFAULT = OM_DEFAULT;

// todo: clean this mess, this should become 'static'/private
engine_pins_s enginePins;

NamedOutputPin::NamedOutputPin() : OutputPin() {
	name = NULL;
}

NamedOutputPin::NamedOutputPin(const char *name) : OutputPin() {
	this->name = name;
}

InjectorOutputPin::InjectorOutputPin() : NamedOutputPin() {
	reset();
}

static const char *sparkNames[IGNITION_PIN_COUNT] = { "c1", "c2", "c3", "c4", "c5", "c6", "c7", "c8",
		"c9", "cA", "cB", "cD"};

static const char *injectorNames[INJECTION_PIN_COUNT] = { "i1", "i2", "i3", "i4", "i5", "i6", "i7", "i8",
		"j9", "iA", "iB", "iC"};


engine_pins_s::engine_pins_s() {
	dizzyOutput.name = DIZZY_NAME;
	tachOut.name = TACH_NAME;

	for (int i = 0; i < IGNITION_PIN_COUNT;i++) {
		enginePins.coils[i].name = sparkNames[i];
	}
	for (int i = 0; i < INJECTION_PIN_COUNT;i++) {
		enginePins.injectors[i].name = injectorNames[i];
	}
}

void engine_pins_s::reset() {
	for (int i = 0; i < INJECTION_PIN_COUNT;i++) {
		injectors[i].reset();
	}
	for (int i = 0; i < IGNITION_PIN_COUNT;i++) {
		coils[i].reset();
	}
}

void InjectorOutputPin::reset() {
	overlappingScheduleOffTime = 0;
	cancelNextTurningInjectorOff = false;
	overlappingCounter = 0;
	// todo: this could be refactored by calling some super-reset method
	currentLogicValue = INITIAL_PIN_STATE;
}

IgnitionOutputPin::IgnitionOutputPin() {
	reset();
}

void IgnitionOutputPin::reset() {
	outOfOrderCounter = 0;
}

OutputPin::OutputPin() {
	modePtr = &OUTPUT_MODE_DEFAULT;
#if EFI_PROD_CODE || defined(__DOXYGEN__)
	port = NULL;
	pin = 0;
#endif
	currentLogicValue = INITIAL_PIN_STATE;
}

bool OutputPin::isInitialized() {
#if EFI_PROD_CODE || defined(__DOXYGEN__)
	return port != NULL;
#else
	return false;
#endif
}

void OutputPin::setValue(int logicValue) {
	doSetOutputPinValue2(this, logicValue);
}

bool OutputPin::getLogicValue() {
	return currentLogicValue;
}

void OutputPin::unregister() {
#if EFI_PROD_CODE || defined(__DOXYGEN__)
	port = NULL;
#endif
}

void OutputPin::setDefaultPinState(pin_output_mode_e *outputMode) {
#if EFI_GPIO || defined(__DOXYGEN__)
	pin_output_mode_e mode = *outputMode;
	assertOMode(mode);
	this->modePtr = outputMode;
#endif
	setValue(false); // initial state
}

#endif /* EFI_GPIO */
