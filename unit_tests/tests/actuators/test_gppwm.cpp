#include "pch.h"

#include "gppwm_channel.h"
#include "gppwm.h"
#include "boost_control.h"

#include "mocks.h"

using ::testing::InSequence;
using ::testing::StrictMock;

TEST(GpPwm, OutputWithPwm) {
	GppwmChannel ch;

	gppwm_channel cfg;

	StrictMock<MockPwm> pwm;

	// Shouldn't throw with no config
	EXPECT_NO_THROW(ch.setOutput(10));

	{
		InSequence i;
		EXPECT_CALL(pwm, setSimplePwmDutyCycle(0.25f));
		EXPECT_CALL(pwm, setSimplePwmDutyCycle(0.75f));
		EXPECT_CALL(pwm, setSimplePwmDutyCycle(0.0f));
		EXPECT_CALL(pwm, setSimplePwmDutyCycle(1.0f));
	}

	ch.init(true, &pwm, nullptr, nullptr, &cfg);

	// Set the output - should set directly to PWM
	ch.setOutput(25.0f);
	ch.setOutput(75.0f);

	// Test clamping behavior - should clamp to [0, 100]
	ch.setOutput(-10.0f);
	ch.setOutput(110.0f);
}

TEST(GpPwm, OutputOnOff) {
	GppwmChannel ch;

	gppwm_channel cfg;
	cfg.onAboveDuty = 50;
	cfg.offBelowDuty = 40;

	MockOutputPin pin;

	{
		InSequence i;

		// Rising edge test
		EXPECT_CALL(pin, setValue(0));
		EXPECT_CALL(pin, setValue(1));
		EXPECT_CALL(pin, setValue(1));

		// Falling edge test
		EXPECT_CALL(pin, setValue(1));
		EXPECT_CALL(pin, setValue(0));
		EXPECT_CALL(pin, setValue(0));
	}

	ch.init(false, nullptr, &pin, nullptr, &cfg);

	// Test rising edge - these should output 0, 1, 1
	ch.setOutput(49.0f);
	ch.setOutput(51.0f);
	ch.setOutput(49.0f);

	// Test falling edge - these should output 1, 0, 0
	ch.setOutput(41.0f);
	ch.setOutput(39.0f);
	ch.setOutput(41.0f);
}

TEST(GpPwm, TestGetOutput) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	GppwmChannel ch;

	gppwm_channel cfg;
	cfg.loadAxis = GPPWM_Tps;
	cfg.rpmAxis = GPPWM_Rpm;
	cfg.dutyIfError = 21.0f;

	MockVp3d table;

	EXPECT_CALL(table, getValue(1200, 35.0f)).WillRepeatedly([](float x, float tps) { return tps; });

	ch.init(false, nullptr, nullptr, &table, &cfg);

	Sensor::resetAllMocks();

	// Should return dutyIfError
	EXPECT_FLOAT_EQ(21.0f, ch.getOutput().Result);

	// Set TPS, should return tps value
	Sensor::setMockValue(SensorType::Tps1, 35.0f);
	Sensor::setMockValue(SensorType::Rpm, 1200);
	EXPECT_FLOAT_EQ(35.0f, ch.getOutput().Result);
}

TEST(GpPwm, TestAdditionalAxisValues) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	Sensor::resetAllMocks();
	Sensor::setMockValue(SensorType::FuelPressureInjector, 420.0f);
	Sensor::setMockValue(SensorType::TurbochargerSpeed, 125000.0f);
	engine->engineState.clutchDownState = true;
	engine->engineState.brakePedalState = true;
	engine->module<BoostController>().unmock().boostControlTarget = 182.5f;

	EXPECT_FLOAT_EQ(182.5f, readGppwmChannel(GPPWM_BoostTarget).Value);
	EXPECT_FLOAT_EQ(420.0f, readGppwmChannel(GPPWM_FuelPressure).Value);
	EXPECT_FLOAT_EQ(100.0f, readGppwmChannel(GPPWM_ClutchState).Value);
	EXPECT_FLOAT_EQ(100.0f, readGppwmChannel(GPPWM_BrakeState).Value);
	EXPECT_FLOAT_EQ(125000.0f, readGppwmChannel(GPPWM_TurboSpeed).Value);

	engine->engineState.clutchDownState = false;
	engine->engineState.brakePedalState = false;
	EXPECT_FLOAT_EQ(0.0f, readGppwmChannel(GPPWM_ClutchState).Value);
	EXPECT_FLOAT_EQ(0.0f, readGppwmChannel(GPPWM_BrakeState).Value);
}
