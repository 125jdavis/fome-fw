package com.rusefi.config.generated;

// this file was generated automatically by rusEFI tool ConfigDefinition.jar based on (unknown script) controllers/actuators/electronic_throttle.txt Sun Nov 27 23:27:34 UTC 2022

// by class com.rusefi.output.FileJavaFieldsConsumer
import com.rusefi.config.*;

public class ElectronicThrottle {
	public static final Field IDLEPOSITION = Field.create("IDLEPOSITION", 0, FieldType.FLOAT).setBaseOffset(1216);
	public static final Field TRIM = Field.create("TRIM", 4, FieldType.FLOAT).setBaseOffset(1216);
	public static final Field LUAADJUSTMENT = Field.create("LUAADJUSTMENT", 8, FieldType.FLOAT).setBaseOffset(1216);
	public static final Field ETBFEEDFORWARD = Field.create("ETBFEEDFORWARD", 12, FieldType.FLOAT).setBaseOffset(1216);
	public static final Field ETBINTEGRALERROR = Field.create("ETBINTEGRALERROR", 16, FieldType.FLOAT).setBaseOffset(1216);
	public static final Field ETBCURRENTTARGET = Field.create("ETBCURRENTTARGET", 20, FieldType.FLOAT).setBaseOffset(1216);
	public static final Field ETBCURRENTADJUSTEDTARGET = Field.create("ETBCURRENTADJUSTEDTARGET", 24, FieldType.FLOAT).setBaseOffset(1216);
	public static final Field ETBREVLIMITACTIVE = Field.create("ETBREVLIMITACTIVE", 28, FieldType.BIT, 0).setBaseOffset(1216);
	public static final Field ETBDUTYRATEOFCHANGE = Field.create("ETBDUTYRATEOFCHANGE", 32, FieldType.FLOAT).setBaseOffset(1216);
	public static final Field[] VALUES = {
	IDLEPOSITION,
	TRIM,
	LUAADJUSTMENT,
	ETBFEEDFORWARD,
	ETBINTEGRALERROR,
	ETBCURRENTTARGET,
	ETBCURRENTADJUSTEDTARGET,
	ETBREVLIMITACTIVE,
	ETBDUTYRATEOFCHANGE,
	};
}
