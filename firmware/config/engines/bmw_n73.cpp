/*
 * bmw_n73.cpp
 *
 * @date Oct 2, 2021
 * @author Andrey Belomutskiy, (c) 2012-2021
 */

#include "bmw_n73.h"

void setEngineProteusBMW_N73_GDI() {

}

void setEngineProteusGearboxManInTheMiddle() {
	strncpy(config->luaScript, R"(

CAN_BMW_E90_RPM_THROTTLE       = 0x0AA
CAN_BMW_E90_TORQUE_DEMAND      = 0x0B6
CAN_BMW_GEAR_TORQUE_DEMAND2    = 0x0B5
CAN_BMW_GEAR_TRANSMISSION_DATA = 0x0BA
CAN_BMW_E90_IGNITION_KEY       = 0x130
CAN_BMW_GEAR_SELECTOR          = 0x192
CAN_BMW_GEAR_GEARBOX_DATA_2    = 0x1A2
CAN_BMW_E90_COOLANT            = 0x1D0
CAN_BMW_GEAR_TRANSMISSION_DISP = 0x1D2
CAN_BMW_GEAR_GANG_STATUS       = 0x304
CAN_BMW_E90_DASH_ON            = 0x332

ECU_BUS = 1
GEAR_BUS = 2

canRxAdd(CAN_BMW_E90_RPM_THROTTLE)
canRxAdd(CAN_BMW_E90_TORQUE_DEMAND)
canRxAdd(CAN_BMW_GEAR_TORQUE_DEMAND2)
canRxAdd(CAN_BMW_GEAR_TRANSMISSION_DATA)
canRxAdd(CAN_BMW_E90_IGNITION_KEY)
canRxAdd(CAN_BMW_GEAR_SELECTOR)
canRxAdd(CAN_BMW_GEAR_GEARBOX_DATA_2)
canRxAdd(CAN_BMW_E65_GEAR_SELECTOR)
canRxAdd(CAN_BMW_E90_COOLANT)
canRxAdd(CAN_BMW_GEAR_TRANSMISSION_DISP)
canRxAdd(CAN_BMW_GEAR_GANG_STATUS)
canRxAdd(CAN_BMW_E90_DASH_ON)

txPayload = {}

function onCanRx(bus, id, dlc, data)
	id = id % 2048
	-- local output = string.format("%x", id)

	if id == CAN_BMW_E90_IGNITION_KEY then
		print('CAN_BMW_E90_IGNITION_KEY')
  elseif id == CAN_BMW_GEAR_TRANSMISSION_DATA then
print('CAN_BMW_GEAR_TRANSMISSION_DATA')
elseif id == CAN_BMW_GEAR_SELECTOR then
print('CAN_BMW_GEAR_SELECTOR') then
elseif id == CAN_BMW_GEAR_TRANSMISSION_DISP then
print('CAN_BMW_GEAR_TRANSMISSION_DISP')
elseif id == CAN_BMW_GEAR_GANG_STATUS then
print('CAN_BMW_GEAR_GANG_STATUS')
	else
		if id == CAN_BMW_E65_GEAR_SELECTOR then
			print('CAN_BMW_E65_GEAR_SELECTOR')
		else
			if id == CAN_BMW_E90_RPM_THROTTLE then
				print('CAN_BMW_E90_RPM_THROTTLE')
			else
				if id == CAN_BMW_E90_COOLANT then
					print('CAN_BMW_E90_COOLANT')
				else
					print('got CAN id=' ..id ..' dlc=' ..dlc)
				end
			end
		end
	end
end


function onTick()
end
)", efi::size(config->luaScript));


}
