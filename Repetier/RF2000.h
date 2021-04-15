/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef RF2000_H
#define RF2000_H

// this file contains all definitions which are specific to the RF2000 hardware
#define UI_PRINTER_NAME "RF2000"

/** \brief Number of extruders */
#define NUM_EXTRUDER 1 // 1 = Single, 2 = Dual

/** \brief Specifies the maximal drive over millimeters which the z-endstop can bear without getting damaged or degraded */
/*
Known Limits for Z_ENDSTOP_DRIVE_OVER to avoid when extending value:
Z-EndStop button crash RF1000: ~ >0.8f
Z-EndStop optical button crash RF2000: ~ >1.3f
Crash with Einhausung/Plexiglas backside RFx000: ~ >5.0 .. 6.0f
Crash with backside metal RFx000: ~ >10.0..12.0f
Overflow in Z-Matrix: >12.7f
*/
#define Z_ENDSTOP_DRIVE_OVER 1.3f //mm

// ##########################################################################################
// ##   main hardware configuration
// ##########################################################################################

#if NUM_EXTRUDER > 2 || NUM_EXTRUDER < 0
#error This Firmware supports up to 2 Extruders. You might have to reimplement the 3+ code or "request it" if you really got hands on a RFx000 board with 3+ Extruders.
#endif // NUM_EXTRUDER > 2 || NUM_EXTRUDER < 0

/** \brief Allows to use the device for milling */
#ifndef FEATURE_MILLING_MODE
#define FEATURE_MILLING_MODE 0 // 1 = on, 0 = off
#endif
#if FEATURE_MILLING_MODE

/** \brief Enables automatic compensation in z direction for the operationg mode "mill" */
#define FEATURE_WORK_PART_Z_COMPENSATION 1 // 1 = on, 0 = off

/** \brief This feature allows to move the milling bed upwards automatically until the miller is hit. The found position is taken over as Z=0 automatically.
     Be aware that mis-using of this functionality can ruin the tool (e.g. in case the tool is placed above the milling bed and not above the to-be-milled object). */
#define FEATURE_FIND_Z_ORIGIN 1 // 1 = on, 0 = off

/** \brief Enables/disables the menu entry which allows to choose the currently installed miller type */
#define FEATURE_CONFIGURABLE_MILLER_TYPE 1 // 1 = on, 0 = off

#if FEATURE_WORK_PART_Z_COMPENSATION && !FEATURE_FIND_Z_ORIGIN
#error It does not make sense to enable the work part z-compensation without enabling of the automatic detection of the z-origin
#endif // FEATURE_WORK_PART_Z_COMPENSATION && !FEATURE_FIND_Z_ORIGIN

/** \brief Define the type of the present miller hardware */
#define MILLER_TYPE MILLER_TYPE_ONE_TRACK
/** \brief Define lower acceleration to reach very small speeds */
#define MILLER_ACCELERATION 15

/** \brief Default operating mode */
#define DEFAULT_OPERATING_MODE OPERATING_MODE_PRINT

#endif // FEATURE_MILLING_MODE

/** \brief Allows to use the 230V output */
#define FEATURE_230V_OUTPUT 1 // 1 = on, 0 = off

/** \brief Allows to use the RGB light effects */
#define FEATURE_RGB_LIGHT_EFFECTS 1 // 1 = on, 0 = off

/**
 *\brief Allows to use the 24V FET outputs
 * Also set the FET1 FET2 pins. They have menu support and their state is remembered using eeprom.
 * Set FET3 pin if you want to be able to switch it by gcode M3300.
 *
 * @RF2000 FET3 per default is the case fan pin CASE_FAN_PIN as well. -> See M3120 / M3121.
 * -> Set FET3 and/or FEATURE_CASE_FAN for working with X45.
 * -> If FEATURE_CASE_FAN is 1 and its CASE_FAN_PIN is set then the FET3 state is not being loaded from eeprom at boot time.
 */
#define FEATURE_24V_FET_OUTPUTS 1 // 1 = on, 0 = off

/** \brief Allows to use the case fan */
#define FEATURE_CASE_FAN 1 // 0 = off, 1 = on

/** \brief Define the type of the present extruders */
#define EXT0_HOTEND_TYPE HOTEND_TYPE_V2
#define EXT1_HOTEND_TYPE HOTEND_TYPE_V2

/** \brief This feature enables a security check so that the Firmware for a RF2000 is only working on a RF2000 Board otherwise the firmware beeps. */
#define FEATURE_TYPE_EEPROM 1 // 1 = on, 0 = off

// ##########################################################################################
// ##   Calibration
// ##########################################################################################

/** \brief maximum positions in mm - only fixed numbers!
For delta robot Z_MAX_LENGTH is the maximum travel of the towers and should be set to the distance between the hotend
and the platform when the printer is at its home position.
If EEPROM is enabled these values will be overidden with the values in the EEPROM */
#if NUM_EXTRUDER == 2
#define X_MAX_LENGTH_PRINT 180 // [mm]
#else
#define X_MAX_LENGTH_PRINT 230 // [mm]
#endif                         // NUM_EXTRUDER == 2

#define X_MAX_LENGTH_MILL 230 // [mm]
#define Y_MAX_LENGTH 245      // [mm]
//RF2000 von Nibbels: ~194,86 -> PAUSE_Z_MAX_SPACING_MM großzügig einplanen
#define Z_MAX_LENGTH 200 // [mm]

/** \brief Drive settings for printers with cartesian drive systems */
/** \brief Number of steps for a 1mm move in x direction.
For xy gantry use 2*belt moved!
Overridden if EEPROM activated. */
#define XAXIS_STEPS_PER_MM float(4.761875 * (float)RF_MICRO_STEPS_XY)

/** \brief Number of steps for a 1mm move in y direction.
For xy gantry use 2*belt moved!
Overridden if EEPROM activated.*/
#define YAXIS_STEPS_PER_MM float(4.761875 * (float)RF_MICRO_STEPS_XY)

/** \brief Number of steps for a 1mm move in z direction  Overridden if EEPROM activated.*/
#define ZAXIS_STEPS_PER_MM float(80 * (float)RF_MICRO_STEPS_Z)

// ##########################################################################################
// ##   Miller type 1 (= one track)
// ##########################################################################################

#define MT1_WORK_PART_SCAN_CONTACT_PRESSURE_DELTA 10 // [digits]
#define MT1_WORK_PART_SCAN_RETRY_PRESSURE_DELTA 5    // [digits]

// ##########################################################################################
// ##   Miller type 2 (= two tracks)
// ##########################################################################################

#define MT2_WORK_PART_SCAN_CONTACT_PRESSURE_DELTA 500 // [digits]
#define MT2_WORK_PART_SCAN_RETRY_PRESSURE_DELTA 250   // [digits]

// ##########################################################################################
// ##   Configuration of the 1. extruder
// ##########################################################################################

#define EXT0_X_OFFSET_MM 0.0f
#define EXT0_Y_OFFSET_MM 0.0f
#define EXT0_Z_OFFSET_MM 0.0f //to support Nozzle-Tip-Down-Hotends

/** \brief for skeinforge 40 and later, steps to pull the plasic 1 mm inside the extruder, not out.  Overridden if EEPROM activated. */
#define EXT0_STEPS_PER_MM (8.75 * RF_MICRO_STEPS_E)

/** \brief What type of sensor is used?
NTC-Thermistors
1: Epcos B57560G0107F000
2: 200k Thermistor
3: Hotend V2 Sensor Conrad Renkforce / mendel-parts thermistor (EPCOS G550) = NTC mit 100kOhm
4: 10k Thermistor
5: USER_THERMISTORTABLE0 als NTC
6: USER_THERMISTORTABLE1 als NTC
7: USER_THERMISTORTABLE2 als NTC
8: E3D Thermistor ATC Semitec 104-GT2 (300°C)
9: 100k Honeywell 135-104LAG-J01
10: 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
11: 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
12: 100k RS Thermistor 198-961 (4.7k pullup)
13: NTC 3950 100k thermistor - Conrad V3
14: Thermistor NTC 3950 100k Ohm
97: USE_GENERIC_THERMISTORTABLE_1 and GENERIC_THERM_NUM_ENTRIES Define Raw Thermistor and Resistor-Settings within configuration.h
98: USE_GENERIC_THERMISTORTABLE_2 and GENERIC_THERM_NUM_ENTRIES Define Raw Thermistor and Resistor-Settings within configuration.h
99: USE_GENERIC_THERMISTORTABLE_3 and GENERIC_THERM_NUM_ENTRIES Define Raw Thermistor and Resistor-Settings within configuration.h
PTC-Thermistors
50: USER_THERMISTORTABLE0 als PTC
51: USER_THERMISTORTABLE1 als PTC
52: USER_THERMISTORTABLE2 als PTC
53: E3D PT100 Board (direct AD voltage in)
60: HEATER_USES_AD8495 (Delivers 5mV/degC)
100: AD595 */
#define EXT0_TEMPSENSOR_TYPE 8

/** \brief Analog input pin for reading temperatures or pin enabling SS for MAX6675 */
#define EXT0_TEMPSENSOR_PIN TEMP_0_PIN

/** \brief Which pin enables the heater */
#define EXT0_HEATER_PIN HEATER_0_PIN
#define EXT0_STEP_PIN E0_STEP_PIN
#define EXT0_DIR_PIN E0_DIR_PIN

/** \brief set to 0/1 for normal/inverse direction */
#define EXT0_INVERSE false
#define EXT0_ENABLE_PIN E0_ENABLE_PIN

/** \brief For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1 */
#define EXT0_ENABLE_ON true

/** \brief The following speed settings are for skeinforge 40+ where e is the
length of filament pulled inside the heater. For repsnap or older
skeinforge use higher values.
Overridden if EEPROM activated. */
#define EXT0_MAX_FEEDRATE 25

/** \brief Feedrate from halted extruder in mm/s
Overridden if EEPROM activated. */
#define EXT0_MAX_START_FEEDRATE 12

/** \brief Acceleration in mm/s^2
Overridden if EEPROM activated. */
#define EXT0_MAX_ACCELERATION 6000

#if EXT0_HOTEND_TYPE == HOTEND_TYPE_V1

/** \brief The maximum value, I-gain can contribute to the output. Overridden if EEPROM activated. */
#define EXT0_PID_INTEGRAL_DRIVE_MAX HT2_PID_INTEGRAL_DRIVE_MAX
/** \brief lower value for integral part. Overridden if EEPROM activated. */
#define EXT0_PID_INTEGRAL_DRIVE_MIN HT2_PID_INTEGRAL_DRIVE_MIN
/** \brief P-gain. Overridden if EEPROM activated. */
#define EXT0_PID_P HT2_PID_P
/** \brief I-gain. Overridden if EEPROM activated. */
#define EXT0_PID_I HT2_PID_I
/** \brief D-gain. Overridden if EEPROM activated.*/
#define EXT0_PID_D HT2_PID_D

#endif // EXT0_HOTEND_TYPE == HOTEND_TYPE_V1

#if EXT0_HOTEND_TYPE == HOTEND_TYPE_V2

/** \brief The maximum value, I-gain can contribute to the output. Overridden if EEPROM activated. */
#define EXT0_PID_INTEGRAL_DRIVE_MAX HT3_PID_INTEGRAL_DRIVE_MAX
/** \brief lower value for integral part. Overridden if EEPROM activated. */
#define EXT0_PID_INTEGRAL_DRIVE_MIN HT3_PID_INTEGRAL_DRIVE_MIN
/** \brief P-gain. Overridden if EEPROM activated. */
#define EXT0_PID_P HT3_PID_P
/** \brief I-gain. Overridden if EEPROM activated. */
#define EXT0_PID_I HT3_PID_I
/** \brief D-gain. Overridden if EEPROM activated.*/
#define EXT0_PID_D HT3_PID_D

#endif // EXT0_HOTEND_TYPE == HOTEND_TYPE_V2

#if EXT0_HOTEND_TYPE == HOTEND_TYPE_V3

/** \brief The maximum value, I-gain can contribute to the output. Overridden if EEPROM activated. */
#define EXT0_PID_INTEGRAL_DRIVE_MAX HT4_PID_INTEGRAL_DRIVE_MAX
/** \brief lower value for integral part. Overridden if EEPROM activated. */
#define EXT0_PID_INTEGRAL_DRIVE_MIN HT4_PID_INTEGRAL_DRIVE_MIN
/** \brief P-gain. Overridden if EEPROM activated. */
#define EXT0_PID_P HT4_PID_P
/** \brief I-gain. Overridden if EEPROM activated. */
#define EXT0_PID_I HT4_PID_I
/** \brief D-gain. Overridden if EEPROM activated.*/
#define EXT0_PID_D HT4_PID_D

#endif // EXT0_HOTEND_TYPE == HOTEND_TYPE_V3

/** \brief maximum time the heater is can be switched on. Max = 255.  Overridden if EEPROM activated. */
#define EXT0_PID_MAX 255

/** \brief Faktor for the advance algorithm. 0 disables the algorithm.  Overridden if EEPROM activated.
L is the linear factor and seems to be working better then the quadratic dependency. */
#define EXT0_ADVANCE_L 0.0f

/** \brief Temperature to retract filament when extruder is heating up. Overridden if EEPROM activated. */
#define EXT0_WAIT_RETRACT_TEMP 150

/** \brief Units (mm/inches) to retract filament when extruder is heating up. Overridden if EEPROM activated. Set
to 0 to disable. */
#define EXT0_WAIT_RETRACT_UNITS 0

/** \brief You can run any gcode command on extruder deselect/select. Seperate multiple commands with a new line \n.
That way you can execute some mechanical components needed for extruder selection or retract filament or whatever you need.
The codes are only executed for multiple extruder when changing the extruder. */
#define EXT0_SELECT_COMMANDS "M117 Extruder 0"
#define EXT0_DESELECT_COMMANDS ""

/** \brief PWM speed for the cooler fan. 0=off 255=full speed */
#define EXT0_EXTRUDER_COOLER_SPEED 255

#if NUM_EXTRUDER > 0 && EXT0_TEMPSENSOR_TYPE < 101

#define EXT0_ANALOG_INPUTS 1
#define EXT0_SENSOR_INDEX 0
#define EXT0_ANALOG_CHANNEL EXT0_TEMPSENSOR_PIN
#define ACCOMMA0 ,

#else

#define EXT0_ANALOG_INPUTS 0
#define EXT0_SENSOR_INDEX EXT0_TEMPSENSOR_PIN
#define EXT0_ANALOG_CHANNEL
#define ACCOMMA0

#endif // NUM_EXTRUDER>0 && EXT0_TEMPSENSOR_TYPE<101

#if NUM_EXTRUDER == 2

#define FEATURE_ALIGN_EXTRUDERS 1

// ##########################################################################################
// ##   Configuration of the 2. extruder
// ##########################################################################################

#define EXT1_X_OFFSET_MM 33.9f // [mm]
#define EXT1_Y_OFFSET_MM 0.1f  // [mm]
#define EXT1_Z_OFFSET_MM 0.0f  // [mm] //to support Nozzle-Tip-Down-Hotends

/** \brief for skeinforge 40 and later, steps to pull the plasic 1 mm inside the extruder, not out.  Overridden if EEPROM activated. */
#define EXT1_STEPS_PER_MM (8.75 * RF_MICRO_STEPS_E)

/** \brief What type of sensor is used?
NTC-Thermistors
1: Epcos B57560G0107F000
2: 200k Thermistor
3: Hotend V2 Sensor Conrad Renkforce / mendel-parts thermistor (EPCOS G550) = NTC mit 100kOhm
4: 10k Thermistor
5: USER_THERMISTORTABLE0 als NTC
6: USER_THERMISTORTABLE1 als NTC
7: USER_THERMISTORTABLE2 als NTC
8: E3D Thermistor ATC Semitec 104-GT2 (300°C)
9: 100k Honeywell 135-104LAG-J01
10: 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
11: 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
12: 100k RS Thermistor 198-961 (4.7k pullup)
13: NTC 3950 100k thermistor - Conrad V3
14: Thermistor NTC 3950 100k Ohm
97: USE_GENERIC_THERMISTORTABLE_1 and GENERIC_THERM_NUM_ENTRIES Define Raw Thermistor and Resistor-Settings within configuration.h
98: USE_GENERIC_THERMISTORTABLE_2 and GENERIC_THERM_NUM_ENTRIES Define Raw Thermistor and Resistor-Settings within configuration.h
99: USE_GENERIC_THERMISTORTABLE_3 and GENERIC_THERM_NUM_ENTRIES Define Raw Thermistor and Resistor-Settings within configuration.h
PTC-Thermistors
50: USER_THERMISTORTABLE0 als PTC
51: USER_THERMISTORTABLE1 als PTC
52: USER_THERMISTORTABLE2 als PTC
53: E3D PT100 Board (direct AD voltage in)
60: HEATER_USES_AD8495 (Delivers 5mV/degC)
100: AD595 */
#define EXT1_TEMPSENSOR_TYPE 3

/** \brief Analog input pin for reading temperatures or pin enabling SS for MAX6675 */
#define EXT1_TEMPSENSOR_PIN TEMP_1_PIN

/** \brief Which pin enables the heater */
#define EXT1_HEATER_PIN HEATER_1_PIN
#define EXT1_STEP_PIN E1_STEP_PIN
#define EXT1_DIR_PIN E1_DIR_PIN

/** \brief set to 0/1 for normal/inverse direction */
#define EXT1_INVERSE true
#define EXT1_ENABLE_PIN E1_ENABLE_PIN

/** \brief For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1 */
#define EXT1_ENABLE_ON true

/** \brief The following speed settings are for skeinforge 40+ where e is the
length of filament pulled inside the heater. For repsnap or older
skeinforge use heigher values.
Overridden if EEPROM activated. */
#define EXT1_MAX_FEEDRATE 25

/** \brief Feedrate from halted extruder in mm/s
Overridden if EEPROM activated. */
#define EXT1_MAX_START_FEEDRATE 12

/** \brief Acceleration in mm/s^2
Overridden if EEPROM activated. */
#define EXT1_MAX_ACCELERATION 6000

#if EXT1_HOTEND_TYPE == HOTEND_TYPE_V1

/** \brief The maximum value, I-gain can contribute to the output. Overridden if EEPROM activated. */
#define EXT1_PID_INTEGRAL_DRIVE_MAX HT2_PID_INTEGRAL_DRIVE_MAX
/** \brief lower value for integral part. Overridden if EEPROM activated. */
#define EXT1_PID_INTEGRAL_DRIVE_MIN HT2_PID_INTEGRAL_DRIVE_MIN
/** \brief P-gain.  Overridden if EEPROM activated. */
#define EXT1_PID_P HT2_PID_P
/** \brief I-gain. Overridden if EEPROM activated. */
#define EXT1_PID_I HT2_PID_I
/** \brief Dgain.  Overridden if EEPROM activated.*/
#define EXT1_PID_D HT2_PID_D

#endif // EXT1_HOTEND_TYPE == HOTEND_TYPE_V1

#if EXT1_HOTEND_TYPE == HOTEND_TYPE_V2

/** \brief The maximum value, I-gain can contribute to the output. Overridden if EEPROM activated. */
#define EXT1_PID_INTEGRAL_DRIVE_MAX HT3_PID_INTEGRAL_DRIVE_MAX
/** \brief lower value for integral part. Overridden if EEPROM activated. */
#define EXT1_PID_INTEGRAL_DRIVE_MIN HT3_PID_INTEGRAL_DRIVE_MIN
/** \brief P-gain.  Overridden if EEPROM activated. */
#define EXT1_PID_P HT3_PID_P
/** \brief I-gain. Overridden if EEPROM activated. */
#define EXT1_PID_I HT3_PID_I
/** \brief D-gain.  Overridden if EEPROM activated.*/
#define EXT1_PID_D HT3_PID_D

#endif // EXT1_HOTEND_TYPE == HOTEND_TYPE_V2

#if EXT1_HOTEND_TYPE == HOTEND_TYPE_V3

/** \brief The maximum value, I-gain can contribute to the output. Overridden if EEPROM activated. */
#define EXT1_PID_INTEGRAL_DRIVE_MAX HT4_PID_INTEGRAL_DRIVE_MAX
/** \brief lower value for integral part. Overridden if EEPROM activated. */
#define EXT1_PID_INTEGRAL_DRIVE_MIN HT4_PID_INTEGRAL_DRIVE_MIN
/** \brief P-gain. Overridden if EEPROM activated. */
#define EXT1_PID_P HT4_PID_P
/** \brief I-gain. Overridden if EEPROM activated. */
#define EXT1_PID_I HT4_PID_I
/** \brief D-gain. Overridden if EEPROM activated.*/
#define EXT1_PID_D HT4_PID_D

#endif // EXT1_HOTEND_TYPE == HOTEND_TYPE_V3

/** \brief maximum time the heater is can be switched on. Max = 255.  Overridden if EEPROM activated. */
#define EXT1_PID_MAX 255

/** \brief Faktor for the advance algorithm. 0 disables the algorithm.  Overridden if EEPROM activated.
L is the linear factor and seems to be working better then the quadratic dependency. */
#define EXT1_ADVANCE_L 0.0f

/** \brief Motor steps to remove backlash for advance alorithm. These are the steps
needed to move the motor cog in reverse direction until it hits the driving
cog. Direct drive extruder need 0. */
#define EXT1_ADVANCE_BACKLASH_STEPS 0

/** \brief Temperature to retract filament when extruder is heating up. Overridden if EEPROM activated. */
#define EXT1_WAIT_RETRACT_TEMP 150

/** \brief Units (mm/inches) to retract filament when extruder is heating up. Overridden if EEPROM activated. Set
to 0 to disable. */
#define EXT1_WAIT_RETRACT_UNITS 0

/** \brief You can run any gcode command on extruder deselect/select. Seperate multiple commands with a new line \n.
That way you can execute some mechanical components needed for extruder selection or retract filament or whatever you need.
The codes are only executed for multiple extruder when changing the extruder. */
#define EXT1_SELECT_COMMANDS "M117 Extruder 1"
#define EXT1_DESELECT_COMMANDS ""

/** \brief PWM speed for the cooler fan. 0=off 255=full speed */
#define EXT1_EXTRUDER_COOLER_SPEED 255
#endif // NUM_EXTRUDER == 2

#if NUM_EXTRUDER > 1 && EXT1_TEMPSENSOR_TYPE < 101

#define EXT1_ANALOG_INPUTS 1
#define EXT1_SENSOR_INDEX EXT0_ANALOG_INPUTS
#define EXT1_ANALOG_CHANNEL ACCOMMA0 EXT1_TEMPSENSOR_PIN
#define ACCOMMA1 ,

#else

#define EXT1_ANALOG_INPUTS 0
#define EXT1_SENSOR_INDEX EXT1_TEMPSENSOR_PIN
#define EXT1_ANALOG_CHANNEL
#define ACCOMMA1 ACCOMMA0

#endif // NUM_EXTRUDER>1 && EXT1_TEMPSENSOR_TYPE<101

// ##########################################################################################
// ##   Configuration of the heated bed
// ##########################################################################################

/** \brief Set true if you have a heated bed conected to your board, false if not */
#define HAVE_HEATED_BED true

/** \brief Maximal temperature which can be set for the heating bed */
#define HEATED_BED_MAX_TEMP 180

/** \brief Select type of your heated bed. It's the same as for EXT0_TEMPSENSOR_TYPE
set to 0 if you don't have a heated bed */
#define HEATED_BED_SENSOR_TYPE 3

/** \brief Analog pin of analog sensor to read temperature of heated bed.  */
#define HEATED_BED_SENSOR_PIN TEMP_2_PIN

/** \brief Pin to enable heater for bed. */
#define HEATED_BED_HEATER_PIN HEATER_2_PIN

/** \brief How often the temperature of the heated bed is set (msec) */
#define HEATED_BED_SET_INTERVAL 5000

/** \brief The maximum value, I-gain can contribute to the output.
The precise values may differ for different nozzle/resistor combination.
 Overridden if EEPROM activated. */
#define HEATED_BED_PID_INTEGRAL_DRIVE_MAX 80

/** \brief lower value for integral part
The I state should converge to the exact heater output needed for the target temperature.
To prevent a long deviation from the target zone, this value limits the lower value.
A good start is 30 lower then the optimal value. You need to leave room for cooling.
 Overridden if EEPROM activated. */
#define HEATED_BED_PID_INTEGRAL_DRIVE_MIN 5

/** \brief P-gain.  Overridden if EEPROM activated. */
#define HEATED_BED_PID_PGAIN 53.74

/** \brief I-gain  Overridden if EEPROM activated.*/
#define HEATED_BED_PID_IGAIN 7.48

/** \brief Dgain.  Overridden if EEPROM activated.*/
#define HEATED_BED_PID_DGAIN 96.52

/** \brief maximum time the heater can be switched on. Max = 255.  Overridden if EEPROM activated.*/
#define HEATED_BED_PID_MAX 255

/** \brief Extreme values to detect defect thermistors. */
#define MIN_DEFECT_TEMPERATURE -10
#define MAX_DEFECT_TEMPERATURE 300

#if HAVE_HEATED_BED == true && HEATED_BED_SENSOR_TYPE < 101

#define BED_ANALOG_INPUTS 1
#define BED_SENSOR_INDEX EXT0_ANALOG_INPUTS + EXT1_ANALOG_INPUTS
#define BED_ANALOG_CHANNEL ACCOMMA1 HEATED_BED_SENSOR_PIN

#else

#define BED_ANALOG_INPUTS 0
#define BED_SENSOR_INDEX HEATED_BED_SENSOR_PIN
#define BED_ANALOG_CHANNEL

#endif // HAVE_HEATED_BED==true && HEATED_BED_SENSOR_TYPE<101

/** \brief Allows to choose whether the setpoint and the current value of the heat bed temperature shall be compensated so that the temperature offset which is caused by the printing plate is reduced */
#define FEATURE_HEAT_BED_TEMP_COMPENSATION 1 // 1 = on, 0 = off

#if FEATURE_HEAT_BED_TEMP_COMPENSATION
/** \brief The following vlaue must be NumberOfTemperatures -1 */
#define BED_TEMP_COMPENSATION_INDEX_MAX 5
/** \brief The following values represent the setpoint temperatures */
#define BED_SETPOINT_TEMPERATURES \
    { 60, 80, 100, 120, 140, 160 }
/** \brief The following values represent the real temperature which is measured at the surface of the printing bed in case the temperature sensor delivers the setpoint temperatures */
#define BED_MEASURED_TEMPERATURES \
    { 58, 76, 94, 112, 130, 150 }
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION

// ##########################################################################################
// ##   Configuration of the 4. temperature sensor
// ##########################################################################################

/** \brief Analog pin of analog sensor to read temperature of the RF2000's optional sensor port.  */
#define RESERVE_ANALOG_TEMP_PIN TEMP_3_PIN
//14 passt für Thermistor NTC 3950 100k Ohm
#define RESERVE_ANALOG_SENSOR_TYPE 14

#define RESERVE_ANALOG_INPUTS 1
#define RESERVE_SENSOR_INDEX EXT0_ANALOG_INPUTS + EXT1_ANALOG_INPUTS + BED_ANALOG_INPUTS
#define RESERVE_ANALOG_CHANNEL ACCOMMA1 TEMP_3_PIN

// ##########################################################################################
// ##   Configuration of the endstops
// ##########################################################################################

#define Z_ENDSTOP_MAX_HYSTERESIS 0.1f //mm

/** \brief By default all endstops are pulled up to HIGH. You need a pullup if you
use a mechanical endstop connected with GND. Set value to false for no pullup
on this endstop. */
#define ENDSTOP_PULLUP_X_MIN true
#define ENDSTOP_PULLUP_Y_MIN true
#define ENDSTOP_PULLUP_Z_MIN true
#define ENDSTOP_PULLUP_X_MAX true
#define ENDSTOP_PULLUP_Y_MAX true
#define ENDSTOP_PULLUP_Z_MAX true

/** \brief set to true to invert the logic of the endstops */
#define ENDSTOP_X_MIN_INVERTING true
#define ENDSTOP_Y_MIN_INVERTING true
#define ENDSTOP_Z_MIN_INVERTING true
#define ENDSTOP_X_MAX_INVERTING true
#define ENDSTOP_Y_MAX_INVERTING true
#define ENDSTOP_Z_MAX_INVERTING true

/** \brief Set the values true where you have a hardware endstop. The Pin number is taken from pins.h. */
#define MIN_HARDWARE_ENDSTOP_X true
#define MIN_HARDWARE_ENDSTOP_Y true
#define MIN_HARDWARE_ENDSTOP_Z true
#define MAX_HARDWARE_ENDSTOP_X false
#define MAX_HARDWARE_ENDSTOP_Y false

/** \brief Define the type Z-Endstop-Installation */
// the RF2000 does not support to use the z-min and z-max endstops within one circle
#define FEATURE_CONFIGURABLE_Z_ENDSTOPS 0
// the RF2000 always has the z-min and z-max endstops
#define MAX_HARDWARE_ENDSTOP_Z true

/** \brief Set order of axis homing. Use HOME_ORDER_XYZ and replace XYZ with your order. */
#if FEATURE_MILLING_MODE
#define HOMING_ORDER_PRINT HOME_ORDER_XYZ //if you work with springloaded hotend or positive Z-Matrix home Z last! You might otherwise hit the surface.
#define HOMING_ORDER_MILL HOME_ORDER_ZXY
#else
#define HOMING_ORDER_PRINT HOME_ORDER_XYZ //if you work with springloaded hotend or positive Z-Matrix home Z last!! You might otherwise hit the surface.
#endif                                    // FEATURE_MILLING_MODE

/** \brief Sets direction of endstops when homing; 1=MAX, -1=MIN */
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
//gets inverted when searching home dir in milling mode:
#define Z_HOME_DIR -1

/** \brief If during homing the endstop is reached, how many mm should the printer move back for the second try */
#define ENDSTOP_X_BACK_MOVE 5.0f
#define ENDSTOP_Y_BACK_MOVE 5.0f
#define ENDSTOP_Z_BACK_MOVE float(Z_ENDSTOP_MAX_HYSTERESIS + Z_ENDSTOP_DRIVE_OVER) //0.1mm sind theoretische maximale hysterese beim Schalter loslassen. Original RF2000: <0.01

/** \brief For higher precision you can reduce the speed for the second test on the endstop
during homing operation. The homing speed is divided by the value. 1 = same speed, 2 = half speed */
#define ENDSTOP_X_RETEST_REDUCTION_FACTOR 20
#define ENDSTOP_Y_RETEST_REDUCTION_FACTOR 20
#define ENDSTOP_Z_RETEST_REDUCTION_FACTOR 20

/** \brief When you have several endstops in one circuit you need to disable it after homing by moving a
small amount back. This is also the case with H-belt systems. */
#define LEAVE_Z_MAX_ENDSTOP_AFTER_HOME 2.0f // [mm] positive value, because homedir is known.

// ##########################################################################################
// ##   Motor Current & Stepper configurations
// ##########################################################################################

/** \brief Motor Current MAX setting */
#define MOTOR_CURRENT_MAX \
    { 145, 140, 120, 105, 105 } // Values 0-255 (126 = ~2A), order: driver 1 (x), driver 2 (y), driver 3 (z), driver 4 (extruder 1), driver 5 (extruder 2)

/** \brief Motor Current settings at start: Tweak with menu for better silence <-> stability
// The RF2000 has one more stepper and the same 8A-24V power supply as RF1000. We think that this is the reason for lower stepper currents set by conrad renkforce - but dont know.
// I increased MAX a little bit over stock settings, but decreased normal settings to fairly low power and noise. Take that into account when tuning in your steppers according your needs.
*/
#define MOTOR_CURRENT_NORMAL \
    { 110, 110, 105, 90, 90 }
#define MOTOR_CURRENT_MIN EXTRUDER_CURRENT_PAUSED

/** \brief Specifies whether the firmware shall wait a short time after turning on of the stepper motors - this shall avoid that the first steps are sent to the stepper before it is ready */
#define STEPPER_ON_DELAY 25 // [ms]

/** \brief If your stepper needs a longer high signal then given, you can add a delay here.
The delay is realized as a simple loop wasting time, which is not available for other
computations. So make it as low as possible. For the most common drivers no delay is needed, as the
included delay is already enough. */
#define STEPPER_HIGH_DELAY 0

// ##########################################################################################
// ##   miscellaneous configurations
// ##########################################################################################

/** \brief number of analog input signals. Normally 1 for each temperature sensor */
#define ANALOG_INPUTS (EXT0_ANALOG_INPUTS + EXT1_ANALOG_INPUTS + BED_ANALOG_INPUTS + RESERVE_ANALOG_INPUTS)

#if ANALOG_INPUTS > 0
/** \brief Channels are the MUX-part of ADMUX register */
#define ANALOG_INPUT_CHANNELS \
    { EXT0_ANALOG_CHANNEL EXT1_ANALOG_CHANNEL BED_ANALOG_CHANNEL RESERVE_ANALOG_CHANNEL }
#endif // ANALOG_INPUTS>0

/** \brief For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1 */
#define X_ENABLE_ON 1
#define Y_ENABLE_ON 1
#define Z_ENABLE_ON 1

/** \brief Inverting axis direction */
#define INVERT_X_DIR true
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false

#define XYZ_STEPPER_HIGH_DELAY 100 // [us] speed for moveZ
#define XYZ_STEPPER_LOW_DELAY 100  // [us] speed for moveZ

// ##########################################################################################
// ##   Script configuration
// ##########################################################################################

/** \brief Automatic filament change, unmounting of the filament - ensure that G1 does not attempt to extrude more than EXTRUDE_MAXLENGTH */
#define UNMOUNT_FILAMENT_SCRIPT_SOFT "M3914 P4500"
#define UNMOUNT_FILAMENT_SCRIPT_HARD "M109 S" xstr(UI_SET_PRESET_EXTRUDER_TEMP_PLA) "\nG21\nG92 E0\nG1 E-90 F500\nG92 E0\nM104 S0"

/** \brief Automatic filament mount, mounting of the filament with heating. Stop @ XXXX digits */
#define MOUNT_FILAMENT_SCRIPT_SOFT "M3913 P3500 F3"
#define MOUNT_FILAMENT_SCRIPT_HARD "M109 S" xstr(UI_SET_PRESET_EXTRUDER_TEMP_ABS) "\nG21\nG92 E0\nG1 E90 F100\nG92 E0\nM104 S0"

// ##########################################################################################
// ##   PWM configuration
// ##########################################################################################

/** \brief Heaters: speed of the PWM signal,          0 = 15.25Hz, 1 = 30.51Hz, 2 = 61.03Hz */
#define HEATER_PWM_SPEED 1

/** \brief Extruder Coolers: speed of the PWM signal, 0 = 15.25Hz, 1 = 30.51Hz, 2 = 61.03Hz */
#define COOLER_PWM_SPEED 0

// ##########################################################################################
// ##   Part Fan configuration
// ##########################################################################################

/** \brief Part Fan:                                  1 = 15.3Hz,  2 = 7.62Hz, 3 = 5.1Hz, 4 = 3.82Hz,  5 = 3.06Hz,  6 = 2.55Hz, 7 = 2.1Hz */
#define PART_FAN_DEFAULT_PWM_SPEED_DIVISOR 3
/** \brief the following values can be used in order to fine-tune the operating range of the cooler: */
#define PART_FAN_PWM_MIN 1   // 1 ... 254 equals 1 ... 99 %
#define PART_FAN_PWM_MAX 160 // 1 ... 254 equals 1 ... 99 %

/** Some fans won't start for low values, but would run if started with higher power at the beginning.
This defines the full power duration before returning to set value. Time is in milliseconds
Only values which are a factor of 10ms or 0==OFF will work precisely */
#define PART_FAN_KICKSTART_THRESHOLD 128   // 0 ... 255 equals 0 ... 100 % (unscaled!)
#define PART_FAN_KICKSTART_TIME_OFF_ON 400 // [ms]
#define PART_FAN_KICKSTART_TIME_BOOST 100  // [ms]

/** \brief use PDM instead of PWM for part fan */
// --> set EEPROM value or change mode in printers menu

// ##########################################################################################
// ##   configuration of the speed vs. cpu usage
// ##########################################################################################

/**
\brief The firmware can only handle a limited interrupt frequency cleanly.
If you need higher speeds
a faster solution is needed, and this is to double/tripple/quadruple the steps in one interrupt call.
This is like reducing your 1/16th microstepping to 1/8 or 1/4. It is much cheaper then 1 or 3
additional stepper interrupts with all it's overhead. As a result you can gain higher Output-Step-Frequencies.

STEP_PACKING_MIN_INTERVAL can be changed in Menu-> Configuration->Stepper->DblFq:
*/
#define STEP_PACKING_MIN_INTERVAL 3300 // (F_CPU / safe Steprate)

#define MIN_STEP_PACKING_MIN_INTERVAL 3300
#define MAX_STEP_PACKING_MIN_INTERVAL 5000

/** \brief Number of moves we can cache in advance.
This number of moves can be cached in advance. If you wan't to cache more, increase this. Especially on
many very short moves the cache may go empty. The minimum value is 5. */
#if SDSUPPORT
#define MOVE_CACHE_SIZE 17
#else
#define MOVE_CACHE_SIZE 24
#endif

/** \brief Low filled cache size.
If the cache contains less then MOVE_CACHE_LOW segments, the time per segment is limited to LOW_TICKS_PER_MOVE clock cycles.
If a move would be shorter, the feedrate will be reduced. This should prevent buffer underflows. Set this to 0 if you
don't care about empty buffers during print. */
#define MOVE_CACHE_LOW 10

/** \brief Cycles per move, if move cache is low.
This value must be high enough, that the buffer has time to fill up. The problem only occurs at the beginning of a print or
if you are printing many very short segments at high speed.*/
#define LOW_TICKS_PER_MOVE 250000

//For configuration of speed vs. cpu RF_MICRO_STEPS_ @ CONFIGURATION.h as well!

// ##########################################################################################
// ##   Extruder control
// ##########################################################################################

/** \brief Enable advance algorithm.
Without a correct adjusted advance algorithm, you get blobs at points, where acceleration changes. The
effect increases with speed and acceleration difference. Using the advance method decreases this effect.
For more informations, read the wiki. */
#define USE_ADVANCE 1

// ##########################################################################################
// ##   Configuration of the heat bed z compensation
// ##########################################################################################

#if FEATURE_HEAT_BED_Z_COMPENSATION

/** \brief Specifies if you want to adjust minimal and maximum compensation steps to first layer */
#define AUTOADJUST_STARTMADEN_AUSSCHLUSS 0.352f

/** \brief Specifies until which height the z compensation must complete */
#define HEAT_BED_Z_COMPENSATION_MAX_MM 10 // [mm]

/** \brief Specifies from which height on the z compensation shall be performed
Below this value the z compensation will only change the z axis so that a constant distance to the heat bed is hold (this is good for the first layer).
Above this value the z compensation will distribute the roughness of the surface over the layers until HEAT_BED_Z_COMPENSATION_MAX_MM is reached. */
#define HEAT_BED_Z_COMPENSATION_MIN_MM float(0.33) // [mm]

/** \brief Configuration of the heat bed scan */
#if NUM_EXTRUDER == 2
#define HEAT_BED_SCAN_X_START_MM 0               // [mm] from the left border of the heat bed
#define HEAT_BED_SCAN_X_END_MM 0                 // [mm] from the right border of the heat bed
#define HEAT_BED_SCAN_X_STEP_SIZE_MM 20          // [mm]
#define HEAT_BED_SCAN_X_STEP_SIZE_MIN_MM 10      // [mm]
#define HEAT_BED_SCAN_X_CALIBRATION_POINT_MM 100 // [mm] from the left border of the heat bed

#define HEAT_BED_SCAN_Y_START_MM 30              // [mm] from the front border of the heat bed
#define HEAT_BED_SCAN_Y_END_MM 5                 // [mm] from the back border of the heat bed
#define HEAT_BED_SCAN_Y_STEP_SIZE_MM 20          // [mm]
#define HEAT_BED_SCAN_Y_STEP_SIZE_MIN_MM 10      // [mm]
#define HEAT_BED_SCAN_Y_CALIBRATION_POINT_MM 100 // [mm] from the front border of the heat bed

#else /* NUM_EXTRUDER != 2 -> */

#define HEAT_BED_SCAN_X_START_MM 15              // [mm] from the left border of the heat bed
#define HEAT_BED_SCAN_X_END_MM 5                 // [mm] from the right border of the heat bed
#define HEAT_BED_SCAN_X_STEP_SIZE_MM 20          // [mm]
#define HEAT_BED_SCAN_X_STEP_SIZE_MIN_MM 10      // [mm]
#define HEAT_BED_SCAN_X_CALIBRATION_POINT_MM 100 // [mm] from the left border of the heat bed

#define HEAT_BED_SCAN_Y_START_MM 30              // [mm] from the front border of the heat bed
#define HEAT_BED_SCAN_Y_END_MM 5                 // [mm] from the back border of the heat bed
#define HEAT_BED_SCAN_Y_STEP_SIZE_MM 20          // [mm]
#define HEAT_BED_SCAN_Y_STEP_SIZE_MIN_MM 10      // [mm]
#define HEAT_BED_SCAN_Y_CALIBRATION_POINT_MM 100 // [mm] from the front border of the heat bed
#endif                                           // NUM_EXTRUDER == 2

//Nibbels: increased from 500 to 1000 in order to avoid problems with Dip-Down-Hotends
#define HEAT_BED_SCAN_Z_START_MM 1.0f // [mm]

#define HEAT_BED_SCAN_CONTACT_PRESSURE_DELTA 10 // [digits]
#define HEAT_BED_SCAN_RETRY_PRESSURE_DELTA 5    // [digits]
#define HEAT_BED_SCAN_IDLE_PRESSURE_DELTA 0     // [digits]
#define HEAT_BED_SCAN_IDLE_PRESSURE_MIN -7500   // [digits]
#define HEAT_BED_SCAN_IDLE_PRESSURE_MAX 7500    // [digits]

#define HEAT_BED_SCAN_UP_FAST_MM -0.02f   // [mm]
#define HEAT_BED_SCAN_DOWN_FAST_MM 0.1f   // [mm]
#define HEAT_BED_SCAN_UP_SLOW_MM -0.004f  // [mm]
#define HEAT_BED_SCAN_DOWN_SLOW_MM 0.004f // [mm]

#define HEAT_BED_SCAN_FAST_STEP_DELAY_MS 1   // [ms]
#define HEAT_BED_SCAN_SLOW_STEP_DELAY_MS 100 // [ms]
#define HEAT_BED_SCAN_IDLE_DELAY_MS 250      // [ms]

#define HEAT_BED_SCAN_RETRIES 5                                       // [-]
#define HEAT_BED_SCAN_PRESSURE_READS 15                               // [-]
#define HEAT_BED_SCAN_PRESSURE_TOLERANCE 15                           // [digits]
#define HEAT_BED_SCAN_PRESSURE_READ_DELAY_MS 15                       // [ms]
#define HEAT_BED_SCAN_DELAY 1000                                      // [ms]
#define HEAT_BED_SCAN_ALIGN_EXTRUDERS_ABORT_DELAY (unsigned long)3600 // [s]

#define PRECISE_HEAT_BED_SCAN_WARMUP_DELAY (uint32_t)600      // [s]
#define PRECISE_HEAT_BED_SCAN_CALIBRATION_DELAY (uint32_t)600 // [s]
#define PRECISE_HEAT_BED_SCAN_BED_TEMP_PLA 60                 // [°C]
#define PRECISE_HEAT_BED_SCAN_BED_TEMP_ABS 100                // [°C]
#define PRECISE_HEAT_BED_SCAN_EXTRUDER_TEMP_SCAN 100          // [°C]
#define PRECISE_HEAT_BED_SCAN_EXTRUDER_TEMP_PLA 210           // [°C]
#define PRECISE_HEAT_BED_SCAN_EXTRUDER_TEMP_ABS 240           // [°C]
#define PRECISE_HEAT_BED_SCAN_EXTRUDER_TEMP_ABS 240           // [°C]

// configuration for the head bet offset search (M3900 command)
#define SEARCH_HEAT_BED_OFFSET_CONTACT_PRESSURE_DELTA 33 // [digits]
#define SEARCH_HEAT_BED_OFFSET_RETRY_PRESSURE_DELTA 30   // [digits]
#define SEARCH_HEAT_BED_OFFSET_IDLE_PRESSURE_DELTA 0     // [digits]
// scan position defined by the index of the heat bed matrix, counting from 1
#define SEARCH_HEAT_BED_OFFSET_SCAN_POSITION_INDEX_X 5
#define SEARCH_HEAT_BED_OFFSET_SCAN_POSITION_INDEX_Y 5
#define SEARCH_HEAT_BED_OFFSET_SCAN_POSITION_RAND_MM 3
// number of scanning iterations 2+
#define SEARCH_HEAT_BED_OFFSET_SCAN_ITERATIONS 1

#endif // FEATURE_HEAT_BED_Z_COMPENSATION

// ##########################################################################################
// ##   Configuration of the work part z compensation
// ##########################################################################################

#if FEATURE_WORK_PART_Z_COMPENSATION

/** \brief Specifies the maximal static z-offset which can be configured.
*/
#define WORK_PART_MAX_STATIC_Z_OFFSET_MM 10 // [mm]

/** \brief Configuration of the work part scan */
#define WORK_PART_SCAN_X_START_MM 5         // [mm] from the left border of the work bed
#define WORK_PART_SCAN_X_END_MM 220         // [mm]
#define WORK_PART_SCAN_X_STEP_SIZE_MM 20    // [mm]
#define WORK_PART_SCAN_X_STEP_SIZE_MIN_MM 5 // [mm]

#define WORK_PART_SCAN_Y_START_MM 55        // [mm] from the front border of the work bed
#define WORK_PART_SCAN_Y_END_MM 170         // [mm]
#define WORK_PART_SCAN_Y_STEP_SIZE_MM 20    // [mm]
#define WORK_PART_SCAN_Y_STEP_SIZE_MIN_MM 5 // [mm]

#define WORK_PART_SCAN_UP_FAST_MM -0.025f   // [mm]
#define WORK_PART_SCAN_UP_SLOW_MM -0.005f   // [mm]
#define WORK_PART_SCAN_DOWN_SLOW_MM 0.0125f // [mm]
#define WORK_PART_SCAN_DOWN_FAST_MM 0.5f    // [mm]

#define WORK_PART_SCAN_FAST_STEP_DELAY_MS 5   // [ms]
#define WORK_PART_SCAN_SLOW_STEP_DELAY_MS 100 // [ms]
#define WORK_PART_SCAN_IDLE_DELAY_MS 250      // [ms]

#if MILLER_TYPE == MILLER_TYPE_ONE_TRACK
#define WORK_PART_SCAN_CONTACT_PRESSURE_DELTA MT1_WORK_PART_SCAN_CONTACT_PRESSURE_DELTA
#define WORK_PART_SCAN_RETRY_PRESSURE_DELTA MT1_WORK_PART_SCAN_RETRY_PRESSURE_DELTA
#elif MILLER_TYPE == MILLER_TYPE_TWO_TRACKS
#define WORK_PART_SCAN_CONTACT_PRESSURE_DELTA MT2_WORK_PART_SCAN_CONTACT_PRESSURE_DELTA
#define WORK_PART_SCAN_RETRY_PRESSURE_DELTA MT2_WORK_PART_SCAN_RETRY_PRESSURE_DELTA
#endif // MILLER_TYPE

#define WORK_PART_SCAN_IDLE_PRESSURE_DELTA 0   // [digits]
#define WORK_PART_SCAN_IDLE_PRESSURE_MIN -5000 // [digits]
#define WORK_PART_SCAN_IDLE_PRESSURE_MAX 5000  // [digits]

#define WORK_PART_SCAN_RETRIES 3                 // [-]
#define WORK_PART_SCAN_PRESSURE_READS 15         // [-]
#define WORK_PART_SCAN_PRESSURE_TOLERANCE 15     // [digits]
#define WORK_PART_SCAN_PRESSURE_READ_DELAY_MS 15 // [ms]
#define WORK_PART_SCAN_DELAY 1000                // [ms]

#define WORK_PART_SCAN_Z_START_MM 15 // [mm]

#endif // FEATURE_WORK_PART_Z_COMPENSATION

// ##########################################################################################
// ##   configuration of the pause functionality
// ##########################################################################################

/** \brief Configuration of the pause steps */
//look here if you want to prevent clamp crashes while milling! choose your pause position right.
#define DEFAULT_PAUSE_MM_X_MILL 0            // [mm]
#define DEFAULT_PAUSE_MM_X_PRINT 0           // [mm]
#define DEFAULT_PAUSE_MM_Y_MILL 200          // [mm]
#define DEFAULT_PAUSE_MM_Y_PRINT 200         // [mm]
#define DEFAULT_PAUSE_MM_Z_MILL 20           // [mm]
#define DEFAULT_PAUSE_MM_Z_PRINT 30          // [mm]
#define DEFAULT_PAUSE_MM_E SCRIPT_RETRACT_MM // [mm] Zahl wird immer vorzeichenlos als Retract benutzt

#define PAUSE_COOLDOWN 100 // [°C] 0=Off and 1..255=Temp down while paused

// ##########################################################################################
// ##   configuration of the z origin search
// ##########################################################################################

#if FEATURE_FIND_Z_ORIGIN

#define SEARCH_Z_ORIGIN_CONTACT_PRESSURE_DELTA 500 // [digits]
#define SEARCH_Z_ORIGIN_BREAKOUT_DELAY 100         // [ms]
#define SEARCH_Z_ORIGIN_BED_UP_MM -0.05f           // [mm]
#define SEARCH_Z_ORIGIN_BED_DOWN_MM 0.025f         // [mm]

/** \brief The following commands are executed before the z-origin is set to 0. */
#define FIND_Z_ORIGIN_SCRIPT "G91\nG1 Z15 F5000"

#endif // FEATURE_FIND_Z_ORIGIN

// ##########################################################################################
// ##   debugging
// ##########################################################################################

#if FEATURE_MILLING_MODE

/** \brief Enables debug outputs from the work part scan */
#define DEBUG_WORK_PART_SCAN 0 // 1 = on, 0 = off

#if FEATURE_FIND_Z_ORIGIN

/** \brief Enables debug outputs from the search of the z-origin */
#define DEBUG_FIND_Z_ORIGIN 0 // 1 = on, 0 = off

#endif // FEATURE_FIND_Z_ORIGIN

#endif // FEATURE_MILLING_MODE

// ##########################################################################################
// ##   configuration of the 230V output
// ##########################################################################################

#if FEATURE_230V_OUTPUT

/** \brief Set the 230V output default  */
#define OUTPUT_230V_DEFAULT_ON 0 // 1 = on, 0 = off

#endif // FEATURE_230V_OUTPUT

// ##########################################################################################
// ##   configuration of the FET outputs
// ##########################################################################################

#if FEATURE_24V_FET_OUTPUTS

/** \brief Set the 24V FET output defaults  */
#define FET1_DEFAULT_ON 0 // 1 = on, 0 = off
#define FET2_DEFAULT_ON 0 // 1 = on, 0 = off
#define FET3_DEFAULT_ON 0 // 1 = on, 0 = off

#endif // FEATURE_24V_FET_OUTPUTS

// ##########################################################################################
// ##   configuration of the RGB light effects
// ##########################################################################################

#if FEATURE_RGB_LIGHT_EFFECTS

/** \brief Specfies the default RGB light mode */
#define RGB_LIGHT_DEFAULT_MODE RGB_MODE_WHITE

/** \brief Specifies the time interval after which the RGB light status switches from idle to color change */
#define RGB_LIGHT_COLOR_CHANGE_DELAY 30 // [s]

/** \brief Specfies the temperature tolerance for the switching of the RGB light */
#define RGB_LIGHT_TEMP_TOLERANCE 5.0 // [°C]

/** \brief Specifies the color for heating up */
#define RGB_HEATING_R 255
#define RGB_HEATING_G 0
#define RGB_HEATING_B 0

/** \brief Specifies the color for printing */
#define RGB_PRINTING_R 255
#define RGB_PRINTING_G 255
#define RGB_PRINTING_B 255

/** \brief Specifies the color for cool down */
#define RGB_COOLING_R 0
#define RGB_COOLING_G 0
#define RGB_COOLING_B 255

/** \brief Specifies the color for idle */
#define RGB_IDLE_R 255
#define RGB_IDLE_G 255
#define RGB_IDLE_B 255

/** \brief Specifies the color for Manual input */
#define RGB_MANUAL_R 128
#define RGB_MANUAL_G 128
#define RGB_MANUAL_B 255

#endif // FEATURE_RGB_LIGHT_EFFECTS

#endif // RF2000_H
