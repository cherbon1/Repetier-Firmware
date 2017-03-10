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


#ifndef UI_LANG_H
#define UI_LANG_H

#if !defined(UI_DISPLAY_CHARSET) || UI_DISPLAY_CHARSET>3
#if MOTHERBOARD	== DEVICE_TYPE_RF1000
#define UI_DISPLAY_CHARSET 1
#else
#define UI_DISPLAY_CHARSET 2
#endif // MOTHERBOARD	== DEVICE_TYPE_RF1000
#endif // !defined(UI_DISPLAY_CHARSET) || UI_DISPLAY_CHARSET>3

#if UI_DISPLAY_CHARSET==0 // ASCII fallback
#define CHAR_RIGHT		'-'
#define CHAR_SELECTOR	'>'
#define CHAR_SELECTED	'*'
#define STR_auml		"ae"
#define STR_Auml		"Ae"
#define STR_uuml		"ue"
#define STR_Uuml		"Ue"
#define STR_ouml		"oe"
#define STR_Ouml		"Oe"
#define STR_szlig		"ss"
#endif

#if UI_DISPLAY_CHARSET==1 // HD44870 charset with knji chars
#define CHAR_RIGHT		0x7e
#define CHAR_SELECTOR	'>'
#define CHAR_SELECTED	'*'
#define STR_auml		"\xe1"
#define STR_Auml		"Ae"
#define STR_uuml		"\365"
#define STR_Uuml		"Ue"
#define STR_ouml		"\357"
#define STR_Ouml		"Oe"
#define STR_szlig		"\342"
#endif

#if UI_DISPLAY_CHARSET==2 // Western charset
#define CHAR_RIGHT		0xbc
#define CHAR_SELECTOR	0xf6
#define CHAR_SELECTED	'*'
#define STR_auml		"\204"
#define STR_Auml		"\216"
#define STR_uuml		"\201"
#define STR_Uuml		"\232"
#define STR_ouml		"\224"
#define STR_Ouml		"\211"
#define STR_szlig		"\160"
#endif

#if UI_DISPLAY_CHARSET==3 // U8glib
#define CHAR_RIGHT		187 //>>
#define CHAR_SELECTOR	255 //'>'
#define CHAR_SELECTED	254 //'*'
#define STR_auml		"\344"
#define STR_Auml		"\304"
#define STR_uuml		"\374"
#define STR_Uuml		"\334"
#define STR_ouml		"\366"
#define STR_Ouml		"\326"
#define STR_szlig		"\337"
#endif

#define TEST176			"176\260\261\262\263\264\265\266\267\270\271\272\273\274\275\276\277"
#define TEST192			"192\300\301\302\303\304\305\306\307\310\311\312\313\314\315\316\317"
#define TEST208			"208\320\321\322\323\324\325\326\327\330\331\332\333\334\335\336\337"
#define TEST224			"224\340\341\342\343\344\345\346\347\350\351\352\353\354\355\356\357"


// ##########################################################################################
// ##    English
// ##########################################################################################

// At first all terms in english are defined. After that the selected language
// can overwrite the definition. That way new strings are at least in english
// available.

#if !defined(UI_LANGUAGE) || UI_LANGUAGE==0
// Nibbels

#define UI_TEXT_ON						"On"
#define UI_TEXT_OFF						"Off"
#define UI_TEXT_0						"0"
#define UI_TEXT_1						"1"
#define	UI_TEXT_UNKNOWN					"?"
#define UI_TEXT_NA						"N/A"						// Output for not available
#define UI_TEXT_YES						"Yes"
#define UI_TEXT_NO						"No"
#define UI_TEXT_SEL						"\003"
#define UI_TEXT_NOSEL					"\004"
#define UI_TEXT_PRINT_POS				"Printing..."
#define UI_TEXT_MILL_POS				"Milling..."
#define	UI_TEXT_START_MILL				"Start Miller"
#define UI_TEXT_PAUSED					"Paused"
#define UI_TEXT_IDLE					"Idle"
#define UI_TEXT_NOSDCARD				"No SD Card"
#define UI_TEXT_BACK					"Back \001"
#define UI_TEXT_QUICK_SETTINGS			"Quick Settings"
#define UI_TEXT_CONFIGURATION			"Configuration"
#define UI_TEXT_POSITION				"Position"
#define UI_TEXT_EXTRUDER				"Extruder"
#define	UI_TEXT_EXTRUDER_OFFSET_X2		"[mm]: %OE ", ""
#define	UI_TEXT_EXTRUDER_OFFSET_Y2		"[mm]: %OF ", "" 
#define UI_TEXT_SD_CARD					"SD Card"
#define UI_TEXT_DEBUGGING				"Debugging"
#define UI_TEXT_HOME_ALL				"Home all"
#define UI_TEXT_HOME_X					"Home X"
#define UI_TEXT_HOME_Y					"Home Y"
#define UI_TEXT_HOME_Z					"Home Z"
#define UI_TEXT_DRIVING_FREE_Z			"Driving free Z"
#define UI_TEXT_PREHEAT_PLA				"Preheat PLA"
#define UI_TEXT_PREHEAT_ABS				"Preheat ABS"
#define UI_TEXT_COOLDOWN				"Cooldown"
#define UI_TEXT_SET_XY_ORIGIN			"Set XY Origin"
#define UI_TEXT_DISABLE_STEPPER			"Disable Stepper"
#define UI_TEXT_UNMOUNT_FILAMENT		"Unload Filament"
#define UI_TEXT_MOUNT_FILAMENT			"Load Filament"
#define UI_TEXT_X_POSITION				"Position X"
#define UI_TEXT_X_POS_FAST				"Pos. X fast"
#define UI_TEXT_Y_POSITION				"Position Y"
#define UI_TEXT_Y_POS_FAST				"Pos. Y fast"
#define UI_TEXT_Z_POSITION				"Position Z"
#define UI_TEXT_Z_POS_FAST				"Pos. Z fast"
#define UI_TEXT_BED_TEMP				"Temp. Bed:%Eb\002C"
#define UI_TEXT_EXTR0_TEMP				"Temp. 0  :%E0\002C"
#define UI_TEXT_EXTR1_TEMP				"Temp. 1  :%E1\002C"
#define UI_TEXT_EXTR2_TEMP				"Temp. 2  :%E2\002C"
#define UI_TEXT_EXTR0_OFF				"Extruder 0 off"
#define UI_TEXT_EXTR1_OFF				"Extruder 1 off"
#define UI_TEXT_EXTR2_OFF				"Extruder 2 off"
#define UI_TEXT_EXTR0_SELECT			"%X0 Select Extr.0"
#define UI_TEXT_EXTR1_SELECT			"%X1 Select Extr.1"
#define UI_TEXT_EXTR2_SELECT			"%X2 Select Extr.2"
#define UI_TEXT_SET_E_ORIGIN			"Set E Origin"
#define UI_TEXT_PRINT_X					"Print X :%ax"
#define UI_TEXT_PRINT_Y					"Print Y :%ay"
#define UI_TEXT_PRINT_Z					"Print Z :%az"
#define UI_TEXT_MILL_X					"Mill X  :%ax"
#define UI_TEXT_MILL_Y					"Mill Y  :%ay"
#define UI_TEXT_MILL_Z					"Mill Z  :%az"
#define UI_TEXT_PRINT_Z_DELTA			"Print:%az"
#define UI_TEXT_MOVE_X					"Move X  :%aX"
#define UI_TEXT_MOVE_Y					"Move Y  :%aY"
#define UI_TEXT_MOVE_Z					"Move Z  :%aZ"
#define UI_TEXT_MOVE_Z_DELTA			"Move:%aZ"
#define UI_TEXT_JERK					"X/Y-Jerk:%aj"
#define UI_TEXT_ZJERK					"Z-Jerk  :%aJ"
#define UI_TEXT_ACCELERATION			"Acceleration"
#define UI_TEXT_DBG_ECHO				"Echo   : %do"
#define UI_TEXT_DBG_INFO				"Info   : %di"
#define UI_TEXT_DBG_ERROR				"Errors : %de"
#define UI_TEXT_DBG_DRYRUN				"Dry Run: %dd"
#define UI_TEXT_PRINT_FILE				"Print File"
#define UI_TEXT_PAUSE_PRINT				"Pause Print"
#define UI_TEXT_CONTINUE_PRINT			"Continue Print"
#define UI_TEXT_MILL_FILE				"Mill File"
#define UI_TEXT_PAUSE_MILL				"Pause Mill"
#define UI_TEXT_CONTINUE_MILL			"Continue Mill"
#define UI_TEXT_UNMOUNT_CARD			"Unmount Card"   
#define UI_TEXT_MOUNT_CARD				"Mount Card"	 	
#define	UI_TEXT_SD_NOT_PRESENT			"Card missing"
#define UI_TEXT_DELETE_FILE				"Delete File"
#define UI_TEXT_FEEDRATE				"Feedrate"
#define UI_TEXT_FEED_MAX_X				"Max X :%fx"
#define UI_TEXT_FEED_MAX_Y				"Max Y :%fy"
#define UI_TEXT_FEED_MAX_Z				"Max Z :%fz"
#define UI_TEXT_FEED_HOME_X				"Home X:%fX"
#define UI_TEXT_FEED_HOME_Y				"Home Y:%fY"
#define UI_TEXT_FEED_HOME_Z				"Home Z:%fZ"
#define UI_TEXT_ACTION_XPOSITION4		"X:%x0 mm","Endstop min:%sx","Endstop max:%sX","%px"
#define UI_TEXT_ACTION_YPOSITION4		"Y:%x1 mm","Endstop min:%sy","Endstop max:%sY","%py"
#define UI_TEXT_ACTION_ZPOSITION4		"Z:%x2 mm %sC","Endstop min:%sz","Endstop max:%sZ","%pz"
#define UI_TEXT_ACTION_XPOSITION_FAST4	"X:%x0 mm","Endstop min:%sx","Endstop max:%sX","%px"
#define UI_TEXT_ACTION_YPOSITION_FAST4	"Y:%x1 mm","Endstop min:%sy","Endstop max:%sY","%py"
#define UI_TEXT_ACTION_ZPOSITION_FAST4	"Z:%x2 mm %sC","Endstop min:%sz","Endstop max:%sZ","%pz"
#define	UI_TEXT_ACTION_ZOFFSET			"Z:%z0 um","","",""
#define UI_TEXT_ACTION_EPOSITION_FAST2	"E:%x3 mm","1 click = 1 mm"
#define UI_TEXT_FANSPEED				"Fan Speed"
#define UI_TEXT_FAN_OFF					"Turn Fan off"
#define UI_TEXT_STEPPER_OFF				"Stepper off"
#define UI_TEXT_STEPPER_OFF2			"[s]: %is","0=never"
#define UI_TEXT_ALL_OFF					"All off"
#define UI_TEXT_ALL_OFF2				"[s]: %ip","0=never"
#define UI_TEXT_BEEPER					"Beeper: %db"
#define UI_TEXT_LIGHTS_ONOFF			"X19 Light: %lo"
#define UI_TEXT_OPERATING_MODE			"Mode: %OM"
#define UI_TEXT_Z_ENDSTOP_TYPE			"Z Type: %OZ"
#define UI_TEXT_HOTEND_TYPE	 			"Hotend: %ht"
#define UI_TEXT_MILLER_TYPE	 			"Miller: %mt"
#define UI_TEXT_GENERAL					"General"
#define UI_TEXT_EXTR_STEPS				"Steps/mm:%Se"
#define UI_TEXT_EXTR_START_FEED			"Start FR:%Xf"
#define UI_TEXT_EXTR_MAX_FEED			"Max FR:%XF"						
#define UI_TEXT_EXTR_ADVANCE_L			"Advance lin:%Xl"
#define UI_TEXT_EXTR_ADVANCE_K			"Advance quad:%Xa"
#define UI_TEXT_EXTR_MANAGER			"Control:%Xh"
#define UI_TEXT_EXTR_PGAIN				"DT/PID P:%Xp"
#define UI_TEXT_EXTR_IGAIN				"PID I:%Xi"
#define UI_TEXT_EXTR_DGAIN				"PID D:%Xd"
#define UI_TEXT_EXTR_DMIN				"Drive min:%Xm"
#define UI_TEXT_EXTR_DMAX				"Drive max:%XM"
#define UI_TEXT_EXTR_PMAX				"PID max:%XD"
#define UI_TEXT_STRING_HM_BANGBANG		"BangBang"
#define UI_TEXT_STRING_HM_PID			"PID"
#define UI_TEXT_STRING_ACTION			"Action:%la"
#define UI_TEXT_HEATING_EXTRUDER		"Heating Extruder"
#define UI_TEXT_HEATING_BED				"Heating Bed"
#define UI_TEXT_HEATING_UP				"Heating up..."
#define UI_TEXT_HEATING					"Heating..."
#define UI_TEXT_COOLING_DOWN			"Cooling down..."
#define UI_TEXT_OUTPUTTING_OBJECT		"Outputting..."
#define UI_TEXT_PAUSING					"Pausing..."
#define UI_TEXT_CONTINUING				"Continuing..."
#define UI_TEXT_KILLED					"Killed"
#define UI_TEXT_STEPPER_DISABLED		"Stepper disabled"
#define UI_TEXT_UPLOADING				"Uploading..."
#define UI_TEXT_PAGE_BUFFER				"Buffer:%oB"						
#define UI_TEXT_SET_Z_ORIGIN			"Set Z Origin"
#define	UI_TEXT_FIND_Z_ORIGIN			"Find Z Origin"
#define	UI_TEXT_TEST_STRAIN_GAUGE		"Test SG"
#define UI_TEXT_ZCALIB					"Z Calibration"
#define UI_TEXT_Z_OFFSET				"Z Offset"
#define UI_TEXT_SET_P1					"Set P1"
#define UI_TEXT_SET_P2					"Set P2"
#define UI_TEXT_SET_P3					"Set P3"
#define UI_TEXT_CALCULATE_LEVELING		"Calculate level."
#define UI_TEXT_LEVEL					"Level Delta"
#define UI_TEXT_EXTR_WAIT_RETRACT_TEMP	"Wait Temp.: %XT\002C"
#define UI_TEXT_EXTR_WAIT_RETRACT_UNITS "Wait Units: %XU mm"
#define UI_TEXT_SD_REMOVED				"SD Card removed"
#define UI_TEXT_SD_INSERTED				"SD Card inserted"
#define UI_TEXT_PRINTER_READY			"Printer ready."
#define UI_TEXT_MILLER_READY			"Miller ready."
#define UI_TEXT_DO_HEAT_BED_SCAN		"Matrix Scan"
#define UI_TEXT_DO_MHIER_BED_SCAN		"Offset Z-Scan"
#define UI_TEXT_HEAT_BED_SCAN_PLA		"Matrix Scan PLA"
#define UI_TEXT_HEAT_BED_SCAN_ABS		"Matrix Scan ABS"
#define UI_TEXT_DO_WORK_PART_SCAN		"Scan"
#define UI_TEXT_HEAT_BED_SCAN			"Heat Bed Scan"
#define UI_TEXT_OUTPUT_OBJECT			"Output Object"
#define UI_TEXT_PARK_HEAT_BED			"Park Heat Bed"
#define UI_TEXT_PAUSE					"Pause"
#define UI_TEXT_HOME					"Home"
#define UI_TEXT_Z_COMPENSATION			"Z Compensation"
#define UI_TEXT_CHANGE_MODE				"Change Mode"
#define UI_TEXT_CHANGE_Z_TYPE			"Change Z Type"
#define UI_TEXT_CHANGE_HOTEND_TYPE		"Change Hotend"
#define UI_TEXT_CHANGE_MILLER_TYPE		"Change Miller"
#define UI_TEXT_X_AXIS					"X-Axis"
#define UI_TEXT_Y_AXIS					"Y-Axis"
#define UI_TEXT_Z_AXIS					"Z-Axis"
#define UI_TEXT_RESET					"Restart"
#define UI_TEXT_RESET_ACK				"Restart now?","","%mYYes","%mNNo"
#define UI_TEXT_HEAT_BED_SCAN_ABORTED	"Scan aborted"
#define UI_TEXT_HEAT_BED_SCAN_DONE		"Scan completed"
#define UI_TEXT_HEAT_BED_SCAN_OFFSET_MIN "mOffset: %HO um"
#define UI_TEXT_ALIGN_EXTRUDERS			"Align Extruders"
#define UI_TEXT_PRINT_MODE				"Printer"
#define UI_TEXT_MILL_MODE				"Miller"
#define UI_TEXT_Z_SINGLE				"Single"
#define UI_TEXT_Z_CIRCUIT				"Circuit"
#define UI_TEXT_Z_MODE					"Z Scale: %zm"
#define UI_TEXT_Z_MODE_MIN				"Z Min"
#define UI_TEXT_HOTEND_V1				"V1"
#define UI_TEXT_HOTEND_V2				"V2"
#define UI_TEXT_Z_COMPENSATION_ACTIVE	"Cmp"
#define	UI_TEXT_FIND_Z_ORIGIN_ABORTED	"Search aborted"
#define UI_TEXT_FIND_Z_ORIGIN_DONE		"Search completed"
#define	UI_TEXT_TEST_STRAIN_GAUGE_ABORTED	"Test aborted"
#define UI_TEXT_TEST_STRAIN_GAUGE_DONE		"Test completed"
#define UI_TEXT_WORK_PART_SCAN			"Work Part Scan"
#define UI_TEXT_WORK_PART_SCAN_ABORTED	"Scan aborted"
#define UI_TEXT_WORK_PART_SCAN_DONE		"Scan completed"
#define UI_TEXT_SET_Z_MATRIX_WORK_PART	"Set Z Matrix:%WP"
#define UI_TEXT_SET_Z_MATRIX_HEAT_BED	"Set Z Matrix:%HB"
#define UI_TEXT_SET_SCAN_XY_START		"Set XY Start"
#define UI_TEXT_SET_SCAN_XY_END			"Set XY End"
#define UI_TEXT_SET_SCAN_DELTA_X		"Set dX: %Dxmm"
#define UI_TEXT_SET_SCAN_DELTA_Y		"Set dY: %Dymm"
#define UI_TEXT_STRAIN_GAUGE			"F: %s1 L: %Fs%%%"
#define UI_TEXT_STRAIN_GAUGE_SPEED		"F: %s1 V: %om%%%"
#define UI_TEXT_SERVICE					"SERVICE"
#define UI_TEXT_SERVICE_TIME			"Until service"
#define	UI_TEXT_PRINTING_ROOM_LIGHT		"PRL: "	
#define UI_TEXT_WHITE					"White"
#define	UI_TEXT_COLOR					"Auto"
#define UI_TEXT_MANUAL					"Manual"
#define UI_TEXT_WRONG_FIRMWARE			"Wrong Firmware"
#define UI_TEXT_MOVE_MODE_SINGLE_STEPS	"Single Steps"
#define UI_TEXT_MOVE_MODE_SINGLE_MOVE	"Single Move"
#define UI_TEXT_MOVE_MODE_1_MM			"1 mm"
#define UI_TEXT_MOVE_MODE_10_MM			"10 mm"
#define UI_TEXT_MOVE_MODE_50_MM			"50 mm"

#define UI_TEXT_ERROR					"Error:"
#define UI_TEXT_WARNING					"Warning:"
#define UI_TEXT_INFORMATION				"Information:"
#define UI_TEXT_SET_ORIGIN				"Set Origin"
#define UI_TEXT_AUTODETECT_PID			"Determine PID"
#define UI_TEXT_HOME_UNKNOWN			"Home unknown"
#define UI_TEXT_SAVING_FAILED			"Saving failed"
#define	UI_TEXT_EMERGENCY_PAUSE			"Emergency Pause"
#define UI_TEXT_EMERGENCY_STOP			"Emergency Block"
#define UI_TEXT_INVALID_MATRIX			"Invalid Matrix"
#define	UI_TEXT_MIN_REACHED				"Min reached"
#define	UI_TEXT_MAX_REACHED				"Max reached"
#define	UI_TEXT_TIMEOUT					"Timeout"
#define UI_TEXT_AUTODETECT_PID_DONE		"PID determined"
#define UI_TEXT_SENSOR_ERROR			"Sensor Error"
#define UI_TEXT_HEAT_BED_ZOFFSET_SEARCH_ABORTED "Scan aborted"

// Printtime output gets aggregated like <Days_5gisgits>UI_TEXT_PRINTTIME_DAYS<Hours>UI_TEXT_PRINTTIME_HOURS<Minutes>UI_TEXT_PRINTTIME_MINUTES
// ___88 days 12:45
#define UI_TEXT_PRINTTIME_DAYS			" days "
#define UI_TEXT_PRINTTIME_HOURS			":"
#define UI_TEXT_PRINTTIME_MINUTES		""
#define UI_TEXT_PRINT_TIME				"Printing time"
#define UI_TEXT_MILL_TIME				"Milling time"
#define UI_TEXT_PRINT_FILAMENT			"Filament printed"
#define UI_TEXT_POWER					"ATX Power on/off"
#define UI_TEXT_STRING_HM_DEADTIME		"Dead Time"
#define UI_TEXT_STRING_HM_SLOWBANG		"SlowBang"
#define UI_TEXT_STOP_PRINT				"Stop Print"
#define UI_TEXT_STOP_PRINT_ACK			"Stop Print now?","","%mYYes","%mNNo"
#define UI_TEXT_STOP_MILL				"Stop Mill"
#define UI_TEXT_STOP_MILL_ACK			"Stop Mill now?","","%mYYes","%mNNo"


#if MOTHERBOARD == DEVICE_TYPE_RF2000
	#define	UI_TEXT_RGB_LIGHT_MODE			"RGB Light: %li"
	#define UI_TEXT_230V_OUTPUT				"230V Output: %ou"
	#define UI_TEXT_FET1_OUTPUT				"X42 Mosfet1: %ol"
	#define UI_TEXT_FET2_OUTPUT				"X44 Mosfet2: %on"
	#define UI_TEXT_BAUDRATE				"Baudrate: %oc"
	#define UI_TEXT_ACTION_FANSPEED			"Fan Speed:%Fs%%%"
	#define UI_TEXT_FAN_25					"Fan to     25%%%"
	#define UI_TEXT_FAN_50					"Fan to     50%%%"
	#define UI_TEXT_FAN_75					"Fan to     75%%%"
	#define UI_TEXT_FAN_FULL				"Fan to    100%%%"
	#define UI_TEXT_PAGE_EXTRUDER			"E: %ec/%Ec\002C->%oC"
	#define UI_TEXT_PAGE_EXTRUDER1			"E0:%e0/%E0\002C->%o0"
	#define UI_TEXT_PAGE_EXTRUDER2			"E1:%e1/%E1\002C->%o1"
	#define UI_TEXT_PAGE_BED				"B: %eb/%Eb\002C->%ob"
	#define	UI_TEXT_EXTRUDER_OFFSET_X		"Extruder Offset X"
	#define	UI_TEXT_EXTRUDER_OFFSET_Y		"Extruder Offset Y"
	#define UI_TEXT_E_POSITION				"Position Extruder"
	#define	UI_TEXT_ACTIVE_EXTRUDER			"Active Extruder:%Oa"
	#define UI_TEXT_EXTR_ACCEL				"Acceleration:%XA"								
	#define UI_TEXT_EXTR_WATCH				"Stabilization Time:%Xw"
	#define UI_TEXT_SPEED_MULTIPLY			"Speed Multiply:%om%%%"
	#define UI_TEXT_FLOW_MULTIPLY			"Flow Multiply :%of%%%"						
	#define UI_TEXT_SHOW_MEASUREMENT		"Show Measurement"					
	#define UI_TEXT_RESET_MEASUREMENT		"Reset Measurement"
	#define UI_TEXT_MILLER_ONE_TRACK		"one track"
	#define UI_TEXT_MILLER_TWO_TRACKS		"two tracks"
	#define	UI_TEXT_RESTORE_DEFAULTS		"Restore Defaults"
	#define UI_TEXT_Z_MODE_SURFACE			"Surface"
	#define UI_TEXT_Z_MODE_Z_ORIGIN			"Z Origin"
	#define UI_TEXT_TEMPERATURE_MANAGER		"Temperatur Manager"
	#define UI_TEXT_OPERATION_DENIED		"Operation denied"
	#define	UI_TEXT_TEMPERATURE_WRONG		"Temperature wrong"
#else	
	#define	UI_TEXT_RGB_LIGHT_MODE			"RGB Light: %li"
	#define UI_TEXT_BAUDRATE				"Baudrate:%oc"
	#define UI_TEXT_ACTION_FANSPEED			"Fan Speed:%Fs%%%"
	#define UI_TEXT_FAN_25					"Fan to 25%%%"
	#define UI_TEXT_FAN_50					"Fan to 50%%%"
	#define UI_TEXT_FAN_75					"Fan to 75%%%"
	#define UI_TEXT_FAN_FULL				"Fan to 100%%%"
	#define UI_TEXT_PAGE_EXTRUDER			"E:%ec/%Ec\002C\176%oC"
	#define UI_TEXT_PAGE_EXTRUDER1			"E0:%e0/%E0\002C\176%o0"
	#define UI_TEXT_PAGE_EXTRUDER2			"E1:%e1/%E1\002C\176%o1"
	#define UI_TEXT_PAGE_BED				"B: %eb/%Eb\002C\176%ob"
	#define	UI_TEXT_EXTRUDER_OFFSET_X		"Extr. Offset X"
	#define	UI_TEXT_EXTRUDER_OFFSET_Y		"Extr. Offset Y"	
	#define UI_TEXT_E_POSITION				"Position Extr."
	#define	UI_TEXT_ACTIVE_EXTRUDER			"Active Extr.  %Oa"
	#define UI_TEXT_EXTR_ACCEL				"Accel:%XA"								
	#define UI_TEXT_EXTR_WATCH				"Stab. Time:%Xw"
	#define UI_TEXT_SPEED_MULTIPLY			"Speed Mul.:%om%%%"
	#define UI_TEXT_FLOW_MULTIPLY			"Flow Mul. :%of%%%"						
	#define UI_TEXT_SHOW_MEASUREMENT		"Show Meas."					
	#define UI_TEXT_RESET_MEASUREMENT		"Reset Meas."	
	#define UI_TEXT_MILLER_ONE_TRACK		"one t."
	#define UI_TEXT_MILLER_TWO_TRACKS		"two t."
	#define	UI_TEXT_RESTORE_DEFAULTS		"Restore Default"
	#define UI_TEXT_Z_MODE_SURFACE			"Surf."
	#define UI_TEXT_Z_MODE_Z_ORIGIN			"Z Ori."
	#define UI_TEXT_TEMPERATURE_MANAGER		"Temp. Manager"
	#define UI_TEXT_OPERATION_DENIED		"Operat. denied"
	#define	UI_TEXT_TEMPERATURE_WRONG		"Temp. wrong"
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000

//#endif
#else
//!defined(UI_LANGUAGE) || UI_LANGUAGE==0 Nibbels

// ##########################################################################################
// ##    German
// ##########################################################################################

//#if UI_LANGUAGE==1

#define UI_TEXT_ON						"An"
#define UI_TEXT_OFF						"Aus"
#define UI_TEXT_0						"0"
#define UI_TEXT_1						"1"
#define	UI_TEXT_UNKNOWN					"?"
#define UI_TEXT_NA						"nv"
#define UI_TEXT_YES						"Ja"
#define UI_TEXT_NO						"Nein"
#define UI_TEXT_SEL						"\003"
#define UI_TEXT_NOSEL					"\004"
#define UI_TEXT_PRINT_POS				"Drucke..."
#define UI_TEXT_MILL_POS				"Fr" STR_auml "se..."
#define	UI_TEXT_START_MILL				"Starte Fr" STR_auml "ser"
#define UI_TEXT_PAUSED					"Pausiert"
#define UI_TEXT_IDLE					"Leerlauf"
#define UI_TEXT_NOSDCARD				"Keine SD Karte"
#define UI_TEXT_BACK					"Zur" STR_uuml "ck \001"
#define UI_TEXT_CONFIGURATION			"Konfiguration"
#define UI_TEXT_POSITION				"Position"
#define UI_TEXT_EXTRUDER				"Extruder"
#define	UI_TEXT_EXTRUDER_OFFSET_X2		"[mm]: %OE ", ""
#define	UI_TEXT_EXTRUDER_OFFSET_Y2		"[mm]: %OF ", "" 
#define UI_TEXT_SD_CARD					"SD Karte"
#define UI_TEXT_DEBUGGING				"Debugging"
#define UI_TEXT_HOME_ALL				"Home alle"
#define UI_TEXT_HOME_X					"Home X"
#define UI_TEXT_HOME_Y					"Home Y"
#define UI_TEXT_HOME_Z					"Home Z"
#define UI_TEXT_DRIVING_FREE_Z			"Z freifahren"
#define UI_TEXT_PREHEAT_PLA				"Vorheizen PLA"
#define UI_TEXT_PREHEAT_ABS				"Vorheizen ABS"
#define UI_TEXT_COOLDOWN				"Abk" STR_uuml "hlen"
#define UI_TEXT_MOUNT_FILAMENT			"Filament laden"
#define UI_TEXT_X_POSITION				"Position X"
#define UI_TEXT_X_POS_FAST				"Pos. X schnell"
#define UI_TEXT_Y_POSITION				"Position Y"
#define UI_TEXT_Y_POS_FAST				"Pos. Y schnell"
#define UI_TEXT_Z_POSITION				"Position Z"
#define UI_TEXT_Z_POS_FAST				"Pos. Z schnell"
#define UI_TEXT_EXTR0_OFF				"Extruder 0 aus"
#define UI_TEXT_EXTR1_OFF				"Extruder 1 aus"
#define UI_TEXT_EXTR2_OFF				"Extruder 2 aus"
#define UI_TEXT_EXTR0_SELECT			"W" STR_auml "hle Extr. 0"
#define UI_TEXT_EXTR1_SELECT			"W" STR_auml "hle Extr. 1"
#define UI_TEXT_EXTR2_SELECT			"W" STR_auml "hle Extr. 2"
#define UI_TEXT_PRINT_X					"Drucken X:%ax"
#define UI_TEXT_PRINT_Y					"Drucken Y:%ay"
#define UI_TEXT_PRINT_Z					"Drucken Z:%az"
#define UI_TEXT_MILL_X					"Fr" STR_auml "sen X :%ax"
#define UI_TEXT_MILL_Y					"Fr" STR_auml "sen Y :%ay"
#define UI_TEXT_MILL_Z					"Fr" STR_auml "sen Z :%az"
#define UI_TEXT_PRINT_Z_DELTA			"Drucken:%az"
#define UI_TEXT_MOVE_X					"Bewegen X:%aX"
#define UI_TEXT_MOVE_Y					"Bewegen Y:%aY"
#define UI_TEXT_MOVE_Z					"Bewegen Z:%aZ"
#define UI_TEXT_MOVE_Z_DELTA			"Bewegen:%aZ"
#define UI_TEXT_ACCELERATION			"Beschleunigung"
#define UI_TEXT_DBG_ECHO				"Echo       :%do"
#define UI_TEXT_DBG_INFO				"Info       :%di"
#define UI_TEXT_DBG_ERROR				"Fehler     :%de"
#define UI_TEXT_DBG_DRYRUN				"Trockenlauf:%dd"
#define UI_TEXT_PRINT_FILE				"Drucke Datei"
#define UI_TEXT_PAUSE_PRINT				"Druck pausieren"
#define UI_TEXT_MILL_FILE				"Fr" STR_auml "se Datei"
#define UI_TEXT_UNMOUNT_CARD			"Unmount Karte"
#define UI_TEXT_MOUNT_CARD				"Mount Karte"						
#define UI_TEXT_SD_NOT_PRESENT			"Karte fehlt" 
#define UI_TEXT_DELETE_FILE				"Datei l" STR_ouml "schen"
#define UI_TEXT_FEEDRATE				"Feedrate"
#define UI_TEXT_FEED_HOME_X				"Home X:%fX"
#define UI_TEXT_FEED_HOME_Y				"Home Y:%fY"
#define UI_TEXT_FEED_HOME_Z				"Home Z:%fZ"
#define UI_TEXT_ACTION_XPOSITION4		"X:%x0 mm","Min Endstopp:%sx","Max Endstopp:%sX","%px"
#define UI_TEXT_ACTION_YPOSITION4		"Y:%x1 mm","Min Endstopp:%sy","Max Endstopp:%sY","%py"
#define UI_TEXT_ACTION_ZPOSITION4		"Z:%x2 mm %sC","Min Endstopp:%sz","Max Endstopp:%sZ","%pz"
#define UI_TEXT_ACTION_XPOSITION_FAST4	"X:%x0 mm","Min Endstopp:%sx","Max Endstopp:%sX","%px"
#define UI_TEXT_ACTION_YPOSITION_FAST4	"Y:%x1 mm","Min Endstopp:%sy","Max Endstopp:%sY","%py"
#define UI_TEXT_ACTION_ZPOSITION_FAST4	"Z:%x2 mm %sC","Min Endstopp:%sz","Max Endstopp:%sZ","%pz"
#define	UI_TEXT_ACTION_ZOFFSET			"Z:%z0 um","","",""
#define UI_TEXT_ACTION_EPOSITION_FAST2	"E:%x3 mm","1 klick = 1 mm"
#define UI_TEXT_FANSPEED				"L" STR_uuml "fter"
#define UI_TEXT_STEPPER_OFF				"Motor aus"
#define UI_TEXT_STEPPER_OFF2			"[s]: %is","0=nie"
#define UI_TEXT_ALL_OFF					"Alles aus"
#define UI_TEXT_ALL_OFF2				"[s]: %ip","0=nie"
#define UI_TEXT_BEEPER					"Piepser:%db"
#define UI_TEXT_LIGHTS_ONOFF			"X19 Licht: %lo"
#define UI_TEXT_OPERATING_MODE			"Modus: %OM"
#define UI_TEXT_Z_ENDSTOP_TYPE			"Z Typ: %OZ"
#define UI_TEXT_HOTEND_TYPE	 			"Hotend: %ht"
#define UI_TEXT_MILLER_TYPE	 			"Fr" STR_auml "ser: %mt"
#define UI_TEXT_GENERAL					"Allgemein"
#define UI_TEXT_EXTR_MAX_FEED			"Max FR:%XF"
#define UI_TEXT_EXTR_ADVANCE_L			"Advance lin:%Xl"
#define UI_TEXT_EXTR_ADVANCE_K			"Advance quad:%Xa"
#define UI_TEXT_EXTR_MANAGER			"Regler:%Xh"
#define UI_TEXT_EXTR_PGAIN				"DT/PID P:%Xp"
#define UI_TEXT_EXTR_IGAIN				"PID I:%Xi"
#define UI_TEXT_EXTR_DGAIN				"PID D:%Xd"
#define UI_TEXT_EXTR_DMIN				"Drive Min:%Xm"
#define UI_TEXT_EXTR_DMAX				"Drive Max:%XM"
#define UI_TEXT_EXTR_PMAX				"PID Max:%XD"
#define UI_TEXT_STRING_HM_BANGBANG		"BangBang"
#define UI_TEXT_STRING_HM_PID			"PID"
#define UI_TEXT_STRING_ACTION			"Aktion:%la"
#define UI_TEXT_HEATING_EXTRUDER		"Heize Extruder"
#define UI_TEXT_HEATING_BED				"Heize Druckbett"
#define UI_TEXT_HEATING_UP				"Aufheizen..."
#define UI_TEXT_HEATING					"Heizen..."
#define UI_TEXT_COOLING_DOWN			"Abk" STR_uuml "hlen..."
#define UI_TEXT_OUTPUTTING_OBJECT		"Ausgeben..."
#define UI_TEXT_PAUSING					"Pausieren..."
#define UI_TEXT_CONTINUING				"Fortfahren..."
#define UI_TEXT_KILLED					"Angehalten"
#define UI_TEXT_STEPPER_DISABLED		"Motoren aus"
#define UI_TEXT_UPLOADING				"Hochladen..."
#define UI_TEXT_PAGE_BUFFER				"Puffer:%oB"
#define UI_TEXT_SHOW_MEASUREMENT		"Zeige Messung"
#define UI_TEXT_RESET_MEASUREMENT		"Reset Messung"
#define	UI_TEXT_TEST_STRAIN_GAUGE		"Teste DMS"
#define UI_TEXT_ZCALIB					"Z Kalibrierung"
#define UI_TEXT_Z_OFFSET				"Z Abstand"
#define UI_TEXT_SET_P1					"Setze P1"
#define UI_TEXT_SET_P2					"Setze P2"
#define UI_TEXT_SET_P3					"Setze P3"
#define UI_TEXT_CALCULATE_LEVELING		"Berechne Level."
#define UI_TEXT_LEVEL					"Level delta"
#define UI_TEXT_EXTR_WAIT_RETRACT_TEMP  "Wait Temp.: %XT\002C"
#define UI_TEXT_EXTR_WAIT_RETRACT_UNITS "Wait Units: %XU mm"
#define UI_TEXT_PRINTER_READY			"Drucker bereit."
#define	UI_TEXT_MILLER_READY			"Fr" STR_auml "ser bereit."
#define UI_TEXT_DO_HEAT_BED_SCAN		"Matrix Scan"
#define UI_TEXT_DO_MHIER_BED_SCAN		"Offset Z-Scan"
#define UI_TEXT_HEAT_BED_SCAN_PLA		"Matrix Scan PLA"
#define UI_TEXT_HEAT_BED_SCAN_ABS		"Matrix Scan ABS"
#define UI_TEXT_DO_WORK_PART_SCAN		"Abtasten"
#define UI_TEXT_HEAT_BED_SCAN			"Heizbett Scan"
#define UI_TEXT_OUTPUT_OBJECT			"Objekt ausgeben"
#define UI_TEXT_PARK_HEAT_BED			"Heizbett parken"
#define UI_TEXT_PAUSE					"Pause"
#define UI_TEXT_HOME					"Home"
#define UI_TEXT_Z_COMPENSATION			"Z Kompensation"
#define UI_TEXT_CHANGE_MODE				"Modus wechseln"
#define UI_TEXT_CHANGE_Z_TYPE			"Z Typ wechseln"
#define UI_TEXT_CHANGE_HOTEND_TYPE		"Hotend wechseln"
#define UI_TEXT_CHANGE_MILLER_TYPE		"Fr" STR_auml "ser wechseln"
#define UI_TEXT_X_AXIS					"X-Achse"
#define UI_TEXT_Y_AXIS					"Y-Achse"
#define UI_TEXT_Z_AXIS					"Z-Achse"
#define UI_TEXT_RESET					"Neustart"
#define UI_TEXT_RESET_ACK				"Neustart jetzt?","","%mYJa","%mNNein"
#define UI_TEXT_HEAT_BED_SCAN_ABORTED	"Scan abgebrochen"
#define UI_TEXT_HEAT_BED_SCAN_DONE		"Scan beendet"
#define UI_TEXT_HEAT_BED_SCAN_OFFSET_MIN "mOffset: %HO um"
#define UI_TEXT_PRINT_MODE				"Drucker"
#define UI_TEXT_MILL_MODE				"Fr" STR_auml "ser"
#define UI_TEXT_Z_SINGLE				"Einfach"
#define UI_TEXT_Z_CIRCUIT				"Kreis"
#define UI_TEXT_Z_MODE_MIN				"Z Min"
#define UI_TEXT_Z_MODE_SURFACE			"Oberf."
#define UI_TEXT_HOTEND_V1				"V1"
#define UI_TEXT_HOTEND_V2				"V2"
#define UI_TEXT_Z_COMPENSATION_ACTIVE	"Kmp"
#define UI_TEXT_FIND_Z_ORIGIN_DONE		"Suche beendet"
#define UI_TEXT_TEST_STRAIN_GAUGE_DONE		"Test beendet"
#define UI_TEXT_WORK_PART_SCAN			"Werkst" STR_uuml "ck Scan"
#define UI_TEXT_WORK_PART_SCAN_ABORTED	"Scan abgebrochen"
#define UI_TEXT_WORK_PART_SCAN_DONE		"Scan beendet"
#define UI_TEXT_SET_SCAN_XY_START		"Setze XY Start"
#define UI_TEXT_SET_SCAN_XY_END			"Setze XY Ende"
#define UI_TEXT_SET_SCAN_DELTA_X		"Setze dX: %Dxmm"
#define UI_TEXT_SET_SCAN_DELTA_Y		"Setze dY: %Dymm"
#define UI_TEXT_STRAIN_GAUGE			"F: %s1 L: %Fs%%%"
#define UI_TEXT_STRAIN_GAUGE_SPEED		"F: %s1 V: %om%%%"
#define UI_TEXT_SERVICE					"WARTUNG"
#define UI_TEXT_SERVICE_TIME			"N" STR_auml "chster Service"
#define	UI_TEXT_PRINTING_ROOM_LIGHT		"PRL: "
#define UI_TEXT_WHITE					"Weiss"
#define	UI_TEXT_COLOR					"Auto"
#define UI_TEXT_MANUAL					"Manuell"
#define UI_TEXT_WRONG_FIRMWARE			"Falsche Firmw."
#define UI_TEXT_MOVE_MODE_SINGLE_STEPS	"Einzelschritte"
#define UI_TEXT_MOVE_MODE_SINGLE_MOVE	"Einzelfahrt"
#define UI_TEXT_MOVE_MODE_1_MM			"1 mm"
#define UI_TEXT_MOVE_MODE_10_MM			"10 mm"
#define UI_TEXT_MOVE_MODE_50_MM			"50 mm"

#define UI_TEXT_ERROR					"Fehler:"
#define UI_TEXT_WARNING					"Warnung:"
#define UI_TEXT_INFORMATION				"Information:"
#define UI_TEXT_SET_ORIGIN				"Setze Ursprung"
#define UI_TEXT_AUTODETECT_PID			"PID ermitteln"	
// Temp. = Temperatur
#define UI_TEXT_HOME_UNKNOWN			"Home unbekannt"
#define UI_TEXT_SAVING_FAILED			"Speichern n.m."		
// n.m. = nicht moeglich
#define UI_TEXT_OPERATION_DENIED		"Operation verw."		
// verw. = verweigert
#define	UI_TEXT_EMERGENCY_PAUSE			"Notfall Pause"
#define UI_TEXT_EMERGENCY_STOP			"Notfall Block"

#define	UI_TEXT_MIN_REACHED				"Min erreicht"
#define	UI_TEXT_MAX_REACHED				"Max erreicht"		
// Temp. = Temperatur
#define	UI_TEXT_TIMEOUT					"Timeout"
#define UI_TEXT_AUTODETECT_PID_DONE		"PID ermittelt"
#define UI_TEXT_SENSOR_ERROR			"Sensorfehler"
#define UI_TEXT_HEAT_BED_ZOFFSET_SEARCH_ABORTED "Scan abgebrochen"

// Printtime output gets aggregated like <Days_5gisgits>UI_TEXT_PRINTTIME_DAYS<Hours>UI_TEXT_PRINTTIME_HOURS<Minutes>UI_TEXT_PRINTTIME_MINUTES
// ___88 days 12:45
#define UI_TEXT_PRINTTIME_DAYS			" Tage "
#define UI_TEXT_PRINTTIME_HOURS			":"
#define UI_TEXT_PRINTTIME_MINUTES		""
#define UI_TEXT_POWER					"ATX Netzteil an/aus"
#define UI_TEXT_STRING_HM_DEADTIME		"Totzeit"
#define UI_TEXT_STRING_HM_SLOWBANG		"Langs.BB"

#if MOTHERBOARD == DEVICE_TYPE_RF2000

	#define	UI_TEXT_EXTRUDER_OFFSET_X		"Extruder Abstand X"
	#define	UI_TEXT_EXTRUDER_OFFSET_Y		"Extruder Abstand Y"
	#define UI_TEXT_SET_Z_ORIGIN			"Setze Z Ursprung"
	#define UI_TEXT_FIND_Z_ORIGIN			"Suche Z Ursprung"
	#define UI_TEXT_BED_TEMP				"Temp. Bett:%Eb\002C"
	#define UI_TEXT_EXTR0_TEMP				"Temp. 0   :%E0\002C"
	#define UI_TEXT_EXTR1_TEMP				"Temp. 1   :%E1\002C"
	#define UI_TEXT_EXTR2_TEMP				"Temp. 2   :%E2\002C"
	#define	UI_TEXT_RGB_LIGHT_MODE			"RGB Licht: %li"
	#define UI_TEXT_230V_OUTPUT				"230V Steckdose: %ou"
	#define UI_TEXT_FET1_OUTPUT				"X42 Mosfet1: %ol"
	#define UI_TEXT_FET2_OUTPUT				"X44 Mosfet2: %on"
	#define UI_TEXT_BAUDRATE				"Baudrate: %oc"
	#define UI_TEXT_FEED_MAX_X				"Max X :%fx"
	#define UI_TEXT_FEED_MAX_Y				"Max Y :%fy"
	#define UI_TEXT_FEED_MAX_Z				"Max Z :%fz"
	#define UI_TEXT_JERK					"X/Y-Ruck :%aj"
	#define UI_TEXT_ZJERK					"Z-Ruck   :%aJ"
	#define UI_TEXT_ACTION_FANSPEED			"L" STR_uuml "fter:    %Fs%%%"
	#define UI_TEXT_FAN_OFF					"L" STR_uuml "fter aus"
	#define UI_TEXT_FAN_25					"L" STR_uuml "fter auf  25%%%"
	#define UI_TEXT_FAN_50					"L" STR_uuml "fter auf  50%%%"
	#define UI_TEXT_FAN_75					"L" STR_uuml "fter auf  75%%%"
	#define UI_TEXT_FAN_FULL				"L" STR_uuml "fter auf 100%%%"
	#define UI_TEXT_PAGE_EXTRUDER			"E: %ec/%Ec\002C->%oC"
	#define UI_TEXT_PAGE_EXTRUDER1			"E0:%e0/%E0\002C->%o0"
	#define UI_TEXT_PAGE_EXTRUDER2			"E1:%e1/%E1\002C->%o1"
	#define UI_TEXT_PAGE_BED				"B: %eb/%Eb\002C->%ob"
	#define UI_TEXT_QUICK_SETTINGS			"Schnelleinstellung"
	#define UI_TEXT_SET_XY_ORIGIN			"Setze XY Ursprung"
	#define UI_TEXT_DISABLE_STEPPER			"Motoren ausschalten"
	#define UI_TEXT_UNMOUNT_FILAMENT		"Filament entladen"
	#define UI_TEXT_E_POSITION				"Position Extruder"
	#define	UI_TEXT_ACTIVE_EXTRUDER			"Aktiver Extruder:%Oa"
	#define UI_TEXT_SET_E_ORIGIN			"Setze E Ursprung"
	#define UI_TEXT_CONTINUE_PRINT			"Druck fortsetzen"
	#define UI_TEXT_PAUSE_MILL				"Fr" STR_auml "sen pausieren"
	#define UI_TEXT_CONTINUE_MILL			"Fr" STR_auml "sen fortsetzen"
	#define UI_TEXT_EXTR_STEPS				"Schritte/mm:%Se"
	#define UI_TEXT_EXTR_ACCEL				"Beschleunigung:%XA"
	#define UI_TEXT_EXTR_WATCH				"Stab. Zeit:%Xw"
	#define UI_TEXT_SPEED_MULTIPLY			"Geschw. Mul.:%om%%%"
	#define UI_TEXT_FLOW_MULTIPLY			"Fluss Mul.  :%of%%%"
	#define UI_TEXT_SD_REMOVED				"SD Karte entfernt"
	#define UI_TEXT_SD_INSERTED				"SD Karte erkannt"
	#define UI_TEXT_ALIGN_EXTRUDERS			"Extruder ausrichten"
	#define UI_TEXT_MILLER_ONE_TRACK		"eine Spur"
	#define UI_TEXT_MILLER_TWO_TRACKS		"zwei Spuren"
	#define UI_TEXT_Z_MODE					"Z Anzeige: %zm"
	#define UI_TEXT_Z_MODE_Z_ORIGIN			"Z Urspr."
	#define	UI_TEXT_FIND_Z_ORIGIN_ABORTED	"Suche abgebrochen"
	#define	UI_TEXT_TEST_STRAIN_GAUGE_ABORTED	"Test abgebrochen"
	#define	UI_TEXT_RESTORE_DEFAULTS		"Werkseinstellungen"
	#define UI_TEXT_PRINT_TIME				"Gesamte Druckzeit"
	#define UI_TEXT_MILL_TIME				"Gesamte Fr" STR_auml "szeit"
	#define UI_TEXT_STOP_MILL				"Fr" STR_auml "sen abbrechen"
	#define UI_TEXT_SET_Z_MATRIX_WORK_PART	"Setze Z Matrix:%WP"
	#define UI_TEXT_SET_Z_MATRIX_HEAT_BED	"Setze Z Matrix:%HB"
	#define UI_TEXT_PRINT_FILAMENT			"Filament gedruckt"
	#define UI_TEXT_STOP_PRINT				"Druck abbrechen"
	#define UI_TEXT_STOP_PRINT_ACK			"Drucken abbrechen?","","%mYJa","%mNNein"
	#define UI_TEXT_STOP_MILL_ACK			"Fr" STR_auml "sen abbrechen?","","%mYJa","%mNNein"
	#define UI_TEXT_TEMPERATURE_MANAGER		"Temperatur Manager"
	#define UI_TEXT_INVALID_MATRIX			"Ung" STR_uuml" ltige Matrix"
	#define	UI_TEXT_TEMPERATURE_WRONG		"Temperatur falsch"
	
#else
	
	#define	UI_TEXT_EXTRUDER_OFFSET_X		"Extr. Abstand X"
	#define	UI_TEXT_EXTRUDER_OFFSET_Y		"Extr. Abstand Y"	
	#define UI_TEXT_SET_Z_ORIGIN			"Setze Z Urspr."
	#define UI_TEXT_FIND_Z_ORIGIN			"Suche Z Urspr."
	#define UI_TEXT_BED_TEMP				"Temp.Bett:%Eb\002C"
	#define UI_TEXT_EXTR0_TEMP				"Temp. 0  :%E0\002C"
	#define UI_TEXT_EXTR1_TEMP				"Temp. 1  :%E1\002C"
	#define UI_TEXT_EXTR2_TEMP				"Temp. 2  :%E2\002C"
	#define	UI_TEXT_RGB_LIGHT_MODE			"RGB Licht: %li"
	#define UI_TEXT_BAUDRATE				"Baudrate: %oc"
	#define UI_TEXT_FEED_MAX_X				"Max X:%fx"
	#define UI_TEXT_FEED_MAX_Y				"Max Y:%fy"
	#define UI_TEXT_FEED_MAX_Z				"Max Z:%fz"
	#define UI_TEXT_JERK					"X/Y-Ruck:%aj"
	#define UI_TEXT_ZJERK					"Z-Ruck  :%aJ"
	#define UI_TEXT_ACTION_FANSPEED			"L" STR_uuml "fter:%Fs%%%"
	#define UI_TEXT_FAN_OFF					"L" STR_uuml "fter aus"
	#define UI_TEXT_FAN_25					"L" STR_uuml "fter auf 25%%%"
	#define UI_TEXT_FAN_50					"L" STR_uuml "fter auf 50%%%"
	#define UI_TEXT_FAN_75					"L" STR_uuml "fter auf 75%%%"
	#define UI_TEXT_FAN_FULL				"L" STR_uuml "fter auf 100%%%"
	#define UI_TEXT_PAGE_EXTRUDER			"E:%ec/%Ec\002C\176%oC"
	#define UI_TEXT_PAGE_EXTRUDER1			"E0:%e0/%E0\002C\176%o0"
	#define UI_TEXT_PAGE_EXTRUDER2			"E1:%e1/%E1\002C\176%o1"
	#define UI_TEXT_PAGE_BED				"B: %eb/%Eb\002C\176%ob"
	#define UI_TEXT_QUICK_SETTINGS			"Schnelleinst."
	#define UI_TEXT_SET_XY_ORIGIN			"Setze XY Urspr."
	#define UI_TEXT_DISABLE_STEPPER			"Motoren aussch."
	#define UI_TEXT_UNMOUNT_FILAMENT		"Filament entl."
	#define UI_TEXT_E_POSITION				"Position Extr."
	#define	UI_TEXT_ACTIVE_EXTRUDER			"Aktiver Extr. %Oa"
	#define UI_TEXT_SET_E_ORIGIN			"Setze E Urspr."
	#define UI_TEXT_CONTINUE_PRINT			"Druck forts."
	#define UI_TEXT_PAUSE_MILL				"Fr" STR_auml "sen paus."
	#define UI_TEXT_CONTINUE_MILL			"Fr" STR_auml "sen forts."
	#define UI_TEXT_EXTR_STEPS				"Schr/mm:%Se"
	#define UI_TEXT_EXTR_ACCEL				"Beschl.:%XA"
	#define UI_TEXT_EXTR_WATCH				"Stab.Zeit:%Xw"
	#define UI_TEXT_SPEED_MULTIPLY			"Geschw.Mul:%om%%%"
	#define UI_TEXT_FLOW_MULTIPLY			"Fluss Mul.:%of%%%"
	#define UI_TEXT_SD_REMOVED				"SD Karte entf."
	#define UI_TEXT_SD_INSERTED				"SD Karte eing."
	#define UI_TEXT_ALIGN_EXTRUDERS			"Extruder ausr."
	#define UI_TEXT_MILLER_ONE_TRACK		"eine S."
	#define UI_TEXT_MILLER_TWO_TRACKS		"zwei S."
	#define UI_TEXT_Z_MODE					"Z Anz.: %zm"
	#define UI_TEXT_Z_MODE_Z_ORIGIN			"Z Urs."
	#define	UI_TEXT_FIND_Z_ORIGIN_ABORTED	"Suche abgebr."
	#define	UI_TEXT_TEST_STRAIN_GAUGE_ABORTED	"Test abgebr."
	#define	UI_TEXT_RESTORE_DEFAULTS		"Werkseinstell."
	#define UI_TEXT_PRINT_TIME				"Ges. Druckzeit"
	#define UI_TEXT_MILL_TIME				"Ges. Fr" STR_auml "szeit"
	#define UI_TEXT_STOP_MILL				"Fr" STR_auml "sen abbr."
	#define UI_TEXT_SET_Z_MATRIX_WORK_PART	"Setze Z Matr.:%WP"
	#define UI_TEXT_SET_Z_MATRIX_HEAT_BED	"Setze Z Matr.:%HB"
	#define UI_TEXT_PRINT_FILAMENT			"Filament gedr."
	#define UI_TEXT_STOP_PRINT				"Druck abbr."
	#define UI_TEXT_STOP_PRINT_ACK			"Drucken abbr.?","","%mYJa","%mNNein"
	#define UI_TEXT_STOP_MILL_ACK			"Fr" STR_auml "sen abbr.?","","%mYJa","%mNNein"
	#define UI_TEXT_TEMPERATURE_MANAGER		"Temp. Manager"		
	#define UI_TEXT_INVALID_MATRIX			"Ung. Matrix"	
	#define	UI_TEXT_TEMPERATURE_WRONG		"Temp. falsch"	

#endif // MOTHERBOARD == DEVICE_TYPE_RF2000


#endif // UI_LANGUAGE==1
#endif // UI_LANG_H
