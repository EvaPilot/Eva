//
//  macro.h
//  EvaAutoPilot
//
//  Copyright (c) 2013  www.hexairbot.com. All rights reserved.
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License V2
//  as published by the Free Software Foundation.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.

#ifndef macro_h
#define macro_h

#define VERSION         30601 		//20130601
#define rad             1.745329251994e-2     // pi/180
#define g0              9.81				   // g
#define BAUD_YS         115200
#define deg             57.29577951308232      // 180/pi
#define VALID_GPSNUM    5
#define DELAY_MAIN_FREQ	1000
#define PI				3.1415926535897932
#define OUTLENGTH       99		//输出每包字节数
#define DISP_LINE_LEN	16 


//Parameters
#define YAW_P 					0
#define POWER_NO 				1
#define YAW_D 					2
#define ROLL_P 					3
#define STATE_CHANNEL_RANGE_I 	4
#define MAX_LIFT_VELOCITY 		5
#define AUTO_LANDING_VELOCITY 	6
#define YAW_I_RANGE 			7
#define THROTTLE_P 				8
#define ERASE_ROMBOOT 			9
#define CALIB_ACCEL 			10 //unused
#define TEST_CONTROL_FREQ 		11
#define YAW_I 					12
#define GIMBAL_ROLL_SENSITIVITY 13
#define FLIGHT_WARNING_TIME 	14
#define TARGET_STATE			15
#define VEL_STATE_RANGE_I		17
#define PERPENDICULAR_STAB_COEF 18
#define GIMBAL_PITCH_SENSIT		19
#define PITCH_D					20
#define STATE_SENSITIVITY		21
#define PITCH_P					22
#define MAGNETIC_VARIATION		23
#define NEW_THROTT_I			24
#define POINT_RADIUS			25
#define STATE_CHANNEL_RANGE_P	26 //unused
#define RESERVED				27 //unused
#define MAX_VELOCITY			28
#define VIB_COMPENSATION		29
#define DRONE_TYPE				30
#define YAW_RC_COEFF			33
#define ROLL_D					34
#define PERP_THROT_COEFF		35 //unused
#define RC_TARGET_STATE_COEFF	36
#define STATE_CHANNEL_RANGE_D	37 //unused
#define CONTROL_METHOD			38
//SBUS, Battery, Gimbal selection
#define	SELECTION_OPTIONS		39

//GPS data
#define CUR_LATITUDE 			4+18
#define CUR_LONGITUDE 			8+18
#define TAR_LONGITUDE 			12+18
#define TAR_LATITUDE 			16+18
#define YAW_ANGLE 				20+18
#define GPS_STRENGTH 			24+18
#define YEAR 					25+18
#define MONTH 					26+18
#define DAY 					27+18
#define HOUR 					28+18
#define MINUTE 					29+18
#define SECOND 					30+18
#define TOTAL_UPLOADED_WAYPTS 	31+18

#define RC_RUDDER_STAT 			32+18
#define RC_AILERON_STAT 		33+18
#define RC_ELEVATOR_STAT 		34+18
#define RC_THROTTLE_STAT 		35+18

//Real Status
#define REAL_RUDDER_STAT 		36+18
#define REAL_AILERON_STAT 		37+18
#define REAL_ELEVATOR_STAT 		38+18
#define REAL_THROTTLE_STAT 		39+18

//Other stuff
#define VELOCITY_NORTH_AFT_FIL 	40+18
#define VELOCITY_NORTH_AFT_FIL2	41+18
#define START_TIME_1			42+18
#define START_TIME_2			43+18
//#define RESERVED 				44
#define HOVER_THROT_POS 		45+18
#define DIST_FROM_TAKE_OFF_HB	46+18
#define GIMBAL_RADIUS			47+18
#define CUR_PRESSURE_1			48+18
#define CUR_PRESSURE_2			49+18
#define GPS_VELOCITY_EAST		50+18
#define DIST_FROM_TAKE_OFF_LB	52+18
#define RECEIVER_STATUS			53+18
#define ROCKING_COEFF			54+18
#define PDOP_VALUE				55+18
#define VIBRATION_COEFF			56+18
#define TEMPERATURE				57+18
#define ACCELERATION_RIGHT_1	58+18
#define ACCELERATION_RIGHT_2	59+18
#define ACCELERATION_BACKWARD_1	60+18
#define ACCELERATION_BACKWARD_2	61+18
#define PITCH_ANGLE				62+18
#define ROLL_ANGLE				66+18
#define FLIGHT_CONTROL_VOL_1	70+18
#define FLIGHT_CONTROL_VOL_2	71+18
#define ACCELERATION_DOWN_1		72+18
#define ACCELERATION_DOWN_2		73+18
#define CURRENT_TASK			74+18
#define FLIGHT_CONTROL_STATUS   75+18
#define POWER_CONSUMPTION_1		76+18
#define POWER_CONSUMPTION_2		77+18
#define WARNING_FLAG			78+18
#define VELOCITY_DOWN_AFT_FIL_H 79+18
#define RUDDER_MIDPOINT			80+18
#define AILERON_MIDPOINT		81+18
#define ELEVATOR_MIDPOINT		82+18
#define VELOCITY_EAST_AFT_FIL_H 83+18
#define TARGET_HEIGHT_1			84+18
#define TARGET_HEIGHT_2			85+18
#define VELOCITY_DOWN_AFT_FIL_L 89+18
#define FLIGHT_TIME_1			90+18
#define FLIGHT_TIME_2			91+18
#define PRESENT_CURRENT			92+18
#define VELOCITY_EAST_AFT_FIL_L 93+18
#define GPS_NORTHWARD_VELOCITY  94+18
#define CURRENT_PROD_NUMBER_1	96+18
#define CURRENT_PROD_NUMBER_2	97+18
#define CHECK_FAIL				98+18


//OUT DATA

#define OUT_NIAN              SndBuf[25+18]
#define OUT_YUE               SndBuf[26+18]
#define OUT_RI                SndBuf[27+18]
#define OUT_SHI               SndBuf[28+18]
#define OUT_FEN               SndBuf[29+18]
#define OUT_MIAO              SndBuf[30+18]
#define OUT_INA_LATTI         SndBuf[4+18]
#define OUT_INA_LONGI         SndBuf[8+18]
#define OUT_INA_GPSHIGH       SndBuf[12+18]
#define OUT_INA_VELS          SndBuf[16+18]
#define OUT_INA_HANGXIANG     SndBuf[20+18]
#define OUT_INA_SATELNUM      SndBuf[24+18]
#define OUT_HAND_AUTOHANDLE   SndBuf[31+18]
#define OUT_HAND_FXDUO        SndBuf[32+18]
#define OUT_HAND_FYDUO        SndBuf[33+18]
#define OUT_HAND_SJDUO        SndBuf[34+18]
#define OUT_HAND_YMDUO        SndBuf[35+18]
#define OUT_ATL_FXDUO         SndBuf[36+18]
#define OUT_ATL_FYDUO         SndBuf[37+18]
#define OUT_ATL_SJDUO         SndBuf[38+18]
#define OUT_ATL_YMDUO         SndBuf[39+18]

#define OUT_HIGH_TEMP         SndBuf[40+18]
#define OUT_HIGH_P            SndBuf[42+18]
#define OUT_HIGH_GROUNDP      SndBuf[44+18]
#define OUT_HIGH_AIR          SndBuf[48+18]

#define OUT_ADX_REF25         SndBuf[50+18]

#define	KONGSU_L		      SndBuf[50+18]

#define OUT_ADX_FUYANGPING    SndBuf[54+18]
#define OUT_ADX_HENGGUNPING   SndBuf[58+18]
#define OUT_ADX_FUYANG        SndBuf[62+18]
#define OUT_ADX_HENGGUN       SndBuf[66+18]
#define OUT_ADX_HANGXIANG     SndBuf[70+18]

#define OUT_ARM_TARGET        SndBuf[74+18]

#define OUT_ARM_AUTOHANDLE    SndBuf[75+18]
#define OUT_ARM_FXDUO         SndBuf[76+18]
#define OUT_ARM_FYDUO         SndBuf[77+18]
#define OUT_ARM_SJDUO         SndBuf[78+18]
#define OUT_ARM_YMDUO         SndBuf[79+18]

#define OUT_ARM_FXZHONG SndBuf[80+18]
#define OUT_ARM_FYZHONG SndBuf[81+18]
#define OUT_ARM_SJZHONG SndBuf[82+18]

#define OUT_ARM_YMZHONG SndBuf[83+18]
#define OUT_ARM_REFHIGH SndBuf[84+18]
#define OUT_ARM_CUR_TAR_ANGLE SndBuf[86+18]
#define OUT_ARM_CUR_SHOULD_ANGLE SndBuf[90+18]
#define OUT_ARM_DELTA_ANGLE SndBuf[94+18]

#define OUT_VOLTAGE SndBuf[73+18]
#define OUT_VOLTAGE2 SndBuf[86+18]
#define OUT_VOLTAGE3 SndBuf[87+18]



//SIMPLE COMMAND

#define CMD_SIMPLE_AIRLINE_DOWNLOAD       82
#define CMD_SIMPLE_POINT_FLIGHT_CANCEL    85
#define CMD_SIMPLE_CIRCLE_FLIGHT_CANCEL   85
#define CMD_SIMPLE_CONFIG_READ            104
#define CMD_SIMPLE_SETUP_ENTER            120
#define CMD_SIMPLE_SETUP_QUIT             121
#define CMD_SIMPLE_TAKE_OFF               88
#define CMD_SIMPLE_BACK_AND_LANDING       87
#define CMD_SIMPLE_PHYSICAL_RC_ENABLE     89
#define CMD_SIMPLE_PHYSICAL_RC_DISABLE    90
#define CMD_SIMPLE_CHANNEL_ADJUST         117
#define CMD_SIMPLE_AIRLINE_ENABLE         122
#define CMD_SIMPLE_AIRLINE_DISABLE        123
#define CMD_SIMPLE_MOTRO_UNLOCK           137
#define CMD_SIMPLE_CAREFREE_ENABLE        149
#define CMD_SIMPLE_CAREFREEE_DISABLE      150
#define CMD_SIMPLE_MAG_DATA_DOWNLOAD      200
#define CMD_SIMPLE_HOME_SET               86
#define CMD_SIMPLE_GYRO_CLEAR             100



//FPGA

#define pmem_cnt5msh (char *)0x3000307c
#define pmem_cnt5msl (char *)0x3000307d

#define pmem_bus1  (char *)0x30003000
#define pmem_bus2  (char *)0x30003001
#define pmem_bus3  (char *)0x30003002
#define pmem_bus4  (char *)0x30003003
#define pmem_bus5  (char *)0x30003004
#define pmem_bus6  (char *)0x30003005
#define pmem_bus7  (char *)0x30003006
#define pmem_bus8  (char *)0x30003007
#define pmem_bus9  (char *)0x30003008
#define pmem_bus10 (char *)0x30003009

#define pmem_bus11 (char *)0x3000300a
#define pmem_bus12 (char *)0x3000300b
#define pmem_bus13 (char *)0x3000300c
#define pmem_bus14 (char *)0x3000300d
#define pmem_bus15 (char *)0x3000300e
#define pmem_bus16 (char *)0x3000300f
#define pmem_bus17 (char *)0x3000301a
#define pmem_bus18 (char *)0x3000301b
#define pmem_bus19 (char *)0x3000301c
#define pmem_bus20 (char *)0x3000301d

#define pmem_bus21 (char *)0x3000301e
#define pmem_bus22 (char *)0x3000301f
//#define pmem_bus23 (char *)0x3000303a
//#define pmem_bus24 (char *)0x3000303b
//#define pmem_bus25 (char *)0x3000303c
//#define pmem_bus26 (char *)0x3000303d
//#define pmem_bus27 (char *)0x3000303e
#define pmem_bus28 (char *)0x3000303f
#define pmem_bus29 (char *)0x3000305a
#define pmem_bus30 (char *)0x3000305b

#define pmem_bus31 (char *)0x3000305c
#define pmem_bus32 (char *)0x3000305d
#define pmem_bus33 (char *)0x30003010

#define pmem_maggo    (char *)0x30003076
#define pmem_magdata  (char *)0x30003075
#define pmem_magdata2 (char *)0x30003077
#define pmem_magdata3 (char *)0x30003078
#define pmem_magdata4 (char *)0x30003079
#define pmem_magdata5 (char *)0x3000307a
#define pmem_magdata6 (char *)0x3000307b

#define pmem_fpgadata13 (char *)0x30003020
#define pmem_fpgadata12 (char *)0x30003021
#define pmem_fpgadata11 (char *)0x30003022
#define pmem_fpgadata23 (char *)0x30003023
#define pmem_fpgadata22 (char *)0x30003024
#define pmem_fpgadata21 (char *)0x30003025
#define pmem_fpgadata33 (char *)0x30003026
#define pmem_fpgadata32 (char *)0x30003027
#define pmem_fpgadata31 (char *)0x30003028
#define pmem_fpgadata43 (char *)0x30003029
#define pmem_fpgadata42 (char *)0x3000302a
#define pmem_fpgadata41 (char *)0x3000302b
#define pmem_fpgadata53 (char *)0x3000302c
#define pmem_fpgadata52 (char *)0x3000302d
#define pmem_fpgadata51 (char *)0x3000302e
#define pmem_fpgadata63 (char *)0x3000302f
#define pmem_fpgadata62 (char *)0x30003040
#define pmem_fpgadata61 (char *)0x30003041
#define pmem_fpgadata73 (char *)0x30003042
#define pmem_fpgadata72 (char *)0x30003043
#define pmem_fpgadata71 (char *)0x30003044
#define pmem_fpgadata83 (char *)0x30003045
#define pmem_fpgadata82 (char *)0x30003046
#define pmem_fpgadata81 (char *)0x30003047

#define pmem_fpga1h (char *)0x30003051
#define pmem_fpga1l (char *)0x30003011		//Video selection
#define pmem_fpga2h (char *)0x30003052
#define pmem_fpga2l (char *)0x30003012

#define pmem_rudderh   (char *)0x30003053
#define pmem_rudderl   (char *)0x30003013
#define pmem_aileronh  (char *)0x30003054
#define pmem_aileronl  (char *)0x30003014
#define pmem_elevatorh (char *)0x30003055
#define pmem_elevatorl (char *)0x30003015
#define pmem_throttleh (char *)0x30003056
#define pmem_throttlel (char *)0x30003016

#define pmem_kaisanh (char *)0x30003057
#define pmem_kaisanl (char *)0x30003017

#define pmem_fpga8h (char *)0x30003058
#define pmem_fpga8l (char *)0x30003018		//Take picture
#define pmem_fpga9h (char *)0x30003059
#define pmem_fpga9l (char *)0x30003019
#define pmem_fpga9t (char *)0x30003039

#define pmem_fpga10h (char *)0x3000303a
#define pmem_fpga10l (char *)0x3000303b
#define pmem_fpga11h (char *)0x3000303c
#define pmem_fpga11l (char *)0x3000303d
#define pmem_fpga12l (char *)0x3000303e
#define pmem_fpga13l (char *)0x3000303f
#define pmem_fpga14l (char *)0x3000303a
#define pmem_fpga15l (char *)0x3000303c

#define pmem_sony (char *)0x3000303e

#define pmem_LED (char *)0x30003070

#define pmem_rollh    (char *)0x30003071
#define pmem_rolll    (char *)0x30003072
#define pmem_pitchh   (char *)0x30003073
#define pmem_pitchl   (char *)0x30003074
#define pmem_yawgyroh (char *)0x30003036
#define pmem_yawgyrol (char *)0x30003038

#define pmem_WORD1    (char *)0x30003071
#define pmem_WORD1new (char *)0x30003072
#define pmem_wrSPI    (char*)0x30003073
#define shua          (char*)0x30003074
#define pmem_355_rst_ (char*)0x30003033
#define pmem_355_cs   (char*)0x30003032
#define pmem_355_sclk (char*)0x30003034
#define pmem_355_din  (char*)0x30003036
#define pmem_355_dout (char*)0x30003038

#define pmem_mag_cs (char*)0x3000307e

#define pmem_width1h (char *)0x30003048
#define pmem_width1l (char *)0x30003049
#define pmem_width2h (char *)0x3000304a
#define pmem_width2l (char *)0x3000304b
#define pmem_width3h (char *)0x3000304c
#define pmem_width3l (char *)0x3000304d
#define pmem_width4h (char *)0x3000304e
#define pmem_width4l (char *)0x3000304f
#define pmem_width5h (char *)0x30003060
#define pmem_width5l (char *)0x30003061
#define pmem_width6h (char *)0x30003062
#define pmem_width6l (char *)0x30003063
#define pmem_width7h (char *)0x30003064
#define pmem_width7l (char *)0x30003065
#define pmem_width8h (char *)0x30003066
#define pmem_width8l (char *)0x30003067
#define pmem_width9h (char *)0x30003068
#define pmem_width9l (char *)0x30003069

#define pmem_poke (char *)0x3000307f


#define LAMP      *pmem_LED
#define OUT_FXDUO *pmem_rudderl
#define OUT_FYDUO *pmem_aileronl
#define OUT_SJDUO *pmem_elevatorl
#define OUT_YMDUO *pmem_throttlel
#define ADI_CS    *pmem_355_cs
/*
#define OUT_NEW   *pmem_NEW
#define OUT_HW7   *pmem_HW7
#define OUT_SJNEW *pmem_SJNEW
#define OUT_FYNEW *pmem_FYNEW
*/


#endif