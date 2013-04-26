#include <AT91RM9200.h> 
#include <lib_AT91RM9200.h> 
#include <main.h>
#include <dataflash.h> 
#include <math.h>
#include <caldata.h>
#include <stdio.h>  
#include <stdlib.h>
#include <string.h>
#include <example-1.h>

#define rad   1.745329251994e-2     // pi/180
#define ADI_CS *pmem_355_cs
#define g0 9.81						// g

#define BAUD_YS 115200

#define deg   57.29577951308232      // 180/pi

#define VALID_GPSNUM 5

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

//RC Status
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
#define CURRENT_TASK			74+18 //Waypoint #
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

int firmware = 30205;		//130205
int cur_address = 0;

char count_unlock = 0;
char write_data_record = 0;

extern int flash_erase (int s_first, int s_last);
extern int write_byte(unsigned long dest, unsigned char udata);
extern int write_buff (unsigned char * src, unsigned long addr, unsigned long cnt);

extern void	AT91_IIC_init(void);
extern int  AT91F_TWI_Write(const AT91PS_TWI pTwi ,int address, char *data2send, int size);
extern int  AT91F_TWI_Read(const AT91PS_TWI pTwi , int address, char *data, int size);

extern void AT91F_DBGU_Printk(char *);
extern void AT91F_UART0_Printk(char *);
extern void AT91F_UART1_Printk(char *);
extern void AT91F_UART2_Printk(char *);

extern void AT91F_ST_ASM_HANDLER(void);
extern void AT91F_UART0_ASM_HANDLER(void);
extern void AT91F_UART1_ASM_HANDLER(void);
extern void AT91F_UART2_ASM_HANDLER(void);
extern void AT91F_IRQ0_ASM_HANDLER(void);
extern float fabsf(float);
extern size_t strlen(const char * /*s*/);
extern char *strcat(char * /*s1*/, const char * /*s2*/);
extern int memcmp(const void * /*s1*/, const void * /*s2*/, size_t /*n*/);

extern float Butterworth(float,float *);

//Flag for RC
char rc_enabled = 1; //1 = enabled, 0 = disabled

//Object file requires these variables/functions
unsigned char	bAutoHand_=0;
float bias_iraw_acc_1=0,bias_iraw_acc_2=0,bias_iraw_acc_3=0,bias_iraw_gyro_1=0,bias_iraw_gyro_2=0,bias_iraw_gyro_3=0,bias_iraw_gyro_1_zu=0,bias_iraw_gyro_2_zu=0,bias_iraw_gyro_3_zu=0;
char satelnum=0,char_satelnum = 0, satelnumflag = 0, t_satelnumflag = 0;
int Channel_5=1000, Channel_Throttle=1930;
char ekff_error = 0;
int  Manual_Mode = 1;
AhrsData alldata;	//Structure used for state calculation and data transfer. Example of its usage is in example.h
float cal_heading = 0;
char mode_set = 0;
char in_flight = 0;
unsigned char VelFilter=0,MagFilter=0;
int tvely_i,tvelx_i,tvelz_i, gps_yaw, old_gps_yaw = 0;	//GPS data, velocity in x,y,z directions, heading direction
float homelatti=0,homelongi=0;
float	aa = 6378137.0;
float	ff = 1.0/298.257223563;
extern float xEKF[16];
double alt0=1;
char vel_down_clear = 0;
unsigned char	Parameter[100];

//State estimation functions
extern void maincall(void);
extern void InitData(void);
extern void TimerINTCall(void);


void LimitAccel(float *);
void LimitCourseAngle(float);

float cos_hx,sin_hx;
int countplotmag = 0;
char Waypoint_carefree = 0;
char outmagdata = 0;
int count_error = 0;
int temperature_high=50;		//temperature_high
int temperature_low=5; 		//temperature_low
char send_exitset=0,send_enterset=0;

float distanceR;			//distanceR
float distance_maxV;		//distance_maxV

char if1 = 0;
char status7 = 0;
int	count_twi = 0;
char twi_error = 0;

extern float rpy[3];

char ppmmode = 0;
char startled = 0;
int stop_throttle_count = -1;
char start_center_gravity = 0;
char firstgps = 0;
char posnew=0,velnew=0;

float tgyro3;
float kgyro1=1,kgyro2=1,kgyro3=1;
char ESC_select=0;
float old_vel_roll=0,old_vel_pitch=0;
char protect=0,reset_throt=0;
float targetspin = 0;

int rpm = 0;

int High_Throttle = 1420;			//58
int Low_Throttle = 1600;			//40

float CURRENT_ZEROV = 0.6445312;

float home_distance = 0;

char carefree = 0;
float cos_care=0,sin_care=0;

int new_target_high_count = 0;
float sumailgyro=0,sumelevgyro=0;

char Enable_Waypoint = 0;

float MAXacc = 0.2;
char init_Current = 0;
char first_ = 0;
float pingacc1,pingacc2,pingacc3;
char ctreceiver = 0;
int ctlock_throttle = 0;
char Lock_Throttle = 0;
float low_power = 3.65*4, emergency_power = 3.6*4;
char enter_set_height = 1;
int ctgohome = 0;

extern float rN,rM,coslat0;

char first6115 = 0;
float acc_ail,acc_elev,acc_dn1,acc_dn2,bacc_dn2;
float targetaccU,acc_U;
float calailacc=0,calelevacc=0;

float TargetRollMid=0,TargetPitchMid=0;

float sum_anglex=0,sum_angley=0;

char firstclearpres = 0; 
char sbus_select = 0;

float magz_estimate=0, magz_max = 0, magz_min=0;
float magx_estimate=0, magx_max = 0, magx_min=0;
float magy_estimate=0, magy_max = 0, magy_min=0;
float magmax_estimate = 0;
float gyro1_estimate=0, gyro1_max = 0, gyro1_min=0;
float gyro2_estimate=0, gyro2_max = 0, gyro2_min=0;
float gyro_estimate = 0;
int ct_estimate = 0;
int ct_quan = 0;

char tmpBuf[6];

char colors[5];
float raw_magx[1200],raw_magy[1200];
float raw_magyz[1200],raw_magzz[1200];
int ct_rawmag = 0, ct_rawmagz = 0, index_mag = 0;
char plotmag = 0, plotmag_end = 0;
int ctplotmag = 0, cthui = 0;

float dlat,dlon,dalt,lat1,lon1,alt1;
extern float lat,lon,lon0,lat0,alt,h0;

char throtqiehuan = 3; //?
float VEL = 1000;
unsigned short parFlag = 0;

float gimbal_target_R = 20; //Radius of the circle around the gimbal target 
char gimbal_target_lock = 0; //flag

char heading_stop = 0;
char color_point = 0;

extern float fD;
unsigned char enable_ctclearpres = 0, ctclearpres = 0;

float fAirHigh = 0;

unsigned short serialnum = 0;
char serialnumok = 0;
int count_show_serial = 0, show_serial=0;

char limit_vel = 0;

float nCalFxWidth=1500,nCalailWidth=1500,nCalelevWidth=1500,nCalthrotWidth=1930;

float sbus_yinzi = 0.62595;

int ctled = 0;
char led_color = 0;
float led_flash_hz = 1;
char start_led_on = 0;
int ct_led_on = 0;

int count_jz = 0, count_fs = 0;
int point_jz = 0;

char pres_error = 0;

float zcp[2] = {0,0};
float zaccdn[2] = {0,0};
float zcq[2] = {0,0};
float zcr[2] = {0,0};
float zgyro3[2] = {0,0};
float bzcp[2] = {0,0};
float bzcq[2] = {0,0};

char sbus_error=0;
int ctsbus_error=0;

char gohome_end = 0,  pregohome = 0, gohome = 0, autotakeoff = 0;

int ct_wifi = 0;

float nDvalue = 0;

char adjust_throt = 0;
int count_adjust = 0;
float Throttle_Factor=1, Rudder_Factor = 1, Aileron_Factor = 1, Elevator_Factor = 1;

float newdistance(void);  //Calculate the distance between the current position and the waypoint
float distance_point(float,float,float,float);  //??
void put_parameter(void);
void do_writebuf1(void);
void do_writebuf2(void);
void do_writebuf3(void);

void cal_mag(void);

unsigned int timemm;		//Used in the GPS function

float CalAtanVector(void); //Calculate the inverse tangent of vectorLongi/vectorLatti; returns a value between 0~PI

float outhz = 400, outwidth;

char startcontrol = 0;		//Start the output pulse?

//float tfacc1,tfacc2,tfacc3,tfgyro1,tfgyro2,tfgyro3,ttemperature,ttpresnew;

int CLR_TIME = 1250;

float hz250 = 250, hz50 = 50; 
int ct4=0;
char ctqi = 0;
char action = 0;

float t_ref_yaw=0, ref_yaw=0;

char SBUS = 1;
char vel_up_caled = 0;
char showmag=1;

void rd_adi1256(void);
void put_data1256(void);

float AccelMAX = 3, AngleMax = 0.523598*2;

extern float dt;

float rr[2],trr[2];

char Parameter1_low = 0;

float acc[3];
char ctlb = 0;
char g87 = 0, g88 = 0;
extern void abcd2rpy(void);
extern void abcd2cbn(void);

void rdTWIad(void);

unsigned char ctmag = 0, ctmag2 = 0, ctmag3 = 0;
unsigned char magdata = 0, magdata2 = 0, magdata3 = 0, magdata4 = 0, magdata5 = 0, magdata6 = 0;

int wifi_data = 0, zlf_data = 0;
char previous_x5=1;

float xiumagg12 = 1.0;
float xiumaggFORCE = 1.255;
float targetYaw = 0;
int temp_targetYaw = 0;
char gfx = 0, ngfx = 0;
char  Read1min = 0;
int SeriesCount = 0;
char mainmag = 0, mainpres = 0;
float mainYaw = 0;
float navigx=0,navigy=0;
char DroneMode = 6;			//4: X4, 8: X8, 6: X6

char versionX = 0;				//10: + copter. 0: X copter
char clear_acc = 0;
float bias_acc1=0,bias_acc2=0,bias_acc3=0;
float nbias_acc1=0,nbias_acc2=0,nbias_acc3=0;
float kacc1=3,kacc2=3,kacc3=3;
float nkacc1=1,nkacc2=1,nkacc3=1;
float facc1_max=1,facc1_min=-1,facc2_max=1,facc2_min=-1,facc3_max=1,facc3_min=-1;

float WaveHigh = 0, fwavehigh = 0, climbrate = 0, ttWaveHigh = 0, oldWaveHigh = 0;
unsigned char cthigh = 0;
int iWaveHigh = 0, twavehigh = 0;

char GPSready = 0;
char gps_buffer[1000],wifi_buffer[1000];
char onground = 1;

char Enable_Control_Integrate = 0, Enable_Control_Integrate_pre = 0;
float tAirHighf=0,AirHighf=0,aver=0,cur_pres1=0,cur_pres2=0;
double press_val, press_val2;
float showpress_val2;
char in88=0;

float CurrentI = 0, BatteryConsume = 0;
int CurrentII = 0, BatteryConsumeI = 0;

int cthucai = 0;
float tlatti1,tlatti2,tlongi1,tlongi2,tdistance;
float curdistance = 0;

float tDvalue = 0;

char initflag = 0;

char cttui = 0;
double pres=1,pres2=1,hh2=1,hh=1,Pres0=1;

char ctduoji = 0; //??

char throt_low = 1; //??
char endland = 0;
float sumx=0,sumy=0;

char ch5_status = 0;
char ch7_status = 0;

int stoptimecount = 0;
char stopflag = 0;

char test_la=60,la_num=0;
int lyjt_num = 0, ljjt_num = 0;  //??
char oper_ = 0, oper_ail = 0,  oper_elev = 0;
char auto_stay_ding = 0, ctnav = 0, vel_caled = 0, target_caled = 1;	//target_caled即target_caled
char throtauto = 0;
float dqthrot = 1930, throtMid = 1930;
char ctirq = 0;
char msg100[400000];
int ctmsg = 0, ctmsg2 = 0;
char status = 0;
char havesendback_qrp1 = 1, havesendback_qrpall = 1;
char havesendback_dat1 = 1, havesendback_dat2 = 1;
char dou_count = 0; //?? vibration?
char tiqu[20];
int	  d13;	//Level flight angle of incidence
int   d32;	//Flatwise angle of incidene
char t2p1 = 0;
int		ct_sendci = 0;
int temperature = 25, wtemperature = 25;
float vouttemperature = 2.5, voutpres = 2.5;

//need1256
float vout1Mid[401],vout2Mid[401],vout3Mid[401];	//Record every vout under at every temperature in null position, 0~40
int vout1count[401],vout2count[401],vout3count[401];			//Record the number of sampling under every temperature
float vout4Mid[401],vout5Mid[401],vout6Mid[401];	//Record every vout under at every temperature in null position, 0~40
int vout4count[401],vout5count[401],vout6count[401];			//Record the number of sampling under every temperature

int countSGPS = 0;
float nnnnn = 0;

char nospin = 0, close = 0;
char ctwave = 0;
char first = 0;
float theta=0;

float cal1=0,cal2=0,cal3=0,cal4=0;
float cal5=0,cal6=0,cal7=0,cal8=0;

float throttle7 = 1930;

float bcal1=0,bcal2=0,bcal3=0,bcal4=0;
float bcal5=0,bcal6=0,bcal7=0,bcal8=0;

float sumcal1=0,sumcal2=0,sumcal3=0,sumcal4=0,sumcal5=0,sumcal6=0;
int countsum = 0;
char error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0;

int painum = 0, pai_curnum=0, qrpnum = 0;
unsigned char havesendback_painum = 1;

float TargetRoll = 0, tTargetRoll;	//Target roll angle
float TargetRollBias = 0, TargetPitchBias = 0;
float fxbias = 0;
float docal_TargetRoll = 0, docal_TargetPitch = 0, tdocal_TargetRoll = 0, tdocal_TargetPitch = 0;
float tTargetPitch = 0, TargetPitch;	//Target pitch angle

float sumail=0,sumelev=0,sumfx=0;
float sumheading = 0;
float sumailzu[100],sumelevzu[100];

float tgtlattis,tgtlongis,tgthighs;
unsigned char ctthrot = 0;

char send_xuanspin = 0, send_fuyang = 0, newpanxuan = 0;
char endinit = 0;				//Flag for the end of initialization

unsigned char haveSend1min = 1;

int ctmanual = 10;
int	ct_hover_time = 0,start_hover = 0;
float hover_time = 0;
float gps_angle0=0,gps_angle1=0,gps_angle2=0,gps_angle3=0,gps_angle4=0;	//Variables for saving the heading direction of the previous few seconds
int errorflash = 0,in_irq0=0;
float vels = 0;	//GPS ground speed
int ivels = 0;
int Simulate_High = 0, climb_rate_05s, oldAirhigh_05s=0, climb_rate_1s,oldAirhigh_1s=0; //Variables related to climb rate, simulation for flying high etc.
unsigned int ct_fall = 0;

int tmph,tmpl;	//Interim variables used to put two bytes in char addresses
int tmphh,tmpll;
char initc[100]={0xb5,0x62,0x06,0x08,0x06,0x00,0xfa,0x00,0x01,0x00,0x00,0x00,0x0f,0x94};	//start GPS 4HZ
char setubx[]={0xb5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xd0,0x08,0x00,0x00,0x00,0x96,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x8b,0x54};	//set UBX protocol
char disable_msg[100]={0xb5,0x62,0x06,0x01,0x06,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x0f,0x94};	//disable GPS without using msg
char enable_msg[100]={0xb5,0x62,0x06,0x01,0x06,0x00,0x01,0x12,0x00,0x00,0x01,0x00,0x0f,0x94};	//enable GPS using msg

int ctstart = 0, ct98 = 0;

char proc_buffer[2000],dbg_buffer[2000],pgps_buffer[2000],pwifi_buffer[2000];	//Processing buffer for UART in DBG port
unsigned int proc_index = 0, proc_end = 0, gps_index = 0, gps_end = 0, wifi_index = 0, wifi_end = 0;	//Indexing processing variable in UART in DBG port

long int   Channel8_position;

float AirHigh = 0;

double clear_pres_val;
float tmd;
char firstqi = 0;
double presnew;
float tpresnew;
double spipn;
double press_data[5000],press_data2[5000];
int ctair = 0, ctair2 = 0;
double  spitempnew=0;
double off,sens,offnew,sensnew;

float sum_raw_mag_1,sum_raw_mag_2,sum_raw_mag_3;
float sumairhigh=0;

float AirHighf,avernew;
float AirHigh5=0,AirHigh4=0,AirHigh3=0,AirHigh2=0,AirHigh1=0,AirHigh0=0;
float AirHigh_5=0,AirHigh_4=0,AirHigh_3=0,AirHigh_2=0,AirHigh_1=0;
int	ut1new,heightnew,groundpnew=10300;	//Variables related to barometers
double DeltaPressure = 0;

int tchange;

float vout1zu[1501],vout2zu[1501],sumvout1,sumvout2,vout1out,vout2out;

int time_count=0, count_AD = 0, count_SPI = 0, count_dis = 0;
int count_current = 0;
unsigned int run_time=0,fly_time=0;	//Start time and the take off time
char fly_timeup = 0;

float rot_vane_angle = 0, tgtangle = 0, tempangle = 0, outrot_vane_angle = 0, zrot_vane_angle = 0;
float liandong;
int tfx;
char fly_flag = 0;
char sel_video = 0x15, video_sel_count = 0;
unsigned char clearsuflag = 0, clearangle = 0, clear_pressure = 0;
unsigned char pdop_h,pdop_l,year_l,year_h,kaisanFlag=0,xihuoFlag=0,huishou_flag = 0;
float PDOP;
int iPDOP;
float tcurlatti,tcurlongi;
float wanjia_latti=0,wanjia_longi=0;
unsigned int ct_kaisan = 0, ct_kaisan_flag = 0;
char HaveCalculateArrive=1;	//Flag to check whether or not the target point has been reached
char buffer[1000],ct_height=0,cha_3=0,count_cha_3=0;
int Channel_6=1000,Channel_Rudder=1500,Channel_Aileron=1500,Channel_Elevator=1500,fpgain7=1500,fpgain8=1500,fpgain9=1000,tChannel_5=1000,tChannel_Rudder=1500,tChannel_Aileron=1500,tChannel_Elevator=1500,tChannel_Throttle=1930,tfpgain7=1500,tfpgain8=1500;
int old_fpgain7 = 1500, old_fpgain8 = 1500;

int yChannel_6 = 1000;
int zlf_fpgain7 = 1500, huan_zlf_fpgain7 = 1500;

int out1=1500,out2=1500, zout1=1500,zout2=1500,jyout1=1500,jyout2=1500;
int Channel_5old=1000,Channel_6old=1000,Channel_Rudderold=1500,Channel_Aileronold=1500,Channel_Elevatorold=1500,Channel_Throttleold=1930,fpgain7old=1500,fpgain8old=1500,fpgain9old=1000;
unsigned int cnt1=0,cnt2=0,cnt3=0,cnt4=0,cnt5=0,cnt6=0,cnt7=0,cnt8=0,cnt9=0;
char writebuf1 = 0, writebuf2 = 0;
char write_bias = 0;
float temp_mti_spinvel,mti_spinvel,mti_spinvel0,mti_spinvel1,mti_spinvel2,mti_spinvel3,q1,q2,q3,nq0,nq1,nq2,nq3;
float r11,r12,r13,r21,r22,r23,r31,r32,r33,zhi_x,zhi_y,zhi_z,zhi_l,s_x,s_y,s_z;
float gacc_x = 0, gacc_y = 0, gacc_z = 0, gyro_x = 0, gyro_y = 0, gyro_z = 0;

char high100 = 0;
int ctrot_spd = 0;

char error_height = 0, error_kongsu = 0;

unsigned char tuiflag = 0, laflag = 0, pingflag = 0;		
unsigned char allready1 = 0, allready = 0;								
int tuicount = 0, lacount = 0;
unsigned int ctkaisan = 0;
char manualkaisanflag = 0, ctmanualkaisan = 0, ctqiesan = 0;

int sendcount = 0; 
char rst355 = 0, cntrst355 = 0, ctfuwei = 0, ctall0 = 0, all0flag = 0;
char docalct=0, calailct = 0, calelevct = 0;
char inwhile1 = 0;
char eraseromboot = 0;
unsigned char cnt_PlanePitch_sum=0,cnt_PlaneRoll_sum=0,bianjiao_temp=1,bianjiao=1;
unsigned int handcount = 0;
char cthavesendbackall = 0, ctindex = 0;
char cthavesendback_pai1 = 0;
char RunOnGnd_status = 0,ji_RunOnGnd_status = 0, shanhe_status = 0, shanhe_b_status = 0;
int glide_angle = 0, glide_flag = 0;

float	alpha=0,alpha_old=0,alpha_sum=0;		//Parameters for errors in the calculation for rudder
float	alpha_angle0,alpha_angle1,alpha_angle2,alpha_angle3,alpha_angle4;
float 	d_yaw_vel=0;

char	clearflag = 0;
float	PlaneRoll_sum=0,PlanePitch_sum=0,jiD = 0, jiBPD = 0;
int		old_DeltaHigh=0,ctd=0,g2j_D,ctd_true =0,ctd1=0,ctdnum = 6;

int		xuanspin_pianyi = 0,fuyang_pianyi = 0;

int		Cnt_SimHigh=0,countBPD = 0,  sumDeltaHigh;
float	ElevatorCompensate;
float	yaw_vel=0, t_yaw_vel = 0;
char yuntai_tmp[2],yuntai_cmp;
int	yuntai_H_bushu,yuntai_V_bushu;
float yuntai_fuyangangle,yuntai_xuanspinangle;
float bias_xuanspinangle = 0, bias_fuyangangle = 0, tbias_xuanspinangle = 0;
float tangle;

char goback = 0,manual_flag = 1;
char shoushu = 0;
unsigned char ctshoushu = 15,SimCount = 0;
char manual_auto = 0;
float distance0,distance1,distance_xishu;		
int idistance;
int docalheight = 0;


unsigned char real = 1;
float hvel=0,oldDistance;

float iraw_acc_1,iraw_acc_2,iraw_acc_3,iraw_gyro_1,iraw_gyro_2,iraw_gyro_3;
int tmpacc_1,tmpacc_2,tmpacc_3, jiaohuan,tt1,tt2,tt3;
int ttmpacc_1,ttmpacc_2,ttmpacc_3;
char magfx = 0;

int iraw_xmag=0,iraw_throtag=0,iraw_zmag=0;
float sum_pj1=0,sum_pj2=0,sum_pj3=0, pj1_tmp,pj2_tmp,pj3_tmp;
int zhi_ptr = 0;
float pj1 = 0, pj2 = 0, pj3 = 0;
float bpj1 = 0, bpj2 = 0, bpj3 = 0;
float pj1_zu[5],pj2_zu[5],pj3_zu[5],pj4_zu[5];

float xmag=0,throtag=0,zmag=0,xmag_max=0,throtag_max=0,zmag_max=0,xmag_min=0,throtag_min=0,zmag_min=0,xmag_bias=0,throtag_bias=0,zmag_bias=0;
float mag_3_zu[100],mag_2_zu[100],mag_1_zu[100];
float fxmag=0,ailmag=0,fzmag=0,fxmag2=0,ailmag2=0,fzmag2=0;
unsigned int ctfashe;
float ciheading = 0;
float theta_m,gama_m;
float Gxmag=0,Gthrotag=0,Gzmag=0;
float Kmag=1,Kmag2=1;


int ACC_NUM = 1, MAG_NUM = 2;

float ACC_YUZHI = 0.2;

float fraw_acc_1,fraw_acc_2,fraw_acc_3,fraw_gyro_1,fraw_gyro_2,fraw_gyro_3;


float voutp_zu[200],sum_voutp=0;
float tbias_iraw_gyro_1,tbias_iraw_gyro_2,tbias_iraw_gyro_3=0;
float sum_raw_acc_1=0,sum_raw_acc_2=0,sum_raw_acc_3=0,sum_gyro_3=0;
float sum_raw_gyro_1=0,sum_raw_gyro_2=0,sum_raw_gyro_3=0;
float acc_1,acc_2,acc_3,gyro_1,gyro_2,gyro_3,bgyro_1,bgyro_2,bgyro_3;
float tg5=0,tg4=0,tg3=0,tg2=0,tg1=0;
float ntg5=0,ntg4=0,ntg3=0,ntg2=0,ntg1=0;
float ztg5=0,ztg4=0,ztg3=0,ztg2=0,ztg1=0;

char chuan1,chuan2,chuan3;
char AirHighCount = 0, AirHighCount2 = 0;

unsigned short count_irq0 =0,old_count_irq0=0;
char kongsu_protect = 1, ControlError = 0;
float all_dis_m = 0, cur_dis = 0, dis_flied = 0, dis_flying = 0, start_dis = 0;


char vv_JF;		//Controls Aileron and Elevator
char vv_FS;		//Controls Rudder and Elevator
char	CK_A,CK_B,real_CK_A,real_CK_B,nCK_A,nCK_B;

int ibus;
float fbus2=1024,fbus1=1024,fbus3=1024,fbus4=1024,fbus5=1024,fbus6=1024,fbus7=1024,fbus8=1024,fbus9=1024;
char cbus1,cbus2,cbus3,cbus4,cbus5,cbus6,cbus7,cbus8,cbus9,cbus10,cbus11,cbus12,cbus13,cbus14,cbus15,cbus16,cbus17,cbus18,cbus19,cbus20,cbus21;

char * pmem_cnt5msh = (char *)0x3000307c;
char * pmem_cnt5msl = (char *)0x3000307d;

char * pmem_bus1 = (char *)0x30003000;
char * pmem_bus2 = (char *)0x30003001;
char * pmem_bus3 = (char *)0x30003002;
char * pmem_bus4 = (char *)0x30003003;
char * pmem_bus5 = (char *)0x30003004;
char * pmem_bus6 = (char *)0x30003005;
char * pmem_bus7 = (char *)0x30003006;
char * pmem_bus8 = (char *)0x30003007;
char * pmem_bus9 = (char *)0x30003008;
char * pmem_bus10 = (char *)0x30003009;

char * pmem_bus11 = (char *)0x3000300a;
char * pmem_bus12 = (char *)0x3000300b;
char * pmem_bus13 = (char *)0x3000300c;
char * pmem_bus14 = (char *)0x3000300d;
char * pmem_bus15 = (char *)0x3000300e;
char * pmem_bus16 = (char *)0x3000300f;
char * pmem_bus17 = (char *)0x3000301a;
char * pmem_bus18 = (char *)0x3000301b;
char * pmem_bus19 = (char *)0x3000301c;
char * pmem_bus20 = (char *)0x3000301d;

char * pmem_bus21 = (char *)0x3000301e;
char * pmem_bus22 = (char *)0x3000301f;
//char * pmem_bus23 = (char *)0x3000303a;
//char * pmem_bus24 = (char *)0x3000303b;
//char * pmem_bus25 = (char *)0x3000303c;
//char * pmem_bus26 = (char *)0x3000303d;
//char * pmem_bus27 = (char *)0x3000303e;
char * pmem_bus28 = (char *)0x3000303f;
char * pmem_bus29 = (char *)0x3000305a;
char * pmem_bus30 = (char *)0x3000305b;

char * pmem_bus31 = (char *)0x3000305c;
char * pmem_bus32 = (char *)0x3000305d;
char * pmem_bus33 = (char *)0x30003010;

char * pmem_maggo = (char *)0x30003076;
char * pmem_magdata = (char *)0x30003075;
char * pmem_magdata2 = (char *)0x30003077;
char * pmem_magdata3 = (char *)0x30003078;
char * pmem_magdata4 = (char *)0x30003079;
char * pmem_magdata5 = (char *)0x3000307a;
char * pmem_magdata6 = (char *)0x3000307b;

char * pmem_fpgadata13 = (char *)0x30003020;
char * pmem_fpgadata12 = (char *)0x30003021;
char * pmem_fpgadata11 = (char *)0x30003022;
char * pmem_fpgadata23 = (char *)0x30003023;
char * pmem_fpgadata22 = (char *)0x30003024;
char * pmem_fpgadata21 = (char *)0x30003025;
char * pmem_fpgadata33 = (char *)0x30003026;
char * pmem_fpgadata32 = (char *)0x30003027;
char * pmem_fpgadata31 = (char *)0x30003028;
char * pmem_fpgadata43 = (char *)0x30003029;
char * pmem_fpgadata42 = (char *)0x3000302a;
char * pmem_fpgadata41 = (char *)0x3000302b;
char * pmem_fpgadata53 = (char *)0x3000302c;
char * pmem_fpgadata52 = (char *)0x3000302d;
char * pmem_fpgadata51 = (char *)0x3000302e;
char * pmem_fpgadata63 = (char *)0x3000302f;
char * pmem_fpgadata62 = (char *)0x30003040;
char * pmem_fpgadata61 = (char *)0x30003041;
char * pmem_fpgadata73 = (char *)0x30003042;
char * pmem_fpgadata72 = (char *)0x30003043;
char * pmem_fpgadata71 = (char *)0x30003044;
char * pmem_fpgadata83 = (char *)0x30003045;
char * pmem_fpgadata82 = (char *)0x30003046;
char * pmem_fpgadata81 = (char *)0x30003047;

char * pmem_fpga1h = (char *)0x30003051;
char * pmem_fpga1l = (char *)0x30003011;		//Video selection
char * pmem_fpga2h = (char *)0x30003052;
char * pmem_fpga2l = (char *)0x30003012;
char * pmem_FANGXIANGh = (char *)0x30003053;
char * pmem_FANGXIANGl = (char *)0x30003013;
char * pmem_FUYIh = (char *)0x30003054;
char * pmem_FUYIl = (char *)0x30003014;
char * pmem_SHENGJIANGh = (char *)0x30003055;
char * pmem_SHENGJIANGl = (char *)0x30003015;
char * pmem_throttleh = (char *)0x30003056;
char * pmem_throttlel = (char *)0x30003016; 
char * pmem_kaisanh = (char *)0x30003057;
char * pmem_kaisanl = (char *)0x30003017;
char * pmem_fpga8h = (char *)0x30003058;
char * pmem_fpga8l = (char *)0x30003018;		//Take picture
char * pmem_fpga9h = (char *)0x30003059;
char * pmem_fpga9l = (char *)0x30003019;
char * pmem_fpga9t = (char *)0x30003039;

char * pmem_fpga10h = (char *)0x3000303a;
char * pmem_fpga10l = (char *)0x3000303b;
char * pmem_fpga11h = (char *)0x3000303c;
char * pmem_fpga11l = (char *)0x3000303d;
char * pmem_fpga12l = (char *)0x3000303e;
char * pmem_fpga13l = (char *)0x3000303f;
char * pmem_fpga14l = (char *)0x3000303a;
char * pmem_fpga15l = (char *)0x3000303c;

char * pmem_sony = (char *)0x3000303e;

char * pmem_LED = (char *)0x30003070;

char * pmem_rollh = (char *)0x30003071;
char * pmem_rolll = (char *)0x30003072;
char * pmem_pitchh = (char *)0x30003073;
char * pmem_pitchl = (char *)0x30003074;

char * pmem_yawgyroh = (char *)0x30003036;
char * pmem_yawgyrol = (char *)0x30003038;

char * pmem_WORD1 = (char *)0x30003071; 
char * pmem_WORD1new = (char *)0x30003072;
char * pmem_wrSPI = (char*)0x30003073;
char * shua = (char*)0x30003074;
char * pmem_355_rst_ = (char*)0x30003033;
char * pmem_355_cs = (char*)0x30003032;
char * pmem_355_sclk = (char*)0x30003034;
char * pmem_355_din = (char*)0x30003036;
char * pmem_355_dout = (char*)0x30003038;

char * pmem_mag_cs = (char*)0x3000307e;

char * pmem_width1h = (char *)0x30003048;
char * pmem_width1l = (char *)0x30003049;
char * pmem_width2h = (char *)0x3000304a;
char * pmem_width2l = (char *)0x3000304b;
char * pmem_width3h = (char *)0x3000304c;
char * pmem_width3l = (char *)0x3000304d;
char * pmem_width4h = (char *)0x3000304e;
char * pmem_width4l = (char *)0x3000304f;
char * pmem_width5h = (char *)0x30003060;
char * pmem_width5l = (char *)0x30003061;
char * pmem_width6h = (char *)0x30003062;
char * pmem_width6l = (char *)0x30003063;
char * pmem_width7h = (char *)0x30003064;
char * pmem_width7l = (char *)0x30003065;
char * pmem_width8h = (char *)0x30003066;
char * pmem_width8l = (char *)0x30003067;
char * pmem_width9h = (char *)0x30003068;
char * pmem_width9l = (char *)0x30003069;

char * pmem_poke = (char *)0x3000307f;

int duan;

unsigned int data_1256, acc1_1256,acc2_1256,acc3_1256,gyro1_1256,gyro2_1256,gyro3_1256;
unsigned int data1[4],data2[4],data3[4],data4[4],data5[4],data6[4],data7[4],data8[4];
unsigned int data1a,data2a,data3a,data4a,data5a,data6a,data7a,data8a;
float idata1a,idata2a,idata3a,idata4a,idata5a,idata6a,idata7a,idata8a,idata9a;
float oldidata4a,oldidata5a,oldidata6a;
int adi1[10],adi2[10],adi3[10],adi4[10],adi5[10],adi6[10],adi7[10],adi8[10],adi9[10];

float WTTIME = 50;
float VREF = 2.5;
float vout1,vout2,vout3,facc1,facc2,facc3,fgyro1,fgyro2,fgyro3,voutp,vout12,voutp_average;
float zero_gyro1,zero_gyro2,zero_gyro3;
float vout4,vout5,vout6;
float facc1_1,facc2_1,facc3_1;
float wfacc3 = 1;

int gpsvalidFlag = 0;


unsigned char iic_wrbuf[30],iic_rdbuf[30],con[50],cont[50],conReverseFlag[20]={1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},tmpcont1,tmpcont2,tmpcont3,tmpcont4,con_error=0;

char oldsatelnum = 5;	//Checks if localized, if localized, starts calculation for state estimation

char ailZW = 1, panflag = 0, ResetFlag = 0;

unsigned char forchar3,trdchar3,sndchar3,fstchar3;
unsigned char ReceiveStdLaneFlag=0,bhead=0,dinflag=0,di2flag=0,bhead2=0,bhead3=0,rdParaHeadFlag=0,wrParaHeadFlag=0,allwrParaHeadFlag1 = 0,allwrParaHeadFlag2=0,newParaData=0,send_satelnumflag=1,haveSendBack=1,haveSendBack_all = 1,sendback_din=1,StandardLane_havesendback=1,senddin=1,changeTargetFlag=0,ytjd1Flag=0,ytjd2Flag=0,changeheightFlag=0,changeSuduFlag=0,Waypoint_number_flag=0,StandardLaneCt = 0,StandardLaneNum=0,bhead_satelnum=0,bhead_position=0,bhead_angle=0,bhead_time=0,sixchar3,fifchar3, sendpara = 1, havesend_t2p = 1, sendparact = 0;

char havesend_imu0 = 1,havesend_imu1 = 1, havesend_temperature1 = 1,havesend_temperature2 = 1,havesend_temperature3 = 1,havesend_temperature4 = 1;
char IMUdata0 = 0, IMUdata1 = 0, temperaturedat1 = 0, temperaturedat2 = 0, temperaturedat3 = 0, temperaturedat4 = 0;

unsigned char send_paraX = 0, send_limited = 0;
int Waypoint_number=0,Waypoint_number_t=0,curnum=0,c_curnum=0;
unsigned char bhead_lyjt=0,bhead_ljjt=0;
unsigned char spahead = 0, para_data, para_num, para_data2, para_num2;
char cthavesendback = 0;
unsigned char XCDJReceived=0,DJFXReceived=0;
unsigned char CaptureFlag=0;
unsigned char enter_set=0, clear_bias=0;
int clear_bias_time = 0;
int npos1,npos12;
unsigned char npos_gps=0;
char Latti_Array[4],Longi_Array[4];
int PlaneRoll=0,PlanePitch=0,old_PlanePitch=0;
float t_PlanePitch = 0, t_PlaneRoll = 0;
float z_PlanePitch = 0, z_PlaneRoll = 0;
float dy = 0;
float valuePitchD;

float	RollCorrect=0,PitchCorrect=0;

float COS_FACTOR= 0.766;

int	vFORCEU=0,v12U=0,vCURRENTU,vduojiU;
int old_v12U=0, old_vFORCEU=0, old_vCURRENTU=0;
char ccterror=0, ccterror_vforce = 0, ccterror_vcurrent=0;
char ccterror_fpgain7=0, ccterror_fpgain8 = 0;
float vforce=0, vcurrent = 0;
int ctpower = 0, power_mag = 0;


char tiBuf[20];

char ctspi = 0,jiangstatus=0;
unsigned char PitchCount = 0,ctwending = 0;

int nian,yue,ri,shi,fen,miao;

unsigned int elevcount1=0,elevcount2=0;

unsigned int 	StTick;
unsigned int avst = 0;
unsigned char OUT_ARM_AUTOHANDLEBF;



unsigned char tspinwan[2],thovertime[2];

char ClickSetTgtFlag = 0;	//Is a fly-to point?
char first_reach = 0, yuntai_lock = 0, yici_lock = 0,count_banmiao = 25;


int tJY;


float tgt_Latti[512];
float tgt_Longi[512];
int	  itgt_Latti[512];
int	  itgt_Longi[512];
int   tgt_High[512];
int   tgt_Vel[512];
unsigned short tgt_zwl[512];		//Early turn var, 2 bytes
unsigned char  tgt_hover_num[512];		//Enter hover mode for a waypoint, 0: cancel; 1 byte
unsigned short tgt_hover_time[512];	//Hover time,  2 bytes
unsigned char  tgt_djnum[512];		//Run steering engine no., 1 byte
unsigned char  tgt_djrpttime[512];	//Start/stop delay time, 1 byte
unsigned char  tgt_djtime[512];		//Repeat time, 1 byte

float tgt_Latti2[512]; 
float tgt_Longi2[512];
int   itgt_Latti2[512];
int   itgt_Longi2[512];

int   tgt_High2[512];
int   tgt_Vel2[512];
unsigned short tgt_zwl2[512];		//Early turn var, 2 bytes
unsigned char  tgt_hover_num2[512];	//Enter hover mode for a waypoint, 0: cancel; 1 byte
unsigned short tgt_hover_time2[512];	//Hover time,  2 bytes
unsigned char  tgt_djnum2[512];		//Run steering engine no., 1 byte
unsigned char  tgt_djrpttime2[512];	//Start/stop delay time, 1 byte
unsigned char  tgt_djtime2[512];	//Repeat time, 1 byte

unsigned char tveriail;

float tempLatti; 	//Coordinates of the current point 
float tempLongi; 

void buf2float(float *tfloat, char *buf);
void buf2float2(float *tfloat, char *buf);
void buf2long(long *tfloat, char *buf);
void buf2int(int *tint, char *buf);
void float2buf(char *buf,float *tfloat);
void int2buf(char *buf,int *tint);

float CalFxWidth(void);
float CalailWidth(void);
float CalelevWidth(void);
float CalthrotWidth(void);

char PointArrive(void);
void doCal(void);

void doCapture(void);
int iCurLatti,iCurLongi;

float inaCurLatti=0,inaCurLongi=0, CurLatti_old = 0;
float sCurLatti,sCurLongi,scalheading;
float CurLongi_old = 0;
int intCurLatti = 0, intCurLongi = 0;
float   tgtLatti2,tgtLongi2,tgtLatti3,tgtLongi3;


double CurLatti = 39.97902, CurLongi = 116.50106;
double	tgtLatti,tgtLongi;
float ytgtLatti,ytgtLongi;
float stgtLatti,stgtLongi;

float gimbal_target_latti = 0,gimbal_target_longi = 0;
long  qCurLatti,qCurLongi,qGPShigh;
float startLatti = 0, startLongi = 0;

double lCurLatti = 0;
double lCurLongi = 0;
float GPShigh,gimbal_TARGET_HEIGHT_1=0;
int iGPShigh;

int	  tvels_i = 0;

float PlaneYaw,Simulate_PlaneYaw=0;
int iPlaneYaw = 0;


unsigned char veriail;
short FX=1500,ail=1500,elev=1500,THROT=1930;
float Xthrot=1930;
float vel_up = 0, curspeed = 0;

float tgyro_1_10=0,tgyro_1_9=0,tgyro_1_8=0,tgyro_1_7=0,tgyro_1_6=0,tgyro_1_5=0,tgyro_1_4=0,tgyro_1_3=0,tgyro_1_2=0,tgyro_1_1=0,tgyro_1=0;
float tgyro_2_10=0,tgyro_2_9=0,tgyro_2_8=0,tgyro_2_7=0,tgyro_2_6=0,tgyro_2_5=0,tgyro_2_4=0,tgyro_2_3=0,tgyro_2_2=0,tgyro_2_1=0,tgyro_2=0;
float tgyro_3_10=0,tgyro_3_9=0,tgyro_3_8=0,tgyro_3_7=0,tgyro_3_6=0,tgyro_3_5=0,tgyro_3_4=0,tgyro_3_3=0,tgyro_3_2=0,tgyro_3_1=0,tgyro_3=0;
float tmag_3_6=0,tmag_3_5=0,tmag_3_4=0,tmag_3_3=0,tmag_3_2=0,tmag_3_1=0,mag_3=0;
float tmag_2_6=0,tmag_2_5=0,tmag_2_4=0,tmag_2_3=0,tmag_2_2=0,tmag_2_1=0,mag_2=0;
float tmag_1_6=0,tmag_1_5=0,tmag_1_4=0,tmag_1_3=0,tmag_1_2=0,tmag_1_1=0,mag_1=0;


int itgtLatti2,itgtLongi2;
float	tgtHigh;	 //Target height
float   th;
float	tgtVel;		 //Target velocity
float	tgtLatti_s;  //The previous task point's y coordinates
float	tgtLongi_s;  //The previous task point's x coordinates
float 	vectorLatti;
float	vectorLongi;

float	AirSpeed,AirSpeedPara = 1;
float   pressure, v_pressure_init;


float zitai_roll=0,zitai_pitch=0;
int  zitaicount = 0;
char SndBuf[OUTLENGTH+119];
char fashe[OUTLENGTH+100],iii=0,fashe_6li[100],fashe2[OUTLENGTH+100]; 
char adnum[2];
char BlackBox[28611+10];
char msg[50000],msg2[10000],msg3[20000];
char teshu[10], gyro_bias_data[10];

unsigned char	mag_0[] = {"$STP"};

unsigned char 	nTrkPt=0,nTrkPtTemp=0,nTrkPtVeriail=0,height_veriail=0,AirHigh_h=0,AirHigh_l=0,theighth,theightl,yheighth=0,yheightl=0,ytheighth,ytheightl;
int fheight;
char GPS_Refresh_error = 0;

unsigned int GPS_Refresh[4];

unsigned char 	bReachTgt=0;


int outt1,outt2,outt3,outt4;
char startengine = 0;
//unsigned short 	tOUT_FXDUO=1500,tOUT_ailDUO=1500,tOUT_elevDUO=1500,tOUT_throtDUO=1500,tOUT_5DUO=1500,tOUT_6DUO=1500,tOUT_7DUO=1500,tOUT_8DUO=1500;
unsigned short		 	tOUT_FXDUO=3000,tOUT_ailDUO=3000,tOUT_elevDUO=3000,tOUT_throtDUO=3000,tOUT_5DUO=3000,tOUT_6DUO=3000,tOUT_7DUO=3000,tOUT_8DUO=3000;
float 			fOUT_FXDUO=1500,fOUT_ailDUO=1500,fOUT_elevDUO=1500,fOUT_throtDUO=1500,fOUT_5DUO=1500,fOUT_6DUO=1500,fOUT_7DUO=1500,fOUT_8DUO=1500;
float 			ofOUT_FXDUO=1930,ofOUT_ailDUO=1930,ofOUT_elevDUO=1930,ofOUT_throtDUO=1930,ofOUT_5DUO=1930,ofOUT_6DUO=1930,ofOUT_7DUO=1930,ofOUT_8DUO=1930;
short  fan_tOUT_elevDUO = 1500;

short  Rudder_wifi=1500,Aileron_wifi=1500,Elevator_wifi=1500,Throttle_wifi=1550;

short	Rudder_MID = 1500;
short 	Aileron_MID = 1500;
short 	Elevator_MID = 1500;

int	xs1=1,xs2=1;

short 	throtMAX=1930;
short	throtMIN=1100;
short  rudMAX = 1500, rudMIN = 1500, ailMAX = 1500, ailMIN = 1500, elevMAX = 1500, elevMIN = 1500;


unsigned char	 bEast,tbeast,ytbeast;
unsigned char	 bNorth,tbnorth,ytbnorth;
unsigned char	 bGT45,bfan,ybfan;
unsigned char	 bLeft;	//1: Currently heading towards the target's right; should fly left. 0: Currently heading towards the target's left; should fly right

unsigned char	ParaData[1000],backup[1000];
int	paraX[33];
char paraUSER[33];

int adi_data;
unsigned char	SetAirSpeed = 90;
float iSetAirSpeed = 90, fSetAirSpeed = 90;

unsigned char ctmoni = 0;
unsigned char ct_mpxh = 0, ct_v12=0;

int sxEKF3,sxEKF4,sxEKF5;
//extern double Venn[3];
extern double jw[3];
//*--------------------------------------------------------------------------------------
//* Function Name       : GetTickCount()
//* Object              : Return the number of systimer tick 
//* Input Parameters    :
//* Output Parameters   :
//*--------------------------------------------------------------------------------------
unsigned int GetTickCount(void)
{
	return StTick;
}

//*-------------------------- Interrupt handlers ----------------------------------------
//*--------------------------------------------------------------------------------------
//* Function Name       : irq1_c_handler()
//* Object              : C Interrupt handler for Interrupt source 1
//* Input Parameters    : none
//* Output Parameters   : none
//* 
//* Interrupts when there is data in the COM port. 
//*--------------------------------------------------------------------------------------

//Gets called when there is data on the GCS to be sent to the flight control. 
void AT91F_ST_HANDLER(void)
{
	int i,kk,ik;
	unsigned char wifi_veriail,wifi_veriail2,wifi_veriail3;
	float ddx,ddy,ddxx,ddyy;
	float limit_distance=0;
	float fxc=0,ailc=0,elevc=0;
	unsigned char veriail2,tt;
	int ti=0;


	if (AT91C_BASE_ST->ST_SR & 0x01) 
	{
		StTick++; 
		return;
	}


	if(AT91C_BASE_DBGU->DBGU_CSR & 0x10 )
	{


		AT91F_PDC_SetNextRx(AT91C_BASE_PDC_DBGU, &wifi_buffer[0], 30);


		if (wifi_end >1500)
		{
			wifi_end = 0;
			wifi_index = 0;
		}

		for (i=0;i<30;i++)
			pwifi_buffer[wifi_end+i] = wifi_buffer[i];

		wifi_end+=30;

wifi_loop:;

		if (wifi_end - wifi_index < 30) 
			goto end_wifi_loop;

		//Reset default settings
		if ((pwifi_buffer[wifi_index]=='$')&&(pwifi_buffer[wifi_index+1]=='H')&&(pwifi_buffer[wifi_index+2]=='F')&&(wifi_end - wifi_index >= 30))
		{
			if ((THROT > 1900)&&(Channel_5<1333))
			{
				Parameter[YAW_P] =	64;		
				Parameter[POWER_NO] =	4;		
				Parameter[YAW_D] =	64;		
				Parameter[ROLL_P] =	80;		
				Parameter[STATE_CHANNEL_RANGE_I] =	10;		
				Parameter[MAX_LIFT_VELOCITY] =	100;	    //*0.8cm/s
				Parameter[AUTO_LANDING_VELOCITY] =	30;		//cm/s
				Parameter[YAW_I_RANGE] =	20;		
				Parameter[THROTTLE_P] =	80;					
				Parameter[ERASE_ROMBOOT] =	0;		
				Parameter[CALIB_ACCEL] =	0;		
				Parameter[TEST_CONTROL_FREQ] =	5;	
				Parameter[YAW_I] =	100;	
				Parameter[GIMBAL_ROLL_SENSITIVITY] =	20;		
				if (Parameter[FLIGHT_WARNING_TIME]!=251)
					Parameter[FLIGHT_WARNING_TIME] =	250;	
				Parameter[TARGET_STATE] =	25;				//helps in wind resistance
				//Parameter[16] =	3;						//为离目标高度多少米到达最大目标速度，即高度的1/P, 一般为3, 即离3米则最大目标速度
				Parameter[VEL_STATE_RANGE_I] =	100;	
				Parameter[PERPENDICULAR_STAB_COEF] =	150;	
				Parameter[GIMBAL_PITCH_SENSIT] =	20;		
				Parameter[PITCH_D] =	45;		
				Parameter[STATE_SENSITIVITY] =	50;		
				Parameter[PITCH_P] =	80;		
				Parameter[MAGNETIC_VARIATION] =	65;			//*10
				Parameter[NEW_THROTT_I] =	50;			
				Parameter[POINT_RADIUS] =	5;		
				Parameter[STATE_CHANNEL_RANGE_P] =	18;		
			//	Parameter[RESERVED] =	10;		//Reserved，以前是离悬停点多少米到达最大目标速度，即位置的1/P, 一般为10, 即离10米则最大目标速度
				Parameter[MAX_VELOCITY] =	120;			//Max Velocity Range*4/100: If 120, Vel: 4.8m/s最大飞行速度限制/4；填写120即为4.8m/s
				Parameter[VIB_COMPENSATION] =	50;			
				Parameter[DRONE_TYPE] =	1;					
			//	Parameter[31] =	150;					
			//	Parameter[32] =	20;		//Reserved，以前的方向舵到航向速度系数,现在为取数据的长度, 秒数
				Parameter[YAW_RC_COEFF] =	10;		//The default is 10, the larger the value, RC effect gets lower
				Parameter[ROLL_D] =	45;		
				Parameter[PERP_THROT_COEFF] =	100;		//Damping coefficient
				Parameter[RC_TARGET_STATE_COEFF] =	16;		//Default is 16, the bigger the value, the lower the effect of RC
				Parameter[STATE_CHANNEL_RANGE_D] =	22;		
				Parameter[CONTROL_METHOD] =	2;				//1: State Calculation, 2: Stabilization using acceleration
				Parameter[SELECTION_OPTIONS] =	0x48;		//sbus, battery, gimbal selection 01 00 10	00
				writebuf1 = 1;
			}
			else
			{
				Parameter[SELECTION_OPTIONS] = 0x48; // 01 00 10 00
				writebuf1 = 1;
			}
		}
		else

			//RC Data
			//Control instructions
			if ((pwifi_buffer[wifi_index]=='$')&&(pwifi_buffer[wifi_index+1]=='I')&&(pwifi_buffer[wifi_index+2]=='W')&&(wifi_end - wifi_index >= 30))	
			{
				if (
					(pwifi_buffer[wifi_index+5]==pwifi_buffer[wifi_index+13])&&(pwifi_buffer[wifi_index+6]==pwifi_buffer[wifi_index+14])
					&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+15])&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+16])
					&&(pwifi_buffer[wifi_index+9]==pwifi_buffer[wifi_index+17])&&(pwifi_buffer[wifi_index+10]==pwifi_buffer[wifi_index+18])
					&&(pwifi_buffer[wifi_index+11]==pwifi_buffer[wifi_index+19])&&(pwifi_buffer[wifi_index+12]==pwifi_buffer[wifi_index+20])
					) //Control using cell phone's + remote （available on GCS)
					//This instruction should be sent extremely frequently (20 times a second)
					//If the autopilot constantly receives this data, this method can be used to control the data
					// but if no data is received for 1 second, the control access is automatically given to the physical Remote Control.
					
				{
					Rudder_wifi = pwifi_buffer[wifi_index+5]*256 + pwifi_buffer[wifi_index+6];
					Aileron_wifi = pwifi_buffer[wifi_index+7]*256 + pwifi_buffer[wifi_index+8];
					Elevator_wifi = pwifi_buffer[wifi_index+9]*256 + pwifi_buffer[wifi_index+10];
					Throttle_wifi = pwifi_buffer[wifi_index+11]*256 + pwifi_buffer[wifi_index+12];
					if (Throttle_wifi > 1899) Throttle_wifi = 1899;
				}
				wifi_data = hz50;						

				for (i=wifi_index;i<wifi_end-30;i++)
					pwifi_buffer[i] = pwifi_buffer[i+30];
				wifi_end -= 30;							
				goto wifi_loop;

			}

			//Data sent through Wifi
			else if ((pwifi_buffer[wifi_index]=='$')&&(pwifi_buffer[wifi_index+1]=='W')&&(pwifi_buffer[wifi_index+2]=='I')&&(wifi_end - wifi_index >= 30))	
			{
				if ((pwifi_buffer[wifi_index+5]==0)&&(pwifi_buffer[wifi_index+6]==0)&&(pwifi_buffer[wifi_index+7]==0))
				{
					Rudder_wifi = Rudder_MID;
					Aileron_wifi = Aileron_MID;
					Elevator_wifi = Elevator_MID;
					Throttle_wifi = 1550;
					wifi_data = hz50;						
				}	
				else if ((pwifi_buffer[wifi_index+5]==1)&&(pwifi_buffer[wifi_index+6]==1)
							&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+8])
							&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+9]))
				{
					Rudder_wifi = Rudder_MID;
					Aileron_wifi = Aileron_MID;
					Elevator_wifi = Elevator_MID - pwifi_buffer[wifi_index+7]*5;
					Throttle_wifi = 1550;
					wifi_data = hz50;						
				}	
				else if ((pwifi_buffer[wifi_index+5]==2)&&(pwifi_buffer[wifi_index+6]==2)&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+8])&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+9]))
				{
					Rudder_wifi = Rudder_MID;
					Aileron_wifi = Aileron_MID + pwifi_buffer[wifi_index+7]*5;
					Elevator_wifi = Elevator_MID;
					Throttle_wifi = 1550;
					wifi_data = hz50;						
				}	
				else if ((pwifi_buffer[wifi_index+5]==3)&&(pwifi_buffer[wifi_index+6]==3)&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+8])&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+9]))
				{
					Rudder_wifi = Rudder_MID;
					Aileron_wifi = Aileron_MID;
					Elevator_wifi = Elevator_MID + pwifi_buffer[wifi_index+7]*5;
					Throttle_wifi = 1550;
					wifi_data = hz50;						
				}	
				else if ((pwifi_buffer[wifi_index+5]==4)&&(pwifi_buffer[wifi_index+6]==4)&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+8])&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+9]))
				{
					Rudder_wifi = Rudder_MID;
					Aileron_wifi = Aileron_MID - pwifi_buffer[wifi_index+7]*5;
					Elevator_wifi = Elevator_MID;
					Throttle_wifi = 1550;
					wifi_data = hz50;						
				}	
				else if ((pwifi_buffer[wifi_index+5]==5)&&(pwifi_buffer[wifi_index+6]==5)&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+8])&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+9]))
				{
					Rudder_wifi = Rudder_MID;
					Aileron_wifi = Aileron_MID;
					Elevator_wifi = Elevator_MID;
					Throttle_wifi = (High_Throttle+Low_Throttle)/2 - pwifi_buffer[wifi_index+7]*10;
					if (Throttle_wifi > 1899) 
						Throttle_wifi = 1899;

					wifi_data = hz50;						
				}	
				else if ((pwifi_buffer[wifi_index+5]==6)&&(pwifi_buffer[wifi_index+6]==6)&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+8])&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+9]))
				{
					Rudder_wifi = Rudder_MID + pwifi_buffer[wifi_index+7]*5;
					Aileron_wifi = Aileron_MID;
					Elevator_wifi = Elevator_MID;
					Throttle_wifi = 1550;
					wifi_data = hz50;						
				}	
				else if ((pwifi_buffer[wifi_index+5]==7)&&(pwifi_buffer[wifi_index+6]==7)&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+8])&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+9]))
				{
					Rudder_wifi = Rudder_MID;
					Aileron_wifi = Aileron_MID;
					Elevator_wifi = Elevator_MID;
					Throttle_wifi = (High_Throttle+Low_Throttle)/2 + pwifi_buffer[wifi_index+7]*10;
					if (Throttle_wifi > 1899) 
						Throttle_wifi = 1899;
					wifi_data = hz50;						
				}	
				else if ((pwifi_buffer[wifi_index+5]==8)&&(pwifi_buffer[wifi_index+6]==8)&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+8])&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+9]))
				{
					Rudder_wifi = Rudder_MID - pwifi_buffer[wifi_index+7]*5;
					Aileron_wifi = Aileron_MID;
					Elevator_wifi = Elevator_MID;
					Throttle_wifi = 1550;
					wifi_data = hz50;						
				}	
				else if ((pwifi_buffer[wifi_index+5]==12)&&(pwifi_buffer[wifi_index+6]==12)&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+9])&&(pwifi_buffer[wifi_index+9]==pwifi_buffer[wifi_index+11])&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+10])&&(pwifi_buffer[wifi_index+10]==pwifi_buffer[wifi_index+12]))
				{
					Rudder_wifi = Rudder_MID;
					Aileron_wifi = Aileron_MID + pwifi_buffer[wifi_index+7]*5;
					Elevator_wifi = Elevator_MID - pwifi_buffer[wifi_index+8]*5;
					Throttle_wifi = 1550;
					wifi_data = hz50;						
				}	
				else if ((pwifi_buffer[wifi_index+5]==11)&&(pwifi_buffer[wifi_index+6]==11)&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+9])&&(pwifi_buffer[wifi_index+9]==pwifi_buffer[wifi_index+11])&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+10])&&(pwifi_buffer[wifi_index+10]==pwifi_buffer[wifi_index+12]))
				{
					Rudder_wifi = Rudder_MID;
					Aileron_wifi = Aileron_MID + pwifi_buffer[wifi_index+7]*5;
					Elevator_wifi = Elevator_MID + pwifi_buffer[wifi_index+8]*5;
					Throttle_wifi = 1550;
					wifi_data = hz50;						
				}	
				else if ((pwifi_buffer[wifi_index+5]==10)&&(pwifi_buffer[wifi_index+6]==10)&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+9])&&(pwifi_buffer[wifi_index+9]==pwifi_buffer[wifi_index+11])&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+10])&&(pwifi_buffer[wifi_index+10]==pwifi_buffer[wifi_index+12]))
				{
					Rudder_wifi = Rudder_MID;
					Aileron_wifi = Aileron_MID - pwifi_buffer[wifi_index+7]*5;
					Elevator_wifi = Elevator_MID + pwifi_buffer[wifi_index+8]*5;	
					Throttle_wifi = 1550;
					wifi_data = hz50;						
				}	
				else if ((pwifi_buffer[wifi_index+5]==9)&&(pwifi_buffer[wifi_index+6]==9)&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+9])&&(pwifi_buffer[wifi_index+9]==pwifi_buffer[wifi_index+11])&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+10])&&(pwifi_buffer[wifi_index+10]==pwifi_buffer[wifi_index+12]))
				{
					Rudder_wifi = Rudder_MID;
					Aileron_wifi = Aileron_MID - pwifi_buffer[wifi_index+7]*5;
					Elevator_wifi = Elevator_MID - pwifi_buffer[wifi_index+8]*5;
					Throttle_wifi = 1550;
					wifi_data = hz50;						
				}	
				
				//Point and fly
				else if (pwifi_buffer[wifi_index+5]==81)							
				{
					buf2float(&tgtLatti3,(char*)&pwifi_buffer[wifi_index+14]);//Put the target point's latitude in the array
					buf2float(&tgtLongi3,(char*)&pwifi_buffer[wifi_index+18]);//Put the target point's longitude in the array

					buf2float(&tgtLatti2,(char*)&pwifi_buffer[wifi_index+6]); //Put the target point's latitude in the array again
					buf2float(&tgtLongi2,(char*)&pwifi_buffer[wifi_index+10]);//Put the target point's longitude in the array again

					//Check whether the protocol is satisfied
					if ((tgtLatti2==tgtLatti3)&&(tgtLongi2==tgtLongi3)&&(pwifi_buffer[wifi_index+22]==81))
					{
						gfx = 0;
						ddy = (tgtLatti2 - CurLatti)*111199;
						ddx = (tgtLongi2 - CurLongi)*111199*COS_FACTOR;
						tgtLatti = tgtLatti2;
						tgtLongi = tgtLongi2;

						senddin = 0;
						ClickSetTgtFlag = 1;
						first_reach = 1;
						tempLatti = tgtLatti;
						tempLongi = tgtLongi;
						sendback_din = 0;
					}
				}	
				//Set flags for the download of all the waypoints on the GCS
				else if ((pwifi_buffer[wifi_index+5]==82)&&(pwifi_buffer[wifi_index+6]==82)&&(pwifi_buffer[wifi_index+7]==82))		
				{
					haveSendBack_all=0;
					ctindex = 0;
				}	
				//Upload a waypoint's latitude and longitude
				else if (pwifi_buffer[wifi_index+5]==83)
				{
				
					//Checksum
					wifi_veriail = 0;
					for(i=0;i<18;i++)
						wifi_veriail += pwifi_buffer[wifi_index+5+i];

					//If checksum value is correct
					if (wifi_veriail==pwifi_buffer[wifi_index+23])
					{
						c_curnum = (unsigned char)pwifi_buffer[wifi_index+1+5]; //waypoint no.
						if (c_curnum > 0)
						{
							buf2float(&ddyy,(char*)&pwifi_buffer[wifi_index+1+6]);//insert waypoint latitude
							buf2float(&ddxx,(char*)&pwifi_buffer[wifi_index+1+10]);//insert waypoint longtude
							ddx = (ddxx - wanjia_longi)*111199*COS_FACTOR;
							ddy = (ddyy - wanjia_latti)*111199;
	
							limit_distance = 500;
							if ((ddx*ddx + ddy*ddy < limit_distance*limit_distance) &&  
									(c_curnum <= 8))
							{
								buf2float(&tgt_Latti[c_curnum-1],(char*)&pwifi_buffer[wifi_index+1+6]);//Insert waypoint latitude
								buf2float(&tgt_Longi[c_curnum-1],(char*)&pwifi_buffer[wifi_index+1+10]);//Insert waypoint longitude
								buf2int(&tgt_High[c_curnum-1],(char*)&pwifi_buffer[wifi_index+1+14]);//Insert waypoint's target height
								tgt_hover_time[c_curnum-1] = pwifi_buffer[wifi_index+1+19]*256+pwifi_buffer[wifi_index+1+18];
								tgt_hover_num[c_curnum-1] = pwifi_buffer[wifi_index+1+20];
								tgt_Vel[c_curnum-1]	= pwifi_buffer[wifi_index+1+21];
								writebuf2 = 1;
								haveSendBack=0;					//Start back transfer
							}
						}
					}
				}	
				//Upload the total number of waypoints
				else if ((pwifi_buffer[wifi_index+5]==84)&&(pwifi_buffer[wifi_index+6]==pwifi_buffer[wifi_index+7])
						&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+8])&&(pwifi_buffer[wifi_index+9]==84))						
				{
					Waypoint_number=pwifi_buffer[wifi_index+6];
					if (Waypoint_number > 8) //the maximum number of waypoints allowed is 8. but this can be changed to one's preferences.
						Waypoint_number = 8;
					if (nTrkPt >= Waypoint_number) 
						nTrkPt = 0;
					writebuf2 = 1;
				}	
				//Exit Gimbal lock and fly
				else if ((pwifi_buffer[wifi_index+5]==85)&&(pwifi_buffer[wifi_index+6]==85)&&(pwifi_buffer[wifi_index+7]==85))		
				{
					gimbal_target_lock = 0;
					ClickSetTgtFlag = 0;
				}
				//Set current position as "home"
				else if ((pwifi_buffer[wifi_index+5]==86)&&(pwifi_buffer[wifi_index+6]==86)&&(pwifi_buffer[wifi_index+7]==86))
				{
					homelatti = CurLatti;
					homelongi = CurLongi;
				}	
				//Autonomosly go "home" and land
				else if ((pwifi_buffer[wifi_index+5]==87)&&(pwifi_buffer[wifi_index+6]==87)&&(pwifi_buffer[wifi_index+7]==87))	
				{
					//If localized
					if ((satelnum > 4)&&(pregohome==0)&&(gohome==0))
					{	
						//If the target is too low
						if ((tgtHigh < 200)&&(AirHighf*10<200))
						{
							tgtHigh = 200;
							AirHigh_h = (int)tgtHigh / 256;
							AirHigh_l = (int)tgtHigh - AirHigh_h * 256;	
						}
						//If the copter is not ready to go home
						else if (pregohome==0)
						{
							tgtHigh = AirHighf*10;
							AirHigh_h = (int)tgtHigh / 256;
							AirHigh_l = (int)tgtHigh - AirHigh_h * 256;	
						}
						//Stop following waypoints
						if (Enable_Waypoint==1)
						{
							Enable_Waypoint = 0;
						}
						//Ready to go home
						pregohome = 1;
				
						gimbal_target_lock = 0;
					}
				}	
				//Auto take-off
				else if ((pwifi_buffer[wifi_index+5]==88)&&(pwifi_buffer[wifi_index+6]==88)&&(pwifi_buffer[wifi_index+7]==88))	
				{
					//If the throttle is not locked, there's enough GPS signal, and the current flight mode is autonomous
					if ((!Lock_Throttle)&&(satelnum>4)&&(bAutoHand_))
					{
						if ((autotakeoff==0)&&(!in_flight))
						{
							clear_pressure = 1;
							autotakeoff = 1;	
							tgtLatti = CurLatti;
							tgtLongi = CurLongi;
							reset_throt = 1;
						}
					}
				}
				
				//TODO:
				//Enable RC
				else if ((pwifi_buffer[wifi_index+5]==89)&&(pwifi_buffer[wifi_index+6]==89)&&(pwifi_buffer[wifi_index+7]==89))	
				{
					if(rc_enabled!=1)
						rc_enabled=1;
				}
				//TODO:
				//Disable RC
				else if ((pwifi_buffer[wifi_index+5]==90)&&(pwifi_buffer[wifi_index+6]==90)&&(pwifi_buffer[wifi_index+7]==90))	
				{
					if(rc_enabled!=0)
						rc_enabled=0;
				}
				
				//Set Follow Me coordinates	and parameters				
				else if (pwifi_buffer[wifi_index+5]==91)	
				{
					//put coordinates in the array
					buf2float(&tgtLatti3,(char*)&pwifi_buffer[wifi_index+14]);
					buf2float(&tgtLongi3,(char*)&pwifi_buffer[wifi_index+18]);
					buf2float(&tgtLatti2,(char*)&pwifi_buffer[wifi_index+6]);
					buf2float(&tgtLongi2,(char*)&pwifi_buffer[wifi_index+10]);

					if ((tgtLatti2==tgtLatti3)&&(tgtLongi2==tgtLongi3)&&(pwifi_buffer[wifi_index+22]==91))
					{
					//??
						fxc = fabsf(FX - Rudder_MID);
						ailc = fabsf(ail - Aileron_MID);
						elevc = fabsf(elev - Elevator_MID);
						if ((fxc<30)&&(ailc<30)&&(elevc<30)&&(Channel_5>1666)&&(Channel_6<1333))	
						//Only works when the cell phone is turned on, and automatic localization is turned on
						{
							tgtLatti = tgtLatti2;
							tgtLongi = tgtLongi2;
							ddy = (tgtLatti2 - CurLatti)*111199;
							ddx = (tgtLongi2 - CurLongi)*111199*COS_FACTOR;
							if (ddx*ddx + ddy*ddy > 5*5)
							{
								vectorLatti = tgtLatti - CurLatti;
								vectorLongi = tgtLongi - CurLongi;
								ref_yaw = CalAtanVector();
								targetYaw = ref_yaw * 180/PI;
							}
						}
					}
				}
				//Gimbal lock target
				else if (pwifi_buffer[wifi_index+5]==92)							
				{
					//insert coordinates in the array
					buf2float(&tgtLatti3,(char*)&pwifi_buffer[wifi_index+14]);
					buf2float(&tgtLongi3,(char*)&pwifi_buffer[wifi_index+18]);
					buf2float(&tgtLatti2,(char*)&pwifi_buffer[wifi_index+6]);
					buf2float(&tgtLongi2,(char*)&pwifi_buffer[wifi_index+10]);

					if ((tgtLatti2==tgtLatti3)&&(tgtLongi2==tgtLongi3)&&(pwifi_buffer[wifi_index+22]==92))
					{
						if ( ((tgtLatti2 > 1)||(tgtLatti2 < -1)) && ((tgtLongi2 > 1)||(tgtLongi2 < -1)) )
						{
							gimbal_target_latti = tgtLatti2;
							gimbal_target_longi = tgtLongi2;
						}
						gimbal_target_lock = 1;
					}
				}	
				//Clear gyro
				else if ((pwifi_buffer[wifi_index+5]==100)&&(pwifi_buffer[wifi_index+6]==100)&&(pwifi_buffer[wifi_index+7]==100))	
				{
					if ((Channel_5<1333)&&(THROT > 1900))
					{
						clear_bias = 1;
						bias_iraw_gyro_1_zu = 0;
						bias_iraw_gyro_2_zu = 0;
						bias_iraw_gyro_3_zu = 0;
						clear_bias_time=0;
					}
				}	
				//Get neutral position on R/C (Get current position on R/C and set as neutral pos)
				else if ((pwifi_buffer[wifi_index+5]==102)&&(pwifi_buffer[wifi_index+6]==102)&&(pwifi_buffer[wifi_index+7]==102))
				{
					CaptureFlag = 1;
				}	
				//Update target point (Auto Nav)
				else if ((pwifi_buffer[wifi_index+5]==103)&&(pwifi_buffer[wifi_index+6]==103))					
				{
					if ((pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+8])&&(pwifi_buffer[wifi_index+7] <= Waypoint_number)&&(pwifi_buffer[wifi_index+7] > 0))
					{
						if (satelnum>4)
						{
							tgtLatti_s = CurLatti;
							tgtLongi_s = CurLongi;
						}
						else
						{
							tgtLatti_s = tgt_Latti[0];
							tgtLongi_s = tgt_Longi[0];
						}
						jiangstatus = 0; //??
						ClickSetTgtFlag = 0;
						panflag = 0;
						newpanxuan = 0;
						first_reach = 0;
						if ((pwifi_buffer[wifi_index+7] <= Waypoint_number)&&(pwifi_buffer[wifi_index+7]>=0))
							nTrkPt = pwifi_buffer[wifi_index+7] - 1;
						if (auto_stay_ding==0)							//Waypoint mode
						{
							tgtLatti = tgt_Latti[nTrkPt];
							tgtLongi = tgt_Longi[nTrkPt];
						}
						start_hover = 0;
						hover_time = 0;
					}
				}	
				//Get flight parameters (Read PID values)
				else if ((pwifi_buffer[wifi_index+5]==104)&&(pwifi_buffer[wifi_index+6]==104)&&(pwifi_buffer[wifi_index+7]==104))	
				{
					rdParaHeadFlag = 1;
				}
				//Update flight parameters (through GCS)
				else if ((pwifi_buffer[wifi_index+5]==105)&&(pwifi_buffer[wifi_index+6]==105)
						&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+9])&&(pwifi_buffer[wifi_index+9]==pwifi_buffer[wifi_index+11])
						&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+10])&&(pwifi_buffer[wifi_index+10]==pwifi_buffer[wifi_index+12]))						
				{
					Parameter[pwifi_buffer[wifi_index+7]] = pwifi_buffer[wifi_index+8];
					writebuf1 = 1;
				}
				//Update/change height
				else if ((pwifi_buffer[wifi_index+5]==106)&&(pwifi_buffer[wifi_index+6]==106)
						&&(pwifi_buffer[wifi_index+7]==pwifi_buffer[wifi_index+9])&&(pwifi_buffer[wifi_index+9]==pwifi_buffer[wifi_index+11])
						&&(pwifi_buffer[wifi_index+8]==pwifi_buffer[wifi_index+10])&&(pwifi_buffer[wifi_index+10]==pwifi_buffer[wifi_index+12]))						
				{
					theighth = pwifi_buffer[wifi_index+8];
					theightl = pwifi_buffer[wifi_index+7];
					fheight = (theighth*256 + theightl);
					theighth = fheight / 256;
					theightl = fheight - theighth*256;
					AirHigh_h = theighth;
					AirHigh_l = theightl;
					th = AirHigh_h*256 + AirHigh_l;
					if (th < 32768)
						tgtHigh = th;
					else
						tgtHigh = th - 65536;

					if (tgtHigh==0)
						tgtHigh = tgt_High[nTrkPt];
					high100 = 0;
				}	
				//Start compass calibration (horiz)
				else if ((pwifi_buffer[wifi_index+5]==110)&&(pwifi_buffer[wifi_index+6]==110)&&(pwifi_buffer[wifi_index+7]==110))	
				{
					Parameter[ERASE_ROMBOOT] = 87;
					rdParaHeadFlag = 1;
				}	
				//Start compass calibration (vertical)
				else if ((pwifi_buffer[wifi_index+5]==111)&&(pwifi_buffer[wifi_index+6]==111)&&(pwifi_buffer[wifi_index+7]==111))   
				{
					Parameter[ERASE_ROMBOOT] = 88;
					rdParaHeadFlag = 1;
				}	
				//Finish compass calibration, and save
				else if ((pwifi_buffer[wifi_index+5]==112)&&(pwifi_buffer[wifi_index+6]==112)&&(pwifi_buffer[wifi_index+7]==112))	
				{
					Parameter[ERASE_ROMBOOT] = 89;
					rdParaHeadFlag = 1;
				}	
				//Read parameters
				else if ((pwifi_buffer[wifi_index+5]==113)&&(pwifi_buffer[wifi_index+6]==113)&&(pwifi_buffer[wifi_index+7]==113))
				{
					Parameter[ERASE_ROMBOOT] = 0;
					rdParaHeadFlag = 1;
				}	
				//Write parameters through GCS
				else if ((pwifi_buffer[wifi_index+5]==115)&&(pwifi_buffer[wifi_index+6]==115))						
				{
					wifi_veriail = 0;
					for(i=0;i<7;i++)
						wifi_veriail += pwifi_buffer[wifi_index+7+i];
					wifi_veriail2 = 0;
					for(i=0;i<7;i++)
						wifi_veriail2 += pwifi_buffer[wifi_index+14+i];
					wifi_veriail3 = 0;
					for(i=0;i<6;i++)
						wifi_veriail3 += pwifi_buffer[wifi_index+21+i];
					if ((wifi_veriail==pwifi_buffer[wifi_index+27])&&(wifi_veriail2==pwifi_buffer[wifi_index+28])
						&&(wifi_veriail3==pwifi_buffer[wifi_index+29])&&(pwifi_buffer[wifi_index+7]!=0))
					{
						if ((pwifi_buffer[wifi_index+8]!=0)&&(pwifi_buffer[wifi_index+9]!=0)
							&&(pwifi_buffer[wifi_index+10]!=0)&&(pwifi_buffer[wifi_index+11]!=0)
							&&(pwifi_buffer[wifi_index+12]!=0)&&(pwifi_buffer[wifi_index+13]!=0)&&(pwifi_buffer[wifi_index+17]!=0)
							&&(pwifi_buffer[wifi_index+18]!=0)&&(pwifi_buffer[wifi_index+21]!=0)&&(pwifi_buffer[wifi_index+22]!=0)
							&&(pwifi_buffer[wifi_index+23]!=0)&&(pwifi_buffer[wifi_index+24]!=0))
						{
							Parameter[ROLL_P] = pwifi_buffer[wifi_index+7];						
							Parameter[VIB_COMPENSATION] = pwifi_buffer[wifi_index+8];			
							Parameter[ROLL_D] = pwifi_buffer[wifi_index+9];			
							Parameter[THROTTLE_P] = pwifi_buffer[wifi_index+10];			
							Parameter[YAW_P] = pwifi_buffer[wifi_index+11];			
							Parameter[YAW_D] = pwifi_buffer[wifi_index+12];			
							Parameter[POWER_NO] = pwifi_buffer[wifi_index+13];		
							Parameter[CONTROL_METHOD] = pwifi_buffer[wifi_index+14];	
							Parameter[MAGNETIC_VARIATION] = pwifi_buffer[wifi_index+15];
							Parameter[DRONE_TYPE] = pwifi_buffer[wifi_index+16];		
							Parameter[PITCH_P] = pwifi_buffer[wifi_index+17];		
							Parameter[PITCH_D] = pwifi_buffer[wifi_index+18];		
							Parameter[GIMBAL_ROLL_SENSITIVITY] = pwifi_buffer[wifi_index+19];
							Parameter[GIMBAL_PITCH_SENSIT] = pwifi_buffer[wifi_index+20];	
							Parameter[PERPENDICULAR_STAB_COEF] = pwifi_buffer[wifi_index+21];
							Parameter[MAX_VELOCITY] = pwifi_buffer[wifi_index+22];		
							Parameter[STATE_SENSITIVITY] = pwifi_buffer[wifi_index+23];
							Parameter[NEW_THROTT_I] = pwifi_buffer[wifi_index+24];		
							if (Parameter[MAX_VELOCITY] > 150)
							{
								Parameter[CONTROL_METHOD] = (Parameter[CONTROL_METHOD] & 0x0f);
								Parameter[CONTROL_METHOD] += 0x20;
							}
							if (enter_set==1)
								Parameter[SELECTION_OPTIONS] = pwifi_buffer[wifi_index+25];		
							if (pwifi_buffer[wifi_index+26]>99)
								Parameter[MAX_LIFT_VELOCITY] = pwifi_buffer[wifi_index+26];
							writebuf1 = 1;
						}
					}	
				}
				//Calibrate the R/C
				else if((pwifi_buffer[wifi_index+5]==117)&&(pwifi_buffer[wifi_index+6]==117)&&(pwifi_buffer[wifi_index+7]==117))	
				{
					adjust_throt = 1;
					throtMAX = 1500;
					throtMIN = 1500;
					rudMAX = 1500;
					rudMIN = 1500;
					ailMAX = 1500;
					ailMIN = 1500;
					elevMAX = 1500;
					elevMIN = 1500;						
					
				}
				//Enter configuration
				else if ((pwifi_buffer[wifi_index+5]==120)&&(pwifi_buffer[wifi_index+6]==120)&&(pwifi_buffer[wifi_index+7]==120))	
				{
					if ((THROT > 1900)&&(Channel_5<1333)&&(enter_set==0))
					{
						enter_set = 1;
						send_enterset = 1;
					}
				}	
				//Exit configuration
				else if ((pwifi_buffer[wifi_index+5]==121)&&(pwifi_buffer[wifi_index+6]==121)&&(pwifi_buffer[wifi_index+7]==121))	
				{
					if (enter_set==1)
					{
						enter_set = 0;
						send_exitset = 1;
						writebuf1 = 1;
					}
				}	
				//Start waypoint navigation
				else if ((pwifi_buffer[wifi_index+5]==122)&&(pwifi_buffer[wifi_index+6]==122)&&(pwifi_buffer[wifi_index+7]==122))	
				{
					bReachTgt = 0; 
					tgtLatti = tgt_Latti[nTrkPt];
					tgtLongi = tgt_Longi[nTrkPt];
					if (Enable_Waypoint==0)
					{
						Enable_Waypoint = 1;
					}
				}
				//Exit waypoint navigation
				else if ((pwifi_buffer[wifi_index+5]==123)&&(pwifi_buffer[wifi_index+6]==123)&&(pwifi_buffer[wifi_index+7]==123))	
				{
					if (Enable_Waypoint==1)
					{
						Enable_Waypoint = 0;
					}
				}	
				//Close motors
				else if ((pwifi_buffer[wifi_index+5]==124)&&(pwifi_buffer[wifi_index+6]==124)&&(pwifi_buffer[wifi_index+7]==124))	
				{
					close = 1;
				}	
				//Open throttle lock
				else if ((pwifi_buffer[wifi_index+5]==137)&&(pwifi_buffer[wifi_index+6]==137)&&(pwifi_buffer[wifi_index+7]==137))	
				{
					if ((Channel_Throttle > 1850)||((Rudder_wifi==Rudder_MID)&&(Aileron_wifi==Aileron_MID)&&(Elevator_wifi==Elevator_MID)&&(Throttle_wifi==1550)))
					{
						Lock_Throttle = 0;
						ctlock_throttle = 0;
					}
				}	
				else if ((pwifi_buffer[wifi_index+5]==138)&&(pwifi_buffer[wifi_index+6]==138)&&(pwifi_buffer[wifi_index+7]==138))
				{
					//Self defined parameters 1
					for(i=0;i<8;i++)
					{
						if (pwifi_buffer[wifi_index+8+i]==pwifi_buffer[wifi_index+16+i])
							paraUSER[i] = pwifi_buffer[wifi_index+8+i];
					}
				}	
				else if ((pwifi_buffer[wifi_index+5]==139)&&(pwifi_buffer[wifi_index+6]==139)&&(pwifi_buffer[wifi_index+7]==139))
				{
					//Self defined parameters 2
					for(i=0;i<8;i++)
					{
						if (pwifi_buffer[wifi_index+8+i]==pwifi_buffer[wifi_index+16+i])
							paraUSER[i+8] = pwifi_buffer[wifi_index+8+i];
					}
				}	
				else if ((pwifi_buffer[wifi_index+5]==140)&&(pwifi_buffer[wifi_index+6]==140)&&(pwifi_buffer[wifi_index+7]==140))
				{
					//Self defined parameters 3
					for(i=0;i<8;i++)
					{
						if (pwifi_buffer[wifi_index+8+i]==pwifi_buffer[wifi_index+16+i])
							paraUSER[i+16] = pwifi_buffer[wifi_index+8+i];
					}
				}	
				else if ((pwifi_buffer[wifi_index+5]==141)&&(pwifi_buffer[wifi_index+6]==141)&&(pwifi_buffer[wifi_index+7]==141))
				{
					//Self defined parameters 4
					for(i=0;i<8;i++)
					{
						if (pwifi_buffer[wifi_index+8+i]==pwifi_buffer[wifi_index+16+i])
							paraUSER[i+24] = pwifi_buffer[wifi_index+8+i];
					}
					writebuf1 = 1;
				}	
				else if ((pwifi_buffer[wifi_index+5]==142)&&(pwifi_buffer[wifi_index+6]==142)&&(pwifi_buffer[wifi_index+7]==142))
				{
					//Send back self defined flight parameters
					send_paraX = 1;
				}
				//Send back limitation parameters
				else if ((pwifi_buffer[wifi_index+5]==143)&&(pwifi_buffer[wifi_index+6]==143)&&(pwifi_buffer[wifi_index+7]==143))
				{
					send_limited = 1;
				}	
				//Start carefree
				else if ((pwifi_buffer[wifi_index+5]==149)&&(pwifi_buffer[wifi_index+6]==149)&&(pwifi_buffer[wifi_index+7]==149))	
				{
					//If currently in waypoint navigation
					if ((Channel_6 > 1333)&&(Channel_6<1666)&&(Enable_Waypoint)&&(Channel_5>1666)&&(in_flight))			
					{
						Waypoint_carefree = 1;
					}
					else
					{
						carefree=1;
						cos_care = cos(scalheading);
						sin_care = sin(scalheading);
					}
				}	
				//Exit Carefree
				else if ((pwifi_buffer[wifi_index+5]==150)&&(pwifi_buffer[wifi_index+6]==150)&&(pwifi_buffer[wifi_index+7]==150))	
				{
					Waypoint_carefree = 0;
					carefree = 0;
				}
				//Send compass data to GCS
				else if ((pwifi_buffer[wifi_index+5]==200)&&(pwifi_buffer[wifi_index+6]==200)&&(pwifi_buffer[wifi_index+7]==200))
				{
					for(i=0;i<ct_rawmag;i++)
					{
						msg[0+13*i]='$';
						msg[1+13*i]='M';
						msg[2+13*i]='G';
						msg[3+13*i]='2';
						float2buf(&msg[4+13*i],&raw_magx[i]);
						float2buf(&msg[8+13*i],&raw_magy[i]);
						msg[12+13*i]=0;
						for(kk=4;kk<12;kk++)
							msg[12+13*i]+=msg[kk+13*i];
					}
					index_mag = ct_rawmag*13;
					for(i=0;i<ct_rawmagz;i++)
					{
						msg[0+13*i+index_mag]='$';
						msg[1+13*i+index_mag]='M';
						msg[2+13*i+index_mag]='G';
						msg[3+13*i+index_mag]='3';
						float2buf(&msg[4+13*i+index_mag],&raw_magyz[i]);
						float2buf(&msg[8+13*i+index_mag],&raw_magzz[i]);
						msg[12+13*i+index_mag]=0;
						for(kk=4;kk<12;kk++)
							msg[12+13*i+index_mag]+=msg[kk+13*i+index_mag];
					}
					plotmag = 1;
					cthui = 0; //??
				}
				//Read the data from the last minute
				else if ((pwifi_buffer[wifi_index+5]==201)&&(pwifi_buffer[wifi_index+6]==201)&&(pwifi_buffer[wifi_index+7]==201))	
				{
					if ((cur_address < 28611)&&(!Read1min))
					{
						if (cur_address > 0)
						{
							read_dataflash (0xc0078002+cur_address, 28611-cur_address, (char*)(&BlackBox[0]));	
							read_dataflash (0xc0078002           ,       cur_address, (char*)(&BlackBox[28611-cur_address]));	
						}
						else
							read_dataflash (0xc0078002+cur_address, 28611-cur_address, (char*)(&BlackBox[0]));	
						Read1min=1;
						count_fs = 0;
					}							
				}
				for (i=wifi_index;i<wifi_end-30;i++)
					pwifi_buffer[i] = pwifi_buffer[i+30];
				wifi_end -= 30;							
				goto wifi_loop;
			}

			wifi_index ++;
			goto wifi_loop;

		end_wifi_loop:;

	return;

	} 

}	


void IO_init(void)
{
	AT91C_BASE_PIOC->PIO_PER = 0x0000003c;
	AT91C_BASE_PIOC->PIO_OER = 0x00000038;		 
	AT91F_PIO_CfgPeriph(
		AT91C_BASE_PIOB, // PIO controller base address
		((unsigned int) AT91C_PB29_IRQ0     ), // Peripheral A
		0); // Peripheral B 

	AT91F_PMC_EnablePeriphClock(
		AT91C_BASE_PMC, // \arg pointer to PMC controller
		(((unsigned int) 1 << AT91C_ID_PIOC) | 
		((unsigned int) 1 << AT91C_ID_PIOB) |
		((unsigned int) 1 << AT91C_PB29_IRQ0) |
		((unsigned int) 1 << AT91C_ID_PIOA)) );  // \arg IDs of peripherals to enable  

	AT91C_BASE_PIOB->PIO_ODR =1 << 29; 
}

void wait(unsigned int nTick)
{
	unsigned int nloop = 0;
	while(nloop++ < nTick)	;		
}

//Interrupt handler. Gets called by FPGA every 20ms.
void AT91F_IRQ0_HANDLER(void)		//这个是最重要的中断函数，由fpga每20ms产生一个低电平触发arm产生一次这个中断
{
	int ytemp,i,kk,len;
	float tout,t14;
	int iTargetRoll,iTargetPitch;
	float max,y19,y13;
	float youxiao_elev;
	unsigned char ctread;
	int rot_spd_dian = 0;

	float ftemperature_low=5,ftemperature_high = 50;

	unsigned int rot_spdh=0,rot_spdl=0,rot_spdt=0;

	float full;
	int sumall;
	float r2f1,p2f1;

	float wcr,wsr,wcp,wsp;
	float a,b,c,d;
	float ab,ac,ad,bb,bc,bd,cc,cd,dd;
	float wcbn11,wcbn12,wcbn13,wcbn21,wcbn22,wcbn23,wcbn31,wcbn32,wcbn33;

	float ddx,ddy;

	int j;
	double yhpres;
	float ailhpres;

	int roll,pitch;


	if (countplotmag>0)
		countplotmag--;

	docalheight++;
	count_twi++;

	if (in_irq0==1) 
	{
		return;
	}

	if (errorflash==1)	return;		//Error while reading flash. Needs to be restarted.
	if (endinit==0) return;			//Initiailization isn't complete yet.

	in_irq0 = 1;

	//Currently in waypoint mode
	//if(Enable_Waypoint)&&(in_flight)&&(start_hover)
	if ((Channel_6 > 1333)&&(Channel_6<1666)&&(Enable_Waypoint)&&(Channel_5>1666)&&(in_flight)&&(start_hover))			
	{
		hover_time --;    
		//If a point has been reached, and the copter has hovered for the required time		
		if (hover_time <= 0)
		{
		
			if (tgt_hover_num[nTrkPt]>24)
				HaveCalculateArrive = 1;

			nTrkPt++;

			if (nTrkPt >= Waypoint_number)		//If all the waypoints have been visited, go back to the take-off spot
			{
				nTrkPt = 0;
			}

			OUT_ARM_TARGET = nTrkPt;	//Send task point to GCS

			tgtLatti_s = tgtLatti;
			tgtLongi_s = tgtLongi;

			tgtLatti = tgt_Latti[nTrkPt];
			tgtLongi = tgt_Longi[nTrkPt];

			start_hover = 0;
			hover_time = 0;
		}
		else
		{

			if (tgt_High[nTrkPt]!=9999)
			{
				tgtHigh = tgt_High[nTrkPt]*10;
				AirHigh_h = (((int)tgtHigh) >> 8);
				AirHigh_l = ((int)tgtHigh) & 0xff;
			}

		}
	}

	cos_hx = cos(scalheading);
	sin_hx = sin(scalheading);

	if (first_ == 0)
	{
		*pmem_fpga9t = 3;
		first_ = 1;
	}
	else if (first_ == 1)
	{
		*pmem_fpga9t = 0;
		first_ = 2;
	}
	else if (first_ == 2)
	{
		*pmem_fpga9t = 2;
		first_ = 3;
	}
	else if (first_ == 3)
	{
		*pmem_fpga9t = 0;
		first_ = 4;
	}
	else
		*pmem_fpga9t = 1;

	SndBuf[GIMBAL_RADIUS] = (unsigned char)  ((int)gimbal_target_R);

	
	//Check for GPS errors
	if (run_time > 5)
		for(i=0;i<4;i++)
			GPS_Refresh[i] ++;

	if (((GPS_Refresh[0]<hz250)&&(GPS_Refresh[1]<hz250)&&(GPS_Refresh[2]<hz250)&&(GPS_Refresh[3]<hz250)))
		GPS_Refresh_error = 0;
	else
		GPS_Refresh_error = 1;

	if (GPS_Refresh_error==1)
		ekff_error = 1;

	count_show_serial++;

	
	if (count_show_serial < 3*hz250)
	{
		show_serial = 0;
	}
	else
		if (count_show_serial < 4*hz250)
			show_serial = 1;
		else
		{
			count_show_serial = 0;
		}

		if ((GPS_Refresh_error==1)&&(run_time>4))
		{
			gpsvalidFlag++;
			if (gpsvalidFlag>=(hz250/4))
			{
				gpsvalidFlag =0;

				press_val = 0;
				for (i=0;i<ctair;i++)
				{
					press_val += press_data[i];
				}

				if (ctair > 0)
				{
					press_val /= (double)ctair;
					clear_pres_val = press_val;
					ctair = 0;
					yhpres = press_val/100/Pres0;

					if (yhpres > 0)
						hh = 0.190263107*log(yhpres);

					tAirHighf = -1 * (alt0 - (1-exp(hh))*44330.7694);
				}

				ctlb++;
				if (ctlb >= 2)
				{
					ctlb = 0;
					alldata.gps_velD = cur_pres2 - tAirHighf;
					if (alldata.gps_velD == alldata.gps_velD)
						;
					else
						alldata.gps_velD = 0;

					cur_pres2 = cur_pres1;
					cur_pres1 = tAirHighf;

					if (satelnumflag)
						VelFilter = 1;


					alldata.gps_valid = 1;
					VelFilter = 1;

				}

			}
		}


		if (startled)
		{
			if ((vforce < low_power)&&(in_flight))
			{
				colors[0] = 11;		//Red
				colors[1] = 11;		//Red
				colors[2] = 11;		//Red
				colors[3] = 11;
				colors[4] = 11;
			}

			if (outmagdata==1)
			{
				*pmem_bus1 = 3;				//Long Violet
			}
			else
				if (Parameter[ERASE_ROMBOOT]==87)
				{
					if ((fabsf(alldata.true_roll)<3)&&(fabsf(alldata.true_pitch)<3))
						*pmem_bus1 = 2;			//Long Blue
					else
						*pmem_bus1 = 0;
				}
				else if (Parameter[ERASE_ROMBOOT]==88)
				{
					if ((fabsf(alldata.true_roll)<3)&&(fabsf(alldata.true_pitch)<3))
						*pmem_bus1 = 6;			//亮色长亮
					else
						*pmem_bus1 = 0;
				}
				else if (ekff_error)
				{
					*pmem_bus1 = 7;				//Long White
				}	
				else if (pres_error)
				{
					*pmem_bus1 = 1;				//Long Red
				}
				else if ((vforce < emergency_power)&&(in_flight))	//low power
				{
					*pmem_bus1 = 1;				//Long Red
				}
				else	
				{	
					ctled++;
					if (colors[color_point] < 10)
						led_flash_hz = 3;
					else
						led_flash_hz = 4.5;


					if (!((vforce < low_power)&&(in_flight)))
						if (color_point==0) 
							led_flash_hz = 1;

					if (ctled>= ((float)hz250/led_flash_hz))
					{
						ctled = 0;

						led_color = colors[color_point];
						if (led_color > 9) 
							led_color -= 10;
						* pmem_bus1 = led_color;
						color_point ++;
						if (color_point>=5) color_point = 0;		//0、1、2、3、4
						start_led_on = 1;
					}		

					if (start_led_on==1)
					{
						ct_led_on ++;
						if (ct_led_on >= (hz250/10))
						{
							ct_led_on = 0;
							start_led_on = 0;
							*pmem_bus1 = 0;
						}
					}
				}
		}

		if (enable_ctclearpres==1)
			ctclearpres ++;


		ct4++;
		if (ct4 >= hz250/10)
		{
			ct4 = 0;
			mainpres = 1;
		}

		ctmag ++;
		if (ctmag>=hz250/50)
		{
			ctmag = 0;		
			if (run_time >= 2)
			{	
				*pmem_maggo = 1;
			}
			else
			{
				*pmem_maggo = 2;
			}
		}


		ctmag3 ++;
		if (ctmag3 >= hz250/20)
		{
			ctmag3 = 0;
			mainmag = 1;
			ctmag2++;
			if (ctmag2 >= 1)
			{
				if (Manual_Mode==0)
					MagFilter = 1;
				ctmag2 = 0;
			}
		}


		ctirq = 0;
		ADI_CS = 0;
		data1[ctirq] = (((unsigned char)*pmem_fpgadata13)<<16) + (((unsigned char)*pmem_fpgadata12)<<8) + (unsigned char)*pmem_fpgadata11;
		data2[ctirq] = (((unsigned char)*pmem_fpgadata23)<<16) + (((unsigned char)*pmem_fpgadata22)<<8) + (unsigned char)*pmem_fpgadata21;
		data3[ctirq] = (((unsigned char)*pmem_fpgadata33)<<16) + (((unsigned char)*pmem_fpgadata32)<<8) + (unsigned char)*pmem_fpgadata31;
		data4[ctirq] = (((unsigned char)*pmem_fpgadata43)<<16) + (((unsigned char)*pmem_fpgadata42)<<8) + (unsigned char)*pmem_fpgadata41;
		data5[ctirq] = (((unsigned char)*pmem_fpgadata53)<<16) + (((unsigned char)*pmem_fpgadata52)<<8) + (unsigned char)*pmem_fpgadata51;
		data6[ctirq] = (((unsigned char)*pmem_fpgadata63)<<16) + (((unsigned char)*pmem_fpgadata62)<<8) + (unsigned char)*pmem_fpgadata61;
		data7[ctirq] = (((unsigned char)*pmem_fpgadata73)<<16) + (((unsigned char)*pmem_fpgadata72)<<8) + (unsigned char)*pmem_fpgadata71;
		data8[ctirq] = (((unsigned char)*pmem_fpgadata83)<<16) + (((unsigned char)*pmem_fpgadata82)<<8) + (unsigned char)*pmem_fpgadata81;
		ADI_CS = 1;
		data1a = data1[0];
		data2a = data2[0];
		data3a = data3[0];
		data4a = data4[0];
		data5a = data5[0];
		data6a = data6[0];
		data7a = data7[0];
		data8a = data8[0];


		if (clear_bias==1)
			clear_bias_time++;

		*pmem_maggo = 3;

		if (run_time > 2)
			satelnumflag = 1;

		if (run_time > 1)
		{

			rd_adi1256();
			put_data1256();					//save gyro and accel data

			if ((initflag == 0)&&(satelnumflag))
			{
				vel_down_clear = 1;
				InitData();					//Initialize state calculation function and other data
				initflag = 1;
			}
			if (satelnumflag)				//If GPS signal is strong enough, start state calculation 
			{
				TimerINTCall();				
			}

			if (showmag==0)
				scalheading = alldata.true_yaw*PI/180;
			else
				scalheading = cal_heading;
			if (scalheading > PI) scalheading -= 2*PI;
			if (scalheading < -PI) scalheading += 2*PI;

		
			if (Manual_Mode) scalheading = 0;

			if (Parameter[ERASE_ROMBOOT]!=90)
				float2buf(&OUT_INA_HANGXIANG,&scalheading);

		}




		t_PlaneRoll = -alldata.true_roll;	//The calculated true roll angle
		t_PlanePitch = alldata.true_pitch;	//The calculated true pitch angle



		PlaneRoll = t_PlaneRoll;		//Remove installation errors
		PlanePitch = t_PlanePitch;	//Remove installation errors



		count_AD++;
		count_current++;
		count_SPI++;





		BatteryConsume += (CurrentI/hz250);				//Unit: Ampere-second



		SndBuf[POWER_CONSUMPTION_2] = (((int)BatteryConsumeI)>>8);
		SndBuf[POWER_CONSUMPTION_1] = (int)BatteryConsumeI & 0xff;





		time_count++;
		if (time_count>=hz250) 
		{
			time_count = 0;
			run_time ++;		//Run time in seconds
			if (in_flight)	
				fly_time++;		//Flight time in seconds
		}

		SndBuf[FLIGHT_TIME_2] = fly_time >> 8;
		SndBuf[FLIGHT_TIME_1] = fly_time & 0xff;

		if (fly_time > Parameter[ROLL_P]*60)
		{
			fly_timeup = 1;
		}
		else
			fly_timeup = 0;


		ctspi++;	//计算rdspipara是否超过固定时间还没有返回的计数器

		sbus_select = Parameter[SELECTION_OPTIONS]&0x3;

	
		//start receiver
		ctreceiver++;
		if (ctreceiver >= hz250/50)
		{
			ctreceiver = 0;

			//yjjxf		

			if ((run_time < 3)&&(sbus_select==0))
			{
				tChannel_Rudder = (((unsigned char)*pmem_width5h)<<8)+(unsigned char)*pmem_width5l;
				tChannel_Aileron = (((unsigned char)*pmem_fpga10h)<<8)+(unsigned char)*pmem_fpga10l;
				tChannel_Elevator = (((unsigned char)*pmem_fpga11h)<<8)+(unsigned char)*pmem_fpga11l;
				tChannel_Throttle = (((unsigned char)*pmem_width4h)<<8)+(unsigned char)*pmem_width4l;
				if ((tChannel_Rudder>1000)&&(tChannel_Rudder<2000)&&(tChannel_Aileron>1000)&&(tChannel_Aileron<2000)&&(tChannel_Elevator>1000)&&(tChannel_Elevator<2000)&&(tChannel_Throttle>700)&&(tChannel_Throttle<2300))
					ppmmode = 1;
				else
					ppmmode = 0;
			}

			if (((ppmmode==1)&&(sbus_select==0))||(sbus_select==3))
			{
				tChannel_5 = (((unsigned char)*pmem_width6h)<<8)+(unsigned char)*pmem_width6l;
				Channel_5 = tChannel_5;


				if (wifi_data>0)	wifi_data--;

				if ((wifi_data == 0) || (Channel_5 < 1666))
				{

					tChannel_Rudder = (((unsigned char)*pmem_width5h)<<8)+(unsigned char)*pmem_width5l;
					tChannel_Aileron = (((unsigned char)*pmem_fpga10h)<<8)+(unsigned char)*pmem_fpga10l;
					tChannel_Elevator = (((unsigned char)*pmem_fpga11h)<<8)+(unsigned char)*pmem_fpga11l;
					tChannel_Throttle = (((unsigned char)*pmem_width4h)<<8)+(unsigned char)*pmem_width4l;
					if (adjust_throt)
					{
						count_adjust ++;
						if (count_adjust >= hz50*5)
						{
							count_adjust = 0;
							adjust_throt = 0;

							Throttle_Factor = 830/(float)(throtMAX - throtMIN);

							Rudder_Factor = 800 / (float)(rudMAX - rudMIN);
							Aileron_Factor = 800 / (float)(ailMAX - ailMIN);
							Elevator_Factor = 800 / (float)(elevMAX - elevMIN);

							
							Rudder_MID = 1500;
							Aileron_MID = 1500;
							Elevator_MID = 1500;

							//Fix the calculated neutral values
							OUT_ARM_FXZHONG = Rudder_MID/10; 
							OUT_ARM_FYZHONG = Aileron_MID/10; 
							OUT_ARM_SJZHONG = Elevator_MID/10;

							TargetRollBias = 0;
							TargetPitchBias = 0;

							writebuf1 = 1;
						}
						else
						{
							if (tChannel_Throttle > throtMAX) throtMAX = tChannel_Throttle;
							if (tChannel_Throttle < throtMIN) throtMIN = tChannel_Throttle;

							if (tChannel_Rudder > rudMAX) rudMAX = tChannel_Rudder;
							if (tChannel_Rudder < rudMIN) rudMIN = tChannel_Rudder;

							if (tChannel_Aileron > ailMAX) ailMAX = tChannel_Aileron;
							if (tChannel_Aileron < ailMIN) ailMIN = tChannel_Aileron;

							if (tChannel_Elevator > elevMAX) elevMAX = tChannel_Elevator;
							if (tChannel_Elevator < elevMIN) elevMIN = tChannel_Elevator;

						}
					}
					else	//Calibrate throttle channel
					{
						//1100~1930		7~90
						tChannel_Throttle = (float)(tChannel_Throttle - throtMIN)*Throttle_Factor + 1100;

						tChannel_Rudder = (float)(tChannel_Rudder - rudMIN)*Rudder_Factor + 1100;
						tChannel_Aileron = (float)(tChannel_Aileron - ailMIN)*Aileron_Factor + 1100;
						tChannel_Elevator = (float)(tChannel_Elevator - elevMIN)*Elevator_Factor + 1100;

					}
				}
				else
				{
					tChannel_Rudder = Rudder_wifi;
					tChannel_Aileron = Aileron_wifi;
					tChannel_Elevator = Elevator_wifi;
					tChannel_Throttle = Throttle_wifi;
				}

				Channel_6 = (((unsigned char)*pmem_width7h)<<8)+(unsigned char)*pmem_width7l;
				yChannel_6 = Channel_6;

				fpgain7 = (((unsigned char)*pmem_width8h)<<8)+(unsigned char)*pmem_width8l;
				fpgain8 = (((unsigned char)*pmem_width9h)<<8)+(unsigned char)*pmem_width9l;
				fpgain9 = 1500;

			}
			else if (((sbus_error==1)||(sbus_select==1))&&(sbus_select!=2))		
					//		if (0)
					//		
				{



					tChannel_5 = (((unsigned char)*pmem_fpga1h)<<8)+(unsigned char)*pmem_fpga1l;


					Channel_5 = tChannel_5;


					if (wifi_data>0)	wifi_data--;

					if ((wifi_data == 0) || (Channel_5 < 1666))
					{

						tChannel_Rudder = (((unsigned char)*pmem_FANGXIANGh)<<8)+(unsigned char)*pmem_FANGXIANGl;
						tChannel_Aileron = (((unsigned char)*pmem_FUYIh)<<8)+(unsigned char)*pmem_FUYIl;
						tChannel_Elevator = (((unsigned char)*pmem_SHENGJIANGh)<<8)+(unsigned char)*pmem_SHENGJIANGl;
						tChannel_Throttle = (((unsigned char)*pmem_throttleh)<<8)+(unsigned char)*pmem_throttlel;



						if (adjust_throt)
						{
							count_adjust ++;
							if (count_adjust >= hz50*5)
							{
								count_adjust = 0;
								adjust_throt = 0;

								Throttle_Factor = 830/(float)(throtMAX - throtMIN);

								Rudder_Factor = 800 / (float)(rudMAX - rudMIN);
								Aileron_Factor = 800 / (float)(ailMAX - ailMIN);
								Elevator_Factor = 800 / (float)(elevMAX - elevMIN);

			
								Rudder_MID = 1500;
								Aileron_MID = 1500;
								Elevator_MID = 1500;

								//Fix the calculated neutral values
								OUT_ARM_FXZHONG = Rudder_MID/10; //定下计算出的副翼中立位值
								OUT_ARM_FYZHONG = Aileron_MID/10; //定下计算出的方向舵中立位值
								OUT_ARM_SJZHONG = Elevator_MID/10;

								TargetRollBias = 0;
								TargetPitchBias = 0;

								writebuf1 = 1;
							}
							else
							{
								if (tChannel_Throttle > throtMAX) throtMAX = tChannel_Throttle;
								if (tChannel_Throttle < throtMIN) throtMIN = tChannel_Throttle;

								if (tChannel_Rudder > rudMAX) rudMAX = tChannel_Rudder;
								if (tChannel_Rudder < rudMIN) rudMIN = tChannel_Rudder;

								if (tChannel_Aileron > ailMAX) ailMAX = tChannel_Aileron;
								if (tChannel_Aileron < ailMIN) ailMIN = tChannel_Aileron;

								if (tChannel_Elevator > elevMAX) elevMAX = tChannel_Elevator;
								if (tChannel_Elevator < elevMIN) elevMIN = tChannel_Elevator;

							}
						}
						else	//Calibrate throttle channel
						{
							//1100~1930		7~90
							tChannel_Throttle = (float)(tChannel_Throttle - throtMIN)*Throttle_Factor + 1100;

							tChannel_Rudder = (float)(tChannel_Rudder - rudMIN)*Rudder_Factor + 1100;
							tChannel_Aileron = (float)(tChannel_Aileron - ailMIN)*Aileron_Factor + 1100;
							tChannel_Elevator = (float)(tChannel_Elevator - elevMIN)*Elevator_Factor + 1100;

						}
					}
					else
					{
						tChannel_Rudder = Rudder_wifi;
						tChannel_Aileron = Aileron_wifi;
						tChannel_Elevator = Elevator_wifi;
						tChannel_Throttle = Throttle_wifi;
					}

					Channel_6 = (((unsigned char)*pmem_fpga2h)<<8)+(unsigned char)*pmem_fpga2l;

					yChannel_6 = Channel_6;

					fpgain7 = (((unsigned char)*pmem_kaisanh)<<8)+(unsigned char)*pmem_kaisanl;
					fpgain8 = (((unsigned char)*pmem_fpga8h)<<8)+(unsigned char)*pmem_fpga8l;
					fpgain9 = (((unsigned char)*pmem_fpga9h)<<8)+(unsigned char)*pmem_fpga9l;
				}
				else
				{
					*pmem_bus33 = 0;
					cbus1 = *pmem_bus1;
					cbus2 = *pmem_bus2;
					cbus3 = *pmem_bus3;
					cbus4 = *pmem_bus4;
					cbus5 = *pmem_bus5;
					cbus6 = *pmem_bus6;
					cbus7 = *pmem_bus7;
					cbus8 = *pmem_bus8;
					cbus9 = *pmem_bus9;
					cbus10 = *pmem_bus10;
					cbus11 = *pmem_bus11;
					cbus12 = *pmem_bus12;
					cbus13 = *pmem_bus13;
					cbus14 = *pmem_bus14;
					cbus15 = *pmem_bus15;
					cbus16 = *pmem_bus16;
					cbus17 = *pmem_bus17;
					cbus18 = *pmem_bus18;
					cbus19 = *pmem_bus19;
					cbus20 = *pmem_bus20;
					cbus21 = *pmem_bus21;
					*pmem_bus33 = 1;

					if ((unsigned char)cbus1 == 248)
					{
						ctsbus_error = 0;

						ibus = ((cbus2 & 0xf8)>>3) + ((cbus3&0x7)<<5) +((cbus3&0x80)<<1) + ((cbus4&0x3)<<9);
						fbus1 = 2047 - ibus;

						ibus = ((cbus4 & 0x7c)>>2) + ((cbus5&0xf8)<<2) +((cbus6&0x01)<<10);
						fbus2 = 2047 - ibus;

						ibus = ((cbus6 & 0x6)>>1) + ((cbus6&0x80)>>5) +((cbus7&0x7f)<<3) + ((cbus8&0x8)<<7);
						fbus3 = 2047 - ibus;

						ibus = ((cbus8 & 0xf0)>>4) + ((cbus9&0x07)<<4) +((cbus9&0x80)) + ((cbus10&0x7)<<8);
						fbus4 = 2047 - ibus;

						ibus = ((cbus10 & 0x78)>>3) + ((cbus11&0xf8)<<1) +((cbus12&0x3)<<9);
						fbus5 = 2047 - ibus;

						ibus = ((cbus12 & 0x4)>>2) + ((cbus12&0x80)>>6) +((cbus13&0x7f)<<2) + ((cbus14&0x18)<<6);
						fbus6 = 2047 - ibus;

						ibus = ((cbus14 & 0xe0)>>5) + ((cbus15&0x07)<<3) +((cbus15&0x80)>>1) + ((cbus16&0x0f)<<7);
						fbus7 = 2047 - ibus;

						ibus = ((cbus16 & 0x70)>>4) + ((cbus17&0xf8)) +((cbus18&0x07)<<8);
						fbus8 = 2047 - ibus;

						ibus = ((cbus18 & 0x80)>>7) + ((cbus19&0x07f)<<1) +((cbus20&0x38)<<8);
						fbus9 = 2047 - ibus;
					}
					else
					{
						ctsbus_error++;
						if ((ctsbus_error >= hz50*0.2)&&(run_time<2)&&(sbus_select==0))
						{
							ctsbus_error = 0;
							sbus_error = 1;
						}
					}


					tChannel_5 = 1520 + (fbus5 - 1024)*sbus_yinzi;
					Channel_5 = tChannel_5;


					if (wifi_data>0)	wifi_data--;

					if ((wifi_data == 0) || (Channel_5 < 1666))
					{
						tChannel_Rudder = 1520 + (fbus4 - 1024)*sbus_yinzi;
						tChannel_Aileron = 1520 + (fbus1 - 1024)*sbus_yinzi;
						tChannel_Elevator = 1520 + (fbus2 - 1024)*sbus_yinzi;
						tChannel_Throttle = 1520 + (fbus3 - 1024)*sbus_yinzi;
						if (adjust_throt)
						{
							count_adjust ++;
							if (count_adjust >= hz50*5)
							{



								count_adjust = 0;
								adjust_throt = 0;

								Rudder_Factor = 800 / (float)(rudMAX - rudMIN);
								Aileron_Factor = 800 / (float)(ailMAX - ailMIN);
								Elevator_Factor = 800 / (float)(elevMAX - elevMIN);

								Rudder_MID = 1500;
								Aileron_MID = 1500;
								Elevator_MID = 1500;

								//Fix the calculated neutral positions
								OUT_ARM_FXZHONG = Rudder_MID/10; 
								OUT_ARM_FYZHONG = Aileron_MID/10;
								OUT_ARM_SJZHONG = Elevator_MID/10;

								Throttle_Factor = 830/(float)(throtMAX - throtMIN);


								TargetRollBias = 0;
								TargetPitchBias = 0;

								writebuf1 = 1;
							}
							else
							{
								if (tChannel_Throttle > throtMAX) throtMAX = tChannel_Throttle;
								if (tChannel_Throttle < throtMIN) throtMIN = tChannel_Throttle;

								if (tChannel_Rudder > rudMAX) rudMAX = tChannel_Rudder;
								if (tChannel_Rudder < rudMIN) rudMIN = tChannel_Rudder;

								if (tChannel_Aileron > ailMAX) ailMAX = tChannel_Aileron;
								if (tChannel_Aileron < ailMIN) ailMIN = tChannel_Aileron;

								if (tChannel_Elevator > elevMAX) elevMAX = tChannel_Elevator;
								if (tChannel_Elevator < elevMIN) elevMIN = tChannel_Elevator;
							}
						}
						else	//Calibrate throttle channel
						{
							//1100~1930		7~90
							tChannel_Throttle = (float)(tChannel_Throttle - throtMIN)*Throttle_Factor + 1100;

							tChannel_Rudder = (float)(tChannel_Rudder - rudMIN)*Rudder_Factor + 1100;
							tChannel_Aileron = (float)(tChannel_Aileron - ailMIN)*Aileron_Factor + 1100;
							tChannel_Elevator = (float)(tChannel_Elevator - elevMIN)*Elevator_Factor + 1100;

						}

					}
					else
					{
						tChannel_Rudder = Rudder_wifi;
						tChannel_Aileron = Aileron_wifi;
						tChannel_Elevator = Elevator_wifi;
						tChannel_Throttle = Throttle_wifi;
					}

					Channel_6 = 1520 + (fbus6 - 1024)*sbus_yinzi;
					yChannel_6 = Channel_6;


					fpgain7 = 1520 + (fbus7 - 1024)*sbus_yinzi;
					fpgain8 = 1520 + (fbus8 - 1024)*sbus_yinzi;
					fpgain9 = 1520 + (fbus9 - 1024)*sbus_yinzi;


				}

				if ((tChannel_Rudder>700)&&(tChannel_Rudder<2300))
					Channel_Rudder = tChannel_Rudder;
				if ((tChannel_Aileron>700)&&(tChannel_Aileron<2300))
					Channel_Aileron = tChannel_Aileron;
				if ((tChannel_Elevator>700)&&(tChannel_Elevator<2300))
					Channel_Elevator = tChannel_Elevator;
				if ((tChannel_Throttle>700)&&(tChannel_Throttle<2300))
					Channel_Throttle = tChannel_Throttle;




				if ((Channel_5 > 1666)&&(Channel_6 > 1666)&&(in_flight)) 
				{
					ctgohome++;
					if (ctgohome > 5*50)									//Only after 3 seconds, it starts going back
					{
						ctgohome = 5*50;
						Channel_6 = 2000;

						auto_stay_ding = 1;

						if (Enable_Waypoint==1)
						{
							Enable_Waypoint = 0;
						}

					}
					else
						Channel_6 = 1500;
				}
				else
				{
					ctgohome = 0;
				}



		}
		//end receiver


		if (Channel_6 < 1333) 
		{	
			if (Enable_Waypoint==1)
			{
				Enable_Waypoint = 0;
			}
		}

		if (run_time > 3)
			startcontrol = 1;


		if (run_time<2)
		{
			Channel_5 = 1000;
			Channel_6 = 1000;
			Channel_Rudder = 1500;
			Channel_Aileron = 1500;
			Channel_Elevator = 1500;
			Channel_Throttle = 1930;
			fpgain7 = 1500;
			fpgain8 = 1500;
			fpgain9 = 1500;
		}




		if (Channel_5 < 1333)
		{
			protect = 0;
			reset_throt = 0;
		}

		if (Channel_5 > 1333)
		{
			if (protect==0)
			{	

				throtMid = Channel_Throttle;	


				if (in_flight)
				{
					tgtHigh = fAirHigh*10;

					if (tgtHigh < 0)
						th = tgtHigh + 65536;
					else
						th = tgtHigh;

					AirHigh_h = (((int)th) >> 8);
					AirHigh_l = (int)th & 0xff;
				}
			}
			protect = 1;
		}

		if (reset_throt)
		{
			if (Channel_Throttle < Low_Throttle)
				reset_throt = 0;
		}


		FX = Channel_Rudder; //yaw
		ail = Channel_Aileron;
		elev = Channel_Elevator;
		THROT = Channel_Throttle;


		if ((reset_throt)&&(autotakeoff))
			THROT = 1550;


		if (Channel_5 < 1333)
			ch7_status = 0;


		if ((Channel_5 < 1666)||(satelnum < VALID_GPSNUM))	//manual
		{

			if (Enable_Waypoint==1)
			{
				Enable_Waypoint = 0;
			}

			carefree = 0;
			ch5_status = 0;
			close = 0;
			sumx = 0;
			sumy = 0;
			endland = 0;
			gohome = 0;
			pregohome = 0;
			gohome_end = 0;
			autotakeoff = 0;
			ClickSetTgtFlag	= 0;
			gimbal_target_lock = 0;


			if (bAutoHand_)						//Switched to manual
			{
				Manual_Mode = 1;

				alldata.gps_latti = 40;
				alldata.gps_longi = 112;

				alldata.gps_velN = 0;
				alldata.gps_velE = 0;

				vel_down_clear = 0;

				InitData();					//Initialize state calculation and other data
			}

			bAutoHand_ = 0;

			if (Channel_5 < 1333)
			{
				colors[3] = 0;		//nothing
				colors[4] = 0;		//nothing
			}
			else
			{
				if ((Channel_Throttle<High_Throttle)||(Channel_Throttle>Low_Throttle))		//Not in the neutral position
				{
					colors[3] = 2;		//blue
					colors[4] = 0;		//nothing
				}
				else
				{
					colors[3] = 12;		//blue, blink twice
					colors[4] = 12;		//blue, blink twice
				}
			}
		}
		else
		{


			if ((fabsf(elev-Elevator_MID)>30)||(fabsf(ail-Aileron_MID)>30)||(Channel_Throttle<High_Throttle)||(Channel_Throttle>Low_Throttle))		//Not in the neutral position
			{
				colors[3] = 4;		//Green
				colors[4] = 0;		//nothing
			}
			else
			{
				colors[3] = 14;		//Green, blink twice
				colors[4] = 14;		//Green, blink twice
			}

			if (!bAutoHand_)		//Switched to autonavigation
			{


				if (satelnum>=VALID_GPSNUM)		//If the GPS signal is strong enough
				{

					Manual_Mode = 0;

					alldata.gps_latti = CurLatti;
					alldata.gps_longi = CurLongi;

					

					alldata.gps_velN = (float)tvely_i/100;	//North Velocity
					alldata.gps_velE = (float)tvelx_i/100;	//East Velocity

					vel_down_clear = 0;
					InitData();					//Initialize state calculation and other data
				}

				scalheading = cal_heading;
				targetYaw = scalheading*180/PI;			//Set current heading direction as the target heading direction
				ref_yaw = scalheading;


			}
			bAutoHand_ = 1;



			if (Channel_6 > 1666)		//Auto land
				auto_stay_ding = 1;

			else



				if ((Channel_6 > 1333)&&(Channel_6<1666)&&(Enable_Waypoint))
				{

					auto_stay_ding = 0;		//Waypoint mode
					if (ch5_status!=1)
					{
						tgtLatti_s = CurLatti;
						tgtLongi_s = CurLongi;

						tgtHigh = fAirHigh*10;

						if (tgtHigh < 0)
							th = tgtHigh + 65536;
						else
							th = tgtHigh;

						AirHigh_h = (((int)th) >> 8);
						AirHigh_l = (int)th & 0xff;
					}
					ch5_status = 1;

				}
				else
				{
					auto_stay_ding = 1;		//现在的遥控器手推导航模式 ??
					if (ch5_status!=2)
					{
						tgtLatti_s = CurLatti;
						tgtLongi_s = CurLongi;
						tgtLatti = CurLatti;
						tgtLongi = CurLongi;
						target_caled = 1;
						ch5_status = 2;
						vel_caled = 0;
					}
				}
		}


		OUT_HAND_FXDUO = FX/10;
		OUT_HAND_FYDUO = ail/10; 
		OUT_HAND_SJDUO = elev/10;
		OUT_HAND_YMDUO = THROT/10; 


		r2f1 = alldata.true_roll * rad / 2;
		p2f1 = alldata.true_pitch * rad / 2;

		wcr = cos(r2f1);
		wsr = sin(r2f1);
		wcp = cos(p2f1);
		wsp = sin(p2f1);


		a = wcr*wcp;
		b = wsr*wcp;
		c = wcr*wsp;
		d = - wsr*wsp;

		ab=a*b;
		ac=a*c;
		ad=a*d;
		bb=b*b;
		bc=b*c;
		bd=b*d;
		cc=c*c;
		cd=c*d;
		dd=d*d;

		wcbn11 = 1- 2*(cc+dd);
		wcbn12 = 2*(bc-ad);
		wcbn13 = 2*(bd+ac);
		wcbn21 = 2*(bc+ad);
		wcbn22 = 1-2*(bb+dd);
		wcbn23 = 2*(cd-ab);
		wcbn31 = 2*(bd-ac);
		wcbn32 = 2*(cd+ab);
		wcbn33 = 1-2*(bb+cc);

		acc_elev = (wcbn11*alldata.ina_acc2+wcbn12*alldata.ina_acc1+wcbn13*(-alldata.ina_acc3))/g0;
		acc_ail = (wcbn21*alldata.ina_acc2+wcbn22*alldata.ina_acc1+wcbn23*(-alldata.ina_acc3))/g0;
		acc_dn1 = (wcbn31*alldata.ina_acc2+wcbn32*alldata.ina_acc1+wcbn33*(-alldata.ina_acc3-xEKF[12]))/g0 + 1;


		bacc_dn2 = Butterworth(acc_dn1, zaccdn);
		acc_dn2 = bacc_dn2;



		if (!bAutoHand_) //Manual
		{

			OUT_ATL_FYDUO = ail/10;  //Input and output are exactly the same
			OUT_ATL_SJDUO = elev/10;  

			if (Channel_5<1333)
				OUT_ARM_AUTOHANDLEBF = 0;  //Set control status to "Manual"
			else
				OUT_ARM_AUTOHANDLEBF = 0x0f;	//Manually set height

			cal1 = 0;
			cal2 = 0;
			cal3 = 0;
			cal4 = 0;
			cal5 = 0;
			cal6 = 0;
			cal7 = 0;
			cal8 = 0;

			if (enter_set!=1)
			{
				if (run_time > 2)
				{


					CalFxWidth();
					CalailWidth();
					CalelevWidth();
					CalthrotWidth();
				}
				else
				{
					nCalthrotWidth = 1930;
					OUT_ATL_YMDUO = nCalthrotWidth/10;
				}
			}
			else
			{
				nCalFxWidth = 0;
				nCalailWidth = 0;
				nCalelevWidth = 0;

				nCalthrotWidth = THROT;
				OUT_ATL_YMDUO = nCalthrotWidth/10;
			}


		}
		else	//Completely autonomous mode
		{



			if (ClickSetTgtFlag==1)
				OUT_ARM_AUTOHANDLEBF = 0x04;			//Point and fly
			else
				if (auto_stay_ding==0)
					OUT_ARM_AUTOHANDLEBF = 0x2;  
				else
					OUT_ARM_AUTOHANDLEBF = 0x1;

			doCal();


			docal_TargetRoll = tdocal_TargetRoll;
			docal_TargetPitch = tdocal_TargetPitch;

			cal1 = 0;
			cal2 = 0;
			cal3 = 0;
			cal4 = 0;
			cal5 = 0;
			cal6 = 0;
			cal7 = 0;
			cal8 = 0;

			if (run_time > 2)
			{


				CalFxWidth();
				CalailWidth();
				CalelevWidth();
				CalthrotWidth();
			}
		} 



		sum_anglex = TargetRollMid * cos_hx + TargetPitchMid * sin_hx;
		sum_angley = - TargetRollMid * sin_hx + TargetPitchMid * cos_hx;
		if ((!in_flight)&&(!start_center_gravity))
		{
			sum_anglex = 0;
			sum_angley = 0;
		}






		if (in_flight==0)
		{
			if (Channel_5 > 1666)
			{
				if (Channel_6 > 1666)
					OUT_ARM_AUTOHANDLEBF = 0xb;
				else
					if (((Channel_6 > 1333)&&(Enable_Waypoint))||( (Channel_6>1333)&&(Channel_Throttle>1900)   ))
						OUT_ARM_AUTOHANDLEBF = 0x2;
					else
						OUT_ARM_AUTOHANDLEBF = 0x1;
			}									
		}

	

		if (DroneMode==4)
		{
			cal1 += nCalFxWidth;		//Positive, Clockwise
			cal2 += -nCalFxWidth;		
			cal3 += nCalFxWidth;
			cal4 += -nCalFxWidth;	

			if (versionX==10)
			{
				//+ copter
				cal1 += 0;
				cal2 += nCalailWidth;		//Positive, Roll towards the right
				cal3 += 0;
				cal4 += -nCalailWidth;	

				cal1 += nCalelevWidth;		
				cal2 += 0;
				cal3 += -nCalelevWidth;
				cal4 += 0;

			}
			else
			{
				//X copter
				cal1 += -nCalailWidth;
				cal2 += nCalailWidth;		//Positive, Roll towards the right
				cal3 += nCalailWidth;
				cal4 += -nCalailWidth;	

				cal1 += nCalelevWidth;		
				cal2 += nCalelevWidth;
				cal3 += -nCalelevWidth;
				cal4 += -nCalelevWidth;
			}

		}
		else if (DroneMode==6)
		{
			//X6
			cal1 += nCalFxWidth;		//正的，往顺时针转
			cal2 += -nCalFxWidth;
			cal3 += nCalFxWidth;		
			cal4 += -nCalFxWidth;	
			cal5 += nCalFxWidth;		//正的，往顺时针转
			cal6 += -nCalFxWidth;

			//X6
			if (versionX==10)
			{
				//十字
				cal1 += 0;
				cal2 += nCalailWidth;
				cal3 += nCalailWidth;		
				cal4 += 0;
				cal5 += -nCalailWidth;		//正的，往顺时针转
				cal6 += -nCalailWidth;

				cal1 += nCalelevWidth;		//正的，仰头
				cal2 += nCalelevWidth;
				cal3 += -nCalelevWidth;
				cal4 += -nCalelevWidth;
				cal5 += -nCalelevWidth;		//正的，仰头
				cal6 += nCalelevWidth;

			}
			else if (versionX==11)
			{
				cal1 += -nCalailWidth;
				cal2 += nCalailWidth;
				cal3 += nCalailWidth*0.87;
				cal4 += nCalailWidth*0.41;
				cal5 += -nCalailWidth*0.41;		//正的，往顺时针转
				cal6 += -nCalailWidth*0.87;

				cal1 += nCalelevWidth;		//正的，仰头
				cal2 += nCalelevWidth;
				cal3 += -nCalelevWidth*0.08;
				cal4 += -nCalelevWidth;
				cal5 += -nCalelevWidth;		//正的，仰头
				cal6 += -nCalelevWidth*0.08;

			}
			else
			{
				cal1 += -nCalailWidth;
				cal2 += nCalailWidth;
				cal3 += nCalailWidth;
				cal4 += nCalailWidth;		
				cal5 += -nCalailWidth;		//正的，往顺时针转
				cal6 += -nCalailWidth;

				cal1 += nCalelevWidth;		//正的，仰头
				cal2 += nCalelevWidth;
				cal3 += 0;
				cal4 += -nCalelevWidth;
				cal5 += -nCalelevWidth;		//正的，仰头
				cal6 += 0;
			}

		}
		else if (DroneMode==8)
		{

			if (versionX==99)				//自定义
			{

				cal1 += nCalFxWidth*(float)paraX[0]/100;
				cal2 += nCalFxWidth*(float)paraX[1]/100;		
				cal3 += nCalFxWidth*(float)paraX[2]/100;
				cal4 += nCalFxWidth*(float)paraX[3]/100;	
				cal5 += nCalFxWidth*(float)paraX[4]/100;
				cal6 += nCalFxWidth*(float)paraX[5]/100;		
				cal7 += nCalFxWidth*(float)paraX[6]/100;
				cal8 += nCalFxWidth*(float)paraX[7]/100;

				cal1 += nCalailWidth*(float)paraX[8]/100;
				cal2 += nCalailWidth*(float)paraX[9]/100;
				cal3 += nCalailWidth*(float)paraX[10]/100;
				cal4 += nCalailWidth*(float)paraX[11]/100;
				cal5 += nCalailWidth*(float)paraX[12]/100;
				cal6 += nCalailWidth*(float)paraX[13]/100;
				cal7 += nCalailWidth*(float)paraX[14]/100;
				cal8 += nCalailWidth*(float)paraX[15]/100;

				cal1 -= nCalelevWidth*(float)paraX[16]/100;
				cal2 -= nCalelevWidth*(float)paraX[17]/100;
				cal3 -= nCalelevWidth*(float)paraX[18]/100;
				cal4 -= nCalelevWidth*(float)paraX[19]/100;
				cal5 -= nCalelevWidth*(float)paraX[20]/100;
				cal6 -= nCalelevWidth*(float)paraX[21]/100;
				cal7 -= nCalelevWidth*(float)paraX[22]/100;
				cal8 -= nCalelevWidth*(float)paraX[23]/100;

			}
			else
				if (versionX==10)
				{

					cal1 += nCalFxWidth;		//正的，往顺时针转
					cal2 += -nCalFxWidth;		
					cal3 += nCalFxWidth;
					cal4 += -nCalFxWidth;	
					cal5 += nCalFxWidth;		//正的，往顺时针转
					cal6 += -nCalFxWidth;		
					cal7 += nCalFxWidth;
					cal8 += -nCalFxWidth;	

					//十字
					cal1 += 0;
					cal2 += nCalailWidth;
					cal3 += nCalailWidth;		
					cal4 += nCalailWidth;
					cal5 += 0;
					cal6 += -nCalailWidth;
					cal7 += -nCalailWidth;
					cal8 += -nCalailWidth;

					cal1 += nCalelevWidth;
					cal2 += nCalelevWidth;
					cal3 += 0;
					cal4 += -nCalelevWidth;
					cal5 += -nCalelevWidth;
					cal6 += -nCalelevWidth;
					cal7 += 0;
					cal8 += nCalelevWidth;

				}
				else if (versionX==0)
				{
					cal1 += nCalFxWidth;		//正的，往顺时针转
					cal2 += -nCalFxWidth;		
					cal3 += nCalFxWidth;
					cal4 += -nCalFxWidth;	
					cal5 += nCalFxWidth;		//正的，往顺时针转
					cal6 += -nCalFxWidth;		
					cal7 += nCalFxWidth;
					cal8 += -nCalFxWidth;	

					//十字
					cal1 += -nCalailWidth;
					cal2 += nCalailWidth;
					cal3 += nCalailWidth;		
					cal4 += nCalailWidth;
					cal5 += nCalailWidth;
					cal6 += -nCalailWidth;
					cal7 += -nCalailWidth;
					cal8 += -nCalailWidth;

					cal1 += nCalelevWidth;
					cal2 += nCalelevWidth;
					cal3 += nCalelevWidth;
					cal4 += -nCalelevWidth;
					cal5 += -nCalelevWidth;
					cal6 += -nCalelevWidth;
					cal7 += -nCalelevWidth;
					cal8 += nCalelevWidth;
				}
				else	
					if (versionX == 20)			//X8，共轴，4轴8电机
					{
						//X8
						cal1 += nCalFxWidth;		//正的，往顺时针转
						cal2 += nCalFxWidth;		
						cal3 += nCalFxWidth;
						cal4 += nCalFxWidth;	
						cal5 += -nCalFxWidth;		//正的，往顺时针转
						cal6 += -nCalFxWidth;		
						cal7 += -nCalFxWidth;
						cal8 += -nCalFxWidth;	

						//X8
						cal1 += -nCalailWidth;
						cal2 += nCalailWidth;		//正的，往右横滚
						cal3 += nCalailWidth;
						cal4 += -nCalailWidth;	
						cal5 += -nCalailWidth;
						cal6 += nCalailWidth;		//正的，往右横滚
						cal7 += nCalailWidth;
						cal8 += -nCalailWidth;	


						//X8
						cal1 += nCalelevWidth;		//正的，仰头
						cal2 += nCalelevWidth;
						cal3 += -nCalelevWidth;
						cal4 += -nCalelevWidth;
						cal5 += nCalelevWidth;		//正的，仰头
						cal6 += nCalelevWidth;
						cal7 += -nCalelevWidth;
						cal8 += -nCalelevWidth;
					}
		}
		else if (DroneMode==3)			//Y6
		{
			if (versionX==10)
			{

				cal1 += nCalFxWidth;		//正的，往顺时针转
				cal2 += nCalFxWidth;		
				cal3 += nCalFxWidth;
				cal4 += -nCalFxWidth;	
				cal5 += -nCalFxWidth;		//正的，往顺时针转
				cal6 += -nCalFxWidth;		

				//十字
				cal1 += 0;
				cal2 += nCalailWidth;
				cal3 += -nCalailWidth;		
				cal4 += 0;
				cal5 += nCalailWidth;
				cal6 += -nCalailWidth;

				cal1 += nCalelevWidth;
				cal2 += -nCalelevWidth;
				cal3 += -nCalelevWidth;
				cal4 += nCalelevWidth;
				cal5 += -nCalelevWidth;
				cal6 += -nCalelevWidth;
			}
			else if (versionX==0)
			{
				cal1 += nCalFxWidth;		//正的，往顺时针转
				cal2 += nCalFxWidth;		
				cal3 += nCalFxWidth;
				cal4 += -nCalFxWidth;	
				cal5 += -nCalFxWidth;		//正的，往顺时针转
				cal6 += -nCalFxWidth;		

				//十字
				cal1 += 0;
				cal2 += -nCalailWidth;
				cal3 += nCalailWidth;		
				cal4 += 0;
				cal5 += -nCalailWidth;
				cal6 += nCalailWidth;

				cal1 += -nCalelevWidth;
				cal2 += nCalelevWidth;
				cal3 += nCalelevWidth;
				cal4 += -nCalelevWidth;
				cal5 += nCalelevWidth;
				cal6 += nCalelevWidth;
			}
		}



		if (!((Parameter[POWER_NO] & 0x20) >> 5))				//Control stick uses this
		{
			if (Lock_Throttle)						//Throttle is locked
				if (((Channel_5<1666)||((Channel_5>1666)&&(Channel_6<1666)))&&(Channel_Throttle>1900))	//Manual hovering/Automatic hovering, with the throttle at its lowest position
					
				{
					if ((Channel_Rudder < Rudder_MID - 350)&&(Channel_Elevator > Elevator_MID + 350)&&(Channel_Aileron > Aileron_MID + 350))	//If control stick action is completed
					{
						if (count_unlock==0)
						{
							if ((magmax_estimate<0.5)&&(gyro_estimate<0.5))
								if1 = 1;
							else
								if1 = 0;
						}

						count_unlock ++;
						if (count_unlock >= hz250/10)
						{
							if((magmax_estimate<1)&&(gyro_estimate<1)&&(if1))
							{
								//CLEAR BIAS
								bias_iraw_gyro_2 = zero_gyro2;
								bias_iraw_gyro_3 = zero_gyro3;
								bias_iraw_gyro_1 = zero_gyro1;

								if (Channel_5<1666)
								{
									alldata.gps_latti = 40;
									alldata.gps_longi = 112;

									alldata.gps_velN = 0;
									alldata.gps_velE = 0;
									alldata.gps_velD = 0;

									rpy[0] = 0;
									rpy[1] = 0;
									rpy[2] = 0;
									vel_down_clear = 0;
									InitData();					//Initialize state calculation
								}
								else
								{
									alldata.gps_latti = CurLatti;
									alldata.gps_longi = CurLongi;
									alldata.gps_velN = (float)tvely_i/100;	//North velocity
									alldata.gps_velE = (float)tvelx_i/100;	//East velocity
									alldata.gps_velD = 0;
									rpy[0] = 0;
									rpy[1] = 0;
									rpy[2] = 0;
									vel_down_clear = 0;
									InitData();					
								}
								writebuf1 = 1;
							}

							count_unlock = 0;

							Lock_Throttle = 0;
							ctlock_throttle = 0;
						}
					}
					else
						count_unlock = 0;

				}
		}


		if ((THROT < 1900)&&(!Lock_Throttle))			//Not locked
		{
			ctlock_throttle = 0;
		}

		if (((THROT > 1900)&&( Channel_5<1333))||((endland==1)&&(Channel_5>1666))||(Lock_Throttle))		//A little throttle and CH5 in the first gear
		{



			Xthrot = 1930;
			if (Lock_Throttle)
			{
				if (Enable_Waypoint==1)
				{
					Enable_Waypoint = 0;
				}

				ch5_status = 0;
				close = 0;
				sumx = 0;
				sumy = 0;
				endland = 0;
				gohome = 0;
				pregohome = 0;
				gohome_end = 0;
				autotakeoff = 0;
				ClickSetTgtFlag	= 0;
				//			gimbal_target_latti = 0;
				//			gimbal_target_longi = 0;
				gimbal_target_lock = 0;
			}


			cal1 = 0;
			cal2 = 0;
			cal3 = 0;
			cal4 = 0;
			cal5 = 0;
			cal6 = 0;
			cal7 = 0;
			cal8 = 0;
			nospin = 1;
			ctgohome = 0;
			in_flight = 0;
			Enable_Control_Integrate = 0;
			Enable_Control_Integrate_pre = 0;
			carefree = 0;
			sumheading = 0;


			TargetRollMid = 0;
			TargetPitchMid = 0;


			ctlock_throttle ++;
			if (ctlock_throttle >= 5*hz250)
			{
				ctlock_throttle = 5*hz250;
				if (!((Parameter[POWER_NO] & 0x10) >> 4))
					Lock_Throttle = 1;
			}
		}
		else 
		{
			if ((Channel_5 > 1333)&&(THROT > 1900)&&(in_flight==0))	//Under manual set height/GPS mode, if the throttle is set low, it doesn't work 
			{
			
				cal1 = 0;
				cal2 = 0;
				cal3 = 0;
				cal4 = 0;
				cal5 = 0;
				cal6 = 0;
				cal7 = 0;
				cal8 = 0;
				nospin = 1;
			

				Enable_Control_Integrate = 0;
				Enable_Control_Integrate_pre = 0;

				sumheading = 0;


				TargetRollMid = 0;
				TargetPitchMid = 0;



				ctlock_throttle ++;
				if (ctlock_throttle >= 5*hz250)
				{
					ctlock_throttle = 5*hz250;
					if (!((Parameter[POWER_NO] & 0x10) >> 4))
						Lock_Throttle = 1;
				}

			}			
			else
			{

				Parameter[ERASE_ROMBOOT] = 0;
				nospin = 0;
			}
		}



		if ((nospin==0)&&(vforce<low_power))
		{
			ctpower ++;
			if (vforce < emergency_power)
			{
				if (ctpower >= hz250/4)
				{
					SndBuf[WARNING_FLAG] = 1;
					power_mag = hz250/3;
					ctpower = 0;
				}
			}
			else
			{
				if (ctpower >= (float)hz250*2)
				{
					SndBuf[WARNING_FLAG] = 1;
					power_mag = hz250/3;
					ctpower = 0;
				}
			}
		}
		else
		{
			power_mag = 0;
			SndBuf[WARNING_FLAG] = 0;
		}


		if (SndBuf[WARNING_FLAG])
		{
			if (power_mag > 0)
			{
				power_mag --;
				if (power_mag <= 0)
					SndBuf[WARNING_FLAG] = 0;
			}
		}


		if (THROT > 1900)
		{
			throt_low = 1;
		}
		else
		{
			if (throt_low==1)
			{
				throt_low = 0;
				if (!bAutoHand_)
					targetYaw = scalheading*180/PI;			//Manually raise the throttle, target heading direction = current heading direction
				ref_yaw = scalheading;

				//When increasing throttle, clear xekf[2]和xekf[5]		
				if (Channel_5 < 1333)
				{
					xEKF[2] = -tAirHighf;
					xEKF[5] = alldata.gps_velD;
				}
			}
		}



		if (((Channel_Throttle > 1900)&&( Channel_5<1333 ))||((endland==1)&&(Channel_5>1666))||(Lock_Throttle))		//小油门，且CH5手动第1档
		{
			nCalthrotWidth = 1930;
			Xthrot = 1930;
			OUT_ATL_YMDUO = nCalthrotWidth/10;
		}		



		fOUT_FXDUO = 3000 - nCalthrotWidth + cal1;
		fOUT_ailDUO = 3000 - nCalthrotWidth + cal2;
		fOUT_elevDUO = 3000 - nCalthrotWidth + cal3;
		fOUT_throtDUO = 3000 - nCalthrotWidth + cal4;



		fOUT_5DUO = 3000 - nCalthrotWidth + cal5;
		fOUT_6DUO = 3000 - nCalthrotWidth + cal6;
		fOUT_7DUO = 3000 - nCalthrotWidth + cal7;
		fOUT_8DUO = 3000 - nCalthrotWidth + cal8;

		bcal1 = cal1;
		bcal2 = cal2;
		bcal3 = cal3;
		bcal4 = cal4;
		bcal5 = cal5;
		bcal6 = cal6;
		bcal7 = cal7;
		bcal8 = cal8;


		if (fOUT_5DUO > 2000) fOUT_5DUO = 2000;
		if (fOUT_5DUO < 1000) fOUT_5DUO = 1000;

		if (fOUT_6DUO > 2000) fOUT_6DUO = 2000;
		if (fOUT_6DUO < 1000) fOUT_6DUO = 1000;

		if (fOUT_7DUO > 2000) fOUT_7DUO = 2000;
		if (fOUT_7DUO < 1000) fOUT_7DUO = 1000;

		if (fOUT_8DUO > 2000) fOUT_8DUO = 2000;
		if (fOUT_8DUO < 1000) fOUT_8DUO = 1000;

		if (fOUT_FXDUO > 2000) fOUT_FXDUO = 2000;
		if (fOUT_FXDUO < 1000) fOUT_FXDUO = 1000;

		if (fOUT_ailDUO > 2000) fOUT_ailDUO = 2000;
		if (fOUT_ailDUO < 1000) fOUT_ailDUO = 1000;

		if (fOUT_elevDUO > 2000) fOUT_elevDUO = 2000;
		if (fOUT_elevDUO < 1000) fOUT_elevDUO = 1000;

		if (fOUT_throtDUO > 2000) fOUT_throtDUO = 2000;
		if (fOUT_throtDUO < 1000) fOUT_throtDUO = 1000;




		ofOUT_FXDUO = fOUT_FXDUO;	
		ofOUT_ailDUO = fOUT_ailDUO;	
		ofOUT_elevDUO = fOUT_elevDUO;	
		ofOUT_throtDUO = fOUT_throtDUO;	

		ofOUT_5DUO = fOUT_5DUO;	
		ofOUT_6DUO = fOUT_6DUO;	
		ofOUT_7DUO = fOUT_7DUO;	
		ofOUT_8DUO = fOUT_8DUO;	

		tOUT_FXDUO = ofOUT_FXDUO*2;
		tOUT_ailDUO = ofOUT_ailDUO*2;
		tOUT_elevDUO = ofOUT_elevDUO*2;
		tOUT_throtDUO = ofOUT_throtDUO*2;

		tOUT_5DUO = ofOUT_5DUO*2;
		tOUT_6DUO = ofOUT_6DUO*2;
		tOUT_7DUO = ofOUT_7DUO*2;
		tOUT_8DUO = ofOUT_8DUO*2;

		tmph = (tOUT_FXDUO >>8);		//输出的第1个电机
		tmpl = tOUT_FXDUO & 0xff;
		*pmem_FANGXIANGl = tmpl;
		*pmem_FANGXIANGh = tmph;

		tmph = (tOUT_ailDUO >>8);		//输出的第2个电机
		tmpl = tOUT_ailDUO & 0xff;
		*pmem_FUYIl = tmpl;
		*pmem_FUYIh = tmph;

		tmph = (tOUT_elevDUO >>8);		//输出的第3个电机
		tmpl = tOUT_elevDUO & 0xff;
		*pmem_SHENGJIANGl = tmpl;
		*pmem_SHENGJIANGh = tmph;

		tmph = (tOUT_throtDUO >>8);		//输出的第4个电机
		tmpl = tOUT_throtDUO & 0xff;
		*pmem_throttlel = tmpl;
		*pmem_throttleh = tmph;

		tmph = (tOUT_5DUO >>8);		//输出的第5个电机
		tmpl = tOUT_5DUO & 0xff;
		*pmem_fpga1l = tmpl;
		*pmem_fpga1h = tmph;

		tmph = (tOUT_6DUO >>8);		//输出的第6个电机
		tmpl = tOUT_6DUO & 0xff;
		*pmem_fpga2l = tmpl;
		*pmem_fpga2h = tmph;

		tmph = (tOUT_7DUO >>8);		//输出的第7个电机
		tmpl = tOUT_7DUO & 0xff;
		*pmem_kaisanl = tmpl;
		*pmem_kaisanh = tmph;

		tmph = (tOUT_8DUO >>8);		//输出的第8个电机
		tmpl = tOUT_8DUO & 0xff;
		*pmem_fpga8l = tmpl;
		*pmem_fpga8h = tmph;



		if (DroneMode==8)
		{
			tmph = (tOUT_7DUO >>8);		//输出的第7个电机
			tmpl = tOUT_7DUO & 0xff;
			*pmem_fpga10l = tmpl;		
			*pmem_fpga10h = tmph;

			tmph = (tOUT_8DUO >>8);		//输出的第8个电机
			tmpl = tOUT_8DUO & 0xff;
			*pmem_fpga11l = tmpl;
			*pmem_fpga11h = tmph;
		}





		if (enter_set==1)
		{
			out2 = Channel_Elevator;
		}
		else
		{
			if (Parameter[GIMBAL_PITCH_SENSIT] > 128) y19 = (float)Parameter[GIMBAL_PITCH_SENSIT] - 256;
			else	y19 = (float)Parameter[GIMBAL_PITCH_SENSIT];

			if (y19==0) y19 = 20;
			y19 = 1000 / y19;

			tout = (t_PlanePitch / y19) * 500;

			if (Parameter[GIMBAL_PITCH_SENSIT]==0)	tout = 0;


			max = (float)25*500/fabsf(y19);
			if (tout > max) tout = max;
			if (tout < -max) tout = -max;

			if (zlf_data>0)	zlf_data--;
			out2 = fpgain7 + tout;
		}


		out2*=2;

		if (Parameter[GIMBAL_ROLL_SENSITIVITY] > 128) y13 = (float)Parameter[GIMBAL_ROLL_SENSITIVITY] - 256;
		else	y13 = (float)Parameter[GIMBAL_ROLL_SENSITIVITY];


		if (y13==0) y13 = 20;
		y13 = 1000 / y13;

		tout = (t_PlaneRoll / y13) * 500;

		if (Parameter[GIMBAL_ROLL_SENSITIVITY]==0)	tout = 0;
		max = (float)25*500/fabsf(y13);
		if (tout > max) tout = max;
		if (tout < -max) tout = -max;

		if (enter_set==1)
		{
			out1 = fpgain7;
			Channel8_position = fpgain7;
		}
		else
		{
			out1 = Channel8_position + tout;
		}

		out1*=2;



		if (out2 > 4400) out2 = 4400;
		if (out2 < 1600) out2 = 1600;

		if (out1 > 4400) out1 = 4400;
		if (out1 < 1600) out1 = 1600;


		tmph = (out2 >> 8);			//7th output channel, gimbal pitch
		tmpl = out2 & 0xff;

		if (DroneMode!=8)
		{
			*pmem_fpga11l = tmpl;		
			*pmem_fpga11h = tmph;
		}




		tmph = (out1 >> 8);			//8th output channel, gimbal roll
		tmpl = out1 & 0xff;

		if (DroneMode!=8)
		{
			*pmem_fpga10l = tmpl;		
			*pmem_fpga10h = tmph;
		}


		if (startcontrol)
			*pmem_poke	= 81;			//Refresh the output of PWM in FPGA

		else
		{
			out1 = 1070*2;

			tmph = (out1 >> 8);			//8th output channel, gimbal roll
			tmpl = out1 & 0xff;

			*pmem_FANGXIANGl = tmpl;
			*pmem_FANGXIANGh = tmph;

			*pmem_FUYIl = tmpl;
			*pmem_FUYIh = tmph;

			*pmem_SHENGJIANGl = tmpl;
			*pmem_SHENGJIANGh = tmph;

			*pmem_throttlel = tmpl;
			*pmem_throttleh = tmph;

			*pmem_fpga1l = tmpl;
			*pmem_fpga1h = tmph;

			*pmem_fpga2l = tmpl;
			*pmem_fpga2h = tmph;

			*pmem_kaisanl = tmpl;
			*pmem_kaisanh = tmph;

			*pmem_fpga8l = tmpl;
			*pmem_fpga8h = tmph;


			out1 = 1500*2;

			tmph = (out1 >> 8);			//8th output channel, gimbal roll
			tmpl = out1 & 0xff;

			*pmem_fpga10l = tmpl;
			*pmem_fpga10h = tmph;

			*pmem_fpga11l = tmpl;
			*pmem_fpga11h = tmph;

			*pmem_poke	= 81; 
		}

		if (enter_set==1)
			OUT_ARM_AUTOHANDLEBF = 0x07;
		if (clear_bias==1)
			OUT_ARM_AUTOHANDLEBF = 0x08;


		if ((gohome==1)||(pregohome==1))
			OUT_ARM_AUTOHANDLEBF = 0x0b;

		OUT_ARM_AUTOHANDLE = OUT_ARM_AUTOHANDLEBF;

		iTargetRoll = (int)(TargetRoll);
		iTargetPitch = (int)(TargetPitch);

		sendcount++;
		if (sendcount>=hz250) sendcount = hz250;




		if(AT91F_PDC_IsTxEmpty(AT91C_BASE_PDC_DBGU))
		{

			if (Read1min==0)
			{

				if ((sendcount>=(hz250/Parameter[TEST_CONTROL_FREQ]))&&(haveSendBack!=0)&&(haveSendBack_all!=0)&&(sendpara!=0)&&(plotmag!=1))
				{
					sendcount = 0;
					for (iii=0;iii<OUTLENGTH;iii++)
					{
						fashe[iii] = SndBuf[iii+18];
						fashe2[iii] = fashe[iii];
					}
					veriail = 0;
					for(ytemp=0;ytemp<OUTLENGTH-1;ytemp++)	veriail = veriail+fashe[ytemp];
					fashe[98] = veriail;
					fashe2[98] = veriail;




					AT91F_PDC_SetTx(AT91C_BASE_PDC_DBGU,&fashe[0],OUTLENGTH+10); //Send params to the port 
					

					if (in_flight)	
						write_data_record = 1;
				}
			}
			else
			{
				ctread = 50;	//BlackBox

				if ((sendcount>=ctread)&&(haveSendBack!=0)&&(haveSendBack_all!=0)&&(sendpara!=0))
				{
					for(iii=0;iii<99;iii++)
						fashe[iii] = BlackBox[99*count_fs+iii];

					sendcount = 0;
					veriail = 0;
					for(ytemp=0;ytemp<OUTLENGTH-1;ytemp++)	veriail = veriail+fashe[ytemp];
					fashe[98] = veriail;
					AT91F_PDC_SetTx(AT91C_BASE_PDC_DBGU,&fashe[0],OUTLENGTH); //Send params to the port
					
					count_fs++;
					if (count_fs >= 289) 
					{
						Read1min = 0;
						count_fs = 0;
						haveSend1min = 0;
					}
				}
			}



			if (senddin==0)
			{
				msg[0]='$';
				msg[1]='D';
				msg[2]='I';
				msg[3]='N';
				ytgtLongi = tgtLongi;
				ytgtLatti = tgtLatti;
				float2buf(&msg[4],&ytgtLongi);
				float2buf(&msg[8],&ytgtLatti);
				msg[12] = 0;
				msg[13] = 0;
				for (i=0;i<13;i++)
					msg[13]+=msg[i];
				AT91F_PDC_SetTx(AT91C_BASE_PDC_DBGU,(char *)msg,14);
				senddin = 1;

			}

			if(send_paraX==1)
			{
				msg[0]='$';
				msg[1]='P';
				msg[2]='U';
				msg[3]='R';
				paraUSER[32]=0x00;
				paraUSER[32]='$'+'P'+'U'+'R';
				for(i=0;i<32;i++)
				{
					msg[i+4]=paraUSER[i];
					paraUSER[32]=paraUSER[32]+paraUSER[i];
				}
				msg[4+32]=paraUSER[32];

				AT91F_PDC_SetTx(AT91C_BASE_PDC_DBGU,(char *)msg,37);
				send_paraX=0;
			}



			if(rdParaHeadFlag==1)
			{
				msg[0]='$';
				msg[1]='P';
				msg[2]='A';
				msg[3]='R';
				Parameter[40]=0x00;
				Parameter[40]='$'+'P'+'A'+'R';
				for(i=0;i<40;i++)
				{
					msg[i+4]=Parameter[i];
					Parameter[40]=Parameter[40]+Parameter[i];
				}
				msg[4+40]=Parameter[40];

				AT91F_PDC_SetTx(AT91C_BASE_PDC_DBGU,(char *)msg,45);
				rdParaHeadFlag=0;
			}


			if(send_enterset==1)
			{
				send_enterset=0;
				msg[0]='$';
				msg[1]='S';
				msg[2]='E';
				msg[3]='T';
				msg[4]='E';
				msg[5]='N';
				AT91F_PDC_SetTx(AT91C_BASE_PDC_DBGU,(char *)msg,6);				
			}

			if(send_exitset==1)
			{
				send_exitset=0;
				msg[0]='$';
				msg[1]='S';
				msg[2]='E';
				msg[3]='T';
				msg[4]='E';
				msg[5]='X';
				AT91F_PDC_SetTx(AT91C_BASE_PDC_DBGU,(char *)msg,6);				
			}

			if(CaptureFlag==1)
			{
				doCapture();
				CaptureFlag=0;
				writebuf1 = 1;
				msg[0]='$';
				msg[1]='D';
				msg[2]='O';
				msg[3]='C';
				msg[4]='A';
				msg[5]='P';
				AT91F_PDC_SetTx(AT91C_BASE_PDC_DBGU,(char *)msg,6);				
			}

			if (haveSend1min==0)
			{
				msg[0]='$';
				msg[1]='1';
				msg[2]='M';
				msg[3]='I';
				msg[4]='N';
				AT91F_PDC_SetTx(AT91C_BASE_PDC_DBGU,(char *)msg,5);
				haveSend1min = 1;
			}


			if (haveSendBack==0)
			{//Send back a waypoint information to the GCS
				cthavesendback++;
				if (cthavesendback >= 40)
				{
					cthavesendback = 0;
					msg[0]='$';
					msg[1]='S';
					msg[2]='E';
					msg[3]='T';
					msg[4]='D';

					i=c_curnum-1;	

					msg[5] = i+1;

					float2buf(&Longi_Array[0],&tgt_Latti[i]);
					msg[6] = Longi_Array[0];
					msg[7] = Longi_Array[1];
					msg[8] = Longi_Array[2];
					msg[9] = Longi_Array[3];


					float2buf(&Longi_Array[0],&tgt_Longi[i]);
					msg[10] = Longi_Array[0];
					msg[11] = Longi_Array[1];
					msg[12] = Longi_Array[2];
					msg[13] = Longi_Array[3];

					int2buf(&Longi_Array[0],&tgt_High[i]);
					msg[14] = Longi_Array[0];
					msg[15] = Longi_Array[1];
					msg[16] = Longi_Array[2];
					msg[17] = Longi_Array[3];

					int2buf(&Longi_Array[0],&tgt_Vel[i]);
					msg[18] = Longi_Array[0];
					msg[19] = Longi_Array[1];
					msg[20] = Longi_Array[2];
					msg[21] = Longi_Array[3];

					msg[23] = tgt_zwl[i]/256;
					msg[22] = tgt_zwl[i]-(unsigned char)(msg[23])*256;
					msg[24] = tgt_hover_num[i];
					msg[26] = tgt_hover_time[i]/256;
					msg[25] = tgt_hover_time[i]-(unsigned char)(msg[26])*256;
					msg[27] = tgt_djnum[i];
					msg[28] = tgt_djtime[i];
					msg[29] = tgt_djrpttime[i];

					len = 31;
					msg[30]=0;
					for(i=0;i<30;i++)
						msg[30]+=msg[i];
					AT91F_PDC_SetTx(AT91C_BASE_PDC_DBGU,(char *)msg,len);
					haveSendBack = 1;
					//						return;
				}
			}




			if (haveSendBack_all==0)
			{//Send back all waypoint information to the GCS
				cthavesendbackall++;
				if (cthavesendbackall >= 40)
				{
					cthavesendbackall = 0;

					for(i=0;i<Waypoint_number;i++)
					{
						msg[0+31*i]='$';
						msg[1+31*i]='S';
						msg[2+31*i]='E';
						msg[3+31*i]='T';
						msg[4+31*i]='D';

						msg[5+31*i] = i+1;

						float2buf(&Longi_Array[0],&tgt_Latti[i]);
						msg[6+31*i] = Longi_Array[0];
						msg[7+31*i] = Longi_Array[1];
						msg[8+31*i] = Longi_Array[2];
						msg[9+31*i] = Longi_Array[3];

						float2buf(&Longi_Array[0],&tgt_Longi[i]);
						msg[10+31*i] = Longi_Array[0];
						msg[11+31*i] = Longi_Array[1];
						msg[12+31*i] = Longi_Array[2];
						msg[13+31*i] = Longi_Array[3];

						int2buf(&Longi_Array[0],&tgt_High[i]);
						msg[14+31*i] = Longi_Array[0];
						msg[15+31*i] = Longi_Array[1];
						msg[16+31*i] = Longi_Array[2];
						msg[17+31*i] = Longi_Array[3];

						int2buf(&Longi_Array[0],&tgt_Vel[i]);
						msg[18+31*i] = Longi_Array[0];
						msg[19+31*i] = Longi_Array[1];
						msg[20+31*i] = Longi_Array[2];
						msg[21+31*i] = Longi_Array[3];

						msg[23+31*i] = tgt_zwl[i]/256;
						msg[22+31*i] = tgt_zwl[i]-(unsigned char)(msg[23])*256;
						msg[24+31*i] = tgt_hover_num[i];
						msg[26+31*i] = tgt_hover_time[i]/256;
						msg[25+31*i] = tgt_hover_time[i]-(unsigned char)(msg[26])*256;
						msg[27+31*i] = tgt_djnum[i];
						msg[28+31*i] = tgt_djtime[i];
						msg[29+31*i] = tgt_djrpttime[i];

						msg[30+31*i]=0;
						for(kk=0;kk<30;kk++)
							msg[30+31*i]+=msg[kk+31*i];

					}



					AT91F_PDC_SetTx(AT91C_BASE_PDC_DBGU,(char *)&msg[31*ctindex],31);
					ctindex ++;
					if (ctindex == Waypoint_number)
					{
						cthavesendbackall = 0;
						haveSendBack_all = 1;
					}				
				}
			}



			if (plotmag==1)
			{//Send back compass data to GCS
				outmagdata = 1;
				ctplotmag++;
				if (ctplotmag >= 10)
				{
					ctplotmag = 0;

					AT91F_PDC_SetTx(AT91C_BASE_PDC_DBGU,(char *)&msg[13*cthui],13);
					cthui ++;
					if (cthui >= ct_rawmag+ct_rawmagz)
					{
						plotmag_end = 3;
						countplotmag = (float)hz250*0.5;
						ctplotmag = 0;
						plotmag = 0;
						cthui = 0;
					}				
				}
			}

			if (countplotmag <= 0)
				if ((plotmag_end>0)&&(AT91F_PDC_IsTxEmpty(AT91C_BASE_PDC_US2))&&(AT91F_PDC_IsTxEmpty(AT91C_BASE_PDC_US1)))
				{
					countplotmag = (float)hz250*0.5;
					plotmag_end --;


					msg[0]='$';
					msg[1]='M';
					msg[2]='G';
					msg[3]='1';

					float2buf(&msg[4],&xmag_bias);
					float2buf(&msg[8],&throtag_bias);
					float2buf(&msg[12],&zmag_bias);
					float2buf(&msg[16],&Kmag);
					float2buf(&msg[20],&Kmag2);

					msg[24]=0;
					for(kk=4;kk<24;kk++)
						msg[24]+=msg[kk];

					AT91F_PDC_SetTx(AT91C_BASE_PDC_DBGU,(char *)&msg[0],25);



					if (plotmag_end<=0)
					{
						outmagdata = 0;
						alldata.gps_latti = 40;
						alldata.gps_longi = 112;

						alldata.gps_velN = 0;
						alldata.gps_velE = 0;

						rpy[0] = 0;
						rpy[1] = 0;
						rpy[2] = 0;

						vel_down_clear = 1;
						InitData();					
					}

				}

		}



		if (ESC_select==2)		//T580电调
			*pmem_maggo = 3;


		rpm = *pmem_cnt5msl;
		SndBuf[44+18] = rpm;
		in_irq0 = 0;
} 


//unused, but can be expanded.
void AT91F_UART2_HANDLER(void) //UART2来数据中断，暂未用，可扩展
{
	return;
}

void GPS_init()
{ 	//波特率配置

	AT91F_US_SetBaudrate ((AT91PS_USART)AT91C_BASE_US0, 59904000, 9600);	//普通GPS


	AT91F_US_SetBaudrate ((AT91PS_USART)AT91C_BASE_US1, 59904000, 115200);

	AT91F_US_SetBaudrate ((AT91PS_USART)AT91C_BASE_DBGU, 59904000, BAUD_YS);	

	while (!AT91F_US_TxReady((AT91PS_USART)AT91C_BASE_US0));
}

void US_PDC_Init(void)
{
	AT91F_PDC_EnableTx((AT91PS_PDC) AT91C_BASE_PDC_US0 );
	AT91F_PDC_EnableTx((AT91PS_PDC) AT91C_BASE_PDC_US1 );
	AT91F_PDC_EnableTx((AT91PS_PDC) AT91C_BASE_PDC_US2 );
	AT91F_PDC_EnableTx((AT91PS_PDC) AT91C_BASE_PDC_DBGU );
}


//*----------------------------------------------------------------------------
//* Function Name       : main
//* Object              : Main function
//* Input Parameters    : none
//* Output Parameters   : True
//*----------------------------------------------------------------------------
int main(void)
{
	int ik = 0;
	int maxtemperature = 60;
	char PTZ_style;
	char power1_select,power2_select;
	float power1_xishu,power2_xishu;
	float fPlaneRoll,fPlanePitch;
	float lDistance,tempj1,beta,beta2,vel_1;

	int ytemp = 0,i=0,j=0;
	double yhpres2;

	unsigned char which;

	float ddx,ddy;

	out1 = 1070*2;

	tmph = (out1 >> 8);			//8th output channel, gimbal roll
	tmpl = out1 & 0xff;

	*pmem_FANGXIANGl = tmpl;   //rudder 
	*pmem_FANGXIANGh = tmph;

	*pmem_FUYIl = tmpl;        //aileron
	*pmem_FUYIh = tmph;

	*pmem_SHENGJIANGl = tmpl;  //pitch
	*pmem_SHENGJIANGh = tmph;

	*pmem_throttlel = tmpl;
	*pmem_throttleh = tmph;

	*pmem_fpga1l = tmpl;
	*pmem_fpga1h = tmph;

	*pmem_fpga2l = tmpl;
	*pmem_fpga2h = tmph;

	*pmem_kaisanl = tmpl; 
	*pmem_kaisanh = tmph;

	*pmem_fpga8l = tmpl;
	*pmem_fpga8h = tmph;

    out1 = 1500*2;

	tmph = (out1 >> 8);			//8th output channel, gimbal roll
	tmpl = out1 & 0xff;

	*pmem_fpga10l = tmpl;
	*pmem_fpga10h = tmph;

	*pmem_fpga11l = tmpl;
	*pmem_fpga11h = tmph;

	*pmem_poke	= 80; 

	endinit = 1;
	startled = 1;

	for (i=0;i<=15;i++)
		xEKF[i] = 0;

	for(i=0;i<4;i++)
		GPS_Refresh[0] = 0;

	led_flash_hz = 3;	

	colors[0] = 1;		//Red
	colors[1] = 1;		//Red
	colors[2] = 1;		//Red
	colors[3] = 0;
	colors[4] = 0;

	alldata.gps_latti = 40;
	alldata.gps_longi = 112;
	alldata.gps_high = 0;
	alldata.gps_velN = 0;
	alldata.gps_velE = 0;
	alldata.gps_velD = 0;
	alldata.gps_angle = 0;

	alldata.ina_gyro1 = 0;
	alldata.ina_gyro2 = 0;
	alldata.ina_gyro3 = 0;
	alldata.ina_acc1 = 0;
	alldata.ina_acc2 = 0;
	alldata.ina_acc3 = g0;

	CLR_TIME = hz250*5;

	alldata.true_roll = 0;
	alldata.true_pitch = 0;

	rr[0] = 0;
	rr[1] = 0;

	*pmem_355_rst_ = 0x01;	//在连接上ADIS16355的复位线时，可通过这几条置低置高的指令来复位它
	*pmem_355_rst_ = 0x00;
	//	for (i=0;i<100;i++)
	//		wait(100000);
	*pmem_355_rst_ = 0x01;
	alldata.gps_satelnum = 0;
	alldata.waittime = 50;		//开机1秒等待传感器预热
	alldata.starttime = 50;	//该数值暂未用，姿态解算函数里设置为了500，即10秒的对准时间
	alldata.algtime = 100;
	alldata.gps_valid = 0;
	//	InitData();					//初始化姿态解算函数的初始数据

	for (i=0;i<100;i++)
	{
		sumailzu[i] = 0;
		sumelevzu[i] = 0;
	}

	Rudder_MID = 1500;
	Aileron_MID = 1500;
	Elevator_MID = 1500;

	SndBuf[DIST_FROM_TAKE_OFF_LB] = 0; 
	SndBuf[FLIGHT_CONTROL_VOL_2] = 0;
	SndBuf[FLIGHT_CONTROL_VOL_1] = 0;
	SndBuf[RECEIVER_STATUS]=0;	//Enable receiver.

	OUT_ARM_FXZHONG = 150;
	OUT_ARM_FYZHONG = 150;
	OUT_ARM_SJZHONG = 150;

	OUT_ATL_FXDUO = 150;
	OUT_ATL_FYDUO = 150;
	OUT_ATL_SJDUO = 150;

	GPS_init();
	IO_init();

	OUT_ATL_FXDUO = Rudder_MID/10; //Set initial values
	OUT_ATL_FYDUO = Aileron_MID/10;
	OUT_ATL_SJDUO = Elevator_MID/10;

	AT91F_PDC_DisableRx(AT91C_BASE_PDC_DBGU);
	AT91F_PDC_DisableTx(AT91C_BASE_PDC_DBGU);
	AT91F_PDC_SetNextRx(AT91C_BASE_PDC_DBGU, dbg_buffer, 1);	
	AT91F_PDC_SetNextRx(AT91C_BASE_PDC_DBGU, dbg_buffer, 1);
	AT91F_PDC_EnableRx(AT91C_BASE_PDC_DBGU);
	AT91F_PDC_EnableTx(AT91C_BASE_PDC_DBGU);

	AT91F_US_EnableRx((AT91PS_USART)AT91C_BASE_US0);	
	AT91F_PDC_DisableRx(AT91C_BASE_PDC_US0);
	AT91F_PDC_DisableTx(AT91C_BASE_PDC_US0);
	AT91F_PDC_SetRx(AT91C_BASE_PDC_US0, &chuan2, 1);
	AT91F_PDC_SetNextRx(AT91C_BASE_PDC_US0, &chuan2, 1);
	AT91F_PDC_EnableRx(AT91C_BASE_PDC_US0);
	AT91F_PDC_EnableTx(AT91C_BASE_PDC_US0);

	AT91F_PDC_DisableRx(AT91C_BASE_PDC_US2);
	AT91F_PDC_DisableTx(AT91C_BASE_PDC_US2);
	AT91F_PDC_SetRx(AT91C_BASE_PDC_US2, wifi_buffer, 30);
	AT91F_PDC_SetNextRx(AT91C_BASE_PDC_US2, wifi_buffer, 30);

	//	AT91F_PDC_SetRx(AT91C_BASE_PDC_US2, inn, 46);
	//	AT91F_PDC_SetNextRx(AT91C_BASE_PDC_US2, inn, 46);
	AT91F_PDC_EnableRx(AT91C_BASE_PDC_US2);
	AT91F_PDC_EnableTx(AT91C_BASE_PDC_US2);

	AT91F_PDC_DisableRx(AT91C_BASE_PDC_US1);
	AT91F_PDC_DisableTx(AT91C_BASE_PDC_US1);
	//	AT91F_PDC_SetRx(AT91C_BASE_PDC_US1,(char*)(uart_data2.rev_rpr), 96);	
	//	AT91F_PDC_SetNextRx(AT91C_BASE_PDC_US1, (char*)(uart_data2.rev_rpr), 96);	
	AT91F_PDC_EnableRx(AT91C_BASE_PDC_US1);
	AT91F_PDC_EnableTx(AT91C_BASE_PDC_US1);

	//* System Timer initialization
	AT91F_AIC_ConfigureIt (
		AT91C_BASE_AIC,                         // AIC base address
		AT91C_ID_SYS,                          // System peripheral ID
		7,               // Max priority
		AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE, // Level sensitive
		AT91F_ST_ASM_HANDLER );						
	//* Enable ST interrupt
	AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_SYS);

	AT91F_DBGU_InterruptEnable(AT91C_BASE_DBGU, 0x08);

	//***********UART 0//

	AT91F_AIC_ConfigureIt (
		AT91C_BASE_AIC,                        // AIC base address
		AT91C_ID_US0,                          // System peripheral ID
		6,               // Max priority
		AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE, // Level sensitive
		AT91F_UART0_ASM_HANDLER );							
	AT91F_US_EnableIt((AT91PS_USART) AT91C_BASE_US0, 0x08);	//Input of port 0: Navigation input
	AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_US0);
	//	AT91F_US_PutChar((AT91PS_USART)AT91C_BASE_US0, 0x43);

	//*******************	


/*	//***********UART 1//

	AT91F_AIC_ConfigureIt (
		AT91C_BASE_AIC,                        // AIC base address
		AT91C_ID_US1,                          // System peripheral ID
		3,               // Max priority
		AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE, // Level sensitive
		AT91F_UART1_ASM_HANDLER );							
	AT91F_US_EnableIt((AT91PS_USART) AT91C_BASE_US1, 0x08); //Input of port 1： gimbal input
	AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_US1);
	
	
	//*******************	
	
	*/

	//***********UART 2//

	AT91F_AIC_ConfigureIt (
		AT91C_BASE_AIC,                        // AIC base address
		AT91C_ID_US2,                          // System peripheral ID
		4,               // Max priority
		AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE, // Level sensitive
		AT91F_UART2_ASM_HANDLER );							
	AT91F_US_EnableIt((AT91PS_USART) AT91C_BASE_US2, 0x08); 
	AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_US2); 

	//*******************	

	AT91F_AIC_ConfigureIt (
		AT91C_BASE_AIC,                        // AIC base address
		AT91C_ID_IRQ0,                         // System peripheral ID
		5,               // Max priority
		AT91C_AIC_SRCTYPE_EXT_POSITIVE_EDGE,   // posedge trigger
		AT91F_IRQ0_ASM_HANDLER );

	// external irq0 interrupt enable				
	AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);
	AT91C_BASE_SMC2->SMC2_CSR[2] = 0xff03cfff;

	AT91F_DataflashInit ();
	AT91_IIC_init();

	//**********************************
	//初始化发送帧的帧头
	//$STP
	for(ytemp=0;ytemp<4;ytemp++)
		SndBuf[ytemp+18]=mag_0[ytemp];
	//**************************************

	//************************************************
	oldsatelnum = 5;


	while(!AT91F_PDC_IsTxEmpty(AT91C_BASE_PDC_US0)) ;
	wait(1000000);
	AT91F_PDC_SetTx(AT91C_BASE_PDC_US0,setubx,28);		//setubx

	while(!AT91F_PDC_IsTxEmpty(AT91C_BASE_PDC_US0)) ;
	wait(1000000);
	AT91F_US_SetBaudrate ((AT91PS_USART)AT91C_BASE_US0, 59904000, 38400);	//普通GPS


	enable_msg[4] = 0x08;
	enable_msg[9] = 0x01;
	enable_msg[10] = 0x00;
	enable_msg[12] = 0x00;
	enable_msg[13] = 0x00;

	while(!AT91F_PDC_IsTxEmpty(AT91C_BASE_PDC_US0)) ;
	wait(1000000);
	
	//enable GPS messages
	for (i=1;i<=4;i++)
	{
		while(!AT91F_PDC_IsTxEmpty(AT91C_BASE_PDC_US0)) ;
		wait(10000);
		if (i==1)	enable_msg[7] = 0x06;
		if (i==2)	enable_msg[7] = 0x12;
		if (i==3)	enable_msg[7] = 0x02;
		if (i==4)	enable_msg[7] = 0x21;
		nCK_A = 0;
		nCK_B = 0;
		for(j=0;j<12;j++)
		{
			nCK_A = nCK_A + enable_msg[j+2];
			nCK_B = nCK_B + nCK_A;
		}
		enable_msg[14] = nCK_A;
		enable_msg[15] = nCK_B;

		AT91F_PDC_SetTx(AT91C_BASE_PDC_US0,enable_msg,16);		//enable msg
	}
	
	wait(1000000);
	while(!AT91F_PDC_IsTxEmpty(AT91C_BASE_PDC_US0)) ;
	AT91F_PDC_SetTx(AT91C_BASE_PDC_US0,initc,14);		//Set the output frequency of the GPS to 4Hz

	//Clear accel, gyro arrays
	for (i=0;i<60;i++)
	{
		voutp_zu[i] = 0;
	}




	read_dataflash (0xc003a000, 500, (char*)(&msg3[0]));	//Read dataflash, read PID values, yaw angle, and other configuration

	if (msg3[0]==255)	//If empty, read again
		read_dataflash (0xc003a000, 492, (char*)(&msg3[0]));

	if (msg3[0]!=255)	//If not empty, assign the values to variables
	{
		read_dataflash (0xc003a330, 6400, (char*)(&msg3[500]));

		read_dataflash (0xc003a258, 8, (char*)(&gyro_bias_data[0]));

		put_parameter();
		read_dataflash (0xc0077a00, 2, (char*)(&serialnum));

		read_dataflash (0xc0050000, 9624, (char*)(&msg2[0]));
		for(i=0;i<=60;i++)
		{
			buf2float(&vout1Mid[i],&msg2[24*i+0]);
			buf2float(&vout2Mid[i],&msg2[24*i+4]);
			buf2float(&vout3Mid[i],&msg2[24*i+8]);
			buf2int(&vout1count[i],&msg2[24*i+12]);
			buf2int(&vout2count[i],&msg2[24*i+16]);
			buf2int(&vout3count[i],&msg2[24*i+20]);



			buf2float(&vout4Mid[i],&msg2[24*i+0+2000]);
			buf2float(&vout5Mid[i],&msg2[24*i+4+2000]);
			buf2float(&vout6Mid[i],&msg2[24*i+8+2000]);
			buf2int(&vout4count[i],&msg2[24*i+12+2000]);
			buf2int(&vout5count[i],&msg2[24*i+16+2000]);
			buf2int(&vout6count[i],&msg2[24*i+20+2000]);

		}			

		buf2int(&temperature_high,&msg2[24*400+12]);
		buf2int(&temperature_low,&msg2[24*400+16]);
		if (temperature_high > 60) temperature_high = 60;
		if (temperature_low < 0) temperature_low = 0;


		read_dataflash (0xc0078000, 2, (char*)(&adnum[0]));
		cur_address = (unsigned char)adnum[1]*256 + (unsigned char)adnum[0];
		if (cur_address > 28611) cur_address = 0;

	}

	else	
	{
		errorflash = 1;
	}


	AT91F_US_SetBaudrate ((AT91PS_USART)AT91C_BASE_US2, 59904000, BAUD_YS);



	tgtLatti = tgt_Latti[1];		//Index starts from 0. Take off: 0; Target: 1
	tgtLongi = tgt_Longi[1];	
	tgtHigh = 30;
	AirHigh_h = 0;
	AirHigh_l = 30;
	tgtVel  = tgt_Vel[1];

	tgtLatti_s = tgt_Latti[0];	//Set the previous point as the take off point, create waypoint line from the previous point to the current target
	tgtLongi_s = tgt_Longi[0];

	nTrkPt = 0;


	for(i=1;i<10;i++) con[i]=150;

	if (real==0) AirHigh = 1;
	tTargetRoll = 0;		//Target Roll Angle = 0

	for (i=0;i<1501;i++)
	{
		vout1zu[i] = 0;
		vout2zu[i] = 0;
	}
	sumvout1 = 0;
	sumvout2 = 0;
	vout1out = 0;
	vout2out = 0;



	pres = spipn / 100;
	Manual_Mode = 1;

	hh = 0;
	Pres0 = pres;
	alt0 = 0;



	*pmem_poke	= 81; 




	while(1)	//main loop 
	{

		if (Parameter[STATE_SENSITIVITY]<50) Parameter[STATE_SENSITIVITY]=50;
		if (Parameter[STATE_SENSITIVITY]>80) Parameter[STATE_SENSITIVITY]=80;

		if ((in_flight)&&(Parameter[TEST_CONTROL_FREQ]==1))
			Parameter[TEST_CONTROL_FREQ] = 5;

		
		//开启、关闭 遥控器
		if (rc_enabled==1)
			SndBuf[RECEIVER_STATUS]=0;
		else
			SndBuf[RECEIVER_STATUS]=1;
			
		if (write_bias)
		{
			write_bias = 0;
			float2buf(&teshu[0],&bias_iraw_gyro_2);
			float2buf(&teshu[4],&bias_iraw_gyro_3);
			write_dataflash (0xc003a258, (long)(&teshu[0]), 8);
		}


		if (write_data_record==1)
		{
			write_data_record = 0;
			write_dataflash (0xc0078002+cur_address, (long)(&fashe2[0]), 99);
			cur_address += 99;
			if (cur_address >= 28611)
			{
				cur_address = 0;
			}

			adnum[1] = cur_address /256;
			adnum[0] = cur_address - adnum[1]*256;
			write_dataflash (0xc0078000, (long)(&adnum[0]), 2);

		}

		if ((Channel_6 > 1333)&&(Channel_6<1666)&&(Enable_Waypoint)&&(Channel_5>1666)&&(in_flight))			//Currently following waypoints
		{
			if (docalheight>=hz250/10)	//Gets incremented in irq0_handler, every 4*20ms, call doCal()
			{	
				docalheight = 0; 

				if (nTrkPt > 0)		//Not heading towards the first point
				{
					if ((tgt_High[nTrkPt-1]!=9999)&&(tgt_High[nTrkPt]!=9999))
					{	
						distance0 = distance_point(CurLatti,CurLongi,tgtLatti,tgtLongi);	//Distance of target from the current position in meters
						distance1 = distance_point(tgt_Latti[nTrkPt-1],tgt_Longi[nTrkPt-1],tgtLatti,tgtLongi);	//Distance of target from the previous target in meters

						distance_xishu = (1 - distance0/distance1);
						if (distance_xishu < 0) distance_xishu = 0;
	
					tgtHigh = (tgt_High[nTrkPt-1] + (float)(tgt_High[nTrkPt] - tgt_High[nTrkPt-1])*distance_xishu)*10;

					AirHigh_h = (int)tgtHigh / 256;
					AirHigh_l = (int)tgtHigh - AirHigh_h * 256;	
				}

			}
			else				//If heading towards the first point, the height doesn't need to be changed
			{

			}
			}
		}





		if (satelnumflag)				//If GPS signal is strong enough
			if (initflag)
				maincall();

		if ( (Parameter[CONTROL_METHOD]&0xf0) == 0x10)
			MAXacc = 0.3;
		else if ( (Parameter[CONTROL_METHOD]&0xf0) == 0x20)
			MAXacc = 0.4;
		else
			MAXacc = 0.2;


		if ((Pres0 > 1200)||(Pres0 < 170))
			pres_error = 1;
		else
			pres_error = 0;
		//Standard value 1013
		

		if (nTrkPt >= Waypoint_number) nTrkPt = 0;

		if ((satelnum > 4)&&(homelatti!=0))
		{
			home_distance = sqrt((homelongi - CurLongi)*111199*COS_FACTOR*(homelongi - CurLongi)*111199*COS_FACTOR + (homelatti - CurLatti)*111199*(homelatti - CurLatti)*111199);
		}
		else
			home_distance = 0;



		//move from irq0

		if (Parameter[ERASE_ROMBOOT]!=90)
		{	
			stgtLongi = tgtLongi;
			stgtLatti = tgtLatti;

			if ((showmag==0)&&(Parameter[TEST_CONTROL_FREQ]<=10))
			{
				float2buf(&OUT_INA_GPSHIGH,&stgtLongi);
			}

			if (Parameter[TEST_CONTROL_FREQ]<=10)
				float2buf(&OUT_INA_VELS,&stgtLatti);


			sxEKF3 = xEKF[3]*100;
			if (sxEKF3 < 0) sxEKF3 += 65536;
			SndBuf[VELOCITY_EAST_AFT_FIL_H] = (unsigned char)(sxEKF3/256);
			SndBuf[VELOCITY_EAST_AFT_FIL_L] = (unsigned char)(sxEKF3 - (unsigned char)(SndBuf[VELOCITY_EAST_AFT_FIL_H])*256);

			sxEKF4 = xEKF[4]*100;
			if (sxEKF4 < 0) sxEKF4 += 65536;
			SndBuf[VELOCITY_NORTH_AFT_FIL] = (unsigned char)(sxEKF4/256);
			SndBuf[VELOCITY_NORTH_AFT_FIL2] = (unsigned char)(sxEKF4 - (unsigned char)(SndBuf[VELOCITY_NORTH_AFT_FIL])*256);


			sxEKF5 = xEKF[5]*100;
			if (sxEKF5 < 0) sxEKF5 += 65536;
			SndBuf[VELOCITY_DOWN_AFT_FIL_H] = (unsigned char)(sxEKF5/256);
			SndBuf[VELOCITY_DOWN_AFT_FIL_L] = (unsigned char)(sxEKF5 - (unsigned char)(SndBuf[VELOCITY_DOWN_AFT_FIL_H])*256);

		}

		int2buf(&OUT_ADX_HENGGUN,&PlaneRoll);	//Put Roll angle in SndBuf, send data to GCS under $STP protocol
		int2buf(&OUT_ADX_FUYANG,&PlanePitch);	//Put Pitch angle in SndBuf, send data to GCS under $STP protocol
		




		idistance = home_distance;
		SndBuf[DIST_FROM_TAKE_OFF_HB] = idistance / 256;					//No. of captures (photos); high bit
		SndBuf[DIST_FROM_TAKE_OFF_LB] = idistance - SndBuf[DIST_FROM_TAKE_OFF_HB]*256;		//No. of captures; low bit

		temp_targetYaw = targetYaw * 10;
		if (temp_targetYaw < 0) temp_targetYaw += 3600;

		SndBuf[FLIGHT_CONTROL_VOL_2] = v12U/256;
		SndBuf[FLIGHT_CONTROL_VOL_1] = v12U - SndBuf[FLIGHT_CONTROL_VOL_2]*256;

		if (run_time < 4)
			clear_pressure = 1;

		power1_select = ((Parameter[SELECTION_OPTIONS]&0x0c) >> 2);				//00 00 11 00

		switch(power1_select)
		{
		case 0:	power1_xishu = 3.55;break;		
		case 1:	power1_xishu = 3.60;break;		
		case 2:	power1_xishu = 3.65;break;		
		case 3:	power1_xishu = 3.70;break;
		default : power1_xishu = 3.65;break;
		}

		power2_xishu = power1_xishu - 0.05;


		Parameter1_low = Parameter[POWER_NO] & 0x0f;

		low_power = (float)Parameter1_low*power1_xishu;
		emergency_power = (float)Parameter1_low*power2_xishu;

		VEL = (float)Parameter[MAX_VELOCITY]*4;


		if ((Channel_5>1666)&&(Channel_6 > 1666)&&(pregohome==0)&&(in_flight==1))		//Under Automatic mode, in "Go home" mode
		{

			if ((tgtHigh < 200)&&(AirHighf*10<200))
			{
				tgtHigh = 200;
				AirHigh_h = (int)tgtHigh / 256;
				AirHigh_l = (int)tgtHigh - AirHigh_h * 256;	
			}
			else if (pregohome==0)
			{
				tgtHigh = AirHighf*10;
				AirHigh_h = (int)tgtHigh / 256;
				AirHigh_l = (int)tgtHigh - AirHigh_h * 256;	
			}

			auto_stay_ding = 1;
			if (Enable_Waypoint==1)
			{
				Enable_Waypoint = 0;
			}

			pregohome = 1;
			gimbal_target_lock = 0;

		}

		if (gohome==1)
		{
			auto_stay_ding = 1;
			if (Enable_Waypoint==1)
			{
				Enable_Waypoint = 0;
			}

			if (gohome_end==0)
			{
				tgtLatti = homelatti;
				tgtLongi = homelongi;
				ClickSetTgtFlag = 1;
			}
			else
				ClickSetTgtFlag = 0;			

			if ((tgtLongi - CurLongi)*111199*COS_FACTOR*(tgtLongi - CurLongi)*111199*COS_FACTOR + (tgtLatti - CurLatti)*111199*(tgtLatti - CurLatti)*111199 < 10*10)
				gohome_end = 1;

		}

		if (bAutoHand_)
		{
			if (pregohome)
			{
				if (gohome_end==0)	//While going home, set the heading direction correctly
				{
					vectorLatti = CurLatti - homelatti;
					vectorLongi = CurLongi - homelongi;
					mainYaw = CalAtanVector();	  
				}
			}
			else
				if ((gimbal_target_latti!=0)&&(gimbal_target_longi!=0)&&(gimbal_target_lock==1))
				{

					//径向速度得是右移速度的1/3
					ddx = (gimbal_target_longi - CurLongi)*111199*COS_FACTOR;
					ddy = (gimbal_target_latti - CurLatti)*111199;

					distanceR = (sqrt(ddx*ddx + ddy*ddy) - gimbal_target_R)/5;

					distance_maxV = (float)Parameter[MAX_VELOCITY]/90 * 1.5;

					if (distanceR > distance_maxV) distanceR = distance_maxV;
					if (distanceR < -distance_maxV) distanceR = -distance_maxV;

					vectorLatti = gimbal_target_latti - CurLatti;
					vectorLongi = gimbal_target_longi - CurLongi;
					mainYaw = CalAtanVector();	  

					mainYaw += (float)(FX - Rudder_MID) * 45 / 400 * rad;	//45 degrees on one side

					if (mainYaw > PI) mainYaw -= 2*PI;
					if (mainYaw < -PI) mainYaw += 2*PI;
				}
				else
					if ((Channel_6 > 1333)&&(Enable_Waypoint))			//GPS waypoints
					{
						if (heading_stop==0)	//Not reached target yet
						{
							
							if ((( fabsf(tgtLongi_s - tgtLongi) > 0.000100 ) || ( fabsf(tgtLatti_s - tgtLatti) > 0.000100)))
							{
								vectorLatti = tgtLatti - tgtLatti_s;
								vectorLongi = tgtLongi - tgtLongi_s;
							}
							else
							{
								vectorLatti = tgtLatti - CurLatti;
								vectorLongi = tgtLongi - CurLongi;
							}
							mainYaw = CalAtanVector();	 
						}
					}
					else if ((Channel_6 < 1333)||(   (Channel_6>1333)&&(Channel_6<1666)&&(!Enable_Waypoint)             ))
					{
						if (ClickSetTgtFlag==1)
						{
							vectorLatti = tgtLatti - CurLatti;
							vectorLongi = tgtLongi - CurLongi;
							mainYaw = CalAtanVector();	  
						}
					}
		}

		if ((auto_stay_ding==0)&&(!ClickSetTgtFlag)&&(bAutoHand_))		//Waypoint mode, needs calibration of velocity direction
		{

			if ((( fabsf(tgtLongi_s - tgtLongi) > 0.000100 ) || ( fabsf(tgtLatti_s - tgtLatti) > 0.000100))) //如果上一点坐标有效，且考虑进入第一点时不贴航线
			{

				lDistance = fabsf(newdistance());

				vectorLatti = tgtLatti - CurLatti;
				vectorLongi = tgtLongi - CurLongi;
				beta = CalAtanVector();	 //The arctan of the line connecting the current point and the target point with a line facing north

				vectorLatti = tgtLatti - tgtLatti_s;
				vectorLongi = tgtLongi - tgtLongi_s;

				beta2 = CalAtanVector();  //arctan of waypoint line and a line facing north

				tempj1 = beta - beta2;
				if (tempj1 > PI)  tempj1 = tempj1 - 2*PI;	//Range: -pi ~ +pi
				if (tempj1 < -PI) tempj1 = 2*PI + tempj1;

				if (tempj1 < 0)	//If the point is on the right of the waypoint line
				{
					beta = beta2 - PI/2;
				}
				else					//Left of waypoint line
				{
					beta = beta2 + PI/2;
				}
				if (beta > PI) beta = beta - 2*PI;
				if (beta < -PI) beta = 2*PI + beta;

				vel_1 = lDistance * (float)36;		//cm/s


				if (vel_1 > 400)	//>150 cm/s
					vel_1 = 400;

				navigx = vel_1 * sin(beta);	//cm/s
				navigy = vel_1 * cos(beta);
			}

		}


		if (mainpres>=1)
		{
			mainpres = 0;
			press_val2 = 0;
			for (i=0;i<ctair2;i++)
			{
				press_val2 += press_data2[i];
			}

			if (ctair2 > 0)
			{
				press_val2 /= (double)ctair2;


				showpress_val2 = press_val2;


				ctair2 = 0;

				yhpres2 = press_val2/100/Pres0;

				if (yhpres2 > 0)
					hh2 = 0.190263107*log(yhpres2);


				AirHighf = -1 * (alt0 - (1-exp(hh2))*44330.7694);
				fAirHigh = AirHighf;									
				AirHigh = AirHighf;
			}
		}

		if (mainmag==1)
		{
			mainmag = 0;
			*pmem_mag_cs = 0;



			magdata = *pmem_magdata;
			magdata2 = *pmem_magdata2;
			magdata3 = *pmem_magdata3;
			magdata4 = *pmem_magdata4;
			magdata5 = *pmem_magdata5;
			magdata6 = *pmem_magdata6;

			*pmem_mag_cs = 1;
			xmag = magdata*256 + magdata2;
			zmag = magdata3*256 + magdata4;
			throtag = magdata5*256 + magdata6;

			if (xmag > 32768) xmag -= 65536;
			if (throtag > 32768) throtag -= 65536;
			if (zmag > 32768) zmag -= 65536;



			for (i=MAG_NUM;i>0;i--)
			{
				mag_1_zu[i] = mag_1_zu[i-1];
				mag_2_zu[i] = mag_2_zu[i-1];
				mag_3_zu[i] = mag_3_zu[i-1];
			}

			mag_1_zu[0] = xmag;
			mag_2_zu[0] = throtag;
			mag_3_zu[0] = zmag;

			sum_raw_mag_1 = 0;
			sum_raw_mag_2 = 0;
			sum_raw_mag_3 = 0;
			for(i=0;i<MAG_NUM;i++)
			{
				sum_raw_mag_1 += mag_1_zu[i];
				sum_raw_mag_2 += mag_2_zu[i];
				sum_raw_mag_3 += mag_3_zu[i];
			}				

			xmag = sum_raw_mag_1 / (float)MAG_NUM;
			throtag = sum_raw_mag_2 / (float)MAG_NUM;
			zmag = sum_raw_mag_3 / (float)MAG_NUM;

			if (showmag==1)
			{
				if ((Parameter[ERASE_ROMBOOT]!=87)&&(Parameter[ERASE_ROMBOOT]!=88)&&(Parameter[ERASE_ROMBOOT]!=89)&&(Parameter[ERASE_ROMBOOT]!=90))
				{
					float2buf(&OUT_INA_LONGI,&xmag);
					float2buf(&OUT_INA_LATTI,&throtag);
					float2buf(&OUT_INA_GPSHIGH,&zmag);
				}
				else if (Parameter[ERASE_ROMBOOT]==87)
				{
					float2buf(&OUT_INA_LONGI,&xmag_max);
					float2buf(&OUT_INA_LATTI,&xmag_min);
				}
				else if (Parameter[ERASE_ROMBOOT]==88)
				{
					float2buf(&OUT_INA_LONGI,&zmag_max);
					float2buf(&OUT_INA_LATTI,&zmag_min);
				}
			}

			cal_mag();
		}


		SndBuf[HOVER_THROT_POS] = (unsigned char)(throtMid/10);

		if ((Parameter[ERASE_ROMBOOT]==87)||(Parameter[ERASE_ROMBOOT]==88)||(Parameter[CONTROL_METHOD]==0)) showmag = 1;
		else showmag = 0;


		inwhile1 = 1;



		switch (Parameter[DRONE_TYPE])
		{
		case 0: DroneMode=4;versionX=10;break;
		case 1: DroneMode=4;versionX=0;break;
		case 2: DroneMode=6;versionX=10;break;
		case 3: DroneMode=6;versionX=0;break;
		case 4: DroneMode=8;versionX=10;break;
		case 5: DroneMode=8;versionX=0;break;

		case 6: DroneMode=3;versionX=10;break;		//Y3	
		case 7: DroneMode=3;versionX=0;break;			//Y3
		case 8: DroneMode=8;versionX=20;break;			//4轴8电机

		case 10: DroneMode=8;versionX=99;break;			//自定义

		case 11: DroneMode=6;versionX=11;break;			//异形

		}

		outhz = 400;



		ESC_select = ((Parameter[SELECTION_OPTIONS]&0x30) >> 4);				//00 11 00 00

		outwidth = 2000000/outhz-1;
		//400HZ
		tmphh = outwidth / 256;
		tmpll = outwidth - tmphh*256;
		*pmem_width3h = tmphh;					//1~9通道
		*pmem_width3l = tmpll;



		PTZ_style = ((Parameter[SELECTION_OPTIONS]&0xc0)>>6);
		if (PTZ_style==0)
			outhz = 50;
		else if (PTZ_style==1)
			outhz = 250;
		else
			outhz = 333;


		if (DroneMode==8)
			outhz = 400;

		outwidth = 2000000/outhz-1;
		//Gimbal
		tmphh = outwidth / 256;
		tmpll = outwidth - tmphh*256;
		*pmem_width2h = tmphh;					//1~9 channels
		*pmem_width2l = tmpll;
		//10~11通道固定为250HZ或者330HZ

		if ((run_time >= 5)&&(eraseromboot==1)&&(tgtHigh == 9999))	//Can erase program
		{
			erase_dataflash(1);
			eraseromboot=0;
		}



		if (writebuf1 == 1)		//If writebuf is set to 1, parameters are written into dataflash
		{
			do_writebuf1();
		}
		if (writebuf2 == 1)		//If writebuf is set to 1, parameters are written into dataflash
		{
			do_writebuf2();
		}



		SndBuf[CURRENT_PROD_NUMBER_2] = (int)(firmware/256);
		SndBuf[CURRENT_PROD_NUMBER_1] = (int)(firmware - SndBuf[CURRENT_PROD_NUMBER_2]*256);


		if (clear_pressure==1)			//Clear pressure values, clear height value
		{

			enable_ctclearpres = 1;
			if (ctclearpres >= (hz250/50))
			{
				clear_pressure = 0;
				enable_ctclearpres = 0;
				ctclearpres = 0;
			}

			hh = log(1-GPShigh/44330.7694);
			Pres0 = clear_pres_val/100*exp(-hh/0.190263107);

			alt0 = GPShigh;


		}



		if ((bAutoHand_)&&(auto_stay_ding==0))
		{
			bReachTgt = 0; 

			if ((bAutoHand_)&&(auto_stay_ding==0))	//If automatic
				bReachTgt = PointArrive();	//Check whether already reached the target point

			if (bReachTgt) {heading_stop = 1;} else heading_stop = 0;



			if (bReachTgt)
				if (!start_hover)
				{
					hover_time = (int)(tgt_hover_time[nTrkPt])*hz250;		
					start_hover = 1;
					//						ct_hover_time = 0;
				}


				HaveCalculateArrive = 0;
		}
		else if ((bAutoHand_)&&(auto_stay_ding==1)&&(ClickSetTgtFlag))
		{
			if (PointArrive())
				ClickSetTgtFlag = 0;
		}







		SndBuf[TOTAL_UPLOADED_WAYPTS] = Waypoint_number;			




		if (!show_serial)
		{
			SndBuf[START_TIME_2] = (unsigned char)(run_time/256);
			OUT_HIGH_P = (unsigned char)(run_time - (unsigned char)(SndBuf[START_TIME_2])*256);
		}
		else
		{
			SndBuf[START_TIME_2] = (unsigned char)(serialnum/256);
			OUT_HIGH_P = (unsigned char)(serialnum - (unsigned char)(SndBuf[START_TIME_2])*256);
		}



		if (Parameter[ERASE_ROMBOOT]==1)		
		{
			Parameter[ERASE_ROMBOOT]=0;
			eraseromboot = 1;		
		}

		th = tgtHigh;
		if (th < 0) th+=65536;

		SndBuf[TARGET_HEIGHT_2] = (unsigned char)(th/256);						
		OUT_ARM_REFHIGH = (unsigned char)(th - SndBuf[TARGET_HEIGHT_2]*256);


		OUT_ARM_TARGET = nTrkPt;	//Write the waypoint into the buffer //GGPS




		if (!ClickSetTgtFlag)	
		{
			if (auto_stay_ding==0)							//in waypoint mode
			{
				tgtLatti = tgt_Latti[nTrkPt];		//Set target point as waypoint target
				tgtLongi = tgt_Longi[nTrkPt];
			}


			th = AirHigh_h*256+AirHigh_l;
			if (th < 32768)
				tgtHigh = th;			
			else
				tgtHigh = th - 65536;

			tgtVel  = tgt_Vel[nTrkPt];
		}

		else
		{
			th = AirHigh_h*256+AirHigh_l;
			if (th < 32768)
				tgtHigh = th;			
			else
				tgtHigh = th - 65536;


		}


		if ((count_current>=hz250/10)&&(twi_error==0))
		{

			iic_wrbuf[0]=0xc0;
			AT91F_TWI_Write(AT91C_BASE_TWI, 0x0, (char *)&iic_wrbuf[0], 1);	
			wait(10000);		
			AT91F_TWI_Read(AT91C_BASE_TWI, 0x0, (char *)&iic_rdbuf[4], 2);

			if (twi_error) goto twi_out;


			if (!(iic_rdbuf[4] & 0x80))
			{
				which = iic_rdbuf[4] & 0x70;
				which = which >> 4;

				if (which == 4)
				{
					vCURRENTU =((unsigned char)iic_rdbuf[5]+((unsigned char)iic_rdbuf[4]&0x0f)*256);
				}
			}




			if (fabsf(vCURRENTU - old_vCURRENTU) > 300)		//Error
			{
				ccterror_vcurrent ++;
				if (ccterror_vcurrent >= 3)
				{
					old_vCURRENTU = vCURRENTU;
				}
				else
				{
					vCURRENTU = old_vCURRENTU;
				}
			}
			else
			{
				old_vCURRENTU = vCURRENTU;
				ccterror_vcurrent = 0;
			}

			vcurrent = (float)vCURRENTU * 5 / 4096;

			if ((THROT > 1900)&&(Channel_5 < 1333)&&(init_Current==0)&&(run_time>4))
			{
				init_Current = 1;
				CURRENT_ZEROV = vcurrent;
				BatteryConsume = 0;
			}

			CurrentI = (vcurrent - CURRENT_ZEROV)*25;
			if (CurrentI < 0) CurrentI = 0;
			BatteryConsumeI = (int)(BatteryConsume/3.6);

			CurrentII = CurrentI;
			SndBuf[PRESENT_CURRENT] = CurrentII;


twi_out:;
			if (twi_error)
			{
				count_error ++;
				if (count_error < 10)
					twi_error = 0;
				else
					count_error = 10;
			}
			else
				count_error = 0;

			count_current = 0;

		}


		if (count_AD >= hz250/2)
		{
			if (twi_error==0)
				rdTWIad();			//If AD7998 is soldered onto the board, this function should be called to display voltage
								//If not soldered, this needs to commented out.


			if (twi_error)
			{
				count_error ++;
				if (count_error < 10)
					twi_error = 0;
				else
					count_error = 10;
			}
			else
				count_error = 0;

			count_AD = 0;
		}
	}

}


void doCal(void)	
{ 
	float ddx,ddy,ddl,vel_1;
	float target_velx=0,target_vely=0;
	int itarget_velx=0,itarget_vely=0;
	float rcal,pcal;
	float sumtemp ;
	int i,j;
	if (auto_stay_ding==1)											//Manual mode
	{
		if ((fabsf(elev-Elevator_MID)>30)||(fabsf(ail-Aileron_MID)>30))
		{
			action = 1;

			target_velx = 0;
			target_vely = 0;

			if (fabsf(elev-Elevator_MID)>30)
			{
				if (gimbal_target_lock==1)
				{

					if (elev > Elevator_MID)
						vel_1 = (float)(-elev+Elevator_MID+30)*(float)Parameter[MAX_VELOCITY]*4/400;
					else
						vel_1 = (float)(-elev+Elevator_MID-30)*(float)Parameter[MAX_VELOCITY]*4/400;

					if (vel_1 > Parameter[MAX_VELOCITY]*4) vel_1 = Parameter[MAX_VELOCITY]*4;
					if (vel_1 < -Parameter[MAX_VELOCITY]*4) vel_1 = -Parameter[MAX_VELOCITY]*4;

					target_velx += vel_1 * sin_hx;
					target_vely += vel_1 * cos_hx;


					ddx = (gimbal_target_longi - CurLongi)*111199*COS_FACTOR;
					ddy = (gimbal_target_latti - CurLatti)*111199;

					gimbal_target_R = sqrt(ddx*ddx + ddy*ddy);

					if (gimbal_target_R < 5) gimbal_target_R = 5;
					if (gimbal_target_R > 200) gimbal_target_R = 200;

				}
				else
				{
					if ((gohome)&&(gohome_end))
					{
						if (elev > Elevator_MID)
							vel_1 = (float)(-elev+Elevator_MID+30)*0.9;
						else
							vel_1 = (float)(-elev+Elevator_MID-30)*0.9;
					}
					else
					{
						if (elev > Elevator_MID)
							vel_1 = (float)(-elev+Elevator_MID+30)*(float)Parameter[MAX_VELOCITY]*4/400;
						else
							vel_1 = (float)(-elev+Elevator_MID-30)*(float)Parameter[MAX_VELOCITY]*4/400;
					}

					if (vel_1 > Parameter[MAX_VELOCITY]*4) vel_1 = Parameter[MAX_VELOCITY]*4;
					if (vel_1 < -Parameter[MAX_VELOCITY]*4) vel_1 = -Parameter[MAX_VELOCITY]*4;

					if (carefree)
					{
						target_velx += vel_1 * sin_care;
						target_vely += vel_1 * cos_care;
					}
					else
					{
						target_velx += vel_1 * sin_hx;
						target_vely += vel_1 * cos_hx;
					}
				}
			}

			if (fabsf(ail-Aileron_MID)>30)
			{


				if ((gohome)&&(gohome_end))
				{
					if (ail > Aileron_MID)
						vel_1 = (float)(ail-Aileron_MID-30)*0.9;
					else
						vel_1 = (float)(ail-Aileron_MID+30)*0.9;
				}
				else
				{
					if (ail > Aileron_MID)
						vel_1 = (float)(ail-Aileron_MID-30)*(float)Parameter[MAX_VELOCITY]*4/400;
					else
						vel_1 = (float)(ail-Aileron_MID+30)*(float)Parameter[MAX_VELOCITY]*4/400;
				}						
				if (vel_1 > Parameter[MAX_VELOCITY]*4) vel_1 = Parameter[MAX_VELOCITY]*4;
				if (vel_1 < -Parameter[MAX_VELOCITY]*4) vel_1 = -Parameter[MAX_VELOCITY]*4;


				if (carefree)
				{
					target_velx += vel_1 * cos_care;
					target_vely += vel_1 * (-sin_care);
				}
				else
				{
					target_velx += vel_1 * cos_hx;
					target_vely += vel_1 * (-sin_hx);
				}						

				if (gimbal_target_lock==1)
				{

					target_velx += distanceR * 100 * sin_hx;
					target_vely += distanceR * 100 * cos_hx;

tgjl:;

				}
			}

			target_caled = 0;
			vel_caled = 1;


		}
		else			//R/C in neutral position, unmoved
		{

			if (action == 1)
			{
				rr[0] = (float)tvely_i/100*cos_hx+(float)tvelx_i/100*sin_hx;
				rr[1] = (float)tvelx_i/100*cos_hx-(float)tvely_i/100*sin_hx;
				stopflag = 1;
			}
			action = 0;

			if ((gohome==1)&&(gohome_end==0))		//ys-x10
			{
				target_caled = 1;
				vel_caled = 0;
			}			 


			if (target_caled==0)			//RC stick moved
			{
				target_vely = 0;
				target_velx = 0;
				vel_caled = 1;

				if ((fabsf(rr[0])<0.1)&&(fabsf(rr[1])<0.1))
				{



					if (stopflag!=0)
					{
						stopflag = 0;
						stoptimecount = sqrt(tvely_i*tvely_i + tvelx_i*tvelx_i) / 0.8 / 100 * hz250;
					}
					else
						stoptimecount --;

					if (stoptimecount <= 0)
					{
						stoptimecount = 0;
						tgtLongi = CurLongi;
						tgtLatti = CurLatti;

						target_caled = 1;
						vel_caled = 0;
					}
				}
			}
		}


	}
	else
		vel_caled = 0;

	if (vel_caled==0)
	{
		ddx = (tgtLongi - CurLongi)*111199*COS_FACTOR;
		ddy = (tgtLatti - CurLatti)*111199;

		ddl = sqrt(ddx*ddx+ddy*ddy);

		vel_1 = ddl * 36;		//Unit: cm/s

		if (vel_1 > Parameter[MAX_VELOCITY]*4)	//>150 cm/s
			vel_1 = Parameter[MAX_VELOCITY]*4;
		if (( tgt_hover_time[nTrkPt]==0)&&(ClickSetTgtFlag!=1)&&(auto_stay_ding==0))	
		{

			limit_vel = 0;

		}
		else
		{


			if ((gohome==0)&&(ddl < 15))
			{
				//					limit_vel = 1;		//限制速度
			}
			else
				if ((gohome==1)&&(ddl < 15))
				{
					if (ddl < 10)
						limit_vel = 2;
					else
						limit_vel = 1;
				}
				else
					limit_vel = 0;


		}


		if (limit_vel)
		{
			if (limit_vel==1)
			{
				if (vel_1 > 220)
					vel_1 = 220;
			}
			else if (limit_vel==2)
			{
				if (vel_1 > 135)
					vel_1 = 135;
			}
		}

		if (ddl!=0)
		{
			target_velx = vel_1 * ddx / ddl;	//单位为厘米/秒
			target_vely = vel_1 * ddy / ddl;
		}
		else
		{
			target_velx = 0;
			target_vely = 0;
		}

		if ((auto_stay_ding==0)&&(!ClickSetTgtFlag))		//Following waypoints, correcting velocity direction
		{

			if ((( fabsf(tgtLongi_s - tgtLongi) > 0.000100 ) || ( fabsf(tgtLatti_s - tgtLatti) > 0.000100))) 
			{
				target_velx += navigx;
				target_vely += navigy;
			}

		}

		if (!in_flight)
		{
			target_velx = 0;
			target_vely = 0;
		}
	}


	trr[0] = target_vely/100;
	trr[1] = target_velx/100;
	LimitAccel(trr);

	target_vely = rr[0]*100*cos_hx-rr[1]*100*sin_hx;
	target_velx = rr[0]*100*sin_hx+rr[1]*100*cos_hx;

	itarget_velx = (int)target_velx;
	itarget_vely = (int)target_vely;

	if (itarget_velx < 0) itarget_velx += 65536;
	if (itarget_vely < 0) itarget_vely += 65536;



	sumtemp = (target_velx - xEKF[4]*100);
	if (sumtemp > 50) sumtemp = 50;
	if (sumtemp < -50) sumtemp = -50;
	sumx += sumtemp/(hz250/4);

	sumtemp = (target_vely - xEKF[3]*100);
	if (sumtemp > 50) sumtemp = 50;
	if (sumtemp < -50) sumtemp = -50;
	sumy += sumtemp/(hz250/4);

	if (sumx > Parameter[VEL_STATE_RANGE_I]*20) sumx = Parameter[VEL_STATE_RANGE_I]*20;
	if (sumx < -Parameter[VEL_STATE_RANGE_I]*20) sumx = -Parameter[VEL_STATE_RANGE_I]*20;
	if (sumy > Parameter[VEL_STATE_RANGE_I]*20) sumy = Parameter[VEL_STATE_RANGE_I]*20;
	if (sumy < -Parameter[VEL_STATE_RANGE_I]*20) sumy = -Parameter[VEL_STATE_RANGE_I]*20;

	rcal = (target_velx/100 - xEKF[4]) * 0.2;
	pcal = (target_vely/100 - xEKF[3]) * 0.2;

	if (rcal > MAXacc) rcal = MAXacc;
	if (rcal < -MAXacc) rcal = -MAXacc;
	if (pcal > MAXacc) pcal = MAXacc;
	if (pcal < -MAXacc) pcal = -MAXacc;


	calailacc = rcal*cos_hx - pcal*sin_hx;
	calelevacc = pcal*cos_hx + rcal*sin_hx;


	tdocal_TargetRoll = rcal*cos_hx - pcal*sin_hx;
	if (tdocal_TargetRoll > Parameter[TARGET_STATE]) tdocal_TargetRoll = Parameter[TARGET_STATE];
	if (tdocal_TargetRoll < -Parameter[TARGET_STATE]) tdocal_TargetRoll = -Parameter[TARGET_STATE];

	tdocal_TargetPitch = pcal*cos_hx + rcal*sin_hx;
	if (tdocal_TargetPitch > Parameter[TARGET_STATE]) tdocal_TargetPitch = Parameter[TARGET_STATE];
	if (tdocal_TargetPitch < -Parameter[TARGET_STATE]) tdocal_TargetPitch = -Parameter[TARGET_STATE];

	if (in_flight==0)
	{
		tdocal_TargetRoll = 0;
		tdocal_TargetPitch = 0;
	}


}

float CalFxWidth(void)
{
	//<150是左舵
	//>150是右舵


	float delta_PlaneYaw,Dvalue;
	int testfx;
	float targetYawvel;
	float ddx,ddy;

	testfx = (float)(Parameter[ROLL_D])*150/60;

	tgyro3 = Butterworth(alldata.ina_gyro3, zgyro3);


	Dvalue = (float)(Parameter[YAW_D])/128*(tgyro3);		//when d_gyro_2 is positive, copter tends to rotate counter-clockwise

	if (!bAutoHand_)			//if manual
	{

		if ((fabsf((int)FX-(int)Rudder_MID)<30)&&(in_flight))
			sumheading += (tgyro3*8/hz250);


		nCalFxWidth =  Dvalue*10 + (float)((int)FX-(int)Rudder_MID)*10/(float)Parameter[YAW_RC_COEFF];




		if (nCalFxWidth > testfx)	nCalFxWidth = testfx;
		if (nCalFxWidth < -testfx)	nCalFxWidth = -testfx;

		nCalFxWidth += sumheading;

		if (nCalFxWidth > 250)	nCalFxWidth = 250;
		if (nCalFxWidth < -250)	nCalFxWidth = -250;
	}
	else
	{
		if ((pregohome==1)&&(gohome_end==0))	//going back
		{
			t_ref_yaw = mainYaw;	//calculate
			LimitCourseAngle(t_ref_yaw);
			targetYaw = ref_yaw * 180/PI;
		}
		else if (gimbal_target_lock==1)		//Locking target
		{


			t_ref_yaw = mainYaw;	//calculate
			LimitCourseAngle(t_ref_yaw);
			targetYaw = ref_yaw * 180/PI;
		}
		else if ((Channel_6 > 1333)&&(Channel_6<1666)&&(!gohome)&&(Enable_Waypoint)&&(Waypoint_carefree==0))	//Not going back, following waypoints
		{
			if (heading_stop==0)	//Not reached waypoint 
			{
				t_ref_yaw = mainYaw;	//calculate
				LimitCourseAngle(t_ref_yaw);
				targetYaw = ref_yaw * 180/PI;
			}
		}
		else						//GPS hovering, or point and fly
		{
			if (ClickSetTgtFlag==1)	//Point and fly
			{
				gfx = 0;

				t_ref_yaw = mainYaw;	//calculate
				LimitCourseAngle(t_ref_yaw);

				targetYaw = ref_yaw * 180/PI;
			}
			else					//GPS hovering
			{
				if (fabsf(FX-Rudder_MID) > 30)
				{
					gfx = 1;
					ngfx = 1;
					if (FX-Rudder_MID>30)
						nCalFxWidth =  Dvalue*10 + (float)(FX-Rudder_MID-30)*10/(float)Parameter[YAW_RC_COEFF];
					else
						nCalFxWidth =  Dvalue*10 + (float)(FX-Rudder_MID+30)*10/(float)Parameter[YAW_RC_COEFF];


					if (nCalFxWidth > testfx)	nCalFxWidth = testfx;
					if (nCalFxWidth < -testfx)	nCalFxWidth = -testfx;

					nCalFxWidth -= (sumfx*10);

					if (nCalFxWidth > 250)	nCalFxWidth = 250;
					if (nCalFxWidth < -250)	nCalFxWidth = -250;
				}
				else
				{
					if (gfx==1)
					{
						gfx = 0;
						targetYaw = scalheading * 180/PI - (tgyro3/5);
						ref_yaw = targetYaw * PI/180;
					}
				}
			}
		}

		if (targetYaw > 180) targetYaw -= 360;
		if (targetYaw < -180) targetYaw += 360;
		delta_PlaneYaw = -targetYaw + scalheading*180/PI;
		if (delta_PlaneYaw > 180) delta_PlaneYaw -= 360;
		if (delta_PlaneYaw < -180) delta_PlaneYaw += 360;

		targetYawvel = (float)(Parameter[YAW_P])/64*delta_PlaneYaw;								//If positive, turn left
		if (targetYawvel > 90) targetYawvel = 90;
		if (targetYawvel < (-90)) targetYawvel = (-90);


		if (  ((fabsf(delta_PlaneYaw) < 10)||(home_distance<5)) &&(pregohome==1)&&(fabsf(tgtHigh/10 - AirHighf)<2))
			gohome = 1;


		if ((bAutoHand_)&&(gfx!=1))
			sumfx += ((targetYawvel - tgyro3)/(float)Parameter[YAW_I]/(hz250/50)/3);	//If positive, turn left

		if (sumfx > Parameter[YAW_I_RANGE]) sumfx = Parameter[YAW_I_RANGE];
		if (sumfx < -Parameter[YAW_I_RANGE]) sumfx = -Parameter[YAW_I_RANGE];

		if (nospin==1) sumfx = 0;

		if (ngfx!=1)
		{

			nCalFxWidth =  -(targetYawvel - tgyro3)*(float)(Parameter[YAW_D])/128*10;


			if (nCalFxWidth > testfx)	nCalFxWidth = testfx;
			if (nCalFxWidth < -testfx)	nCalFxWidth = -testfx;

			nCalFxWidth -= (sumfx*10);

			if (nCalFxWidth > 250)	nCalFxWidth = 250;
			if (nCalFxWidth < -250)	nCalFxWidth = -250;
		}
		else
			ngfx = 0;
	}



	OUT_ATL_FXDUO = (Rudder_MID + nCalFxWidth)/10;

	return nCalFxWidth;

}

float CalailWidth(void)
{
	float delta_PlaneRoll,Dvalue,Pvalue;
	float Targetacc,acc_cha;
	float target_vel,vel_error;
	float dvel;
	float rcal;
	//Roll angle, Right: negative, Left: Positive

	if (!bAutoHand_)
	{
		if (( (Parameter[CONTROL_METHOD]&0x0f)  ==1)||(in_flight==0))
			TargetRoll = -(float)(ail - Aileron_MID)/(float)Parameter[RC_TARGET_STATE_COEFF] + TargetRollBias;
		else
		{

			rcal = ( - xEKF[4]) * 0.2;
			if (rcal > MAXacc) rcal = MAXacc;
			if (rcal < -MAXacc) rcal = -MAXacc;
			Targetacc = rcal;
			Targetacc = 0;

			acc_cha = Targetacc - acc_ail;

			if (fabsf(ail-Aileron_MID)>30) 
			{
				acc_cha = 0;
				oper_ail = 1;
				TargetRollMid = 0;
			}
			else
			{
				if (oper_ail)
				{
					xEKF[4] = 0;
					acc_cha = - acc_ail;	
				}
				oper_ail = 0;
			}




			TargetRollMid = 0;

			TargetRoll = TargetRollMid - ((float)Parameter[STATE_SENSITIVITY]/4/0.2)* acc_cha -(float)(ail - Aileron_MID)/(float)Parameter[RC_TARGET_STATE_COEFF] + TargetRollBias;


			if (TargetRoll > 35) TargetRoll = 35;
			if (TargetRoll < -35) TargetRoll = -35;

			TargetRollMid -= ((float)2/hz250/0.1)*acc_cha;				

			if (TargetRollMid > 20) TargetRollMid = 20;
			if (TargetRollMid < -20) TargetRollMid = -20;
		}
	}
	else
	{
		Targetacc = calailacc;		//Target Acceleration (Right)

		acc_cha = Targetacc - acc_ail;

		TargetRoll = TargetRollMid - ((float)Parameter[STATE_SENSITIVITY]/4/0.2)* acc_cha + TargetRollBias;


		if ((!in_flight)&&(!start_center_gravity))
			TargetRoll = -(float)(ail - Aileron_MID)/(float)Parameter[RC_TARGET_STATE_COEFF] + TargetRollBias;



		if (TargetRoll > 35) TargetRoll = 35;
		if (TargetRoll < -35) TargetRoll = -35;

		if ((in_flight)||(start_center_gravity))
		{
			TargetRollMid = sum_anglex * cos_hx - sum_angley * sin_hx;
			TargetRollMid -= ((float)2/hz250/0.1)*acc_cha;				//Should be alterable. Unstable in strong winds due to high I value?
		}
		else
			TargetRollMid = 0;



		if (TargetRollMid > 20) TargetRollMid = 20;
		if (TargetRollMid < -20) TargetRollMid = -20;


	}

	delta_PlaneRoll = t_PlaneRoll - TargetRoll;




	target_vel =  delta_PlaneRoll*3;								//Target angular acceleration

	if (target_vel > 50) target_vel = 50;
	if (target_vel < -50) target_vel = -50;


	vel_error = (alldata.ina_gyro2 - target_vel);					//Angular acceleration error
	if (vel_error > 120) vel_error = 120;
	if (vel_error < -120) vel_error = -120;



	if (Enable_Control_Integrate==1)
	{

		if (fabsf(vel_error) <= 10)
			sumail += (vel_error/(float)150/2/(hz250/50));
		else if (vel_error > 10)
			sumail += 10/(float)150/2/(hz250/50);
		else if (vel_error < -10)
			sumail += -10/(float)150/2/(hz250/50);

	}
	else
	{
		sumail = 0;
		if (((fabsf(xEKF[3])>0.1)||(fabsf(xEKF[4])>0.1))&&(Enable_Control_Integrate_pre))
			Enable_Control_Integrate = 1;
	}		

	if (sumail > Parameter[STATE_CHANNEL_RANGE_I]) sumail = Parameter[STATE_CHANNEL_RANGE_I]; 
	if (sumail < -Parameter[STATE_CHANNEL_RANGE_I]) sumail = -Parameter[STATE_CHANNEL_RANGE_I];




	Dvalue = (float)(Parameter[ROLL_D])/512*vel_error;		//when d_gyro2 is positive, the copter tends to rotate right

	nCalailWidth =  -Dvalue*10;



	dvel = (bgyro_3 - old_vel_roll);
	old_vel_roll = bgyro_3;
	//dvel: angular acceleration, if positive, copter is turning right

	nCalailWidth -= dvel*(float)Parameter[VIB_COMPENSATION]*60/150/50*10/2*3;


	//	nCalailWidth += (sumail*10);
	nCalailWidth -= (sumail*10);

	OUT_ATL_FYDUO = (Aileron_MID + nCalailWidth)/10;


	return nCalailWidth;

}

float CalelevWidth(void)
{
	float delta_PlanePitch,Dvalue,Pvalue;
	float Targetacc,acc_cha;
	float target_vel,vel_error;
	float dvel;
	float pcal;
	//pitch angle, positive: up

	//	TargetPitchBias = 0;

	if (!bAutoHand_)
	{

		if (( (Parameter[CONTROL_METHOD]&0x0f)  ==1)||(in_flight==0))
			TargetPitch = (float)(elev - Elevator_MID)/(float)Parameter[RC_TARGET_STATE_COEFF] + TargetPitchBias;
		else
		{

			pcal = ( - xEKF[3]) * 0.2;
			if (pcal > MAXacc) pcal = MAXacc;
			if (pcal < -MAXacc) pcal = -MAXacc;
			Targetacc = pcal;
			Targetacc = 0;


			acc_cha = -(Targetacc - acc_elev);					

			if (fabsf(elev-Elevator_MID)>30) 
			{
				acc_cha = 0;
				oper_elev = 1;
				TargetPitchMid = 0;
			}
			else
			{
				if (oper_elev)
				{
					xEKF[3] = 0;
					acc_cha = acc_elev;
				}
				oper_elev = 0;

			}


			TargetPitchMid = 0;

			TargetPitch = TargetPitchMid + ((float)Parameter[STATE_SENSITIVITY]/4/0.2)* acc_cha + (float)(elev - Elevator_MID)/(float)Parameter[RC_TARGET_STATE_COEFF] + TargetPitchBias;

			if (TargetPitch > 35) TargetPitch = 35;
			if (TargetPitch < -35) TargetPitch = -35;

			if (in_flight)
				TargetPitchMid += ((float)2/hz250/0.1)*acc_cha;

			if (TargetPitchMid > 20) TargetPitchMid = 20;
			if (TargetPitchMid < -20) TargetPitchMid = -20;
		}

	}
	else
	{
		Targetacc = calelevacc;		//Target acceleration (forward)

		acc_cha = -(Targetacc - acc_elev);					

		TargetPitch = TargetPitchMid + ((float)Parameter[STATE_SENSITIVITY]/4/0.2)* acc_cha + TargetPitchBias;

		if ((!in_flight)&&(!start_center_gravity))
			TargetPitch = (float)(elev - Elevator_MID)/(float)Parameter[RC_TARGET_STATE_COEFF] + TargetPitchBias;

		if (TargetPitch > 35) TargetPitch = 35;
		if (TargetPitch < -35) TargetPitch = -35;

		if ((in_flight)||(start_center_gravity))
		{
			TargetPitchMid = sum_angley * cos_hx + sum_anglex * sin_hx;
			TargetPitchMid += ((float)2/hz250/0.1)*acc_cha;				
		}
		else
			TargetPitchMid = 0;

		if (TargetPitchMid > 20) TargetPitchMid = 20;
		if (TargetPitchMid < -20) TargetPitchMid = -20;


	}

	delta_PlanePitch = t_PlanePitch - TargetPitch;



	target_vel = delta_PlanePitch*3;								//Target angular velocity
	if (target_vel > 50) target_vel = 50;
	if (target_vel < -50) target_vel = -50;


	vel_error = (alldata.ina_gyro1 + target_vel);					//Angular velocity error
	if (vel_error > 120) vel_error = 120;
	if (vel_error < -120) vel_error = -120;



	if (Enable_Control_Integrate==1)
	{
		if (fabsf(vel_error) <= 10)
			sumelev += (vel_error/(float)150/2/(hz250/50));
		else if (vel_error > 10)
			sumelev += 10/(float)150/2/(hz250/50);
		else if (vel_error < -10)
			sumelev += -10/(float)150/2/(hz250/50);

	}
	else
	{
		sumelev = 0;
		if (((fabsf(xEKF[3])>0.1)||(fabsf(xEKF[4])>0.1))&&(Enable_Control_Integrate_pre))
			Enable_Control_Integrate = 1;
	}

	if (sumelev > Parameter[STATE_CHANNEL_RANGE_I]) sumelev = Parameter[STATE_CHANNEL_RANGE_I]; 
	if (sumelev < -Parameter[STATE_CHANNEL_RANGE_I]) sumelev = -Parameter[STATE_CHANNEL_RANGE_I];

	Dvalue = (float)(Parameter[PITCH_D])/512*vel_error;		//if d_gyro2 is positive, the copter tends to turn right

	nCalelevWidth =  -Dvalue*10;




	dvel = (-bgyro_2 - old_vel_pitch);
	old_vel_pitch = -bgyro_2;

	nCalelevWidth -= dvel*(float)Parameter[VIB_COMPENSATION]*60/150/50*10/2*3;


	nCalelevWidth -=  (sumelev*10);

	OUT_ATL_SJDUO = (Elevator_MID + nCalelevWidth)/10;

	return nCalelevWidth;
}



float CalthrotWidth(void)
{
	float delta_vel,tmp;
	int ctqict;
	float ddx;
	float maxim;



	if (in_flight==0)
	{

		if ((THROT < 1700)&&(Channel_5 < 1333)&&(!Lock_Throttle))
		{
			clear_pressure = 1;

			in_flight = 1;



			if (satelnum>4)
			{
				homelatti = CurLatti;
				homelongi = CurLongi;
			}

			throtMid = THROT+20;

			tgtHigh = 50;
			AirHigh_h = (int)tgtHigh / 256;
			AirHigh_l = (int)tgtHigh - AirHigh_h * 256;	
			vel_up_caled = 0;
			ch7_status = 2;
			throtqiehuan = 2;
			ctqi = 0;

			Enable_Control_Integrate_pre = 1;

			tgtLatti = CurLatti;
			tgtLongi = CurLongi;

		}
		else
		{
			if ((acc_dn1 < -0.05)&&((THROT < 1800)||(autotakeoff==1))&&(!Lock_Throttle)&&(     !((Channel_5>1666)&&(Channel_6>1666))       ))			
			//or if the upward acceleration is more than g, the average time is ?? ms
			{
				ctqi++;
				ctqict = hz250/5;
				if (ctqi > ctqict)
				{

					start_center_gravity = 0;

					clear_pressure = 1;		

					in_flight = 1;



					if (satelnum>4)
					{
						homelatti = CurLatti;
						homelongi = CurLongi;
					}

					if (Channel_5<1333)
						throtMid = THROT+20;
					else
						throtMid = Xthrot+20;
					tgtHigh = 50;
					AirHigh_h = (int)tgtHigh / 256;
					AirHigh_l = (int)tgtHigh - AirHigh_h * 256;	
					vel_up_caled = 0;
					ch7_status = 2;
					throtqiehuan = 2;
					ctqi = 0;


					new_target_high_count = 3*hz250;

					Enable_Control_Integrate = 1;

					tgtLatti = CurLatti;
					tgtLongi = CurLongi;
				}
			}
			else
			{
				ctqi = 0;
			}
		}
	}




	if (Channel_5 > 1333)							
	{
		if (in_flight==1)
		{				


			if (     ((Channel_5>1666)&&(Channel_6 > 1333)&&(Channel_6 < 1666)&&(Enable_Waypoint))	||	((Channel_5>1666)&&(Channel_6 > 1666)&&(!gohome_end))  ||(pregohome==1)   )		//走航线或返航
				vel_up_caled = 0;
			else
				if (THROT < High_Throttle)				//High throttle, increase height
				{
					new_target_high_count = 0;
					vel_up_caled	= 1;

					vel_up = (float)(High_Throttle+26.25 - (float)THROT)/280 *(float)Parameter[MAX_LIFT_VELOCITY]*2;



					if (vel_up > (float)Parameter[MAX_LIFT_VELOCITY]*2) vel_up = (float)Parameter[MAX_LIFT_VELOCITY]*2;

					ch7_status = 1;

					throtqiehuan = 1;
					enter_set_height = 0;
				}
				else if (THROT < Low_Throttle)		//Middle, stop
				{

					if (throtqiehuan!=2)
					{
						stop_throttle_count = hz250*1;
						throtqiehuan = 2;
					}

					if ((fabsf(xEKF[5]) > 0.2)&&(ch7_status!=2))
					{
						vel_up = 0;
						vel_up_caled = 1;

						stop_throttle_count = hz250*1;
					}
					else
					{
						if (stop_throttle_count > 0)
						{
							vel_up = 0;
							vel_up_caled = 1;
							stop_throttle_count --;
						}
						else
						{
							vel_up_caled = 0;

							if (stop_throttle_count==0)
							{

								if (new_target_high_count>0)
								{
									new_target_high_count--;
								}
								else
									tgtHigh = fAirHigh*10;
								stop_throttle_count = -1;
								enter_set_height = 1;
							}

							if (tgtHigh < 0)
								th = tgtHigh + 65536;
							else
								th = tgtHigh;

							AirHigh_h = (int)th / 256;
							AirHigh_l = (int)th - AirHigh_h * 256;	


						}
						ch7_status = 2;


					}

				}
				else							//Low throttle, decrease height
				{
					new_target_high_count = 0;
					enter_set_height = 0;
					throtqiehuan = 3;

					vel_up_caled = 1;
					vel_up = -(float)((float)THROT-Low_Throttle+26.25)/280 *(float)Parameter[MAX_LIFT_VELOCITY]*2;


					if (vel_up < -(float)Parameter[MAX_LIFT_VELOCITY]*2) vel_up = -(float)Parameter[MAX_LIFT_VELOCITY]*2;

					if (tgtHigh < 0)
						th = tgtHigh + 65536;
					else
						th = tgtHigh;

					AirHigh_h = (int)th / 256;
					AirHigh_l = (int)th - AirHigh_h * 256;	
					ch7_status = 3;
				}


		} 
		else		//in_flight==0，hasn't taken off.
		{
			if ((autotakeoff==1)&&(Channel_5>1666)&&(Channel_6<1666))
			{
				THROT = 1500;
			}

			if ((THROT > 1800)||(Channel_6>1666)) {nCalthrotWidth = 1930;Xthrot = 1930;}
			else
				if (THROT > 1500) 
				{
					start_center_gravity = 0;
					nCalthrotWidth = 1780 + 0.43 * (THROT-1500);
					Xthrot = nCalthrotWidth;
				}
				else if ((THROT <= 1500)&&(Channel_6<1666))
				{
					start_center_gravity = 1;
					Xthrot-=(2.5/(hz250/80));
					if (Xthrot < 1250) Xthrot = 1250;
					nCalthrotWidth = Xthrot;
				}

				if (close==1)
				{
					if (!((Parameter[POWER_NO] & 0x10) >> 4))
					{
						Lock_Throttle = 1;
						ctlock_throttle = 5*hz250;
						close = 0;
					}
					else
						endland = 1;

				}



				goto endthrot;
		}
	}



	if (Channel_5>1333)		//If switched to "Set height"
	{

		if ((gohome_end)&&(Channel_5>1666))			
		{
			if (fAirHigh > 30)
				vel_up = -(float)Parameter[AUTO_LANDING_VELOCITY]*4/3*4;	//cm/s
			else if (fAirHigh > 25)
				vel_up = -(float)Parameter[AUTO_LANDING_VELOCITY]*4/3*3;	//cm/s
			else
				vel_up = -(float)Parameter[AUTO_LANDING_VELOCITY]*4/3;		//cm/s
		}			
		else
		{
			if (vel_up_caled==0)
			{

				if (tgtHigh/10 > fAirHigh)
				{
					vel_up = (float)(tgtHigh/10 - fAirHigh) *0.5  *(float)200 / (float)3;		//unit cm/s, if positive, upward

					if (vel_up > (float)Parameter[MAX_LIFT_VELOCITY]*2) vel_up = (float)Parameter[MAX_LIFT_VELOCITY]*2;
					if ((enter_set_height)&&((!pregohome)||(gohome)))
					{
						if (tgtHigh/10 - fAirHigh < 3)	//less than 5m
						{
							if (vel_up > 20) vel_up = 20;
						}
						else
							if (tgtHigh/10 - fAirHigh < 4)	//less than 5m
							{
								if (vel_up > 30) vel_up = 30;
							}
							else
								if (tgtHigh/10 - fAirHigh < 5)	//less than 5m
								{
									if (vel_up > 40) vel_up = 40;
								}
					}
				}
				else
				{
					vel_up = (float)(tgtHigh/10 - fAirHigh) *0.5  *(float)200 / (float)3;		//unit: cm/s, if positive, upward
					if (vel_up < -(float)Parameter[MAX_LIFT_VELOCITY]*2) vel_up = -(float)Parameter[MAX_LIFT_VELOCITY]*2;

					if ((enter_set_height)&&((!pregohome)||(gohome)))
					{

						if (fAirHigh - tgtHigh/10 < 3)	//less than 5m
						{
							if (vel_up < -20) vel_up = -20;
						}
						else
							if (fAirHigh - tgtHigh/10 < 4)	//less than 5m
							{
								if (vel_up < -30) vel_up = -30;
							}
							else
								if (fAirHigh - tgtHigh/10 < 5)	//less than 5m
								{
									if (vel_up < -40) vel_up = -40;
								}
					}
				}

			}
		}






		delta_vel = vel_up + (float)xEKF[5]*100;		//If positive, add throttle


		targetaccU = delta_vel/250;						//unit: g			//if velocity error is 100cm/s, target acceleration = 0.4g; within 1/4s the velocity can be fixed
		if (targetaccU > 0.2) targetaccU = 0.2;
		if (targetaccU < -0.2) targetaccU = -0.2;
		acc_U = -acc_dn2;								//unit: g			upward acceleration


		//P
		tmp = (targetaccU - acc_U)*40/0.1 *(float)Parameter[THROTTLE_P]/140;		//When error: 0.1g, output 4 gears of throttle
		if (tmp > 200) tmp = 200;
		if (tmp < -200) tmp = -200;

		dqthrot = throtMid - tmp;

		maxim = 0;

		if (maxim < bcal1) maxim = bcal1;
		if (maxim < bcal2) maxim = bcal2;
		if (maxim < bcal3) maxim = bcal3;
		if (maxim < bcal4) maxim = bcal4;
		if (maxim < bcal5) maxim = bcal5;
		if (maxim < bcal6) maxim = bcal6;
		if (maxim < bcal7) maxim = bcal7;
		if (maxim < bcal8) maxim = bcal8;


		if (maxim > 250)			//Less than 65
			maxim = 250;

		if (dqthrot < 1100 + maxim)  dqthrot = 1100 + maxim;

		//I
		delta_vel = (targetaccU - acc_U)*g0*100;		//Unit: cm/s^2
		tmp = delta_vel/2*(float)Parameter[NEW_THROTT_I]/50;


		if (tmp > 98*(float)Parameter[NEW_THROTT_I]/50) tmp = 98*(float)Parameter[NEW_THROTT_I]/50;
		if (tmp < -98*(float)Parameter[NEW_THROTT_I]/50) tmp = -98*(float)Parameter[NEW_THROTT_I]/50; 

		if (throtMid > 1100 + maxim)		//Mid throttle relatively low
		{

			throtMid -= tmp/hz250;
		}
		else							//中油门过大,如果要减小可以,要加大则不行
		{
			if (tmp < 0)
				throtMid -= tmp/hz250;
		}

		if ((throtMid > 1900)&&(in_flight))
		{
			if (!((Parameter[POWER_NO] & 0x10) >> 4))
			{
				Lock_Throttle=1;
				ctlock_throttle=5*hz250;
			}
			else
				endland = 1;
		}

		if (throtMid > 1930) throtMid = 1930;



		nCalthrotWidth = dqthrot;



		if (close==1)
		{
			if (!((Parameter[POWER_NO] & 0x10) >> 4))
			{
				Lock_Throttle = 1;
				ctlock_throttle = 5*hz250;
				close = 0;
			}
			else
				endland = 1;

		}

	}
	else					//Completely manual, increase throttle support
	{
		if (THROT > 1900)
			nCalthrotWidth = THROT;
		else
		{
			delta_vel = 0 + (float)xEKF[5]*100;		//if positive, increase throttle


			tmp = (delta_vel*20/40)*(float)Parameter[PERPENDICULAR_STAB_COEF]*140/150/100;
			if (tmp > 100) tmp = 100;
			if (tmp < -100) tmp = -100;

			nCalthrotWidth = THROT - tmp;

			maxim = 0;

			if (maxim < bcal1) maxim = bcal1;
			if (maxim < bcal2) maxim = bcal2;
			if (maxim < bcal3) maxim = bcal3;
			if (maxim < bcal4) maxim = bcal4;
			if (maxim < bcal5) maxim = bcal5;
			if (maxim < bcal6) maxim = bcal6;
			if (maxim < bcal7) maxim = bcal7;
			if (maxim < bcal8) maxim = bcal8;


			if (maxim > 250)			//<65
				maxim = 250;

			if (nCalthrotWidth < 1100 + maxim)  nCalthrotWidth = 1100 + maxim;

		}
	}

endthrot:;
	OUT_ATL_YMDUO = nCalthrotWidth/10;
}


float CalAtanVector(void) // Calculate arc-tan of vectorLongi/vectorLatti. Range: -PI~PI
{
	float beta;
	float lTemp;
	if(vectorLatti > 0)
		bNorth = 1;
	else
		bNorth = 0;

	if(vectorLongi > 0)
		bEast = 1;
	else
		bEast = 0;

	vectorLatti = fabsf(vectorLatti);
	vectorLongi = fabsf(vectorLongi);
	vectorLongi = vectorLongi * COS_FACTOR;

	if(vectorLongi > vectorLatti)
	{
		lTemp = vectorLongi;
		vectorLongi = vectorLatti;
		vectorLatti = lTemp;
		bGT45 = 1;
	}
	else
	{
		bGT45 = 0;
	}
	if (vectorLatti!=0)
		beta= atan(vectorLongi/vectorLatti);   	//0~PI/4
	else
		beta = PI/2;
	if(bGT45)
		beta = PI/2 - beta;					//PI/4~PI/2
	if(bNorth && bEast)
		beta = beta;						//0~PI/4
	if(bNorth && !bEast)
		beta = 0 - beta;					//-PI/4~0
	if(!bNorth && !bEast)
		beta = beta - PI;					//-PI~-3PI/4
	if(!bNorth && bEast)
		beta = PI - beta;					//3PI/4~PI
	return beta;
}

char PointArrive(void)
{
	float ddx,ddy;

	ddx = (tgtLongi - CurLongi)*111199*COS_FACTOR;
	ddy = (tgtLatti - CurLatti)*111199;


	if (ClickSetTgtFlag==1)
	{
		if (ddx*ddx + ddy*ddy > Parameter[POINT_RADIUS]*Parameter[POINT_RADIUS]*2*2)
			return 0;
		else
			return 1;
	}
	else
	{
		if (ddx*ddx + ddy*ddy > Parameter[POINT_RADIUS]*Parameter[POINT_RADIUS])
			return 0;
	}
	return 1;
}


void buf2float2(float *tfloat, char *buf)
{
	int i;
	char * p1 = (char *)tfloat;
	char * p3 = buf+3;
	for(i=0;i<4;i++)
	{
		*p1 = *p3;
		p1++;
		p3--;
	}

}

void buf2float(float *tfloat, char *buf)
{
	int i;
	char * p1 = (char *)tfloat;
	char * p3 = buf;
	for(i=0;i<4;i++)
	{
		*p1 = *p3;
		p1++;
		p3++;
	}

}

void buf2long(long *tfloat, char *buf)
{
	int i;
	char * p1 = (char *)tfloat;
	char * p3 = buf;
	for(i=0;i<4;i++)
	{
		*p1 = *p3;
		p1++;
		p3++;
	}
}

void buf2int(int *tint, char *buf)
{
	int i;
	char * p1 = (char *)tint;
	char * p3 = buf;
	for(i=0;i<4;i++)
	{
		*p1 = *p3;
		p1++;
		p3++;
	}
}

void float2buf(char *buf,float *tfloat)
{
	int i;
	char * p1 = (char *)tfloat;
	char * p3 = buf;
	for(i=0;i<4;i++)
	{
		*p3 = *p1;
		p1++; 
		p3++;
	}
}

void int2buf(char *buf,int *tint)
{
	int i;
	char * p1 = (char *)tint;
	char * p3 = buf;
	for(i=0;i<4;i++)
	{
		*p3 = *p1;
		p1++;
		p3++;
	}
} 

float distance_point(float xa, float ya, float xb, float yb)
{
	float distance;
	return distance = sqrt((xa - xb)*111199*(xa - xb)*111199 + (ya - yb)*111199*COS_FACTOR*(ya - yb)*111199*COS_FACTOR);
}

float newdistance(void)  //Calculate the distance between the current point and the current waypoint target
{
	float dis,sin_t,cos_t;
	dis = sqrt( ((double)tgtLatti - (double)tgtLatti_s) * ((double)tgtLatti - (double)tgtLatti_s) + ((double)tgtLongi - (double)tgtLongi_s) * ((double)tgtLongi - (double)tgtLongi_s) * (double)COS_FACTOR * (double)COS_FACTOR );
	if (dis!=0)
	{
		sin_t = (tgtLatti - tgtLatti_s) / dis;
		cos_t = (tgtLongi - tgtLongi_s) * COS_FACTOR / dis;
	}
	else 
	{
		sin_t = 0;
		cos_t = 0;
	}


	dis = (CurLongi - tgtLongi) * sin_t * COS_FACTOR + (tgtLatti - CurLatti) * cos_t;

	dis = fabsf(dis * 110000);
	return dis;
}






void rd_adi1256(void)
{
	int i;

	data_1256 = data3a;
	if (data_1256 & 0xc00000)						//negative
	{
		data_1256 = data_1256 & 0x3fffff;

		if (data_1256 < 335544)					//incorrect sign, should be positive
		{
			duan = data_1256;
			vout1 = VREF + (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = 0x400000 - data_1256;
			vout1 = VREF - (float)duan/8388607*2*VREF;
		}
	}
	else											//positive
	{
		if (data_1256 > 3858759)					//incorrect sign, should be negative
		{
			duan = 0x400000 - data_1256;
			vout1 = VREF - (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = data_1256 & 0x3fffff;
			vout1 = VREF + (float)duan/8388607*2*VREF;
		}
	}

	vout4 = vout1;
	if (vout4Mid[temperature]!=0)
		vout1 = vout4 - (vout4Mid[temperature] - vout4Mid[25]);

	facc2 = -(vout1 - 1.5)/0.3;

	data_1256 = data1a;
	if (data_1256 & 0xc00000)						//negative
	{
		data_1256 = data_1256 & 0x3fffff;

		if (data_1256 < 335544)					//incorrect sign, should be positive
		{
			duan = data_1256;
			vout2 = VREF + (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = 0x400000 - data_1256;
			vout2 = VREF - (float)duan/8388607*2*VREF;
		}
	}
	else											//positive
	{
		if (data_1256 > 3858759)					//incorrect sign, should be negative
		{
			duan = 0x400000 - data_1256;
			vout2 = VREF - (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = data_1256 & 0x3fffff;
				vout2 = VREF + (float)duan/8388607*2*VREF;
		}
	}

	vout5 = vout2;
	if (vout5Mid[temperature]!=0)
		vout2 = vout5 - (vout5Mid[temperature] - vout5Mid[25]);
	facc1 = -(vout2 - 1.5)/0.3;


	data_1256 = data2a;
	if (data_1256 & 0xc00000)						//negative
	{
		data_1256 = data_1256 & 0x3fffff;

		if (data_1256 < 335544)					//incorrect sign, should be positive
		{
			duan = data_1256;
			vout3 = VREF + (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = 0x400000 - data_1256;
			vout3 = VREF - (float)duan/8388607*2*VREF;
		}
	}
	else											//positive
	{
		if (data_1256 > 3858759)					//incorrect sign, should be negative
		{
			duan = 0x400000 - data_1256;
			vout3 = VREF - (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = data_1256 & 0x3fffff;
			vout3 = VREF + (float)duan/8388607*2*VREF;
		}
	}

	vout6 = vout3;
	if (vout6Mid[temperature]!=0)
		vout3 = vout6 - (vout6Mid[temperature] - vout6Mid[25]);

	facc3 = (vout3 - 1.5)/0.3;


	data_1256 = data4a;
	if (data_1256 & 0xc00000)						//negative
	{
		data_1256 = data_1256 & 0x3fffff;

		if (data_1256 < 335544)					//incorrect sign, should be positive
		{
			duan = data_1256;
			vout1 = VREF + (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = 0x400000 - data_1256;
			vout1 = VREF - (float)duan/8388607*2*VREF;
		}
	}
	else											//positive
	{
		if (data_1256 > 3858759)					//incorrect sign, should be negative
		{
			duan = 0x400000 - data_1256;
			vout1 = VREF - (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = data_1256 & 0x3fffff;
			vout1 = VREF + (float)duan/8388607*2*VREF;
		}
	}


	data_1256 = data8a;
	if (data_1256 & 0xc00000)						//negative
	{
		data_1256 = data_1256 & 0x3fffff;

		if (data_1256 < 335544)					//incorrect sign, should be positive
		{
			duan = data_1256;
			vouttemperature = VREF + (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = 0x400000 - data_1256;
			vouttemperature = VREF - (float)duan/8388607*2*VREF;
		}
	}
	else											//positive
	{
		if (data_1256 > 3858759)					//incorrect sign, should be negative
		{
			duan = 0x400000 - data_1256;
			vouttemperature = VREF - (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = data_1256 & 0x3fffff;
			vouttemperature = VREF + (float)duan/8388607*2*VREF;
		}
	}

	temperature = (vouttemperature - 2.98) / 0.010 * 1.167 + 25;

	SndBuf[TEMPERATURE] = temperature;



	wtemperature = temperature;

	if (temperature < temperature_low) temperature = temperature_low;
	if (temperature > temperature_high) temperature = temperature_high;


	data_1256 = data7a;
	if (data_1256 & 0xc00000)						//negative
	{
		data_1256 = data_1256 & 0x3fffff;

		if (data_1256 < 335544)					//incorrect sign, should be positive
		{
			duan = data_1256;
			voutpres = VREF + (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = 0x400000 - data_1256;
			voutpres = VREF - (float)duan/8388607*2*VREF;
		}
	}
	else											//positive
	{
		if (data_1256 > 3858759)					//incorrect sign, should be negative
		{
			duan = 0x400000 - data_1256;
			voutpres = VREF - (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = data_1256 & 0x3fffff;
			voutpres = VREF + (float)duan/8388607*2*VREF;
		}
	}

	presnew = voutpres / 45 * 1000000 + (101212 - 90456);		//unit: Pascal
	//standard pressure 101212
	tpresnew = presnew;

	if (first6115==0)
	{
		first6115 = 1;
		Pres0 = presnew;
	}

	spipn = presnew;

	if (ctair > 4999) ctair = 4999;
	press_data[ctair] = spipn;
	ctair++;

	if (ctair2 > 4999) ctair2 = 4999;
	press_data2[ctair2] = spipn;
	ctair2++;

	if (tAirHighf >= 0)
	{
		SndBuf[CUR_PRESSURE_2] = (unsigned char)(tAirHighf*10/256);
		OUT_HIGH_AIR = (unsigned char)(tAirHighf*10 - (unsigned char)(SndBuf[CUR_PRESSURE_2])*256);
	}
	else
	{
		SndBuf[CUR_PRESSURE_2] = (unsigned char)((tAirHighf*10+65536)/256);
		OUT_HIGH_AIR = (unsigned char)((tAirHighf*10+65536) - (unsigned char)(SndBuf[CUR_PRESSURE_2])*256);
	}



	fgyro3 = (vout1 - vout1Mid[temperature]) * 1000 / 3.33;
	zero_gyro3 = fgyro3;

	if (clear_bias==1)
	{
		if (clear_bias_time<=CLR_TIME)
		{
			bias_iraw_gyro_3_zu	+=  fgyro3;
			fgyro3 = 0;
		}
	}
	else
		fgyro3 = fgyro3 - bias_iraw_gyro_3;


	fgyro3 /= kgyro1;



	data_1256 = data5a;
	if (data_1256 & 0xc00000)						//negative
	{
		data_1256 = data_1256 & 0x3fffff;

		if (data_1256 < 335544)					//incorrect sign, should be positive
		{
			duan = data_1256;
			vout2 = VREF + (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = 0x400000 - data_1256;
			vout2 = VREF - (float)duan/8388607*2*VREF;
		}
	}
	else											//positive
	{
		if (data_1256 > 3858759)					//incorrect sign, should be negative
		{
			duan = 0x400000 - data_1256;
			vout2 = VREF - (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = data_1256 & 0x3fffff;
			vout2 = VREF + (float)duan/8388607*2*VREF;
		}
	}


	fgyro2 = (vout2 - vout2Mid[temperature]) * 1000 / 3.33;
	zero_gyro2 = fgyro2;


	if (clear_bias==1)
	{
		if (clear_bias_time<=CLR_TIME)
		{
			bias_iraw_gyro_2_zu	+=  fgyro2;
			fgyro2 = 0;
		}
	}
	else
		fgyro2 = fgyro2 - bias_iraw_gyro_2;

	fgyro2 /= kgyro2;							//pitch



	data_1256 = data6a;
	if (data_1256 & 0xc00000)						//negative
	{
		data_1256 = data_1256 & 0x3fffff;

		if (data_1256 < 335544)					//incorrect sign, should be positive
		{
			duan = data_1256;
			vout3 = VREF + (float)duan/8388607*2*VREF;
		}
		else
		{
			duan = 0x400000 - data_1256;
			vout3 = VREF - (float)duan/8388607*2*VREF;
		}
	}
	else											//positive
	{
		if (data_1256 > 3858759)					//incorrect sign, should be negative
		{
			duan = 0x400000 - data_1256;
			vout3 = VREF - (float)duan/8388607*2*VREF;
		}
		else 
		{
			duan = data_1256 & 0x3fffff;
			vout3 = VREF + (float)duan/8388607*2*VREF;
		}
	}



	fgyro1 = -(vout3 - vout3Mid[temperature]) * 1000 / 3.33;
	zero_gyro1 = fgyro1;

	if (clear_bias==1)
	{
		if (clear_bias_time<=CLR_TIME)
		{
			bias_iraw_gyro_1_zu	+=  fgyro1;
			fgyro1 = 0;
		}
	}
	else
		fgyro1 = fgyro1 - bias_iraw_gyro_1;

	fgyro1 /= kgyro3;							






	if ((clear_bias==1)&&(clear_bias_time>CLR_TIME))
	{
		bias_iraw_gyro_1 = bias_iraw_gyro_1_zu / CLR_TIME;
		bias_iraw_gyro_2 = bias_iraw_gyro_2_zu / CLR_TIME;
		bias_iraw_gyro_3 = bias_iraw_gyro_3_zu / CLR_TIME;
		clear_bias = 0;
		clear_bias_time = 0;
		//		do_writebuf();
		writebuf1 = 1;
		Manual_Mode = 1;

		alldata.gps_latti = 40;
		alldata.gps_longi = 112;

		//自动切回手动时，alt0不变，Pres0不变

		alldata.gps_velN = 0;
		alldata.gps_velE = 0;

		rpy[0] = 0;
		rpy[1] = 0;
		rpy[2] = 0;

		vel_down_clear = 1;
		InitData();					

	}

}



void put_data1256(void)
{
	float a,b,c,d,n,p0,q0,r0;
	int i;
	facc1_1 = kacc1 * (facc1 - bias_acc1);
	facc2_1 = kacc2 * (facc2 - bias_acc2);
	facc3_1 = kacc3 * (facc3 - bias_acc3);

	tmpacc_1 = facc1_1/0.002522;
	tmpacc_2 = facc2_1/0.002522;
	tmpacc_3 = facc3_1/0.002522;


	ttmpacc_1 = tmpacc_1;
	ttmpacc_2 = tmpacc_2;
	ttmpacc_3 = tmpacc_3;



	if (tmpacc_1 < 0) tmpacc_1 += 65536;
	if (tmpacc_2 < 0) tmpacc_2 += 65536;
	if (tmpacc_3 < 0) tmpacc_3 += 65536;

	tmpBuf[0] = tmpacc_1 / 256;
	tmpBuf[1] = tmpacc_1 - tmpBuf[0]*256;

	tmpBuf[2] = tmpacc_2 / 256;
	tmpBuf[3] = tmpacc_2 - tmpBuf[2]*256;

	tmpBuf[4] = tmpacc_3 / 256;
	tmpBuf[5] = tmpacc_3 - tmpBuf[4]*256;


	SndBuf[ACCELERATION_BACKWARD_2] = tmpBuf[0];
	SndBuf[ACCELERATION_BACKWARD_1] = tmpBuf[1];
	SndBuf[ACCELERATION_RIGHT_2] = tmpBuf[2];
	SndBuf[ACCELERATION_RIGHT_1] = tmpBuf[3];
	SndBuf[ACCELERATION_DOWN_2] = tmpBuf[4];
	SndBuf[ACCELERATION_DOWN_1] = tmpBuf[5];

	//Calculate vibration and rocking coeffecients
	ct_estimate++;
	if (ct_estimate >= hz250/10)
	{
		magz_max = ttmpacc_3;
		magz_min = ttmpacc_3;

		magx_max = ttmpacc_2;
		magx_min = ttmpacc_2;

		magy_max = ttmpacc_1;
		magy_min = ttmpacc_1;

		gyro1_max = gyro_2;
		gyro1_min = gyro_2;

		gyro2_max = gyro_3;
		gyro2_min = gyro_3;

		ct_estimate = 0;
	}
	else
	{


		if (gyro_2 > gyro1_max) gyro1_max = gyro_2;
		if (gyro_2 < gyro1_min) gyro1_min = gyro_2;

		if (gyro_3 > gyro2_max) gyro2_max = gyro_3;
		if (gyro_3 < gyro2_min) gyro2_min = gyro_3;

		gyro1_estimate = gyro1_max - gyro1_min;
		gyro2_estimate = gyro2_max - gyro2_min;
		//以正负25为10
		if (gyro1_estimate > gyro2_estimate)
			gyro_estimate = gyro1_estimate;
		else
			gyro_estimate = gyro2_estimate;

		gyro_estimate = gyro_estimate / 5;

		SndBuf[ROCKING_COEFF] = gyro_estimate;


		if (ttmpacc_3 > magz_max) magz_max = ttmpacc_3;
		if (ttmpacc_3 < magz_min) magz_min = ttmpacc_3;
		magz_estimate = (magz_max - magz_min)*0.02522;

		if (ttmpacc_2 > magx_max) magx_max = ttmpacc_2;
		if (ttmpacc_2 < magx_min) magx_min = ttmpacc_2;
		magx_estimate = (magx_max - magx_min)*0.02522;

		if (ttmpacc_1 > magy_max) magy_max = ttmpacc_1;
		if (ttmpacc_1 < magy_min) magy_min = ttmpacc_1;
		magy_estimate = (magy_max - magy_min)*0.02522;

		if (magx_estimate > magy_estimate)
		{
			if (magx_estimate > magz_estimate)
			{
				magfx = 0;		//east
				magmax_estimate = magx_estimate;
			}
			else
			{
				magfx = 2;		//ground
				magmax_estimate = magz_estimate;
			}
		}
		else
		{
			if (magy_estimate > magz_estimate)
			{
				magfx = 1;		//north
				magmax_estimate = magy_estimate;
			}
			else
			{
				magfx = 2;
				magmax_estimate = magz_estimate;
			}
		}


		SndBuf[VIBRATION_COEFF] = magmax_estimate;

		if (magfx==0)
		{
			SndBuf[VIBRATION_COEFF] += 100;
		}
		else if (magfx==2)
		{
			SndBuf[VIBRATION_COEFF] += 200;
		}
	}





	facc1 -= bias_acc1;
	facc2 -= bias_acc2;
	facc3 -= bias_acc3;

	facc1 *= kacc1;
	facc2 *= kacc2;
	facc3 *= kacc3;

	nDvalue = Butterworth(facc1, zcr);
	acc_1 = nDvalue*g0;

	nDvalue = Butterworth(facc2, zcq);
	acc_2 = nDvalue*g0;

	nDvalue = Butterworth(facc3, zcp);
	acc_3 = nDvalue*g0;



	gyro_1 = fgyro1;
	gyro_2 = fgyro2;
	gyro_3 = fgyro3;		





	nDvalue = Butterworth(fgyro2, bzcq);
	bgyro_2 = nDvalue;

	nDvalue = Butterworth(fgyro3, bzcp);
	bgyro_3 = nDvalue;		


	alldata.ina_acc1 = -acc_2;
	alldata.ina_acc2 = acc_1;
	alldata.ina_acc3 = acc_3;

	alldata.ina_gyro1 = -gyro_2;
	alldata.ina_gyro2 = gyro_3;
	alldata.ina_gyro3 = -gyro_1;



	if (Parameter[ERASE_ROMBOOT]==88)
	{
		if (in88==0)
		{
			in88=1;

			a=xEKF[6];
			b=xEKF[7];
			c=xEKF[8];
			d=xEKF[9];

			p0 = 0;
			q0 = PI/2;
			r0 = 0;

			xEKF[6] = a + 0.5*(-b*p0-c*q0-d*r0);
			xEKF[7] = b + 0.5*(a*p0-d*q0+c*r0);
			xEKF[8] = c + 0.5*(d*p0+a*q0-b*r0);
			xEKF[9] = d + 0.5*(-c*p0+b*q0+a*r0);
			a=xEKF[6];
			b=xEKF[7];
			c=xEKF[8];
			d=xEKF[9];
			n= sqrt(a*a+b*b+c*c+d*d);
			xEKF[6] = a/n;
			xEKF[7] = b/n;
			xEKF[8] = c/n;
			xEKF[9] = d/n;

			abcd2rpy();
			abcd2cbn();

			alldata.true_pitch = rpy[1] * deg;
			alldata.true_roll = rpy[0] * deg;

		}

		alldata.ina_acc1 = -acc_2;
		alldata.ina_acc2 = acc_3;
		alldata.ina_acc3 = -acc_1;

		alldata.ina_gyro1 = -gyro_2;
		alldata.ina_gyro2 = -gyro_1;
		alldata.ina_gyro3 = -gyro_3;
	}
	else
	{
		if (in88==1)
		{
			in88 = 0;
			a=xEKF[6];
			b=xEKF[7];
			c=xEKF[8];
			d=xEKF[9];

			p0 = 0;
			q0 = -PI/2;
			r0 = 0;

			xEKF[6] = a + 0.5*(-b*p0-c*q0-d*r0);
			xEKF[7] = b + 0.5*(a*p0-d*q0+c*r0);
			xEKF[8] = c + 0.5*(d*p0+a*q0-b*r0);
			xEKF[9] = d + 0.5*(-c*p0+b*q0+a*r0);
			a=xEKF[6];
			b=xEKF[7];
			c=xEKF[8];
			d=xEKF[9];
			n= sqrt(a*a+b*b+c*c+d*d);
			xEKF[6] = a/n;
			xEKF[7] = b/n;
			xEKF[8] = c/n;
			xEKF[9] = d/n;

			abcd2rpy();
			abcd2cbn();

			alldata.true_pitch = rpy[1] * deg;
			alldata.true_roll = rpy[0] * deg;

		}
	}


}






void rdTWIad(void)		//mode 2
{ 
	int loop;
	unsigned char which;


	iic_wrbuf[0]=0x80;
	AT91F_TWI_Write(AT91C_BASE_TWI, 0x0, (char *)&iic_wrbuf[0], 1);
	wait(100000);	
	AT91F_TWI_Read(AT91C_BASE_TWI, 0x0, (char *)&iic_rdbuf[0], 2);

	if (twi_error) return;

	iic_wrbuf[0]=0xb0;
	AT91F_TWI_Write(AT91C_BASE_TWI, 0x0, (char *)&iic_wrbuf[0], 1);	
	wait(10000);		
	AT91F_TWI_Read(AT91C_BASE_TWI, 0x0, (char *)&iic_rdbuf[2], 2);

	if (twi_error) return;

	/*
	iic_wrbuf[0]=0xc0;
	AT91F_TWI_Write(AT91C_BASE_TWI, 0x0, (char *)&iic_wrbuf[0], 1);	
	wait(10000);		
	AT91F_TWI_Read(AT91C_BASE_TWI, 0x0, (char *)&iic_rdbuf[4], 2);
	*/

	for (loop=0;loop<2;loop++)
	{
		if (!(iic_rdbuf[loop*2] & 0x80))
		{
			which = iic_rdbuf[loop*2] & 0x70;
			which = which >> 4;

			switch (which)
			{
			case 0: {if (loop==0) {v12U = ((unsigned char)iic_rdbuf[loop*2+1]+((unsigned char)iic_rdbuf[loop*2]&0x0f)*256);}break;}
			case 3: {if (loop==1) {vFORCEU = ((unsigned char)iic_rdbuf[loop*2+1]+((unsigned char)iic_rdbuf[loop*2]&0x0f)*256);}break;}
					//				case 4: {if (loop==2) {vCURRENTU =((unsigned char)iic_rdbuf[loop*2+1]+((unsigned char)iic_rdbuf[loop*2]&0x0f)*256);}break;}
			}
		}
	}


	if (fabsf(v12U - old_v12U) > 50)		//error
	{
		ccterror ++;
		if (ccterror >= 3)
		{
			old_v12U = v12U;
		}
		else
		{
			v12U = old_v12U;
		}
	}
	else
	{
		old_v12U = v12U;
		ccterror = 0;
	}


	if (fabsf(vFORCEU - old_vFORCEU) > 50)		//error
	{
		ccterror_vforce ++;
		if (ccterror_vforce >= 3)
		{
			old_vFORCEU = vFORCEU;
		}
		else
		{
			vFORCEU = old_vFORCEU;
		}
	}
	else
	{
		old_vFORCEU = vFORCEU;
		ccterror_vforce = 0;
	}

	v12U *= xiumaggFORCE;

	vFORCEU *= xiumaggFORCE;



	vFORCEU = v12U;
	vforce = (float)vFORCEU * 4 * 5 / 4096;		//distributed voltage resistor 3:1

	if (fly_timeup==1)
		vforce = emergency_power - 0.1;



}

void doCapture(void)		//get neutral positions
{
	if (enter_set!=1)
	{
		TargetRollBias += -(float)(ail - Aileron_MID)/(float)Parameter[RC_TARGET_STATE_COEFF];
		TargetPitchBias += (float)(elev - Elevator_MID)/(float)Parameter[RC_TARGET_STATE_COEFF];
		fxbias = 0;
	}
	else
	{
		TargetRollBias = 0;
		TargetPitchBias = 0;
		fxbias = 0;
	}


	Rudder_MID = FX;
	Aileron_MID = ail;
	Elevator_MID = elev;
	OUT_ARM_FXZHONG = Rudder_MID/10;
	OUT_ARM_FYZHONG = Aileron_MID/10;
	OUT_ARM_SJZHONG = Elevator_MID/10;
}


void do_writebuf1(void)
{
	int i,j;
	char temp[10];

	////////////Save already set values in flash
	msg3[0]=Rudder_MID/10;
	msg3[1]=Aileron_MID/10;
	msg3[2]=Elevator_MID/10;
	msg3[3]=throtMAX/10;
	msg3[4]=throtMIN/10;
	for(i=1;i<=9;i++)
		msg3[i+5]=conReverseFlag[i];	//reverse parameters
	for(i=1;i<=40;i++)
		msg3[i+14]=Parameter[i-1];

	float2buf(&temp[0],&RollCorrect);
	msg3[65] = temp[0];
	msg3[66] = temp[1];
	msg3[67] = temp[2];
	msg3[68] = temp[3];

	float2buf(&temp[0],&PitchCorrect);
	msg3[69] = temp[0];
	msg3[70] = temp[1];
	msg3[71] = temp[2];
	msg3[72] = temp[3];


	msg3[333] = Channel8_position/256;
	msg3[332] = Channel8_position - msg3[333]*256;


	float2buf(&msg3[351],&bias_iraw_gyro_1);
	float2buf(&msg3[355],&bias_iraw_gyro_2);
	float2buf(&msg3[359],&bias_iraw_gyro_3);


	float2buf(&msg3[363],&xmag_bias);
	float2buf(&msg3[367],&throtag_bias);
	float2buf(&msg3[371],&zmag_bias);
	float2buf(&msg3[375],&Kmag);
	float2buf(&msg3[379],&Kmag2);
	float2buf(&msg3[383],&TargetRollBias);
	float2buf(&msg3[387],&TargetPitchBias);
	float2buf(&msg3[391],&fxbias);


	float2buf(&msg3[395],&bias_acc1);
	float2buf(&msg3[399],&bias_acc2);
	float2buf(&msg3[403],&bias_acc3);

	float2buf(&msg3[407],&kacc1);
	float2buf(&msg3[411],&kacc2);
	float2buf(&msg3[415],&kacc3);


	msg3[420] = rudMAX/256;
	msg3[421] = rudMAX - (unsigned char)msg3[420]*256;

	msg3[422] = rudMIN/256;
	msg3[423] = rudMIN - (unsigned char)msg3[422]*256;

	msg3[424] = ailMAX/256;
	msg3[425] = ailMAX - (unsigned char)msg3[424]*256;

	msg3[426] = ailMIN/256;
	msg3[427] = ailMIN - (unsigned char)msg3[426]*256;

	msg3[428] = elevMAX/256;
	msg3[429] = elevMAX - (unsigned char)msg3[428]*256;

	msg3[430] = elevMIN/256;
	msg3[431] = elevMIN - (unsigned char)msg3[430]*256;


	//msg3[432]~msg3[463]	32 messages in total
	for(i=0;i<32;i++)
		msg3[432+i] = paraUSER[i];



	float2buf(&msg3[464],&kgyro1);
	float2buf(&msg3[468],&kgyro2);
	float2buf(&msg3[472],&kgyro3);



	write_dataflash (0xc003a000, (long)(&msg3[0]), 492);

	writebuf1 = 0;

}

void do_writebuf2(void)
{
	int i;

	msg3[500] = Waypoint_number;

	for(i=0;i<Waypoint_number;i++)
	{ 

		msg3[501+25*i] = i+1; //curnum

		float2buf(&Longi_Array[0],&tgt_Latti[i]);
		msg3[502+25*i] = Longi_Array[0];
		msg3[503+25*i] = Longi_Array[1];
		msg3[504+25*i] = Longi_Array[2];
		msg3[505+25*i] = Longi_Array[3];

		float2buf(&Longi_Array[0],&tgt_Longi[i]);
		msg3[506+25*i] = Longi_Array[0];
		msg3[507+25*i] = Longi_Array[1];
		msg3[508+25*i] = Longi_Array[2];
		msg3[509+25*i] = Longi_Array[3];

		int2buf(&Longi_Array[0],&tgt_High[i]);
		msg3[510+25*i] = Longi_Array[0];
		msg3[511+25*i] = Longi_Array[1];
		msg3[512+25*i] = Longi_Array[2];
		msg3[513+25*i] = Longi_Array[3];

		int2buf(&Longi_Array[0],&tgt_Vel[i]);
		msg3[514+25*i] = Longi_Array[0];
		msg3[515+25*i] = Longi_Array[1];
		msg3[516+25*i] = Longi_Array[2];
		msg3[517+25*i] = Longi_Array[3];

		msg3[519+25*i] = tgt_zwl[i]/256;
		msg3[518+25*i] = tgt_zwl[i] - (unsigned char)(msg3[519+25*i])*256;

		msg3[520+25*i] = tgt_hover_num[i];

		msg3[522+25*i] = tgt_hover_time[i]/256;
		msg3[521+25*i] = tgt_hover_time[i] - (unsigned char)(msg3[522+25*i])*256;

		msg3[523+25*i] = tgt_djnum[i];

		msg3[524+25*i] = tgt_djtime[i];

		msg3[525+25*i] = tgt_djrpttime[i];

	}

	write_dataflash (0xc003a330, (long)(&msg3[500]), 25*Waypoint_number+1);
	writebuf2 = 0;

}

void put_parameter(void)
{
	int i,j;
	char temp[10];
	short *ztmp;
	Rudder_MID = msg3[0]*10;
	Aileron_MID = msg3[1]*10;
	Elevator_MID = msg3[2]*10;

	throtMAX = msg3[3]*10;
	throtMIN = msg3[4]*10;



	rudMAX = msg3[420]*256+msg3[421];
	rudMIN = msg3[422]*256+msg3[423];
	ailMAX = msg3[424]*256+msg3[425];
	ailMIN = msg3[426]*256+msg3[427];
	elevMAX = msg3[428]*256+msg3[429];
	elevMIN = msg3[430]*256+msg3[431];

	if (throtMAX - throtMIN != 0)
		Throttle_Factor = 830/(float)(throtMAX - throtMIN);
	else
		Throttle_Factor = 1;

	if (rudMAX - rudMIN != 0)
		Rudder_Factor = 800/(float)(rudMAX - rudMIN);
	else
		Rudder_Factor = 1;

	if (ailMAX - ailMIN != 0)
		Aileron_Factor = 800/(float)(ailMAX - ailMIN);
	else
		Aileron_Factor = 1;

	if (elevMAX - elevMIN != 0)
		Elevator_Factor = 800/(float)(elevMAX - elevMIN);
	else
		Elevator_Factor = 1;


	for(i=1;i<=9;i++)
		conReverseFlag[i]=msg3[5+i];	//reverse3
	for(i=1;i<=40;i++)
		Parameter[i-1]=msg3[14+i];



	if ((Parameter[ERASE_ROMBOOT]==87)||(Parameter[ERASE_ROMBOOT]==88)||(Parameter[ERASE_ROMBOOT]==89)||(Parameter[ERASE_ROMBOOT]==90))
		Parameter[ERASE_ROMBOOT] = 0;



	if (!((Parameter[POWER_NO] & 0x10) >> 4))		//Safety
	{
		ctlock_throttle = 5*hz250;
		Lock_Throttle = 1;
	}
	else									
	{
		ctlock_throttle = 0;
		Lock_Throttle = 0;
	}



	OUT_ARM_FXZHONG = Rudder_MID/10; //Fix calculated rudder neutral value
	OUT_ARM_FYZHONG = Aileron_MID/10; //Fix calculated aileron neutral value
	OUT_ARM_SJZHONG = Elevator_MID/10; //Fix calculated elevator neutral value

	for(i=0;i<4;i++)
		temp[i] = msg3[65+i];
	buf2float(&RollCorrect,&temp[0]);
	for(i=0;i<4;i++)
		temp[i] = msg3[69+i];
	buf2float(&PitchCorrect,&temp[0]);


	Channel8_position = msg3[332]+msg3[333]*256;

	if ((Channel8_position < 1100)||(Channel8_position > 1900))
		Channel8_position = 1500;


	buf2float(&bias_iraw_gyro_1,&msg3[351]);



	if ((gyro_bias_data[0]==0xff)&&(gyro_bias_data[1]==0xff)&&(gyro_bias_data[2]==0xff)&&(gyro_bias_data[3]==0xff))
		buf2float(&bias_iraw_gyro_2,&msg3[355]);
	else
		buf2float(&bias_iraw_gyro_2,&gyro_bias_data[0]);

	if ((gyro_bias_data[4]==0xff)&&(gyro_bias_data[5]==0xff)&&(gyro_bias_data[6]==0xff)&&(gyro_bias_data[7]==0xff))
		buf2float(&bias_iraw_gyro_3,&msg3[359]);
	else
		buf2float(&bias_iraw_gyro_3,&gyro_bias_data[4]);


	buf2float(&bias_acc1,&msg3[395]);
	buf2float(&bias_acc2,&msg3[399]);
	buf2float(&bias_acc3,&msg3[403]);

	buf2float(&kacc1,&msg3[407]);
	buf2float(&kacc2,&msg3[411]);
	buf2float(&kacc3,&msg3[415]);






	if (bias_iraw_acc_3 > 32768) bias_iraw_acc_3 -= 65536;
	if (bias_iraw_acc_2 > 32768) bias_iraw_acc_2 -= 65536;
	if (bias_iraw_acc_1 > 32768) bias_iraw_acc_1 -= 65536;



	buf2float(&xmag_bias,&msg3[363]);
	buf2float(&throtag_bias,&msg3[367]);
	buf2float(&zmag_bias,&msg3[371]);

	buf2float(&Kmag,&msg3[375]);
	buf2float(&Kmag2,&msg3[379]);
	buf2float(&TargetRollBias,&msg3[383]);
	buf2float(&TargetPitchBias,&msg3[387]);
	buf2float(&fxbias,&msg3[391]);


	Waypoint_number = (unsigned char)msg3[500];
	for(i=0;i<Waypoint_number;i++)
	{
		curnum = (unsigned char)msg3[501+25*i]-1;

		Longi_Array[0] = msg3[502+25*i];
		Longi_Array[1] = msg3[503+25*i];
		Longi_Array[2] = msg3[504+25*i];
		Longi_Array[3] = msg3[505+25*i];
		buf2float(&tgt_Latti[curnum],&Longi_Array[0]);

		Longi_Array[0] = msg3[506+25*i];
		Longi_Array[1] = msg3[507+25*i];
		Longi_Array[2] = msg3[508+25*i];
		Longi_Array[3] = msg3[509+25*i];
		buf2float(&tgt_Longi[curnum],&Longi_Array[0]);

		Longi_Array[0] = msg3[510+25*i];
		Longi_Array[1] = msg3[511+25*i];
		Longi_Array[2] = msg3[512+25*i];
		Longi_Array[3] = msg3[513+25*i];
		buf2int(&tgt_High[curnum],&Longi_Array[0]);

		Longi_Array[0] = msg3[514+25*i];
		Longi_Array[1] = msg3[515+25*i];
		Longi_Array[2] = msg3[516+25*i];
		Longi_Array[3] = msg3[517+25*i];
		buf2int(&tgt_Vel[curnum],&Longi_Array[0]);


		tgt_zwl[curnum] = msg3[519+25*i]*256+msg3[518+25*i];
		tgt_hover_num[curnum] = msg3[520+25*i];
		tgt_hover_time[curnum] = msg3[522+25*i]*256+msg3[521+25*i];
		tgt_djnum[curnum] = msg3[523+25*i];
		tgt_djtime[curnum] = msg3[524+25*i];
		tgt_djrpttime[curnum] = msg3[525+25*i];
	}


	//msg3[432]~msg3[463]	共32个
	for(i=0;i<32;i++)
		paraUSER[i] = msg3[432+i];


	for (i=0;i<8;i++)
	{
		paraX[i] = paraUSER[i+8];
		if (paraX[i] > 127) paraX[i] -= 256;
	}
	for (i=8;i<16;i++)
	{
		paraX[i] = paraUSER[i+16];
		if (paraX[i] > 127) paraX[i] -= 256;
	}
	for (i=16;i<24;i++)
	{
		paraX[i] = paraUSER[i];
		if (paraX[i] > 127) paraX[i] -= 256;
	}
	for (i=24;i<32;i++)
	{
		paraX[i] = paraUSER[i-24];
		if (paraX[i] > 127) paraX[i] -= 256;
		paraX[i] = 200 - paraX[i];
	}



	buf2float(&kgyro1,&msg3[464]);
	buf2float(&kgyro2,&msg3[468]);
	buf2float(&kgyro3,&msg3[472]);

	if ((kgyro1 > 0.6)&&(kgyro1 < 1.4)&&(kgyro2 > 0.6)&&(kgyro2 < 1.4)&&(kgyro3 > 0.6)&&(kgyro3 < 1.4))
		serialnumok = 1;
	else
		serialnumok = 0;



}


void AT91F_UART1_HANDLER(void) //UART2 unused, but can be extended.
{
	return;
}







void LimitAccel(float *v)
{
	char i;
	float deltaMax[4];
	float tv[2];

	tv[0] = v[0]*cos_hx+v[1]*sin_hx;			//Used in pitch, downward = positive
	tv[1] = v[1]*cos_hx-v[0]*sin_hx;			//Used in roll, right = positive

	AccelMAX = 9.8*MAXacc;

	deltaMax[0] = AccelMAX*dt;
	deltaMax[1] = AccelMAX*dt;

	for (i=0;i<=1;i++)
	{
		rr[i] = tv[i];
	}
}





void cal_mag()
{
	float sin_gama,cos_gama,cos_theta,sin_theta;
	float tzmag_bias;
	float mag_dec;

	alldata.true_pitch = rpy[1] * deg;
	alldata.true_roll = rpy[0] * deg;

	if (Parameter[ERASE_ROMBOOT]==87)
	{

		if ((fabsf(alldata.true_roll)<3)&&(fabsf(alldata.true_pitch)<3))
		{
			if (g87==0)
			{
				g87 = 1;
				xmag_max = xmag;
				xmag_min = xmag;
				throtag_max = throtag;
				throtag_min = throtag;

				ct_rawmag = 0;

			}


			raw_magx[ct_rawmag] = xmag;
			raw_magy[ct_rawmag] = throtag;
			ct_rawmag++;
			if (ct_rawmag > 1199) ct_rawmag = 1199;

			if (xmag > xmag_max) xmag_max = xmag;
			if (throtag > throtag_max) throtag_max = throtag;

			if (xmag < xmag_min) xmag_min = xmag;
			if (throtag < throtag_min) throtag_min = throtag;

			xmag_bias = (xmag_max + xmag_min)/2;
			throtag_bias = (throtag_max + throtag_min)/2;
		}
	}
	else
		g87 = 0;

	if (Parameter[ERASE_ROMBOOT]==88)
	{
		if ((fabsf(alldata.true_roll)<3)&&(fabsf(alldata.true_pitch)<3))
		{
			if (g88==0)
			{
				g88 = 1;
				zmag_max = zmag;
				zmag_min = zmag;
				ct_rawmagz = 0;
			}

			raw_magyz[ct_rawmagz] = throtag;
			raw_magzz[ct_rawmagz] = zmag;

			ct_rawmagz++;
			if (ct_rawmagz > 1199) ct_rawmagz = 1199;

			if (zmag > zmag_max) zmag_max = zmag;
			if (zmag < zmag_min) zmag_min = zmag;

			zmag_bias = (zmag_max + zmag_min)/2;
		}
	}
	else
		g88 = 0;


	if (Parameter[ERASE_ROMBOOT]==89)
	{
		if (throtag_max - throtag_min != 0)
			Kmag = (xmag_max - xmag_min) / (throtag_max - throtag_min);
		else
			Kmag = 1;

		if (zmag_max - zmag_min !=0)
			Kmag2 = (xmag_max - xmag_min) / (zmag_max - zmag_min);
		else
			Kmag2 = 1;

		writebuf1 = 1;
		Parameter[ERASE_ROMBOOT] = 0;
	}


	if ((Parameter[ERASE_ROMBOOT]!=87)&&(Parameter[ERASE_ROMBOOT]!=88))
	{
		xmag -= xmag_bias;
		throtag -= throtag_bias;
		zmag -= zmag_bias;
		throtag *= Kmag;
		zmag *= Kmag2;
	}

	if ((Parameter[ERASE_ROMBOOT]==90)&&(enter_set!=1))		//display kmag params
	{
		float2buf(&OUT_INA_LONGI,&Kmag);
		float2buf(&OUT_INA_LATTI,&Kmag2);
		float2buf(&OUT_INA_GPSHIGH,&xmag_bias);
		float2buf(&OUT_INA_VELS,&throtag_bias);
		tzmag_bias = zmag_bias * PI / 180;
		float2buf(&OUT_INA_HANGXIANG,&tzmag_bias);	
	}

	gama_m = alldata.true_roll*PI/180;
	theta_m = alldata.true_pitch*PI/180;

	sin_gama = sin(gama_m);
	cos_gama = cos(gama_m);
	cos_theta = cos(theta_m);
	sin_theta = sin(theta_m);
	Gxmag = xmag * cos_theta + throtag*sin_theta*sin_gama + zmag*sin_theta*cos_gama;
	Gthrotag = throtag * cos_gama - zmag*sin_gama;

	if (Gxmag!=0)
		ciheading = atan(fabsf(Gthrotag/Gxmag));
	else
		if (Gthrotag>0)
			ciheading = -PI/2;
		else
			ciheading = PI/2;

	if ((Gthrotag<=0)&&(Gxmag>0)) ciheading = ciheading;
	if ((Gthrotag<=0)&&(Gxmag<0)) ciheading = PI - ciheading;
	if ((Gthrotag>=0)&&(Gxmag<0)) ciheading = PI + ciheading;
	if ((Gthrotag>=0)&&(Gxmag>0)) ciheading = -ciheading;

	if ((unsigned char)Parameter[MAGNETIC_VARIATION] < 127)										//positive
	{
		if ((unsigned char)Parameter[MAGNETIC_VARIATION] <= 100)
			mag_dec =  (float)((unsigned char)Parameter[MAGNETIC_VARIATION])/10;
		else
			mag_dec = (float)((unsigned char)Parameter[MAGNETIC_VARIATION]) - 100;
	}
	else																		//negative
	{
		mag_dec = (float)((unsigned char)Parameter[MAGNETIC_VARIATION]-256);
		if (mag_dec >= -100)
			mag_dec = mag_dec / 10;
		else
			mag_dec = mag_dec + 100;
	}

	ciheading -= mag_dec*PI/180;

	if (ciheading > PI) ciheading -= (float)2*PI;
	if (ciheading < -PI) ciheading += (float)2*PI;

	cal_heading = ciheading;

}



void LimitCourseAngle(float v)
{
	float deltaMax;
	float delta_Angle;
	deltaMax = AngleMax*dt;

	delta_Angle = v-ref_yaw;
	if (delta_Angle > PI) delta_Angle -= 2*PI;
	if (delta_Angle < -PI) delta_Angle += 2*PI;

	if (fabsf(delta_Angle) <= deltaMax)
	{
		ref_yaw = v;
	}
	else
	{
		if (delta_Angle > 0)
			ref_yaw = ref_yaw + deltaMax;
		else
			ref_yaw = ref_yaw - deltaMax;
	}
	if (ref_yaw > PI) ref_yaw -= 2*PI;
	if (ref_yaw < -PI) ref_yaw += 2*PI;

}