
#define g0 9.81

#define deg   57.29577951308232      //弧度转度的因子 180/pi

#define ADI_CS *pmem_355_cs
#define ADI_SCLK *pmem_355_sclk
#define ADI_DIN *pmem_355_din
#define ADI_DOUT *pmem_355_dout
#define con_status SndBuf[92+18]

#define HPD 22
#define UTCtime0 30
#define UTCtime1 31
#define UTCtime2 32
#define UTCtime3 33
#define UTCtime4 34
#define UTCtime5 35
#define UTCtime6 36
#define UTCtime7 37
#define UTCtime8 38
#define UTCtime9 39


#define OUTMIN 100 //输出到舵机的脉宽底限
#define OUTMAX 200 //输出到舵机的脉宽顶限
#define BIG 1
#define SMALL 0
#define OLD 1
#define NEW 0
#define LEA4S 1
#define LEA5S 0
#define CLR_TIME 1000

//#define WENDU spitemp/10


//#define SAMSUNG 1
#define JIDIANQI 1
#define ZHILIAN 0
#define PWM 0
#define HL_H 1
#define HL_L 2
#define SJ2 3
#define BIANJIAO 5

#define NOJY 6
#define JY8 4
#define JY1 5

#define ZIDONG 1
#define SHOUDONG 2

//#define EKFF 1
//#define BANZIDONG	1
#define YUYAN 1
//#define AINCOM 1
#define VERSION1256 1
#define UART_LIANXU 1
#define BAUD_YS 115200

#define dingdianpaizhao 1

#define XUANPIAN 1
#define CRU 1
#define START 2
#define DOWN 3
#define SPIWTTIME 100











#define SLNUM 128

//* prototypes
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
extern double atan(double);
extern double cos(double);
extern size_t strlen(const char * /*s*/);
extern char *strcat(char * /*s1*/, const char * /*s2*/);
extern int memcmp(const void * /*s1*/, const void * /*s2*/, size_t /*n*/);
extern double sqrt(double);
extern double asin(double);

extern float rpy[3];

//下面是3个姿态解算函数
extern void MainCall(void);
extern void InitData(void);
extern void TimerINTCall(void);

float distance_chui(float,float,float,float,float,float);
int tell_line(int,int,int);

#ifdef EKFF
extern float xEKF[16];
#endif 

extern float Butterworth(float ,float *);
float zkongsu[2] = {0,0};

float zp[2] = {0,0};
float zq[2] = {0,0};
float zr[2] = {0,0};

int		count_twi = 0;
char 	twi_error = 0;

extern void fkfangzhenback(void);
extern int fkfangzhen(void);
char startled = 0;
int ctled = 0;
char led_color = 0;
float led_flash_hz = 1;
char start_led_on = 0;
int ct_led_on = 0;
char color_zu[5];
char color_point = 0;


char g87 = 0, g88 = 0;
float raw_magx[1200],raw_magy[1200];
float raw_magyz[1200],raw_magzz[1200];
int ct_rawmag = 0, ct_rawmagz = 0, index_mag = 0;
char plotmag = 0, plotmag_end = 0;
int ctplotmag = 0, cthui = 0;

float nDvalue=0;

extern char CheckValid(float,float);
float zp[2],zq[2],zr[2],zx[2],zy[2],zz[2];
unsigned char IsVelDataNew=0,IsPosDataNew=0,IsMagDataNew=0;
char parameter23 = 65;
unsigned char enable_ctqingqiya = 0, ctqingqiya = 0;

float tCurLongi=0,tCurLatti = 0;

int LaneCount_smooth=0;

char sbus_error=0,ctsbus_error=0;
float sbus_yinzi = 0.62595;

char SAMSUNGstatus = 0;

char sumstatus = 0;

double presnew;
float tpresnew;
float dianya1,dianya2,dianya3;
char showmag=0;


char first_ = 0;

//char YOU_MAG = 1;
char YOU_MAG = 0;

char error_limit = 0;

int count_error = 0;

float kgyro1=1,kgyro2=1,kgyro3=1;

float distance(float,float); //在俯视平面内计算x,y到 与 上一目标点与当前目标点连线 相垂直 且过当前目标点 直线的"距离"
float distance2(float,float); //在俯视平面内计算x,y到 与 上一目标点与当前目标点连线 相垂直 且过当前目标点 直线的"距离"
float newdistance(void);  //计算当前点与当前航线的距离
float distance_photo(int,int);  //计算当前点与照片连线的距离
float distance_dian(float,float,float,float);  //wangnew
void put_parameter(void);
void do_writebuf1(void);
void do_writebuf2(void);

extern void cal_mag(void);

char mainmag = 0, mainqiya = 0;

int ACC_NUM = 1, MAG_NUM = 2;
float xmag=0,ymag=0,zmag=0,xmag_max=0,ymag_max=0,zmag_max=0,xmag_min=0,ymag_min=0,zmag_min=0,xmag_bias=0,ymag_bias=0,zmag_bias=0;
float mag_3_zu[100],mag_2_zu[100],mag_1_zu[100];
float fxmag=0,fymag=0,fzmag=0,fxmag2=0,fymag2=0,fzmag2=0;
unsigned int ctfashe;
float cihangxiang = 0;
float calhangxiang = 0;
float theta_m,gama_m;
float Gxmag=0,Gymag=0,Gzmag=0;
float Kmag=1,Kmag2=1;


unsigned char ctmag = 0, ctmag2 = 0, ctmag3 = 0;
unsigned char magdata = 0, magdata2 = 0, magdata3 = 0, magdata4 = 0, magdata5 = 0, magdata6 = 0;

float sum_raw_mag_1,sum_raw_mag_2,sum_raw_mag_3;

void wrSPIbit1(void);
void wrSPIbit0(void);

void reset5534(void);
void rd_adi355(void);
void wr_adi355(unsigned char, unsigned char);
void put_data(void);
void put_data1256(void);
void rd_adi1256(void);
void rd_adi355_addr(unsigned char);
void rdSPIpara(void);
void rdTWIad(void);
unsigned short rdSPIw1(void);
unsigned short rdSPIw2(void);
unsigned short rdSPIw3(void);
unsigned short rdSPIw4(void);

void rdSPID1(void);
void rdSPID2(void);
void put_newboard_parameter(void);
unsigned short rdSPIw1new(void);//add
unsigned short rdSPIw2new(void);
unsigned short rdSPIw3new(void);
unsigned short rdSPIw4new(void);

void cal_skyway_distance(void);
int tc2 = 3000;



char gpsshua_error = 0;

unsigned int gpsshua[4];

char clear_acc = 0;
float bias_acc1=0,bias_acc2=0,bias_acc3=0;
float kacc1=1,kacc2=1,kacc3=1;


float xiuzheng12 = 1.0;


float true_heading = 0;

//float DIANLIU_ZEROV = 0.63;
float DIANLIU_ZEROV = 5;

char SBUS = 0;		//1为6通道接SBUS

float lad = 0;

unsigned short serialnum = 0;
char serialnumok = 0;

unsigned char count_show_serial = 0, show_serial=0;

float deltalongi=0,deltalatti=0;

char  Receiver_Status = 1;	//接收机状态，开还是关
char  TakePhotoFlag = 0, ReadPhotoInfo = 0;	//拍照的相关变量
char  TakePhotoSeries = 0;
int SeriesCount = 0;

char GPSready = 0;
char gps_buffer[1000],wifi_buffer[1000];



float ymsum= 0;

int takephototime = 0;
int   TakePhoto_time = 0, data_size = 0;	//拍照的相关变量
float dianliuI = 0, BatteryConsume = 0;
int dianliuII = 0, BatteryConsumeI = 0;

char  TakePhotoAtPoint = 0, newphoto = 0;
int   photo_num = 0, AtPhoto;
float tlatti1,tlatti2,tlongi1,tlongi2,photolatti[1000],photolongi[1000],dphotolatti[1000],dphotolongi[1000],tdistance;
float curdistance = 0;
float photo_latti[5000],photo_latti2[5000];
float photo_longi[5000],photo_longi2[5000];

char test_la=60,la_num=0;
int lyjt_num = 0, ljjt_num = 0;

char SGPS_VALID = 0;		//双GPS定向是否有效
char ctirq = 0;

char status = 0;
int cur_photo_num = 0;
char havesendback_qrp1 = 1, havesendback_qrpall = 1;
char havesendback_dat1 = 1, havesendback_dat2 = 1;
char dou_count = 0;
char tiqu[20];
int	  d13;	//平飞迎角
int   d32;	//平放迎角
char t2p1 = 0;
int		ct_sendci = 0;
int WENDU = 0;
//need1256
char spitempvalid = 0;
float temp1256;
float vout1zhong[401],vout2zhong[401],vout3zhong[401];	//记录每个vout在每个温度的零位, 0~40度
int vout1count[401],vout2count[401],vout3count[401];			//记录每个温度值里采样的数量
float vout4zhong[401],vout5zhong[401],vout6zhong[401];	//记录每个vout在每个温度的零位, 0~40度
int vout4count[401],vout5count[401],vout6count[401];			//记录每个温度值里采样的数量
 
int countSGPS = 0;
float nnnnn = 0;
int  shoudongzitai = 0;

float theta=0;

int painum = 0, pai_curnum=0, qrpnum = 0;
unsigned char havesendback_painum = 1;

int oldym = 193;

int nloop = 0;
int nMainloop = 1;
int nfangzhenTestCount = 0;
int nfangzhenTestPos = 0;

struct FANGZHEN_1_DATA gfangzhen_1_data;
struct FANGZHEN_2_DATA gfangzhen_2_data;

struct FANGZHEN_1_DATA g_fangzhendata;


int groundFangzhen = -1111;
int FangZhenDataFlag = 0;
int gpsvalidFlag = 0;
int FangZhenDataclearflag= 0;

float fAirHigh = 0;

unsigned short parFlag = 0;
char pulse4hz,cnt4hz;
float gpsground;
int fangzhenmode = 0;
char initflag = 0;

float pres,h,alt0,Pres0;
char ctclearflag = 0;

int hz = 250;


float TargetRoll = 0, tTargetRoll;	//目标横滚角，单位为度
float TargetRollBias = 0, TargetPitchBias = 0;
float fxbias = 0;


float tTargetPitch = 0, TargetPitch;	//目标俯仰角

float RADIUS = 0.5;					//盘旋半径系数 
char intestxuanpian = 0; 

int JIANSHI = 0, NUM = 500;
float pianhangju = 0;

float tgtlattis,tgtlongis,tgthighs;

float	aa = 6378137.0;
float	ff = 1.0/298.257223563;
float	e2;


char send_xuanzhuan = 0, send_fuyang = 0, newpanxuan = 0;
char endinit = 0;				//初始化结束标志
unsigned char newBoard = 0;	//是否新的空板子，1为空板子 

unsigned char version16365 = ZIDONG;//SHOUDONG为原来的程序，ZIDONG为FPGA自动按时序读取数据
unsigned int versionDD =  2;	//1为电动版配16355，0为配中星IMU，2为最新AD1256，16365为使用ADIS16365

unsigned char versionFAN = 1;	//前后是否反装
unsigned char versionRW8 = HL_H;	//8通道用做熄火,高电平熄火
unsigned char versionXIANGJI = ZHILIAN;	//JIDIANQI为继电器，ZHILIAN为直连；
//unsigned char versionXIANGJI = ZHILIAN;	//JIDIANQI为继电器，ZHILIAN为直连；
//unsigned char versionDIAN = 0;	//1为电动,测量的是电流

unsigned char versionKAISAN_YANSHI = 1;


char init16355 = 0;

//以下的不动
unsigned char versionJY = NOJY;
unsigned char versionXUANPIAN = 0;
unsigned char versionSJ2_1 = 0;

unsigned char bhead_pai=0, haveSendBack_pai1 = 1;
int protect_high = 80;
int ctshoudong = 10;
int ctmieche = 10;
int tvely_i,tvelx_i,tvelz_i, gps_yaw, old_gps_yaw = 0;	//GPS数据，各向速度、航向等
int	zhishi_time = 0,ct_zhishi_time = 0,start_zhishi = 0;
float gps_angle0=0,gps_angle1=0,gps_angle2=0,gps_angle3=0,gps_angle4=0;	//存上几秒航向的变量
int errorflash = 0,in_irq0=0;	//见程序里的注释
float vels = 0;	//GPS地速
int ivels = 0;
//float air_xishu = 0.911;
//char air_xishu_flag = 0;
int Simulate_High = 0, climb_rate_05s, oldAirhigh_05s=0, climb_rate_1s,oldAirhigh_1s=0; //跟爬升率、模拟飞行高度等有关的变量
unsigned int ct_fall = 0;

int	swordw1,swordw2,swordw3,swordw4,swordD1,swordD2;	//跟压力传感器有关的变量
int	swordw1new,swordw2new,swordw3new,swordw4new,swordD1new,swordD2new;//跟压力传感器有关的变量
int		c1,c2,c3,c4,c5,c6;					//跟压力传感器有关的变量
int		c1new,c2new,c3new,c4new,c5new,c6new;//跟压力传感器有关的变量
float yt_fuyang,yt_pingzhuan;
int pingzhuantmp,fuyangtmp;

int tmph,tmpl;	//用来将两字节数放到某个char型地址去的中间变量
int tmphh,tmpll;
float outhz = 400, outwidth = 2500;


int limited = 0;
char GPS_status = 0, NEI_status = 0, xiaoshu = 0;
int tintt = 0;
int newposition = 0;
char initc[100]={0xb5,0x62,0x06,0x08,0x06,0x00,0xfa,0x00,0x01,0x00,0x00,0x00,0x0f,0x94};	//start GPS 4HZ
char setubx[]={0xb5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xd0,0x08,0x00,0x00,0x00,0x96,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x8b,0x54};	//set UBX protocol
char disable_msg[100]={0xb5,0x62,0x06,0x01,0x06,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x0f,0x94};	//disable GPS no use msg
char enable_msg[100]={0xb5,0x62,0x06,0x01,0x06,0x00,0x01,0x12,0x00,0x00,0x01,0x00,0x0f,0x94};	//enable GPS need msg

int ctstart = 0;
float kwendu = 0;
//以下为云台控制协议的数据
char yuntai_ting[]={0xa0,0x10,0x00,0x00,0x00,0xaf};
char yuntai_lajin[]={0xa0,0x11,0x00,0x05,0x00,0xaf};
char yuntai_layuan[]={0xa0,0x11,0x01,0x05,0x00,0xaf};
char yuntai_xuanzhuan[]={0xff,0x01,0x00,0x91,0x00,0x00,0x00};
char yuntai_fuyang[]={0xff,0x01,0x00,0x93,0x00,0x00,0x00};

char yuntai_osdon[]={0xa0,0xa4,0x01,0x00,0x00,0xaf};
char yuntai_osdoff[]={0xa0,0xa4,0x00,0x00,0x00,0xaf};

char proc_buffer[2000],dbg_buffer[2000],pgps_buffer[2000],pwifi_buffer[2000];	//DBG口的UART中断的处理缓存
unsigned int proc_index = 0, proc_end = 0, gps_index = 0, gps_end = 0, wifi_index = 0, wifi_end = 0;	//DBG口的UART中断的处理索引变量

char inn[120],innci[120];
char jiou = 0, P11 = 1;
float tyaw = 0;

short	Standard_Lane[8][SLNUM*4],Standard_Lane2[8][32];	//一共有8种制式，每种制式航线包括8个航点，每个航点有相对坐标经度、纬度，都用short，共32字节；

char dbg_uart_mode = 1;	//有否地面站板子的标志，如果为0，则有，需要来一串数据再产生一次中断然后集中处理

int	ut1,dt_5534,spitemp=0,tspitemp = 0,off,sens,x5534,spip,p2_5534,t2_5534,groundp,AirHigh=0,p2new,t2new,photoAirHigh;

float AirHighf,aver;

float AirHigh5=0,AirHigh4=0,AirHigh3=0,AirHigh2=0,AirHigh1=0,AirHigh0=0;
int	ut1new,dtnew,spitempnew,offnew,sensnew,xnew,spipnew,gaodunew,groundpnew=10300,DeltaPressure=0;	//跟压力传感器相关的变量
int tchange;
int AirHighCorrect=0,AirSpeedCorrect=0;	//跟压力传感器相关的变量

float vout1zu[1501],vout2zu[1501],sumvout1,sumvout2,vout1out,vout2out;



unsigned char ct_gps = 0;//计算GPS是否到了4次，即可以参与一次运算的变量
int time_count=0, count_AD = 0, count_SPI = 0, count_dis = 0;
int count_current = 0;
unsigned short run_time=0,fly_time=0;	//开机时间和起飞时间

float xuanpianjiaodu = 0, tgtjiaodu = 0, tempjiaodu = 0, outxuanpianjiaodu = 0, zxuanpianjiaodu = 0;
float liandong;
int tfx;
char fly_flag = 0;
float CIhangxiang = 0;
char sel_video = 0x15, video_sel_count = 0;
unsigned char qingkongsuflag = 0, qingjiaodu = 0, qingqiya = 0;
unsigned char pdop_h,pdop_l,year_l,year_h,kaisanFlag=0,xihuoFlag=0,huishou_flag = 0;
float PDOP;
float tcurlatti,tcurlongi;
float photoCurLatti,photoCurLongi,photoPitch,photoRoll,photoPlaneYaw;
unsigned int ct_kaisan = 0, ct_kaisan_flag = 0;
char HaveCalculateArrive=1;	//是否已判断过到达目标点
char buffer[1000],ct_gaodu=0,cha_3=0,count_cha_3=0;
unsigned char error_zhendong=0;
unsigned short QIDONG,YUNXING,RENWU8;
int	fpgain1=1000,fpgain2,fpgain3=1500,fpgain4=1500,fpgain5=1500,fpgain6=1500,fpgain7,fpgain8,fpgain9=1000,tfpgain1=1000,tfpgain3=1500,tfpgain4=1500,tfpgain5=1500,tfpgain6=1500,tfpgain7=1500,tfpgain8=1500;
int out1=1500,out2=1500, zout1=1500,zout2=1500,jyout1=1500,jyout2=1500;
int fpgain1old,fpgain2old,fpgain3old,fpgain4old,fpgain5old,fpgain6old,fpgain7old,fpgain8old,fpgain9old;
unsigned int cnt1=0,cnt2=0,cnt3=0,cnt4=0,cnt5=0,cnt6=0,cnt7=0,cnt8=0,cnt9=0;
char writebuf1 = 0, writebuf2 = 0, WritePhoto = 0;
float temp_mti_zhuanvel,mti_zhuanvel,mti_zhuanvel0,mti_zhuanvel1,mti_zhuanvel2,mti_zhuanvel3;
float nq0,nq1,nq2,nq3;
float fcr,fcp,fcy,fsr,fsp,fsy,r11,r12,r13,r21,r22,r23,r31,r32,r33,zhi_x,zhi_y,zhi_z,zhi_l,s_x,s_y,s_z;
float gacc_x = 0, gacc_y = 0, gacc_z = 0, gyro_x = 0, gyro_y = 0, gyro_z = 0;

char high100 = 0;	//zhuhai
int ctzhuansu = 0;

char error_gaodu = 0, error_kongsu = 0;

unsigned char tuiflag = 0, laflag = 0, pingflag = 0;		//zhuhai
unsigned char allready1 = 0, allready = 0;								//zhuhai
int tuicount = 0, lacount = 0;
unsigned int ctkaisan = 0;
char shoudongkaisanflag = 0, ctshoudongkaisan = 0, ctqiesan = 0;

int sendcount = 0; 
char rst355 = 0, cntrst355 = 0, ctfuwei = 0, ctall0 = 0, all0flag = 0;
char docalct=0, calfyct = 0, calsjct = 0;
int calymct = 0;
char inwhile1 = 0;
char eraseromboot = 0;
unsigned char cnt_PlanePitch_sum=0,cnt_PlaneRoll_sum=0,bianjiao_temp=1,bianjiao=1;
unsigned int handcount = 0;
char cthavesendbackall = 0, ctindex = 0;
char cthavesendback_pai1 = 0;
char RunOnGnd_status = 0,ji_RunOnGnd_status = 0, shanhe_status = 0, shanhe_b_status = 0;
int glide_angle = 0, glide_flag = 0;

float	alpha=0,alpha_old=0,alpha_sum=0;		//方向舵计算时的误差各参数；
float	alpha_angle0,alpha_angle1,alpha_angle2,alpha_angle3,alpha_angle4;
float 	d_yaw_vel=0;

char	clearflag = 0;
float	PlaneRoll_sum=0,PlanePitch_sum=0,jiD = 0, jiBPD = 0;
int		old_DeltaHigh=0,ctd=0,g2j_D,ctd_true =0,ctd1=0,ctdnum = 6;

int		xuanzhuan_pianyi = 0,fuyang_pianyi = 0;

int		Cnt_SimHigh=0,countBPD = 0,  sumDeltaHigh;
float	ElevatorCompensate;
float	yaw_vel=0, t_yaw_vel = 0;
char yuntai_tmp[2],yuntai_cmp;
int	yuntai_H_bushu,yuntai_V_bushu;
float yuntai_fuyangjiaodu,yuntai_xuanzhuanjiaodu;
float bias_xuanzhuanjiaodu = 0, bias_fuyangjiaodu = 0, tbias_xuanzhuanjiaodu = 0;
float tjiaodu;

char goback = 0,shoudong_flag = 1;
char shoushu = 0;
unsigned char ctshoushu = 15,SimCount = 0;
char manual_auto = 0;
float juli0,juli1,juli_xishu;		//wangnew
int docalgaodu = 0;


unsigned char real = 1;
float hvel=0,oldDistance;
int intgyro_1,intgyro_2,intgyro_3;
int iraw_acc_1,iraw_acc_2,iraw_acc_3,iraw_gyro_1,iraw_gyro_2,iraw_gyro_3,tmpacc_1,tmpacc_2,tmpacc_3, jiaohuan,tt1,tt2,tt3;
float acc1,acc2,acc3;
float fraw_acc_1,fraw_acc_2,fraw_acc_3,fraw_gyro_1,fraw_gyro_2,fraw_gyro_3;
float irawacc_1_zu[200],irawacc_2_zu[200],irawacc_3_zu[200],gyro_3_zu[200];
float voutp_zu[200],sum_voutp=0;
float bias_iraw_acc_1=0,bias_iraw_acc_2=0,bias_iraw_acc_3=0,bias_iraw_gyro_1=0,bias_iraw_gyro_2=0,bias_iraw_gyro_3=0,bias_iraw_gyro_1_zu=0,bias_iraw_gyro_2_zu=0,bias_iraw_gyro_3_zu=0;
float tbias_iraw_gyro_1,tbias_iraw_gyro_2,tbias_iraw_gyro_3=0;
float sum_raw_acc_1=0,sum_raw_acc_2=0,sum_raw_acc_3=0,sum_gyro_3=0;
float acc_1,acc_2,acc_3,gyro_1,gyro_2,gyro_3;
float tg5=0,tg4=0,tg3=0,tg2=0,tg1=0,d_gyro_1;
float ntg5=0,ntg4=0,ntg3=0,ntg2=0,ntg1=0,d_gyro_2;
float ztg5=0,ztg4=0,ztg3=0,ztg2=0,ztg1=0,d_gyro_3;

char chuan1,chuan2,chuan3;
char AirHighCount = 0, AirHighCount2 = 0;

unsigned short count_irq0 =0,old_count_irq0=0;
char kongsu_protect = 1, ControlError = 0;
float all_dis_m = 0, cur_dis = 0, dis_flied = 0, dis_flying = 0, start_dis = 0;

char StandardLaneFlag = 0;
int  StandardLaneCount = 0;
char vv_JF;		//副翼、升降混控
char vv_FS;		//方向、升降混控
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
char * pmem_bus23 = (char *)0x3000303a;
char * pmem_bus24 = (char *)0x3000303b;
char * pmem_bus25 = (char *)0x3000303c;
char * pmem_bus26 = (char *)0x3000303d;
char * pmem_bus27 = (char *)0x3000303e;
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
char * pmem_fpga1l = (char *)0x30003011;		//视频选择
char * pmem_fpga2h = (char *)0x30003052;
char * pmem_fpga2l = (char *)0x30003012;
char * pmem_FANGXIANGh = (char *)0x30003053;
char * pmem_FANGXIANGl = (char *)0x30003013;
char * pmem_FUYIh = (char *)0x30003054;
char * pmem_FUYIl = (char *)0x30003014;
char * pmem_SHENGJIANGh = (char *)0x30003055;
char * pmem_SHENGJIANGl = (char *)0x30003015;
char * pmem_YOUMENh = (char *)0x30003056;
char * pmem_YOUMENl = (char *)0x30003016; 
char * pmem_kaisanh = (char *)0x30003057;
char * pmem_kaisanl = (char *)0x30003017;
char * pmem_fpga8h = (char *)0x30003058;
char * pmem_fpga8l = (char *)0x30003018;		//拍照
char * pmem_fpga9h = (char *)0x30003059;
char * pmem_fpga9l = (char *)0x30003019;
char * pmem_fpga9t = (char *)0x30003039;

char * pmem_WORD1 = (char *)0x30003071; 
char * pmem_WORD1new = (char *)0x30003072;
char * pmem_wrSPI = (char*)0x30003073;
char * shua = (char*)0x30003074;
char * pmem_355_rst_ = (char*)0x30003033;
char * pmem_355_cs = (char*)0x30003032;
char * pmem_355_sclk = (char*)0x30003034;
char * pmem_355_din = (char*)0x30003036;
char * pmem_355_dout = (char*)0x30003038;

char * pmem_poke = (char *)0x3000307f;

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

unsigned int data_1256, acc1_1256,acc2_1256,acc3_1256,gyro1_1256,gyro2_1256,gyro3_1256;
int duan;

int data1[10],data2[10],data3[10],data4[10],data5[10],data6[10],data7[10],data8[10];
float dy1[10],dy2[10],dy3[10],dy4[10],dy5[10],dy6[10],dy7[10],dy8[10];

int adi1[5],adi2[5],adi3[5],adi4[5],adi5[5],adi6[5],adi7[5],adi8[5];
int data1a,data2a,data3a,data4a,data5a,data6a,data7a,data8a;
float dy1a,dy2a,dy3a,dy4a,dy5a,dy6a,dy7a,dy8a;
int idata1a,idata2a,idata3a,idata4a,idata5a,idata6a,idata7a,idata8a;
float WTTIME = 50;
float VREF = 2.5;
float vout1,vout2,vout3,facc1,facc2,facc3,fgyro1,fgyro2,fgyro3,voutp,vout12,voutp_average;
float vout4,vout5,vout6;

float voutwendu = 2.5, voutqiya = 2.5;

unsigned char iic_wrbuf[30],iic_rdbuf[30],con[50],cont[50],conReverseFlag[20]={1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},tmpcont1,tmpcont2,tmpcont3,tmpcont4,con_error=0;

char oldsatelnum = 5;	//用来判断是否刚定位，以便启动姿态解算函数

char FYZW = 1, panflag = 0, ResetFlag = 0, StartTakePhotoAtInterval = 0;

unsigned char painum_flag = 0, qrphead = 0;
unsigned char painum_t = 0, painum_t2 = 0, qrpnum_t, qrpnum_t2;
unsigned char sixchar,fifthchar,forchar,trdchar,sndchar,fstchar,forchar2,trdchar2,sndchar2,fstchar2,forchar3,trdchar3,sndchar3,fstchar3,sixchar4,fifchar4,forchar4,trdchar4,sndchar4,fstchar4;
unsigned char ReceiveStdLaneFlag=0,bhead=0,dinflag=0,di2flag=0,bhead2=0,bhead3=0,rdParaHeadFlag=0,wrParaHeadFlag=0,allwrParaHeadFlag1 = 0,allwrParaHeadFlag2=0,newParaData=0,send_satelnumflag=1,haveSendBack=1,haveSendBack_all = 1,sendback_din=1,StandardLane_havesendback=1,senddin=1,changeTargetFlag=0,ytjd1Flag=0,ytjd2Flag=0,changeGaoduFlag=0,changeSuduFlag=0,tgtnum_flag=0,StandardLaneCt = 0,StandardLaneNum=0,bhead_satelnum=0,bhead_position=0,bhead_angle=0,bhead_time=0,sixchar3,fifchar3, sendpara = 1, havesend_t2p = 1, sendparact = 0;
int tgtnum=0,tgtnum_t=0,curnum=0,c_curnum=0;
unsigned char bhead_lyjt=0,bhead_ljjt=0;
unsigned char spahead = 0, para_data, para_num, para_data2, para_num2;
char cthavesendback = 0;
unsigned char XCDJReceived=0,DJFXReceived=0;
unsigned char CaptureFlag=0,CaptureFlagmax=0,CaptureFlagmin=0,CaptureFlagstop=0,Captureguansan=0,Capturekaisan=0,CaptureCUT=0,CaptureNOTCUT=0,CaptureS1=0,CaptureS2=0,CaptureS3=0,CaptureWEI91=0,CaptureWEI92=0;
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

float COS_FACTOR= 0.766, SIN_FACTOR = 0.6427876;

int	v33U=0,v12U=0,vdianhuoU,vduojiU;

int	vFORCEU=0,vCURRENTU;
int old_v12U=0, old_vFORCEU=0, old_vCURRENTU=0;
char cctcuowu=0, cctcuowu_vforce = 0, cctcuowu_vcurrent=0;
float vforce=0, vcurrent = 0;
int ctpower = 0, power_zhen = 0;
float xiuzhengFORCE = 1.255;

char tiBuf[20];

char ctspi = 0,jiangstatus=0;
unsigned char PitchCount = 0,ctwending = 0;

int nian,yue,ri,shi,fen,miao;

unsigned int sjcount1=0,sjcount2=0;
unsigned char flystatus = START;

unsigned int 	StTick;
unsigned int avst = 0;
unsigned char OUT_ARM_AUTOHANDLEBF;

int CH5;

unsigned char tzhuanwan[2],tzhishitime[2];

char ClickSetTgtFlag = 0;	//是否点哪飞哪的标志位
char first_reach = 0, yuntai_suoding = 0, yici_suoding = 0,count_banmiao = 25;

unsigned short tOUT_ATL_FXDUO,tOUT_ATL_FYDUO,tOUT_ATL_SJDUO,tOUT_ATL_YMDUO;
unsigned short tOUT_ARM_FXDUO,tOUT_ARM_FYDUO,tOUT_ARM_SJDUO,tOUT_ARM_YMDUO;

int tJY;

float TakePhoto_s_Latti;
float TakePhoto_s_Longi;

float tgt_Latti[512];
float tgt_Longi[512];
int	  itgt_Latti[512];
int	  itgt_Longi[512];
int   tgt_High[512];
int   tgt_Vel[512];
unsigned short tgt_zwl[512];		//提前转弯量, 2 bytes
unsigned char  tgt_zhinum[512];	//进入制式航线序号，0为无效, 1 byte
unsigned short tgt_zshitime[512];	//制式时间,  2 bytes
unsigned char  tgt_djnum[512];		//执行舵机序号, 1 byte
unsigned char  tgt_djrpttime[512];	//打开关闭延时时间, 1 byte
unsigned char  tgt_djtime[512];	//重复时间, 1 byte

float tgt_Latti2[512]; 
float tgt_Longi2[512];
int   itgt_Latti2[512];
int   itgt_Longi2[512];

int   tgt_High2[512];
int   tgt_Vel2[512];
unsigned short tgt_zwl2[512];		//提前转弯量, 2 bytes
unsigned char  tgt_zhinum2[512];	//进入制式航线序号，0为无效, 1 byte
unsigned short tgt_zshitime2[512];	//制式时间,  2 bytes
unsigned char  tgt_djnum2[512];		//执行舵机序号, 1 byte
unsigned char  tgt_djrpttime2[512];	//打开关闭延时时间, 1 byte
unsigned char  tgt_djtime2[512];	//重复时间, 1 byte
 
unsigned char tverify;

float tempLatti; 	//当前盘旋坐标点
float tempLongi; 

void buf2float(float *tfloat, char *buf);
void buf2float2(float *tfloat, char *buf);
void buf2long(long *tfloat, char *buf);
void buf2int(int *tint, char *buf);
void float2buf(char *buf,float *tfloat);
void int2buf(char *buf,int *tint);
char CalFxWidth(void);
float CalFyWidth(void);
float CalSjWidth(void);
unsigned char CalYmWidth(void);
float CalAtanVector(void); //计算vectorLongi/vectorLatti的反正切函数，返回范围在0~PI之间，考虑cos_factor
float CalAtanVectorN(float ta, float tb); //计算vectorLongi/vectorLatti的反正切函数，返回范围在0~PI之间，不考虑cos_factor

char PointArrive(void);
void doCal(void);

void doCapture(void);
void doCaptureYMMAX(void);		//捕获油门最大位置
void doCaptureYMMIN(void);		//捕获油门最小位置
void doCaptureYMSTOP(void);		//捕获油门熄火位置
void doCapturekaisan(void);
void doCaptureguansan(void);
void doCaptureCUT(void);
void doCaptureNOTCUT(void);
void doCaptureS1(void);
void doCaptureS2(void);
void doCaptureS3(void);
void doCaptureWEI91(void);
void doCaptureWEI92(void);
int iCurLatti,iCurLongi;

float CurLatti = 0,inaCurLatti=0,inaCurLongi=0, CurLatti_old = 0;
float CurLongi = 0, CurLongi_old = 0;
int intCurLatti = 0, intCurLongi = 0;

float ytmb_latti = 0,ytmb_longi = 0;
long  qCurLatti,qCurLongi,qGPShigh;
float startLatti = 0, startLongi = 0;

float lCurLatti = 0;
float lCurLongi = 0;
float GPShigh,ytmb_height=0;
int iGPShigh;

int	  tvels_i = 0;

float PlaneYaw,Simulate_PlaneYaw=0;
int iPlaneYaw = 0;


char satelnum=0,char_satelnum = 0, satelnumflag = 0, t_satelnumflag = 0;
unsigned char verify;
unsigned short FX=1500,FY=1500,SJ=1500,YM=1000;


float tgyro_1_10=0,tgyro_1_9=0,tgyro_1_8=0,tgyro_1_7=0,tgyro_1_6=0,tgyro_1_5=0,tgyro_1_4=0,tgyro_1_3=0,tgyro_1_2=0,tgyro_1_1=0,tgyro_1=0;
float tgyro_2_10=0,tgyro_2_9=0,tgyro_2_8=0,tgyro_2_7=0,tgyro_2_6=0,tgyro_2_5=0,tgyro_2_4=0,tgyro_2_3=0,tgyro_2_2=0,tgyro_2_1=0,tgyro_2=0;
float tgyro_3_10=0,tgyro_3_9=0,tgyro_3_8=0,tgyro_3_7=0,tgyro_3_6=0,tgyro_3_5=0,tgyro_3_4=0,tgyro_3_3=0,tgyro_3_2=0,tgyro_3_1=0,tgyro_3=0;

float	tgtLatti,tgtLatti2,tgtLongi,tgtLongi2;
int itgtLatti2,itgtLongi2;
float	tgtHigh;	 //目标飞行高度
float	tgtVel;		 //目标飞行速度
float	tgtLatti_s;  //上一任务点y坐标
float	tgtLongi_s;  //上一任务点x坐标
float 	vectorLatti;
float	vectorLongi;

float	AirSpeed,AirSpeedPara = 1;
float   pressure, v_pressure_init;

char SndBuf[OUTLENGTH+119];
char fashe[OUTLENGTH+100],iii=0,fashe_6li[100]; 
char msg[150000],Photo_Info[160000],msg2[10000],msg3[20000];
int ReadPhotoEndCount = 0;
char PhotoInfo_SndBuf[400];
unsigned char	ZHEN_0[] = {"$STP"};

unsigned char 	nTrkPt=0,nTrkPtTemp=0,nTrkPtVerify=0,gaodu_verify=0,AirHigh_h=0,AirHigh_l=0,tgaoduh,tgaodul,ygaoduh=0,ygaodul=0,ytgaoduh,ytgaodul;
unsigned char	bAutoHand_=0;
unsigned char 	bReachTgt=0;

unsigned short 	tOUT_FXDUO=1500,tOUT_FYDUO=1500,tOUT_SJDUO=1500,tOUT_YMDUO=1500;
unsigned short  fan_tOUT_SJDUO = 1500;

unsigned short	FXZHONG = 1500;
unsigned short 	FYZHONG = 1500;
unsigned short 	SJZHONG = 1500;
unsigned short 	ZUO = 1500, YOU = 1500;
int	xs1=1,xs2=1;

unsigned short 	YMMAX=1000, tYMMAX = 2000;
unsigned short	YMMIN=1900;
unsigned short	YMSTOP=2000,XUNIYOUMEN=2000;
unsigned short 	KAISAN,GUANSAN,WEI91=1500,WEI92,CUT,NOTCUT;
int	TAIQI=1000,ANXIA=2000;

unsigned char	 bEast,tbeast,ytbeast;
unsigned char	 bNorth,tbnorth,ytbnorth;
unsigned char	 bGT45,bfan,ybfan;
unsigned char	 bLeft;	//1:表示航向在目标的右边,飞机该往左飞;
						//0:表示航向在目标的左边,飞机该往右飞;
						
unsigned char	ParaData[1000],backup[1000];
unsigned char	Parameter[100];//实际的参数
unsigned char ddata[2000];
int adi_data;
unsigned char	SetAirSpeed = 90;
float iSetAirSpeed = 90, fSetAirSpeed = 90;

unsigned char ctmoni = 0;
unsigned char ct_mpxh = 0, ct_v12=0;

unsigned char havesendback_takephoto = 1;  
AhrsData alldata;	//用来与姿态解算函数进行数据交换的结构体，其定义参见example.h
