
#ifndef FANZHENDATA_h
#define FANZHENDATA_h

#include "uartComm.h"

struct FANGZHEN_1_DATA {

    unsigned short head;//包头	2
    unsigned char len ;//包长	1
    unsigned char type;//包类型	1
    unsigned short PWM1 ;//PWM1	2
    unsigned short PWM2 ;//PWM2	2
    unsigned short PWM3 ;//PWM3	2
    unsigned short PWM4 ;//PWM4	2
    unsigned short PWM5 ;//PWM5	2
    unsigned short PWM6 ;//PWM6	2
    unsigned short PWM7 ;//PWM7	2
    unsigned short PWM8 ;//PWM8	2
    unsigned short PWM9 ;//PWM9	2
    unsigned short PWM10 ;//PWM10	2
    float weidu;//纬度	4
        float jingdu;//经度	4
    float gaodu;//海拔高度	4
    float disu_bei;//地速-北	4
    float disu_dong;//地速-东	4
    float disu_xia;//地速-下	4
    float zhoujiaosu_qian;//三轴角速度-向前	4
    float zhoujiaosu_you;//三轴角速度-向右	4
    float zhoujiaosu_xia;//三轴角速度-向下	4
    float zhouxiansu_qian;//三轴线速度-向前	4
    float zhouxiansu_you;//三轴线速度-向右	4
    float zhouxiansu_xia;//三轴线速度-向下	4
    unsigned short jingya ;//静压	2
    unsigned short dongya ;//动压	2
    float yingjiao ;//迎角	4
    float cehuajiao ;//侧滑角	4
    
    float Undefined1;	// 4
    float Undefined2;	// 4
    unsigned short Undefined3;	// 2

	
    unsigned char sum ;//校验和	1
    unsigned char tail ;//包尾	1

	
} ;
struct FANGZHEN_2_DATA {
    unsigned short head;//包头	1
    unsigned char len ;//包长	2
    unsigned char type;//包类型	1
    unsigned short PWM1 ;//PWM1	2
    unsigned short PWM2 ;//PWM2	2
    unsigned short PWM3 ;//PWM3	2
    unsigned short PWM4 ;//PWM4	2
    unsigned short PWM5 ;//PWM5	2
    unsigned short PWM6 ;//PWM6	2
    unsigned short PWM7 ;//PWM7	2
    unsigned short PWM8 ;//PWM8	2
    unsigned short PWM9 ;//PWM9	2
    unsigned short PWM10 ;//PWM10	2
    float fuyang;//俯仰角	4
    float gunzhuan;//滚转角	4

    float touyingjuli;//当前导航点地面投影直线距离	4
    float hangdiansu;//接近导航点速度	4
    float xiangduifangwei;//当前导航点所在相对方位	4
    float hangxiangfangwe;//参考航线航向方位	4
    float xhuizhijuli;//飞机到参考航线的水平面垂直(最短)距离	4
    float fangxiangqingjiao;//垂直平面内当前导航点所在方向倾角	4
    float hangjiqingjiao;//参考航线航迹倾角	4
    float qianchuijuli;//飞机到参考航线的铅垂平面垂直距离，（飞机在参考航线上侧为正）	4
    float gaoducha;//飞机到导航点的高度差	4
    float yudingsu;//预订飞行速度	4
    
    unsigned short xuhao ;//前导航点序号	2
        
    unsigned char sum ;//校验和	1
    unsigned char tail ;//包尾	1
	
} ;

extern unsigned short parFlag;
extern int dofangzhen_1_data(struct UART_DATA *uart_data,struct FANGZHEN_1_DATA *fangzhendata);

#endif