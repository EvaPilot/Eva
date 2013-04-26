
#ifndef FANZHENDATA_h
#define FANZHENDATA_h

#include "uartComm.h"

struct FANGZHEN_1_DATA {

    unsigned short head;//��ͷ	2
    unsigned char len ;//����	1
    unsigned char type;//������	1
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
    float weidu;//γ��	4
        float jingdu;//����	4
    float gaodu;//���θ߶�	4
    float disu_bei;//����-��	4
    float disu_dong;//����-��	4
    float disu_xia;//����-��	4
    float zhoujiaosu_qian;//������ٶ�-��ǰ	4
    float zhoujiaosu_you;//������ٶ�-����	4
    float zhoujiaosu_xia;//������ٶ�-����	4
    float zhouxiansu_qian;//�������ٶ�-��ǰ	4
    float zhouxiansu_you;//�������ٶ�-����	4
    float zhouxiansu_xia;//�������ٶ�-����	4
    unsigned short jingya ;//��ѹ	2
    unsigned short dongya ;//��ѹ	2
    float yingjiao ;//ӭ��	4
    float cehuajiao ;//�໬��	4
    
    float Undefined1;	// 4
    float Undefined2;	// 4
    unsigned short Undefined3;	// 2

	
    unsigned char sum ;//У���	1
    unsigned char tail ;//��β	1

	
} ;
struct FANGZHEN_2_DATA {
    unsigned short head;//��ͷ	1
    unsigned char len ;//����	2
    unsigned char type;//������	1
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
    float fuyang;//������	4
    float gunzhuan;//��ת��	4

    float touyingjuli;//��ǰ���������ͶӰֱ�߾���	4
    float hangdiansu;//�ӽ��������ٶ�	4
    float xiangduifangwei;//��ǰ������������Է�λ	4
    float hangxiangfangwe;//�ο����ߺ���λ	4
    float xhuizhijuli;//�ɻ����ο����ߵ�ˮƽ�洹ֱ(���)����	4
    float fangxiangqingjiao;//��ֱƽ���ڵ�ǰ���������ڷ������	4
    float hangjiqingjiao;//�ο����ߺ������	4
    float qianchuijuli;//�ɻ����ο����ߵ�Ǧ��ƽ�洹ֱ���룬���ɻ��ڲο������ϲ�Ϊ����	4
    float gaoducha;//�ɻ���������ĸ߶Ȳ�	4
    float yudingsu;//Ԥ�������ٶ�	4
    
    unsigned short xuhao ;//ǰ���������	2
        
    unsigned char sum ;//У���	1
    unsigned char tail ;//��β	1
	
} ;

extern unsigned short parFlag;
extern int dofangzhen_1_data(struct UART_DATA *uart_data,struct FANGZHEN_1_DATA *fangzhendata);

#endif