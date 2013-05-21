//
//  main.h
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

#ifndef main_h
#define main_h

typedef struct AhrsData
{
	float ina_acc1;	
	float ina_acc2;	
	float ina_acc3;	
	float ina_gyro1;	
	float ina_gyro2;
	float ina_gyro3;
	float gps_latti;
	float gps_longi;
	float gps_high;
	float gps_velN;
	float gps_velE;
	float gps_velD;
	float gps_angle;
	unsigned char gps_satelnum;
	float gps_pdop;	
	
	unsigned char gps_valid;

	float true_roll;
	float true_pitch;
	float true_yaw;
	float true_latti;
	float true_longi;
	
	int   waittime;	
	int   starttime;

	int algtime;	
	
	float true_velN;
	float true_velE;
	float true_velD;
	float true_high;
	
} AhrsData;



#define isSimpleCmd(cmd) (pwifi_buffer[wifi_index + 5] == (cmd)) && (pwifi_buffer[wifi_index + 6] == (cmd)) && (pwifi_buffer[wifi_index + 7] == (cmd))


typedef unsigned char bool_t;
#define YES 1
#define NO  0

#endif