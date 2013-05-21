//
//  util.c
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


#include "util.h"


void buf2float2(float *tfloat, char *buf)
{
	int i;
	char * p1 = (char *)tfloat;
	char * p3 = buf+3;
	for(i = 0; i < 4; i++)
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
	for(i=0; i < 4;i++)
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
	for(i = 0;i < 4;i++)
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
	for(i = 0; i < 4; i++)
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
	for(i=0; i < 4; i++)
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
	for(i=0; i < 4; i++)
	{
		*p3 = *p1;
		p1++;
		p3++;
	}
}


void wait(unsigned int nTick)
{
	unsigned int nloop = 0;
	while(nloop++ < nTick)	;
}