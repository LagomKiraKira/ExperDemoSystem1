#pragma once
#include <string>

#define EXP_ERROR    -1
#define EXP_SUCCESS  0
#define NSIZEX       1600     
#define NSIZEY       1100

/*
* 电场刺激电压的类型
*/
enum WaveType
{
	None,          //无电压
	Square,        //方波电压
	Sine,          //正弦电压
	DC             //直流电压
};