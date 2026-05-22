/*
* Talk2MomentCamera.h
*
* Created on March 2024
* by Shuhao Yang
*
*/

#pragma once
#pragma warning(disable : 4996)

#ifndef TALK2TLDC4104_H_
#define TALK2TLDC4104_H_

#include <visa.h>
#include "TLDC4100.h"
#include <iostream>
#include <stdio.h>

#define LED0 0
#define LED1 1
#define LED2 2
#define LED3 3
#define LEDERR false
#define LEDSUCC true

typedef struct LedDataStruct {
	ViSession instr;    // instrument handle

	ViStatus err;    // error variable
	ViSession resMgr;     // resource manager
	ViFindList findList;    //find list
	ViChar* rscPtr; // pointer to resource string
	ViUInt32 cnt;  // counts found devices
	ViChar* alias;
	ViChar* name;
	ViChar* rscStr;

	ViAccessMode lockState;   // lock state of a device 
	ViUInt32 i;  // loop variable
}LedData;

bool AllocateLedDataStructMemory(LedData** TLed);

void TLDC_InitLed(LedData& TLed);

void TLDC_UnInitLed(LedData& TLed);

bool TLDC_ComScan(LedData& TLed);

bool TLDC_ComActivateAndSetOff(LedData& TLed);

void TLDC_SetBrightnessPerc(LedData& TLed, ViUInt32 LedNum, ViReal32 BriPerc);

void TLDC_SetLedOnOff(LedData& TLed, ViUInt32 LedNum, ViBoolean status);

void TLDC_TurnLedOnOff(LedData& TLed, ViUInt32 LedNum, int Led_status, int BriPerc);

static ViStatus readInstrData(ViSession resMgr, ViRsrc rscStr, ViChar* name, ViChar* descr, ViAccessMode* lockState);

static ViStatus CleanupScan(ViSession resMgr, ViFindList findList, ViStatus err);

void waitKeypress(void);

#endif
