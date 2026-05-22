#include "Talk2TLDC4104.h"

bool AllocateLedDataStructMemory(LedData** TLed)
{
	*TLed = new LedData;
	if (*TLed == NULL)
	{
		std::cout << "Can't allocate memory for LedDataStruct." << std::endl;
		return LEDERR;
	}

	std::cout << "Allocate memory for LedDataStruct." << std::endl;
	return LEDSUCC;
}

void TLDC_InitLed(LedData& TLed)
{
	TLed.instr = VI_NULL;
	TLed.err = VI_SUCCESS;
	TLed.resMgr = VI_NULL;
	TLed.findList = VI_NULL;
	TLed.rscPtr = VI_NULL;
	TLed.cnt = 0;

	TLed.alias = new ViChar[DC4100_BUFFER_SIZE];
	TLed.name = new ViChar[DC4100_BUFFER_SIZE];
	TLed.rscStr = new ViChar[VI_FIND_BUFLEN];

	TLed.i = 0;
}

void TLDC_UnInitLed(LedData& TLed)
{
	delete[] TLed.alias;
	delete[] TLed.name;
	delete[] TLed.rscStr;

	// switch off all LEDs
	for (TLed.i = 0; TLed.i < NUM_CHANNELS; TLed.i++) 	TLDC4100_setLedOnOff(TLed.instr, TLed.i, 0);

	// Close device
	TLDC4100_close(TLed.instr);
}

bool TLDC_ComScan(LedData& TLed)
{
	std::cout << "Scanning for DC4100 Series instruments ..." << std::endl;
	if ((TLed.err = viOpenDefaultRM(&TLed.resMgr))) return LEDERR;
	if ((TLed.err = viFindRsrc(TLed.resMgr, DC4100_FIND_PATTERN, &TLed.findList, &TLed.cnt, TLed.rscStr))) return LEDERR;

	// Information line
	printf("Found %u port%s ...\n\n", TLed.cnt, (TLed.cnt > 1) ? "s" : "");

	// Iterate all found devices
	while (TLed.cnt)
	{
		// Read information
		if (readInstrData(TLed.resMgr, TLed.rscStr, TLed.name, TLed.alias, &TLed.lockState) == VI_SUCCESS)
		{
			// Check for the devices name
			if ((strstr(TLed.alias, "DC4100")) || (strstr(TLed.alias, "DC4104")))
			{
				// If found we store the device description pointer
				TLed.rscPtr = TLed.rscStr;
				CleanupScan(TLed.resMgr, TLed.findList, VI_SUCCESS);
				break;
			}
		}
		TLed.cnt--;
		if (TLed.cnt)
		{
			// Find next device
			if ((TLed.err = viFindNext(TLed.findList, TLed.rscStr))) return (CleanupScan(TLed.resMgr, TLed.findList, TLed.err));
		}
	}

	if (!TLed.rscPtr)
	{
		std::cout << "No DC4104 Series device found ..." << std::endl;
		return LEDERR;
	}

	return LEDSUCC;
}

bool TLDC_ComActivateAndSetOff(LedData& TLed)
{
	TLed.err = TLDC4100_init(TLed.rscStr, VI_OFF, VI_OFF, &TLed.instr);

	// Error handling
	if (TLed.err)	return LEDERR;

	// Set all LEDs off
	for (TLed.i = 0; TLed.i < NUM_CHANNELS; TLed.i++)
	{
		TLDC4100_setLedOnOff(TLed.instr, TLed.i, 0);
	}

	// Set the modus to Constant Current
	TLed.err = TLDC4100_setOperationMode(TLed.instr, MODUS_PERCENT_CURRENT);
	if (TLed.err)	return LEDERR;

	// Set Selection Mode
	TLed.err = TLDC4100_setSelectionMode(TLed.instr, MULTI_SELECT);
	if (TLed.err)	return LEDERR;

	return LEDSUCC;
}

void TLDC_SetBrightnessPerc(LedData& TLed, ViUInt32 LedNum, ViReal32 BriPerc)
{
	TLDC4100_setPercentalBrightness(TLed.instr, LedNum, BriPerc);
}

void TLDC_SetLedOnOff(LedData& TLed, ViUInt32 LedNum, ViBoolean status)
{
	TLDC4100_setLedOnOff(TLed.instr, LedNum, status);
}

void TLDC_TurnLedOnOff(LedData& TLed, ViUInt32 LedNum, int Led_status, int BriPerc)
{
	ViBoolean Led_now = 0;
	TLDC4100_getLedOnOff(TLed.instr, LedNum, &Led_now);

	//查看当前灯的状态
	if (Led_status)   //如果设定灯打开
	{
		if (Led_now)   //当前灯是开的
			TLDC_SetBrightnessPerc(TLed, LedNum, BriPerc);   //仅用设定亮度值
		else
		{
			TLDC_SetLedOnOff(TLed, LedNum, 1);   //如果灯是关的，就打开灯
		}
	}
	else
	{
		if (Led_now)
			TLDC_SetLedOnOff(TLed, LedNum, 0);
		else
		{
			TLDC_SetBrightnessPerc(TLed, LedNum, BriPerc);
		}
	}
}

/*---------------------------------------------------------------------------
  Print keypress message and wait
---------------------------------------------------------------------------*/
void waitKeypress(void)
{
	printf("Press <ENTER> to exit\n");
	while (getchar() == EOF);
}


/*---------------------------------------------------------------------------
	Read instrument data. Reads the data from the opened VISA instrument
	session.
	Return value: VISA library error code.

	Note: For Serial connections we use the VISA alias as name and the
		  'VI_ATTR_INTF_INST_NAME' as description.
---------------------------------------------------------------------------*/
static ViStatus readInstrData(ViSession resMgr, ViRsrc rscStr, ViChar* name, ViChar* descr, ViAccessMode* lockState)
{
#define DEF_VI_OPEN_TIMEOUT	1500 				// open timeout in milliseconds
#define DEF_INSTR_NAME			"Unknown"		// default device name
#define DEF_INSTR_ALIAS			""					// default alias
#define DEF_LOCK_STATE			VI_NO_LOCK		// not locked by default

	ViStatus		err;
	ViSession	instr;
	ViUInt16		intfType, intfNum;
	ViInt16     ri_state, cts_state, dcd_state;

	// Default values
	strcpy(name, DEF_INSTR_NAME);
	strcpy(descr, DEF_INSTR_ALIAS);
	*lockState = DEF_LOCK_STATE;

	// Get alias
	if ((err = viParseRsrcEx(resMgr, rscStr, &intfType, &intfNum, VI_NULL, VI_NULL, name)) < 0) return (err);
	if (intfType != VI_INTF_ASRL) return (VI_ERROR_INV_RSRC_NAME);
	// Open resource
	err = viOpen(resMgr, rscStr, VI_NULL, DEF_VI_OPEN_TIMEOUT, &instr);
	if (err == VI_ERROR_RSRC_BUSY)
	{
		// Port is open in another application
		if (strncmp(name, "LPT", 3) == 0)
		{
			// Probably we have a LPT port - do not show this
			return (VI_ERROR_INV_OBJECT);
		}
		// display port as locked
		*lockState = VI_EXCLUSIVE_LOCK;
		return (VI_SUCCESS);
	}
	if (err) return (err);
	// Get attribute data
	err = viGetAttribute(instr, VI_ATTR_RSRC_LOCK_STATE, lockState);
	if (!err) err = viGetAttribute(instr, VI_ATTR_INTF_INST_NAME, descr);
	// Get wire attributes to exclude LPT...
	if (!err) err = viGetAttribute(instr, VI_ATTR_ASRL_RI_STATE, &ri_state);
	if (!err) err = viGetAttribute(instr, VI_ATTR_ASRL_CTS_STATE, &cts_state);
	if (!err) err = viGetAttribute(instr, VI_ATTR_ASRL_DCD_STATE, &dcd_state);
	if (!err)
	{
		if ((ri_state == VI_STATE_UNKNOWN) && (cts_state == VI_STATE_UNKNOWN) && (dcd_state == VI_STATE_UNKNOWN)) err = VI_ERROR_INV_OBJECT;
	}
	// Closing
	viClose(instr);
	return (err);
}


/*---------------------------------------------------------------------------
	Cleanup scan. Frees the data structures used for scanning instruments.
	Return value: value passed via the 'err' parameter.
---------------------------------------------------------------------------*/
static ViStatus CleanupScan(ViSession resMgr, ViFindList findList, ViStatus err)
{
	if (findList != VI_NULL) viClose(findList);
	if (resMgr != VI_NULL) viClose(resMgr);
	return (err);
}

