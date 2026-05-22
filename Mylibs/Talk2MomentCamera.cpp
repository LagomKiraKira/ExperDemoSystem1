#include "Talk2MomentCamera.h"

using namespace std;

// Flag used to break the periodic timer thread
bool g_periodicTimerActive = false;
// Timer thread issuing the software trigger
std::thread g_periodicTimerThread;

/* Check Pvcam Information before operations on Camera */
bool T2Cam_ShowPvcamInfo(int argc, char* argv[])
{
	return ShowAppInfo(argc, argv);
}

/* Allocate Memory for CamData structure. */
bool T2Cam_AllocateCamDataStructMemory(CamData** Camera)
{
	*Camera = new CamData;
	if (*Camera == NULL)
	{
		cout << "Can't allocate memory for CamDataStruct." << endl;
		return CAMERROR;
	}

	cout << "Allocate memory for CamDataStruct." << endl;
	return CAMSUCC;
}

/* Connect to the camera and open Specific Camera*/
bool T2Cam_InitAndOpenSpecificCamera(CamData& Camera)
{
	//vector<CameraContext*> contexts;
	if (!InitAndOpenOneCamera(Camera.contexts, cSingleCamIndex))
	{
		cout << "Can't init and open Specific Camera. Error Coed:1" << endl;
		return CAMERROR;
	}

	Camera.ctx = new CameraContext;
	Camera.ctx = Camera.contexts[cSingleCamIndex];
	//cout << Camera.ctx << endl;
	
	//相机参数设置
	Camera.exposureBytes = 0;
	Camera.exposureTime = 10;
	Camera.circBufferFrames = 20;
	Camera.bufferMode = CIRC_OVERWRITE;
	Camera.iFrameNumber = 0;
	Camera.minValue = 0;
	Camera.maxValue = 0;
	Camera.errorOccurred = FALSE;
	Camera.resetMaxAndMin = FALSE;

	// Install the generic ctrl+c handler
	if (!InstallGenericCliTerminationHandler(Camera.contexts))
	{
		CloseAllCamerasAndUninit(Camera.contexts);
		return CAMERROR;
	}

	if (Camera.ctx == NULL)
	{
		cout << "Can't init and open Specific Camera. Error Coed:2" << endl;
		return CAMERROR;
	}

	cout << "Inited and Opened Specific Camera" << endl;
	return CAMSUCC;
}

/**
Callback function, called automatically by PVCAM every time an EOF event arrives
(every time a frame is written to the user buffer).相机采集回调函数，需要开启另外一个线程一直执行
*/
void PV_DECL CustomEofHandler(FRAME_INFO* pFrameInfo, void* pContext)
{
    if (!pFrameInfo || !pContext)
        return;
    auto ctx = static_cast<CameraContext*>(pContext);

    /**
    Because the callback handler is invoked by PVCAM from a different thread than the
    main() thread, printing to standard output should be properly synchronized. Otherwise
    the text printed from multiple threads could be interleaved and output formatting
    could become broken. For simplicity, this code sample relies on relatively low
    camera frame rate and basic synchronization with the eofEvent.
    */
	//删除了计数器

    // Store the frame information for later use on the main thread
    ctx->eofFrameInfo = *pFrameInfo;

    // Obtain a pointer to the last acquired frame
    if (PV_OK != pl_exp_get_latest_frame(ctx->hcam, &ctx->eofFrame))
    {
        PrintErrorMessage(pl_error_code(), "pl_exp_get_latest_frame() error");
        ctx->eofFrame = nullptr;
    }

    // Unblock the acquisition thread
    {
        std::lock_guard<std::mutex> lock(ctx->eofEvent.mutex);
        ctx->eofEvent.flag = true;
    }
    ctx->eofEvent.cond.notify_all();
}

/**
This function starts a timer thread that calls pl_exp_trigger() periodically.
Please note that this is a simple implementation of a periodic timer, using
a thread 'sleep' approach. This timer may develop a slack over time. For more
accurate 'time-lapse' triggering in exact time intervals, a more accurate, high
precision timer based on real-time clock would be recommended.
*/
void PeriodicTimerStart(CameraContext* ctx, unsigned int intervalMs)
{
	g_periodicTimerActive = true;
	g_periodicTimerThread = std::thread([ctx, intervalMs]()
		{
			printf("Periodic timer started with interval of %ums\n", intervalMs);
			unsigned int triggerCount = 0;
			auto nextTrigTime = std::chrono::steady_clock::now();
			while (g_periodicTimerActive)
			{
				nextTrigTime += std::chrono::milliseconds(intervalMs);

				// Trigger the camera
				uns32 flags = 0;
				if (PV_OK != pl_exp_trigger(ctx->hcam, &flags, 0))
				{
					PrintErrorMessage(pl_error_code(), "SW Trigger failed, pl_exp_trigger() error");
					// Break the while loop
					break;
				}
				++triggerCount;

				// This thread triggers the camera independently without waiting for a confirmation
				// on the frame delivery (EOF callback). This simulates a 'time-lapse' triggering.
				// If the thread triggers too fast, the camera will respond with an 'ignored' flag.
				switch (flags)
				{
				case PL_SW_TRIG_STATUS_IGNORED:
					printf("SW Trigger #%u ignored (triggering too fast?)\n", triggerCount);
					break;
				case PL_SW_TRIG_STATUS_TRIGGERED:
					//printf("SW Trigger #%u accepted\n", triggerCount);
					break;
				default:
					printf("SW Trigger #%u: Unknown status\n", triggerCount);
					break;
				}
				std::this_thread::sleep_until(nextTrigTime);
			}
			printf("Periodic timer stopped\n");
		});
}

/**
Stops the timer thread that has been started by PeriodicTimerStart() call.
*/
void PeriodicTimerStop()
{
	g_periodicTimerActive = false;
	g_periodicTimerThread.join();
}

/* Start grabbing frame. */
bool T2Cam_GrabFramesAsFastAsYouCan(CamData& Camera)
{
	CameraContext* ctx = Camera.ctx;

	//预先读取相机拍摄的ExposeOut模式
	const int16 selectedExposureMode = EXT_TRIG_SOFTWARE_EDGE;

	// Retrieve a list of camera-supported expose out modes.
	// These modes control the behavior of the expose-out hardware signal. We will
	// simply pick the first available one.
	NVPC supportedExposeOutModes;
	if (!ReadEnumeration(ctx->hcam, &supportedExposeOutModes, PARAM_EXPOSE_OUT_MODE,
		"PARAM_EXPOSE_OUT_MODE"))
	{
		printf("Unable to read PARAM_EXPOSE_OUT_MODE parameter\n");
		CloseAllCamerasAndUninit(Camera.contexts);
		return APP_EXIT_ERROR;
	}
	const int16 selectedExposeOutMode = static_cast<int16>(supportedExposeOutModes[0].value);

	/**
	Register a callback handler to receive a notification when EOF event arrives. Similar
	handler is implemented in the common code that is used across the SDK examples.
	However, additionally to the usual CameraContext, this specific example also
	uses two custom counters initialized above.
	*/
	if (PV_OK != pl_cam_register_callback_ex3(ctx->hcam, PL_CALLBACK_EOF,
		(void*)GenericEofHandler, ctx))
	{
		PrintErrorMessage(pl_error_code(), "pl_cam_register_callback() error");
		CloseAllCamerasAndUninit(Camera.contexts);
		return CAMERROR;
	}
	printf("EOF callback handler registered on camera %d\n", ctx->hcam);

	//设置像素融合因子
	ctx->region.sbin = 2;
	ctx->region.pbin = 2;

	// Select the appropriate, internal-trigger mode for this camera.
	const int16 expMode = selectedExposureMode | selectedExposeOutMode;
	//if (!SelectCameraExpMode(ctx, expMode, TIMED_MODE, EXT_TRIG_INTERNAL))
	//{
	//	CloseAllCamerasAndUninit(Camera.contexts);
	//	return CAMERROR;
	//}

	/**
   Prepare the continuous acquisition with circular buffer mode. The
   pl_exp_setup_cont() function returns the size of one frame (unlike
   the pl_exp_setup_seq() which returns a buffer size for the entire sequence).
   */
	if (PV_OK != pl_exp_setup_cont(ctx->hcam, 1, &ctx->region, expMode,
		Camera.exposureTime, &Camera.exposureBytes, Camera.bufferMode))
	{
		PrintErrorMessage(pl_error_code(), "pl_exp_setup_cont() error");
		CloseAllCamerasAndUninit(Camera.contexts);
		return APP_EXIT_ERROR;
	}
	printf("Acquisition setup successful on camera %d\n", ctx->hcam);
	UpdateCtxImageFormat(ctx);

	//初始化Camera结构里的图像
	Camera.iImageData = (unsigned char*)malloc(WIDTH * HEIGHT * sizeof(unsigned char));

	Camera.circBufferBytes = Camera.circBufferFrames * Camera.exposureBytes;
	/**
	Now allocate the buffer memory. The application is in control of the circular buffer
	and should allocate memory of appropriate size.
	*/
	Camera.circBufferInMemory = new (std::nothrow) uns8[Camera.circBufferBytes];
	cout << "circBufferBytes:" << Camera.circBufferBytes << endl;

	/**
	Start the continuous acquisition. By passing the entire size of the buffer
	to pl_exp_start_cont(), PVCAM can calculate the capacity of the circular buffer.
	*/
	if (PV_OK != pl_exp_start_cont(ctx->hcam, Camera.circBufferInMemory, Camera.circBufferBytes))
	{
		PrintErrorMessage(pl_error_code(), "pl_exp_start_cont() error");
		CloseAllCamerasAndUninit(Camera.contexts);
		delete[] Camera.circBufferInMemory;
		return APP_EXIT_ERROR;
	}
	printf("Acquisition started on camera %d\n", ctx->hcam);

	PeriodicTimerStart(ctx, 32);

    return CAMSUCC;
}

void FindMinAndMaxValueInFirstFrame(const unsigned short* srcImage, CamData& Camera, int width, int height)
{
	unsigned short minValue = srcImage[0];
	unsigned short maxValue = srcImage[0];

	// Find the minimum and maximum values in the 16-bit image
	for (int i = 0; i < width * height; i++) {
		if (srcImage[i] < minValue)
			minValue = srcImage[i];
		if (srcImage[i] > maxValue)
			maxValue = srcImage[i];
	}

	Camera.minValue = minValue;
	Camera.maxValue = maxValue;
}

//将图像从16bit映射到8bit
void convert16to8(const unsigned short* srcImage, CamData& Camera, int width, int height) {
	
	// Calculate the scaling factor
	double scale = 255.0 / (Camera.maxValue - Camera.minValue);

	// Convert 16-bit to 8-bit
	for (int i = 0; i < width * height; i++) {
		Camera.iImageData[i] = (unsigned char)((srcImage[i] - Camera.minValue) * scale);
	}
}


/* Retrieve the Grab Result from camera to CamData structure.
 * Note: this function needs to be called every time you want a new frame
 * to do further analysis. */
bool T2Cam_RetrieveAnImage(CamData& Camera)
{
	//需要补充抓取图像代码
	//printf("Waiting for EOF event to occur on camera %d\n", Camera.ctx->hcam);
	if (!WaitForEofEvent(Camera.ctx, 5000, Camera.errorOccurred))
		return CAMERROR;

	// Timestamp is in hundreds of microseconds
	//printf("Frame #%d acquired, timestamp = %lldus\n",
	//	Camera.ctx->eofFrameInfo.FrameNr, 100 * Camera.ctx->eofFrameInfo.TimeStamp);
	//ShowImage(Camera.ctx, Camera.ctx->eofFrame, Camera.exposureBytes);

	//需要将eofFrame存入Camera.iImageData，需要先将图片从12bit转化为8bit
	const uns16* pBuffer = reinterpret_cast<const uns16*>(Camera.ctx->eofFrame);    //先将eofFrame指针转化为2个字节的指针
	
	if (Camera.iFrameNumber == 0 || Camera.resetMaxAndMin == TRUE) {
		FindMinAndMaxValueInFirstFrame(pBuffer, Camera, WIDTH, HEIGHT);
		Camera.resetMaxAndMin = FALSE;
	}
		//FindMinAndMaxValueInFirstFrame(pBuffer, Camera, 3200, 2200);
	
	convert16to8(pBuffer, Camera, WIDTH, HEIGHT);

	//计算帧数
	Camera.iFrameNumber++;

	//cout << "iFrameNumber:" << Camera.iFrameNumber << endl;

	return CAMSUCC;
}

/* Turn the camera off and deallocate CamData structure and iPylon decive. */
void T2Cam_TurnOff(CamData& Camera)
{
	PeriodicTimerStop();

	/**
	Once we have acquired the desired number of frames, the continuous acquisition
	should be stopped with the pl_exp_abort() function. The abort is also called when
	the user interrupts the acquisition or when an error occurs during the acquisition.
	*/
	if (PV_OK != pl_exp_abort(Camera.ctx->hcam, CCS_HALT))
	{
		PrintErrorMessage(pl_error_code(), "pl_exp_abort() error");
	}
	else
	{
		printf("Acquisition stopped on camera %d\n", Camera.ctx->hcam);
	}

	delete[] Camera.circBufferInMemory;

	CloseAllCamerasAndUninit(Camera.contexts);
}