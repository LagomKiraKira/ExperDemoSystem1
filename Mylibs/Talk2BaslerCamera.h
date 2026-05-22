/*
* Talk2BaslerCamera.h
*
* Created on AUG 2018
* by Tianqi Xu
*
*
*   Talk2BaslerCamera is a library to interface with Basler USB camera's
*   It uses a number of supplied wrapper libraries from Pylon 5
*
*
*
*/

#pragma once

//#include <pylon/PylonIncludes.h>
#include "Common.h"

#define SERIALNUM "22491964"
#define CAMERROR -1;
#define CAMSUCC 0;

typedef struct CamDataStruct {
	CameraContext* myInstCam;
	unsigned char *iImageData;
	uint64_t  iFrameNumber;
}CamData;

/* Initialize Pylon runtime library before operations on camera. */
void T2Cam_InitializeLib();

/* Terminate the Pylon runtime library when we are done with entire work. */
void T2Cam_CloseLib();

/* Allocate Memory for CamData structure. */
void T2Cam_AllocateCamDataStructMemory(CamData**);

/* Connect to the camera with specified serial number: SERIALNUM. */
int T2Cam_CreateSpecificCamera(CamData& CamInfo);

/* Start grabbing frame. */
int T2Cam_GrabFramesAsFastAsYouCan(CamData& Camera);

/* Retrieve the Grab Result from camera to CamData structure. 
 * Note: this function needs to be called every time you want a new frame
 * to do further analysis. */
int T2Cam_RetrieveAnImage(CamData& Camera);

/* Turn the camera off and deallocate CamData structure and iPylon decive. */
void T2Cam_TurnOff(CamData& Camera);
