/*
 * Copyright 2010 Andrew Leifer et al <leifer@fas.harvard.edu>
 * This file is part of MindControl.
 *
 * MindControl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU  General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MindControl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MindControl. If not, see <http://www.gnu.org/licenses/>.
 *
 * For the most up to date version of this software, see:
 * http://github.com/samuellab/mindcontrol
 *
 *
 *
 * NOTE: If you use any portion of this code in your research, kindly cite:
 * Leifer, A.M., Fang-Yen, C., Gershow, M., Alkema, M., and Samuel A. D.T.,
 * 	"Optogenetic manipulation of neural activity with high spatial resolution in
 *	freely moving Caenorhabditis elegans," Nature Methods, Submitted (2010).
 */

/*
 * Talk2BaslerCamera.c
 *
 * Created on AUG 2018
 *   by Tianqi Xu
 *   
 *
 *   Talk2BaslerCamera is a library to interface with Basler USB camera's
 *   It uses a number of supplied wrapper libraries from Pylon 5
 *
 * 
 *
 */


#include <iostream>
#include "Talk2BaslerCamera.h"

 // namespace files to use the pylon API.
using namespace Pylon;

using namespace std;

/* Initialize Pylon runtime library before operations on camera. */
void T2Cam_InitializeLib() {
	PylonInitialize();
	cout << "Pylon run time library has been loaded." << endl;
}

/* Terminate the Pylon runtime library when we are done with entire work. */
void T2Cam_CloseLib() {
	PylonTerminate();
	cout << "Pylon run time library has been closed." << endl;
}

/* Allocate Memory for CamData structure. */
void T2Cam_AllocateCamDataStructMemory(CamData** Camera)
{
	cout << "Allocate memory for CamDataStruct." << endl;
	*Camera = new CamData;
}

/* Connect to the camera with specified serial number: SERIALNUM. */
int T2Cam_CreateSpecificCamera(CamData& CamInfo) {
	try {
		CTlFactory& TlFactory = CTlFactory::GetInstance();
		CDeviceInfo di;

		// Assign the specific camera with SERIALNUM serial number.
		di.SetSerialNumber(SERIALNUM);
		di.SetDeviceClass(BaslerGigEDeviceClass);

		// Creat the device within pylon.
		IPylonDevice* device = TlFactory.CreateDevice(di);
		CamInfo.myInstCam.Attach(device);
		cout << "Load camera with serial number: " << SERIALNUM << endl;
		return CAMSUCC;

	}
	catch (const GenericException& e) {
		cerr << "Failed to find camera: " << e.GetDescription() << endl;
		return CAMERROR;
	}
}


/* Start grabbing frame. */
int T2Cam_GrabFramesAsFastAsYouCan(CamData& Camera)
{
	try
	{
		Camera.myInstCam.Open();

		// Start grabing in a GrabLoop.		
		Camera.myInstCam.StartGrabbing();
		cout << "Start grabbing image." << endl;
		return CAMSUCC;
	}
	catch (const GenericException &e) {
		cerr << "An exception occurred inside frame grabbing. code: " << e.GetDescription() << endl;
		return CAMERROR;
	}
}

/* Retrieve the Grab Result from camera to CamData structure.
 * Note: this function needs to be called every time you want a new frame
 * to do further analysis. */
int T2Cam_RetrieveAnImage(CamData & Camera)
{
	CGrabResultPtr temp_ptrGrabResult;
	Camera.myInstCam.RetrieveResult(500, temp_ptrGrabResult);
	if (temp_ptrGrabResult->GrabSucceeded())
	{
		Camera.iFrameNumber = temp_ptrGrabResult->GetBlockID();
		Camera.iImageData = (unsigned char *)temp_ptrGrabResult->GetBuffer();
		return CAMSUCC;
	}
	else
		return CAMERROR;

}

/* Turn the camera off and deallocate CamData structure and iPylon decive. */
void T2Cam_TurnOff(CamData& Camera)
{
	try
	{
		if (Camera.myInstCam.IsGrabbing())
		{
			Camera.myInstCam.StopGrabbing();
			cout << "Stop grabbing." << endl;
		}
		Camera.myInstCam.Close();

		// IPylonDevice should be destroy before close library.
		Camera.myInstCam.DestroyDevice();

		delete &Camera;
		cout << "CamDataStrut destoried." << endl;
	}
	catch (const GenericException &e)
	{
		cerr << "An exception occurred. code: " << e.GetDescription() << endl;
	}
}




