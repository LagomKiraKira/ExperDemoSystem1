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
 * NeuronAnalysis.h
 *
 *  Created on: Oct 12, 2009
 *      Author: andy
 *
 *      This library contains functions that are specific to analyzing Neurons.
 *
 *      Functions in this library depend on:
 *      	AndysOpenCVLib.h
 *      	AndysComputations.h
 */

#ifndef NEURONANALYSIS_H_
#define NEURONALYSIS_H_

#include "def_common.h"

//#ifndef ANDYSOPENCVLIB_H_
// #error "#include AndysOpenCVLib.h" must appear in source files before "#include NeuronAnalysis.h"
//#endif

typedef struct NeuronAnalysisParamStruct{
	/** Turn Analysis On Generally **/
	int OnOff;
	int Record;

	/** Single Frame Analysis Parameters**/
	int BinThresh;
	int LengthScale; //前向向量或后向向量端点间隔点的个数
	int GaussSize;
	int BoundSmoothSize;
	int DilateErode;
	int NumSegments;
	int DilatePm;
	int ErodePm;
	
	/** Frame to Frame Temporal Analysis**/
	int TemporalOn;
	int InduceHeadTailFlip;
	int MaxLocationChange;

	/** Mask Diameter for Neuron Tracking **/
	int MaskStyle;
	int MaskDiameter;
	int LowBoundary;
	int RectHeigh;

	/** Display Stuff**/
	int DispRate; //Deprecated
	int Display;

	/** Stage Control Parameters **/
	int stageTrackingOn;
	int stageSpeedFactor;
	double maxstagespeed;
	int stageTargetSegment; //segment along the worms centerline used for targeting
	int AxisLockState;

	/** Led Control Parameters **/
	int BrightnessPerc;
	int Led0_status;
	int Led1_status;

	/** Electrical Field Control**/
	int EF_Mode;
	double standardCorX;   //相对于坐标点的标准坐标
	double standardCorY;
	int Voltage;  //记录电场的电压
	double cur_velocity;   //线虫的速度  单位um/s
	WaveType curType;   //当前电压的波形
	char vChannel;
	char vDir;
	char vNum;
	char vWave;
} NeuronAnalysisParam;


/** These are computed and segmented information about the worm at the current frame**/
typedef struct SegmentedWormStruct {
	CvSeq* Centerline;
	CvSeq* LeftBound;
	CvSeq* RightBound;
	CvPoint* Head;
	CvPoint* Tail;
	CvMemStorage* MemSegStorage;
	int NumSegments;
	CvPoint* centerOfWorm;
	CvPoint* PtOnWorm;
} SegmentedWorm;

typedef struct NeuronImageAnalysisStruct{
	CvSize SizeOfImage;

	/** Frame Info **/
	int frameNum;
	int frameNumCamInternal;

	/** Images **/
	IplImage* ImgOrig;
	IplImage* ImgThresh;
	IplImage* ImgSmooth;
	
	/** mask for ROI selection **/
	IplImage* Mask;

	/** Memory **/
	CvMemStorage* MemStorage;
	CvMemStorage* MemScratchStorage; 
	CvMemStorage* MemCenterLine;

	/** Features **/
	CvSeq* Boundary;
	CvPoint* Head;
	CvPoint* Tail;
	CvPoint* prevHead;
	CvPoint* prevTail;
	int TailIndex;
	int HeadIndex;
	CvSeq* Centerline;

	/** Segmented Worm **/
	SegmentedWorm* Segmented;      

	/** Neuron Features **/
	CvPoint* centerOfNeuron;
	CvMoments* mom;


	/** TimeStamp **/
	unsigned long timestamp;


	/** Information about location on plate **/
	CvPoint stageVelocity; //compensating velocity of stage.
	CvPoint prestageVelocity;
	double stagePosition_x; //Position of the motorized stage.
	double stagePosition_y;

}NeuronAnalysisData;





/*
 * Struct to hold basic geometric information about
 * the location of the previous worm
 *
 * This can be used in combination with the
 * Parameters->PersistantOn=1 to do frame-to-frame
 * intelligent temporal error checking
 *
 *
 */
typedef struct WormGeomStruct {
	CvPoint Head;
	CvPoint Tail;
	int Perimeter;
}WormGeom;

/*
 *
 * Every function here should have the word Neuron in it
 * because every function here is Neuron specific
 */



/************************************************************/
/* Creating, Destroying and Memory for 						*/
/*  NeuronAnalysisDataStruct 									*/
/*															*/
/************************************************************/


/*
 *  Create the NeuronAnalysisDataStruct
 *  Initialize Memory Storage
 *  Set all Pointers to Null.
 *  Run CvCreate Sequence
 *
 *  Note this does not allocate memory for images because the user may not know
 *  what size image is wanted yet.
 *
 *  To do that use LoadNeuronColorOriginal()
 *
 */
NeuronAnalysisData* CreateNeuronAnalysisDataStruct();




/*
 *
 * Clear's all the Memory and De-Allocates it
 */
void DestroyNeuronAnalysisDataStruct(NeuronAnalysisData* NeuronPtr);


 /*
 * Create dynamic memory storage for the worm
 *
 */
void InitializeNeuronMemStorage(NeuronAnalysisData* Neuron);

/*
 * Refersh dynamic memory storage for the worm
 * (clear the memory without freing it)
 *
 */
int RefreshNeuronMemStorage(NeuronAnalysisData* Neuron);


/*
 * Create Blank Images for NeuronAnalysisData given the image size.
 *
 */
void InitializeEmptyNeuronImages(NeuronAnalysisData* Neuron, CvSize ImageSize);

/*
 * This function is run after IntializeEmptyImages.
 * And it loads a color original into the WoirmAnalysisData structure.
 * The color image is converted to an 8 bit grayscale.
 *
 *
 */
void LoadNeuronColorOriginal(NeuronAnalysisData* Neuron, IplImage* ImgColorOrig);


/*
 * This function is run after IntializeEmptyImages.
 * And it loads a properly formated 8 bit grayscale image
 * into the NeuronAnalysisData structure.
 */
int LoadNeuronImg(NeuronAnalysisData* Neuron, NeuronAnalysisParam* Params, IplImage* Img, CvPoint stageFeedbackTargetOffset);



/************************************************************/
/* Creating, Destroying NeuronAnalysisParam					*/
/*  					 									*/
/*															*/
/************************************************************/

/*
 *  Allocate memory for a NeuronAnalysisParam struct
 */
NeuronAnalysisParam* CreateNeuronAnalysisParam();

void DestroyNeuronAnalysisParam(NeuronAnalysisParam* ParamPtr);




/************************************************************/
/* Higher Level Routines									*/
/*  					 									*/
/*															*/
/************************************************************/


/*
 * thresholds and finds the center of the neuron.
 * The original image must already be loaded into Neuron.ImgOrig
 * The thresholded image is deposited into Neuron.ImgThresh
 *
 */
int FindNeuronCenter(NeuronAnalysisData* Neuron, NeuronAnalysisParam* NeuronParams);


/*
 * Generates the original image and the center of the neuron
 *
 */
void DisplayNeuronCenter(NeuronAnalysisData* Neuron,const char* WinDisp);


/*
 * Generates the original image of the worm
 * with segmentation
 * And also the head and tail.
 */
void DisplayWormSegmentation(NeuronAnalysisData* Worm, const char* WinDisp);

/**
 *
 * Creates the Worm heads up display for monitoring or for saving to disk
 * You must first pass a pointer to an IplImage that has already been allocated and
 * has dimensions of Worm->SizeOfImage
 *
 *
 */
int CreateWormHUDS(IplImage* TempImage, NeuronAnalysisData* Neuron, NeuronAnalysisParam* Params);

/*****************************************
 *
 * Monitoring functions
 *
 */

 /*
  * Displays the original image of the worm
  * highlighting the head and tail in the window WindowName
  *
  */
void DisplayWormHeadTail(NeuronAnalysisData* Worm, char* WindowName);

/*
 * This function overlays the illumination frame translucently
 * over the original image.
 * It also draws the worm's boundary and the worm's head and tail.
 *
 */
void DisplayWormHUDS(NeuronAnalysisData* Worm, NeuronAnalysisParam* Params, char* WindowName);


/*
 * Smooths, thresholds and finds the worms contour.
 * The original image must already be loaded into Worm.ImgOrig
 * The Smoothed image is deposited into Worm.ImgSmooth
 * The thresholded image is deposited into Worm.ImgThresh
 * The Boundary is placed in Worm.Boundary
 *
 */
void FindWormBoundary(NeuronAnalysisData* Neuron, NeuronAnalysisParam* Params);

/*
 * Finds the Worm's Head and Tail.
 * Requires Worm->Boundary
 *
 */
int GivenBoundaryFindWormHeadTail(NeuronAnalysisData* Neuron, NeuronAnalysisParam* Params);

/*
 * This function reverses the head and the tail of a worm.
 *
 * Note: it does not reverse the sequences that describe the worm's boundary
 * or its segmentation.
 *
 */
int ReverseWormHeadTail(NeuronAnalysisData* Neuron);


/*****
 * Function related to the simpler Worm Geometery OBject
 */

 /* Create a Worm Geometry Object
  *
  */
WormGeom* CreateWormGeom();

/*
 * Set the values inside the Worm Geometry object to NULL
 *
 */
void ClearWormGeom(WormGeom* SimpleWorm);

/*
 * Frees the memory allocated to the Worm Geometry object
 * and sets its pointer to NULL
 */
void DestroyWormGeom(WormGeom** SimpleWorm);

/*
 *Populates LoadWormGeom with geometry data from Worm Object Worm
 */
void LoadWormGeom(WormGeom* SimpleWorm, NeuronAnalysisData* Worm);

/***********************
 * Temporal Analysis Tools
 *
 */

 /*
  *
  * Returns 1 if the worm is consistent with previous frame.
  * Returns 0 if the worm's head and tail had been reversed from
  *      	  previous frame and fixes the problem.
  * Returns -1 if the head and the tail do not match the previous frame at all
  * Returns 2 if there is no previous worm information
  */
int PrevFrameImproveWormHeadTail(NeuronAnalysisData* Worm, NeuronAnalysisParam* Params, WormGeom* PrevWorm);


/*
 * This Function segments a worm.
 * It requires that certain information be present in the WormAnalysisData struct Worm
 * It requires Worm->Boundary be full
 * It requires that Params->NumSegments be greater than zero
 *
 */
int SegmentWorm(NeuronAnalysisData* Worm, NeuronAnalysisParam* Params);


/************************************************************/
/* Creating, Destroying SegmentedWormStruct					*/
/*  					 									*/
/*															*/
/************************************************************/

/*
 * Creates a Segmented Worm Struct
 * Creates memory for the associated worm struct
 * and initializes the centerline and L&R boundaries
 * and sets everything else to null
 */
SegmentedWorm* CreateSegmentedWormStruct();

void DestroySegmentedWormStruct(SegmentedWorm* SegWorm);

/** Clear a SegmentedWorm struct but not dallocate memory. **/
void ClearSegmentedInfo(SegmentedWorm* SegWorm);



#endif /* NeuronANALYSIS_H_ */


//#endif /* NeuronANALYSIS_H_ */
