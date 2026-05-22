/*
 * NeuronAnalysis.cpp
 *
 *  Created on: Sep 5, 2013
 *      Author: quan
 */
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <time.h>
#include <iostream>


 //OpenCV Headers
#include <opencv\cxcore.h>
#include <opencv\highgui.h>
#include <opencv\cv.h>

//Timer Lib
#include "../3rdPartyLibs/tictoc.h"
#include "AndysOpenCVLib.h"
#include "AndysComputations.h"

// Andy's Libraries
#include "NeuronAnalysis.h"

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
NeuronAnalysisData* CreateNeuronAnalysisDataStruct() {
	NeuronAnalysisData* NeuronPtr;
	NeuronPtr = (NeuronAnalysisData*)malloc(sizeof(NeuronAnalysisData));

	NeuronPtr->SizeOfImage.height = 0;
	NeuronPtr->SizeOfImage.width = 0;

	/*** Set Everything To NULL ***/
	/** Images **/
	NeuronPtr->ImgOrig = NULL;
	NeuronPtr->ImgThresh = NULL;
	NeuronPtr->ImgSmooth = NULL;

	/** mask for ROI selection **/
	NeuronPtr->Mask = NULL;

	/** Memory **/
	NeuronPtr->frameNum = 0;
	NeuronPtr->frameNumCamInternal = 0;

	/** Features **/
	NeuronPtr->Boundary = NULL;
	NeuronPtr->Head = NULL;
	NeuronPtr->Tail = NULL;
	NeuronPtr->prevHead = NULL;
	NeuronPtr->prevTail = NULL;
	NeuronPtr->TailIndex = 0;
	NeuronPtr->HeadIndex = 0;
	NeuronPtr->Centerline = NULL;

	/** Segmented Worm **/
	NeuronPtr->Segmented = NULL;
	NeuronPtr->Segmented = CreateSegmentedWormStruct();

	/** TimeStamp **/
	NeuronPtr->timestamp = 0;

	/** Position of the neuron on the image **/
	NeuronPtr->centerOfNeuron = NULL;
	NeuronPtr->mom = NULL;

	/** Position on plate information **/
	NeuronPtr->stageVelocity = cvPoint(0, 0);
	NeuronPtr->stagePosition_x = 0;
	NeuronPtr->stagePosition_y = 0;

	return NeuronPtr;
}

/*
 *  Allocate memory for a NeuronAnalysisParam struct
 *  And set default values for the parameters.
 */
NeuronAnalysisParam* CreateNeuronAnalysisParam() {
	NeuronAnalysisParam* ParamPtr;
	ParamPtr = (NeuronAnalysisParam*)malloc(sizeof(NeuronAnalysisParam));

	/** Single Frame Analysis Parameters**/
	ParamPtr->BinThresh = 19;
	ParamPtr->LengthScale = 5;   //需要根据实际点的个数实验确定
	ParamPtr->GaussSize = 1;
	ParamPtr->BoundSmoothSize = 20;
	ParamPtr->DilateErode = 1;
	ParamPtr->DilatePm = 0;
	ParamPtr->ErodePm = 0;
	ParamPtr->NumSegments = 100;   //边界采样点个数

	/** Frame to Frame Temporal Analysis**/
	ParamPtr->TemporalOn = 0;
	ParamPtr->InduceHeadTailFlip = 0;

	/** Turn the System On or Off **/
	ParamPtr->OnOff = 0;
	ParamPtr->Record = 0;

	/** Mask Diameter **/
	ParamPtr->MaskStyle = 0;   //ROI风格
	ParamPtr->MaskDiameter = 1100;
	ParamPtr->LowBoundary = 1;  //后续该处调整ROI的数值可能需要根据原图像的大小进行修改
	ParamPtr->RectHeigh = 500;  //还需要保障的一点就是左边界需要永远小于右边界，否则Mask将被全覆盖

	/** Display Parameters **/
	ParamPtr->DispRate = 1;
	ParamPtr->Display = 0;

	/** Stage Control Parameters **/
	ParamPtr->stageTrackingOn = 0;
	ParamPtr->stageSpeedFactor = 50;
	ParamPtr->maxstagespeed = 3;
	ParamPtr->stageTargetSegment = 50;
	ParamPtr->AxisLockState = 0;

	/** Led Control Parameters **/
	ParamPtr->BrightnessPerc = 0;
	ParamPtr->Led0_status = 0;
	ParamPtr->Led1_status = 0;

	ParamPtr->EF_Mode = 0;
	ParamPtr->standardCorX = 0;
	ParamPtr->standardCorY = 0;
	ParamPtr->Voltage = 0;
	ParamPtr->cur_velocity = 0;
	ParamPtr->curType = WaveType::None;
	ParamPtr->vChannel = '0';
	ParamPtr->vDir = '0';
	ParamPtr->vNum = '0';
	ParamPtr->vWave = '0';

	return ParamPtr;
}

/*
 * Create Blank Images for NeuronAnalysisData
 *
 */
void InitializeEmptyNeuronImages(NeuronAnalysisData* Neuron, CvSize ImageSize) {
	Neuron->SizeOfImage = ImageSize;
	Neuron->ImgOrig = cvCreateImage(ImageSize, IPL_DEPTH_8U, 1);
	Neuron->ImgThresh = cvCreateImage(ImageSize, IPL_DEPTH_8U, 1);
	Neuron->ImgSmooth = cvCreateImage(ImageSize, IPL_DEPTH_8U, 1);
	Neuron->Mask = cvCreateImage(ImageSize, IPL_DEPTH_8U, 1);

	/** Clear the Time Stamp **/   
	Neuron->timestamp = 0;

}

/*
 * Create dynamic memory storage for the worm
 *
 */
void InitializeNeuronMemStorage(NeuronAnalysisData* Neuron) {
	Neuron->MemScratchStorage = cvCreateMemStorage(0);
	Neuron->MemStorage = cvCreateMemStorage(0);
	Neuron->MemCenterLine = cvCreateMemStorage(0);
}

/*
 * Creates a Segmented Worm Struct
 * Creates memory for the associated worm struct
 * and initializes the centerline and L&R boundaries
 * and sets everything else to null
 */
SegmentedWorm* CreateSegmentedWormStruct() {
	/** Create a new instance of SegWorm **/
	SegmentedWorm* SegWorm;
	SegWorm = (SegmentedWorm*)malloc(sizeof(SegmentedWorm));

	SegWorm->Head = (CvPoint*)malloc(sizeof(CvPoint));
	SegWorm->Tail = (CvPoint*)malloc(sizeof(CvPoint));

	SegWorm->centerOfWorm = (CvPoint*)malloc(sizeof(CvPoint));
	SegWorm->PtOnWorm = (CvPoint*)malloc(sizeof(CvPoint));
	SegWorm->NumSegments = 0;

	/*** Setup Memory storage ***/
	SegWorm->MemSegStorage = cvCreateMemStorage(0);

	/*** Allocate Memory for the sequences ***/
	SegWorm->Centerline = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), SegWorm->MemSegStorage);
	SegWorm->LeftBound = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), SegWorm->MemSegStorage);
	SegWorm->RightBound = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), SegWorm->MemSegStorage);

	return SegWorm;
}

/*
 *
 * Clears all the Memory and De-Allocates it
 */
void DestroyNeuronAnalysisDataStruct(NeuronAnalysisData* Neuron) {
	if (Neuron->ImgOrig != NULL)	cvReleaseImage(&(Neuron->ImgOrig));
	if (Neuron->ImgThresh != NULL) cvReleaseImage(&(Neuron->ImgThresh));
	if (Neuron->Mask != NULL) cvReleaseImage(&(Neuron->Mask));
	if (Neuron->mom != NULL) free(Neuron->mom);

#if 0   //添加一下语句会导致程序退出错误
	if (Neuron->Segmented != NULL) DestroySegmentedWormStruct(Neuron->Segmented);

	cvReleaseMemStorage(&(Neuron->MemScratchStorage));
	cvReleaseMemStorage(&(Neuron->MemStorage));

	if (Neuron->Head != NULL) free(Neuron->Head);
	if (Neuron->Tail != NULL) free(Neuron->Tail);
	if (Neuron->prevHead != NULL) free(Neuron->prevHead);
	if (Neuron->prevTail != NULL) free(Neuron->prevTail);
#endif
	free(Neuron);
	Neuron = NULL;
}

void DestroyNeuronAnalysisParam(NeuronAnalysisParam* ParamPtr) {
	free(ParamPtr);
}

void DestroySegmentedWormStruct(SegmentedWorm* SegWorm) {
	cvReleaseMemStorage(&(SegWorm->MemSegStorage));
	free((SegWorm->Head));
	free((SegWorm->Tail));
	free((SegWorm->centerOfWorm));
	free((SegWorm->PtOnWorm));

	free(SegWorm);
}

/** Clear a SegmentedWorm struct but not dallocate memory. **/
void ClearSegmentedInfo(SegmentedWorm* SegWorm) {
	//SegWorm->Head=NULL; /** This is probably a mistake **/
	//SegWorm->Tail=NULL; /** This is probably a mistake  because memory is not reallocated later.**/

	if (SegWorm->LeftBound != NULL) {
		cvClearSeq(SegWorm->LeftBound);
	}
	else {
		printf("SegWorm->LeftBound==NULL");
	}
	if (SegWorm->RightBound != NULL) {
		cvClearSeq(SegWorm->RightBound);
	}
	else {
		printf("SegWorm->RightBound==NULL");
	}

	if (SegWorm->Centerline != NULL) {
		cvClearSeq(SegWorm->Centerline);
	}
	else {
		printf("SegWorm->Centerline==NULL");
	}
}

/*
 * Refersh dynamic memory storage for the worm
 * (clear the memory without freing it)
 *
 */
int RefreshNeuronMemStorage(NeuronAnalysisData* Neuron) {
	if (Neuron->MemScratchStorage != NULL) {
		cvClearMemStorage(Neuron->MemScratchStorage);
	}
	else {
		printf("Error! MemScratchStorage is NULL in RefreshWormMemStorage()!\n");
		return -1;
	}
	if (Neuron->MemStorage != NULL) {
		cvClearMemStorage(Neuron->MemStorage);
	}
	else {
		printf("Error! MemStorage is NULL in RefreshWormMemStorage()!\n");
		return -1;
	}
	return 0;
}

/*
 * This function is run after IntializeEmptyImages.
 * And it loads a properly formated 8 bit grayscale image
 * into the NeuronAnalysisData strucutre.
 *
 * It also sets the timestamp.
 */
int LoadNeuronImg(NeuronAnalysisData* Neuron, NeuronAnalysisParam* Params, IplImage* Img, CvPoint stageFeedbackTargetOffset) {
	CvSize CurrentSize = cvGetSize(Img);
	if ((Neuron->SizeOfImage.height != CurrentSize.height) || (Neuron->SizeOfImage.width != CurrentSize.width)) {
		printf("Error. Image size does not match in  LoadNeuronImg()");
		return -1;
	}
	/** Set the TimeStamp **/
	Neuron->timestamp = clock();
	//std::cout << "Neuron->timestamp: " << Neuron->timestamp << std::endl;

   //及时调整圆形的Mask和方形的Mask
	if (Params->MaskStyle == 0) {
		CvPoint maskCenter; // Center of an mask

		maskCenter = cvPoint(CurrentSize.width / 2 + stageFeedbackTargetOffset.x, CurrentSize.height / 2 + stageFeedbackTargetOffset.y);
		//maskCenter = cvPoint(CurrentSize.width / 2, CurrentSize.height / 2);

		cvZero(Neuron->Mask);
		cvCircle(Neuron->Mask, maskCenter, (Params->MaskDiameter) / 2, cvScalar(255), -1, CV_AA, 0);
	}
	else {
		CvRect maskRect;
		int temp = Params->LowBoundary - Params->RectHeigh / 2;
		int Up = temp > 0 ? temp : 0;

		maskRect = cvRect(0, Up, 1600, Params->RectHeigh);

		cvZero(Neuron->Mask);
		cvRectangleR(Neuron->Mask, maskRect, cvScalar(255), -1, CV_AA, 0);
	}

	/** Copy the Image **/
	IplImage* TempImage = cvCreateImage(CurrentSize, IPL_DEPTH_8U, 1);
	cvZero(TempImage);
	cvCopy(Img, TempImage, Neuron->Mask);
	cvCopy(TempImage, Neuron->ImgOrig, NULL);
	cvReleaseImage(&TempImage);

	return 0;
}

/*
 * Smooths, thresholds and finds the worms contour.
 * The original image must already be loaded into Worm.ImgOrig
 * The Smoothed image is deposited into Worm.ImgSmooth
 * The thresholded image is deposited into Worm.ImgThresh
 * The Boundary is placed in Worm.Boundary
 *
 */
void FindWormBoundary(NeuronAnalysisData* Neuron, NeuronAnalysisParam* Params) {
	/** This function currently takes around 5-7 ms **/
	/**
	 * Before I forget.. plan to make this faster by:
	 *  a) using region of interest
	 *  b) decimating to make it smaller (maybe?)
	 *  c) resize
	 *  d) not using CV_GAUSSIAN for smoothing
	 */

	 /** Smooth the Image **/
	//TICTOC::timer().tic("cvSmooth");
	cvSmooth(Neuron->ImgOrig, Neuron->ImgSmooth, CV_GAUSSIAN, Params->GaussSize * 2 + 1);
	//TICTOC::timer().toc("cvSmooth");

	/** Threshold the Image **/
	//TICTOC::timer().tic("cvThreshold");
	cvThreshold(Neuron->ImgSmooth, Neuron->ImgThresh, Params->BinThresh, 255, CV_THRESH_BINARY);
	//TICTOC::timer().toc("cvThreshold");


	/** Dilate and Erode **/
	if (Params->DilateErode == 1) {
		//TICTOC::timer().tic("DilateAndErode");
		cvDilate(Neuron->ImgThresh, Neuron->ImgThresh, NULL, Params->DilatePm);
		cvErode(Neuron->ImgThresh, Neuron->ImgThresh, NULL, Params->ErodePm);
		//TICTOC::timer().toc("DilateAndErode");
	}

	/** Find Contours **/
	CvSeq* contours;
	IplImage* TempImage = cvCreateImage(cvGetSize(Neuron->ImgThresh), IPL_DEPTH_8U, 1);
	cvCopy(Neuron->ImgThresh, TempImage);
	//TICTOC::timer().tic("cvFindContours");
	cvFindContours(TempImage, Neuron->MemStorage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));
	//TICTOC::timer().toc("cvFindContours");

	if (contours == nullptr) {
		std::cout << "No contours found";
		cvReleaseImage(&TempImage);
		return;
	}


	CvSeq* rough = NULL;
	/** Find Longest Contour **/
	//TICTOC::timer().tic("cvLongestContour");
	if (contours)
	{
		LongestContour(contours, &rough);
		/** Smooth the Boundary **/
		if (Params->BoundSmoothSize > 0) {
			//TICTOC::timer().tic("SmoothBoundary");
			CvSeq* smooth = smoothPtSequence(rough, Params->BoundSmoothSize, Neuron->MemStorage);

			// 释放之前分配的Boundary序列，避免内存泄漏
			//if (Neuron->Boundary) {
			//	cvClearSeq(Neuron->Boundary);
			//	//cvReleaseMemStorage(&(Neuron->Boundary->storage));
			//}

			Neuron->Boundary = cvCloneSeq(smooth);
			//TICTOC::timer().toc("SmoothBoundary");

		}
		else {
			Neuron->Boundary = cvCloneSeq(rough);
		}
	}
	//TICTOC::timer().toc("cvLongestContour");
	cvReleaseImage(&TempImage);
}

/*
 * Finds the Worm's Head and Tail.
 * Requires Worm->Boundary
 *
 */
int GivenBoundaryFindWormHeadTail(NeuronAnalysisData* Neuron, NeuronAnalysisParam* Params) {
	if (Neuron->Boundary == NULL) return -1;
	
	if (Neuron->Boundary->total < 2 * Params->NumSegments) {
		//printf("Error in GivenBoundaryFindWormHeadTail(). The Boundary has too few points.");
		return -1;
	}

	/*** Clear Out Scratch Storage ***/
	cvClearMemStorage(Neuron->MemScratchStorage);

	/* **********************************************************************/
	/*  Express the Boundary in the form of a series of vectors connecting 	*/
	/*  two pixels a Delta pixels apart.									*/
	/* **********************************************************************/

	/* Create A Matrix to store all of the dot products along the boundary.
	 */
	CvSeq* DotProds = cvCreateSeq(CV_32SC1, sizeof(CvSeq), sizeof(int), Neuron->MemScratchStorage);

	/* Create A Matrix to store all of the cross products along the boundary.
	 */
	CvSeq* CrossProds = cvCreateSeq(CV_32SC1, sizeof(CvSeq), sizeof(int), Neuron->MemScratchStorage);

	//We walk around the boundary using the high-speed reader and writer objects.
	//CvSeqReader ForeReader; //ForeReader reads delta pixels ahead
	// Reader; 	//Reader reads delta pixels behind
	//CvSeqReader BackReader; //BackReader reads delta pixels behind


	/**** Local Variables ***/
	int i;
	CvPoint* Pt;
	CvPoint* AheadPt;
	CvPoint* BehindPt;
	CvPoint AheadVec;
	CvPoint BehindVec;
	int TotalBPts = Neuron->Boundary->total;

	/*** Initializing Read & Write Apparatus ***/
	int AheadPtr = 0;
	int BehindPtr = 0;
	int Ptr = 0;
	int* DotProdPtr;
	int* CrossProdPtr;
	int DotProdVal;
	int CrossProdVal;


	/*
	 * Loop through all the boundary and compute the dot products between the ForeVec and BackVec.
	 *
	 * Note: ForeVec and BackVec have the same "handedness" along the boundary.
	 */
	 //printf ("total boundary elements = %d\n", TotalBPts); //debug MHG 10/19/09
	for (i = 0; i < TotalBPts; i++) {
		AheadPtr = (i + Params->LengthScale) % TotalBPts;
		BehindPtr = (i + TotalBPts - Params->LengthScale) % TotalBPts;
		Ptr = (i) % TotalBPts;

		//printf("AheadPtr=%d, BehindPtr=%d,Ptr=%d\n", AheadPtr,BehindPtr,Ptr);


		AheadPt = (CvPoint*)cvGetSeqElem(Neuron->Boundary, AheadPtr);
		Pt = (CvPoint*)cvGetSeqElem(Neuron->Boundary, Ptr);
		BehindPt = (CvPoint*)cvGetSeqElem(Neuron->Boundary, BehindPtr);


		/** Compute the Forward Vector **/
		AheadVec = cvPoint((AheadPt->x) - (Pt->x), (AheadPt->y) - (Pt->y));

		/** Compute the Rear Vector **/
		BehindVec = cvPoint((Pt->x) - (BehindPt->x), (Pt->y) - (BehindPt->y));

		/** Store the Dot Product in our Mat **/
		DotProdVal = PointDot(&AheadVec, &BehindVec);
		cvSeqPush(DotProds, &DotProdVal); //<--- ANDY CONTINUE HERE!

		/** Store the Cross Product in our Mat **/
		CrossProdVal = PointCross(&AheadVec, &BehindVec);
		cvSeqPush(CrossProds, &CrossProdVal);

		//	printf("i= %d, DotProdVal=%d\n", i, DotProdVal);
		//	cvWaitKey(0);
	}


	/* **********************************************************************/
	/*  Find the Tail 													 	*/
	/*  Take dot product of neighboring vectors. Tail is location of		*/
	/*	 smallest dot product												*/
	/* **********************************************************************/


	/*
	 * Now Let's loop through the entire boundary to find the tail, which will be the curviest point.
	 */
	float MostCurvy = 1000; //Smallest value.
	//float CurrentCurviness; //Metric of CurrentCurviness. In this case the dot product.
	int MostCurvyIndex = 0;
	//int TailIndex;

	for (i = 0; i < TotalBPts; i++) {
		DotProdPtr = (int*)cvGetSeqElem(DotProds, i);
		CrossProdPtr = (int*)cvGetSeqElem(CrossProds, i);
		if (*DotProdPtr < MostCurvy && *CrossProdPtr > 0) { //If this locaiton is curvier than the previous MostCurvy location
			MostCurvy = static_cast<float>(*DotProdPtr); //replace the MostCurvy point
			MostCurvyIndex = i;
		}
	}

	//Set the tail to be the point on the boundary that is most curvy.
	Neuron->Tail = (CvPoint*)cvGetSeqElem(Neuron->Boundary, MostCurvyIndex);
	Neuron->TailIndex = MostCurvyIndex;

	/* **********************************************************************/
	/*  Find the Head 													 	*/
	/* 	Excluding the neighborhood of the Tail, the head is the location of */
	/*	 the smallest dot product											*/
	/* **********************************************************************/

	float SecondMostCurvy = 1000;
	int DistBetPtsOnBound = 0;

	/* Set the fallback head location to be halfway away from the tail along the boundary. 	*/
	/* That way, if for some reason there is no reasonable head found, the default 			*/
	/* will at least be a pretty good gueess												*/
	int SecondMostCurvyIndex = (Neuron->TailIndex + TotalBPts / 2) % TotalBPts;



	for (i = 0; i < TotalBPts; i++) {
		DotProdPtr = (int*)cvGetSeqElem(DotProds, i);
		CrossProdPtr = (int*)cvGetSeqElem(CrossProds, i);
		DistBetPtsOnBound = DistBetPtsOnCircBound(TotalBPts, i, MostCurvyIndex);
		//If we are at least a 1/4 of the total boundary away from the most curvy point.
		if (DistBetPtsOnBound > (TotalBPts / 4)) {
			//If this location is curvier than the previous SecondMostCurvy location & is not an invagination
			if (*DotProdPtr < SecondMostCurvy && *CrossProdPtr > 0) {
				SecondMostCurvy = static_cast<float>(*DotProdPtr); //replace the MostCurvy point
				SecondMostCurvyIndex = i;
			}
		}
	}

	Neuron->Head = (CvPoint*)cvGetSeqElem(Neuron->Boundary, SecondMostCurvyIndex);
	Neuron->HeadIndex = SecondMostCurvyIndex;

	cvClearMemStorage(Neuron->MemScratchStorage);
	return 0;
}
//我现在想通过这个函数找到线虫的头和尾巴，但是这个函数使用的opencv版本太老了，我想你帮我改成C++ 的

/*
 * This function reverses the head and the tail of a worm.
 *
 * Note: it does not reverse the sequences that describe the worm's boundary
 * or its segmentation.
 *
 */
int ReverseWormHeadTail(NeuronAnalysisData* Neuron) {
	if (Neuron->Head == NULL || Neuron->Tail == NULL) {
		printf("Error! Head or Tail is NULL in ReverseWormHeadTail().\n");
		return -1;
	}
	CvPoint* tempa = Neuron->Head;
	CvPoint* tempb = Neuron->Tail;
	int tempindexa = Neuron->HeadIndex;
	int tempindexb = Neuron->TailIndex;

	/** Reverse Head and Tail **/
	Neuron->Head = tempb;
	Neuron->Tail = tempa;

	/*** Reverse Head and Tail Index **/
	Neuron->HeadIndex = tempindexb;
	Neuron->TailIndex = tempindexa;
	return 0;
}

/*
 *
 * Returns 1 if the worm is consistent with previous frame.
 * Returns 0 if the worm's head and tail had been reversed from
 *      	  previous frame and fixes the problem.
 * Returns -1 if the head and the tail do not match the previous frame at all
 * Returns 2 if there is no previous worm information
 */
int PrevFrameImproveWormHeadTail(NeuronAnalysisData* Worm, NeuronAnalysisParam* Params, WormGeom* PrevWorm) {

	int DEBUG = 0;
	if (PrevWorm->Head.x == NULL || PrevWorm->Head.y == NULL
		|| PrevWorm->Tail.y == NULL || PrevWorm->Tail.x == NULL
		|| PrevWorm->Perimeter == NULL) {
		/** No previous worm to provide information **/
		if (DEBUG)
			printf("No previous worm to provide information.\n");
		return 2;
	}


	/** Is the Worm's Head and Tail Close to the Previous Frames **/
	CvPoint CurrHead = cvPoint(Worm->Head->x, Worm->Head->y);
	CvPoint CurrTail = cvPoint(Worm->Tail->x, Worm->Tail->y);
	int SqDeltaHead = sqDist(CurrHead, PrevWorm->Head);
	int SqDeltaTail = sqDist(CurrTail, PrevWorm->Tail);
	if (DEBUG) printf("=======================\n");
	if (DEBUG) printf("CurrHead=(%d,%d),CurrTail=(%d,%d)\n", Worm->Head->x, Worm->Head->y, Worm->Tail->x, Worm->Tail->y);
	if (DEBUG) printf("PrevHead=(%d,%d),PrevTail=(%d,%d)\n", PrevWorm->Head.x, PrevWorm->Head.y, PrevWorm->Tail.x, PrevWorm->Tail.y);
	if (DEBUG) printf("SqDeltaTail=%d,SqDeltaHead=%d\n", SqDeltaTail, SqDeltaHead);

	int rsquared = (Params->MaxLocationChange) * (Params->MaxLocationChange);

	if ((SqDeltaHead > rsquared) || (SqDeltaTail > rsquared)) {
		/** The previous head/tail locations aren't close.. **/
		/** Is the inverse close? **/
		int SqDeltaHeadInv = sqDist(CurrHead, PrevWorm->Tail);
		int SqDeltaTailInv = sqDist(CurrTail, PrevWorm->Head);
		if (DEBUG) printf("SqDeltaTailInv=%d,SqDeltaHeadInv=%d\n", SqDeltaTailInv, SqDeltaTailInv);
		if ((SqDeltaHeadInv < rsquared) || (SqDeltaTailInv < rsquared)) {
			/** The inverse is close, so let's reverse the Head Tail**/
			ReverseWormHeadTail(Worm);
			if (DEBUG) printf("ReversedWormHeadTail\n");
			return 0;

		}
		else {
			/** The Head and Tail is screwed up and its not related to simply inverted **/
			if (DEBUG) printf(
				"Head moved by a squared distance of %d pixels\n Tail moved by a squared distance of %d pixels\n",
				SqDeltaHead, SqDeltaTail);
			if (DEBUG)
				printf("Head and Tail Screwed Up");
			return -1;

		}
	}
	if (DEBUG)
		printf("All good.\n");
	return 1; /** The Head and Tail are within the required distance **/

}

/*
 * This Function segments a worm.
 * It requires that certain information be present in the WormAnalysisData struct Worm
 * It requires Worm->Boundary be full
 * It requires that Params->NumSegments be greater than zero
 *
 */
int SegmentWorm(NeuronAnalysisData* Worm, NeuronAnalysisParam* Params) {

	if (cvSeqExists(Worm->Boundary) == 0) {
		printf("Error! No boundary found in SegmentWorm()\n");
		return -1;
	}


	Worm->Segmented->NumSegments = Params->NumSegments;

	/***Clear Out any stale Segmented Information Already in the Worm Structure***/
	ClearSegmentedInfo(Worm->Segmented);

	Worm->prevHead = Worm->Head;
	Worm->prevTail = Worm->Tail;
	Worm->Segmented->Head = Worm->Head;
	Worm->Segmented->Tail = Worm->Tail;

	/*** It would be nice to check that Worm->Boundary exists ***/

	/*** Clear Out Scratch Storage ***/
	cvClearMemStorage(Worm->MemScratchStorage);


	/*** Slice the boundary into left and right components ***/
	if (Worm->HeadIndex == Worm->TailIndex) printf("Error! Worm->HeadIndex == Worm->TailIndex in SegmentWorm()!\n");
	CvSeq* OrigBoundA = cvSeqSlice(Worm->Boundary, cvSlice(Worm->HeadIndex, Worm->TailIndex), Worm->MemScratchStorage, 1);
	CvSeq* OrigBoundB = cvSeqSlice(Worm->Boundary, cvSlice(Worm->TailIndex, Worm->HeadIndex), Worm->MemScratchStorage, 1);

	if (OrigBoundA->total < Params->NumSegments || OrigBoundB->total < Params->NumSegments) {
		printf("Error in SegmentWorm():\n\tWhen splitting  the original boundary into two, one or the other has less than the number of desired segments!\n");
		printf("OrigBoundA->total=%d\nOrigBoundB->total=%d\nParams->NumSegments=%d\n", OrigBoundA->total, OrigBoundB->total, Params->NumSegments);
		printf("Worm->HeadIndex=%d\nWorm->TailIndex=%d\n", Worm->HeadIndex, Worm->TailIndex);
		printf("It could be that your worm is just too small\n");
		return -1; /** Andy make this return -1 **/

	}

	cvSeqInvert(OrigBoundB);


	/*** Resample One of the Two Boundaries so that both are the same length ***/

	//Create sequences to store the Normalized Boundaries
	CvSeq* NBoundA = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), Worm->MemScratchStorage);
	CvSeq* NBoundB = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), Worm->MemScratchStorage);

	//Resample L&R boundary to have the same number of points as min(L,R)
	if (OrigBoundA->total > OrigBoundB->total) {
		resampleSeq(OrigBoundA, NBoundA, OrigBoundB->total);
		NBoundB = OrigBoundB;
	}
	else {
		resampleSeq(OrigBoundB, NBoundB, OrigBoundA->total);
		NBoundA = OrigBoundA;
	}
	//Now both NBoundA and NBoundB are the same length.



	/*
	 * Now Find the Centerline
	 *
	 */

	 /*** Clear out Stale Centerline Information ***/
	cvClearSeq(Worm->Centerline);

	/*** Compute Centerline, from Head To Tail ***/
	FindCenterline(NBoundA, NBoundB, Worm->Centerline);
#if 1

	/*** Smooth the Centerline***/
	CvSeq* SmoothUnresampledCenterline = smoothPtSequence(Worm->Centerline, 0.5 * Worm->Centerline->total / Params->NumSegments, Worm->MemScratchStorage);

	/*** Note: If you wanted to you could smooth the centerline a second time here. ***/


	/*** Resample the Centerline So it has the specified Number of Points ***/
	//resampleSeq(SmoothUnresampledCenterline,Worm->Segmented->Centerline,Params->NumSegments);

	resampleSeqConstPtsPerArcLength(SmoothUnresampledCenterline, Worm->Segmented->Centerline, Params->NumSegments);

	/** Save the location of the centerOfWorm as the point halfway down the segmented centerline **/
	Worm->Segmented->centerOfWorm = CV_GET_SEQ_ELEM(CvPoint, Worm->Segmented->Centerline, Worm->Segmented->NumSegments / 2);

	if (Worm->Segmented->PtOnWorm == NULL) Worm->Segmented->PtOnWorm = (CvPoint*)malloc(sizeof(CvPoint));
	Worm->Segmented->PtOnWorm = (CvPoint*)cvGetSeqElem(Worm->Segmented->Centerline, Params->stageTargetSegment);
#endif
	/*** Remove Repeat Points***/
	//RemoveSequentialDuplicatePoints (Worm->Segmented->Centerline);

	/*** Use Marc's Perpendicular Segmentation Algorithm
	 *   To Segment the Left and Right Boundaries and store them
	 */
	 //	SegmentSides(OrigBoundA, OrigBoundB, Worm->Segmented->Centerline, Worm->Segmented->LeftBound, Worm->Segmented->RightBound);
	return 0;

}
//我现在想通过这个函数把线虫的轮廓分成上下两个部分，并找到线虫的中心线，但是这个函数使用的opencv版本太老了，我想你帮我改成C++ 的