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
 *
 * This library contains functions to write out frame-by-frame information about
 * the worm's position, orientation and the illumination stimuli that it is receiving.
 *
 * All of the data is written out using YAML which is a human and computer readable file
 * format.
 *
 */

/*
 * WriteOutWorm.c
 *
 *  Created on: Nov 3, 2009
 *      Author: Andy
 */
#pragma warning(disable : 4996)

#include <stdio.h>
#include <time.h>
#include <string.h>

//OpenCV Headers
#include <cxcore.h>
#include "opencv2/highgui/highgui_c.h"
#include <cv.h>

// Andy's Libraries

#include "AndysComputations.h"
#include "AndysOpenCVLib.h"
#include "NeuronAnalysis.h"

#include "WriteOutNeuron.h"





/*
 * Shortcut function to create a filename.
 * Basically just concatenates three strings together.
 *
 */
char* CreateFileName(const char* dir, const char* core, const char* suffix){

	/* Prepare String with Time Stamp */
	char timestamp[30];
	time_t curtime;
	struct tm *loctime;
	/* Get the current time. */
		curtime= time (NULL);
	/* Convert to Local Time */
	loctime = localtime (&curtime);
	strftime(timestamp,30,"%Y%m%d_%H%M_",loctime);
	/* Allocate memory for filename */
	char* filename= (char*) malloc(strlen(timestamp)+strlen(dir)+strlen(core)+strlen(suffix)+1);


	/*Concatenate Strings */
	strcpy(filename,dir);
	strcat(filename,timestamp);
	strcat(filename,core);
	strcat(filename,suffix);
	printf("Preparing filename: %s\n",filename);

	return filename;
}

/*
 * Destroy's a filename and deallocates the poitner and such.
 */
void DestroyFilename(char** filename){
	free(*filename);
	*filename=NULL;
}


/*
 * Allocates memory for and creates a DataWriter object and sets all values to zero or NULL;
 */
WriteOut* CreateDataWriter(){
	WriteOut* DataWriter =(WriteOut*) malloc(sizeof(WriteOut));
	DataWriter->error=0;
	DataWriter->filename=NULL;
	DataWriter->fs=NULL;
	return DataWriter;
}

/*
 * Sets up the WriteToDisk  given the base of a filname.
 * Creates a WriteOut Object.
 *
 * e.g. if you want to create a files named myexperiment.yaml and myexperiment.mov
 * pass in the string "myexperiment"
 *
 */
WriteOut* SetUpWriteToDisk(const char* dirfilename, const char* outfilename,  CvMemStorage* Mem){
	/** Create new instance of WriteOut object **/
	WriteOut* DataWriter = CreateDataWriter();

	/** Create Filenames **/
	char* YAMLDataFileName = CreateFileName(dirfilename, outfilename, ".yaml");

	/** Open YAML Data File for writing**/
	DataWriter->fs=cvOpenFileStorage(YAMLDataFileName,Mem,CV_STORAGE_WRITE);
	if (DataWriter->fs==0) {
		printf("DataWriter->fs is zero! Could you have specified the wrong directory?\n");
		--(DataWriter->error);
	}

	/** If there were errors, return immediately **/
	if (DataWriter->error < 0) return DataWriter;


	/** Write the header for the YAML data file **/
	cvWriteComment(DataWriter->fs, "Remote Control Worm Experiment Data Log\nMade by OpticalMindControl software\nShYang_0594@bupt.edu.cn\n",0);

	/** Write Out Current Time**/
	  struct tm *local;
	  time_t t;

	  t = time(NULL);
	  local = localtime(&t);

	cvWriteString(DataWriter->fs, "ExperimentTime",asctime(local),0);

	DataWriter->filename=YAMLDataFileName;

	return DataWriter;
}


/*
 * Start the process of writing out frames. (Formerly this was contained in SetUpWriteToDisk)
 */
void BeginToWriteOutFrames(WriteOut* DataWriter){
	cvStartWriteStruct(DataWriter->fs,"Frames",CV_NODE_SEQ,NULL);
	return;
}


/*
 * Writes Out information of one frame of the worm to a disk
 * in YAML format.
 *
 * Note the Worm object must have the following fields
 * Worm->frameNum
 * Worm->Segmented->Head
 * Worm->Segmented->Tail
 * Worm->Segmented->LeftBound
 * Worm->Segmented->RightBound
 * Worm->Segmented->Centerline
 *
 * and Params object must have
 * Params->DLPOn
 * Params->IllumSegCenter
 * Params->IllumSegRadius
 * Params->IllumLRC
 *
 * And more now!
 */
int AppendNeuronFrameToDisk(NeuronAnalysisData* Neuron, NeuronAnalysisParam* Params, WriteOut* DataWriter){

	CvFileStorage* fs=DataWriter->fs;

	cvStartWriteStruct(fs,NULL,CV_NODE_MAP,NULL);
		/** Frame Number Info **/
		cvWriteInt(fs,"FrameNumber",Neuron->frameNum);

		/** TimeStamp **/
		cvWriteInt(fs,"sElapsed",GetSeconds(Neuron->timestamp));
		cvWriteInt(fs,"msRemElapsed",GetMilliSeconds(Neuron->timestamp));

		cvStartWriteStruct(fs, "StagePosition", CV_NODE_MAP, NULL);
			cvWriteReal(fs, "x", Neuron->stagePosition_x);
			cvWriteReal(fs, "y", Neuron->stagePosition_y);
		cvEndWriteStruct(fs);

		//添加LED的信息
		cvWriteInt(fs, "LedStatus", Params->Led0_status);
		if (Params->Led0_status == 1) {
			cvWriteInt(fs, "Brightness", Params->BrightnessPerc);
		}

		//添加线虫的速度 单位um/s
		cvWriteReal(fs, "CurVelocity", Params->cur_velocity);

		//添加当前的相对坐标 
		cvStartWriteStruct(fs, "StandardCoordinates", CV_NODE_MAP, NULL);
		cvWriteReal(fs, "x", Params->standardCorX);
		cvWriteReal(fs, "y", Params->standardCorY);
		cvEndWriteStruct(fs);

		//添加电场的信息
		cvStartWriteStruct(fs, "EF_Status", CV_NODE_MAP, NULL);
		char eleStatus[7] = { Params->vChannel, ':', Params->vWave, '!', Params->vDir, Params->vNum, '\0'};
		cvWriteString(fs, "SignalType", eleStatus);

		cvEndWriteStruct(fs);

	cvEndWriteStruct(fs);
	return 0;
}

int DestroyDataWriter(WriteOut** DataWriter){
	cvReleaseFileStorage(&((*DataWriter)->fs));
	free((*DataWriter)->filename);
	free(*DataWriter);
	*DataWriter=NULL;
	return 0;
}

/*
 * Finish writing to disk and close the file and such.
 * Destroys the Data Writer
 *
 */
int FinishWriteToDisk(WriteOut** DataWriter){
	CvFileStorage* fs=(*DataWriter)->fs;
	/** Finish writing this structure **/
	cvEndWriteStruct(fs);

	/** Close File Storage and Finish Writing Out to File **/
	DestroyDataWriter(DataWriter);
	return 0;
}
