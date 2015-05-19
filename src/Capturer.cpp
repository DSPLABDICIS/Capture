/*
 * Capturer.cpp
 *
 *  Created on: Jun 16, 2014
 *      Author: dsplab
 */

#include "../include/Capturer.h"

namespace capture {

Capturer::Capturer() : save(false),saveEvery(1),saveCounter(1),nextFrameReady(false),
		timeFeedPrevious(boost::posix_time::microsec_clock::local_time()),
		deltaCaptureTime(0),
		timeCapturePrevious(boost::posix_time::microsec_clock::local_time()),
		deltaTimeStored(0){

}

Capturer::~Capturer() {
}

bool Capturer::saveFeed(const char* str1, const char* str2, int saveEvery){
	saveS1 = str1;
	saveS2 = str2;
	this->saveEvery = saveEvery;
	return true;
}

bool Capturer::saveMat(cv::Mat& matrix, const char* name, double dt)
{
	FILE *fp = NULL;
	float * ptr;
	fp = fopen(name,"w+b");
	int size[]={matrix.rows,matrix.cols,(int)matrix.step1(1)};

	if(fp != NULL){
		cout<<"Time since last Save "<<dt<<endl;
		fwrite(&dt,sizeof(double),1,fp);
		fwrite(size,sizeof(int),3,fp);
		ptr = matrix.ptr<float>();
		fwrite(ptr,sizeof(float),matrix.rows*matrix.cols*matrix.step1(1),fp);
		fclose(fp);
		return true;
	}
	return false;
}

bool Capturer::loadMat(const char* name)
{
	FILE *fp = NULL;
	float * ptr;
	fp = fopen(name,"rb");
	int size[3];


	if(fp != NULL){
		fread(&deltaTimeStored,sizeof(double),1,fp);
		fread(size,sizeof(int),3,fp);
		ptr = (float*)malloc(size[0]*size[1]*size[2]*sizeof(float));
		fread(ptr,sizeof(float),size[0]*size[1]*size[2],fp);

		if(size[2]==1 || size[2]==3){
			cv::Mat matrix(size[0],size[1],CV_MAKETYPE(CV_32F,size[2]),ptr);
			positionFrameStored = matrix;
		}
		else
		{
			cout<< "File "<< name<<"'s Header Corrupted"<<endl;
			return false;
		}

		cout<<deltaTimeStored<<"   "<<size[0]<<"   "<<size[1]<<"   "<<size[2]<<endl;
		fclose(fp);
		return true;
	}
	cout<< "Position file "<< name<<" not found"<<endl;
	return false;
}

} /* namespace capture */

