/*
 * MonoCamera.cpp
 *
 *  Created on: Sep 11, 2014
 *      Author: crono21
 */

#include "../include/MonoCamera.h"

namespace capture {

MonoCamera::MonoCamera():live_feed(true), start(0), finish(0), increment(0), current(0),
		runLiveFeed(false){

}

MonoCamera::MonoCamera(const char* _filename1, const char* _extension1,
		int _start, int _finish, int _increment):live_feed(false),
				start(_start), finish(_finish), increment(_increment), current(start),
				runLiveFeed(false){
	loadS1 = _filename1;
	ext1=_extension1;
}

MonoCamera::~MonoCamera() {

}

bool MonoCamera::initilize(double dt){
	deltaTimeStored = dt;
	return true;
}

imageState MonoCamera::get_frame(Mat & image, double * dt){
	char buffer[255];
	if(current > finish){
		cout<<"End of frames"<<endl;
		return CAPTURE_END_OF_CAPTURE;
	}

	sprintf(buffer,"%s%i.%s",loadS1.data(),current,ext1.data());
	colorFrameStored = cv::imread(buffer,0);
	if(colorFrameStored.data == NULL){
		cout<< "image "<< buffer<<" not found"<<endl;
		return CAPTURE_NOT_FOUND;
	}
	image = colorFrameStored;

	*dt = deltaTimeStored;
	current++;
	cout<<"Read image "<<loadS1.data()<<current-1<<endl;
	return CAPTURE_SUCESSFUL;
}

void MonoCamera::projectFeatureList(KLT_FeatureList fl){
	//No projection for a Monocamera setup
}


} /* namespace capture */
