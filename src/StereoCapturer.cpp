/*
 * StereoCaturer.cpp
 *
 *  Created on: Aug 25, 2014
 *      Author: crono21
 */

#include "../include/StereoCapturer.h"

namespace capture {

StereoCapturer::StereoCapturer(bool _fullCapture): live_feed(true), start(0), finish(0), increment(0), current(0),
		runLiveFeed(false),fullCapture(_fullCapture), filter(7,3.5,3.5),camera(new cameras::CameraGrabber){
}

StereoCapturer::StereoCapturer(const char* _filename1, const char* _extension1, const char* _filename2,
		const char* _extension2,  int _start, int _finish, int _increment, const char* _intrinsics,
		const char* _extrinsics): live_feed(false),
				start(_start), finish(_finish), increment(_increment), current(start),
				runLiveFeed(false), fullCapture(false), filter(7,3.5,3.5),camera(NULL){
	loadS1 = _filename1;
	ext1=_extension1;
	loadS2=_filename2;
	ext2=_extension2;
	intrinsics = _intrinsics;
	extrinsics = _extrinsics;
}

StereoCapturer::~StereoCapturer() {
	runLiveFeed = false;
	feedThread.join();
	cout<<"Deleted Stereo Capturer"<<endl;

	if(camera!=NULL)
		delete(camera);
}

bool StereoCapturer::initilize(double dt){

	deltaTimeStored = dt;
	if(live_feed)
	{
		camera->initializeCameras();
		if(startLiveFeed())
			return false;

	}
	else {
		char buffer[100];
		sprintf(buffer,"%s%i.%s",loadS1.data(),current,ext1.data());
		colorFrameStored = cv::imread(buffer,1);
		if(colorFrameStored.data == NULL){
			cout<< "image "<< buffer<<" not found"<<endl;
			return 0;
		}
		img_size = colorFrameStored.size();

		cv::FileStorage fs(this->intrinsics, CV_STORAGE_READ);
		if(!fs.isOpened())
		{
			printf("Failed to open file %s\n", this->intrinsics.data());
			return true;
		}

		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;

		fs.open(this->extrinsics, CV_STORAGE_READ);
		if(!fs.isOpened())
		{
			printf("Failed to open file %s\n", this->extrinsics.data());
			return true;
		}

		fs["R"] >> R;
		fs["T"] >> T;

		cv::stereoRectify( M1, D1, M2, D2, this->img_size, R, T, R1, R2, P1, P2, Q, 0/*cv::CALIB_ZERO_DISPARITY*/, -1, this->img_size, &roi1, &roi2 );


		cv::initUndistortRectifyMap(M1, D1, R1, P1, this->img_size, CV_16SC2, map11, map12);
		cv::initUndistortRectifyMap(M2, D2, R2, P2, this->img_size, CV_16SC2, map21, map22);

		if(!ext2.compare("DMX")){
			bm.state->roi1 = this->roi1;
			bm.state->roi2 = this->roi2;
			bm.state->preFilterSize = 63;
			bm.state->preFilterCap = 63;
			bm.state->SADWindowSize = 9;
			bm.state->minDisparity = 0;
			bm.state->numberOfDisparities = 64;
			bm.state->textureThreshold = 0;
			bm.state->uniquenessRatio = 0;
			bm.state->speckleWindowSize = 100;
			bm.state->speckleRange = 100;
		}
	}
	return true;
}

imageState StereoCapturer::get_frame(Mat & image, double * dt){
	cout<<"Running thread "<<runLiveFeed<<endl;
	if(live_feed){
		cerr<<"Due to processing time limitations, the image cannot be grabbed from a live feed"<<endl;
		return CAPTURE_NOT_FOUND;
	}
	else
		if(!ext2.compare("DMX")){
			char buffer[255];
			if(current > finish){
				cout<<"End of frames"<<endl;
				return CAPTURE_END_OF_CAPTURE;
			}

			sprintf(buffer,"%s%i.%s",loadS1.data(),current,ext1.data());
			colorFrameStored = cv::imread(buffer,1);
			if(colorFrameStored.data == NULL){
				cout<< "image "<< buffer<<" not found"<<endl;
				return CAPTURE_NOT_FOUND;
			}
			image = colorFrameStored;

			sprintf(buffer,"%s%i.%s",loadS2.data(),current,ext2.data());
			if(!loadMat(buffer))
				return CAPTURE_NOT_FOUND;

			*dt = deltaTimeStored;
			current+=increment;
			cout<<"Read image "<<loadS1.data()<<current-1<<endl;
			return CAPTURE_SUCESSFUL;
		}
		else{
			cerr<<"To Compute the disparity call the StereoCapturer::computeStereo member function "<<endl;
			return CAPTURE_NOT_FOUND;
		}
}

int StereoCapturer::startLiveFeed(){

	return camera->startGrabb(fullCapture,"images/testStereoNear");
}


void StereoCapturer::computeStereo(float dt, bool display, bool save_Rectified){

	double min, max;
	cv::Mat imgD,imgD2;
	int i,w,h;
	char buffer[255];


	if(intrinsics.empty()||extrinsics.empty()){
		cerr<< "Missing intrinsic and extrinsic file, file paths required at constructor"<<endl;
		return;
	}

	while(true){
		if(current > finish){
			cout<<"End of frames"<<endl;
			return;
		}

		sprintf(buffer,"%s%i.%s",loadS1.data(),current,ext1.data());
		imgLeft = cv::imread(buffer,0);							//left Image
		if(imgLeft.data == NULL){
			cout<< "image "<< buffer<<" not found"<<endl;
			return;
		}

		sprintf(buffer,"%s%i.%s",loadS2.data(),current,ext2.data());
		imgRight = cv::imread(buffer,0);							//Right Image
		if(imgRight.data == NULL){
			cout<< "image "<< buffer<<" not found"<<endl;
			return;
		}


		cout<<"Read image "<<loadS1.data()<<current<<endl;

		boost::posix_time::ptime startTime, stopTime;


		startTime = boost::posix_time::microsec_clock::local_time();
		computeDisparity(imgDepth);
		stopTime = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration dur = stopTime - startTime;
		double milliseconds = dur.total_milliseconds();
		cout<<"Time Passed "<<milliseconds<<" Milli seconds"<<endl;

		cout<<(int)imgDepth.step1(1)<<endl;

		cv::minMaxIdx(imgDepth, &min, &max);
		cout<<min<<"    "<<max<<endl;
		cout<<255 / (max-min)<<endl;
		cv::normalize(imgDepth,imgD,0, 255,NORM_MINMAX,CV_8UC1);
		cv::minMaxIdx(imgD, &min, &max);
		cout<<min<<"    "<<max<<endl;

		sprintf(buffer,"disparity/depthImage%i.png",current);
		imwrite(buffer,imgD);									//Viewing purposes
		sprintf(buffer,"disparity/depthImage%i.DMX",current);
		saveMat(imgDepth,buffer,dt);							//Processing purposes

		if(save_Rectified)
		{
			sprintf(buffer,"%sRectified%i.png",loadS1.data(),current);
			imwrite(buffer,imgLeftr);
			sprintf(buffer,"%sRectified%i.png",loadS2.data(),current);
			imwrite(buffer,imgRight);
		}
		if(display){

			w = imgRightr.size().width;
			h = imgRightr.size().height;
			Mat canvas;
			Mat tImage;
			canvas.create(h, w*2, CV_8UC3);

			Mat canvasPart = canvas(Rect(0, 0, w, h));
			cvtColor(imgLeftr, tImage, COLOR_GRAY2BGR);
			resize(tImage, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
			Rect vroil(cvRound(roi1.x), cvRound(roi1.y),
					cvRound(roi1.width), cvRound(roi1.height));
			rectangle(canvasPart, vroil, Scalar(0,0,255), 3, 8);
			canvasPart = canvas(Rect(w, 0, w, h));
			cvtColor(imgRightr, tImage, COLOR_GRAY2BGR);
			resize(tImage, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
			Rect vroir(cvRound(roi2.x), cvRound(roi2.y),
					cvRound(roi2.width), cvRound(roi2.height));
			rectangle(canvasPart, vroir, Scalar(0,0,255), 3, 8);




			for( i = 0; i < canvas.rows; i += 16 )
				line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);

			//			imshow("left Image",imgLeftr);
			//			imshow("right Image", imgRightr);
			imshow("Rectified Images",canvas);
			imshow("Depth image",imgD);
			waitKey(0);
		}
		current++;
	}
}

void StereoCapturer::rectify(cv::Mat& imglr, cv::Mat& imgrr)
{
	cv::remap(this->imgLeft, imglr, map11, map12, cv::INTER_LINEAR);
	cv::remap(this->imgRight, imgrr, map21, map22, cv::INTER_LINEAR);
}

void StereoCapturer::computeDisparity(cv::Mat &output)
{
	rectify(this->imgLeftr,this->imgRightr);
	bm(this->imgLeftr, this->imgRightr, output,CV_32F);
}

void StereoCapturer::projectFeatureList(KLT_FeatureList fl){
	filter.StereoPointProjection(positionFrameStored,fl,Q);
}

void StereoCapturer::project(bool save){

	int x,y;
	int i=0;
	char buffer[255];
	int cols = colorFrameStored.cols;
	int rows = colorFrameStored.rows;
	float* matrix = new float[3];
	float* ptr = positionFrameStored.ptr<float>();
	FILE *fp;
	sprintf(buffer,"%s3d.txt",loadS2.data());
	if (save)
		fp = fopen(buffer,"w");

	for(x=0;x<cols;x++)
		for(y=0;y<rows;y++){
			if(ptr[y*cols+x]!=-1){
				cv::Mat_<double> vec(4,1);
				vec(0)=x; vec(1)=y; vec(2)=ptr[y*cols+x]; vec(3)=1;
				vec = Q*vec;
				vec /= vec(3);
				matrix[0]=vec(0);
				matrix[1]=vec(1);
				matrix[2]=vec(2);
			}
			else{
				matrix[0]=0;
				matrix[1]=0;
				matrix[2]=0;
			}
			if(save)
				fprintf(fp,"%3.7f\t%3.7f\t%3.7f\n",matrix[0],matrix[1],matrix[2]);
		}

	if(save)
		fclose(fp);
}

} /* namespace cluster */
