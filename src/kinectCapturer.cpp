/*
 * kinectCapturer.cpp
 *
 *  Created on: May 14, 2014
 *      Author: dsplab
 */

#include "../include/kinectCapturer.h"



namespace capture{

kinectCapturer::kinectCapturer():live_feed(true), start(0), finish(0), increment(0), current(0),
		runLiveFeed(false), filter(7,3.5,3.5){

}

kinectCapturer::kinectCapturer(const char* _filename1, const char* _extension1, const char* _filename2,
		const char* _extension2, int _start, int _finish, int _increment):live_feed(false),
				start(_start), finish(_finish), increment(_increment), current(start),
				runLiveFeed(false), filter(7,3.5,3.5){
	loadS1 = _filename1;
	ext1=_extension1;
	loadS2=_filename2;
	ext2=_extension2;
}

kinectCapturer::~kinectCapturer() {
	runLiveFeed = false;
	feedThread.join();
	cout<<"Deleted kinect Capturer"<<endl;
}

bool kinectCapturer::initilize(double dt){
	if(live_feed)
	{
		capturer = VideoCapture(CV_CAP_OPENNI);
		try{
			startLiveFeed();
		}catch (char const * str){
			cout<<str<<endl;
			return false;
		}
	}
	return true;
}

bool kinectCapturer::saveFeed(const char* str1, const char* str2, int saveEvery){
	Capturer::saveFeed(str1,str2,saveEvery);
	if(saveS1.size()<1){
		cerr<<"Invalid Color name (name less than 1 character)"<<endl;
		return false;
	}
	if(saveS2.size()<1){
		cerr<<"Invalid Position name (name less than 1 character)"<<endl;
		return false;
	}
	save = true;
	cout<<"Start Saving"<<endl;
	return true;
}

imageState kinectCapturer::get_frame(Mat & image, double * dt){
	//cout<<"Running thread "<<runLiveFeed<<endl;
	if(live_feed){
		//cout<<"Copy mutex"<<endl;
		while(nextFrameReady == false){
		}
		boost::mutex::scoped_lock lock(captureMutex);
		colorFrameStored = colorFrame;
		image = colorFrameStored;
		positionFrameStored = positionFrame;
		boost::posix_time::time_duration dur = timeCurrent - timeFeedPrevious;
		deltaTimeStored =  dur.total_milliseconds();
		*dt = deltaTimeStored;
		timeFeedPrevious = timeCurrent;
		nextFrameReady = false;
		//cout<<"Free copy mutex"<<endl;
		return CAPTURE_SUCESSFUL;
	}
	else{
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
		//cv::FileStorage file(buffer,cv::FileStorage::READ);
		//if(file.isOpened() == false){
		//cout<< "Position file "<< buffer<<" not found"<<endl;
		//return CAPTURE_NOT_FOUND;
		//}
		//file["Position"] >> positionFrameStored;
		*dt = deltaTimeStored;
		current+=increment;
		cout<<endl<<endl<<"Read image "<<loadS1.data()<<current-1<<endl;
		if(*dt<=0)
		{
			std::cerr<<"The camera is surely not fast to capture every 0 seconds overriding time value to 15fps"<<std::endl;
			*dt = 0.06666f;
		}

		return CAPTURE_SUCESSFUL;
	}

}

Mat* kinectCapturer::capture(){

	if(!capturer.isOpened())
	{
		cout<<"Device not opened"<<endl;
		return NULL;
	}

	this->capturer.grab();

	this->capturer.retrieve( positionFrame, CV_CAP_OPENNI_POINT_CLOUD_MAP );
	this->capturer.retrieve( colorFrame, CV_CAP_OPENNI_BGR_IMAGE );

	return &colorFrame;

}

void kinectCapturer::startLiveFeed(){

	if(!capturer.isOpened())
		throw("Start Live Feed : Device not opened");

	if(runLiveFeed == true)
		return;
	runLiveFeed = true;
	feedThread = boost::thread(&kinectCapturer::livefeedThread,this);
	cout<<"Kinect Live Feed started"<<endl;
}

void kinectCapturer::stopLiveFeed(){

	runLiveFeed = false;
}

void kinectCapturer::livefeedThread(){
	char buffer[100];
	int ind = -1;
	bool show = 0;
	boost::posix_time::ptime startTime, stopTime, deltaCaptureStart, deltaCaptureStop;

	if(show)
		namedWindow("Image RGB",CV_WINDOW_AUTOSIZE);

	while(runLiveFeed)
	{
		startTime = boost::posix_time::microsec_clock::local_time();

		this->capturer.grab();
		{	//Save Scope for mutex
			//cout<<"try lock"<<endl;
			boost::mutex::scoped_try_lock lock(captureMutex);
			if(lock.owns_lock())
			{
				//cout<<"Got Lock"<<endl;
				timeCurrent = boost::posix_time::microsec_clock::local_time();
				this->capturer.retrieve( positionFrame, CV_CAP_OPENNI_POINT_CLOUD_MAP );
				this->capturer.retrieve( colorFrame, CV_CAP_OPENNI_BGR_IMAGE );
				nextFrameReady = true;
				//cout<<"release lock "<<deltaFeedTime<<endl;
			}

		}

		if (show){
			imshow("Image RGB",this->colorFrame);
			cv::waitKey(1);
		}

		if(save){
			cout<<saveCounter<<" less than "<<saveEvery<<endl;
			if(saveCounter++ >= saveEvery){
				sprintf(buffer,"%s%i.tiff",saveS1.data(),++ind);
				cout<<buffer<<endl;
				if(!cv::imwrite( buffer, colorFrame))
				{
					cerr<<"Save Error in live feed : color frame, check parameters"<<endl;
					save = false;
				}
				else{
					boost::posix_time::time_duration dur = timeCurrent - timeCapturePrevious;
					timeCapturePrevious = timeCurrent;
					sprintf(buffer,"%s%i.M3D",saveS2.data(),ind);
					cout<<buffer<<endl;
					if(!saveMat(positionFrame,buffer,(double)dur.total_milliseconds()/1000.0*saveEvery)){
						cerr<<"Save Error in live feed : position frame, check parameters"<<endl;
						save = false;
					}
				}
				saveCounter = 1;
			}
		}

	}

	stopTime = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration dur = stopTime - startTime;
	double milliseconds = dur.total_milliseconds();
	//cout<<"Time Passed "<<milliseconds<<" Milli seconds"<<endl;
	if(milliseconds < 33){
		//cout<<"Wait "<< 33.0-milliseconds <<endl;
		boost::this_thread::sleep(boost::posix_time::milliseconds(33.0-milliseconds)); //Frames per second = delay + processing time
	}

	//		stopTime = boost::posix_time::microsec_clock::local_time();
	//		dur = stopTime - startTime;
	//		milliseconds = dur.total_milliseconds();
	//		cout<<"fps "<<1000/(milliseconds)<<endl;

	//			cout<<"Fps: "<<(1000/milliseconds)<<endl;
}

void kinectCapturer::projectFeatureList(KLT_FeatureList fl){

	filter.pointProjection(positionFrameStored, fl);

}

}//end namesapce capture

