/*
 * StereoCaturer.h
 *
 *  Created on: Aug 25, 2014
 *      Author: crono21
 */

#ifndef STEREOCATURER_H_
#define STEREOCATURER_H_

#include "Capturer.h"
#include "../../libCam/include/cameraGrabber.h"

namespace capture {
class StereoCapturer: public Capturer{
public:
	StereoCapturer(bool _fullCapture= false);																	//Live Feed
	StereoCapturer(const char* _filename1, const char* _extension1, const char* _filename2,					//Stereo Computing or Image processing
			const char* _extension2,  int _start, int _finish, int _increment, const char* _intrinsics,
			const char* _extrinsics);
	virtual ~StereoCapturer();
	bool initilize(double dt = 0);
	bool saveFeed(const char* str1, const char* str2="",int saveEvery = 1){return true;}
	imageState get_frame(Mat & image, double * dt);
	void projectFeatureList(KLT_FeatureList fl);
	void computeStereo(float dt, bool display, bool save_Rectified);
	void project(bool save);

private:
	int startLiveFeed();
	void livefeedThread(){};




	bool live_feed;
	int start, finish, increment, current;
	boost::thread feedThread;
	//	boost::mutex captureMutex;
	bool runLiveFeed;
	bool fullCapture;
	DepthFilter::DepthFilter filter;

	//Stereo Functions and variables
	cameras::CameraGrabber * camera;
	void rectify(cv::Mat& imglr, cv::Mat& imgrr);
	void computeBM(cv::Mat &output);
	void computeDisparity(cv::Mat &output);

	//	Mat colorFrame;
	//	Mat positionFrame;
	string intrinsics;
	string extrinsics;
	cv::Mat imgLeft;
	cv::Mat imgRight;
	cv::Mat imgLeftr;
	cv::Mat imgRightr;
	cv::Size img_size;
	cv::Mat M1, D1, M2, D2;
	cv::Mat R, T, R1, P1, R2, P2;
	cv::Mat Q;
	cv::Mat map11, map12, map21, map22;
	cv::Rect roi1,roi2;
	cv::StereoBM bm;
	cv::Mat imgDepth;
};

} /* namespace cluster */

#endif /* STEREOCATURER_H_ */
