/*
 * kinectCapturer.h
 *
 *  Created on: May 14, 2014
 *      Author: José Juan Hernández López
 */

#ifndef KINECTCAPTURER_H_
#define KINECTCAPTURER_H_


#include "Capturer.h"

namespace capture{
using namespace DepthFilter;
class kinectCapturer : public Capturer{
public:
	kinectCapturer();
	kinectCapturer(const char* _filename1, const char* _extension1, const char* _filename2
			, const char* _extension2,  int _start, int _finish, int increment);
	virtual ~kinectCapturer();
	bool initilize(double dt = 0);
	bool saveFeed(const char* str1, const char* str2="",int saveEvery = 1);
	imageState get_frame(Mat & image, double * dt);
	Mat* capture();

	void projectFeatureList(KLT_FeatureList fl);


private:

	void startLiveFeed();
	void stopLiveFeed();
	void livefeedThread();

	bool live_feed;
	int start, finish, increment, current;
	boost::thread feedThread;
	boost::mutex captureMutex;
	bool runLiveFeed;

	DepthFilter::DepthFilter filter;
	VideoCapture capturer;
	Mat colorFrame;
	Mat positionFrame;
};

}//end namespace capture
#endif /* KINECTCAPTURER_H_ */
