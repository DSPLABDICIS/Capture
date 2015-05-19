/*
 * Capturer.h
 *
 *  Created on: Jun 16, 2014
 *      Author: dsplab
 */

#ifndef CAPTURER_H_
#define CAPTURER_H_

#include "cv.h"
#include "highgui.h"
#include "boost/thread.hpp"
#include "boost/timer.hpp"
#include "klt.h"
#include "DepthFilter/DepthFilter.hpp"

namespace capture {

using namespace std;
using namespace cv;

enum imageState{
	CAPTURE_NOT_FOUND = -1,
	CAPTURE_SUCESSFUL,
	CAPTURE_END_OF_CAPTURE
};

class Capturer {
public:
	Capturer();
	virtual ~Capturer();

	virtual bool initilize(double dt = 0) = 0;                       //virtual function
	virtual bool saveFeed(const char* str1, const char* str2="",int saveEvery = 1);
	virtual imageState get_frame(Mat & image, double * dt) = 0;		//virtual function
	virtual void projectFeatureList(KLT_FeatureList fl) = 0;

protected:
	bool loadMat(const char* name);
	bool saveMat(cv::Mat& matrix, const char* name, double dt);

	bool save;
	int saveEvery;
	int saveCounter;
	bool nextFrameReady;
	string saveS1, saveS2;
	string loadS1, loadS2;
	string ext1, ext2;
	boost::posix_time::ptime timeFeedPrevious, timeCurrent;    					//Livefeed
	double deltaCaptureTime;
	boost::posix_time::ptime timeCapturePrevious;				//Save Feed

public:
	Mat colorFrameStored;
	Mat positionFrameStored;
	double deltaTimeStored;

};

} /* namespace capture */

#endif /* CAPTURER_H_ */
