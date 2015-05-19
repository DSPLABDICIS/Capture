/*
 * MonoCamera.h
 *
 *  Created on: Sep 11, 2014
 *      Author: crono21
 */

#ifndef MONOCAMERA_H_
#define MONOCAMERA_H_

#include "Capturer.h"

namespace capture {

class MonoCamera: public Capturer {
public:
	MonoCamera();
	MonoCamera(const char* _filename1, const char* _extension1,
			int _start, int _finish, int _increment);
	virtual ~MonoCamera();

	virtual bool initilize(double dt = 0);
	virtual imageState get_frame(Mat & image, double * dt);
	virtual void projectFeatureList(KLT_FeatureList fl);


private:
	bool live_feed;
	int start, finish, increment, current;
	bool runLiveFeed;


};

} /* namespace capture */

#endif /* MONOCAMERA_H_ */
