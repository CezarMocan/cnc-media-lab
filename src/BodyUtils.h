#pragma once
#include "ofMain.h"

class BodyUtils {
public:
	static float getVectorAngle(ofVec2f a, ofVec2f b);
	static float getVectorAngleDeg(ofVec2f a, ofVec2f b);
	static float getPolylineSquaredDistanceWithOffset(ofPolyline a, ofPolyline b, int offset);
};