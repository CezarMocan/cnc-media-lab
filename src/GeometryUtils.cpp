#include "GeometryUtils.h"

float GeometryUtils::getVectorAngle(ofVec2f a, ofVec2f b)
{
	return atan2((b - a).y, (b - a).x);
}

float GeometryUtils::getVectorAngleDeg(ofVec2f a, ofVec2f b)
{
	return getVectorAngle(a, b) * 180 / PI;
}

float GeometryUtils::getPolylineSquaredDistanceWithOffset(ofPolyline a, ofPolyline b, int offset)
{
	auto aV = a.getVertices();
	auto bV = b.getVertices();
	float distance = 0;
	for (int i = 0; i < aV.size(); i++) {
		int bIndex = (i + offset) % bV.size();
		distance += ofVec2f(aV[i]).squareDistance(ofVec2f(bV[bIndex]));
	}
	return distance;
}
