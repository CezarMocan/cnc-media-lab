#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"

class TrackedJoint {
public:
	TrackedJoint();
	TrackedJoint(TrackedJoint* j);
	TrackedJoint(JointType type);
	void setPosition(ofVec2f pos, float smoothing = 0);
	ofVec2f getPosition();
	ofVec2f getVelocity();
	ofVec2f getTargetPosition();
	float getSpeed();
	
	void update();

private:
	JointType type;
	ofVec2f currentPosition;
	ofVec2f targetPosition;
	ofVec2f velocity;
	float smoothingFactor;
	bool initialized;
	void updateVelocity(ofVec2f oldPosition, ofVec2f newPosition);
};