#pragma once

#include "TrackedBody.h"

class TrackedBodyRecording : public TrackedBody {
public:
	TrackedBodyRecording(int index, float smoothingFactor, int contourPoints = 150) : TrackedBody(index, smoothingFactor, contourPoints) {
		this->isPlaying = false;
		this->isRecording = false;
		this->playhead = 0;
	};


	void startRecording();
	void stopRecording();
	void startPlayOnce();
	void startPlayLoop();
	void stopPlay();

	void update() override;
	void draw() override;
	void updateSkeletonData(map<JointType, ofxKinectForWindows2::Data::Joint> joints, ICoordinateMapper* coordinateMapper) override;
	void updateContourData(vector<ofPolyline> contours) override;
	void updateTextureData(ofImage texture) override;
private:
	int playhead;
	bool isRecording;
	bool isPlaying;

	vector<map<JointType, TrackedJoint*> > recordedJoints;
	vector<ofPolyline> recordedContours;	
	vector<ofImage> recordedTextures;

};