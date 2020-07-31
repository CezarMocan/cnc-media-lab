#pragma once

#include "TrackedBody.h"

class TrackedBodyRecording : public TrackedBody {
public:
	TrackedBodyRecording(int index, float smoothingFactor, int contourPoints, int noDelayedContours = 20) : TrackedBody(index, smoothingFactor, contourPoints, noDelayedContours) {
		this->isPlaying = false;
		this->isRecording = false;
		this->playhead = 0;
		this->playDirection = 1;
	};

	int getTrackedBodyIndex();
	void setTrackedBodyIndex(int index);

	bool getIsRecording();
	bool getIsPlaying();

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
	void sendOSCData() override;
private:
	int playhead;
	int playDirection;
	bool isRecording;
	bool isPlaying;
	int trackedBodyIndex;

	vector<map<JointType, TrackedJoint> > recordedJoints;
	vector<ofPolyline> recordedContours;
	vector<ofPolyline> recordedRawContours;
	vector<ofImage> recordedTextures;

};