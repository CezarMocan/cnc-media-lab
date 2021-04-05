#include "TrackedBodyShadow.h"

int TrackedBodyShadow::getTrackedBodyIndex()
{
	return this->trackedBodyIndex;
}

void TrackedBodyShadow::setTrackedBodyIndex(int index)
{
	this->trackedBodyIndex = index;
}

bool TrackedBodyShadow::getIsRecording()
{
	return this->isRecording;
}

bool TrackedBodyShadow::getIsPlaying()
{
	return this->isPlaying;
}

void TrackedBodyShadow::startRecording()
{
	this->isRecording = true;
	this->isPlaying = false;

	this->recordedJoints.clear();
	this->recordedContours.clear();
	this->recordedRawContours.clear();
	this->recordedTextures.clear();
}

void TrackedBodyShadow::stopRecording()
{
	this->isRecording = false;
}

void TrackedBodyShadow::startPlayOnce()
{
	this->startPlayLoop();
}

void TrackedBodyShadow::startPlayLoop()
{
	this->isRecording = false;
	this->isPlaying = true;
	this->playhead = 0;
}

void TrackedBodyShadow::stopPlay()
{
	this->isPlaying = false;
}

void TrackedBodyShadow::update()
{
	if (isRecording) {
		TrackedBody::update();
		if (this->contour.size() < 5) return;
		// Record contour
		this->recordedContours.push_back(ofPolyline(this->contour));
		this->recordedRawContours.push_back(ofPolyline(this->rawContour));

		// Record joints
		map<JointType, TrackedJoint> newJoints;
		for (auto it = this->joints.begin(); it != this->joints.end(); ++it) {
			newJoints[it->first] = TrackedJoint(it->second);
		}
		this->recordedJoints.push_back(newJoints);
	}
	else if (isPlaying) {
		if (this->recordedContours.size() == 0) return;
		this->playhead += this->playDirection;
		if (this->playhead == this->recordedContours.size() - 1 || this->playhead == 0)
			this->playDirection *= -1;		

		this->rawContour = this->recordedRawContours[this->playhead];
		this->contour = this->recordedContours[this->playhead];
		
		this->joints.clear();
		for (auto it = this->recordedJoints[this->playhead].begin(); it != this->recordedJoints[this->playhead].end(); ++it) {
			this->joints[it->first] = &(it->second);
		}
				
		this->bodySoundPlayer->setInterestPoints(this->getInterestPoints());
		this->bodySoundPlayer->update();
	}
}

void TrackedBodyShadow::draw()
{
	if (this->isPlaying) {
		if (this->recordedContours.size() == 0) return;
		TrackedBody::draw();
	}
}

void TrackedBodyShadow::updateSkeletonData(map<JointType, ofxKinectForWindows2::Data::Joint> joints, ICoordinateMapper* coordinateMapper)
{
	if (this->isRecording) TrackedBody::updateSkeletonData(joints, coordinateMapper);
}

void TrackedBodyShadow::updateContourData(vector<ofPolyline> contours)
{
	if (this->isRecording) TrackedBody::updateContourData(contours);
}

void TrackedBodyShadow::sendDataToMaxMSP()
{
	if (this->isPlaying) TrackedBody::sendDataToMaxMSP();
}
