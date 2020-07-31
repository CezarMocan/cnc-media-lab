#include "TrackedBodyRecording.h"

int TrackedBodyRecording::getTrackedBodyIndex()
{
	return this->index - Constants::BODY_RECORDINGS_ID_OFFSET;
}

bool TrackedBodyRecording::getIsRecording()
{
	return this->isRecording;
}

bool TrackedBodyRecording::getIsPlaying()
{
	return this->isPlaying;
}

void TrackedBodyRecording::startRecording()
{
	this->isRecording = true;
	this->isPlaying = false;

	this->recordedJoints.clear();
	this->recordedContours.clear();
	this->recordedRawContours.clear();
	this->recordedTextures.clear();
}

void TrackedBodyRecording::stopRecording()
{
	this->isRecording = false;
}

void TrackedBodyRecording::startPlayOnce()
{
	this->startPlayLoop();
}

void TrackedBodyRecording::startPlayLoop()
{
	this->isRecording = false;
	this->isPlaying = true;
	this->playhead = 0;
}

void TrackedBodyRecording::stopPlay()
{
	this->isPlaying = false;
}

void TrackedBodyRecording::update()
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

		// Record raster
		//this->recordedTextures.push_back(this->texture);
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
			this->joints[it->first] = &(it->second);//new TrackedJoint(it->second);
		}

		//this->joints = this->recordedJoints[this->playhead];
		//this->texture = this->recordedTextures[this->playhead];
				
		this->bodySoundPlayer->setInterestPoints(this->getInterestPoints());
		this->bodySoundPlayer->setAccentSpeed(this->getJointSpeed(JointType_WristRight));
		this->bodySoundPlayer->setAccentSpeed2(this->getJointSpeed(JointType_WristLeft));
		this->bodySoundPlayer->update();
				
	}
}

void TrackedBodyRecording::draw()
{
	if (this->isPlaying) {
		if (this->recordedContours.size() == 0) return;
		TrackedBody::draw();
	}
}

void TrackedBodyRecording::updateSkeletonData(map<JointType, ofxKinectForWindows2::Data::Joint> joints, ICoordinateMapper* coordinateMapper)
{
	if (this->isRecording) TrackedBody::updateSkeletonData(joints, coordinateMapper);
}

void TrackedBodyRecording::updateContourData(vector<ofPolyline> contours)
{
	if (this->isRecording) TrackedBody::updateContourData(contours);
}

void TrackedBodyRecording::updateTextureData(ofImage texture)
{
	if (this->isRecording) TrackedBody::updateTextureData(texture);
}

void TrackedBodyRecording::sendOSCData()
{
	if (this->isPlaying) TrackedBody::sendOSCData();
}
