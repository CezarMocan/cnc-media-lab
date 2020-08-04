#pragma once
#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxClipper.h"
#include "TrackedBody.h"

using namespace std;

class BodyCapture {
public:
	BodyCapture() {
		this->body = NULL;
		this->joint = JointType_Head;
		this->strokeColor = ofColor(0, 0, 0, 0);
		this->fillColor = ofColor(0, 0, 0, 0);
	};
	BodyCapture(TrackedBody* body, JointType joint, ofColor strokeColor, ofColor fillColor) {
		this->body = body;
		this->joint = joint;
		this->strokeColor = strokeColor;
		this->fillColor = fillColor;
	};
	TrackedBody* body;
	JointType joint;
	ofColor strokeColor;
	ofColor fillColor;
};

class SequencerStep {
public:
	SequencerStep();
	SequencerStep(float x, float y, float size, JointType joint, ofColor strokeColor, ofColor highlightColor);
	void registerBody(TrackedBody* body, ofColor strokeColor, ofColor fillColor);
	void update();
	void draw(bool isHighlighted = false);
private:
	float x, y, size;
	float clipSize;
	JointType joint;
	ofColor strokeColor;
	ofColor highlightColor;
	vector<BodyCapture> bodies;
	ofPath currentPath;
	vector<ofPath> paths;
	ofx::Clipper clipper;
};