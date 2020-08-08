#pragma once
#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "SequencerStep.h"
#include "TrackedBody.h"

using namespace std;

class Sequencer {
public:
	Sequencer();
	Sequencer(int x, int y, int elementsPerRow, int elementSize, int padding, ofColor color, ofColor accentColor, ofColor highlightColor);	
	void addSequencerStepForJoint(JointType j);
	void addSequencerStepForJoints(vector<JointType> v);
	void setStepOrder(vector<JointType> order);
	void setTrackedBody(TrackedBody* b);
	void setCurrentHighlight(int highlightedStep);
	void update();
	void draw();
private:
	int x, y;
	int elementsPerRow, elementSize, padding;
	int highlightedStep;
	JointType highlightedJoint;
	ofColor color, accentColor, highlightColor;
	vector<JointType> stepOrder;
	map<JointType, SequencerStep*> steps;
	TrackedBody* trackedBody;

	ofVec2f getPositionForIndex(int index);
	void addSequencerStep(SequencerStep* s);
};
