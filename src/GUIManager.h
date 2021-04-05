#pragma once

#include "ofMain.h"
#include "Sequencer.h"
#include "TrackedBody.h"
#include "Constants.h"
#include "ofxKinectForWindows2.h"

using namespace std;

class GUIManager
{
public:
	GUIManager();
	void update(TrackedBody* leftBody, TrackedBody* rightBody, int currentSequencerStep, bool isConnected, string latency);

	TrackedBody* leftBody;
	TrackedBody* rightBody;
	int currentSequencerStep;
	bool isConnected;
	string latency;

	// UI component at the bottom, for drawing current sound frequency
	ofMesh frequencyGradient;

	// Sequencer UI component (squares at the top of the interface)
	Sequencer* sequencerLeft;
	Sequencer* sequencerRight;
	void updateSequencer();
	void drawSequencer();

	// Body contour tracing backgrounds
	pair<ofPath*, ofRectangle> leftBackgroundContour;
	pair<ofPath*, ofRectangle> rightBackgroundContour;
	void updateBackgroundContours();
	void drawBackgroundContours();

	// System status (connection indicator, latency, IPs & so on.)
	void drawSystemStatus();

	// Are the bodies currently tracked or not
	void drawBodyTrackedStatus();

	// Gradient at the bottom, showing frequency of current sound
	void drawFrequencyGradient();

	// Rectangle frame around whole composition
	void drawRectangularFrame();

	//// Fonts
	ofTrueTypeFont fontRegular;
	ofTrueTypeFont fontBold;
};

