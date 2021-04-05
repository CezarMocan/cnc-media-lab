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
	void update(TrackedBody* leftBody, TrackedBody* rightBody);
	void draw();

	TrackedBody* leftBody;
	TrackedBody* rightBody;

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

	// Remaining UI components
	void drawSystemStatus();
	void drawBodyTrackedStatus();
	void drawFrequencyGradient();
	void drawRectangularFrame();

	//// Fonts
	ofTrueTypeFont fontRegular;
	ofTrueTypeFont fontBold;
};

