#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "MaxMSPNetworkManager.h"
#include "PeerNetworkManager.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "Constants.h"
#include "TrackedBody.h"
#include "TrackedBodyShadow.h"
#include "BodySoundPlayer.h"
#include "ofxClipper.h"
#include "GUIManager.h"
#include "Sequencer.h"

class ofApp : public ofBaseApp {

public:
	// oF lifecycle
	void setup();
	void update();
	void draw();
	void drawInterface();
	void keyPressed(int key);

	// Networking
	MaxMSPNetworkManager* maxMSPNetworkManager;
	PeerNetworkManager* peerNetworkManager;

	void peerConnectButtonPressed();

	// Kinect and bodies management

	// // Kinect, detecting body contours
	ofxKFW2::Device kinect;
	ICoordinateMapper* coordinateMapper;
	ofxCv::ContourFinder contourFinder;

	map<int, TrackedBody*> trackedBodies;
	map<int, TrackedBody*> remoteBodies;
	vector<int> trackedBodyIds;

	TrackedBody* getLocalBody();
	int getLocalBodyIndex();

	TrackedBody* getRemoteBody();
	int getRemoteBodyIndex();

	TrackedBody* getLeftBody();
	int getLeftBodyIndex();

	TrackedBody* getRightBody();
	int getRightBodyIndex();

	void detectBodies();
	void computeBodyContours();
	void resolveInstrumentConflicts();

	void updateTrackedBodies();
	void drawTrackedBodies();

	void updateRemoteBodies();
	void drawRemoteBodies();

	//// Body intersections between local and remote
	ofx::Clipper bodiesIntersectionClipper;
	bool bodiesIntersectionActive;
	float bodiesIntersectionStartTimestamp;
	ofPath* bodiesIntersectionPath;

	void updateBodiesIntersection();
	void drawBodiesIntersection();

	//// Body shadows management
	vector<TrackedBodyShadow*> activeBodyShadows;
	vector<pair<int, pair<float, float> > > activeBodyShadowsParams;

	void spawnBodyShadow();
	void playBodyShadow(int index);
	void clearBodyShadow(int index);
	void updateBodyShadows();
	void drawBodyShadows();

	// Visuals, GUI

	//// Final shader pass, for applying grain on top of everything
	ofShader grainShader;
	ofFbo grainFbo;

	GUIManager* appGuiManager;

	// ofMesh frequencyGradient;

	//// Simple panel for setting global parameters
	bool parametersPanelVisible;
	ofxPanel parametersPanel;
	ofParameter<int> bodyContourPolygonFidelity;
	ofParameter<bool> isLeftPlayer;
	ofParameter<bool> automaticShadowsEnabled;

	//// Networking GUI, for connecting with peer
	ofxPanel networkGui;
	ofParameter<string> peerIp;
	ofParameter<string> peerPort;
	ofParameter<string> localPort;
	ofxButton peerConnectButton;

	//// Sequencer GUI (squares at the top of the interface)
	// Sequencer* sequencerLeft;
	// Sequencer* sequencerRight;
	// void updateSequencer();
	// void drawSequencer();

	//// Body contour tracing backgrounds
	// pair<ofPath*, ofRectangle> leftBackgroundContour;
	// pair<ofPath*, ofRectangle> rightBackgroundContour;
	// void updateBackgroundContours();
	// void drawBackgroundContours();

	//// Remaining GUI elements
	// void drawSystemStatus();
	// void drawBodyTrackedStatus();
	// void drawFrequencyGradient();
	// void drawRectangularFrame();

	//// Fonts
	// ofTrueTypeFont fontRegular;
	// ofTrueTypeFont fontBold;
};