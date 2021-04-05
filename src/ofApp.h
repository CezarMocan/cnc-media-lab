#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "MaxMSPNetworkManager.h"
#include "PeerNetworkManager.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "Constants.h"
#include "TrackedBody.h"
#include "TrackedBodyRecording.h"
#include "BodySoundPlayer.h"
#include "ofxClipper.h"
#include "Sequencer.h"

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void updateSequencer();
	void drawSequencer();

	ofPath* intersectionPath;
	void updateIntersection();
	void drawIntersection();

	pair<ofPath*, ofRectangle> leftCtr;
	pair<ofPath*, ofRectangle> rightCtr;
	void updateBackgrounds();
	void drawBackgrounds();
	
	void drawTrackedBodies();
	void drawRemoteBodies();
	void drawTrackedBodyRecordings();

	void drawSystemStatus();
	void drawBodyTrackedStatus();
	void drawFrequencyGradient();
	void drawFrame();

	void drawInterface();

	void resolveInstrumentConflicts();

	void detectBodySkeletons();
	void detectBodyContours();


	TrackedBody* getLocalBody();
	TrackedBody* getRemoteBody();
	TrackedBody* getLeftBody();
	TrackedBody* getRightBody();
	int getLocalBodyIndex();
	int getRemoteBodyIndex();
	int getLeftBodyIndex();
	int getRightBodyIndex();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofxKFW2::Device kinect;
	ICoordinateMapper* coordinateMapper;

	MaxMSPNetworkManager* oscSoundManager;
	map<int, TrackedBody*> trackedBodies;
	map<int, TrackedBody*> remoteBodies;
	vector<int> trackedBodyIds;

	ofShader bodyIndexShader;
	ofShader grainShader;
	ofFbo grainFbo;

	ofFbo bodyFbo;
	ofFbo bodyDebugFbo;
	ofPixels bodyPixels;
	ofImage bodyImage;

	ofMesh frequencyGradient;

	ofxCv::ContourFinder contourFinder;

	float headX = 0, headY = 0;
	int currView = 0;


	bool guiVisible;
	ofxPanel gui;
	ofParameter<int> radius;
	ofParameter<int> polygonFidelity;
	
	ofParameter<bool> isLeft;

	ofParameter<bool> automaticShadows;

	ofxPanel networkGui;	
	ofParameter<string> peerIp;
	ofParameter<string> peerPort;
	ofParameter<string> localPort;
	ofxButton peerConnectButton;
	void peerConnectButtonPressed();
	PeerNetworkManager* networkManager;

	ofx::Clipper clipper;

	bool remoteIntersectionActive;
	float remoteIntersectionStartTimestamp;
	Sequencer* sequencerLeft;
	Sequencer* sequencerRight;

	void spawnBodyRecording();
	void playBodyRecording(int index);
	void clearBodyRecording(int index);
	void manageBodyShadows();

	vector<TrackedBodyRecording*> activeBodyRecordings;
	vector<pair<int, pair<float, float> > > activeBodyRecordingsParams;

	ofTrueTypeFont fontRegular;
	ofTrueTypeFont fontBold;
};