#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofOSCManager.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "Constants.h"
#include "TrackedBody.h"
#include "TrackedBodyRecording.h"
#include "MidiPlayer.h"
#include "BodySoundPlayer.h"
#include "NetworkManager.h"
#include "ofxClipper.h"
#include "Sequencer.h"

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();
	void drawDebug();
	void drawSequencer();
	void drawIntersection();
	void drawBackgrounds();
	void drawAlternate();
	void drawTrackedBodies(int drawMode);
	void drawRemoteBodies(int drawMode);
	void drawTrackedBodyRecordings(int drawMode);
	void resolveInstrumentConflicts();
	void drawVoronoi();

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

	bool isBorder(ofDefaultVec3 _pt);

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

	ofImage bodyIndexImg, foregroundImg;
	vector<ofVec2f> colorCoords;
	int numBodiesTracked;
	bool bHaveAllStreams;

	ofOSCManager* oscSoundManager;
	map<int, TrackedBody*> trackedBodies;
	map<int, TrackedBody*> remoteBodies;
	vector<int> trackedBodyIds;

	ofxVoronoi voronoi;
	ofPolyline voronoiRandomPoints;

	ofShader bodyIndexShader;
	ofFbo bodyFbo;
	ofFbo bodyDebugFbo;
	ofPixels bodyPixels;
	ofImage bodyImage;

	ofxCv::ContourFinder contourFinder;

	float headX = 0, headY = 0;
	int currView = 0;


	bool guiVisible;
	ofxPanel gui;
	ofParameter<int> radius;
	ofParameter<int> polygonFidelity;
	
	ofParameter<bool> isLeft;

	ofParameter<bool> automaticShadows;

	ofParameter<bool> localBodyDrawsGeometry;
	ofParameter<bool> localBodyDrawsJoints;
	ofParameter<bool> localBodyDrawsContour;
	ofParameter<bool> localBodyDrawsFill;
	ofParameter<bool> localBodyDrawsHLines;
	ofParameter<bool> localBodyDrawsVLines;
	ofParameter<bool> localBodyDrawsDots;
	ofParameter<bool> localBodyDrawsGrid;	

	ofParameter<bool> remoteBodyDrawsGeometry;
	ofParameter<bool> remoteBodyDrawsJoints;	
	ofParameter<bool> remoteBodyDrawsContour;
	ofParameter<bool> remoteBodyDrawsFill;
	ofParameter<bool> remoteBodyDrawsHLines;
	ofParameter<bool> remoteBodyDrawsVLines;
	ofParameter<bool> remoteBodyDrawsDots;
	ofParameter<bool> remoteBodyDrawsGrid;

	ofParameter<bool> recordedBodyDrawsGeometry;
	ofParameter<bool> recordedBodyDrawsJoints;
	ofParameter<bool> recordedBodyDrawsContour;
	ofParameter<bool> recordedBodyDrawsFill;
	ofParameter<bool> recordedBodyDrawsHLines;
	ofParameter<bool> recordedBodyDrawsVLines;
	ofParameter<bool> recordedBodyDrawsDots;
	ofParameter<bool> recordedBodyDrawsGrid;


	ofParameter<bool> useGaussian;
	ofParameter<bool> useBlur;
	ofParameter<bool> useErode;
	ofParameter<bool> useDilate;

	ofxPanel voronoiGui;
	ofParameter<int> voronoiEnvironmentCells;
	ofParameter<int> voronoiBodyCells;
	ofParameter<bool> voronoiForceCellsInsideBody;
	ofParameter<bool> voronoiFillMode;
	ofParameter<bool> voronoiDrawCellCenters;	
	ofParameter<int> voronoiBackgroundHue;
	ofParameter<int> voronoiBodyHue;
	ofParameter<float> voronoiSmoothing;
	ofParameter<float> voronoiEnvironmentNoise;
	ofParameter<bool> voronoiConnectCellCenters;

	ofxPanel networkGui;	
	ofParameter<string> peerIp;
	ofParameter<string> peerPort;
	ofParameter<string> localPort;
	ofxButton peerConnectButton;
	void peerConnectButtonPressed();
	NetworkManager* networkManager;

	ofx::Clipper clipper;

	bool remoteIntersectionActive;
	float remoteIntersectionStartTimestamp;
	Sequencer* sequencerLeft;
	Sequencer* sequencerRight;

	void spawnBodyRecording();
	void playBodyRecording(int index);
	void clearBodyRecording(int index);
	void manageBodyRecordings();

	vector<TrackedBodyRecording*> activeBodyRecordings;
	vector<pair<int, pair<float, float> > > activeBodyRecordingsParams;
};