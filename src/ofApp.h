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

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();
	void drawDebug();
	void drawAlternate();
	void drawTrackedBodies(int drawMode);
	void drawRemoteBodies(int drawMode);
	void drawTrackedBodyRecordings(int drawMode);
	void drawVoronoi();

	void detectBodySkeletons();
	void detectBodyContours();

	TrackedBodyRecording* createBodyRecording(int bodyId);
	vector<TrackedBodyRecording*> activeBodyRecordings;

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
	map<int, TrackedBodyRecording*> trackedBodyRecordings;
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

	ofxPanel gui;
	ofParameter<int> radius;
	ofParameter<int> polygonFidelity;
	ofParameter<bool> useGaussian;
	ofParameter<bool> useBlur;
	ofParameter<bool> useErode;
	ofParameter<bool> useDilate;

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
	ofParameter<bool> isServer;
	ofParameter<string> serverIp;
	ofParameter<string> serverPort;
	ofxButton serverConnectButton;
	void serverConnectButtonPressed();
	NetworkManager* networkManager;

};