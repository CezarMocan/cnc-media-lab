#pragma once

#include "ofMain.h"
#include "Sequencer.h"
#include "TrackedBody.h"
#include "TrackedBodyShadow.h"
#include "Constants.h"
#include "ofxKinectForWindows2.h"
#include "MaxMSPNetworkManager.h"
#include "PeerNetworkManager.h"

using namespace std;

class BodiesManager
{
public:
	BodiesManager();
	void setNetworkManagers(PeerNetworkManager* peerNetworkManager, MaxMSPNetworkManager* maxMSPNetworkManager);
	void setIsLeftPlayer(bool isLeftPlayer);
	void setAutomaticShadowsEnabled(bool automaticShadowsEnabled);
	void setBodyContourPolygonFidelity(int bodyContourPolygonFidelity);

	void update();

	TrackedBody* getLocalBody();
	int getLocalBodyIndex();

	TrackedBody* getRemoteBody();
	int getRemoteBodyIndex();

	TrackedBody* getLeftBody();
	int getLeftBodyIndex();

	TrackedBody* getRightBody();
	int getRightBodyIndex();

	void drawTrackedBodies();	
	void drawRemoteBodies();	
	void drawBodiesIntersection();
	void drawBodyShadows();

	void spawnBodyShadow();
	void playBodyShadow(int index);
	void clearBodyShadow(int index);

private:
	// App state
	bool isLeftPlayer;
	bool automaticShadowsEnabled;
	int bodyContourPolygonFidelity;

	// Network managers
	MaxMSPNetworkManager* maxMSPNetworkManager;
	PeerNetworkManager* peerNetworkManager;

	// Updates at every frame
	void detectBodies();
	void computeBodyContours();
	void updateTrackedBodies();
	void updateBodyShadows();
	void updateRemoteBodies();
	void updateBodiesIntersection();
	void resolveInstrumentConflicts();

	// // Kinect, detecting body contours
	ofxKFW2::Device kinect;
	ICoordinateMapper* coordinateMapper;
	void initKinect();

	ofxCv::ContourFinder contourFinder;

	map<int, TrackedBody*> trackedBodies;
	map<int, TrackedBody*> remoteBodies;
	vector<int> trackedBodyIds;

	//// Body intersections between local and remote
	ofx::Clipper bodiesIntersectionClipper;
	bool bodiesIntersectionActive;
	float bodiesIntersectionStartTimestamp;
	ofPath* bodiesIntersectionPath;

	//// Body shadows management
	vector<TrackedBodyShadow*> activeBodyShadows;
	vector<pair<int, pair<float, float> > > activeBodyShadowsParams;
};

