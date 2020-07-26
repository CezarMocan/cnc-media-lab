#pragma once
#include "ofxKinectForWindows2.h"
#include "ofMain.h"
#include "ofxBlur.h"
#include "ofxCv.h"
#include "TrackedJoint.h"
#include "BodyUtils.h"
#include "Constants.h"
#include "ofOSCManager.h"
#include "ofxVoronoi.h"
#include "BodySoundPlayer.h"

using namespace std;
using namespace Constants;

enum BodyDrawMode {
	BDRAW_MODE_NONE = 0b0000000000,
	BDRAW_MODE_MOVEMENT = 0b0000000001,
	BDRAW_MODE_JOINTS = 0b0000000010,
	BDRAW_MODE_CONTOUR = 0b0000000100,
	BDRAW_MODE_RASTER = 0b0000001000,
	BDRAW_MODE_SOUND = 0b0000010000
};

class TrackedBody {
public:
	TrackedBody(int index, float smoothingFactor, int contourPoints = 150);
	void setOSCManager(ofOSCManager* m);
	void setBodySoundPlayer(BodySoundPlayer* bsp);
	void setTracked(bool isTracked);
	void setContourPoints(int contourPoints);
	virtual void updateSkeletonData(map<JointType, ofxKinectForWindows2::Data::Joint> joints, ICoordinateMapper* coordinateMapper);
	virtual void updateContourData(vector<ofPolyline> contours);
	virtual void updateTextureData(ofImage texture);
	void updateSkeletonContourDataFromSerialized(string s);
	void setDrawMode(int drawMode);

	float getJointsDistance(JointType a, JointType b);
	float getNormalizedJointsDistance(JointType a, JointType b);
	float getNormalizedArea();
	float getJointsAngle(JointType a, JointType b, JointType c);
	float getJointSpeed(JointType a);
	float getJointNormalizedSpeed(JointType a);

	ofPolyline getVoronoiPolyline(int bodyInsideCells, bool forceCellsInsideBody);

	vector<ofVec2f> getInterestPoints();
	
	virtual void update();
	void drawJointLine(JointType a, JointType b);
	void drawJointPolygon(vector<JointType> v);
	void drawJointArc(JointType a, JointType b, JointType c);
	void drawMovement();
	void drawJoints();
	void drawContours();
	void drawSoundPlayer();
	void drawRaster();

	virtual void draw();

	string serialize();	

	void sendOSCData();

protected:
	int index;
	int drawMode;
	float smoothingFactor;
	int contourPoints;
	int noContours;
	bool isTracked;
	ofOSCManager* oscManager;	

	BodySoundPlayer* bodySoundPlayer;

	map<JointType, ofxKinectForWindows2::Data::Joint> latestSkeleton;
	ICoordinateMapper* coordinateMapper;

	map<JointType, TrackedJoint*> joints;

	ofPolyline rawContour;
	ofPolyline contour;
	vector<ofPolyline> delayedContours;
	ofImage texture;

	vector < pair<pair<int, int>, float> > voronoiPoints;

	ofFbo mainFbo;
	ofxBlur blurShader;
	ofShader speedShader;

	map<JointType, float> JOINT_WEIGHTS;

	void updateJointPosition(JointType joint, ofVec2f position);
	void setJointUniform(JointType joint, string uniformName, ofShader shader);	
};