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

#ifndef TRACKED_BODY_H
#define TRACKED_BODY_H

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
	static void initialize();

	TrackedBody(int index, float smoothingFactor, int contourPoints = 150, int noDelayedContours = 20);
	void setOSCManager(ofOSCManager* m);
	void setBodySoundPlayer(BodySoundPlayer* bsp);
	void setTracked(bool isTracked);
	void setContourPoints(int contourPoints);

	void setIsRecording(bool isRecording);
	bool getIsRecording();

	virtual void updateSkeletonData(map<JointType, ofxKinectForWindows2::Data::Joint> joints, ICoordinateMapper* coordinateMapper);
	virtual void updateContourData(vector<ofPolyline> contours);
	virtual void updateTextureData(ofImage texture);
	void updateDelayedContours();
	void updateSkeletonContourDataFromSerialized(string s);
	void setDrawMode(int drawMode);

	float getJointsDistance(JointType a, JointType b);
	float getNormalizedJointsDistance(JointType a, JointType b);
	float getNormalizedArea();
	float getJointsAngle(JointType a, JointType b, JointType c);
	float getJointSpeed(JointType a);
	float getJointNormalizedSpeed(JointType a);
	ofVec2f getJointPosition(JointType a);
	float getScreenRatio();

	ofPolyline getVoronoiPolyline(int bodyInsideCells, bool forceCellsInsideBody);

	vector<pair<JointType, ofVec2f> > getInterestPoints();
	
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

	void assignInstrument();
	void reassignInstrument();
	void removeInstrument();
	void assignInstrument(int instrumentId);
	int getInstrumentId();

	static int instruments[Constants::MAX_INSTRUMENTS];
	static int getFirstFreeInstrument();
	static void acquireInstrument(int instrumentId);
	static void releaseInstrument(int instrumentId);

	string serialize();	

	virtual void sendOSCData();

	map<JointType, ofxKinectForWindows2::Data::Joint> latestSkeleton;
	ICoordinateMapper* coordinateMapper;
	ofPolyline rawContour;
	ofPolyline contour;
	ofImage texture;

	int index;

	static bool interestPointComparator(pair<JointType, ofVec2f> a, pair<JointType, ofVec2f> b);
	vector<JointType> getCurrentlyPlayingJoints();
	vector<JointType> getCurrentlyPlaying16Joints();

protected:	
	int instrumentId;
	int drawMode;
	float smoothingFactor;
	int contourPoints;
	int noContours;
	bool isTracked;
	int contourIndexOffset;
	bool isRecording;
	ofOSCManager* oscManager;	

	BodySoundPlayer* bodySoundPlayer;

	map<JointType, TrackedJoint*> joints;
		
	ofPath contourPath;
	vector<ofPolyline> delayedContours;	

	vector < pair<pair<int, int>, float> > voronoiPoints;

	ofFbo mainFbo;
	ofFbo polyFbo;
	ofxBlur blurShader;
	ofShader speedShader;

	map<JointType, float> JOINT_WEIGHTS;

	void updateJointPosition(JointType joint, ofVec2f position);
	void setJointUniform(JointType joint, string uniformName, ofShader shader);	
};

#endif