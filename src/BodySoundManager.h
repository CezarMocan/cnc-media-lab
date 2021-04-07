#pragma once

#include "ofMain.h"
#include "MidiNote.h"
#include "MaxMSPNetworkManager.h"
#include <algorithm>

class BodySoundManager {
public:
	BodySoundManager(int index, int canvasWidth, int canvasHeight, vector<MidiNote*> scale);
	void setOscManager(MaxMSPNetworkManager* oscManager);
	void setInterestPoints(vector<pair<JointType, ofVec2f> > points);
	void update();
	void draw();
	void sendOSC(int instrumentId);
	vector<JointType> getCurrentlyPlayingJoints();
	vector<JointType> getCurrentlyPlaying16Joints();
	vector<float> getCurrentlyPlaying16Frequencies();
	MidiNote* pointToMidi(ofVec2f point);

	void start();
	void stop();
	
private:
	int index;
	int canvasWidth, canvasHeight;
	vector<MidiNote*> scale;
	MaxMSPNetworkManager* oscManager;
	void sendMidiSequenceOsc(int instrumentId);
	float getDurationForIndex(int index);

	vector<pair<JointType, ofVec2f> > interestPoints;
	vector<pair<JointType, ofVec2f> > iP;
	vector<JointType> currentlyPlayingJoints;
	vector<JointType> currentlyPlaying16Joints;
	vector<float> currentlyPlaying16Frequencies;

	int currentNoteIndex;
	float currentNoteStartTime;

	float startTime;
	float currentTime;
	
	bool playing;
	int previousSequencerStep;	
};
