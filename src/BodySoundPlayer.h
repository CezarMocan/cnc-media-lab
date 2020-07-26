#pragma once

#include "ofMain.h"
#include "MidiNote.h"
#include "MidiPlayer.h"
#include "ofOSCManager.h"
#include <algorithm>

class BodySoundPlayer {
public:
	BodySoundPlayer(int index, int canvasWidth, int canvasHeight, vector<MidiNote*> scale);
	void setOscManager(ofOSCManager* oscManager);
	void setInterestPoints(vector<ofVec2f> points);
	void setAccentSpeed(float s);
	void setAccentSpeed2(float s);
	void update();
	void draw();
	void sendOSC();
	MidiNote* pointToMidi(ofVec2f point);

	static bool timeBasedComparator(ofVec2f a, ofVec2f b);

	void start();
	void stop();
	
private:
	int index;
	int canvasWidth, canvasHeight;
	vector<MidiNote*> scale;
	ofOSCManager* oscManager;
	void playNextNote();
	void playAccentNote();
	void playAccentNote2();
	void sendMidiSequenceOsc();
	float getDurationForIndex(int index);

	vector<ofVec2f> interestPoints;
	vector<ofVec2f> currentlyPlayingInterestPoints;

	int currentNoteIndex;
	float currentNoteStartTime;

	float startTime;
	float currentTime;

	float accentStartTime;
	float accentSpeed;

	float accentStartTime2;
	float accentSpeed2;
	
	bool playing;

	vector<ofVec2f> iP;
};
