#include "BodySoundPlayer.h"

BodySoundPlayer::BodySoundPlayer(int index, int canvasWidth, int canvasHeight, vector<MidiNote*> scale)
{
	this->index = index;
	this->canvasWidth = canvasWidth;
	this->canvasHeight = canvasHeight;
	this->scale = scale;
	this->oscManager = oscManager;
	this->startTime = ofGetElapsedTimeMillis();
	this->playing = false;
}

void BodySoundPlayer::setOscManager(ofOSCManager* oscManager)
{
	this->oscManager = oscManager;
}

void BodySoundPlayer::setInterestPoints(vector<ofVec2f> points)
{
	this->interestPoints = points;
}

void BodySoundPlayer::setAccentSpeed(float s)
{
	this->accentSpeed = s;
}

void BodySoundPlayer::setAccentSpeed2(float s)
{
	this->accentSpeed2 = s;
}

void BodySoundPlayer::update()
{
	if (!this->playing) return;
	if (this->interestPoints.size() == 0) return;

	this->currentTime = ofGetElapsedTimeMillis();

	float currentDuration = this->getDurationForIndex(this->currentNoteIndex);

	if (this->currentTime - this->accentStartTime > 500 && this->accentSpeed > 1.5) {
		this->playAccentNote();
	}

	if (this->currentTime - this->accentStartTime2 > 2000 && this->accentSpeed2 > 1.5) {
		this->playAccentNote2();
	}

	if (this->currentTime - this->currentNoteStartTime > currentDuration) {
		this->playNextNote();
	} else {

	}	
}

bool BodySoundPlayer::timeBasedComparator(ofVec2f a, ofVec2f b) {
	if (a.x <= b.x) return true;
	return false;
}

void BodySoundPlayer::playNextNote() {
	if (this->interestPoints.size() == 0) return;
	this->currentNoteIndex = (this->currentNoteIndex + 1) % this->interestPoints.size();

	vector<ofVec2f> iP = this->interestPoints;
	sort(iP.begin(), iP.end(), BodySoundPlayer::timeBasedComparator);
	this->currentlyPlayingInterestPoints = iP;

	this->currentNoteStartTime = ofGetElapsedTimeMillis();

	MidiNote* note = this->pointToMidi(this->interestPoints[this->currentNoteIndex]);
	float currentDuration = this->getDurationForIndex(this->currentNoteIndex);

//	MidiPlayer::playNote(note->note, note->octave, currentDuration);
}

void BodySoundPlayer::playAccentNote() {
	if (this->interestPoints.size() == 0) return;

	this->accentStartTime = this->currentTime;
	int accentIndex = (int)floor(ofRandom(this->scale.size() - 1));
	MidiNote* accent = this->scale[accentIndex];

	//MidiNote* note = this->pointToMidi(this->interestPoints[3]);

	//MidiPlayer::playNoteWithInstrument(accent->note, 3 + (int)floor(ofRandom(2)), Instruments::VIOLIN, 500);
}

void BodySoundPlayer::playAccentNote2() {
	if (this->interestPoints.size() == 0) return;

	this->accentStartTime2 = this->currentTime;
	int accentIndex = (int)floor(ofRandom(this->scale.size() - 1));
	MidiNote* accent = this->scale[accentIndex];

	//MidiNote* note = this->pointToMidi(this->interestPoints[3]);

	//MidiPlayer::playNoteWithInstrument(accent->note, 3 + (int)floor(ofRandom(2)), Instruments::SEASHORE, 2000);
}

void BodySoundPlayer::sendOSC(int instrumentId) {
	this->sendMidiSequenceOsc(instrumentId);
}

void BodySoundPlayer::sendMidiSequenceOsc(int instrumentId)
{
	if (this->interestPoints.size() == 0) return;	

	this->iP = this->interestPoints;
	sort(this->iP.begin(), this->iP.end(), BodySoundPlayer::timeBasedComparator);
	
	this->currentlyPlayingInterestPoints = iP;

	// Sequence of raw Y values, normalized to 1024
	vector<int> jointSequenceRaw;
	for (int i = 0; i < iP.size(); i++) {
		const float y = ofMap(iP[i].y, 0, Constants::DEPTH_HEIGHT, 0, 1024);
		jointSequenceRaw.push_back((int)floor(y));
	}

	// Sequence of MIDI messages, mapped on a pattern of 16
	int whichPattern = (int)floor(ofRandom(MappingPatterns::to16[this->iP.size()].size()));
	vector<int> pattern = MappingPatterns::to16[this->iP.size()][whichPattern];
	vector<int> jointSequencePatternMidi;

	for (int i = 0; i < pattern.size(); i++) {
		if (pattern[i] == 0) jointSequencePatternMidi.push_back(0);
		else {
			int index = pattern[i] - 1;
			MidiNote* note = this->pointToMidi(this->iP[index]);
			jointSequencePatternMidi.push_back(note->getCode());
		}
	}	
	
	this->oscManager->sendBodyMidiSequence(instrumentId, jointSequencePatternMidi, jointSequenceRaw);
}


void BodySoundPlayer::draw()
{
	if (!this->playing) return;
	if (this->interestPoints.size() == 0) return;

	ofPushStyle();
	ofPushMatrix();
	//ofScale(2.0);
	ofSetColor(200, 59, 25);
	ofFill();
	if (this->currentNoteIndex < this->currentlyPlayingInterestPoints.size())
		ofDrawCircle(this->currentlyPlayingInterestPoints[this->currentNoteIndex], 5.);
	ofPopStyle();
	ofPopMatrix();
}

void BodySoundPlayer::start()
{
	this->playing = true;
}

void BodySoundPlayer::stop()
{
	this->playing = false;
	this->currentNoteIndex = -1;
	this->currentNoteStartTime = 0;
}


float BodySoundPlayer::getDurationForIndex(int index) {
	vector<int> durations;
	if (ofRandom(2) < 0.3)
		durations = { 700, 1000, 700, 700, 1000 };
	else
		durations = { 700, 700, 700, 1000, 1000 };
	return durations[index % durations.size()] * 1.;
	//return 350 * (floor(ofRandom(2)) + 1);
}

MidiNote* BodySoundPlayer::pointToMidi(ofVec2f point)
{
	const int width = this->canvasWidth;
	const int height = this->canvasHeight;

	int x = (int)point.x;
	int y = (int)point.y;
	float divisionSize = (1.0 * height) / (1.0 * this->scale.size());

	int index = floor(1.0 * y / divisionSize);	
	return this->scale[index % this->scale.size()];	
}
