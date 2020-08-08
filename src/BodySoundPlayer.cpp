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
	this->previousSequencerStep = -1;
	this->currentlyPlayingJoints.clear();
	this->currentlyPlaying16Joints.clear();
}

void BodySoundPlayer::setOscManager(ofOSCManager* oscManager)
{
	this->oscManager = oscManager;
}

void BodySoundPlayer::setInterestPoints(vector<pair<JointType, ofVec2f> > points)
{
	this->interestPoints = points;
}

void BodySoundPlayer::update()
{
	if (!this->playing) return;
	if (this->interestPoints.size() == 0) return;
}

void BodySoundPlayer::sendOSC(int instrumentId) {
	int sequencerStep = this->oscManager->getSequencerStep();
	// Only send joint data to Max on the last frame of the sequencer
	if ((sequencerStep == 0 || sequencerStep == 16) && sequencerStep != this->previousSequencerStep) {
		this->sendMidiSequenceOsc(instrumentId);
	}
	this->previousSequencerStep = sequencerStep;
}

vector<JointType> BodySoundPlayer::getCurrentlyPlayingJoints()
{
	return this->currentlyPlayingJoints;
}

vector<JointType> BodySoundPlayer::getCurrentlyPlaying16Joints()
{
	return this->currentlyPlaying16Joints;
}

void BodySoundPlayer::sendMidiSequenceOsc(int instrumentId)
{
	if (this->interestPoints.size() == 0) return;

	JointType lastPlayingJoint = JointType_HandTipLeft;
	if (this->currentlyPlayingJoints.size() != 0)
		lastPlayingJoint = this->currentlyPlayingJoints[this->currentlyPlayingJoints.size() - 1];
	this->currentlyPlayingJoints.clear();
	this->currentlyPlaying16Joints.clear();

	this->iP = this->interestPoints;

	// Sequence of raw Y values, normalized to 1024
	vector<int> jointSequenceRaw;
	for (int i = 0; i < iP.size(); i++) {
		const float y = ofMap(iP[i].second.y, 0, Constants::DEPTH_HEIGHT, 0, 1024);
		jointSequenceRaw.push_back((int)floor(y));
		currentlyPlayingJoints.push_back(this->iP[i].first);
	}

	// Sequence of MIDI messages, mapped on a pattern of 16
	int whichPattern = (int)floor(ofRandom(MappingPatterns::to16[this->iP.size()].size()));
	vector<int> pattern = MappingPatterns::to16[this->iP.size()][whichPattern];
	vector<int> jointSequencePatternMidi;

	JointType prevJoint = lastPlayingJoint;
	for (int i = 0; i < pattern.size(); i++) {
		if (pattern[i] == 0) {
			jointSequencePatternMidi.push_back(0);
			this->currentlyPlaying16Joints.push_back(prevJoint);
		}
		else {
			int index = pattern[i] - 1;
			MidiNote* note = this->pointToMidi(this->iP[index].second);
			jointSequencePatternMidi.push_back(note->getCode());
			prevJoint = this->iP[index].first;
			this->currentlyPlaying16Joints.push_back(prevJoint);
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
	if (this->currentNoteIndex < this->iP.size())
		ofDrawCircle(this->iP[this->currentNoteIndex].second, 5.);
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
