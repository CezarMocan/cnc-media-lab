#include "BodySoundManager.h"

BodySoundManager::BodySoundManager(int index, int canvasWidth, int canvasHeight, vector<MidiNote*> scale)
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
	this->currentlyPlaying16Frequencies.clear();
}

void BodySoundManager::setOscManager(MaxMSPNetworkManager* oscManager)
{
	this->oscManager = oscManager;
}

void BodySoundManager::setInterestPoints(vector<pair<JointType, ofVec2f> > points)
{
	this->interestPoints = points;
}

void BodySoundManager::update()
{
	if (!this->playing) return;
	if (this->interestPoints.size() == 0) return;
}

void BodySoundManager::sendOSC(int instrumentId) {
	int sequencerStep = this->oscManager->getSequencerStep();
	// Only send joint data to Max on the last frame of the sequencer
	if ((sequencerStep == 0 || sequencerStep == 16) && sequencerStep != this->previousSequencerStep) {
		this->sendMidiSequenceOsc(instrumentId);
	}
	this->previousSequencerStep = sequencerStep;
}

vector<JointType> BodySoundManager::getCurrentlyPlayingJoints()
{
	return this->currentlyPlayingJoints;
}

vector<JointType> BodySoundManager::getCurrentlyPlaying16Joints()
{
	return this->currentlyPlaying16Joints;
}

vector<float> BodySoundManager::getCurrentlyPlaying16Frequencies()
{
	return this->currentlyPlaying16Frequencies;
}

void BodySoundManager::sendMidiSequenceOsc(int instrumentId)
{
	if (this->interestPoints.size() == 0) return;

	JointType lastPlayingJoint = JointType_HandTipLeft;	
	if (this->currentlyPlayingJoints.size() != 0) {
		lastPlayingJoint = this->currentlyPlayingJoints[this->currentlyPlayingJoints.size() - 1];		
	}

	float lastPlayingFrequency = 0;
	if (this->currentlyPlaying16Frequencies.size() != 0) {
		lastPlayingFrequency = this->currentlyPlaying16Frequencies[this->currentlyPlaying16Frequencies.size() - 1];
	}

	this->currentlyPlayingJoints.clear();
	this->currentlyPlaying16Joints.clear();
	this->currentlyPlaying16Frequencies.clear();

	this->iP = this->interestPoints;

	// Sequence of raw Y values, normalized to 1024
	vector<int> jointSequenceRaw;
	for (int i = 0; i < iP.size(); i++) {
		const float y = ofMap(iP[i].second.y, 0, Constants::DEPTH_HEIGHT, 0, 1024);
		const float x = ofMap(iP[i].second.x, 0, Constants::DEPTH_WIDTH, 0, 1024);
		jointSequenceRaw.push_back((int)floor(x));
		currentlyPlayingJoints.push_back(this->iP[i].first);
	}

	// Sequence of MIDI messages, mapped on a pattern of 16
	int whichPattern = (int)floor(ofRandom(MappingPatterns::to16[this->iP.size()].size()));
	vector<int> pattern = MappingPatterns::to16[this->iP.size()][whichPattern];
	vector<int> jointSequencePatternMidi;

	JointType prevJoint = lastPlayingJoint;
	float prevFrequency = lastPlayingFrequency;
	for (int i = 0; i < pattern.size(); i++) {
		if (pattern[i] == 0) {
			jointSequencePatternMidi.push_back(0);
			this->currentlyPlaying16Joints.push_back(prevJoint);
			this->currentlyPlaying16Frequencies.push_back(prevFrequency);
		}
		else {
			int index = pattern[i] - 1;
			MidiNote* note = this->pointToMidi(this->iP[index].second);
			jointSequencePatternMidi.push_back(note->getCode());
			prevJoint = this->iP[index].first;
			prevFrequency = note->getFrequency();
			this->currentlyPlaying16Joints.push_back(prevJoint);
			this->currentlyPlaying16Frequencies.push_back(prevFrequency);
		}
	}	
	
	this->oscManager->sendBodyMidiSequence(instrumentId, jointSequencePatternMidi, jointSequenceRaw);
}


void BodySoundManager::draw()
{
	if (!this->playing) return;
	if (this->interestPoints.size() == 0) return;

	ofPushStyle();
	ofPushMatrix();
	ofSetColor(200, 59, 25);
	ofFill();
	if (this->currentNoteIndex < this->iP.size())
		ofDrawCircle(this->iP[this->currentNoteIndex].second, 5.);
	ofPopStyle();
	ofPopMatrix();
}

void BodySoundManager::start()
{
	this->playing = true;
}

void BodySoundManager::stop()
{
	this->playing = false;
	this->currentNoteIndex = -1;
	this->currentNoteStartTime = 0;
}


float BodySoundManager::getDurationForIndex(int index) {
	vector<int> durations;
	if (ofRandom(2) < 0.3)
		durations = { 700, 1000, 700, 700, 1000 };
	else
		durations = { 700, 700, 700, 1000, 1000 };
	return durations[index % durations.size()] * 1.;
}

MidiNote* BodySoundManager::pointToMidi(ofVec2f point)
{
	const int width = this->canvasWidth;
	const int height = this->canvasHeight;

	int x = (int)point.x;
	int y = (int)point.y;
	float divisionSize = (1.0 * width) / (1.0 * this->scale.size());

	int index = floor(1.0 * x / divisionSize);	
	return this->scale[index % this->scale.size()];	
}
