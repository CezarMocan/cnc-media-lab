#include "ofOSCManager.h"

ofOSCManager::ofOSCManager(string host, int port, int receivePort)
{
	this->oscHost = host;
	this->oscPort = port;
	this->oscReceivePort = receivePort;
	this->updateOscSender();
	this->updateOscReceiver();
	this->sequencerStep = 0;
	ofLogNotice() << "OSC Sender sending to: " << host << ":" << port;
}

void ofOSCManager::setHost(string host) {
	this->oscHost = host;
	this->updateOscSender();
}

void ofOSCManager::setPort(int port) {
	this->oscPort = port;
	this->updateOscSender();
}

void ofOSCManager::setReceivePort(int receivePort)
{
	this->oscReceivePort = receivePort;
	this->updateOscReceiver();
}

void ofOSCManager::updateOscSender() {
	this->oscSender.setup(this->oscHost, this->oscPort);
}

void ofOSCManager::updateOscReceiver()
{
	this->oscReceiver.setup(this->oscReceivePort);
}

void ofOSCManager::update() {
	// Check for incoming messages from the peer
	while (oscReceiver.hasWaitingMessages()) {
		ofxOscMessage m;
		oscReceiver.getNextMessage(&m);

		if (m.getAddress().compare("/" + OscCategories::SEQUENCER_STEP) == 0) {
			this->sequencerStep = m.getArgAsInt(0);
		}
		else {
			ofLogWarning() << "Unrecognized message coming from OSC peer!";
		}
	}
}

int ofOSCManager::getSequencerStep()
{
	return this->sequencerStep;
}

void ofOSCManager::sendStringMessageToAddress(string address, string message) {
	ofxOscMessage m;
	m.setAddress(address);
	m.addStringArg(message);
	this->oscSender.sendMessage(m);
	ofLogNotice() << "Sending message: " << message << " to address: " << address;
}

void ofOSCManager::sendIntMessageToAddress(string address, int message) {
	ofxOscMessage m;
	m.setAddress(address);
	m.addInt32Arg(message);
	this->oscSender.sendMessage(m);
	ofLogNotice() << "Sending message: " << message << " to address: " << address;
}

void ofOSCManager::sendFloatMessageToAddress(string address, float message) {
	ofxOscMessage m;
	m.setAddress(address);
	m.addFloatArg(message);
	this->oscSender.sendMessage(m);
	ofLogNotice() << "Sending message: " << message << " to address: " << address;
}

void ofOSCManager::sendBodyMessage(int bodyId, string category, string parameter, int value)
{
	stringstream ss;
	ss << "/" << bodyId << " /" << category << " /" << parameter << " " << value;
	this->sendStringMessageToAddress(OscCategories::BODY, ss.str());
}

void ofOSCManager::sendEnvironmentMessage(string parameter, int value)
{
}

void ofOSCManager::sendBodyMidiSequence(int bodyId, vector<int> midiSequence, vector<int> jointSequenceRaw)
{
	stringstream ss;
	ss << "/" << bodyId;
	for (int i = 0; i < midiSequence.size(); i++) {
		ss << " " << midiSequence[i];
	}
	this->sendStringMessageToAddress(OscCategories::BODY_SEQUENCE, ss.str());	

	ss.str(string());
	ss << "/" << bodyId;
	for (int i = 0; i < jointSequenceRaw.size(); i++) {
		ss << " " << jointSequenceRaw[i];
	}
	this->sendStringMessageToAddress(OscCategories::BODY_SEQUENCE_RAW, ss.str());
}

void ofOSCManager::sendIsRecording(int bodyId, bool isRecording)
{
	stringstream ss;
	ss << "/" << bodyId << " " << (int)isRecording;
	this->sendStringMessageToAddress(OscCategories::BODY_IS_RECORDING, ss.str());
}

void ofOSCManager::sendBodyIntersection(float area, int noPolys, float duration)
{
	stringstream ss;
	ss << area << " " << noPolys << " " << duration;
	this->sendStringMessageToAddress(OscCategories::BODY_INTERSECTION, ss.str());
}

void ofOSCManager::sendNewBody(int bodyId)
{
	stringstream ss;
	ss << bodyId;
	this->sendStringMessageToAddress(OscCategories::NEW_BODY, ss.str());
}

void ofOSCManager::sendAllData()
{
	int time = ofGetSystemTimeMillis();
	if (time - this->lastMessageTimestamp < this->MESSAGE_INTERVAL_MS) {
		return;
	}

	this->lastMessageTimestamp = time;
	this->sendIntMessageToAddress("/test2", time);

}

void ofOSCManager::sendHandDistanceData()
{
}
