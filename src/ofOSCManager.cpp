#include "ofOSCManager.h"

ofOSCManager::ofOSCManager(string host, int port)
{
	this->oscHost = host;
	this->oscPort = port;
	this->updateOscSender();
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

void ofOSCManager::updateOscSender() {
	this->oscSender.setup(this->oscHost, this->oscPort);
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
