#pragma once

#include <string>
#include "ofxOsc.h"
#include "Constants.h"

using namespace std;

class ofOSCManager
{
public: 
	ofOSCManager(string host, int port);
	void setHost(string host);
	void setPort(int port);

	void sendStringMessageToAddress(string address, string message);
	void sendIntMessageToAddress(string address, int message);
	void sendFloatMessageToAddress(string address, float message);

	void sendBodyMessage(int bodyId, string category, string parameter, int value);
	void sendEnvironmentMessage(string parameter, int value);

	void sendBodyMidiSequence(int bodyId, vector<int> midiSequence, vector<int> jointSequenceRaw);
	void sendIsRecording(int bodyId, bool isRecording);

	void sendAllData();
	void sendHandDistanceData();

private:
	ofxOscSender oscSender;
	string oscHost;
	int oscPort;

	float lastMessageTimestamp;
	const int MESSAGE_INTERVAL_MS = 3000;

	void updateOscSender();
};

