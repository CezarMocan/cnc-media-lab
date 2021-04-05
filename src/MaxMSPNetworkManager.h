#pragma once

#include <string>
#include "ofxOsc.h"
#include "Constants.h"

using namespace std;

class MaxMSPNetworkManager
{
public: 
	MaxMSPNetworkManager(string host, int port, int receivePort);
	void setHost(string host);
	void setPort(int port);
	void setReceivePort(int receivePort);

	void sendStringMessageToAddress(string address, string message);
	void sendIntMessageToAddress(string address, int message);
	void sendFloatMessageToAddress(string address, float message);

	void sendBodyMessage(int bodyId, string category, string parameter, int value);
	void sendEnvironmentMessage(string parameter, int value);

	void sendBodyMidiSequence(int bodyId, vector<int> midiSequence, vector<int> jointSequenceRaw);
	void sendIsRecording(int bodyId, bool isRecording);

	void sendBodyIntersection(float area, int noPolys, float duration);

	void sendNewBody(int bodyId);

	void sendAllData();
	void sendHandDistanceData();

	void update();

	int getSequencerStep();

private:
	ofxOscSender oscSender;
	ofxOscReceiver oscReceiver;
	string oscHost;
	int oscPort;
	int oscReceivePort;
	int sequencerStep;

	float lastMessageTimestamp;
	const int MESSAGE_INTERVAL_MS = 3000;

	void updateOscSender();
	void updateOscReceiver();
};

