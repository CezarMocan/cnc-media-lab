#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "Constants.h"

#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

using namespace std;

class PeerNetworkManager {
public:
	PeerNetworkManager(string remoteIp, int remotePort, int localPort);
	
	void update();

	void sendBodyData(int index, string data);
	string getBodyData(int index);
	bool isBodyActive(int index);

	bool isConnected();
	string getLatency();

private:
	bool isThisServer;
	string remoteIp;
	int remotePort;
	int localPort;
	map<int, string> serializedData;
	map<int, int> dataTimestamps;

	ofxOscSender oscSender;
	ofxOscReceiver oscReceiver;

	int latestTimestamp;
	float smoothLatency;
	float latency;
	float displayLatency;
};

#endif // !NETWORK_MANAGER_H
