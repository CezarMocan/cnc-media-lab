#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "Constants.h"

#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

using namespace std;

class NetworkManager {
public:
	NetworkManager(string remoteIp, int remotePort, int localPort);
	bool isServer();
	bool isClient();
	
	void update();

	void sendBodyData(int index, string data);
	string getBodyData(int index);
	bool isBodyActive(int index);

private:
	bool isThisServer;
	string remoteIp;
	int remotePort;
	int localPort;
	map<int, string> serializedData;
	map<int, int> dataTimestamps;

	ofxOscSender oscSender;
	ofxOscReceiver oscReceiver;
};

#endif // !NETWORK_MANAGER_H
