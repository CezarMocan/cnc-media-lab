#include "assert.h"
#include "NetworkManager.h"

NetworkManager::NetworkManager(bool isServer, string remoteIp, int remotePort)
{
	if (isServer) {
		ofLogError() << "This NetworkManager constructor is supposed to only be called for a client!";
		abort();
	}
	this->isThisServer = isServer;
	this->remoteIp = remoteIp;
	this->remotePort = remotePort;

	this->oscSender.setup(this->remoteIp, this->remotePort);
}

NetworkManager::NetworkManager(bool isServer, int localPort)
{
	if (!isServer) {
		ofLogError() << "This NetworkManager constructor is supposed to only be called for a client!";
		abort();
	}
	this->isThisServer = isServer;
	this->localPort = localPort;

	this->oscReceiver.setup(this->localPort);
}

void NetworkManager::update() 
{
	if (this->isClient()) return;

	// Only need to check for incoming messages for the server
	while (oscReceiver.hasWaitingMessages()) {
		ofxOscMessage m;
		oscReceiver.getNextMessage(&m);

		if (m.getAddress().compare(OscCategories::REMOTE_BODY_DATA) == 0) {
			int bodyIndex = m.getArgAsInt(0);
			string data = m.getArgAsString(1);
			this->serializedData[bodyIndex] = data;
			this->dataTimestamps[bodyIndex] = ofGetSystemTimeMillis();
		}
		else if (m.getAddress().compare("/update") == 0) {
			ofLogNotice() << "Client /update!";
		}
		else {

		}
	}

}

bool NetworkManager::isServer()
{
	return this->isThisServer;
}

bool NetworkManager::isClient()
{
	return !this->isThisServer;
}

void NetworkManager::sendBodyData(int index, string data)
{
	if (this->isServer()) return;
	ofxOscMessage m;
	m.setAddress(OscCategories::REMOTE_BODY_DATA);
	m.addInt32Arg(index);
	m.addStringArg(data);
	this->oscSender.sendMessage(m);
}

string NetworkManager::getBodyData(int index)
{
	if (this->serializedData.find(index) == this->serializedData.end()) {
		return "";
	}
	else {
		return this->serializedData[index];
	}
}

bool NetworkManager::isBodyActive(int index)
{
	if (this->dataTimestamps.find(index) == this->dataTimestamps.end())
		return false;
	return (ofGetSystemTimeMillis() - this->dataTimestamps[index] < Constants::NETWORK_TRAFFIC_MAX_LATENCY_MS);
}


