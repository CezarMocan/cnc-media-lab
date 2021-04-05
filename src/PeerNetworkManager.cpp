#include "assert.h"
#include "PeerNetworkManager.h"

PeerNetworkManager::PeerNetworkManager(string remoteIp, int remotePort, int localPort)
{
	this->remoteIp = remoteIp;
	this->remotePort = remotePort;
	this->latestTimestamp = 0;
	this->latency = this->smoothLatency = -1;
	this->oscSender.setup(this->remoteIp, this->remotePort);

	this->localPort = localPort;
	this->oscReceiver.setup(this->localPort);
}

void PeerNetworkManager::update() 
{	
	// Check for incoming messages from the peer
	while (oscReceiver.hasWaitingMessages()) {
		ofxOscMessage m;
		oscReceiver.getNextMessage(&m);

		if (m.getAddress().compare(OscCategories::REMOTE_BODY_DATA) == 0) {
			int bodyIndex = m.getArgAsInt(0);
			string data = m.getArgAsString(1);
			this->serializedData[bodyIndex] = data;
			int timestamp = ofGetSystemTimeMillis();
			this->dataTimestamps[bodyIndex] = timestamp;

			if (this->latestTimestamp == 0) {
				this->latency = this->smoothLatency = this->displayLatency = 500;
			} else {
				this->latency = (timestamp - this->latestTimestamp);
				this->smoothLatency = 0.9 * this->smoothLatency + 0.1 * this->latency;
			}

			this->latestTimestamp = timestamp;
		} else {
			ofLogWarning() << "Unrecognized message coming from OSC peer!";
		}
	}

	if (ofGetFrameNum() % 40 == 0) this->displayLatency = this->smoothLatency;

}

void PeerNetworkManager::sendBodyData(int index, string data)
{
	ofxOscMessage m;
	m.setAddress(OscCategories::REMOTE_BODY_DATA);
	m.addInt32Arg(index);
	m.addStringArg(data);
	this->oscSender.sendMessage(m);
}

string PeerNetworkManager::getBodyData(int index)
{
	if (this->serializedData.find(index) == this->serializedData.end()) {
		return "";
	}
	else {
		return this->serializedData[index];
	}
}

bool PeerNetworkManager::isBodyActive(int index)
{
	if (this->dataTimestamps.find(index) == this->dataTimestamps.end())
		return false;
	return (ofGetSystemTimeMillis() - this->dataTimestamps[index] < Constants::NETWORK_TRAFFIC_MAX_LATENCY_MS);
}

bool PeerNetworkManager::isConnected() {
	return (ofGetSystemTimeMillis() - this->latestTimestamp < Constants::NETWORK_TRAFFIC_MAX_LATENCY_MS);
}

string PeerNetworkManager::getLatency() {
	if (!this->isConnected()) return "N/A";
	stringstream ss;
	ss << ((int)this->displayLatency) << "ms";
	return ss.str();
}
