// This example shows how to work with the BodyIndex image in order to create
// a green screen effect. Note that this isn't super fast, but is helpful
// in understanding how the different image types & coordinate spaces work
// together. If you need performance, you will probably want to do this with shaders!

#include "ofApp.h"

using namespace std;
using namespace ofxCv;
using namespace cv;
using namespace Constants;

//--------------------------------------------------------------
void ofApp::setup() {
	// Application window setup
	int windowWidth = 2 * DEPTH_WIDTH;
	ofSetWindowShape(windowWidth + 2 * Layout::WINDOW_PADDING, windowWidth * 3 / 4 + Layout::WINDOW_PADDING);

	// Bodies detection & processing manager setup
	bodiesManager = new BodiesManager();

	// General app interface manager setup
	guiManager = new GUIManager();
	
	// Load shaders
	grainFbo.allocate(ofGetWindowWidth(), ofGetWindowHeight());
	grainShader.load("shaders_gl3/grain");

	// OSC networking setup
	maxMSPNetworkManager = new MaxMSPNetworkManager(Constants::OSC_HOST, Constants::OSC_PORT, Constants::OSC_RECEIVE_PORT);
	// This only gets initialized after the first screen, once the user hits "connect"
	peerNetworkManager = NULL;

	// Settings panel setup
	parametersPanelVisible = false;
	parametersPanel.setup();
	parametersPanel.add(bodyContourPolygonFidelity.set("Contour #points", 200, 10, 1000));
	parametersPanel.add(automaticShadowsEnabled.set("Auto Shadows", true));	

	// Networking panel setup
	peerConnectButton.addListener(this, &ofApp::peerConnectButtonPressed);

	networkPanel.setup();	
	networkPanel.add(peerIp.set("Peer IP", Constants::CEZAR_IP));
	networkPanel.add(peerPort.set("Peer Port", "12346"));
	networkPanel.add(localPort.set("Local Port", "12347"));
	networkPanel.add(isLeftPlayer.set("Left Side", true));
	networkPanel.add(peerConnectButton.setup("Connect"));	
}

void ofApp::peerConnectButtonPressed() {
	this->peerNetworkManager = new PeerNetworkManager(this->peerIp.get(), atoi(this->peerPort.get().c_str()), atoi(this->localPort.get().c_str()));
	this->bodiesManager->setNetworkManagers(this->peerNetworkManager, this->maxMSPNetworkManager);
	this->bodiesManager->setIsLeftPlayer(this->isLeftPlayer.get());
	this->bodiesManager->setAutomaticShadowsEnabled(this->automaticShadowsEnabled.get());
	this->bodiesManager->setBodyContourPolygonFidelity(this->bodyContourPolygonFidelity);
}

//--------------------------------------------------------------
void ofApp::update() {
	// Nothing to update before the user's hit 'Connect' to start the app.
	if (this->peerNetworkManager == NULL) {
		return;
	}

	this->bodiesManager->update();

	this->maxMSPNetworkManager->update();

	this->peerNetworkManager->update();

	this->guiManager->update(
		this->bodiesManager->getLeftBody(), 
		this->bodiesManager->getRightBody(), 
		this->maxMSPNetworkManager->getSequencerStep() - 1, 
		this->peerNetworkManager->isConnected(),
		this->peerNetworkManager->getLatency()
	);	
}

//--------------------------------------------------------------
void ofApp::draw() {
	if (this->peerNetworkManager == NULL)
		this->networkPanel.draw();
	else {
		ofClear(0, 0, 0, 255);
		grainFbo.begin();
		ofClear(0, 0, 0, 255);
		this->drawInterface();
		grainFbo.end();

		grainShader.begin();
		grainFbo.draw(0, 0);
		grainShader.end();

		// Draw parameters panel
		if (this->parametersPanelVisible) {
			parametersPanel.draw();
			stringstream ss;
			ss << "fps : " << ofGetFrameRate() << endl;
			ofDrawBitmapStringHighlight(ss.str(), 20, ofGetWindowHeight() - 40);
		}
	}
}

void ofApp::drawInterface() {
	int previewWidth = DEPTH_WIDTH;
	int previewHeight = DEPTH_HEIGHT;

	ofVec2f winSize = ofGetWindowSize();

	ofPushStyle();
	ofSetColor(Colors::BACKGROUND);
	ofDrawRectangle(0, 0, winSize.x, winSize.y);
	ofPopStyle();

	ofPushMatrix();
	ofTranslate(Layout::WINDOW_PADDING, Layout::WINDOW_PADDING);
	ofScale(Layout::WINDOW_SCALE);

	this->guiManager->drawBackgroundContours();

	this->bodiesManager->drawBodyShadows();
	this->bodiesManager->drawTrackedBodies();
	this->bodiesManager->drawRemoteBodies();
	this->bodiesManager->drawBodiesIntersection();

	this->guiManager->drawSequencer();
	this->guiManager->drawBodyTrackedStatus();
	this->guiManager->drawFrequencyGradient();

	ofPopMatrix();

	this->guiManager->drawRectangularFrame();
	this->guiManager->drawSystemStatus();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	switch (key) {
	case 'h':
		this->parametersPanelVisible = !this->parametersPanelVisible;
		break;
	case 'a':
		this->bodiesManager->spawnBodyShadow();
		break;
	/*
	case 's':
		for (int index = 0; index < this->activeBodyShadows.size(); index++) {
			this->playBodyShadow(index);
		}
		break;
	*/
	case 'd':
		this->bodiesManager->clearBodyShadow(0);
		break;
	}
}