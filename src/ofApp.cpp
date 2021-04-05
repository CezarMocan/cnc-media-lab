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
	// Window setup
	int windowWidth = 2 * DEPTH_WIDTH;
	ofSetWindowShape(windowWidth + 2 * Layout::WINDOW_PADDING, windowWidth * 3 / 4 + Layout::WINDOW_PADDING);

	// Kinect setup
	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();

	if (kinect.getSensor()->get_CoordinateMapper(&coordinateMapper) < 0) {
		ofLogError() << "Could not acquire CoordinateMapper!";
	}
	
	// Load shaders
	grainFbo.allocate(ofGetWindowWidth(), ofGetWindowHeight());
	grainShader.load("shaders_gl3/grain");

	// Load fonts
	fontRegular.load("fonts/be-ag-light.ttf", Layout::FONT_SIZE);
	fontBold.load("fonts/be-ag-medium.ttf", Layout::FONT_SIZE - 2);

	// OSC setup
	maxMSPNetworkManager = new MaxMSPNetworkManager(Constants::OSC_HOST, Constants::OSC_PORT, Constants::OSC_RECEIVE_PORT);

	// Tracked bodies initialize
	TrackedBody::initialize();

	// Contour finder setup
	contourFinder.setMinAreaRadius(10);
	contourFinder.setMaxAreaRadius(1000);
	contourFinder.setThreshold(15);

	// GUI setup	
	guiVisible = false;
	gui.setup();
	gui.add(polygonFidelity.set("Contour #points", 200, 10, 1000));
	gui.add(automaticShadows.set("Auto Shadows", true));	

	// Network GUI setup
	peerConnectButton.addListener(this, &ofApp::peerConnectButtonPressed);

	networkGui.setup();	
	networkGui.add(peerIp.set("Peer IP", Constants::CEZAR_IP));
	networkGui.add(peerPort.set("Peer Port", "12346"));
	networkGui.add(localPort.set("Local Port", "12347"));
	networkGui.add(isLeft.set("Left Side", true));
	networkGui.add(peerConnectButton.setup("Connect"));

	// Sequencer UI
	sequencerLeft = new Sequencer(Layout::FRAME_PADDING, Layout::FRAME_PADDING, 
		Layout::SEQUENCER_ROW_SIZE, Layout::SEQUENCER_ELEMENT_SIZE, 
		Layout::FRAME_PADDING, 
		Colors::BLUE, Colors::BLUE_ACCENT, Colors::YELLOW);

	sequencerLeft->addSequencerStepForJoints({
		JointType_HandLeft, JointType_ElbowLeft, JointType_ShoulderLeft, 
		JointType_Head, JointType_ShoulderRight, JointType_ElbowRight, 
		JointType_HandRight, JointType_SpineBase,
		JointType_AnkleLeft, JointType_AnkleRight,
		JointType_SpineMid,
		JointType_KneeLeft, JointType_KneeRight,
	});

	int srX = DEPTH_WIDTH - (Layout::FRAME_PADDING + Layout::SEQUENCER_ELEMENT_SIZE) * Layout::SEQUENCER_ROW_SIZE;
	sequencerRight = new Sequencer(srX, Layout::FRAME_PADDING, 
		Layout::SEQUENCER_ROW_SIZE, Layout::SEQUENCER_ELEMENT_SIZE, 
		Layout::FRAME_PADDING, 
		Colors::RED, Colors::RED_ACCENT, Colors::YELLOW);
	sequencerRight->addSequencerStepForJoints({
		JointType_HandLeft, JointType_ElbowLeft, JointType_ShoulderLeft,
		JointType_Head, JointType_ShoulderRight, JointType_ElbowRight,
		JointType_HandRight, JointType_SpineBase,
		JointType_AnkleLeft, JointType_AnkleRight,
		JointType_SpineMid,
		JointType_KneeLeft, JointType_KneeRight,
		});

	// Network manager initialization
	peerNetworkManager = NULL;
	
	// Remote bodies intersection setup
	bodiesIntersectionPath = new ofPath();
	bodiesIntersectionActive = false;
	bodiesIntersectionStartTimestamp = 0;
}

void ofApp::peerConnectButtonPressed() {
	this->peerNetworkManager = new PeerNetworkManager(this->peerIp.get(), atoi(this->peerPort.get().c_str()), atoi(this->localPort.get().c_str()));
}

//--------------------------------------------------------------
void ofApp::update() {
	if (this->peerNetworkManager == NULL) {
		return;
	}	
	this->kinect.update();
	this->detectBodies();
	this->computeBodyContours();

	this->maxMSPNetworkManager->update();
	this->peerNetworkManager->update();

	// Update each tracked body after skeleton data was entered
	// Send data over the network
	for (int i = 0; i < this->trackedBodyIds.size(); i++) {
		const int bodyId = this->trackedBodyIds[i];
		this->trackedBodies[bodyId]->update();

		// Send sound data to MaxMSP
		this->trackedBodies[bodyId]->sendDataToMaxMSP();
		
		// Send serialized body data over the network (every 3 frames seems enough)
		string data = this->trackedBodies[bodyId]->serialize();
		if (ofGetFrameNum() % 3 == 0)
			this->peerNetworkManager->sendBodyData(bodyId, data);
	}

	// Update each recording and send data over OSC to MaxMSP and the peer
	for (auto it = this->activeBodyShadows.begin(); it != this->activeBodyShadows.end(); ++it) {
		TrackedBodyShadow* rec = *it;
		rec->update();

		if (rec->getIsPlaying()) {
			// Send sound data to MaxMSP
			rec->sendDataToMaxMSP();

			// Send serialized body data over the network
			string data = rec->serialize();
			if (ofGetFrameNum() % 3 == 1)
				this->peerNetworkManager->sendBodyData(rec->index, data);
		}
	}

	// Get data from peer and forward to MaxMSP
	for (int bodyId = 0; bodyId < Constants::MAX_BODY_RECORDINGS + Constants::BODY_RECORDINGS_ID_OFFSET; bodyId++) {
		if (!this->peerNetworkManager->isBodyActive(bodyId)) {
			if (this->remoteBodies.find(bodyId) != this->remoteBodies.end()) {
				this->remoteBodies[bodyId]->setIsTracked(false);
				this->remoteBodies.erase(bodyId);
			}
		}
		else {
			string bodyData = this->peerNetworkManager->getBodyData(bodyId);
			if (bodyData.size() < 2) continue;
			if (this->remoteBodies.find(bodyId) == this->remoteBodies.end()) {
				this->remoteBodies[bodyId] = new TrackedBody(bodyId, 0.75, 400, 2, true);
				this->remoteBodies[bodyId]->setOSCManager(this->maxMSPNetworkManager);
				this->remoteBodies[bodyId]->setIsTracked(true);
				this->maxMSPNetworkManager->sendNewBody(this->remoteBodies[bodyId]->getInstrumentId());
			}
			this->remoteBodies[bodyId]->updateSkeletonContourDataFromSerialized(bodyData);
			this->remoteBodies[bodyId]->update();
			this->remoteBodies[bodyId]->sendDataToMaxMSP();
		}
	}

	this->updateBodyShadows();
	this->resolveInstrumentConflicts();
	this->updateBackgroundContours();
	this->updateSequencer();
	this->updateBodiesIntersection();
}

void ofApp::resolveInstrumentConflicts() {
	// If left body and right body are on the same instrument, reassign instrument to left body
	if (!this->isLeft.get()) return;

	TrackedBody* rightBody = this->getRightBody();
	TrackedBody* leftBody = this->getLeftBody();

	if (leftBody == NULL || rightBody == NULL) return;

	if (leftBody->getInstrumentId() == rightBody->getInstrumentId()) {
		leftBody->assignInstrument();
	}
}

void ofApp::detectBodies()
{
	// Count number of tracked bodies and update skeletons for each tracked body
	auto& bodies = kinect.getBodySource()->getBodies();
	vector<int> oldTrackedBodyIds = this->trackedBodyIds;
	this->trackedBodyIds.clear();

	for (auto& body : bodies) {
		if (body.tracked) {
			// Update body skeleton data for tracked bodies
			this->trackedBodyIds.push_back(body.bodyId);

			if (this->trackedBodies.find(body.bodyId) == this->trackedBodies.end()) {
				this->trackedBodies[body.bodyId] = new TrackedBody(body.bodyId, 0.75, 400, 2, false);
				this->trackedBodies[body.bodyId]->setOSCManager(this->maxMSPNetworkManager);
				this->trackedBodies[body.bodyId]->setIsTracked(true);

				this->maxMSPNetworkManager->sendNewBody(this->trackedBodies[body.bodyId]->getInstrumentId());
			}
			
			this->trackedBodies[body.bodyId]->updateSkeletonData(body.joints, coordinateMapper);
			this->trackedBodies[body.bodyId]->setNumberOfContourPoints(this->polygonFidelity);
		}
		else {
			// Remove untracked bodies from map
			for (auto it = oldTrackedBodyIds.begin(); it != oldTrackedBodyIds.end(); ++it) {
				int index = *it;
				if (index == body.bodyId) {
					this->trackedBodies[index]->setIsTracked(false);
					this->trackedBodies.erase(index);
				}
			}
		}
	}
}

void ofApp::computeBodyContours() {
	int previewWidth = DEPTH_WIDTH;
	int previewHeight = DEPTH_HEIGHT;

	// WARNING: This code works under the assumption that there is only one tracked body
	// (this is our installation setup.)
	for (int i = 0; i < this->trackedBodyIds.size(); i++) {
		const int bodyId = this->trackedBodyIds[i];
		contourFinder.setUseTargetColor(true);
		contourFinder.setTargetColor(ofColor(bodyId));
		contourFinder.setThreshold(0);
		contourFinder.findContours(kinect.getBodyIndexSource()->getPixels());
		
		TrackedBody* currentBody = this->trackedBodies[bodyId];
		currentBody->updateContourData(contourFinder.getPolylines());
	}
}

TrackedBody* ofApp::getLocalBody()
{
	if (this->trackedBodyIds.size() > 0) {
		int bodyId = trackedBodyIds[0];
		return this->trackedBodies[bodyId];
	}
	else {
		return NULL;
	}
}

TrackedBody* ofApp::getRemoteBody()
{
	TrackedBody* remoteBody = NULL;

	for (int bodyId = 0; bodyId < Constants::MAX_BODY_RECORDINGS + Constants::BODY_RECORDINGS_ID_OFFSET; bodyId++) {
		if (!this->peerNetworkManager->isBodyActive(bodyId)) continue;
		TrackedBody* body = this->remoteBodies[bodyId];
		if (body->getIsRecording()) continue;
		remoteBody = body;
	}

	return remoteBody;
}

TrackedBody* ofApp::getLeftBody()
{
	if (this->isLeft.get()) {
		return this->getLocalBody();
	}
	else {
		return this->getRemoteBody();
	}
}

TrackedBody* ofApp::getRightBody()
{
	if (this->isLeft.get()) {
		return this->getRemoteBody();
	}
	else {
		return this->getLocalBody();
	}
}

int ofApp::getLocalBodyIndex()
{
	TrackedBody* body = this->getLocalBody();
	return (body == NULL ? -1 : body->index);
}

int ofApp::getRemoteBodyIndex()
{
	TrackedBody* body = this->getRemoteBody();
	return (body == NULL ? -1 : body->index);
}

int ofApp::getLeftBodyIndex()
{
	TrackedBody* body = this->getLeftBody();
	return (body == NULL ? -1 : body->index);
}

int ofApp::getRightBodyIndex()
{
	TrackedBody* body = this->getRightBody();
	return (body == NULL ? -1 : body->index);
}

void ofApp::clearBodyShadow(int index)
{
	if (index >= this->activeBodyShadows.size()) return;

	this->activeBodyShadows[index]->removeInstrument();
	this->activeBodyShadows.erase(this->activeBodyShadows.begin() + index);
	this->activeBodyShadowsParams.erase(this->activeBodyShadowsParams.begin() + index);
}

void ofApp::spawnBodyShadow()
{
	TrackedBody* originalBody = this->getLocalBody();
	if (originalBody == NULL) return;
	const int instrumentId = originalBody->getInstrumentId();
	const int bodyId = originalBody->index;
	const int recordingIndex = Constants::BODY_RECORDINGS_ID_OFFSET + this->activeBodyShadows.size();

	TrackedBodyShadow* rec = new TrackedBodyShadow(recordingIndex, 0.75, 400, 2);

	rec->setTrackedBodyIndex(bodyId);
	rec->setOSCManager(this->maxMSPNetworkManager);
	rec->setIsTracked(true);
	rec->setIsRecording(true);
	rec->assignInstrument(instrumentId);

	int spawnTime = ofGetSystemTimeMillis();
	float recordingDuration = 1000 * ofRandom(Constants::SHADOW_REC_MIN_DURATION_SEC, Constants::SHADOW_REC_MAX_DURATION_SEC);
	float playDuration = 1000 * ofRandom(Constants::SHADOW_PLAY_MIN_DURATION_SEC, Constants::SHADOW_PLAY_MAX_DURATION_SEC);

	this->activeBodyShadows.push_back(rec);
	this->activeBodyShadowsParams.push_back(make_pair(spawnTime, make_pair(recordingDuration, playDuration)));

	rec->startRecording();
}

void ofApp::playBodyShadow(int index)
{
	if (index >= this->activeBodyShadows.size()) return;

	TrackedBodyShadow* rec = this->activeBodyShadows[index];
	if (rec->getIsPlaying()) return;

	int originalBodyId = rec->getTrackedBodyIndex();
	TrackedBody* originalBody = this->trackedBodies[originalBodyId];
	originalBody->assignInstrument();
	rec->startPlayLoop();
	this->maxMSPNetworkManager->sendNewBody(rec->getInstrumentId());
}

void ofApp::updateBodyShadows()
{
	// Update data for shadows based on new frame data for tracked bodies
	for (auto it = this->activeBodyShadows.begin(); it != this->activeBodyShadows.end(); ++it) {
		TrackedBodyShadow* rec = *it;
		if (!rec->getIsRecording()) continue;

		int trackedBodyId = rec->getTrackedBodyIndex();

		if (this->trackedBodies.find(trackedBodyId) == this->trackedBodies.end()) {
			rec->stopRecording();
		}
		else {
			TrackedBody* body = this->trackedBodies[trackedBodyId];
			rec->updateSkeletonData(body->latestSkeleton, body->coordinateMapper);
			rec->setNumberOfContourPoints(this->polygonFidelity);
			rec->updateContourData({ body->rawContour });
		}
	}

	if (!this->automaticShadows.get()) return;

	// Spawn shadow if random is good
	int spawnRand = (int) ofRandom(0, Constants::SHADOW_EXPECTED_FREQUENCY_SEC * ofGetFrameRate());
	if (spawnRand == 2 && this->activeBodyShadows.size() < 2) {
		bool isRecording = false;
		if (this->activeBodyShadows.size() == 1 && this->activeBodyShadows[0]->getIsRecording())
			isRecording = true;
		if (!isRecording) this->spawnBodyShadow();
	}

	vector<int> indicesToRemove;
	// Manage existing shadows
	int currentTime = ofGetSystemTimeMillis();
	for (int index = 0; index < this->activeBodyShadows.size(); index++) {
		TrackedBodyShadow* rec = this->activeBodyShadows[index];
		int spawnTime = this->activeBodyShadowsParams[index].first;
		float recordingDuration = this->activeBodyShadowsParams[index].second.first;
		float playDuration = this->activeBodyShadowsParams[index].second.second;

		if (currentTime - spawnTime >= recordingDuration && rec->getIsRecording()) {
			this->playBodyShadow(index);
		}

		if (currentTime - spawnTime >= recordingDuration + playDuration) {
			indicesToRemove.push_back(index);			
		}
	}

	for (auto it = indicesToRemove.begin(); it != indicesToRemove.end(); ++it) {
		this->clearBodyShadow(*it);
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	if (this->peerNetworkManager == NULL)
		this->networkGui.draw();
	else {
		ofClear(0, 0, 0, 255);
		grainFbo.begin();
		ofClear(0, 0, 0, 255);
		this->drawInterface();
		grainFbo.end();

		grainShader.begin();
		grainFbo.draw(0, 0);
		grainShader.end();

		// Draw GUI		
		if (this->guiVisible) {
			gui.draw();
			stringstream ss;
			ss << "fps : " << ofGetFrameRate() << endl;
			ofDrawBitmapStringHighlight(ss.str(), 20, ofGetWindowHeight() - 40);		
		}
	}
}

void ofApp::drawTrackedBodies() {
	for (int i = 0; i < this->trackedBodyIds.size(); i++) {
		const int bodyId = this->trackedBodyIds[i];
		TrackedBody* body = this->trackedBodies[bodyId];
		body->draw();
	}
}

void ofApp::drawRemoteBodies() {
	for (int bodyId = 0; bodyId < Constants::MAX_BODY_RECORDINGS + Constants::BODY_RECORDINGS_ID_OFFSET; bodyId++) {
		if (!this->peerNetworkManager->isBodyActive(bodyId)) continue;

		TrackedBody* body = this->remoteBodies[bodyId];
		if (body->getIsRecording()) {
			if (this->getLeftBody() == this->getRemoteBody()) {
				body->setGeneralColor(Colors::BLUE_SHADOW);
			} else {
				body->setGeneralColor(Colors::RED_SHADOW);
			}
		}
		body->draw();
	}
}

void ofApp::drawBodyShadows() {
	for (auto it = this->activeBodyShadows.begin(); it != this->activeBodyShadows.end(); ++it) {
		TrackedBodyShadow* rec = *it;
		if (this->getLocalBody() == this->getLeftBody()) {
			rec->setGeneralColor(Colors::BLUE_SHADOW);
		} else if (this->getLocalBody() == this->getRightBody()) {
			rec->setGeneralColor(Colors::RED_SHADOW);
		}
		rec->draw();
	}
}

void ofApp::updateSequencer() {
	this->sequencerLeft->setTrackedBody(this->getLeftBody());
	this->sequencerRight->setTrackedBody(this->getRightBody());

	this->sequencerLeft->setCurrentHighlight(this->maxMSPNetworkManager->getSequencerStep() - 1);
	this->sequencerLeft->update();

	this->sequencerRight->setCurrentHighlight(this->maxMSPNetworkManager->getSequencerStep() - 1);
	this->sequencerRight->update();
}

void ofApp::drawSequencer() {
	this->sequencerLeft->draw();
	this->sequencerRight->draw();
}

void ofApp::updateBodiesIntersection() {
	TrackedBody* body = this->getLocalBody();
	TrackedBody* remoteMainBody = this->getRemoteBody();

	if (body == NULL || remoteMainBody == NULL) {
		bodiesIntersectionActive = false;
		this->bodiesIntersectionPath->clear();
		return;
	}

	this->bodiesIntersectionClipper.Clear();

	body->contour.close();
	remoteMainBody->contour.close();

	this->bodiesIntersectionClipper.addPolyline(body->contour, ClipperLib::ptSubject);
	this->bodiesIntersectionClipper.addPolyline(remoteMainBody->contour, ClipperLib::ptClip);
	auto intersection = bodiesIntersectionClipper.getClipped(ClipperLib::ClipType::ctIntersection);

	if (intersection.size() == 0) {
		if (bodiesIntersectionActive) this->maxMSPNetworkManager->sendBodyIntersection(0, 0, 0);
		bodiesIntersectionActive = false;
		this->bodiesIntersectionPath->clear();
		return;
	}

	if (!bodiesIntersectionActive) {
		bodiesIntersectionActive = true;
		bodiesIntersectionStartTimestamp = ofGetSystemTimeMillis();
	}

	this->bodiesIntersectionPath->clear();
	float totalArea = 0;
	for (auto& line : intersection) {
		this->bodiesIntersectionPath->moveTo(line[0]);
		for (int i = 1; i < line.size(); i++) {
			this->bodiesIntersectionPath->lineTo(line[i]);
		}
		this->bodiesIntersectionPath->close();
		totalArea += line.getArea();
	}

	float localBodyArea = fabs(body->contour.getArea());
	float remoteBodyArea = fabs(remoteMainBody->contour.getArea());
	float normalizedArea = (totalArea / (fmin(localBodyArea, remoteBodyArea)));
	float duration = (1.0 * ofGetSystemTimeMillis() - bodiesIntersectionStartTimestamp) / 1000.0;

	this->maxMSPNetworkManager->sendBodyIntersection(normalizedArea, intersection.size(), duration);
}

void ofApp::drawBodiesIntersection() {
	this->bodiesIntersectionPath->setFillColor(Colors::YELLOW);
	this->bodiesIntersectionPath->setFilled(true);
	this->bodiesIntersectionPath->draw();
}

void ofApp::updateBackgroundContours() {
	TrackedBody* leftBody = this->getLeftBody();
	ofVec2f winSize = ofGetWindowSize() / 2.0;
	ofVec2f padding = ofVec2f(25, 25);

	if (leftBody != NULL) {
		int framesOnPosition = 2;
		int pathStart = (ofGetFrameNum() % (framesOnPosition * 1000)) / framesOnPosition;
		int noPoints = 50; //(sin(ofGetFrameNum() * 1.0 / 100.0) + 1) * 25;
		this->leftBackgroundContour = leftBody->getContourSegment(pathStart, noPoints);
		this->leftBackgroundContour.first->translate(glm::vec2(-this->leftBackgroundContour.second.x, -this->leftBackgroundContour.second.y));
		this->leftBackgroundContour.first->scale((winSize.x / 2 - 2 * padding.x) / this->leftBackgroundContour.second.width, ((winSize.y - 2 * padding.y) / this->leftBackgroundContour.second.height));
		this->leftBackgroundContour.first->translate(glm::vec2(winSize.x / 2 + padding.x / 2.0 - 5, padding.y / 2.0 - 5));
	}

	TrackedBody* rightBody = this->getRightBody();
	if (rightBody != NULL) {
		int framesOnPosition = 2;
		int pathStart = (ofGetFrameNum() % (framesOnPosition * 1000)) / framesOnPosition;
		int noPoints = 50; //(sin(ofGetFrameNum() * 1.0 / 100.0) + 1) * 25;
		this->rightBackgroundContour = rightBody->getContourSegment(pathStart, noPoints);
		this->rightBackgroundContour.first->translate(glm::vec2(-this->rightBackgroundContour.second.x, -this->rightBackgroundContour.second.y));
		this->rightBackgroundContour.first->scale((winSize.x / 2 - 2 * padding.x) / this->rightBackgroundContour.second.width, ((winSize.y - 2 * padding.y) / this->rightBackgroundContour.second.height));
		this->rightBackgroundContour.first->translate(glm::vec2(padding.x / 2.0 - 5, padding.y / 2.0 - 5));
	}
}

void ofApp::drawBackgroundContours()
{
	if (this->leftBackgroundContour.first != NULL) {		
		this->leftBackgroundContour.first->setFilled(true);
		this->leftBackgroundContour.first->setColor(Colors::BLUE_TRANSPARENT);
		this->leftBackgroundContour.first->draw();

		this->leftBackgroundContour.first->setFilled(false);
		this->leftBackgroundContour.first->setStrokeColor(Colors::BLUE_TRANSPARENT);
		this->leftBackgroundContour.first->setStrokeWidth(1.);
		this->leftBackgroundContour.first->draw();
	}

	if (this->rightBackgroundContour.first != NULL) {
		this->rightBackgroundContour.first->setFilled(true);
		this->rightBackgroundContour.first->setColor(Colors::RED_TRANSPARENT);
		this->rightBackgroundContour.first->draw();

		this->rightBackgroundContour.first->setFilled(false);
		this->rightBackgroundContour.first->setStrokeColor(Colors::RED_TRANSPARENT);
		this->rightBackgroundContour.first->setStrokeWidth(1.);
		this->rightBackgroundContour.first->draw();
	}
}

void ofApp::drawSystemStatus() {
	TrackedBody* leftBody = this->getLeftBody();
	TrackedBody* rightBody = this->getRightBody();

	bool isConnected = this->peerNetworkManager->isConnected();
	float width, totalWidth;
	ofPushStyle();
	ofSetColor(Colors::YELLOW);

	// Status	
	width = fontBold.stringWidth("Status_ ");
	fontBold.drawString("Status_ ", Layout::WINDOW_PADDING, Layout::WINDOW_PADDING - 7);
	string status = isConnected ? "Connected" : "Not Connected";
	fontRegular.drawString(status, Layout::WINDOW_PADDING + width, Layout::WINDOW_PADDING - 7);
	
	// IP 1
	width = fontBold.stringWidth("IP_1_ ");
	totalWidth = width + fontRegular.stringWidth(Constants::CEZAR_IP);
	fontBold.drawString("IP_1_ ", (ofGetWindowWidth() - totalWidth) / 2, Layout::WINDOW_PADDING - 7);	
	fontRegular.drawString(Constants::CEZAR_IP, (ofGetWindowWidth() - totalWidth) / 2 + width, Layout::WINDOW_PADDING - 7);

	// IP 2
	width = fontBold.stringWidth("IP_2_ ");
	totalWidth = width + fontRegular.stringWidth(Constants::CY_IP);
	fontBold.drawString("IP_2_ ", ofGetWindowWidth() - Layout::WINDOW_PADDING - totalWidth, Layout::WINDOW_PADDING - 7);
	fontRegular.drawString(Constants::CY_IP, ofGetWindowWidth() - Layout::WINDOW_PADDING - totalWidth + width, Layout::WINDOW_PADDING - 7);

	// Instrument 1
	if (leftBody != NULL) {
		int sz = Instruments::INSTRUMENT_LIST.size();
		string instrument1 = Instruments::INSTRUMENT_LIST[leftBody->getInstrumentId() % sz];
		ofPushMatrix();
		ofRotateDeg(270);
		width = fontBold.stringWidth("Instrument_1_ ");
		totalWidth = width + fontRegular.stringWidth(instrument1);
		fontBold.drawString("Instrument_1_ ", -(Layout::WINDOW_PADDING + totalWidth), Layout::WINDOW_PADDING - 7);
		fontRegular.drawString(instrument1, -(Layout::WINDOW_PADDING + totalWidth - width), Layout::WINDOW_PADDING - 7);
		ofPopMatrix();
	}

	if (rightBody != NULL) {
		// Instrument 2
		int sz = Instruments::INSTRUMENT_LIST.size();
		string instrument2 = Instruments::INSTRUMENT_LIST[rightBody->getInstrumentId() % sz];
		ofPushMatrix();
		ofRotateDeg(90);
		width = fontBold.stringWidth("Instrument_2_ ");
		fontBold.drawString("Instrument_2_ ", Layout::WINDOW_PADDING, -(ofGetWindowWidth() - Layout::WINDOW_PADDING + 7));
		fontRegular.drawString(instrument2, Layout::WINDOW_PADDING + width, -(ofGetWindowWidth() - Layout::WINDOW_PADDING + 7));
		ofPopMatrix();
	}

	// Latency
	string latency = this->peerNetworkManager->getLatency();
	width = fontBold.stringWidth("Latency_ ");
	totalWidth = width + fontRegular.stringWidth(latency);
	fontBold.drawString("Latency_ ", (ofGetWindowWidth() - totalWidth) / 2, ofGetWindowHeight() - Layout::WINDOW_PADDING + 15);
	fontRegular.drawString(latency, (ofGetWindowWidth() - totalWidth) / 2 + width, ofGetWindowHeight() - Layout::WINDOW_PADDING + 15);

	ofPopStyle();
}

void ofApp::drawBodyTrackedStatus() {
	TrackedBody* leftBody = this->getLeftBody();
	TrackedBody* rightBody = this->getRightBody();
	float currentScale = Layout::WINDOW_SCALE;
	int bottom = ofGetWindowHeight() / currentScale - Layout::WINDOW_PADDING;

	// Left body status
	int leftX = Layout::FRAME_PADDING;
	int leftY = bottom - Layout::FRAME_PADDING - Layout::SEQUENCER_ELEMENT_SIZE;
	int squareSize = Layout::SEQUENCER_ELEMENT_SIZE;
	int circleRadius = (Layout::SEQUENCER_ELEMENT_SIZE - 6) / 2;

	ofPushStyle();
	ofSetColor(Colors::BACKGROUND);
	ofFill();
	ofDrawRectangle(leftX, leftY, squareSize, squareSize);
	ofSetColor(Colors::YELLOW);
	ofNoFill();
	ofDrawRectangle(leftX, leftY, squareSize, squareSize);

	if (leftBody != NULL) {
		ofSetColor(Colors::BLUE_ACCENT);
		ofFill();
		ofDrawCircle(leftX + squareSize / 2, leftY + squareSize / 2, circleRadius);
	}

	ofNoFill();
	ofSetColor(Colors::YELLOW);
	ofDrawCircle(leftX + squareSize / 2, leftY + squareSize / 2, circleRadius);
	ofPopStyle();

	// Right body status
	int rightX = ofGetWindowWidth() / currentScale - Layout::WINDOW_PADDING - Layout::FRAME_PADDING - Layout::SEQUENCER_ELEMENT_SIZE;
	int rightY = leftY;

	ofPushStyle();
	ofSetColor(Colors::BACKGROUND);
	ofFill();
	ofDrawRectangle(rightX, rightY, squareSize, squareSize);
	ofSetColor(Colors::YELLOW);
	ofNoFill();
	ofDrawRectangle(rightX, rightY, squareSize, squareSize);

	if (rightBody != NULL) {
		ofSetColor(Colors::RED);
		ofFill();
		ofDrawCircle(rightX + squareSize / 2, rightY + squareSize / 2, circleRadius);
	}

	ofNoFill();
	ofSetColor(Colors::YELLOW);
	ofDrawCircle(rightX + squareSize / 2, rightY + squareSize / 2, circleRadius);
	ofPopStyle();
}

void ofApp::drawFrequencyGradient() {
	float currentScale = Layout::WINDOW_SCALE;
	int bottom = ofGetWindowHeight() / currentScale - Layout::WINDOW_PADDING;
	int leftX = Layout::FRAME_PADDING * 2 + Layout::SEQUENCER_ELEMENT_SIZE;
	int leftY = bottom - Layout::FRAME_PADDING - Layout::SEQUENCER_ELEMENT_SIZE;
	int width = ofGetWindowWidth() / currentScale - 2 * (Layout::WINDOW_PADDING / currentScale + Layout::SEQUENCER_ELEMENT_SIZE + 2 * Layout::FRAME_PADDING);
	int height = Layout::SEQUENCER_ELEMENT_SIZE;

	// Big rectangle gradient
	this->frequencyGradient.clear();
	this->frequencyGradient.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
	this->frequencyGradient.addVertex(ofPoint(leftX, leftY));
	this->frequencyGradient.addColor(Colors::BACKGROUND);
	this->frequencyGradient.addVertex(ofPoint(leftX + width / 2, leftY));
	this->frequencyGradient.addColor(Colors::YELLOW);
	this->frequencyGradient.addVertex(ofPoint(leftX, leftY + height));
	this->frequencyGradient.addColor(Colors::BACKGROUND);
	this->frequencyGradient.addVertex(ofPoint(leftX + width / 2, leftY + height));
	this->frequencyGradient.addColor(Colors::YELLOW);
	this->frequencyGradient.draw();
	this->frequencyGradient.clear();
	this->frequencyGradient.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
	this->frequencyGradient.addVertex(ofPoint(leftX + width / 2, leftY));
	this->frequencyGradient.addColor(Colors::YELLOW);
	this->frequencyGradient.addVertex(ofPoint(leftX + width, leftY));
	this->frequencyGradient.addColor(Colors::BACKGROUND);
	this->frequencyGradient.addVertex(ofPoint(leftX + width / 2, leftY + height));
	this->frequencyGradient.addColor(Colors::YELLOW);
	this->frequencyGradient.addVertex(ofPoint(leftX + width, leftY + height));
	this->frequencyGradient.addColor(Colors::BACKGROUND);
	this->frequencyGradient.draw();

	// 0Hz -> 5KHz text
	ofPushMatrix();
	ofPushStyle();
	ofSetColor(Colors::YELLOW);
	ofScale(1.0 / currentScale);
	fontRegular.drawString("0Hz", currentScale * leftX, currentScale * (leftY - 3.5));
	int textWidth = fontRegular.stringWidth("2kHz");
	fontRegular.drawString("2kHz", currentScale * (leftX + width) - textWidth, currentScale * (leftY - 3.5));
	ofPopStyle();
	ofPopMatrix();

	// Frequency indicator for left body
	int indicatorPadding = 4;
	int indicatorHeight = Layout::SEQUENCER_ELEMENT_SIZE - 2 * indicatorPadding;
	int indicatorWidth = indicatorHeight / 2;
	int maxFreq = 2000;

	TrackedBody* leftBody = this->getLeftBody();
	if (leftBody != NULL) {
		vector<float> freqs = leftBody->getCurrentlyPlaying16Frequencies();
		int index = this->maxMSPNetworkManager->getSequencerStep() - 1;
		if (freqs.size() > index) {
			float frequency = freqs[index];
			int indicatorX = leftX + ofMap(frequency, 0, maxFreq, 0, width);
			int indicatorY = leftY + indicatorPadding;

			ofPushStyle();
			ofSetColor(Colors::BLUE_ACCENT);
			ofFill();
			ofDrawRectangle(indicatorX, indicatorY, indicatorWidth, indicatorHeight);
			ofPopStyle();
		}
	}

	// Frequency indicator for right body
	TrackedBody* rightBody = this->getRightBody();
	if (rightBody != NULL) {
		vector<float> freqs = rightBody->getCurrentlyPlaying16Frequencies();
		int index = this->maxMSPNetworkManager->getSequencerStep() - 1;
		if (freqs.size() > index) {
			float frequency = freqs[index];
			int indicatorX = leftX + ofMap(frequency, 0, maxFreq, 0, width);
			int indicatorY = leftY + indicatorPadding;

			ofPushStyle();
			ofSetColor(Colors::RED);
			ofFill();
			ofDrawRectangle(indicatorX, indicatorY, indicatorWidth, indicatorHeight);
			ofPopStyle();
		}
	}
}

void ofApp::drawRectangularFrame() {
	ofVec2f winSize = ofGetWindowSize();
	ofPushStyle();
	ofSetColor(Colors::BACKGROUND);
	ofFill();
	ofDrawRectangle(0, 0, Layout::WINDOW_PADDING, winSize.y);
	ofDrawRectangle(0, winSize.y - Layout::WINDOW_PADDING, winSize.x, Layout::WINDOW_PADDING);
	ofDrawRectangle(winSize.x - Layout::WINDOW_PADDING, 0, Layout::WINDOW_PADDING, winSize.y);
	ofDrawRectangle(0, 0, winSize.x, Layout::WINDOW_PADDING);

	ofSetColor(Colors::YELLOW);
	ofNoFill();
	ofDrawRectangle(Layout::WINDOW_PADDING, Layout::WINDOW_PADDING, winSize.x - 2 * Layout::WINDOW_PADDING, winSize.y - 2 * Layout::WINDOW_PADDING);
	ofPopStyle();
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

	this->drawBackgroundContours();

	TrackedBody* leftBody = this->getLeftBody();
	if (leftBody != NULL) leftBody->setGeneralColor(Colors::BLUE);
	TrackedBody* rightBody = this->getRightBody();
	if (rightBody != NULL) rightBody->setGeneralColor(Colors::RED);

	this->drawBodyShadows();
	this->drawTrackedBodies();
	this->drawRemoteBodies();
	this->drawBodiesIntersection();
	this->drawSequencer();	
	this->drawBodyTrackedStatus();
	this->drawFrequencyGradient();

	ofPopMatrix();

	this->drawRectangularFrame();
	this->drawSystemStatus();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	switch (key) {
	case 'h':
		this->guiVisible = !this->guiVisible;
		break;
	case 'a':
		this->spawnBodyShadow();
		break;
	case 's':
		for (int index = 0; index < this->activeBodyShadows.size(); index++) {
			this->playBodyShadow(index);
		}
		break;
	case 'd':
		this->clearBodyShadow(0);
		break;
	}
}