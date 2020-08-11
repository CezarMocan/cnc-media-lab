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
	// Kinect setup
	int windowWidth = 2 * DEPTH_WIDTH;
	ofSetWindowShape(windowWidth + 100, windowWidth * 3 / 4 + 50);

	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();

	if (kinect.getSensor()->get_CoordinateMapper(&coordinateMapper) < 0) {
		ofLogError() << "Could not acquire CoordinateMapper!";
	}

	numBodiesTracked = 0;
	bHaveAllStreams = false;

	bodyIndexImg.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR);
	foregroundImg.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR);

	colorCoords.resize(DEPTH_WIDTH * DEPTH_HEIGHT);
	
	remoteIntersectionActive = false;
	remoteIntersectionStartTimestamp = 0;

	// Load shaders
	bodyFbo.allocate(DEPTH_WIDTH, DEPTH_HEIGHT);
	bodyDebugFbo.allocate(DEPTH_WIDTH, DEPTH_HEIGHT);
	bodyIndexShader.load("shaders_gl3/bodyIndex");

	// OSC setup
	this->oscSoundManager = new ofOSCManager(Constants::OSC_HOST, Constants::OSC_PORT, Constants::OSC_RECEIVE_PORT);

	// Tracked bodies initialize
	TrackedBody::initialize();

	// Contour finder setup
	contourFinder.setMinAreaRadius(10);
	contourFinder.setMaxAreaRadius(1000);
	contourFinder.setThreshold(15);

	// GUI setup	
	this->useBlur = false;
	this->useErode = false;
	this->useDilate = false;

	this->voronoiForceCellsInsideBody = false;
	this->voronoiFillMode = false;
	this->voronoiDrawCellCenters = false;
	this->voronoiConnectCellCenters = false;	

	this->guiVisible = true;
	gui.setup();
	gui.add(polygonFidelity.set("Contour #points", 200, 10, 1000));
	gui.add(automaticShadows.set("Auto Shadows", true));

	/*
	gui.add(localBodyDrawsContour.set("Local Body: Contour", true));
	gui.add(localBodyDrawsJoints.set("Local Body: Joints", false));
	gui.add(localBodyDrawsFill.set("Local Body: Fill", false));
	gui.add(localBodyDrawsHLines.set("Local Body: HLines", false));
	gui.add(localBodyDrawsVLines.set("Local Body: VLines", false));
	gui.add(localBodyDrawsDots.set("Local Body: Dots", false));
	gui.add(localBodyDrawsGrid.set("Local Body: Grid", false));

	gui.add(remoteBodyDrawsContour.set("Remote Body: Contour", true));
	gui.add(remoteBodyDrawsJoints.set("Remote Body: Joints", false));
	gui.add(remoteBodyDrawsFill.set("Remote Body: Fill", false));
	gui.add(remoteBodyDrawsHLines.set("Remote Body: HLines", false));
	gui.add(remoteBodyDrawsVLines.set("Remote Body: VLines", false));
	gui.add(remoteBodyDrawsDots.set("Remote Body: Dots", false));
	gui.add(remoteBodyDrawsGrid.set("Remote Body: Grid", false));


	gui.add(recordedBodyDrawsContour.set("Recorded Body: Contour", true));
	gui.add(recordedBodyDrawsJoints.set("Recorded Body: Joints", false));
	gui.add(recordedBodyDrawsFill.set("Recorded Body: Fill", false));
	gui.add(recordedBodyDrawsHLines.set("Recorded Body: HLines", false));
	gui.add(recordedBodyDrawsVLines.set("Recorded Body: VLines", false));
	gui.add(recordedBodyDrawsDots.set("Recorded Body: Dots", false));
	gui.add(recordedBodyDrawsGrid.set("Recorded Body: Grid", false));
	*/


	ofParameter<float> voronoiEnvironmentNoise;
	voronoiGui.setup();
	voronoiGui.add(voronoiEnvironmentCells.set("[V] Background Cells", 50, 0, 1000));
	voronoiGui.add(voronoiBodyCells.set("[V] Body Cells", 250, 0, 1000));
	voronoiGui.add(voronoiForceCellsInsideBody.set("[V] Force cells inside body", false));
	voronoiGui.add(voronoiSmoothing.set("[V] Cell smoothing", 0., 0., 30.));
	voronoiGui.add(voronoiEnvironmentNoise.set("[V] Background Noise", 0., 0., 3.));
	voronoiGui.add(voronoiFillMode.set("[V] Fill mode", false));
	voronoiGui.add(voronoiBackgroundHue.set("[V] Background Hue", 0, 0, 255));
	voronoiGui.add(voronoiBodyHue.set("[V] Body Hue", 128, 0, 255));
	voronoiGui.add(voronoiDrawCellCenters.set("[V] Draw Cell Centers", false));
	voronoiGui.add(voronoiConnectCellCenters.set("[V] Connect Cell Centers", false));

	MidiPlayer::loadSoundFonts();

	// Network GUI setup
	peerConnectButton.addListener(this, &ofApp::peerConnectButtonPressed);

	networkGui.setup();	

	networkGui.add(peerIp.set("Peer IP", Constants::CEZAR_IP));
	networkGui.add(peerPort.set("Peer Port", "12346"));
	networkGui.add(localPort.set("Local Port", "12347"));
	networkGui.add(isLeft.set("Left Side", true));
	networkGui.add(peerConnectButton.setup("Connect"));

	this->networkManager = NULL;

	// Sequencer UI
	this->sequencerLeft = new Sequencer(4, 4, 8, 20, 4, Colors::BLUE, Colors::BLUE_ACCENT, Colors::YELLOW);
	this->sequencerLeft->addSequencerStepForJoints({
		JointType_HandLeft, JointType_ElbowLeft, JointType_ShoulderLeft, 
		JointType_Head, JointType_ShoulderRight, JointType_ElbowRight, 
		JointType_HandRight, JointType_SpineBase,
		JointType_AnkleLeft, JointType_AnkleRight,
		JointType_SpineMid,
		JointType_KneeLeft, JointType_KneeRight,
	});

	this->sequencerRight = new Sequencer(DEPTH_WIDTH / 2 + 74, 4, 8, 20, 4, Colors::RED, Colors::RED_ACCENT, Colors::YELLOW);
	this->sequencerRight->addSequencerStepForJoints({
		JointType_HandLeft, JointType_ElbowLeft, JointType_ShoulderLeft,
		JointType_Head, JointType_ShoulderRight, JointType_ElbowRight,
		JointType_HandRight, JointType_SpineBase,
		JointType_AnkleLeft, JointType_AnkleRight,
		JointType_SpineMid,
		JointType_KneeLeft, JointType_KneeRight,
		});

	this->intersectionPath = new ofPath();
}

void ofApp::peerConnectButtonPressed() {
	this->networkManager = new NetworkManager(this->peerIp.get(), atoi(this->peerPort.get().c_str()), atoi(this->localPort.get().c_str()));
}

//--------------------------------------------------------------
void ofApp::update() {
	if (this->networkManager == NULL) {
		return;
	}	
	kinect.update();

	this->detectBodySkeletons();
	this->detectBodyContours();

	this->oscSoundManager->update();
	this->networkManager->update();

	// Update each tracked body after skeleton data was entered
	// Send data over the network
	for (int i = 0; i < this->trackedBodyIds.size(); i++) {
		const int bodyId = this->trackedBodyIds[i];
		this->trackedBodies[bodyId]->update();

		// Send sound data to MaxMSP
		this->trackedBodies[bodyId]->sendOSCData();
		
		// Send serialized body data over the network
		string data = this->trackedBodies[bodyId]->serialize();
		if (ofGetFrameNum() % 3 == 0)
			this->networkManager->sendBodyData(bodyId, data);
	}

	// Update each recording and send data over OSC to MaxMSP and the peer
	for (auto it = this->activeBodyRecordings.begin(); it != this->activeBodyRecordings.end(); ++it) {
		TrackedBodyRecording* rec = *it;		
		rec->update();

		if (rec->getIsPlaying()) {
			// Send sound data to MaxMSP
			rec->sendOSCData();

			// Send serialized body data over the network
			string data = rec->serialize();
			if (ofGetFrameNum() % 3 == 1)
				this->networkManager->sendBodyData(rec->index, data);
		}
	}

	// Get data from peer and forward to MaxMSP
	for (int bodyId = 0; bodyId < Constants::MAX_BODY_RECORDINGS + Constants::BODY_RECORDINGS_ID_OFFSET; bodyId++) {
		if (!this->networkManager->isBodyActive(bodyId)) {
			if (this->remoteBodies.find(bodyId) != this->remoteBodies.end()) {
				this->remoteBodies[bodyId]->setTracked(false);
				this->remoteBodies.erase(bodyId);
			}
		}
		else {
			string bodyData = this->networkManager->getBodyData(bodyId);
			if (bodyData.size() < 2) continue;
			if (this->remoteBodies.find(bodyId) == this->remoteBodies.end()) {
				this->remoteBodies[bodyId] = new TrackedBody(bodyId, 0.75, 400, 2, true);
				this->remoteBodies[bodyId]->setOSCManager(this->oscSoundManager);
				this->remoteBodies[bodyId]->setTracked(true);
				this->oscSoundManager->sendNewBody(this->remoteBodies[bodyId]->getInstrumentId());
			}
			this->remoteBodies[bodyId]->updateSkeletonContourDataFromSerialized(bodyData);
			this->remoteBodies[bodyId]->update();
			this->remoteBodies[bodyId]->sendOSCData();
		}
	}

	this->manageBodyRecordings();
	this->resolveInstrumentConflicts();
	this->updateBackgrounds();
	this->updateSequencer();
	this->updateIntersection();
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

void ofApp::detectBodySkeletons()
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
				this->trackedBodies[body.bodyId]->setOSCManager(this->oscSoundManager);
				this->trackedBodies[body.bodyId]->setTracked(true);

				this->oscSoundManager->sendNewBody(this->trackedBodies[body.bodyId]->getInstrumentId());
			}
			
			this->trackedBodies[body.bodyId]->updateSkeletonData(body.joints, coordinateMapper);
			this->trackedBodies[body.bodyId]->setContourPoints(this->polygonFidelity);
		}
		else {
			// Remove untracked bodies from map
			for (auto it = oldTrackedBodyIds.begin(); it != oldTrackedBodyIds.end(); ++it) {
				int index = *it;
				if (index == body.bodyId) {
					this->trackedBodies[index]->setTracked(false);
					this->trackedBodies.erase(index);
				}
			}
		}
	}

	// Update data for recordings
	for (auto it = this->activeBodyRecordings.begin(); it != this->activeBodyRecordings.end(); ++it) {
		TrackedBodyRecording* rec = *it;
		if (!rec->getIsRecording()) continue;

		int trackedBodyId = rec->getTrackedBodyIndex();

		// TODO (cez): This only applies to recordings of local bodies. Need to do the same for remote.
		if (this->trackedBodies.find(trackedBodyId) == this->trackedBodies.end()) {
			rec->stopRecording();
		}
		else {
			TrackedBody* body = this->trackedBodies[trackedBodyId];
			rec->updateSkeletonData(body->latestSkeleton, body->coordinateMapper);
			rec->setContourPoints(this->polygonFidelity);
		}
	}
}

void ofApp::detectBodyContours() {
	int previewWidth = DEPTH_WIDTH;
	int previewHeight = DEPTH_HEIGHT;

	// Split up the tracked bodies onto different textures
	for (int i = 0; i < this->trackedBodyIds.size(); i++) {
		const int bodyId = this->trackedBodyIds[i];
		/*
		bodyFbo.begin();
		ofClear(0, 0, 0, 255);
		bodyIndexShader.begin();
		bodyIndexShader.setUniformTexture("uBodyIndexTex", kinect.getBodyIndexSource()->getTexture(), 1);
		bodyIndexShader.setUniform1f("uBodyIndexToExtract", bodyId);
		kinect.getBodyIndexSource()->draw(0, 0, previewWidth, previewHeight);
		bodyIndexShader.end();
		bodyFbo.end();


		bodyFbo.getTexture().readToPixels(bodyPixels);
		bodyImage.setFromPixels(bodyPixels);
		bodyImage.update();
		*/

		//contourFinder.findContours(bodyImage);
		contourFinder.setUseTargetColor(true);
		contourFinder.setTargetColor(ofColor(bodyId));
		contourFinder.findContours(kinect.getBodyIndexSource()->getPixels());
		
		TrackedBody* currentBody = this->trackedBodies[bodyId];
		currentBody->updateContourData(contourFinder.getPolylines());

		//ofLogNotice() << (t2 - t1) << " " << (t3 - t2) << " " << (t4 - t3) << " " << (t5 - t4) << " " << (t6 - t5) << " " << (t7 - t6);		
	}

	// Update data for recordings
	for (auto it = this->activeBodyRecordings.begin(); it != this->activeBodyRecordings.end(); ++it) {
		TrackedBodyRecording* rec = *it;
		if (!rec->getIsRecording()) continue;

		int trackedBodyId = rec->getTrackedBodyIndex();

		//TODO (cez): This only works for local recordings
		if (this->trackedBodies.find(trackedBodyId) == this->trackedBodies.end()) {
			rec->stopRecording();
		}
		else {
			TrackedBody* body = this->trackedBodies[trackedBodyId];
			rec->updateContourData({ body->rawContour });
			//rec->updateTextureData(body->texture);
		}
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
		if (!this->networkManager->isBodyActive(bodyId)) continue;
		TrackedBody* body = this->remoteBodies[bodyId];
		if (body->getIsRecording()) continue;
		remoteBody = body;
	}

	return remoteBody;
}

TrackedBody* ofApp::getLeftBody()
{
	// Cy is on the left, Cezar is on the right
	if (this->isLeft.get()) {
	//if (this->peerIp.get().compare(Constants::CEZAR_IP)) {
		// Means that we're on Cy's computer
		return this->getLocalBody();
	}
	else {
		return this->getRemoteBody();
	}
}

TrackedBody* ofApp::getRightBody()
{
	// Cy is on the left, Cezar is on the right
	//if (this->peerIp.get().compare(Constants::CEZAR_IP)) {
		// Means that we're on Cy's computer
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

void ofApp::clearBodyRecording(int index)
{
	if (index >= this->activeBodyRecordings.size()) return;

	this->activeBodyRecordings[index]->removeInstrument();
	this->activeBodyRecordings.erase(this->activeBodyRecordings.begin() + index);
	this->activeBodyRecordingsParams.erase(this->activeBodyRecordingsParams.begin() + index);
}

void ofApp::spawnBodyRecording()
{
	TrackedBody* originalBody = this->getLocalBody();
	if (originalBody == NULL) return;
	const int instrumentId = originalBody->getInstrumentId();
	const int bodyId = originalBody->index;
	const int recordingIndex = Constants::BODY_RECORDINGS_ID_OFFSET + this->activeBodyRecordings.size();

	TrackedBodyRecording* rec = new TrackedBodyRecording(recordingIndex, 0.75, 400, 2);

	rec->setTrackedBodyIndex(bodyId);
	rec->setOSCManager(this->oscSoundManager);
	rec->setTracked(true);
	rec->setIsRecording(true);
	rec->assignInstrument(instrumentId);

	int spawnTime = ofGetSystemTimeMillis();
	float recordingDuration = 1000 * ofRandom(Constants::SHADOW_REC_MIN_DURATION_SEC, Constants::SHADOW_REC_MAX_DURATION_SEC);
	float playDuration = 1000 * ofRandom(Constants::SHADOW_PLAY_MIN_DURATION_SEC, Constants::SHADOW_PLAY_MAX_DURATION_SEC);

	this->activeBodyRecordings.push_back(rec);
	this->activeBodyRecordingsParams.push_back(make_pair(spawnTime, make_pair(recordingDuration, playDuration)));

	rec->startRecording();
}

void ofApp::playBodyRecording(int index)
{
	if (index >= this->activeBodyRecordings.size()) return;

	TrackedBodyRecording* rec = this->activeBodyRecordings[index];
	if (rec->getIsPlaying()) return;

	int originalBodyId = rec->getTrackedBodyIndex();
	TrackedBody* originalBody = this->trackedBodies[originalBodyId];
	originalBody->assignInstrument();
	rec->startPlayLoop();
	this->oscSoundManager->sendNewBody(rec->getInstrumentId());
}

void ofApp::manageBodyRecordings()
{
	if (!this->automaticShadows.get()) return;

	// Spawn if random is good
	int spawnRand = ofRandom(0, Constants::SHADOW_EXPECTED_FREQUENCY_SEC * ofGetFrameRate());
	if (spawnRand == 1 && this->activeBodyRecordings.size() < 2) {
		this->spawnBodyRecording();
	}

	vector<int> indicesToRemove;
	// Manage existing ones
	int currentTime = ofGetSystemTimeMillis();
	for (int index = 0; index < this->activeBodyRecordings.size(); index++) {
		TrackedBodyRecording* rec = this->activeBodyRecordings[index];
		int spawnTime = this->activeBodyRecordingsParams[index].first;
		float recordingDuration = this->activeBodyRecordingsParams[index].second.first;
		float playDuration = this->activeBodyRecordingsParams[index].second.second;

		if (currentTime - spawnTime >= recordingDuration && rec->getIsRecording()) {
			this->playBodyRecording(index);
		}

		if (currentTime - spawnTime >= recordingDuration + playDuration) {
			indicesToRemove.push_back(index);			
		}
	}

	for (auto it = indicesToRemove.begin(); it != indicesToRemove.end(); ++it) {
		this->clearBodyRecording(*it);
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	if (this->networkManager == NULL)
		this->networkGui.draw();
	else {
		ofClear(0, 0, 0, 255);
		//this->drawDebug();
		this->drawAlternate();
		// Draw GUI		
		if (this->guiVisible) {
			gui.draw();
			stringstream ss;
			ss << "fps : " << ofGetFrameRate() << endl;
			ofDrawBitmapStringHighlight(ss.str(), 20, ofGetWindowHeight() - 40);		
		}
	}
}

void ofApp::drawTrackedBodies(int drawMode) {
	for (int i = 0; i < this->trackedBodyIds.size(); i++) {
		const int bodyId = this->trackedBodyIds[i];
		TrackedBody* body = this->trackedBodies[bodyId];
		body->setDrawMode(drawMode);
		body->draw();
	}
}

void ofApp::drawRemoteBodies(int drawMode) {
	for (int bodyId = 0; bodyId < Constants::MAX_BODY_RECORDINGS + Constants::BODY_RECORDINGS_ID_OFFSET; bodyId++) {
		if (!this->networkManager->isBodyActive(bodyId)) continue;

		int remoteDrawMode;
		
		TrackedBody* body = this->remoteBodies[bodyId];

		if (body->getIsRecording()) {
			remoteDrawMode = (recordedBodyDrawsContour.get() ? BDRAW_MODE_CONTOUR : 0) |
				(recordedBodyDrawsFill.get() ? BDRAW_MODE_RASTER : 0) |
				(recordedBodyDrawsHLines.get() ? BDRAW_MODE_HLINES : 0) |
				(recordedBodyDrawsVLines.get() ? BDRAW_MODE_VLINES : 0) |
				(recordedBodyDrawsGrid.get() ? BDRAW_MODE_GRID : 0) |
				(recordedBodyDrawsDots.get() ? BDRAW_MODE_DOTS : 0) |
				(recordedBodyDrawsJoints.get() ? BDRAW_MODE_MOVEMENT : 0);
			if (this->getLeftBody() == this->getRemoteBody()) {
				body->setGeneralColor(Colors::BLUE_ACCENT_TRANSPARENT);
			} else {
				body->setGeneralColor(Colors::RED_ACCENT_TRANSPARENT);
			}
		}
		else {
			remoteDrawMode = (remoteBodyDrawsContour.get() ? BDRAW_MODE_CONTOUR : 0) |
				(remoteBodyDrawsFill.get() ? BDRAW_MODE_RASTER : 0) |
				(remoteBodyDrawsHLines.get() ? BDRAW_MODE_HLINES : 0) |
				(remoteBodyDrawsVLines.get() ? BDRAW_MODE_VLINES : 0) |
				(remoteBodyDrawsGrid.get() ? BDRAW_MODE_GRID : 0) |
				(remoteBodyDrawsDots.get() ? BDRAW_MODE_DOTS : 0) |
				(remoteBodyDrawsJoints.get() ? BDRAW_MODE_MOVEMENT : 0);
		}
		body->setDrawMode(remoteDrawMode);
		body->draw();
	}
}

void ofApp::drawTrackedBodyRecordings(int drawMode) {
	for (auto it = this->activeBodyRecordings.begin(); it != this->activeBodyRecordings.end(); ++it) {
		TrackedBodyRecording* rec = *it;
		if (this->getLocalBody() == this->getLeftBody()) {
			rec->setGeneralColor(Colors::BLUE_ACCENT_TRANSPARENT);
		} else if (this->getLocalBody() == this->getRightBody()) {
			rec->setGeneralColor(Colors::RED_ACCENT_TRANSPARENT);
		}
		rec->setDrawMode(drawMode);
		rec->draw();
	}
}

void ofApp::updateSequencer() {
	this->sequencerLeft->setTrackedBody(this->getLeftBody());
	this->sequencerRight->setTrackedBody(this->getRightBody());

	this->sequencerLeft->setCurrentHighlight(this->oscSoundManager->getSequencerStep() - 1);
	this->sequencerLeft->update();

	this->sequencerRight->setCurrentHighlight(this->oscSoundManager->getSequencerStep() - 1);
	this->sequencerRight->update();
}

void ofApp::drawSequencer() {
	this->sequencerLeft->draw();
	this->sequencerRight->draw();
}

void ofApp::updateIntersection() {
	TrackedBody* body = this->getLocalBody();
	TrackedBody* remoteMainBody = this->getRemoteBody();

	if (body == NULL || remoteMainBody == NULL) {
		remoteIntersectionActive = false;
		this->intersectionPath->clear();
		return;
	}

	this->clipper.Clear();

	body->contour.close();
	remoteMainBody->contour.close();

	this->clipper.addPolyline(body->contour, ClipperLib::ptSubject);
	this->clipper.addPolyline(remoteMainBody->contour, ClipperLib::ptClip);
	auto intersection = clipper.getClipped(ClipperLib::ClipType::ctIntersection);

	if (intersection.size() == 0) {
		if (remoteIntersectionActive) this->oscSoundManager->sendBodyIntersection(0, 0, 0);
		remoteIntersectionActive = false;
		this->intersectionPath->clear();
		return;
	}

	if (!remoteIntersectionActive) {
		remoteIntersectionActive = true;
		remoteIntersectionStartTimestamp = ofGetSystemTimeMillis();
	}

	this->intersectionPath->clear();
	float totalArea = 0;
	for (auto& line : intersection) {
		this->intersectionPath->moveTo(line[0]);
		for (int i = 1; i < line.size(); i++) {
			this->intersectionPath->lineTo(line[i]);
		}
		this->intersectionPath->close();
		totalArea += line.getArea();
	}

	float localBodyArea = fabs(body->contour.getArea());
	float remoteBodyArea = fabs(remoteMainBody->contour.getArea());
	float normalizedArea = (totalArea / (fmin(localBodyArea, remoteBodyArea)));
	float duration = (1.0 * ofGetSystemTimeMillis() - remoteIntersectionStartTimestamp) / 1000.0;

	this->oscSoundManager->sendBodyIntersection(normalizedArea, intersection.size(), duration);
}

void ofApp::drawIntersection() {
	this->intersectionPath->setFillColor(Colors::YELLOW);
	this->intersectionPath->setFilled(true);
	this->intersectionPath->draw();
}

void ofApp::updateBackgrounds() {
	TrackedBody* leftBody = this->getLeftBody();
	ofVec2f winSize = ofGetWindowSize() / 2.0;
	ofVec2f padding = ofVec2f(25, 25);

	if (leftBody != NULL) {
		int framesOnPosition = 2;
		int pathStart = (ofGetFrameNum() % (framesOnPosition * 1000)) / framesOnPosition;
		int noPoints = 50; //(sin(ofGetFrameNum() * 1.0 / 100.0) + 1) * 25;
		this->leftCtr = leftBody->getContourSegment(pathStart, noPoints);
		this->leftCtr.first->translate(glm::vec2(-this->leftCtr.second.x, -this->leftCtr.second.y));
		this->leftCtr.first->scale((winSize.x / 2 - 2 * padding.x) / this->leftCtr.second.width, ((winSize.y - 2 * padding.y) / this->leftCtr.second.height));
		this->leftCtr.first->translate(glm::vec2(winSize.x / 2 + padding.x / 2.0 - 5, padding.y / 2.0 - 5));
	}

	TrackedBody* rightBody = this->getRightBody();
	if (rightBody != NULL) {
		int framesOnPosition = 2;
		int pathStart = (ofGetFrameNum() % (framesOnPosition * 1000)) / framesOnPosition;
		int noPoints = 50; //(sin(ofGetFrameNum() * 1.0 / 100.0) + 1) * 25;
		this->rightCtr = rightBody->getContourSegment(pathStart, noPoints);

		this->rightCtr.first->translate(glm::vec2(-this->rightCtr.second.x, -this->rightCtr.second.y));
		this->rightCtr.first->scale((winSize.x / 2 - 2 * padding.x) / this->rightCtr.second.width, ((winSize.y - 2 * padding.y) / this->rightCtr.second.height));
		this->rightCtr.first->translate(glm::vec2(padding.x / 2.0 - 5, padding.y / 2.0 - 5));
	}
}

void ofApp::drawBackgrounds()
{
	if (this->leftCtr.first != NULL) {		
		this->leftCtr.first->setFilled(true);
		this->leftCtr.first->setColor(Colors::BLUE_TRANSPARENT);
		this->leftCtr.first->draw();

		this->leftCtr.first->setFilled(false);
		this->leftCtr.first->setStrokeColor(Colors::BLUE_TRANSPARENT);
		this->leftCtr.first->setStrokeWidth(1.);
		this->leftCtr.first->draw();
	}

	if (this->rightCtr.first != NULL) {
		this->rightCtr.first->setFilled(true);
		this->rightCtr.first->setColor(Colors::RED_TRANSPARENT);
		this->rightCtr.first->draw();

		this->rightCtr.first->setFilled(false);
		this->rightCtr.first->setStrokeColor(Colors::RED_TRANSPARENT);
		this->rightCtr.first->setStrokeWidth(1.);
		this->rightCtr.first->draw();
	}
}

void ofApp::drawAlternate() {
	int previewWidth = DEPTH_WIDTH;
	int previewHeight = DEPTH_HEIGHT;

	ofVec2f winSize = ofGetWindowSize();

	ofPushStyle();
	ofSetColor(Colors::BACKGROUND);
	ofDrawRectangle(0, 0, winSize.x, winSize.y);

	ofSetColor(Colors::YELLOW);
	ofNoFill();
	ofDrawRectangle(40, 40, winSize.x - 80, winSize.y - 80);
	//ofDrawRectangle(DEPTH_WIDTH / 2 - 0.5, 0, 1, DEPTH_HEIGHT);
	ofPopStyle();

	ofPushMatrix();
	ofTranslate(40, 40);
	ofScale(2.0);

	this->drawBackgrounds();

	TrackedBody* leftBody = this->getLeftBody();
	if (leftBody != NULL) leftBody->setGeneralColor(Colors::BLUE);
	TrackedBody* rightBody = this->getRightBody();
	if (rightBody != NULL) rightBody->setGeneralColor(Colors::RED);

	int recordedDrawMode = (recordedBodyDrawsContour.get() ? BDRAW_MODE_CONTOUR : 0) |
		(recordedBodyDrawsFill.get() ? BDRAW_MODE_RASTER : 0) |
		(recordedBodyDrawsHLines.get() ? BDRAW_MODE_HLINES : 0) |
		(recordedBodyDrawsVLines.get() ? BDRAW_MODE_VLINES : 0) |
		(recordedBodyDrawsGrid.get() ? BDRAW_MODE_GRID : 0) |
		(recordedBodyDrawsDots.get() ? BDRAW_MODE_DOTS : 0) |
		(recordedBodyDrawsJoints.get() ? BDRAW_MODE_MOVEMENT : 0);

	this->drawTrackedBodyRecordings(recordedDrawMode);

	int localDrawMode = (localBodyDrawsContour.get() ? BDRAW_MODE_CONTOUR : 0) |
		(localBodyDrawsFill.get() ? BDRAW_MODE_RASTER : 0) |
		(localBodyDrawsHLines.get() ? BDRAW_MODE_HLINES : 0) |
		(localBodyDrawsVLines.get() ? BDRAW_MODE_VLINES : 0) |
		(localBodyDrawsGrid.get() ? BDRAW_MODE_GRID : 0) |
		(localBodyDrawsDots.get() ? BDRAW_MODE_DOTS : 0) |
		(localBodyDrawsJoints.get() ? BDRAW_MODE_MOVEMENT : 0);

	this->drawTrackedBodies(localDrawMode);

	this->drawRemoteBodies(0);

	this->drawIntersection();

	this->drawSequencer();

	ofPopMatrix();
}

bool ofApp::isBorder(ofDefaultVec3 _pt) {
	ofRectangle bounds = this->voronoi.getBounds();

	return (_pt.x == bounds.x || _pt.x == bounds.x + bounds.width
		|| _pt.y == bounds.y || _pt.y == bounds.y + bounds.height);
}

void ofApp::drawVoronoi() {
	ofRectangle bbox;
	ofPolyline allPoints;
	ofPushStyle();
	ofSetColor(ofColor(248, 247, 232));
	ofFill();
	ofRect(ofRectangle(0, 0, DEPTH_WIDTH, DEPTH_HEIGHT));

	// Set up random cells in the environment
	if (this->voronoiEnvironmentCells.get() != this->voronoiRandomPoints.size()) {
		this->voronoiRandomPoints.clear();
		for (int i = 0; i < this->voronoiEnvironmentCells.get(); i++) {
			this->voronoiRandomPoints.addVertex(ofRandom(DEPTH_WIDTH), ofRandom(DEPTH_HEIGHT));
		}
	}
	else {
		for (int i = 0; i < this->voronoiRandomPoints.size(); i++) {
			//this->voronoiRandomPoints.getVertices()[i].x += ofRandom(voronoiEnvironmentNoise.get()) - voronoiEnvironmentNoise.get() / 2;
			//this->voronoiRandomPoints.getVertices()[i].y += ofRandom(voronoiEnvironmentNoise.get()) - voronoiEnvironmentNoise.get() / 2;
		}
	}

	allPoints.addVertices(this->voronoiRandomPoints.getVertices());

	// Add points from bodies
	for (int i = 0; i < this->trackedBodyIds.size(); i++) {
		const int bodyId = this->trackedBodyIds[i];
		TrackedBody* body = this->trackedBodies[bodyId];
		ofPolyline p = body->getVoronoiPolyline(voronoiBodyCells.get(), voronoiForceCellsInsideBody.get());
		allPoints.addVertices(p.getVertices());
		bbox = p.getBoundingBox();
	}

	bbox.y = 0;
	bbox.height = DEPTH_HEIGHT;

	//this->voronoi.setBounds(bbox);
	this->voronoi.setBounds(ofRectangle(0, 0, DEPTH_WIDTH, DEPTH_HEIGHT));
	this->voronoi.setPoints(allPoints.getVertices());
	this->voronoi.generate();
	//this->voronoi.draw();	
	vector<ofxVoronoiCell> cells = voronoi.getCells();
	for (int i = 0; i < cells.size(); i++) {
		ofSetColor(0);
		ofNoFill();
		ofMesh mesh;
		ofPolyline pl;
		if (this->voronoiFillMode.get())
			mesh.setMode(OF_PRIMITIVE_TRIANGLE_FAN);
		//mesh.setMode(OF_PRIMITIVE_LINE_LOOP);
		//mesh.addVertices(cells[i].points);
		//ofSetColor(120);
		//mesh.draw();

		//mesh.clear();		

		for (int j = 0; j < cells[i].points.size(); j++) {
			//if (!this->isBorder(cells[i].points[j]))
			if (this->voronoiFillMode.get()) {
				mesh.addVertex(cells[i].points[j]);
			}
			else {
				if (this->voronoiConnectCellCenters.get())
					pl.addVertex(cells[i].centroid);
				pl.addVertex(cells[i].points[j]);
			}
		}
		pl.close();
		//ofSetColor(30);
		//ofSetColor(ofColor::fromHsb(255. * i / cells.size(), 255, 128. * i / cells.size() + 128));
		if (i < this->voronoiRandomPoints.size())
			ofSetColor(ofColor::fromHsb(voronoiBackgroundHue.get(), 255, 10 * (i % 4) + 128));
		else
			ofSetColor(ofColor::fromHsb(voronoiBodyHue.get(), 255, 5 * (i % 4) + 128));

		if (this->voronoiFillMode.get())
			mesh.draw();
		else
			pl.getSmoothed(this->voronoiSmoothing.get()).draw();
		// Draw cell points
		if (this->voronoiDrawCellCenters.get()) {
			ofSetColor(40);
			ofFill();
			ofDrawCircle(cells[i].centroid, 1);
		}
	}

	ofPopStyle();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	switch (key) {
	case 'h':
		this->guiVisible = !this->guiVisible;
		break;
	case 'a':
		this->spawnBodyRecording();
		break;
	case 's':
		for (int index = 0; index < this->activeBodyRecordings.size(); index++) {
			this->playBodyRecording(index);
		}
		break;
	case 'd':
		this->clearBodyRecording(0);
		break;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}