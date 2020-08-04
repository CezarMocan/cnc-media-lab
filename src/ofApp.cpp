﻿// This example shows how to work with the BodyIndex image in order to create
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
	ofSetWindowShape(2 * DEPTH_WIDTH, 2 * DEPTH_HEIGHT);

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
	
	// Load shaders
	bodyFbo.allocate(DEPTH_WIDTH, DEPTH_HEIGHT);
	bodyDebugFbo.allocate(DEPTH_WIDTH, DEPTH_HEIGHT);
	bodyIndexShader.load("shaders_gl3/bodyIndex");

	// OSC setup
	this->oscSoundManager = new ofOSCManager(Constants::OSC_HOST, Constants::OSC_PORT);

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
	gui.add(localBodyDrawsContour.set("Local Body: Contour", true));
	gui.add(localBodyDrawsGeometry.set("Local Body: Geometric", false));
	gui.add(localBodyDrawsFill.set("Local Body: Fill", false));
	gui.add(localBodyDrawsJoints.set("Local Body: Joints", false));

	gui.add(remoteBodyDrawsContour.set("Remote Body: Contour", true));
	gui.add(remoteBodyDrawsGeometry.set("Remote Body: Geometric", false));
	gui.add(remoteBodyDrawsFill.set("Remote Body: Fill", false));
	gui.add(remoteBodyDrawsJoints.set("Remote Body: Joints", false));

	gui.add(recordedBodyDrawsContour.set("Recorded Body: Contour", true));
	gui.add(recordedBodyDrawsGeometry.set("Recorded Body: Geometric", false));
	gui.add(recordedBodyDrawsFill.set("Recorded Body: Fill", false));
	gui.add(recordedBodyDrawsJoints.set("Recorded Body: Joints", false));

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

	networkGui.add(peerIp.set("Peer IP", "10.147.20.54"));
	networkGui.add(peerPort.set("Peer Port", "12346"));
	networkGui.add(localPort.set("Local Port", "12347"));
	networkGui.add(peerConnectButton.setup("Connect"));

	this->networkManager = NULL;

	// Sequencer UI
	this->sequencerSteps.push_back(new SequencerStep(5, 5, 25, JointType_HandLeft, ofColor(241, 113, 97), ofColor(249, 206, 42)));
	this->sequencerSteps.push_back(new SequencerStep(35, 5, 25, JointType_ElbowLeft, ofColor(241, 113, 97), ofColor(249, 206, 42)));
	this->sequencerSteps.push_back(new SequencerStep(65, 5, 25, JointType_ShoulderLeft, ofColor(241, 113, 97), ofColor(249, 206, 42)));
	this->sequencerSteps.push_back(new SequencerStep(95, 5, 25, JointType_Head, ofColor(241, 113, 97), ofColor(249, 206, 42)));
	this->sequencerSteps.push_back(new SequencerStep(125, 5, 25, JointType_ShoulderRight, ofColor(241, 113, 97), ofColor(249, 206, 42)));
	this->sequencerSteps.push_back(new SequencerStep(155, 5, 25, JointType_ElbowRight, ofColor(241, 113, 97), ofColor(249, 206, 42)));
	this->sequencerSteps.push_back(new SequencerStep(185, 5, 25, JointType_HandRight, ofColor(241, 113, 97), ofColor(249, 206, 42)));
	this->sequencerSteps.push_back(new SequencerStep(215, 5, 25, JointType_SpineBase, ofColor(241, 113, 97), ofColor(249, 206, 42)));
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
				this->remoteBodies[bodyId] = new TrackedBody(bodyId, 0.75, 400, 2);
				this->remoteBodies[bodyId]->setOSCManager(this->oscSoundManager);
				this->remoteBodies[bodyId]->setTracked(true);
			}
			this->remoteBodies[bodyId]->updateSkeletonContourDataFromSerialized(bodyData);
			this->remoteBodies[bodyId]->update();
			this->remoteBodies[bodyId]->sendOSCData();
		}
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
				this->trackedBodies[body.bodyId] = new TrackedBody(body.bodyId, 0.75, 400, 2);
				this->trackedBodies[body.bodyId]->setOSCManager(this->oscSoundManager);
			}

			this->trackedBodies[body.bodyId]->setTracked(body.tracked);
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
		float t1 = ofGetSystemTimeMillis();
		bodyFbo.begin();
		ofClear(0, 0, 0, 255);
		bodyIndexShader.begin();
		bodyIndexShader.setUniformTexture("uBodyIndexTex", kinect.getBodyIndexSource()->getTexture(), 1);
		bodyIndexShader.setUniform1f("uBodyIndexToExtract", bodyId);
		kinect.getBodyIndexSource()->draw(0, 0, previewWidth, previewHeight);
		bodyIndexShader.end();
		bodyFbo.end();
		float t2 = ofGetSystemTimeMillis();

		/*
		bodyDebugFbo.begin();
		bodyIndexShader.begin();
		bodyIndexShader.setUniformTexture("uBodyIndexTex", kinect.getBodyIndexSource()->getTexture(), 1);
		bodyIndexShader.setUniform1f("uBodyIndexToExtract", bodyId);
		kinect.getBodyIndexSource()->draw(0, 0, previewWidth, previewHeight);
		bodyIndexShader.end();
		bodyDebugFbo.end();
		*/

		bodyFbo.getTexture().readToPixels(bodyPixels);
		float t3 = ofGetSystemTimeMillis();		
		bodyImage.setFromPixels(bodyPixels);
		float t4 = ofGetSystemTimeMillis();
		bodyImage.update();
		float t5 = ofGetSystemTimeMillis();

		contourFinder.findContours(bodyImage);
		float t6 = ofGetSystemTimeMillis();
		
		TrackedBody* currentBody = this->trackedBodies[bodyId];
		currentBody->updateContourData(contourFinder.getPolylines());
		currentBody->updateTextureData(bodyImage);

		float t7 = ofGetSystemTimeMillis();

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

TrackedBodyRecording* ofApp::createBodyRecording(int recordingIndex, int bodyId, int instrumentId)
{
	TrackedBodyRecording* rec = new TrackedBodyRecording(recordingIndex, 0.75, 400, 2);
	rec->setTrackedBodyIndex(bodyId);
	rec->setOSCManager(this->oscSoundManager);
	rec->setTracked(true);
	rec->setIsRecording(true);
	rec->assignInstrument(instrumentId);
	this->activeBodyRecordings.push_back(rec);

	return rec;
}

//--------------------------------------------------------------
void ofApp::draw() {
	if (this->networkManager == NULL)
		this->networkGui.draw();
	else {
		ofClear(0, 0, 0, 255);

		bodyDebugFbo.begin();
		ofClear(0, 0, 0, 255);
		bodyDebugFbo.end();

		this->detectBodySkeletons();
		this->detectBodyContours();

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
				(recordedBodyDrawsGeometry.get() ? BDRAW_MODE_JOINTS : 0) |
				(recordedBodyDrawsJoints.get() ? BDRAW_MODE_MOVEMENT : 0);
		}
		else {
			remoteDrawMode = (remoteBodyDrawsContour.get() ? BDRAW_MODE_CONTOUR : 0) |
				(remoteBodyDrawsFill.get() ? BDRAW_MODE_RASTER : 0) |
				(remoteBodyDrawsGeometry.get() ? BDRAW_MODE_JOINTS : 0) |
				(remoteBodyDrawsJoints.get() ? BDRAW_MODE_MOVEMENT : 0);
		}
		body->setDrawMode(remoteDrawMode);
		body->draw();
	}
}

void ofApp::drawTrackedBodyRecordings(int drawMode) {
	for (auto it = this->activeBodyRecordings.begin(); it != this->activeBodyRecordings.end(); ++it) {
		TrackedBodyRecording* rec = *it;
		rec->setDrawMode(drawMode);
		rec->draw();
	}
}

void ofApp::drawDebug()
{
	int previewWidth = DEPTH_WIDTH;
	int previewHeight = DEPTH_HEIGHT;

	ofPushMatrix();
	ofTranslate(0, DEPTH_HEIGHT);
	bodyDebugFbo.draw(0, 0);
	ofPopMatrix();

	ofPushMatrix();
	ofTranslate(0, 0);
	this->drawTrackedBodies(BDRAW_MODE_JOINTS | BDRAW_MODE_SOUND);
	ofPopMatrix();

	ofPushMatrix();
	ofTranslate(DEPTH_WIDTH, 0);
	this->drawTrackedBodies(BDRAW_MODE_MOVEMENT | BDRAW_MODE_SOUND);
	ofPopMatrix();

	// Draw each tracked body
	ofPushMatrix();
	ofTranslate(2 * DEPTH_WIDTH, 0);
	this->drawTrackedBodies(BDRAW_MODE_RASTER | BDRAW_MODE_SOUND);
	ofPopMatrix();

	float colorHeight = previewWidth * (kinect.getColorSource()->getHeight() / kinect.getColorSource()->getWidth());

	ofPushMatrix();
	ofTranslate(DEPTH_WIDTH, DEPTH_HEIGHT);
	kinect.getBodySource()->drawProjected(0, 0, DEPTH_WIDTH, DEPTH_HEIGHT);
	ofPopMatrix();

	ofPushMatrix();
	ofTranslate(2 * DEPTH_WIDTH, DEPTH_HEIGHT);
	this->drawTrackedBodies(BDRAW_MODE_CONTOUR | BDRAW_MODE_SOUND);
	ofPopMatrix();
}

void ofApp::drawSequencer() {
	for (int i = 0; i < this->trackedBodyIds.size(); i++) {
		int bodyId = trackedBodyIds[i];
		TrackedBody* body = this->trackedBodies[bodyId];
		for (auto ss : this->sequencerSteps) {
			ss->registerBody(body, ofColor(241, 113, 97), ofColor(252, 36, 21));
		}
	}
	int index = 0;
	for (auto ss : this->sequencerSteps) {
		ss->update();

		float stepSize = 200.0 / (1.0 * this->sequencerSteps.size());
		bool isHighlighted = ((int)floor(1.0 * (ofGetFrameNum() % 200) / stepSize)) == index;
		ss->draw(isHighlighted);
		index++;
	}
	/*
	ofPushStyle();
	ofSetColor(241, 113, 97);
	ofNoFill();
	int seqStepSize = 25;
	ofDrawRectangle(5, 5, seqStepSize, seqStepSize);
	for (int i = 0; i < this->trackedBodyIds.size(); i++) {
		int bodyId = trackedBodyIds[i];
		TrackedBody* body = this->trackedBodies[bodyId];
		ofVec2f leftHandPos = body->getJointPosition(JointType_AnkleLeft);

		clipper.Clear();
		clipper.addPolyline(this->trackedBodies[bodyId]->contour, ClipperLib::ptSubject);

		int clipSize = 50;
		ofRectangle rect = ofRectangle(leftHandPos.x - clipSize / 2, leftHandPos.y - clipSize / 2, clipSize, clipSize);
		clipper.addRectangle(rect, ClipperLib::ptClip);

		auto intersection = clipper.getClipped(ClipperLib::ClipType::ctIntersection);
		glm::vec2 lineOffset = leftHandPos - ofVec2f(clipSize / 2, clipSize / 2);
		ofPushStyle();		
		for (auto& line : intersection) {
			for (auto& point : line) {
				point = point - lineOffset;
				float scale = (1.0 * seqStepSize) / (1.0 * clipSize);
				point.x *= scale; point.y *= scale;
				point.x += 5; point.y += 5;
			}			
		}

		ofPath currentPath;
		currentPath.clear();
		for (auto& line : intersection) {
			currentPath.moveTo(line[0]);
			for (int i = 1; i < line.size(); i++) {
				currentPath.lineTo(line[i]);
			}
			currentPath.close();
		}
		currentPath.setFillColor(ofColor(252, 36, 21));
		currentPath.setFilled(true);
		currentPath.draw();

		ofSetColor(241, 113, 97);
		for (auto& line : intersection) {
			line.draw();
		}
		ofPopStyle();

	}
	ofPopStyle();
	*/
}

void ofApp::drawAlternate() {
	int previewWidth = DEPTH_WIDTH;
	int previewHeight = DEPTH_HEIGHT;

	ofPushMatrix();
	ofScale(2.0);

	ofPushStyle();
	ofSetColor(25, 32, 28);
	ofDrawRectangle(0, 0, DEPTH_WIDTH, DEPTH_HEIGHT);
	ofPopStyle();

	this->drawSequencer();

	/*
	ofPushStyle();
	ofSetLineWidth(0.5);
	ofSetColor(255, 225, 128, 255);	
	vector<ofColor> colors = { ofColor::fromHex(0xDDD8C4), ofColor::fromHex(0xA3C9A8), ofColor::fromHex(0x84B59F), ofColor::fromHex(0x69A297), ofColor::fromHex(0xA97C73) };

	const int SEQ_FREQ = 1;
	
	if (ofGetFrameNum() % SEQ_FREQ == 0)
		sequencerShapes.clear();

	for (int i = 0; i < Scales::PENTATONIC.size(); i++) {
		float currLineY = (1.0 * i) / (1.0 * Scales::PENTATONIC.size()) * DEPTH_HEIGHT;
		float nextLineY = (1.0 * i + 1) / (1.0 * Scales::PENTATONIC.size()) * DEPTH_HEIGHT;

		ofNoFill();

		//ofSetColor(colors[i % colors.size()]);
		//ofDrawRectangle(0, currLineY, DEPTH_WIDTH / 2, nextLineY - currLineY);

		//ofSetColor(colors[(int)(i + 3) % colors.size()]);
		//ofDrawRectangle(DEPTH_WIDTH / 2, currLineY, DEPTH_WIDTH / 2, nextLineY - currLineY);
		//ofDrawLine(0, currLineY, DEPTH_WIDTH, currLineY);		
		if (this->trackedBodyIds.size() > 0 && ofGetFrameNum() % SEQ_FREQ == 0) {
			float sliceSize = (1.0 * DEPTH_HEIGHT) / (1.0 * Scales::PENTATONIC.size());			
			int bodyId = this->trackedBodyIds[0];

			vector<ofVec2f> slicesOfInterest = {
				this->trackedBodies[bodyId]->getJointPosition(JointType_WristLeft),
				this->trackedBodies[bodyId]->getJointPosition(JointType_WristRight),
				this->trackedBodies[bodyId]->getJointPosition(JointType_SpineMid),
				this->trackedBodies[bodyId]->getJointPosition(JointType_Head),
				this->trackedBodies[bodyId]->getJointPosition(JointType_KneeLeft),
				this->trackedBodies[bodyId]->getJointPosition(JointType_KneeRight),
				this->trackedBodies[bodyId]->getJointPosition(JointType_HandLeft),
				this->trackedBodies[bodyId]->getJointPosition(JointType_HandRight),
				this->trackedBodies[bodyId]->getJointPosition(JointType_ElbowLeft),
				this->trackedBodies[bodyId]->getJointPosition(JointType_ElbowRight),
				this->trackedBodies[bodyId]->getJointPosition(JointType_HipRight),
				this->trackedBodies[bodyId]->getJointPosition(JointType_HipLeft),
				this->trackedBodies[bodyId]->getJointPosition(JointType_AnkleLeft),
				this->trackedBodies[bodyId]->getJointPosition(JointType_AnkleRight),
			};
			int found = -1;
			vector<int> foundSlices;
			for (int j = 0; j < slicesOfInterest.size(); j++) {
				int slice = (int)floor(slicesOfInterest[j].y / sliceSize);
				if (i == slice) foundSlices.push_back(j);
			}
			if (foundSlices.size() == 0) continue;

			for (auto& found : foundSlices) {
				clipper.Clear();
				clipper.addPolyline(this->trackedBodies[bodyId]->contour, ClipperLib::ptSubject);
				ofPolyline rect;
				
				//rect.addVertex(0, currLineY);
				//rect.addVertex(DEPTH_WIDTH, currLineY);
				//rect.addVertex(DEPTH_WIDTH, currLineY + 1 * (nextLineY - currLineY));
				//rect.addVertex(0, currLineY + 1 * (nextLineY - currLineY));
				
				float vSliceSize = DEPTH_WIDTH;//10 * sliceSize;
				float start, stop;
				start = slicesOfInterest[found].x - vSliceSize;
				stop = slicesOfInterest[found].x + vSliceSize;
				int steps = 50;
				rect.addVertex(slicesOfInterest[found].x - vSliceSize, currLineY);
				for (int i = 1; i < steps; i++) {
					float offset = (1.0 * i) / (1.0 * steps) * 2 * vSliceSize;
					float noise = ofNoise(offset / 50.0, found, ofGetFrameNum() / 100.);
					float val = ofMap(noise, 0., 1., -10., 10.);
					rect.addVertex(slicesOfInterest[found].x - vSliceSize + offset, currLineY + val);
				}				
				rect.addVertex(slicesOfInterest[found].x + vSliceSize, currLineY);

				rect.addVertex(slicesOfInterest[found].x + vSliceSize, currLineY + 1 * (nextLineY - currLineY));
				for (int i = 1; i < steps; i++) {
					float offset = (1.0 * i) / (1.0 * steps) * 2 * vSliceSize;
					float noise = ofNoise(offset / 50.0, found, ofGetFrameNum() / 100.);
					float val = ofMap(noise, 0., 1., -10., 10.);
					rect.addVertex(slicesOfInterest[found].x + vSliceSize - offset, currLineY + 1 * (nextLineY - currLineY) + val);
				}
				rect.addVertex(slicesOfInterest[found].x - vSliceSize, currLineY + 1 * (nextLineY - currLineY));

				rect.close();
				clipper.addPolyline(rect, ClipperLib::ptClip);

				auto intersection = clipper.getClipped(ClipperLib::ClipType::ctIntersection);
				ofPushStyle();
				int sgn = (i % 2) ? 1 : -1;
				//ofTranslate(((i % 6) * 10 * sgn) % DEPTH_WIDTH, 0);
				//ofSetColor(255, 128, 64, 255);
				ofSetColor(255, 225, 128, 255);

				if (intersection.size() == 0) {
					ofPopStyle();
					continue;
				}
				ofPolyline line;
				float minDistance = 1000000000;

				for (auto& currLine : intersection) {
					if (slicesOfInterest[found].squareDistance(currLine.getCentroid2D()) < minDistance) {
						minDistance = slicesOfInterest[found].squareDistance(currLine.getCentroid2D());
						line = currLine;
					}
				}
				line.close();

				sequencerShapes.push_back(line);

				ofPopStyle();
			}
		}
	}
	ofDrawLine(DEPTH_WIDTH / 2, 0, DEPTH_WIDTH / 2, DEPTH_HEIGHT);
	ofPopStyle();

	ofPath fillPath;
	int index = 0;
	for (auto& line : this->sequencerShapes) {
		fillPath.clear();
		fillPath.moveTo(line[0]);
		for (int i = 1; i < line.size(); i++) {
			fillPath.lineTo(line[i]);
		}
		fillPath.close();
		fillPath.setFilled(true);
		float window = (SEQ_FREQ) / (1.0 * this->sequencerShapes.size());
		if (ofGetFrameNum() % SEQ_FREQ >= index * window && ofGetFrameNum() % SEQ_FREQ < (index + 1) * window)
			fillPath.setFillColor(ofColor(255, 125, 28));
		else
			fillPath.setFillColor(ofColor(255, 225, 128));
		fillPath.draw();
		index++;
	}
	*/

	int recordedDrawMode = (recordedBodyDrawsContour.get() ? BDRAW_MODE_CONTOUR : 0) |
		(recordedBodyDrawsFill.get() ? BDRAW_MODE_RASTER : 0) |
		(recordedBodyDrawsGeometry.get() ? BDRAW_MODE_JOINTS : 0) |
		(recordedBodyDrawsJoints.get() ? BDRAW_MODE_MOVEMENT : 0);

	this->drawTrackedBodyRecordings(recordedDrawMode);

	int localDrawMode = (localBodyDrawsContour.get() ? BDRAW_MODE_CONTOUR : 0) |
		(localBodyDrawsFill.get() ? BDRAW_MODE_RASTER : 0) |
		(localBodyDrawsGeometry.get() ? BDRAW_MODE_JOINTS : 0) |
		(localBodyDrawsJoints.get() ? BDRAW_MODE_MOVEMENT : 0);

	this->drawTrackedBodies(localDrawMode);

	this->drawRemoteBodies(0);

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
	case 'a':
		for (int i = 0; i < this->trackedBodyIds.size(); i++) {
			const int bodyId = this->trackedBodyIds[i];
			TrackedBody* originalBody = this->trackedBodies[bodyId];
			const int instrumentId = originalBody->getInstrumentId();

			TrackedBodyRecording* body = this->createBodyRecording(Constants::BODY_RECORDINGS_ID_OFFSET + this->activeBodyRecordings.size(), bodyId, instrumentId);
			body->startRecording();
		}
		break;
	case 's':
		for (auto it = this->activeBodyRecordings.begin(); it != this->activeBodyRecordings.end(); ++it) {
			TrackedBodyRecording* rec = *it;
			if (!rec->getIsPlaying()) {
				int originalBodyId = rec->getTrackedBodyIndex();
				TrackedBody* originalBody = this->trackedBodies[originalBodyId];
				originalBody->assignInstrument();
				rec->startPlayLoop();
			}
		}
		break;
	case 'd':
		for (auto it = this->activeBodyRecordings.begin(); it != this->activeBodyRecordings.end(); ++it) {
			TrackedBodyRecording* rec = *it;
			rec->removeInstrument();
		}
		this->activeBodyRecordings.clear();
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