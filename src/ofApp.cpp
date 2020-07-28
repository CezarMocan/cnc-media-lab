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
	ofSetWindowShape(3 * DEPTH_WIDTH, 2 * DEPTH_HEIGHT);

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
	contourFinder.setMinAreaRadius(3);
	contourFinder.setMaxAreaRadius(1000);
	contourFinder.setThreshold(0);

	// GUI setup	
	this->useBlur = false;
	this->useErode = false;
	this->useDilate = false;

	this->voronoiForceCellsInsideBody = false;
	this->voronoiFillMode = false;
	this->voronoiDrawCellCenters = false;
	this->voronoiConnectCellCenters = false;

	ofParameter<float> voronoiEnvironmentNoise;


	gui.setup();
	gui.add(polygonFidelity.set("Contour #points", 200, 10, 1000));
	gui.add(voronoiEnvironmentCells.set("[V] Background Cells", 50, 0, 1000));
	gui.add(voronoiBodyCells.set("[V] Body Cells", 250, 0, 1000));
	gui.add(voronoiForceCellsInsideBody.set("[V] Force cells inside body", false));
	gui.add(voronoiSmoothing.set("[V] Cell smoothing", 0., 0., 30.));
	gui.add(voronoiEnvironmentNoise.set("[V] Background Noise", 0., 0., 3.));
	gui.add(voronoiFillMode.set("[V] Fill mode", false));	
	gui.add(voronoiBackgroundHue.set("[V] Background Hue", 0, 0, 255));
	gui.add(voronoiBodyHue.set("[V] Body Hue", 128, 0, 255));
	gui.add(voronoiDrawCellCenters.set("[V] Draw Cell Centers", false));
	gui.add(voronoiConnectCellCenters.set("[V] Connect Cell Centers", false));

	MidiPlayer::loadSoundFonts();

	// Network GUI setup
	serverConnectButton.addListener(this, &ofApp::serverConnectButtonPressed);

	networkGui.setup();	

	isServer = false;
	networkGui.add(isServer.set("Is Server", false));

	networkGui.add(serverIp.set("Server IP", "10.147.20.54"));
	networkGui.add(serverPort.set("Server Port", "12346"));
	networkGui.add(serverConnectButton.setup("Connect"));

	this->networkManager = NULL;	
}

void ofApp::serverConnectButtonPressed() {
	if (this->isServer.get()) {
		this->networkManager = new NetworkManager(true, atoi(this->serverPort.get().c_str()));
	}
	else {
		this->networkManager = new NetworkManager(false, this->serverIp.get(), atoi(this->serverPort.get().c_str()));
	}
}

//--------------------------------------------------------------
void ofApp::update() {
	if (this->networkManager == NULL) {
		return;

	}
	kinect.update();
	this->networkManager->update();
	// Update each tracked body after skeleton data was entered
	// Also send the data over OSC
	for (int i = 0; i < this->trackedBodyIds.size(); i++) {
		const int bodyId = this->trackedBodyIds[i];
		this->trackedBodies[bodyId]->update();
		this->trackedBodyRecordings[bodyId]->update();
		if (this->isServer.get()) {
			this->trackedBodies[bodyId]->sendOSCData();
			//this->trackedBodyRecordings[bodyId]->sendOSCData();
		}
		else {
			string data = this->trackedBodies[bodyId]->serialize();
			if (ofGetFrameNum() % 3 == 0)
				this->networkManager->sendBodyData(bodyId, data);
		}
	}	

	if (this->isServer.get()) {
		for (int bodyId = 0; bodyId < Constants::MAX_TRACKED_BODIES; bodyId++) {
			if (!this->networkManager->isBodyActive(bodyId)) continue;
			string bodyData = this->networkManager->getBodyData(bodyId);
			if (bodyData.size() < 2) continue;
			if (this->remoteBodies.find(bodyId) == this->remoteBodies.end()) {
				this->remoteBodies[bodyId] = new TrackedBody(bodyId, 0.75, 400);
				this->remoteBodies[bodyId]->setOSCManager(this->oscSoundManager);
				this->remoteBodies[bodyId]->setTracked(true);
			}
			this->remoteBodies[bodyId]->updateSkeletonContourDataFromSerialized(bodyData);
			this->remoteBodies[bodyId]->update();
			this->remoteBodies[bodyId]->sendOSCData();
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
	if (!this->isServer.get()) return;
	for (int bodyId = 0; bodyId < Constants::MAX_TRACKED_BODIES; bodyId++) {
		if (!this->networkManager->isBodyActive(bodyId)) continue;
		TrackedBody* body = this->remoteBodies[bodyId];
		body->setDrawMode(drawMode);
		body->draw();
	}
}

void ofApp::drawTrackedBodyRecordings(int drawMode) {
	for (int i = 0; i < this->trackedBodyIds.size(); i++) {
		const int bodyId = this->trackedBodyIds[i];
		TrackedBodyRecording* body = this->trackedBodyRecordings[bodyId];
		body->setDrawMode(drawMode);
		body->draw();
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

void ofApp::drawAlternate() {
	const int frame = ofGetFrameNum();
	if (frame % 20 == 0) {
		currView = (currView + 1) % 7;
	}
	int previewWidth = DEPTH_WIDTH;
	int previewHeight = DEPTH_HEIGHT;

	ofPushMatrix();
	ofScale(2.0);

	currView = 1;

	switch (currView) {
	case 0:
		this->drawTrackedBodies(BDRAW_MODE_JOINTS | BDRAW_MODE_SOUND);
		break;
	case 1:
		this->drawTrackedBodyRecordings(BDRAW_MODE_RASTER | BDRAW_MODE_SOUND);
		this->drawTrackedBodies(BDRAW_MODE_CONTOUR | BDRAW_MODE_MOVEMENT | BDRAW_MODE_SOUND);
		this->drawRemoteBodies(BDRAW_MODE_CONTOUR | BDRAW_MODE_MOVEMENT | BDRAW_MODE_SOUND);
		//this->drawTrackedBodyRecordings(BDRAW_MODE_CONTOUR | BDRAW_MODE_MOVEMENT | BDRAW_MODE_SOUND);
		break;
	case 2:
		this->drawTrackedBodies(BDRAW_MODE_RASTER | BDRAW_MODE_SOUND);
		break;
	case 3:
		bodyDebugFbo.draw(0, 0);
		break;
	case 4:
		kinect.getBodySource()->drawProjected(0, 0, DEPTH_WIDTH, DEPTH_HEIGHT);
		break;
	case 5:
		this->drawTrackedBodies(BDRAW_MODE_CONTOUR | BDRAW_MODE_SOUND);
		break;
	case 6:
		this->drawVoronoi();
		break;
	default:
		bodyDebugFbo.draw(0, 0);
		break;
	}

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

void ofApp::detectBodySkeletons()
{
	// Count number of tracked bodies and update skeletons for each tracked body
	numBodiesTracked = 0;
	auto& bodies = kinect.getBodySource()->getBodies();
	this->trackedBodyIds.clear();

	for (auto& body : bodies) {
		if (this->trackedBodies.find(body.bodyId) == this->trackedBodies.end()) {
			this->trackedBodies[body.bodyId] = new TrackedBody(body.bodyId, 0.75, 400);
			this->trackedBodies[body.bodyId]->setOSCManager(this->oscSoundManager);
			this->trackedBodyRecordings[body.bodyId] = new TrackedBodyRecording(body.bodyId, 0.75, 400);
		}

		this->trackedBodies[body.bodyId]->setTracked(body.tracked);
		this->trackedBodyRecordings[body.bodyId]->setTracked(true);

		if (body.tracked) {
			numBodiesTracked++;
			auto joints = body.joints;
			this->trackedBodyIds.push_back(body.bodyId);

			this->trackedBodies[body.bodyId]->updateSkeletonData(joints, coordinateMapper);
			this->trackedBodyRecordings[body.bodyId]->updateSkeletonData(joints, coordinateMapper);
		}
	}

	// Update each tracked body after skeleton data was entered
	// Also send the data over OSC
	for (int i = 0; i < this->trackedBodyIds.size(); i++) {
		const int bodyId = this->trackedBodyIds[i];
		this->trackedBodies[bodyId]->setContourPoints(this->polygonFidelity);
		this->trackedBodyRecordings[bodyId]->setContourPoints(this->polygonFidelity);
	}
}

void ofApp::detectBodyContours() {
	int previewWidth = DEPTH_WIDTH;
	int previewHeight = DEPTH_HEIGHT;

	// Split up the tracked bodies onto different textures
	for (int i = 0; i < this->trackedBodyIds.size(); i++) {
		const int bodyId = this->trackedBodyIds[i];
		bodyFbo.begin();
		ofClear(0, 0, 0, 255);
		bodyIndexShader.begin();
		bodyIndexShader.setUniformTexture("uBodyIndexTex", kinect.getBodyIndexSource()->getTexture(), 1);
		bodyIndexShader.setUniform1f("uBodyIndexToExtract", bodyId);
		kinect.getBodyIndexSource()->draw(0, 0, previewWidth, previewHeight);
		bodyIndexShader.end();
		bodyFbo.end();

		bodyDebugFbo.begin();
		bodyIndexShader.begin();
		bodyIndexShader.setUniformTexture("uBodyIndexTex", kinect.getBodyIndexSource()->getTexture(), 1);
		bodyIndexShader.setUniform1f("uBodyIndexToExtract", bodyId);
		kinect.getBodyIndexSource()->draw(0, 0, previewWidth, previewHeight);
		bodyIndexShader.end();
		bodyDebugFbo.end();

		bodyFbo.getTexture().readToPixels(bodyPixels);
		bodyImage.setFromPixels(bodyPixels);
		bodyImage.update();

		contourFinder.findContours(bodyImage);

		TrackedBody* currentBody = this->trackedBodies[bodyId];
		currentBody->updateContourData(contourFinder.getPolylines());
		currentBody->updateTextureData(bodyImage);

		TrackedBodyRecording* currentBodyRecording = this->trackedBodyRecordings[bodyId];
		currentBodyRecording->updateContourData(contourFinder.getPolylines());
		currentBodyRecording->updateTextureData(bodyImage);

		//string serialized = currentBody->serialize();
		//currentBody->deserialize(serialized);
	}
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
		/*
		stringstream ss;
		ss << "fps : " << ofGetFrameRate() << endl;
		ss << "Tracked bodies: " << numBodiesTracked << endl;
		if (!bHaveAllStreams) ss << endl << "Not all streams detected!";
		ofDrawBitmapStringHighlight(ss.str(), 20, 20);
		*/
		//gui.draw();	
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	switch (key) {
	case 'm':
		break;
	case 'r':
		break;
	case 't':
		break;
	case 'a':
		for (int i = 0; i < this->trackedBodyIds.size(); i++) {
			const int bodyId = this->trackedBodyIds[i];
			TrackedBodyRecording* body = this->trackedBodyRecordings[bodyId];
			body->startRecording();			
		}
		break;
	case 's':
		for (int i = 0; i < this->trackedBodyIds.size(); i++) {
			const int bodyId = this->trackedBodyIds[i];
			TrackedBodyRecording* body = this->trackedBodyRecordings[bodyId];
			body->startPlayLoop();
		}
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