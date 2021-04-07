#include "BodiesManager.h"

BodiesManager::BodiesManager()
{	
	this->initKinect();
	TrackedBody::initialize();

	// Body contour finder setup
	contourFinder.setMinAreaRadius(10);
	contourFinder.setMaxAreaRadius(1000);
	contourFinder.setThreshold(15);

	// Remote bodies intersection setup
	bodiesIntersectionPath = new ofPath();
	bodiesIntersectionActive = false;
	bodiesIntersectionStartTimestamp = 0;
}

void BodiesManager::initKinect()
{
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
}

// ------ App state, set once the user has connected ------

void BodiesManager::setNetworkManagers(PeerNetworkManager* peerNetworkManager, MaxMSPNetworkManager* maxMSPNetworkManager)
{
	this->peerNetworkManager = peerNetworkManager;
	this->maxMSPNetworkManager = maxMSPNetworkManager;
}

void BodiesManager::setIsLeftPlayer(bool isLeftPlayer)
{
	this->isLeftPlayer = isLeftPlayer;
}

void BodiesManager::setAutomaticShadowsEnabled(bool automaticShadowsEnabled)
{
	this->automaticShadowsEnabled = automaticShadowsEnabled;
}

void BodiesManager::setBodyContourPolygonFidelity(int bodyContourPolygonFidelity)
{
	this->bodyContourPolygonFidelity = bodyContourPolygonFidelity;
}

//------ Per frame updates ------

void BodiesManager::update()
{
	this->kinect.update();
	this->detectBodies();
	this->computeBodyContours();

	this->updateTrackedBodies();
	this->updateBodyShadows();
	this->updateRemoteBodies();

	this->updateBodiesIntersection();

	this->resolveInstrumentConflicts();
}

void BodiesManager::detectBodies() {
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
			this->trackedBodies[body.bodyId]->setNumberOfContourPoints(this->bodyContourPolygonFidelity);
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

void BodiesManager::computeBodyContours() {
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

void BodiesManager::updateTrackedBodies()
{
	// Update each tracked body after contour was detected
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

	TrackedBody* leftBody = this->getLeftBody();
	if (leftBody != NULL) leftBody->setGeneralColor(Colors::BLUE);

	TrackedBody* rightBody = this->getRightBody();
	if (rightBody != NULL) rightBody->setGeneralColor(Colors::RED);
}

void BodiesManager::updateBodyShadows()
{
	// Update each shadow and send data over OSC to MaxMSP and the peer
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

	// Update data for currently recording shadows, based on new frame data for tracked bodies
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
			rec->setNumberOfContourPoints(this->bodyContourPolygonFidelity);
			rec->updateContourData({ body->rawContour });
		}
	}

	// Manage auto-spawning for shadows
	if (!this->automaticShadowsEnabled) return;

	// Spawn new shadow at random interval
	int spawnRand = (int)ofRandom(0, Constants::SHADOW_EXPECTED_FREQUENCY_SEC * ofGetFrameRate());
	if (spawnRand == 0 && this->activeBodyShadows.size() < 2) {
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

void BodiesManager::updateRemoteBodies()
{
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
			this->remoteBodies[bodyId]->deserialize(bodyData);
			this->remoteBodies[bodyId]->update();
			this->remoteBodies[bodyId]->sendDataToMaxMSP();
		}
	}
}

void BodiesManager::updateBodiesIntersection() {
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

void BodiesManager::resolveInstrumentConflicts() {
	// If left body and right body are on the same instrument, reassign instrument to left body
	if (!this->isLeftPlayer) return;

	TrackedBody* rightBody = this->getRightBody();
	TrackedBody* leftBody = this->getLeftBody();

	if (leftBody == NULL || rightBody == NULL) return;

	if (leftBody->getInstrumentId() == rightBody->getInstrumentId()) {
		leftBody->assignInstrument();
	}
}

// ------ Per frame drawing ------

void BodiesManager::drawTrackedBodies() {
	for (int i = 0; i < this->trackedBodyIds.size(); i++) {
		const int bodyId = this->trackedBodyIds[i];
		TrackedBody* body = this->trackedBodies[bodyId];
		body->draw();
	}
}

void BodiesManager::drawRemoteBodies() {
	for (int bodyId = 0; bodyId < Constants::MAX_BODY_RECORDINGS + Constants::BODY_RECORDINGS_ID_OFFSET; bodyId++) {
		if (!this->peerNetworkManager->isBodyActive(bodyId)) continue;

		TrackedBody* body = this->remoteBodies[bodyId];
		if (body->getIsRecording()) {
			if (this->getLeftBody() == this->getRemoteBody()) {
				body->setGeneralColor(Colors::BLUE_SHADOW);
			}
			else {
				body->setGeneralColor(Colors::RED_SHADOW);
			}
		}
		body->draw();
	}
}

void BodiesManager::drawBodyShadows() {
	for (auto it = this->activeBodyShadows.begin(); it != this->activeBodyShadows.end(); ++it) {
		TrackedBodyShadow* rec = *it;
		if (this->getLocalBody() == this->getLeftBody()) {
			rec->setGeneralColor(Colors::BLUE_SHADOW);
		}
		else if (this->getLocalBody() == this->getRightBody()) {
			rec->setGeneralColor(Colors::RED_SHADOW);
		}
		rec->draw();
	}
}

void BodiesManager::drawBodiesIntersection() {
	this->bodiesIntersectionPath->setFillColor(Colors::YELLOW);
	this->bodiesIntersectionPath->setFilled(true);
	this->bodiesIntersectionPath->draw();
}


// ------ Body getters ------

TrackedBody* BodiesManager::getLocalBody()
{
	if (this->trackedBodyIds.size() > 0) {
		int bodyId = trackedBodyIds[0];
		return this->trackedBodies[bodyId];
	}
	else return NULL;
}

TrackedBody* BodiesManager::getRemoteBody()
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

TrackedBody* BodiesManager::getLeftBody()
{
	if (this->isLeftPlayer) return this->getLocalBody();
	else return this->getRemoteBody();
}

TrackedBody* BodiesManager::getRightBody()
{
	if (this->isLeftPlayer) return this->getRemoteBody();
	else return this->getLocalBody();
}

int BodiesManager::getLocalBodyIndex()
{
	TrackedBody* body = this->getLocalBody();
	return (body == NULL ? -1 : body->index);
}

int BodiesManager::getRemoteBodyIndex()
{
	TrackedBody* body = this->getRemoteBody();
	return (body == NULL ? -1 : body->index);
}

int BodiesManager::getLeftBodyIndex()
{
	TrackedBody* body = this->getLeftBody();
	return (body == NULL ? -1 : body->index);
}

int BodiesManager::getRightBodyIndex()
{
	TrackedBody* body = this->getRightBody();
	return (body == NULL ? -1 : body->index);
}

// ------ Body Shadow Management ------
void BodiesManager::clearBodyShadow(int index)
{
	if (index >= this->activeBodyShadows.size()) return;

	this->activeBodyShadows[index]->removeInstrument();
	this->activeBodyShadows.erase(this->activeBodyShadows.begin() + index);
	this->activeBodyShadowsParams.erase(this->activeBodyShadowsParams.begin() + index);
}

void BodiesManager::spawnBodyShadow()
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

void BodiesManager::playBodyShadow(int index)
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
