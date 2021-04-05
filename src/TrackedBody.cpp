#include <sstream>
#include "TrackedBody.h"

int TrackedBody::instruments[Constants::MAX_INSTRUMENTS];

void TrackedBody::initialize() {
	memset(TrackedBody::instruments, 0, Constants::MAX_INSTRUMENTS * sizeof(int));
}

TrackedBody::TrackedBody(int index, float smoothingFactor, int contourPoints, int noDelayedContours, bool isRemote)
{	
	this->index = index;
	this->smoothingFactor = smoothingFactor;
	this->contourPoints = contourPoints;
	this->instrumentId = -1;	
	this->speedShader.load("shaders_gl3/bodySpeed");
	this->hlinesShader.load("shaders_gl3/hlines");
	this->vlinesShader.load("shaders_gl3/vlines");
	this->gridShader.load("shaders_gl3/grid");
	this->fillShader.load("shaders_gl3/fill");
	this->dotsShader.load("shaders_gl3/dots");

	this->mainFbo.allocate(DEPTH_WIDTH, DEPTH_HEIGHT);
	this->polyFbo.allocate(DEPTH_WIDTH, DEPTH_HEIGHT);

	this->noContours = noDelayedContours;
	this->isRemote = isRemote;
	this->isTracked = false;

	this->setBodySoundPlayer(new BodySoundPlayer(index, DEPTH_WIDTH, DEPTH_HEIGHT, Scales::PENTATONIC));

	this->JOINT_WEIGHTS = {
		{JointType_WristLeft, 1.0}, {JointType_WristRight, 1.0},
		{JointType_HipLeft, 4.0}, {JointType_HipRight, 4.0},
		{JointType_ElbowLeft, 2.0}, {JointType_ElbowRight, 2.0},
		{JointType_ShoulderLeft, 3.0}, {JointType_ShoulderRight, 3.0},
		{JointType_AnkleLeft, 1.0}, {JointType_AnkleRight, 1.0},
		{JointType_KneeLeft, 1.5}, {JointType_KneeRight, 1.5},
	};

	this->isRecording = false;
	this->generalColor = ofColor(255, 225, 128, 255);
	this->segment = new ofPath();
}

void TrackedBody::setOSCManager(MaxMSPNetworkManager* m)
{
	this->maxMSPNetworkManager = m;
	this->bodySoundPlayer->setOscManager(m);
}

void TrackedBody::setBodySoundPlayer(BodySoundPlayer* bsp)
{
	this->bodySoundPlayer = bsp;
	this->bodySoundPlayer->start();
}

void TrackedBody::setIsTracked(bool isTracked)
{
	if (!this->isRemote) {
		if (isTracked != this->isTracked) {
			if (!isTracked) this->removeInstrument();
			else this->assignInstrument();
		}
	}

	this->isTracked = isTracked;
}

void TrackedBody::setNumberOfContourPoints(int contourPoints)
{
	if (contourPoints != this->contourPoints) {
		this->contourPoints = contourPoints;
		this->contour.clear();
		this->delayedContours.clear();
		this->voronoiPoints.clear();
	}
}

void TrackedBody::setIsRecording(bool isRecording)
{
	this->isRecording = isRecording;
}

bool TrackedBody::getIsRecording()
{
	return this->isRecording;
}

void TrackedBody::updateSkeletonData(map<JointType, ofxKinectForWindows2::Data::Joint> skeleton, ICoordinateMapper* coordinateMapper)
{
	this->latestSkeleton = skeleton;
	this->coordinateMapper = coordinateMapper;

	for (auto it = skeleton.begin(); it != skeleton.end(); ++it) {
		JointType currentJoint = it->first;
		if (skeleton[currentJoint].getTrackingState() == TrackingState_Tracked || skeleton[currentJoint].getTrackingState() == TrackingState_Inferred) {
			this->updateJointPosition(currentJoint, it->second.getProjected(coordinateMapper, ofxKFW2::ProjectionCoordinates::DepthCamera));
		}
	}
}

void TrackedBody::updateJointPosition(JointType joint, ofVec2f position)
{	
	if (this->joints.find(joint) == joints.end()) {
		this->joints[joint] = new TrackedJoint(joint);
	}

	TrackedJoint* currentJoint = this->joints[joint];
	currentJoint->setPosition(position, this->smoothingFactor);
}

void TrackedBody::updateContourData(vector<ofPolyline> contours)
{
	if (contours.size() == 0) return;
	// 1. Discard all contours except for the one of maximum perimeter
	// If this is not precise enough, can use the area (but that's probably more
	// computationally expensive)
	pair<int, float> maxContour = make_pair(-1, -1.0f);
	for (int i = 0; i < contours.size(); i++) {
		float perimeter = contours[i].getPerimeter();
		if (perimeter > maxContour.second) {
			maxContour = make_pair(i, perimeter);
		}
	}

	// 2. Save largest area contour and resample it to the desired number of points.
	ofPolyline newContour = contours[maxContour.first].getResampledByCount(this->contourPoints);
	this->rawContour = ofPolyline(newContour);

	// 3. Match with persistent contour
	if (this->contour.size() == 0) {
		this->contour = newContour;
		for (int i = 0; i < this->noContours; i++) {
			this->delayedContours.push_back(ofPolyline(newContour));
		}
	} 
	else {
		// Select matchesToCheck closest points in the new line to the first point in the persistent line
		// And then checked the total distance between the circular permutations, in order to find the right order.
		const int matchesToCheck = 5;

		auto newVertices = newContour.getVertices();
		auto persistentVertices = this->contour.getVertices();
		ofVec2f referencePersistentVertex = ofVec2f(persistentVertices[0]);		
		vector<float> distances;

		for (auto it = newVertices.begin(); it != newVertices.end(); ++it) {
			distances.push_back(referencePersistentVertex.squareDistance(ofVec2f(*it)));
		}
		
		nth_element(distances.begin(), distances.begin() + matchesToCheck, distances.end());
		float thresholdDistance = distances[matchesToCheck - 1];

		pair<int, float> minDistance = make_pair(-1, 1000000000.0f);
		for (int i = 0; i < newVertices.size(); i++) {
			float sqDistance = referencePersistentVertex.squareDistance(ofVec2f(newVertices[i]));
			if (sqDistance <= thresholdDistance + 0.1f) {
				// We have a candidate
				float distance = BodyUtils::getPolylineSquaredDistanceWithOffset(this->contour, newContour, i);
				if (distance < minDistance.second) minDistance = make_pair(i, distance);			
			}
		}

		this->contourIndexOffset = minDistance.first;

		// 4. Update persistent contour, with smoothing
		for (int i = 0; i < this->contour.size(); i++) {
			int newIndex = (i + this->contourIndexOffset) % newContour.size();
			this->contour[i].x = (1 - this->smoothingFactor) * newContour[newIndex].x + this->smoothingFactor * this->contour[i].x;
			this->contour[i].y = (1 - this->smoothingFactor) * newContour[newIndex].y + this->smoothingFactor * this->contour[i].y;
		}
	}	
}

void TrackedBody::updateDelayedContours() {
	if (this->contour.size() == 0) return;
	for (int ct = 0; ct < this->delayedContours.size(); ct++) {
		float smoothing = ofMap(sqrt(ct), 0, sqrt(this->delayedContours.size()), 0.9, 0.999);
		for (int i = 0; i < delayedContours[ct].size(); i++) {
			int newIndex = (i + this->contourIndexOffset) % this->contour.size();
			delayedContours[ct][i].x = (1 - smoothing) * this->contour[newIndex].x + smoothing * delayedContours[ct][i].x;
			delayedContours[ct][i].y = (1 - smoothing) * this->contour[newIndex].y + smoothing * delayedContours[ct][i].y;
		}
	}
}

float TrackedBody::getJointsDistance(JointType a, JointType b)
{
	if (this->joints.find(a) == this->joints.end()) return -1;
	if (this->joints.find(b) == this->joints.end()) return -1;
	ofVec2f j1 = this->joints[a]->getPosition();
	ofVec2f j2 = this->joints[b]->getPosition();
	return (j2 - j1).length();
}

float TrackedBody::getNormalizedJointsDistance(JointType a, JointType b)
{
	float d = this->getJointsDistance(a, b);
	float unit = this->getJointsDistance(JointType_ShoulderLeft, JointType_ShoulderRight);
	return d / (8.0 * unit);
}

float TrackedBody::getJointSpeed(JointType a)
{
	if (this->joints.find(a) == this->joints.end()) return 0.0f;
	return this->joints[a]->getSpeed();
}

float TrackedBody::getJointNormalizedSpeed(JointType a)
{
	float speed = this->getJointSpeed(a);
	float unit = this->getJointsDistance(JointType_ShoulderLeft, JointType_ShoulderRight);
	float weightMultiplier = (JOINT_WEIGHTS.find(a) != JOINT_WEIGHTS.end()) ? JOINT_WEIGHTS[a] : 1.0;
	return speed * weightMultiplier / unit * 200;
}

ofVec2f TrackedBody::getJointPosition(JointType a)
{
	if (this->joints.find(a) == this->joints.end()) return ofVec2f(0, 0);
	return this->joints[a]->getPosition();
}

float TrackedBody::getScreenRatio()
{
	float unit = this->getJointsDistance(JointType_ShoulderLeft, JointType_ShoulderRight);
	return unit / (1.0 * DEPTH_WIDTH);
}

pair<ofPath*, ofRectangle> TrackedBody::getContourSegment(int start, int amount)
{
	//ofPolyline ctr = this->contour;
	ofPolyline* ctr = &(this->delayedContours[this->delayedContours.size() - 1]);
	this->segment->clear();
	int index = start % ctr->size();
	int total = 0;
	this->segment->moveTo((*ctr)[index]);

	ofRectangle rect;
	rect.x = rect.width = (*ctr)[index].x;
	rect.y = rect.height = (*ctr)[index].y;

	while (total < amount) {
		index = (index + 1) % (*ctr).size();
		total++;
		this->segment->lineTo((*ctr)[index]);

		rect.x = fmin(rect.x, (*ctr)[index].x);
		rect.y = fmin(rect.y, (*ctr)[index].y);
		rect.width = fmax(rect.width, (*ctr)[index].x);
		rect.height = fmax(rect.height, (*ctr)[index].y);
	}

	rect.width -= rect.x;
	rect.height -= rect.y;
	return make_pair(this->segment, rect);
}

void TrackedBody::update()
{
	if (!this->isTracked) return;

	for (auto it = this->joints.begin(); it != this->joints.end(); ++it) {
		it->second->update();
	}

	this->bodySoundPlayer->setInterestPoints(this->getInterestPoints());
	this->bodySoundPlayer->update();

}

void TrackedBody::drawContours()
{
	if (!this->isTracked) return;
	if (this->contour.size() < 3) return;
	ofPushStyle();
	ofSetColor(this->generalColor);
	this->contour.draw();
	ofPopStyle();
}

void TrackedBody::setJointUniform(JointType joint, string uniformName, ofShader shader) {
	if (this->joints.find(joint) != this->joints.end()) {
		auto j = this->joints[joint];
		shader.setUniform3f(uniformName, j->getPosition().x, j->getPosition().y, 5.0f * j->getSpeed());
	}
}

vector<pair<JointType, ofVec2f> > TrackedBody::getInterestPoints()
{
	vector<pair<JointType, ofVec2f> > interestPoints;

	float leftRightDistance = this->getNormalizedJointsDistance(JointType_WristLeft, JointType_WristRight);
	float topBottomDistance = this->getNormalizedJointsDistance(JointType_Head, JointType_AnkleLeft);

	vector<JointType> interestJoints;
	interestJoints.push_back(JointType_SpineBase);
	interestJoints.push_back(JointType_Head);

	if (leftRightDistance >= 0.2f) {
		interestJoints.push_back(JointType_HandLeft);
		interestJoints.push_back(JointType_HandRight);
	}

	if (leftRightDistance >= 0.325f) {
		interestJoints.push_back(JointType_ElbowLeft);
		interestJoints.push_back(JointType_ElbowRight);
	}

	if (leftRightDistance >= 0.4f) {
		interestJoints.push_back(JointType_ShoulderLeft);
		interestJoints.push_back(JointType_ShoulderRight);
	}

	if (topBottomDistance >= 0.25f) {
		interestJoints.push_back(JointType_AnkleLeft);
		interestJoints.push_back(JointType_AnkleRight);
	}

	if (topBottomDistance >= 0.375f) {
		interestJoints.push_back(JointType_KneeLeft);
		interestJoints.push_back(JointType_KneeRight);
	}

	if (topBottomDistance >= 0.5f) {
		interestJoints.push_back(JointType_SpineMid);
	}

	for (auto it = interestJoints.begin(); it != interestJoints.end(); ++it) {
		if (this->joints.find(*it) != this->joints.end()) {
			pair<JointType, ofVec2f> currentPoint = make_pair(*it, this->joints[*it]->getPosition());
			interestPoints.push_back(currentPoint);
		}
	}

	if (interestPoints.size() > 0)
		sort(interestPoints.begin(), interestPoints.end(), TrackedBody::interestPointComparator);
	
	return interestPoints;
}

void TrackedBody::drawContourForRaster(ofColor color) {
	this->contourPath.clear();
	this->contourPath.moveTo(this->contour[0]);
	for (int i = 1; i < this->contour.size(); i++)
		this->contourPath.lineTo(this->contour[i]);
	this->contourPath.close();

	this->contourPath.setFilled(true);
	this->contourPath.setFillColor(color);
	this->contourPath.draw(0, 0);
}

void TrackedBody::drawRaster()
{
	this->drawContourForRaster(this->generalColor);
}

void TrackedBody::drawHLines()
{
	this->drawWithShader(&this->hlinesShader);
}

void TrackedBody::drawVLines()
{
	this->drawWithShader(&this->vlinesShader);
}

void TrackedBody::drawGrid()
{
	this->drawWithShader(&this->gridShader);
}

void TrackedBody::drawDots()
{
	this->drawWithShader(&this->dotsShader);
}

void TrackedBody::drawWithShader(ofShader* shader) {
	if (!this->isTracked) return;
	if (this->contour.size() < 3) return;

	MainFboManager::end();
	this->polyFbo.begin();
	ofClear(0, 0, 0, 255);
	this->drawContourForRaster(ofColor(255, 128, 128));
	this->polyFbo.end();
	MainFboManager::begin();

	float time = ofGetSystemTimeMillis();
	glm::vec4 color = glm::vec4(this->generalColor.r, this->generalColor.g, this->generalColor.b, this->generalColor.a) / 255.0;

	shader->begin();
	shader->setUniform1f("uTime", time);
	shader->setUniform4f("color", color);
	this->polyFbo.draw(0, 0);
	shader->end();
}

void TrackedBody::draw()
{
	this->updateDelayedContours();

	switch (this->instrumentId) {
	case 0:
		this->drawMode = BDRAW_MODE_CONTOUR;
		break;
	case 1:
		this->drawMode = BDRAW_MODE_HLINES;
		break;
	case 2:
		this->drawMode = BDRAW_MODE_RASTER;
		break;
	case 3:
		this->drawMode = BDRAW_MODE_DOTS;
		break;
	default:
		this->drawMode = BDRAW_MODE_VLINES;
		break;
	}

	if (this->drawMode & BDRAW_MODE_RASTER) this->drawRaster();
	if (this->drawMode & BDRAW_MODE_HLINES) this->drawHLines();
	if (this->drawMode & BDRAW_MODE_VLINES) this->drawVLines();
	if (this->drawMode & BDRAW_MODE_GRID) this->drawGrid();
	if (this->drawMode & BDRAW_MODE_DOTS) this->drawDots();
	if (this->drawMode & BDRAW_MODE_CONTOUR) this->drawContours();
}

void TrackedBody::assignInstrument()
{
	this->assignInstrument(TrackedBody::getFirstFreeInstrument());
}

void TrackedBody::reassignInstrument()
{
	this->assignInstrument();
}

void TrackedBody::assignInstrument(int instrumentId)
{
	int oldInstrumentId = this->instrumentId;
	this->instrumentId = instrumentId;
	TrackedBody::acquireInstrument(this->instrumentId);
	TrackedBody::releaseInstrument(oldInstrumentId);
}

int TrackedBody::getInstrumentId()
{
	return this->instrumentId;
}

void TrackedBody::removeInstrument() {
	TrackedBody::releaseInstrument(this->instrumentId);
	this->instrumentId = -1;
}

int TrackedBody::getFirstFreeInstrument()
{
	for (int i = 0; i < Constants::MAX_INSTRUMENTS; i++) {
		if (TrackedBody::instruments[i] == 0) return i;
	}

	return 0;
}

void TrackedBody::acquireInstrument(int instrumentId)
{
	if (instrumentId < 0 || instrumentId >= Constants::MAX_INSTRUMENTS) return;
	TrackedBody::instruments[instrumentId]++;
}

void TrackedBody::releaseInstrument(int instrumentId)
{
	if (instrumentId < 0 || instrumentId >= Constants::MAX_INSTRUMENTS) return;
	TrackedBody::instruments[instrumentId]--;
}

string TrackedBody::serialize()
{
	/*
	Message format is as following:
	----
	2 // Body index
	__SKELETON__ // Constants::SKELETON_DELIMITER
	3 // number of joints in message
	1 245 218 // JointType, xPos, yPos
	2 188 197
	8 12 297
	__CONTOUR__ // Constants::CONTOUR_DELIMITER
	100 // number of points on the contour
	12 23 // xPos, yPos for first point
	68 72
	...
	108, 112
	__IS_RECORDING__ // Constants::IS_RECORDING_DELIMITER
	0 // or 1
	__INSTRUMENT_ID__ // Constants::INSTRUMENT_ID_DELIMITER
	2
	----
	*/
	stringstream ss;
	ss << this->index << "\n";
	ss << Constants::SKELETON_DELIMITER << "\n";

	int noJoints = this->joints.size();
	ss << noJoints << "\n";

	for (auto it = this->joints.begin(); it != this->joints.end(); ++it) {
		JointType currentJoint = it->first;
		TrackedJoint* j = it->second;
		ss << currentJoint << " " << j->getTargetPosition().x << " " << j->getTargetPosition().y << "\n";
	}

	ss << Constants::CONTOUR_DELIMITER << "\n";
	// Resample contour we're sending in order to save bandwidth
	ofPolyline ctr = this->rawContour.getResampledByCount(this->contourPoints);
	ss << ctr.size() << "\n";
	for (int i = 0; i < ctr.size(); i++) {
		ss << ctr[i].x << " " << ctr[i].y << "\n";
	}

	ss << Constants::IS_RECORDING_DELIMITER << "\n";
	ss << (int)this->isRecording << "\n";

	ss << Constants::INSTRUMENT_ID_DELIMITER << "\n";
	ss << this->getInstrumentId();

	return ss.str();
}

void TrackedBody::updateSkeletonContourDataFromSerialized(string s)
{
	istringstream ss(s);
	int bodyIndex;
	ss >> bodyIndex;
	this->index = bodyIndex;

	string delimiter;
	ss >> delimiter;

	int noSkeletonPoints, joint;
	float x, y;
	ss >> noSkeletonPoints;
	
	for (int i = 0; i < noSkeletonPoints; i++) {
		ss >> joint >> x >> y;
		this->updateJointPosition(static_cast<JointType>(joint), ofVec2f(x, y));
	}
	
	ss >> delimiter;

	int noContourPoints;
	ss >> noContourPoints;

	ofPolyline c;

	for (int i = 0; i < noContourPoints; i++) {
		ss >> x >> y;
		c.addVertex(x, y);
	}

	this->updateContourData({ c });

	ss >> delimiter;
	bool isRecording;
	ss >> isRecording;
	this->setIsRecording(isRecording);

	ss >> delimiter;
	int instrumentId;
	ss >> instrumentId;
	this->assignInstrument(instrumentId);
}

void TrackedBody::sendDataToMaxMSP()
{	
	float value;
	float normalizedValue;	
	// Sequencer sound data
	this->bodySoundPlayer->sendOSC(this->instrumentId);

	if (ofGetFrameNum() % 3 != 0) return;
	// Send whether is recording
	this->maxMSPNetworkManager->sendIsRecording(this->instrumentId, this->getIsRecording());

	// Distances
	value = this->getNormalizedJointsDistance(JointType_WristLeft, JointType_KneeLeft);
	normalizedValue = ofMap(value, 0, 1, 0, 1023);
	this->maxMSPNetworkManager->sendBodyMessage(this->instrumentId, OscCategories::DISTANCE, "l-hand-l-knee", normalizedValue);

	value = this->getNormalizedJointsDistance(JointType_KneeRight, JointType_WristRight);
	normalizedValue = ofMap(value, 0, 1, 0, 1023);
	this->maxMSPNetworkManager->sendBodyMessage(this->instrumentId, OscCategories::DISTANCE, "r-hand-r-knee", normalizedValue);

	// Movements	
	value = this->getJointNormalizedSpeed(JointType_WristLeft);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->maxMSPNetworkManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "l-hand", normalizedValue);

	value = this->getJointNormalizedSpeed(JointType_WristRight);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->maxMSPNetworkManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "r-hand", normalizedValue);

	value = this->getJointNormalizedSpeed(JointType_AnkleLeft);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->maxMSPNetworkManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "l-foot", normalizedValue);

	value = this->getJointNormalizedSpeed(JointType_AnkleRight);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->maxMSPNetworkManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "r-foot", normalizedValue);

	value = this->getJointNormalizedSpeed(JointType_KneeLeft);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->maxMSPNetworkManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "l-knee", normalizedValue);

	value = this->getJointNormalizedSpeed(JointType_KneeRight);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->maxMSPNetworkManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "r-knee", normalizedValue);
}

bool TrackedBody::interestPointComparator(pair<JointType, ofVec2f> a, pair<JointType, ofVec2f> b)
{
	return (a.second.y <= b.second.y);
}

vector<JointType> TrackedBody::getCurrentlyPlayingJoints()
{
	return this->bodySoundPlayer->getCurrentlyPlayingJoints();
}

vector<JointType> TrackedBody::getCurrentlyPlaying16Joints()
{
	return this->bodySoundPlayer->getCurrentlyPlaying16Joints();
}

vector<float> TrackedBody::getCurrentlyPlaying16Frequencies()
{
	return this->bodySoundPlayer->getCurrentlyPlaying16Frequencies();
}

void TrackedBody::setGeneralColor(ofColor color)
{
	this->generalColor = color;
}
