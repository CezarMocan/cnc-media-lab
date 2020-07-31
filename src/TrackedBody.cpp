#include <sstream>
#include "TrackedBody.h"

int TrackedBody::instruments[Constants::MAX_INSTRUMENTS];

void TrackedBody::initialize() {
	memset(TrackedBody::instruments, 0, Constants::MAX_INSTRUMENTS * sizeof(int));
}

TrackedBody::TrackedBody(int index, float smoothingFactor, int contourPoints, int noDelayedContours)
{	
	this->index = index;
	this->smoothingFactor = smoothingFactor;
	this->contourPoints = contourPoints;
	this->instrumentId = -1;	
	this->speedShader.load("shaders_gl3/bodySpeed");
	this->blurShader.setup(DEPTH_WIDTH, DEPTH_HEIGHT, 6, .2, 2);
	this->mainFbo.allocate(DEPTH_WIDTH, DEPTH_HEIGHT);
	this->polyFbo.allocate(DEPTH_WIDTH, DEPTH_HEIGHT);

	this->noContours = noDelayedContours;
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
}

void TrackedBody::setOSCManager(ofOSCManager* m)
{
	this->oscManager = m;
	this->bodySoundPlayer->setOscManager(m);
}

void TrackedBody::setBodySoundPlayer(BodySoundPlayer* bsp)
{
	this->bodySoundPlayer = bsp;
	this->bodySoundPlayer->start();
}

void TrackedBody::setTracked(bool isTracked)
{
	if (isTracked != this->isTracked) {
		if (!isTracked) this->removeInstrument();
		else this->assignInstrument();
	}

	this->isTracked = isTracked;
}

void TrackedBody::setContourPoints(int contourPoints)
{
	if (contourPoints != this->contourPoints) {
		this->contourPoints = contourPoints;
		this->contour.clear();
		this->delayedContours.clear();
		this->voronoiPoints.clear();
	}
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
	// If this is not precise enough, can get the area. I think area is
	// more computationally expensive.
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
		const int matchesToCheck = 10;

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
		// float smoothing = ofMap(ct, 0, this->delayedContours.size(), this->smoothingFactor, 0.99);
		float smoothing = ofMap(sqrt(ct), 0, sqrt(this->delayedContours.size()), 0.9, 0.999);
		for (int i = 0; i < delayedContours[ct].size(); i++) {
			int newIndex = (i + this->contourIndexOffset) % this->contour.size();
			delayedContours[ct][i].x = (1 - smoothing) * this->contour[newIndex].x + smoothing * delayedContours[ct][i].x;
			delayedContours[ct][i].y = (1 - smoothing) * this->contour[newIndex].y + smoothing * delayedContours[ct][i].y;
		}
	}
}

void TrackedBody::updateTextureData(ofImage texture)
{
	// TODO (cez): This might be inefficient.
	//this->texture = ofImage(texture);
}

void TrackedBody::setDrawMode(int drawMode)
{
	this->drawMode = drawMode;
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

float TrackedBody::getNormalizedArea() {
	if (this->contour.size() < 4) return 0;
	float area = fabs(this->contour.getArea());
	float unit = this->getJointsDistance(JointType_ShoulderLeft, JointType_ShoulderRight);
	return (area / (unit * unit));
}

float TrackedBody::getJointsAngle(JointType a, JointType b, JointType c)
{
	if (this->joints.find(a) == this->joints.end()) return -1;
	if (this->joints.find(b) == this->joints.end()) return -1;
	if (this->joints.find(c) == this->joints.end()) return -1;
	ofVec2f j1 = this->joints[a]->getPosition();
	ofVec2f j2 = this->joints[b]->getPosition();
	ofVec2f j3 = this->joints[c]->getPosition();
	float angle = BodyUtils::getVectorAngleDeg(j2, j1) - BodyUtils::getVectorAngleDeg(j2, j3);//(360.0f - BodyUtils::getVectorAngleDeg((j3 - j2), (j2 - j1)));
	while (angle < 0) angle += 360;
	while (angle >= 360) angle -= 360;	
	return angle;
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

void TrackedBody::update()
{
	if (!this->isTracked) return;

	for (auto it = this->joints.begin(); it != this->joints.end(); ++it) {
		it->second->update();
	}

	this->bodySoundPlayer->setInterestPoints(this->getInterestPoints());
	this->bodySoundPlayer->setAccentSpeed(this->getJointSpeed(JointType_WristRight));
	this->bodySoundPlayer->setAccentSpeed2(this->getJointSpeed(JointType_WristLeft));
	this->bodySoundPlayer->update();

}

void TrackedBody::drawJointLine(JointType a, JointType b)
{
	if (this->joints.find(a) == this->joints.end()) return;
	if (this->joints.find(b) == this->joints.end()) return;
	ofVec2f p1 = this->joints[a]->getPosition();
	ofVec2f p2 = this->joints[b]->getPosition();
	float distance = p1.distance(p2);
	ofSetLineWidth(400.0f / distance);
	//ofSetLineWidth(10);
	ofDrawLine(p1, p2);
	ofSetLineWidth(1);

}

void TrackedBody::drawJointPolygon(vector<JointType> v)
{
	for (int i = 0; i < v.size(); i++) {
		if (this->joints.find(v[i]) == this->joints.end()) return;
	}
	for (int i = 0; i < v.size() - 1; i++) {
		this->drawJointLine(v[i], v[i + 1]);
		//ofDrawLine(this->joints[v[i]]->getPosition(), this->joints[v[i + 1]]->getPosition());
	}
	this->drawJointLine(v[v.size() - 1], v[0]);	
}

void TrackedBody::drawJointArc(JointType a, JointType b, JointType c)
{	
	if (this->joints.find(a) == this->joints.end()) return;
	if (this->joints.find(b) == this->joints.end()) return;
	if (this->joints.find(c) == this->joints.end()) return;

	ofVec2f center = this->joints[b]->getPosition();
	ofVec2f inner = this->joints[a]->getPosition();
	ofVec2f outer = this->joints[c]->getPosition();
	ofVec2f circleCenter = (inner + outer) / 2;

	ofPolyline curve;	
	ofLogNotice() << center.angleRad(outer) << " " << atan2((outer - center).y, (outer - center).x);
	//float outerAngleDeg = atan2((outer - center).y, (outer - center).x) * 180 / PI;
	//float innerAngleDeg = atan2((inner - center).y, (inner - center).x) * 180 / PI;
	//curve.arc(center.x, center.y, center.distance(inner), center.distance(outer), outerAngleDeg, innerAngleDeg, 50);

	float outerAngleDeg = BodyUtils::getVectorAngleDeg(circleCenter, outer);
	float innerAngleDeg = BodyUtils::getVectorAngleDeg(circleCenter, inner);
	curve.arc(circleCenter.x, circleCenter.y, circleCenter.distance(inner), circleCenter.distance(outer), outerAngleDeg, innerAngleDeg, 50);
	curve.draw();
	
	this->drawJointLine(a, b);
	this->drawJointLine(c, b);
}

void TrackedBody::drawMovement()
{
	if (!this->isTracked) return;
	for (auto it = this->joints.begin(); it != this->joints.end(); ++it) {
		ofVec2f position = it->second->getPosition();
		float speed = max(1.0f, it->second->getSpeed());
		ofDrawCircle(position, speed);
	}
}

void TrackedBody::drawJoints()
{
	if (!this->isTracked) return;
	this->drawJointLine(JointType_WristLeft, JointType_WristRight);
	this->drawJointLine(JointType_ElbowLeft, JointType_ElbowRight);
	this->drawJointLine(JointType_HandLeft, JointType_HandRight);
	this->drawJointLine(JointType_ShoulderLeft, JointType_ShoulderRight);

	vector<JointType> poly1{ JointType_WristLeft, JointType_AnkleLeft, JointType_AnkleRight, JointType_WristRight };
	this->drawJointPolygon(poly1);

	this->drawJointArc(JointType_HipRight, JointType_ShoulderRight, JointType_WristRight);
	this->drawJointArc(JointType_WristLeft, JointType_ShoulderLeft, JointType_HipLeft);
	this->drawJointArc(JointType_HandRight, JointType_ElbowRight, JointType_ShoulderRight);
	this->drawJointArc(JointType_ShoulderLeft, JointType_ElbowLeft, JointType_HandLeft);
	this->drawJointArc(JointType_SpineBase, JointType_SpineMid, JointType_Head);
	this->drawJointLine(JointType_WristLeft, JointType_KneeLeft);
	this->drawJointLine(JointType_WristLeft, JointType_HipLeft);
	this->drawJointLine(JointType_WristLeft, JointType_AnkleLeft);
}

void TrackedBody::drawContours()
{
	if (!this->isTracked) return;
	if (this->contour.size() < 3) return;
	ofPushStyle();
	ofSetColor(255, 225, 128, 255);

	this->contour.draw();
	for (int i = 0; i < this->delayedContours.size(); i++) {
		ofSetColor(255, 225, 128, ofMap(i, 0, this->delayedContours.size(), 255, 0));
		this->delayedContours[i].draw();
	}
	ofPopStyle();
}

void TrackedBody::drawSoundPlayer()
{
	if (!this->isTracked) return;
	this->bodySoundPlayer->draw();
}

void TrackedBody::setJointUniform(JointType joint, string uniformName, ofShader shader) {
	if (this->joints.find(joint) != this->joints.end()) {
		auto j = this->joints[joint];
		shader.setUniform3f(uniformName, j->getPosition().x, j->getPosition().y, 5.0f * j->getSpeed());
	}
}

ofPolyline TrackedBody::getVoronoiPolyline(int bodyInsideCells, bool forceCellsInsideBody) {	
	int noPoints = this->contourPoints;
	if (this->delayedContours.size() == 0) return ofPolyline();
	ofPolyline resampled = this->delayedContours[0];//.getResampledByCount(noPoints);
	ofPolyline originalPolyline = ofPolyline(resampled);

	
	if (this->voronoiPoints.size() != bodyInsideCells) {
		this->voronoiPoints.clear();
		for (int i = 0; i < bodyInsideCells; i++) {
			int id1 = (int)ofRandom(noPoints);
			int id2 = (int)ofRandom(noPoints);
			this->voronoiPoints.push_back(make_pair(make_pair(id1, id2), ofRandom(1.0)));
		}
	}

	for (int i = 0; i < this->voronoiPoints.size(); i++) {
		ofPoint p1 = resampled.getVertices()[this->voronoiPoints[i].first.first];
		ofPoint p2 = resampled.getVertices()[this->voronoiPoints[i].first.second];
		ofPoint p3 = (p2 - p1) * this->voronoiPoints[i].second + p1;
		if (forceCellsInsideBody) {
			if (originalPolyline.inside(p3)) {
				resampled.addVertex(p3.x, p3.y);
			}
		}
		else {
			resampled.addVertex(p3.x, p3.y);
		}
	}
		
	return resampled;
}

vector<ofVec2f> TrackedBody::getInterestPoints()
{
	vector<ofVec2f> interestPoints;

	float leftRightDistance = this->getNormalizedJointsDistance(JointType_WristLeft, JointType_WristRight);
	float topBottomDistance = this->getNormalizedJointsDistance(JointType_Head, JointType_AnkleLeft);

	vector<JointType> interestJoints;
	interestJoints.push_back(JointType_SpineBase);
	interestJoints.push_back(JointType_Head);

	if (leftRightDistance >= 0.2f) {
		interestJoints.push_back(JointType_ShoulderLeft);
		interestJoints.push_back(JointType_ShoulderRight);
	}

	if (leftRightDistance >= 0.325f) {
		interestJoints.push_back(JointType_WristLeft);
		interestJoints.push_back(JointType_WristRight);
	}

	if (leftRightDistance >= 0.4f) {
		interestJoints.push_back(JointType_ElbowLeft);
		interestJoints.push_back(JointType_ElbowRight);
	}

	if (topBottomDistance >= 0.25f) {
		interestJoints.push_back(JointType_AnkleLeft);
		interestJoints.push_back(JointType_AnkleRight);
	}

	if (topBottomDistance >= 0.375f) {
		interestJoints.push_back(JointType_SpineMid);
	}

	if (topBottomDistance >= 0.5f) {
		interestJoints.push_back(JointType_KneeLeft);
		interestJoints.push_back(JointType_KneeRight);
	}

	/*
	vector<JointType> interestJoints = { 
		JointType_Head,
		//JointType_ShoulderLeft,
		//JointType_ShoulderRight,
		JointType_ElbowLeft,
		JointType_ElbowRight,
		JointType_WristLeft,
		JointType_WristRight,
		JointType_KneeLeft,
		JointType_KneeRight,		
		JointType_HandLeft,
		JointType_HandRight,
		JointType_Neck,
		JointType_SpineMid,
		JointType_SpineBase,
		JointType_HipLeft,
		JointType_HipRight,
		JointType_FootLeft,
		JointType_FootRight
	};
	*/

	for (auto it = interestJoints.begin(); it != interestJoints.end(); ++it) {
		if (this->joints.find(*it) != this->joints.end())
			interestPoints.push_back(this->joints[*it]->getPosition());
	}
	
	return interestPoints;
}

void TrackedBody::drawRaster()
{
	if (!this->isTracked) return;
	if (this->contour.size() < 3) return;
		
	this->polyFbo.begin();
	ofClear(0, 0, 0, 1);
	this->contourPath.clear();
	//this->contourPath.setMode(ofPath::POLYLINES);
	this->contourPath.moveTo(this->contour[0]);
	for (int i = 1; i < this->contour.size(); i++)
		this->contourPath.lineTo(this->contour[i]);
	this->contourPath.close();

	this->contourPath.setFilled(true);
	this->contourPath.setFillColor(ofColor(255, 128, 128));
	this->contourPath.draw(0, 0);
	this->polyFbo.end();


	this->mainFbo.begin();
	ofClear(0, 0, 0, 3);

	this->speedShader.begin();
	this->setJointUniform(JointType_WristLeft, "vLeftWrist", this->speedShader);
	this->setJointUniform(JointType_WristRight, "vRightWrist", this->speedShader);
	this->setJointUniform(JointType_ShoulderLeft, "vLeftShoulder", this->speedShader);
	this->setJointUniform(JointType_ShoulderRight, "vRightShoulder", this->speedShader);
	this->setJointUniform(JointType_HipLeft, "vLeftHip", this->speedShader);
	this->setJointUniform(JointType_HipRight, "vRightHip", this->speedShader);
	this->setJointUniform(JointType_Head, "vHead", this->speedShader);

	float time = ofGetSystemTimeMillis();
	this->speedShader.setUniform1f("uTime", time);

	float hueShift = ofClamp(this->index / 10.0, 0.05, 0.95);
	this->speedShader.setUniform1f("uHueShiftParam", hueShift);

	float rightArmAngle = this->getJointsAngle(JointType_HipRight, JointType_ShoulderRight, JointType_WristRight);
	float leftArmAngle = this->getJointsAngle(JointType_WristLeft, JointType_ShoulderLeft, JointType_HipLeft);
	this->speedShader.setUniform1f("uArmsParam", (leftArmAngle + rightArmAngle + 20) / (2.0 * 180));

	this->polyFbo.draw(0, 0);

	this->speedShader.end();
	this->mainFbo.end();

	ofPushMatrix();
	//ofScale(2.0);
	this->blurShader.begin();
	this->mainFbo.draw(0, 0);
	this->blurShader.end();
	this->blurShader.draw();	

	ofPopMatrix();
}

void TrackedBody::draw()
{
	this->updateDelayedContours();
	if (this->drawMode & BDRAW_MODE_RASTER) this->drawRaster();
	if (this->drawMode & BDRAW_MODE_JOINTS) this->drawJoints();
	if (this->drawMode & BDRAW_MODE_MOVEMENT) this->drawMovement();
	if (this->drawMode & BDRAW_MODE_CONTOUR) this->drawContours();
	if (this->drawMode & BDRAW_MODE_SOUND) this->drawSoundPlayer();
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
	----
	*/
	stringstream ss;
	ss << this->index << "\n";
	ss << Constants::SKELETON_DELIMITER << "\n";
	int noJoints = 0;
	for (auto it = this->latestSkeleton.begin(); it != this->latestSkeleton.end(); ++it) {
		JointType currentJoint = it->first;
		if (this->latestSkeleton[currentJoint].getTrackingState() == TrackingState_Tracked || this->latestSkeleton[currentJoint].getTrackingState() == TrackingState_Inferred) {
			noJoints++;
		}
	}
	ss << noJoints << "\n";

	for (auto it = this->latestSkeleton.begin(); it != this->latestSkeleton.end(); ++it) {
		JointType currentJoint = it->first;
		if (this->latestSkeleton[currentJoint].getTrackingState() == TrackingState_Tracked || this->latestSkeleton[currentJoint].getTrackingState() == TrackingState_Inferred) {
			ofVec2f position = it->second.getProjected(this->coordinateMapper, ofxKFW2::ProjectionCoordinates::DepthCamera);
			ss << currentJoint << " " << position.x << " " << position.y << "\n";
		}
	}

	ss << Constants::CONTOUR_DELIMITER << "\n";
	// Resample contour we're sending in order to save bandwidth
	//this->rawContour.getResampledByCount(100);
	ss << this->rawContour.size() << "\n";
	for (int i = 0; i < this->rawContour.size(); i++) {
		ss << this->rawContour[i].x << " " << this->rawContour[i].y << "\n";
	}

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
}

void TrackedBody::sendOSCData()
{
	float value;
	float normalizedValue;
	// Sequencer sound data
	this->bodySoundPlayer->sendOSC(this->instrumentId);

	// Distances
	value = this->getNormalizedJointsDistance(JointType_WristLeft, JointType_WristRight);
	normalizedValue = ofMap(value, 0, 1, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::DISTANCE, "l-hand-r-hand", normalizedValue);

	value = this->getNormalizedJointsDistance(JointType_WristLeft, JointType_KneeLeft);
	normalizedValue = ofMap(value, 0, 1, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::DISTANCE, "l-hand-l-knee", normalizedValue);

	value = this->getNormalizedJointsDistance(JointType_KneeRight, JointType_WristRight);
	normalizedValue = ofMap(value, 0, 1, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::DISTANCE, "r-hand-r-knee", normalizedValue);

	value = this->getNormalizedJointsDistance(JointType_Head, JointType_AnkleLeft);
	normalizedValue = ofMap(value, 0, 1, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::DISTANCE, "l-knee-r-knee", normalizedValue);

	value = this->getNormalizedArea();
	normalizedValue = ofMap(value, 0, 15, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::DISTANCE, "body-area", normalizedValue);

	// Angles
	value = (this->getJointsAngle(JointType_HipRight, JointType_ShoulderRight, JointType_WristRight) + 30);
	if (value >= 360) value -= 360;
	normalizedValue = ofMap(value, 0, 360, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::ANGLE, "r-arm", normalizedValue);

	value = (this->getJointsAngle(JointType_WristLeft, JointType_ShoulderLeft, JointType_HipLeft) + 30);
	if (value >= 360) value -= 360;
	normalizedValue = ofMap(value, 0, 360, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::ANGLE, "l-arm", normalizedValue);

	value = this->getJointsAngle(JointType_HandRight, JointType_ElbowRight, JointType_ShoulderRight) + 10;
	if (value >= 360) value -= 360;
	normalizedValue = ofMap(180 - value, 0, 180, 0, 1023, true);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::ANGLE, "r-elbow", normalizedValue);

	value = this->getJointsAngle(JointType_ShoulderLeft, JointType_ElbowLeft, JointType_HandLeft) + 10;
	if (value >= 360) value -= 360;
	normalizedValue = ofMap(180 - value, 0, 180, 0, 1023, true);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::ANGLE, "l-elbow", normalizedValue);

	value = this->getJointsAngle(JointType_AnkleLeft, JointType_KneeLeft, JointType_HipLeft) + 20;
	if (value >= 360) value -= 360;
	normalizedValue = ofMap(180 - value, 0, 180, 0, 1023, true);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::ANGLE, "l-knee", normalizedValue);

	value = this->getJointsAngle(JointType_HipRight, JointType_KneeRight, JointType_AnkleRight) + 20;
	if (value >= 360) value -= 360;
	normalizedValue = ofMap(180 - value, 0, 180, 0, 1023, true);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::ANGLE, "r-knee", normalizedValue);

	value = this->getJointsAngle(JointType_KneeLeft, JointType_SpineBase, JointType_KneeRight) + 15;
	if (value >= 360) value -= 360;
	normalizedValue = ofMap(value, 0, 180, 0, 1023, true);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::ANGLE, "legs", normalizedValue);

	// Movements	
	value = this->getJointNormalizedSpeed(JointType_WristLeft);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "l-hand", normalizedValue);

	value = this->getJointNormalizedSpeed(JointType_WristRight);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "r-hand", normalizedValue);

	value = this->getJointNormalizedSpeed(JointType_AnkleLeft);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "l-foot", normalizedValue);

	value = this->getJointNormalizedSpeed(JointType_AnkleRight);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "r-foot", normalizedValue);

	value = this->getJointNormalizedSpeed(JointType_HipLeft);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "l-hip", normalizedValue);

	value = this->getJointNormalizedSpeed(JointType_HipRight);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "r-hip", normalizedValue);

	value = this->getJointNormalizedSpeed(JointType_KneeLeft);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "l-knee", normalizedValue);

	value = this->getJointNormalizedSpeed(JointType_KneeRight);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "r-knee", normalizedValue);

	value = this->getJointNormalizedSpeed(JointType_ShoulderLeft);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "l-shoulder", normalizedValue);

	value = this->getJointNormalizedSpeed(JointType_ShoulderRight);
	normalizedValue = ofMap(value, 0, 60, 0, 1023);
	this->oscManager->sendBodyMessage(this->instrumentId, OscCategories::MOVEMENT, "r-shoulder", normalizedValue);

}
