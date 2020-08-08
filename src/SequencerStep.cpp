#include "SequencerStep.h"

SequencerStep::SequencerStep()
{
	this->x = this->y = this->size = this->clipSize = 0;
	this->strokeColor = ofColor(0, 0, 0, 0);
	this->highlightColor = ofColor(0, 0, 0, 0);
	this->bodies.clear();
	this->initializeClipSizes();
}

SequencerStep::SequencerStep(float x, float y, float size, JointType joint, ofColor strokeColor, ofColor highlightColor)
{
	this->x = x;
	this->y = y;
	this->size = size;
	this->joint = joint;
	this->strokeColor = strokeColor;
	this->highlightColor = highlightColor;
	this->bodies.clear();
	// Eventually this will become specific for each joint
	this->initializeClipSizes();
	if (this->clipSizes.find(joint) != this->clipSizes.end())
		this->clipSize = this->clipSizes[joint];
	else
		this->clipSize = 400;
}

void SequencerStep::registerBody(TrackedBody* body, ofColor strokeColor, ofColor fillColor)
{
	this->paths.clear();
	this->bodies.clear();
	BodyCapture b = BodyCapture(body, this->joint, strokeColor, fillColor);
	this->bodies.push_back(b);
}

void SequencerStep::update()
{
	for (auto& bc : this->bodies) {
		TrackedBody* body = bc.body;
		ofVec2f clipPosition = body->getJointPosition(bc.joint);

		this->clipper.Clear();
		this->clipper.addPolyline(body->contour, ClipperLib::ptSubject);
		
		float normalizedClipSize = this->clipSize * body->getScreenRatio();

		ofRectangle rect = ofRectangle(clipPosition.x - normalizedClipSize / 2, clipPosition.y - normalizedClipSize / 2, normalizedClipSize, normalizedClipSize);
		this->clipper.addRectangle(rect, ClipperLib::ptClip);

		auto intersection = clipper.getClipped(ClipperLib::ClipType::ctIntersection);
		glm::vec2 lineOffset = clipPosition - ofVec2f(normalizedClipSize / 2, normalizedClipSize / 2);
		
		this->currentPath.clear();
		for (auto& line : intersection) {
			this->currentPath.moveTo(line[0]);
			for (int i = 1; i < line.size(); i++) {
				this->currentPath.lineTo(line[i]);
			}
			this->currentPath.close();
		}
		this->currentPath.translate(-lineOffset);
		float scale = (1.0 * this->size) / (1.0 * normalizedClipSize);
		this->currentPath.scale(scale, scale);

		this->paths.push_back(this->currentPath);
	}
}

void SequencerStep::draw(float x, float y, bool isHighlighted) {
	this->x = x;
	this->y = y;
	this->draw(isHighlighted);
}

void SequencerStep::initializeClipSizes()
{
	this->clipSizes[JointType_Head] = 550.0;
	this->clipSizes[JointType_SpineMid] = 900.0;
	this->clipSizes[JointType_SpineBase] = 900.0;
	this->clipSizes[JointType_ShoulderLeft] = 500.0;
	this->clipSizes[JointType_ShoulderRight] = 500.0;
}

void SequencerStep::draw(bool isHighlighted)
{
	ofPushStyle();
	ofPushMatrix();
	ofTranslate(this->x, this->y);

	// Draw paths
	for (int i = 0; i < this->paths.size(); i++) {
		if (this->bodies[i].fillColor.a != 0) {
			this->paths[i].setFillColor(this->bodies[i].fillColor);
			this->paths[i].setFilled(true);
			this->paths[i].draw();
		}
		if (this->bodies[i].strokeColor.a != 0) {
			this->paths[i].setStrokeColor(isHighlighted ? this->highlightColor : this->bodies[i].strokeColor);
			this->paths[i].setStrokeWidth(1);
			this->paths[i].setColor(isHighlighted ? this->highlightColor : this->bodies[i].strokeColor);
			this->paths[i].setFilled(false);
			this->paths[i].draw();
		}
	}

	// Draw container on top
	ofSetColor(isHighlighted ? this->highlightColor : this->strokeColor);
	ofNoFill();
	ofDrawRectangle(0, 0, this->size, this->size);


	ofPopMatrix();
	ofPopStyle();
}
