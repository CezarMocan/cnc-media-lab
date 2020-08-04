#include "SequencerStep.h"

SequencerStep::SequencerStep()
{
	this->x = this->y = this->size = this->clipSize = 0;
	this->strokeColor = ofColor(0, 0, 0, 0);
	this->highlightColor = ofColor(0, 0, 0, 0);
	this->bodies.clear();
}

SequencerStep::SequencerStep(float x, float y, float size, JointType joint, ofColor strokeColor, ofColor highlightColor)
{
	this->x = x;
	this->y = y;
	this->size = size;
	this->joint = joint;
	this->strokeColor = strokeColor;
	this->highlightColor = highlightColor;

	// Eventually this will become specific for each joint
	this->clipSize = 50.0;
}

void SequencerStep::registerBody(TrackedBody* body, ofColor strokeColor, ofColor fillColor)
{
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
		
		ofRectangle rect = ofRectangle(clipPosition.x - clipSize / 2, clipPosition.y - clipSize / 2, clipSize, clipSize);
		this->clipper.addRectangle(rect, ClipperLib::ptClip);

		auto intersection = clipper.getClipped(ClipperLib::ClipType::ctIntersection);
		glm::vec2 lineOffset = clipPosition - ofVec2f(clipSize / 2, clipSize / 2);

		/*
		for (auto& line : intersection) {
			for (auto& point : line) {
				point = point - lineOffset;
				float scale = (1.0 * this->size) / (1.0 * this->clipSize);
				point.x *= scale; point.y *= scale;
			}
		}
		*/
		
		this->currentPath.clear();
		for (auto& line : intersection) {
			this->currentPath.moveTo(line[0]);
			for (int i = 1; i < line.size(); i++) {
				this->currentPath.lineTo(line[i]);
			}
			this->currentPath.close();
		}
		this->currentPath.translate(-lineOffset);
		float scale = (1.0 * this->size) / (1.0 * this->clipSize);
		this->currentPath.scale(scale, scale);

		this->paths.push_back(this->currentPath);
	}
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

	// At the very end, clean everything up for next frame
	this->bodies.clear();
	this->paths.clear();
}
