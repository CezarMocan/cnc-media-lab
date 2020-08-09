#include "Sequencer.h"

Sequencer::Sequencer()
{
	this->x = this->y = this->elementsPerRow = this->padding = this->elementSize = this->highlightedStep = 0;
}

Sequencer::Sequencer(int x, int y, int elementsPerRow, int elementSize, int padding, ofColor color, ofColor accentColor, ofColor highlightColor)
{
	this->x = x;
	this->y = y;
	this->elementsPerRow = elementsPerRow;
	this->elementSize = elementSize;
	this->padding = padding;
	this->color = color;
	this->accentColor = accentColor;
	this->highlightColor = highlightColor;
	this->highlightedStep = 0;
	this->trackedBody = NULL;
}

void Sequencer::addSequencerStepForJoint(JointType j)
{
	SequencerStep* newStep = new SequencerStep(0, 0, this->elementSize, j, this->color, this->highlightColor);
	this->steps[j] = newStep;
}

void Sequencer::addSequencerStepForJoints(vector<JointType> v)
{
	for (auto it = v.begin(); it != v.end(); ++it) {
		this->addSequencerStepForJoint(*it);
	}
}

ofVec2f Sequencer::getPositionForIndex(int index)
{
	int x, y;
	
	int row = index / this->elementsPerRow;
	y = this->y + row * (this->elementSize + this->padding);
	
	int column = index % this->elementsPerRow;
	x = this->x + column * (this->elementSize + this->padding);

	return ofVec2f(x, y);
}

void Sequencer::addSequencerStep(SequencerStep* s)
{
	this->steps[s->joint] = s;
}

void Sequencer::setStepOrder(vector<JointType> order)
{
	this->stepOrder = order;
}

void Sequencer::setTrackedBody(TrackedBody* b)
{
	this->trackedBody = b;
}

void Sequencer::setCurrentHighlight(int highlightedStep)
{
	this->highlightedStep = highlightedStep;
}

void Sequencer::update()
{
	if (this->trackedBody == NULL) return;

	this->setStepOrder(this->trackedBody->getCurrentlyPlayingJoints());
	if (this->trackedBody->getCurrentlyPlaying16Joints().size() > this->highlightedStep)
		this->highlightedJoint = this->trackedBody->getCurrentlyPlaying16Joints()[this->highlightedStep];

	for (auto const &it : this->steps) {
		JointType joint = it.first;
		SequencerStep* step = it.second;	
		step->registerBody(this->trackedBody, this->color, this->accentColor);
		step->update();
	}
}

void Sequencer::draw()
{
	int index = 0;
	if (this->stepOrder.size() > 0) {
		for (auto it = this->stepOrder.begin(); it != this->stepOrder.end(); ++it) {
			JointType j = static_cast<JointType>(*it);
			if (this->steps.find(j) == this->steps.end()) continue;
			SequencerStep* step = this->steps[j];
			ofVec2f position = this->getPositionForIndex(index);
			bool isHighlighted = (j == this->highlightedJoint);
			step->draw(position.x, position.y, isHighlighted);
			index++;
		}
	}
	else {
		ofPushStyle();
		for (int index = 0; index < 16; index++) {
			ofVec2f position = this->getPositionForIndex(index);
			ofNoFill();
			ofSetColor(this->color);
			ofDrawRectangle(position.x, position.y, this->elementSize, this->elementSize);
		}
		ofPopStyle();
	}
}
