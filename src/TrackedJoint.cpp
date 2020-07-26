#include "TrackedJoint.h"

TrackedJoint::TrackedJoint(TrackedJoint* j)
{
	this->type = j->type;
	this->initialized = j->initialized;
	this->smoothingFactor = j->smoothingFactor;
	this->currentPosition = j->currentPosition;
	this->targetPosition = j->currentPosition;
	this->velocity = j->velocity;
}

TrackedJoint::TrackedJoint(JointType type)
{
	this->type = type;
	this->initialized = false;
	this->smoothingFactor = 0.0f;
	this->currentPosition = ofVec2f(0.0f, 0.0f);
	this->targetPosition = ofVec2f(0.0f, 0.0f);
}

void TrackedJoint::setPosition(ofVec2f pos, float smoothing)
{
	if (!this->initialized) {
		this->initialized = true;
		this->currentPosition = pos;
		
	}
	this->smoothingFactor = smoothing;
	this->targetPosition = pos;
}

ofVec2f TrackedJoint::getPosition()
{
	return this->currentPosition;
}

ofVec2f TrackedJoint::getTargetPosition()
{
	return this->targetPosition;
}

ofVec2f TrackedJoint::getVelocity()
{
	return this->velocity;
}

float TrackedJoint::getSpeed()
{
	return this->velocity.length();
}

void TrackedJoint::update()
{
	if (!this->initialized)	return;
	ofVec2f newPosition = this->targetPosition * (1 - this->smoothingFactor) + this->currentPosition * this->smoothingFactor;
	this->updateVelocity(this->currentPosition, newPosition);
	this->currentPosition = newPosition;
}

void TrackedJoint::updateVelocity(ofVec2f oldPosition, ofVec2f newPosition)
{
	float velocitySmoothing = 0;
	this->velocity = (1 - this->smoothingFactor) * (newPosition - oldPosition) + this->velocity * this->smoothingFactor;
}
