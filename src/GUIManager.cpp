#include "GUIManager.h"

GUIManager::GUIManager()
{
	// Load fonts
	fontRegular.load("fonts/be-ag-light.ttf", Layout::FONT_SIZE);
	fontBold.load("fonts/be-ag-medium.ttf", Layout::FONT_SIZE - 2);

	// Sequencer UI
	sequencerLeft = new Sequencer(Layout::FRAME_PADDING, Layout::FRAME_PADDING,
		Layout::SEQUENCER_ROW_SIZE, Layout::SEQUENCER_ELEMENT_SIZE,
		Layout::FRAME_PADDING,
		Colors::BLUE, Colors::BLUE_ACCENT, Colors::YELLOW);

	sequencerLeft->addSequencerStepForJoints({
		JointType_HandLeft, JointType_ElbowLeft, JointType_ShoulderLeft,
		JointType_Head, JointType_ShoulderRight, JointType_ElbowRight,
		JointType_HandRight, JointType_SpineBase,
		JointType_AnkleLeft, JointType_AnkleRight,
		JointType_SpineMid,
		JointType_KneeLeft, JointType_KneeRight,
		});

	int srX = DEPTH_WIDTH - (Layout::FRAME_PADDING + Layout::SEQUENCER_ELEMENT_SIZE) * Layout::SEQUENCER_ROW_SIZE;
	sequencerRight = new Sequencer(srX, Layout::FRAME_PADDING,
		Layout::SEQUENCER_ROW_SIZE, Layout::SEQUENCER_ELEMENT_SIZE,
		Layout::FRAME_PADDING,
		Colors::RED, Colors::RED_ACCENT, Colors::YELLOW);
	sequencerRight->addSequencerStepForJoints({
		JointType_HandLeft, JointType_ElbowLeft, JointType_ShoulderLeft,
		JointType_Head, JointType_ShoulderRight, JointType_ElbowRight,
		JointType_HandRight, JointType_SpineBase,
		JointType_AnkleLeft, JointType_AnkleRight,
		JointType_SpineMid,
		JointType_KneeLeft, JointType_KneeRight,
		});
}

void GUIManager::update(TrackedBody* leftBody, TrackedBody* rightBody, int currentSequencerStep, bool isConnected, string latency)
{
	this->leftBody = leftBody;
	this->rightBody = rightBody;
	this->currentSequencerStep = currentSequencerStep;
	this->isConnected = isConnected;
	this->latency = latency;

	this->updateSequencer();
	this->updateBackgroundContours();
}

void GUIManager::updateSequencer()
{
	this->sequencerLeft->setTrackedBody(this->leftBody);
	this->sequencerRight->setTrackedBody(this->rightBody);

	this->sequencerLeft->setCurrentHighlight(this->currentSequencerStep);
	this->sequencerLeft->update();

	this->sequencerRight->setCurrentHighlight(this->currentSequencerStep);
	this->sequencerRight->update();
}

void GUIManager::drawSequencer()
{
	this->sequencerLeft->draw();
	this->sequencerRight->draw();
}

void GUIManager::updateBackgroundContours()
{	
	ofVec2f winSize = ofGetWindowSize() / 2.0;
	ofVec2f padding = ofVec2f(25, 25);

	if (leftBody != NULL) {
		int framesOnPosition = 2;
		int pathStart = (ofGetFrameNum() % (framesOnPosition * 1000)) / framesOnPosition;
		int noPoints = 50;
		this->leftBackgroundContour = leftBody->getContourSegment(pathStart, noPoints);
		this->leftBackgroundContour.first->translate(glm::vec2(-this->leftBackgroundContour.second.x, -this->leftBackgroundContour.second.y));
		this->leftBackgroundContour.first->scale((winSize.x / 2 - 2 * padding.x) / this->leftBackgroundContour.second.width, ((winSize.y - 2 * padding.y) / this->leftBackgroundContour.second.height));
		this->leftBackgroundContour.first->translate(glm::vec2(winSize.x / 2 + padding.x / 2.0 - 5, padding.y / 2.0 - 5));
	}

	if (rightBody != NULL) {
		int framesOnPosition = 2;
		int pathStart = (ofGetFrameNum() % (framesOnPosition * 1000)) / framesOnPosition;
		int noPoints = 50;
		this->rightBackgroundContour = rightBody->getContourSegment(pathStart, noPoints);
		this->rightBackgroundContour.first->translate(glm::vec2(-this->rightBackgroundContour.second.x, -this->rightBackgroundContour.second.y));
		this->rightBackgroundContour.first->scale((winSize.x / 2 - 2 * padding.x) / this->rightBackgroundContour.second.width, ((winSize.y - 2 * padding.y) / this->rightBackgroundContour.second.height));
		this->rightBackgroundContour.first->translate(glm::vec2(padding.x / 2.0 - 5, padding.y / 2.0 - 5));
	}
}

void GUIManager::drawBackgroundContours()
{
	if (this->leftBackgroundContour.first != NULL) {
		this->leftBackgroundContour.first->setFilled(true);
		this->leftBackgroundContour.first->setColor(Colors::BLUE_TRANSPARENT);
		this->leftBackgroundContour.first->draw();

		this->leftBackgroundContour.first->setFilled(false);
		this->leftBackgroundContour.first->setStrokeColor(Colors::BLUE_TRANSPARENT);
		this->leftBackgroundContour.first->setStrokeWidth(1.);
		this->leftBackgroundContour.first->draw();
	}

	if (this->rightBackgroundContour.first != NULL) {
		this->rightBackgroundContour.first->setFilled(true);
		this->rightBackgroundContour.first->setColor(Colors::RED_TRANSPARENT);
		this->rightBackgroundContour.first->draw();

		this->rightBackgroundContour.first->setFilled(false);
		this->rightBackgroundContour.first->setStrokeColor(Colors::RED_TRANSPARENT);
		this->rightBackgroundContour.first->setStrokeWidth(1.);
		this->rightBackgroundContour.first->draw();
	}
}

void GUIManager::drawSystemStatus()
{
	float width, totalWidth;
	ofPushStyle();
	ofSetColor(Colors::YELLOW);

	// Status	
	width = fontBold.stringWidth("Status_ ");
	fontBold.drawString("Status_ ", Layout::WINDOW_PADDING, Layout::WINDOW_PADDING - 7);
	string status = isConnected ? "Connected" : "Not Connected";
	fontRegular.drawString(status, Layout::WINDOW_PADDING + width, Layout::WINDOW_PADDING - 7);

	// IP 1
	width = fontBold.stringWidth("IP_1_ ");
	totalWidth = width + fontRegular.stringWidth(Constants::CEZAR_IP);
	fontBold.drawString("IP_1_ ", (ofGetWindowWidth() - totalWidth) / 2, Layout::WINDOW_PADDING - 7);
	fontRegular.drawString(Constants::CEZAR_IP, (ofGetWindowWidth() - totalWidth) / 2 + width, Layout::WINDOW_PADDING - 7);

	// IP 2
	width = fontBold.stringWidth("IP_2_ ");
	totalWidth = width + fontRegular.stringWidth(Constants::CY_IP);
	fontBold.drawString("IP_2_ ", ofGetWindowWidth() - Layout::WINDOW_PADDING - totalWidth, Layout::WINDOW_PADDING - 7);
	fontRegular.drawString(Constants::CY_IP, ofGetWindowWidth() - Layout::WINDOW_PADDING - totalWidth + width, Layout::WINDOW_PADDING - 7);

	// Instrument 1
	if (leftBody != NULL) {
		int sz = Instruments::INSTRUMENT_LIST.size();
		string instrument1 = Instruments::INSTRUMENT_LIST[leftBody->getInstrumentId() % sz];
		ofPushMatrix();
		ofRotateDeg(270);
		width = fontBold.stringWidth("Instrument_1_ ");
		totalWidth = width + fontRegular.stringWidth(instrument1);
		fontBold.drawString("Instrument_1_ ", -(Layout::WINDOW_PADDING + totalWidth), Layout::WINDOW_PADDING - 7);
		fontRegular.drawString(instrument1, -(Layout::WINDOW_PADDING + totalWidth - width), Layout::WINDOW_PADDING - 7);
		ofPopMatrix();
	}

	if (rightBody != NULL) {
		// Instrument 2
		int sz = Instruments::INSTRUMENT_LIST.size();
		string instrument2 = Instruments::INSTRUMENT_LIST[rightBody->getInstrumentId() % sz];
		ofPushMatrix();
		ofRotateDeg(90);
		width = fontBold.stringWidth("Instrument_2_ ");
		fontBold.drawString("Instrument_2_ ", Layout::WINDOW_PADDING, -(ofGetWindowWidth() - Layout::WINDOW_PADDING + 7));
		fontRegular.drawString(instrument2, Layout::WINDOW_PADDING + width, -(ofGetWindowWidth() - Layout::WINDOW_PADDING + 7));
		ofPopMatrix();
	}

	// Latency
	width = fontBold.stringWidth("Latency_ ");
	totalWidth = width + fontRegular.stringWidth(latency);
	fontBold.drawString("Latency_ ", (ofGetWindowWidth() - totalWidth) / 2, ofGetWindowHeight() - Layout::WINDOW_PADDING + 15);
	fontRegular.drawString(latency, (ofGetWindowWidth() - totalWidth) / 2 + width, ofGetWindowHeight() - Layout::WINDOW_PADDING + 15);

	ofPopStyle();
}

void GUIManager::drawBodyTrackedStatus()
{
	float currentScale = Layout::WINDOW_SCALE;
	int bottom = ofGetWindowHeight() / currentScale - Layout::WINDOW_PADDING;

	// Left body status
	int leftX = Layout::FRAME_PADDING;
	int leftY = bottom - Layout::FRAME_PADDING - Layout::SEQUENCER_ELEMENT_SIZE;
	int squareSize = Layout::SEQUENCER_ELEMENT_SIZE;
	int circleRadius = (Layout::SEQUENCER_ELEMENT_SIZE - 6) / 2;

	ofPushStyle();
	ofSetColor(Colors::BACKGROUND);
	ofFill();
	ofDrawRectangle(leftX, leftY, squareSize, squareSize);
	ofSetColor(Colors::YELLOW);
	ofNoFill();
	ofDrawRectangle(leftX, leftY, squareSize, squareSize);

	if (leftBody != NULL) {
		ofSetColor(Colors::BLUE_ACCENT);
		ofFill();
		ofDrawCircle(leftX + squareSize / 2, leftY + squareSize / 2, circleRadius);
	}

	ofNoFill();
	ofSetColor(Colors::YELLOW);
	ofDrawCircle(leftX + squareSize / 2, leftY + squareSize / 2, circleRadius);
	ofPopStyle();

	// Right body status
	int rightX = ofGetWindowWidth() / currentScale - Layout::WINDOW_PADDING - Layout::FRAME_PADDING - Layout::SEQUENCER_ELEMENT_SIZE;
	int rightY = leftY;

	ofPushStyle();
	ofSetColor(Colors::BACKGROUND);
	ofFill();
	ofDrawRectangle(rightX, rightY, squareSize, squareSize);
	ofSetColor(Colors::YELLOW);
	ofNoFill();
	ofDrawRectangle(rightX, rightY, squareSize, squareSize);

	if (rightBody != NULL) {
		ofSetColor(Colors::RED);
		ofFill();
		ofDrawCircle(rightX + squareSize / 2, rightY + squareSize / 2, circleRadius);
	}

	ofNoFill();
	ofSetColor(Colors::YELLOW);
	ofDrawCircle(rightX + squareSize / 2, rightY + squareSize / 2, circleRadius);
	ofPopStyle();
}

void GUIManager::drawFrequencyGradient()
{
	float currentScale = Layout::WINDOW_SCALE;
	int bottom = ofGetWindowHeight() / currentScale - Layout::WINDOW_PADDING;
	int leftX = Layout::FRAME_PADDING * 2 + Layout::SEQUENCER_ELEMENT_SIZE;
	int leftY = bottom - Layout::FRAME_PADDING - Layout::SEQUENCER_ELEMENT_SIZE;
	int width = ofGetWindowWidth() / currentScale - 2 * (Layout::WINDOW_PADDING / currentScale + Layout::SEQUENCER_ELEMENT_SIZE + 2 * Layout::FRAME_PADDING);
	int height = Layout::SEQUENCER_ELEMENT_SIZE;

	// Big rectangle gradient
	this->frequencyGradient.clear();
	this->frequencyGradient.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
	this->frequencyGradient.addVertex(ofPoint(leftX, leftY));
	this->frequencyGradient.addColor(Colors::BACKGROUND);
	this->frequencyGradient.addVertex(ofPoint(leftX + width / 2, leftY));
	this->frequencyGradient.addColor(Colors::YELLOW);
	this->frequencyGradient.addVertex(ofPoint(leftX, leftY + height));
	this->frequencyGradient.addColor(Colors::BACKGROUND);
	this->frequencyGradient.addVertex(ofPoint(leftX + width / 2, leftY + height));
	this->frequencyGradient.addColor(Colors::YELLOW);
	this->frequencyGradient.draw();
	this->frequencyGradient.clear();
	this->frequencyGradient.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
	this->frequencyGradient.addVertex(ofPoint(leftX + width / 2, leftY));
	this->frequencyGradient.addColor(Colors::YELLOW);
	this->frequencyGradient.addVertex(ofPoint(leftX + width, leftY));
	this->frequencyGradient.addColor(Colors::BACKGROUND);
	this->frequencyGradient.addVertex(ofPoint(leftX + width / 2, leftY + height));
	this->frequencyGradient.addColor(Colors::YELLOW);
	this->frequencyGradient.addVertex(ofPoint(leftX + width, leftY + height));
	this->frequencyGradient.addColor(Colors::BACKGROUND);
	this->frequencyGradient.draw();

	// 0Hz -> 5KHz text
	ofPushMatrix();
	ofPushStyle();
	ofSetColor(Colors::YELLOW);
	ofScale(1.0 / currentScale);
	fontRegular.drawString("0Hz", currentScale * leftX, currentScale * (leftY - 3.5));
	int textWidth = fontRegular.stringWidth("2kHz");
	fontRegular.drawString("2kHz", currentScale * (leftX + width) - textWidth, currentScale * (leftY - 3.5));
	ofPopStyle();
	ofPopMatrix();

	// Frequency indicator for left body
	int indicatorPadding = 4;
	int indicatorHeight = Layout::SEQUENCER_ELEMENT_SIZE - 2 * indicatorPadding;
	int indicatorWidth = indicatorHeight / 2;
	int maxFreq = 2000;

	if (leftBody != NULL) {
		vector<float> freqs = leftBody->getCurrentlyPlaying16Frequencies();
		int index = this->currentSequencerStep;
		if (freqs.size() > index) {
			float frequency = freqs[index];
			int indicatorX = leftX + ofMap(frequency, 0, maxFreq, 0, width);
			int indicatorY = leftY + indicatorPadding;

			ofPushStyle();
			ofSetColor(Colors::BLUE_ACCENT);
			ofFill();
			ofDrawRectangle(indicatorX, indicatorY, indicatorWidth, indicatorHeight);
			ofPopStyle();
		}
	}

	// Frequency indicator for right body
	if (rightBody != NULL) {
		vector<float> freqs = rightBody->getCurrentlyPlaying16Frequencies();
		int index = this->currentSequencerStep;
		if (freqs.size() > index) {
			float frequency = freqs[index];
			int indicatorX = leftX + ofMap(frequency, 0, maxFreq, 0, width);
			int indicatorY = leftY + indicatorPadding;

			ofPushStyle();
			ofSetColor(Colors::RED);
			ofFill();
			ofDrawRectangle(indicatorX, indicatorY, indicatorWidth, indicatorHeight);
			ofPopStyle();
		}
	}
}

void GUIManager::drawRectangularFrame()
{
	ofVec2f winSize = ofGetWindowSize();
	ofPushStyle();
	ofSetColor(Colors::BACKGROUND);
	ofFill();
	ofDrawRectangle(0, 0, Layout::WINDOW_PADDING, winSize.y);
	ofDrawRectangle(0, winSize.y - Layout::WINDOW_PADDING, winSize.x, Layout::WINDOW_PADDING);
	ofDrawRectangle(winSize.x - Layout::WINDOW_PADDING, 0, Layout::WINDOW_PADDING, winSize.y);
	ofDrawRectangle(0, 0, winSize.x, Layout::WINDOW_PADDING);

	ofSetColor(Colors::YELLOW);
	ofNoFill();
	ofDrawRectangle(Layout::WINDOW_PADDING, Layout::WINDOW_PADDING, winSize.x - 2 * Layout::WINDOW_PADDING, winSize.y - 2 * Layout::WINDOW_PADDING);
	ofPopStyle();
}
