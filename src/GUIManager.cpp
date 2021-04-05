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

void GUIManager::update(TrackedBody* leftBody, TrackedBody* rightBody)
{
	this->leftBody = leftBody;
	this->rightBody = rightBody;
}

void GUIManager::draw()
{
}
