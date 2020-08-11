#include "MainFboManager.h"

ofFbo* MainFboManager::mainFbo;

void MainFboManager::initialize()
{
	mainFbo = NULL;
}

void MainFboManager::setMainFbo(ofFbo* fbo)
{
	mainFbo = fbo;
}

ofFbo* MainFboManager::getMainFbo()
{
	return mainFbo;
}

void MainFboManager::begin()
{
	if (mainFbo != NULL) {
		mainFbo->begin();
		ofPushMatrix();
		ofTranslate(Layout::WINDOW_PADDING, Layout::WINDOW_PADDING);
		ofScale(Layout::WINDOW_SCALE);
	}
}

void MainFboManager::end()
{
	if (mainFbo != NULL) {
		ofPopMatrix();
		mainFbo->end();
	}
}
