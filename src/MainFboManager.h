#pragma once

#include "ofMain.h"
#include "Constants.h"

class MainFboManager {
public:
	static void initialize();
	static void setMainFbo(ofFbo* mainFbo);
	static ofFbo* getMainFbo();
	static void begin();
	static void end();
private:
	static ofFbo* mainFbo;
};