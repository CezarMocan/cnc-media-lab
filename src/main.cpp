#include "ofMain.h"
#include "ofApp.h"
#include "Constants.h"

//========================================================================
int main( ){
#ifdef USE_PROGRAMMABLE_PIPELINE
	ofGLWindowSettings settings;
	settings.setGLVersion(4, 3);
	settings.setSize(Constants::WINDOW_WIDTH, Constants::WINDOW_HEIGHT);
	ofCreateWindow(settings);
#else
	ofSetupOpenGL(1024, 768, OF_WINDOW);
#endif	

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new ofApp());

}
