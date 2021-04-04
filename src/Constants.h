#pragma once

#include <string>
#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "MidiNote.h"

#ifndef CONSTANTS_H
#define CONSTANTS_H

using namespace std;

#define USE_PROGRAMMABLE_PIPELINE 1
//ofColor(241, 113, 97), ofColor(252, 36, 21)

namespace Colors {
	const ofColor RED = ofColor(241, 113, 97);
	const ofColor RED_TRANSPARENT = ofColor(241, 113, 97, 25);
	const ofColor RED_ACCENT = ofColor(252, 36, 21);
	const ofColor RED_ACCENT_TRANSPARENT = ofColor(252, 36, 21, 75);
	const ofColor RED_SHADOW = ofColor(252, 96, 21, 60);
	const ofColor BLUE = ofColor(31, 126, 240);
	const ofColor BLUE_TRANSPARENT = ofColor(31, 126, 240, 25);
	const ofColor BLUE_ACCENT = ofColor(0, 105, 205);
	const ofColor BLUE_ACCENT_TRANSPARENT = ofColor(0, 105, 205, 75);
	const ofColor BLUE_SHADOW = ofColor(0, 195, 205, 60);
	const ofColor YELLOW = ofColor(249, 206, 42);
	const ofColor BACKGROUND = ofColor(25, 32, 28);
};

namespace Midi {
	const map<string, int> noteToId = {
		{ "C", 0 },
		{ "Db", 1 },
		{ "D", 2 },
		{ "Eb", 3 },
		{ "E", 4 },
		{ "F", 5 },
		{ "Gb", 6 },
		{ "G", 7 },
		{ "Ab", 8 },
		{ "A", 9 },
		{ "Bb", 10 },
		{ "B", 11 }
	};
}

namespace MappingPatterns {
	const vector<vector<vector<int> > > to16 = {
		// 0 items to a beat of 16
		{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		// 1 item to a beat of 16
		{ { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 } },
		// 2 items to a beat of 16
		{ { 1, 2, 0, 1, 2, 0, 1, 2, 2, 1, 0, 1, 2, 0, 1, 0 } },
		// 3 items to a beat of 16
		{ { 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 1, 2, 3, 0, 0 } },
		// 4 items to a beat of 16
		{ { 1, 2, 0, 3, 4, 3, 4, 0, 1, 2, 0, 3, 4, 3, 4, 0 } },
		// 5 items to a beat of 16
		{ { 1, 2, 3, 4, 5, 0, 1, 3, 5, 2, 4, 0, 1, 2, 3, 0 } },
		// 6 items to a beat of 16
		{ { 1, 2, 3, 4, 5, 6, 0, 0, 1, 2, 3, 4, 5, 6, 0, 0 } },
		// 7 items to a beat of 16
		{ { 1, 0, 2, 0, 0, 3, 4, 0, 5, 6, 0, 0, 0, 7, 0, 0 }, {1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 0, 0, 4, 5, 6}, {1, 1, 2, 2, 0, 3, 3, 4, 5, 5, 0, 6, 6, 7, 7, 0} },
		// 8 items to a beat of 16
		{ { 1, 0, 2, 0, 3, 0, 4, 5, 0, 6, 0, 7, 0, 8, 0, 0 } },
		// 9 items to a beat of 16
		{ { 1, 0, 2, 0, 3, 0, 4, 5, 6, 0, 7, 0, 8, 0, 9, 0 } },
		// 10 items to a beat of 16
		{ { 1, 0, 2, 3, 0, 0, 4, 5, 0, 6, 7, 0, 8, 9, 0, 10 } },
		// 11 items to a beat of 16
		{ { 1, 0, 2, 3, 4, 0, 0, 5, 6, 7, 0, 0, 8, 9, 10, 11 } },
		// 12 items to a beat of 16
		{ { 1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 9, 0, 0, 10, 11, 12 } },
		// 13 items to a beat of 16
		{ { 1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 9, 0, 10, 11, 12, 13 } },
		// 14 items to a beat of 16
		{ { 1, 2, 3, 4, 0, 5, 6, 7, 8, 0, 9, 10, 11, 12, 13, 14 } },
		// 15 items to a beat of 16
		{ { 1, 2, 3, 4, 5, 6, 0, 7, 8, 9, 10, 11, 12, 13, 14, 15 } },
		// 16 items to a beat of 16
		{ { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 } }
	};
}

namespace Scales {
	const vector<MidiNote*> PENTATONIC = { 
		new MidiNote("C", 2), new MidiNote("D", 2), new MidiNote("E", 2), new MidiNote("G", 2), new MidiNote("A", 2),
		new MidiNote("C", 3), new MidiNote("D", 3),	new MidiNote("E", 3), new MidiNote("G", 3),	new MidiNote("A", 3),
		new MidiNote("C", 4), new MidiNote("D", 4), new MidiNote("E", 4), new MidiNote("G", 4), new MidiNote("A", 4),
		new MidiNote("C", 5), new MidiNote("D", 5), new MidiNote("E", 5), new MidiNote("G", 5), new MidiNote("A", 5),
		new MidiNote("C", 6), new MidiNote("D", 6), new MidiNote("E", 6), new MidiNote("G", 6), new MidiNote("A", 6),
		new MidiNote("C", 7), new MidiNote("D", 7), new MidiNote("E", 7), new MidiNote("G", 7), new MidiNote("A", 7),
	};
}

namespace Instruments {
	const string DEFAULT_INSTRUMENT = "acoustic_grand_piano";
	//const string DEFAULT_INSTRUMENT = "shakuhachi";
	const string CELESTA = "celesta";
	const string GRAND_PIANO = "acoustic_grand_piano";
	const string ELECTRIC_GUITAR_CLEAN = "electric_guitar_clean";
	const string CHOIR = "choir_aahs";
	const string FX_2_SOUNDTRACK = "fx_2_soundtrack";
	const string SHAKUHACHI = "shakuhachi";
	const string VIOLIN = "violin";
	const string SEASHORE = "electric_guitar_clean";
	const vector<string> INSTRUMENT_LIST = { "Pan Flute", "Steel Drums", "Sitar", "Echoes", "Timpani", "Woodblock", "", "", "", "" };
}

namespace Constants {
	const int WINDOW_WIDTH = 1024;
	const int WINDOW_HEIGHT = 768;

	const int DEPTH_WIDTH = 512;
	const int DEPTH_HEIGHT = 424;
	const int DEPTH_SIZE = DEPTH_WIDTH * DEPTH_HEIGHT;

	const int COLOR_WIDTH = 1920;
	const int COLOR_HEIGHT = 1080;

	const int MAX_TRACKED_BODIES = 9;
	const int BODY_RECORDINGS_ID_OFFSET = 10;
	const int MAX_BODY_RECORDINGS = 20;
	const int MAX_INSTRUMENTS = 30;

	const string OSC_HOST = "127.0.0.1";
	const int OSC_PORT = 12345;
	const int OSC_RECEIVE_PORT = 12344;

	const string SKELETON_DELIMITER = "__SKELETON__";
	const string CONTOUR_DELIMITER = "__CONTOUR__";
	const string IS_RECORDING_DELIMITER = "__IS_RECORDING__";
	const string INSTRUMENT_ID_DELIMITER = "__INSTRUMENT_ID__";
	const int NETWORK_TRAFFIC_MAX_LATENCY_MS = 750;	

	const string CEZAR_IP = "10.147.20.54";
	const string CY_IP = "10.147.20.159";

	const float SHADOW_EXPECTED_FREQUENCY_SEC = 50;
	const float SHADOW_REC_MAX_DURATION_SEC = 15;
	const float SHADOW_REC_MIN_DURATION_SEC = 7;
	const float SHADOW_PLAY_MIN_DURATION_SEC = 15;
	const float SHADOW_PLAY_MAX_DURATION_SEC = 40;
}

namespace Layout {
	const int WINDOW_PADDING = 40;
	const float WINDOW_SCALE = 2.0;
	const int FRAME_PADDING = 4;
	const int SEQUENCER_ROW_SIZE = 8;
	const int SEQUENCER_ELEMENT_SIZE = 22;
	const int FONT_SIZE = 10;
}

namespace OscCategories {
	const string SEQUENCER_STEP = "sequencer_step";
	const string REMOTE_BODY_DATA = "remote_body_data";
	const string NEW_BODY = "new_body";

	const string BODY_SEQUENCE = "body_sequence";
	const string BODY_SEQUENCE_RAW = "raw_body_sequence";
	const string BODY_IS_RECORDING = "body_is_recording";
	const string BODY_INTERSECTION = "body_intersection";

	const string BODY = "body";	
	const string ENVIRONMENT = "env";

	const string DISTANCE = "distance";
	const string ANGLE = "angle";
	const string MOVEMENT = "movement";
	const string GESTURE = "gesture";
	const string ABSTRACT = "abstract";
}

namespace OscMessages {

}

#endif