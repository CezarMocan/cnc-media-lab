#pragma once

#include <string>
#include "ofxKinectForWindows2.h"
#include "MidiNote.h"

#ifndef CONSTANTS_H
#define CONSTANTS_H

using namespace std;

#define USE_PROGRAMMABLE_PIPELINE 1

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
	const int MAX_INSTRUMENTS = 30;

	const string OSC_HOST = "127.0.0.1";
	const int OSC_PORT = 12345;

	const string SKELETON_DELIMITER = "__SKELETON__";
	const string CONTOUR_DELIMITER = "__CONTOUR__";
	const int NETWORK_TRAFFIC_MAX_LATENCY_MS = 750;	
}

namespace OscCategories {
	const string REMOTE_BODY_DATA = "remote_body_data";

	const string BODY_SEQUENCE = "body_sequence";
	const string BODY_SEQUENCE_RAW = "raw_body_sequence";

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