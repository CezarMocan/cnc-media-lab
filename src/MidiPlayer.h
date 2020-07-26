#pragma once
#include <string>
#include "ofMain.h"
#include "MidiNote.h"
#include "Constants.h"

using namespace std;

class MidiPlayer {
public:	
	static void loadSoundFonts();
	static void playNote(MidiNote* mNote, float duration = -1.);
	static void playNote(MidiNote* mNote, string instrument, float duration);
	static void playNote(string note, int octave, float duration = -1.);
	static void playNoteWithInstrument(string note, int octave, string instrument, float duration = -1.);	
};