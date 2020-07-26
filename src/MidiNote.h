#pragma once

#include <string>
#include <sstream>

using namespace std;

class MidiNote {
public:
	MidiNote(string instrument, string note, int octave);
	MidiNote(string note, int octave);
	string getString();
	int getCode();

	static MidiNote* toMidiNote(string instrument, string note, int octave);

	string instrument;
	string note;
	int octave;
};