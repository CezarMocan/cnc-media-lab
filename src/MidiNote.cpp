#include "MidiNote.h"
#include "Constants.h"

MidiNote::MidiNote(string instrument, string note, int octave)
{
	this->instrument = (instrument == "") ? Instruments::DEFAULT_INSTRUMENT : instrument;
	this->note = note;
	this->octave = octave;
}

MidiNote::MidiNote(string note, int octave)
{
	this->instrument = "";
	this->note = note;
	this->octave = octave;
}

MidiNote* MidiNote::toMidiNote(string instrument, string note, int octave)
{
	return new MidiNote(instrument, note, octave);
}

string MidiNote::getString()
{
	stringstream fullNote;
	fullNote << this->note << this->octave;
	string str = instrument + fullNote.str();
	return str;
}

int MidiNote::getCode()
{
	int base = this->octave * 12 + 12;
	auto it = Midi::noteToId.find(this->note);
	if (it == Midi::noteToId.end())
		return 0;
	else
		return base + it->second;
}

float MidiNote::getFrequency()
{
	int midiCode = this->getCode();
	float a = 440; // a is 440 hz...
	float freq = (a / 32.0) * (1 << ((midiCode - 9) / 12));
	return freq;
}


