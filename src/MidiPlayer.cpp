#include "MidiPlayer.h"

map<string, ofSoundPlayer> sounds;

void MidiPlayer::loadSoundFonts()
{
	vector<string> instruments = { 
		//Instruments::DEFAULT_INSTRUMENT, 
		/*
		Instruments::GRAND_PIANO, 
		Instruments::CELESTA, 
		Instruments::ELECTRIC_GUITAR_CLEAN, 
		Instruments::FX_2_SOUNDTRACK, 
		Instruments::CHOIR,
		Instruments::VIOLIN,
		Instruments::SEASHORE,
		*/
		//Instruments::SHAKUHACHI
	};
	vector<string> notes = { "A", "Ab", "B", "Bb", "C", "D", "Db", "E", "Eb", "F", "G", "Gb" };
	vector<int> octaves = { 1, 2, 3, 4, 5, 6, 7 };

	for (auto instrument = instruments.begin(); instrument != instruments.end(); ++instrument) {
		for (auto note = notes.begin(); note != notes.end(); ++note) {
			for (auto octave = octaves.begin(); octave != octaves.end(); ++octave) {
				stringstream fullNote;
				fullNote << (*note) << (*octave);

				stringstream filePath;
				filePath << "soundfonts\\" << (*instrument) << "-mp3\\" << fullNote.str() << ".mp3";
				ofSoundPlayer s;
				s.load(filePath.str());

				string key = MidiNote::toMidiNote(*instrument, *note, *octave)->getString();
				sounds[key] = s;

				ofLogNotice() << "Loaded: " << filePath.str();
			}
		}
	}
}

void MidiPlayer::playNote(MidiNote* mNote, float duration)
{
	MidiPlayer::playNoteWithInstrument(mNote->note, mNote->octave, mNote->instrument, duration);
}

void MidiPlayer::playNote(MidiNote* mNote, string instrument, float duration)
{
	MidiPlayer::playNoteWithInstrument(mNote->note, mNote->octave, instrument, duration);
}

void MidiPlayer::playNote(string note, int octave, float duration)
{
	MidiPlayer::playNoteWithInstrument(note, octave, Instruments::DEFAULT_INSTRUMENT, duration);
}

void MidiPlayer::playNoteWithInstrument(string note, int octave, string instrument, float duration)
{
	string key = MidiNote::toMidiNote(instrument, note, octave)->getString();
	if (instrument.compare(Instruments::VIOLIN) == 0)
		sounds[key].setVolume(0.7);
	else if (instrument.compare(Instruments::SEASHORE) == 0)
		sounds[key].setVolume(0.5);
	else
		sounds[key].setVolume(1.);
	sounds[key].play();


	string key2 = MidiNote::toMidiNote(Instruments::CHOIR, note, octave - 1)->getString();
	sounds[key2].setVolume(0.04);
	sounds[key2].play();
}
