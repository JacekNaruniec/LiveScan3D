#pragma once

#include <windows.h>
#include <mmsystem.h>
#include <stdio.h>
#include <string>

#define N_AUDIO_BUFFERS	10
#define MAX_DEVICES		32
#define NEW_DATA_READY	2345

class CSoundInput
{
public:
	CSoundInput();
	int InitSound(int device, int mode, int _buffer_size);

	~CSoundInput();
	std::wstring devices_list[MAX_DEVICES];
	char modes[16][1024];
	char device_map[16];
	int GetNDevices() { return n_audio_devices; }
	int GetNModes() { return n_modes; }
	void ListModes(int device);
	void ProcessHeader(WAVEHDR * pHdr);

	void GetProperties(int idevice, int imode, int &sample_rate, int &bps);

	int GetSelectedSampleRate() { return selected_sample_rate; }
	int GetSelectedBitsPerSample() { return selected_bits_per_sample; }
	int CopyLastAudioBuffer(unsigned char *dest);
	HWND callbackWindow;
    int prefered_mode = 0;


private:
	int InitDevice(int device, int mode);
	void ZeroVariables();
	void ListDevices();

	bool last_buffer_changed;
	bool device_opened;
	int buffer_size;
	int n_audio_devices;
	int n_modes;
	
	int chosen_device;
	int chosen_mode;

	int selected_sample_rate;
	int selected_bits_per_sample;

	bool terminate;

	unsigned char *last_samples_buffer;
	unsigned char *audio_buffer[N_AUDIO_BUFFERS];
	WAVEHDR headers[N_AUDIO_BUFFERS];

	HWAVEIN hWaveIn; /* device handle */

	CRITICAL_SECTION last_buffer_cs;
};