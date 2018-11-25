#include "stdafx.h"
#include "soundinput.h"

static int rates[] = {11025, 11025, 11025, 11025, 22050, 22050, 22050, 22050,
	44100, 44100, 44100, 44100, 96000, 96000, 96000, 96000};	
static int channels[] = {1, 1, 2, 2, 1, 1, 2, 2, 1, 1, 2, 2, 1, 2, 1, 2};
static int bits_per_sample[] = {8, 16, 8, 16, 8, 16, 8, 16, 8, 16, 8, 16, 8, 8, 16, 16};

void CALLBACK waveInProc(HWAVEIN hwi,UINT uMsg,DWORD_PTR dwInstance,DWORD_PTR dwParam1,DWORD_PTR dwParam2)
{
	WAVEHDR *pHdr=NULL;
	switch(uMsg)
	{
		case WIM_CLOSE:
			break;

		case WIM_DATA:
			{
				CSoundInput *sound_input=(CSoundInput*)dwInstance;
				sound_input->ProcessHeader((WAVEHDR *)dwParam1);
			}
			break;

		case WIM_OPEN:
			break;

		default:
			break;
	}
}

void CSoundInput::ProcessHeader(WAVEHDR * pHdr)
{
	MMRESULT mRes=0;
	if (terminate) return;

	if(WHDR_DONE==(WHDR_DONE &pHdr->dwFlags))
	{
		EnterCriticalSection(&last_buffer_cs);
		if (!pHdr->lpData || terminate) 
		{
			LeaveCriticalSection(&last_buffer_cs);
			return;
		}
		memcpy(last_samples_buffer, (unsigned char*)pHdr->lpData, buffer_size);
		last_buffer_changed = true;
		LeaveCriticalSection(&last_buffer_cs);
		waveInAddBuffer(hWaveIn,pHdr,sizeof(WAVEHDR));
		SendMessage(callbackWindow, NEW_DATA_READY, NULL, NULL);
	}
}

int CSoundInput::CopyLastAudioBuffer(unsigned char *dest)
{
	int ret_val;
	if (last_buffer_changed) ret_val=0; else ret_val = -1;	
	if (!last_samples_buffer) return -1;
	
	EnterCriticalSection(&last_buffer_cs);
	memcpy(dest, last_samples_buffer, buffer_size);
	last_buffer_changed = false;
	LeaveCriticalSection(&last_buffer_cs);

	return ret_val;
}

CSoundInput::CSoundInput()
{
	InitializeCriticalSection(&last_buffer_cs);
	ZeroVariables();
	ListDevices();
}

void CSoundInput::ListDevices()
{
	WAVEINCAPS stWIC={0};
	n_audio_devices = waveInGetNumDevs();
	MMRESULT mRes;

	for (int i=0; i<n_audio_devices; i++)
	{
		ZeroMemory(&stWIC,sizeof(WAVEINCAPS));
		mRes=waveInGetDevCaps(i,&stWIC,sizeof(WAVEINCAPS));
        devices_list[i] = stWIC.szPname;
	}
}

void CSoundInput::ListModes(int device)
{
	WAVEINCAPS stWIC={0};
	MMRESULT mRes;

	int mode = 0;
	n_modes = 0;

	ZeroMemory(&stWIC,sizeof(WAVEINCAPS));
	mRes=waveInGetDevCaps(device,&stWIC,sizeof(WAVEINCAPS));
	if(mRes==0)
	{
			if(WAVE_FORMAT_1M08==(stWIC.dwFormats&WAVE_FORMAT_1M08))
			{ sprintf_s(modes[n_modes], 1024, "11.025 kHz, mono, 8-bit"); device_map[n_modes]=mode; n_modes++; }
			mode++;
			
			if(WAVE_FORMAT_1M16==(stWIC.dwFormats&WAVE_FORMAT_1M16))
			{ sprintf_s(modes[n_modes], 1024, "11.025 kHz, mono, 16-bit"); device_map[n_modes]=mode; n_modes++; }
			mode++;
			if(WAVE_FORMAT_1S08==(stWIC.dwFormats&WAVE_FORMAT_1S08))
			{ sprintf_s(modes[n_modes], 1024, "11.025 kHz, stereo, 8-bit"); device_map[n_modes]=mode; n_modes++; }
			mode++;
			if(WAVE_FORMAT_1S16==(stWIC.dwFormats&WAVE_FORMAT_1S16))
			{ sprintf_s(modes[n_modes], 1024, "11.025 kHz, stereo, 16-bit");device_map[n_modes]=mode; n_modes++; }
			mode++;
			if(WAVE_FORMAT_2M08==(stWIC.dwFormats&WAVE_FORMAT_2M08))
			{ sprintf_s(modes[n_modes], 1024, "22.05 kHz, mono, 8-bit"); device_map[n_modes]=mode; n_modes++; }
			mode++;
			if(WAVE_FORMAT_2M16==(stWIC.dwFormats&WAVE_FORMAT_2M16))
			{ sprintf_s(modes[n_modes], 1024, "22.05 kHz, mono, 16-bit");device_map[n_modes]=mode; n_modes++; }
			mode++;
			if(WAVE_FORMAT_2S08==(stWIC.dwFormats&WAVE_FORMAT_2S08))
			{ sprintf_s(modes[n_modes], 1024, "22.05 kHz, stereo, 8-bit"); device_map[n_modes]=mode; n_modes++; }
			mode++;
			if(WAVE_FORMAT_2S16==(stWIC.dwFormats&WAVE_FORMAT_2S16))
			{ sprintf_s(modes[n_modes], 1024, "22.05 kHz, stereo, 16-bit"); device_map[n_modes]=mode; n_modes++; }
			mode++;
			if(WAVE_FORMAT_4M08==(stWIC.dwFormats&WAVE_FORMAT_4M08))
			{ sprintf_s(modes[n_modes], 1024, "44.1 kHz, mono, 8-bit"); device_map[n_modes]=mode; n_modes++; }
			mode++;
			if(WAVE_FORMAT_4M16==(stWIC.dwFormats&WAVE_FORMAT_4M16))
			{ sprintf_s(modes[n_modes], 1024, "44.1 kHz, mono, 16-bit"); device_map[n_modes]=mode; 
              prefered_mode = n_modes;  n_modes++;
            }
			mode++;
			if(WAVE_FORMAT_4S08==(stWIC.dwFormats&WAVE_FORMAT_4S08))
			{ sprintf_s(modes[n_modes], 1024, "44.1 kHz, stereo, 8-bit"); device_map[n_modes]=mode; n_modes++; }
			mode++;
			if(WAVE_FORMAT_4S16==(stWIC.dwFormats&WAVE_FORMAT_4S16))
			{ sprintf_s(modes[n_modes], 1024, "44.1 kHz, stereo, 16-bit"); device_map[n_modes]=mode; n_modes++; }
			mode++;
			if(WAVE_FORMAT_96M08==(stWIC.dwFormats&WAVE_FORMAT_96M08))
			{ sprintf_s(modes[n_modes], 1024, "96 kHz, mono, 8-bit"); device_map[n_modes]=mode; n_modes++; }
			mode++;
			if(WAVE_FORMAT_96S08==(stWIC.dwFormats&WAVE_FORMAT_96S08))
			{ sprintf_s(modes[n_modes], 1024, "96 kHz, stereo, 8-bit"); device_map[n_modes]=mode; n_modes++; }
			mode++;
			if(WAVE_FORMAT_96M16==(stWIC.dwFormats&WAVE_FORMAT_96M16))
			{ sprintf_s(modes[n_modes], 1024, "96 kHz, mono, 16-bit"); device_map[n_modes]=mode; n_modes++; }
			mode++;
			if(WAVE_FORMAT_96S16==(stWIC.dwFormats&WAVE_FORMAT_96S16))
			{ sprintf_s(modes[n_modes], 1024, "96 kHz, stereo, 16-bit"); device_map[n_modes]=mode; n_modes++; }
			mode++;
	}

}

void CSoundInput::ZeroVariables()
{
	device_opened = false;
	n_modes = 0;
	chosen_device = -1;
	chosen_mode = -1;
	hWaveIn = NULL;
	selected_bits_per_sample = -1;
	selected_sample_rate = -1;
	last_buffer_changed = false;
	terminate = false;
	last_samples_buffer = NULL;
	for (int i=0; i<N_AUDIO_BUFFERS; i++)
		audio_buffer[i] = NULL;
}

int CSoundInput::InitDevice(int device, int mode)
{
	WAVEFORMATEX wfx; /* look this up in your documentation */

	WAVEINCAPS stWIC={0};
	MMRESULT mRes;

	mRes=waveInGetDevCaps(device,&stWIC,sizeof(WAVEINCAPS));
	/*
	* first we need to set up the WAVEFORMATEX structure. 
	* the structure describes the format of the audio.
	*/
	wfx.nSamplesPerSec = rates[device_map[mode]]; /* sample rate */
	wfx.wBitsPerSample = bits_per_sample[device_map[mode]]; /* sample size */
	wfx.nChannels = 1;//channels[device_map[mode]]; /* channels*/
	/*
	* WAVEFORMATEX also has other fields which need filling.
	* as long as the three fields above are filled this should
	* work for any PCM (pulse code modulation) format.
	*/
	wfx.cbSize = 0; /* size of _extra_ info */
	wfx.wFormatTag = WAVE_FORMAT_PCM;
	wfx.nBlockAlign = (wfx.wBitsPerSample >> 3) * wfx.nChannels;
	wfx.nAvgBytesPerSec = wfx.nBlockAlign * wfx.nSamplesPerSec;
	/*
	* try to open the default wave device. WAVE_MAPPER is
	* a constant defined in mmsystem.h, it always points to the
	* default wave device on the system (some people have 2 or
	* more sound cards).
	*/
	if(waveInOpen(
		&hWaveIn, 
		device, 
		&wfx, 
		(DWORD_PTR)waveInProc, 
		(DWORD_PTR)this, 
		CALLBACK_FUNCTION
		) != MMSYSERR_NOERROR) {
			device_opened = false;
			return -1;
	}
	device_opened = true;
	return 0;
}

void CSoundInput::GetProperties(int idevice, int imode, int &sample_rate, int &bps)
{
	sample_rate = rates[device_map[imode]];
	bps = bits_per_sample[device_map[imode]];
}

int CSoundInput::InitSound(int device, int mode, int _buffer_size)
{
	if (device_opened) return -1;

	buffer_size = _buffer_size;
	chosen_device = device;
	chosen_mode = mode;
	selected_sample_rate = rates[device_map[mode]];
	selected_bits_per_sample = bits_per_sample[device_map[mode]];

	if (InitDevice(device, mode)!=0) return -1;

	/*
	* initialise the block headers with the size
	* and pointer.
	*/
	for (int i=0; i<N_AUDIO_BUFFERS; i++)
	{
		audio_buffer[i] = new unsigned char[buffer_size];

		ZeroMemory(&headers[i], sizeof(WAVEHDR));
		headers[i].dwBufferLength = buffer_size;
		headers[i].lpData = (LPSTR)audio_buffer[i];

		waveInPrepareHeader(hWaveIn, &headers[i], sizeof(WAVEHDR));
		waveInAddBuffer(hWaveIn,&headers[i],sizeof(WAVEHDR));

	}
	last_samples_buffer = new unsigned char[buffer_size];

	waveInStart(hWaveIn);
	
	return 0;
}

CSoundInput::~CSoundInput()
{
	terminate = true;
		
	if(hWaveIn)
	{
		waveInReset(hWaveIn);
		for(int i=0;i<N_AUDIO_BUFFERS;i++)
		{
			if(headers[i].lpData)
			{
				waveInUnprepareHeader(hWaveIn,&headers[i],sizeof(WAVEHDR));
				ZeroMemory(&headers[i],sizeof(WAVEHDR));
			}
		}
		for (int i=0; i<N_AUDIO_BUFFERS; i++)
		{
			if (audio_buffer[i]!=NULL) 
				delete []audio_buffer[i];
			
			audio_buffer[i] = 0;
		}

		waveInClose(hWaveIn);
		if (last_samples_buffer!=NULL) delete []last_samples_buffer;
	}

	DeleteCriticalSection(&last_buffer_cs);
}
