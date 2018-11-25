using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net.Sockets;
using System.Net;
using System.Threading;


using NAudio.Wave;

namespace KinectServer
{
    class AudioTransferServer
    {
        WaveIn waveIn = new WaveIn();
        AudioSettings audioSettings;
        float[] audioBuffer = new float[1024];
        float[] audioBufferCopy = new float[1024]; 

        public bool bServerRunning = false;
        TcpListener oListener;
        List<TransferSocket> lClientSockets = new List<TransferSocket>();
        object oClientSocketLock = new object();
        object oAudioBufferCopyLock = new object();
        bool bufferReady = false;

        const int nBufferParts = 10;
        int bufferPos = 0;

        public AudioTransferServer(AudioSettings _audioSettings)
        {
            audioSettings = _audioSettings;        
        }

        public void StartServer()
        {
            StartAudioRecording();

            if (!bServerRunning)
            {
                oListener = new TcpListener(IPAddress.Any, 48003);
                oListener.Start();

                bServerRunning = true;
                Thread listeningThread = new Thread(this.ListeningWorker);
                listeningThread.Start();
                Thread receivingThread = new Thread(this.ReceivingWorker);
                receivingThread.Start();
            }
        }

        private void ReceivingWorker()
        {
            System.Timers.Timer checkConnectionTimer = new System.Timers.Timer();
            checkConnectionTimer.Interval = 100;

            checkConnectionTimer.Elapsed += delegate (object sender, System.Timers.ElapsedEventArgs e)
            {
                lock (oClientSocketLock)
                {
                    for (int i = 0; i < lClientSockets.Count; i++)
                    {
                        if (!lClientSockets[i].SocketConnected())
                        {
                            lClientSockets.RemoveAt(i);
                            i--;
                        }
                    }
                }
            };

            checkConnectionTimer.Start();

            while (bServerRunning)
            {
                lock (oClientSocketLock)
                {

                    for (int i = 0; i < lClientSockets.Count; i++)
                    {
                        byte[] buffer = lClientSockets[i].Receive(1);

                        while (buffer.Length != 0)
                        {
                            while (!bufferReady)
                                Thread.Sleep(1);

                            if (buffer[0] == 0)
                            {
                                lock (oAudioBufferCopyLock)
                                {
                                    if (audioBufferCopy.Count() != audioBuffer.Count())
                                        audioBufferCopy = new float[audioBuffer.Count()];
                                    Array.Copy(audioBuffer, audioBufferCopy, audioBuffer.Count());
                                    bufferReady = false;
                                }

                                lClientSockets[i].WriteInt(audioSettings.nChannels);
                                lClientSockets[i].WriteInt(audioSettings.samplingRate);
                                lClientSockets[i].SendFloatArray(audioBufferCopy);
                            }
                            buffer = lClientSockets[i].Receive(1);
                        }
                    }
                }

                Thread.Sleep(1);
            }

            checkConnectionTimer.Stop();
        }


        private void ListeningWorker()
        {
            while (bServerRunning)
            {
                try
                {
                    TcpClient newClient = oListener.AcceptTcpClient();

                    lock (oClientSocketLock)
                    {
                        lClientSockets.Add(new TransferSocket(newClient));
                    }
                }
                catch (SocketException)
                {

                }
                System.Threading.Thread.Sleep(10);
            }
        }


        public void StopServer()
        {
            StopAudioRecording();

            if (bServerRunning)
            {
                bServerRunning = false;

                oListener.Stop();
                lock (oClientSocketLock)
                    lClientSockets.Clear();
            }
        }
    

        void waveIn_DataAvailable(object sender, WaveInEventArgs e)
        {
            if (!audioSettings.audioEnabled)
                return;

            /*
            lock (oAudioBufferCopyLock)
            {
                if (audioBuffer.Count() != (e.BytesRecorded / 2))
                    audioBuffer = new float[e.BytesRecorded / 2];
               
                for (int index = 0; index < e.BytesRecorded; index += 2)
                {
                    short sample = (short)((e.Buffer[index + 1] << 8) |
                                            e.Buffer[index + 0]);
                    float sample32 = sample / 32768f;
                    audioBuffer[index / 2] = sample32;
                }
                bufferReady = true;
            } 
            */
            lock (oAudioBufferCopyLock)
            {
                if (audioBuffer.Count() != (e.BytesRecorded / 2 * nBufferParts))
                    audioBuffer = new float[e.BytesRecorded / 2 * nBufferParts];

                for (int index = 0; index < e.BytesRecorded; index += 2)
                {
                    short sample = (short)((e.Buffer[index + 1] << 8) |
                                            e.Buffer[index + 0]);
                    float sample32 = sample / 32768f;
                    audioBuffer[index / 2 + bufferPos * e.BytesRecorded / 2] = sample32;
                }

                bufferPos++;

                if (bufferPos == nBufferParts)
                {
                    bufferReady = true;
                    bufferPos = 0;
                }
            }

        }

        void StopAudioRecording()
        {
            waveIn.StopRecording(); 
        }


        void StartAudioRecording()
        {
            waveIn.DeviceNumber = audioSettings.deviceIndex;
            waveIn.DataAvailable += waveIn_DataAvailable;
            int sampleRate = audioSettings.samplingRate;
            int channels = audioSettings.nChannels;
            waveIn.WaveFormat = new WaveFormat(sampleRate, channels);
            waveIn.StartRecording();
        }
    }
}
