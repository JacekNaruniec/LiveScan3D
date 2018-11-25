using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using NAudio.Wave;

namespace KinectServer
{
    public partial class AudioSettingsForm : Form
    {
        private AudioSettings audioSettings; 

        public AudioSettingsForm(AudioSettings _audioSettings)
        {
            InitializeComponent();
            audioSettings = _audioSettings;

            int waveInDevices = WaveIn.DeviceCount;
            for (int waveInDevice = 0; waveInDevice < waveInDevices; waveInDevice++)
            {
                WaveInCapabilities deviceInfo = WaveIn.GetCapabilities(waveInDevice);
                String deviceInfoString = "Device " + waveInDevice.ToString() + ": "  
                    + deviceInfo.ProductName.ToString() + " " + deviceInfo.Channels.ToString() + " channels";
                cbDevices.Items.Add(deviceInfoString);
            }

            cbDevices.SelectedIndex = audioSettings.deviceIndex;
        }

        private void cbDevices_SelectedIndexChanged(object sender, EventArgs e)
        {
            audioSettings.deviceIndex = cbDevices.SelectedIndex; 
        }

        private void cbAudioEnabled_CheckedChanged(object sender, EventArgs e)
        {
            audioSettings.audioEnabled = cbAudioEnabled.Checked;
        }

        private void tbSamplingRate_TextChanged(object sender, EventArgs e)
        {
            int newSamplingRate = 0;
            bool correctValue;
            correctValue = Int32.TryParse(tbSamplingRate.Text, out newSamplingRate);
            if (correctValue && newSamplingRate > 1000 && newSamplingRate < 200000)
            {
                audioSettings.samplingRate = newSamplingRate;
            }
        }

        private void AudioSettingsForm_Load(object sender, EventArgs e)
        {
            cbDevices.SelectedIndex = audioSettings.deviceIndex;
            tbSamplingRate.Text = audioSettings.samplingRate.ToString();
            cbAudioEnabled.Checked = audioSettings.audioEnabled;
               
        }

        private void btApply_Click(object sender, EventArgs e)
        {
            this.DialogResult = DialogResult.OK;
            this.Close();
        }
    }
}
