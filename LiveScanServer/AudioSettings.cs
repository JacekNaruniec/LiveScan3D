using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KinectServer
{
    public class AudioSettings
    {
        public bool audioEnabled = true; 
        public int deviceIndex = 0;
        public int samplingRate = 44100;
        public int nChannels = 1; 
    }
}
