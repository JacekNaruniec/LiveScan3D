using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using KinectServer;

namespace LiveScanPlayer
{
    interface IFrameFileReader
    {
        int frameIdx
        {
            get;
            set;
        }

        void ReadFrame(List<VertexC4ubV3f> vertices, List<int> triangles);

        void JumpToFrame(int frameIdx);

        void Rewind();
    }
}
