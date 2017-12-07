using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using KinectServer;

namespace LiveScanPlayer
{
    class FrameFileReaderPly : IFrameFileReader
    {
        string[] filenames;
        int currentFrameIdx = 0;

        public FrameFileReaderPly(string[] filenames)
        {
            this.filenames = filenames;
        }

        public int frameIdx
        {
            get
            {
                return currentFrameIdx;
            }
            set
            {
                JumpToFrame(value);
            }
        }

        public void ReadFrame(List<VertexC4ubV3f> vertices, List<int> triangles)
        {
            BinaryReader reader = new BinaryReader(new FileStream(filenames[currentFrameIdx], FileMode.Open));
            int nPoints = 0, nTriangles = 0;

            string line = ReadLine(reader);
            while (!line.Contains("end_header"))
            {
                if (line.Contains("element vertex"))
                {
                    string[] lineElems = line.Split(' ');
                    nPoints = Int32.Parse(lineElems[2]);
                }
                if (line.Contains("element face"))
                {
                    string[] lineElems = line.Split(' ');
                    nTriangles = Int32.Parse(lineElems[2]);
                }
                line = ReadLine(reader);
            }

            for (int i = 0; i < nPoints; i++)
            {
                VertexC4ubV3f v = new VertexC4ubV3f();
                v.X = reader.ReadSingle();
                v.Y = reader.ReadSingle();
                v.Z = reader.ReadSingle();
                v.R = reader.ReadByte();
                v.G = reader.ReadByte();
                v.B = reader.ReadByte();
                vertices.Add(v);
            }

            for (int i = 0; i < nTriangles; i++)
            {
                reader.ReadByte();
                for (int j=0; j<3; j++)
                    triangles.Add(reader.ReadInt32());
            }
        
            reader.Dispose();

            currentFrameIdx++;
            if (currentFrameIdx >= filenames.Length)
                currentFrameIdx = 0;
        }

        public void JumpToFrame(int frameIdx)
        {
            currentFrameIdx = frameIdx;
            if (currentFrameIdx >= filenames.Length)
                currentFrameIdx = 0;
        }

        public void Rewind()
        {
            currentFrameIdx = 0;
        }

        public string ReadLine(BinaryReader binaryReader)
        {
            StringBuilder builder = new StringBuilder();
            byte buffer = binaryReader.ReadByte();

            while (buffer != '\n')
            {
                builder.Append((char)buffer);
                buffer = binaryReader.ReadByte();
            }

            return builder.ToString();
        }

    }
}
