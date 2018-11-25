using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Net.Sockets;
using System.Runtime.Serialization.Formatters.Binary;

namespace KinectServer
{
    public class TransferSocket
    {
        TcpClient oSocket;

        public TransferSocket(TcpClient clientSocket)
        {
            oSocket = clientSocket;
        }

        public byte[] Receive(int nBytes)
        {
            byte[] buffer;
            if (oSocket.Available != 0)
            {
                buffer = new byte[Math.Min(nBytes, oSocket.Available)];
                oSocket.GetStream().Read(buffer, 0, nBytes);
            }
            else
                buffer = new byte[0];

            return buffer;
        }

        public bool SocketConnected()
        {
            return oSocket.Connected;
        }

        public void WriteInt(int val)
        {
            oSocket.GetStream().Write(BitConverter.GetBytes(val), 0, 4);
        }

        public void WriteFloat(float val)
        {
            oSocket.GetStream().Write(BitConverter.GetBytes(val), 0, 4);
        }

        public void SendFloatArray(float[] data)
        {
            byte[] buffer = new byte[sizeof(float) * data.Count()];
            Buffer.BlockCopy(data, 0, buffer, 0, sizeof(float) * data.Count());
            WriteInt(data.Count() * 4);
            oSocket.GetStream().Write(buffer, 0, buffer.Count());
        }

        public void SendFrame(MeshChunks meshChunks)
        {
            //VertexC4ubV3s[] sVertices = Array.ConvertAll(meshChunks.lVertices.ToArray(), x => x.toVertexC4ubV3s());
            VertexC4ubV3f[] sVertices = meshChunks.lVertices;

            int nVerticesToSend = meshChunks.lVertices.Count();
            int nTrianglesToSend = meshChunks.lTriangles.Count() / 3;
            int nChunks = meshChunks.trianglesChunkSizes.Count(); 

            byte[] colorsArray = new byte[sizeof(byte) * 3 * nVerticesToSend];
            float[] verticesArray = new float[sizeof(float) * 3 * nVerticesToSend];

            int[] triangles = meshChunks.lTriangles;
            
            int pos1 = 0;
            int pos2 = 0;
            for (int i = 0; i < nVerticesToSend; i++)
            {
                colorsArray[pos1++] = sVertices[i].R;
                colorsArray[pos1++] = sVertices[i].G;
                colorsArray[pos1++] = sVertices[i].B;
                verticesArray[pos2++] = sVertices[i].X;
                verticesArray[pos2++] = sVertices[i].Y;
                verticesArray[pos2++] = sVertices[i].Z;
            }

            byte[] colorsBuffer = new byte[sizeof(byte) * 3 * nVerticesToSend];
            Buffer.BlockCopy(colorsArray, 0, colorsBuffer, 0, sizeof(byte) * 3 * nVerticesToSend);

            byte[] verticesBuffer = new byte[sizeof(float) * 3 * nVerticesToSend];
            Buffer.BlockCopy(verticesArray, 0, verticesBuffer, 0, sizeof(float) * 3 * nVerticesToSend);

            byte[] trianglesBuffer = new byte[sizeof(int) * 3 * nTrianglesToSend];
            Buffer.BlockCopy(triangles, 0, trianglesBuffer, 0, sizeof(int) * 3 * nTrianglesToSend);

            byte[] chunksVerticesSizesBuffer = new byte[sizeof(int) * nChunks];
            Buffer.BlockCopy(meshChunks.verticesChunkSizes.ToArray(), 0, chunksVerticesSizesBuffer, 0, sizeof(int) * nChunks);

            byte[] chunksTrianglesSizesBuffer = new byte[sizeof(int) * nChunks];
            Buffer.BlockCopy(meshChunks.trianglesChunkSizes.ToArray(), 0, chunksTrianglesSizesBuffer, 0, sizeof(int) * nChunks);

            try
            {                 
                WriteInt(nVerticesToSend);
                WriteInt(nTrianglesToSend);
                WriteInt(nChunks);
                oSocket.GetStream().Write(chunksVerticesSizesBuffer, 0, chunksVerticesSizesBuffer.Length);
                oSocket.GetStream().Write(chunksTrianglesSizesBuffer, 0, chunksTrianglesSizesBuffer.Length);
                oSocket.GetStream().Write(verticesBuffer, 0, verticesBuffer.Length);
                oSocket.GetStream().Write(colorsBuffer, 0, colorsBuffer.Length);
                oSocket.GetStream().Write(trianglesBuffer, 0, trianglesBuffer.Length);
            }
            catch (Exception)
            {
            }
        }
    }
}
