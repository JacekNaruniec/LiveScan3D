using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;

using System.Net.Sockets;
using System.Net;
using System.Diagnostics;

namespace KinectServer
{
    public class MeshChunks
    {
        public List<VertexC4ubV3f> lVertices;
        public List<int> lTriangles;
        public List<int> verticesChunkSizes;
        public List<int> trianglesChunkSizes; 
    }
    public class TransferServer
    {
        List<VertexC4ubV3f> lVerticesCloned = new List<VertexC4ubV3f>();
        List<int> lTrianglesCloned = new List<int>();

        object meshCopyLock = new object();

        TcpListener oListener;
        List<TransferSocket> lClientSockets = new List<TransferSocket>();

        object oClientSocketLock = new object();
        bool bServerRunning = false;

        ~TransferServer()
        {
            StopServer();
        }

        public void StartServer()
        {
            if (!bServerRunning)
            {
                oListener = new TcpListener(IPAddress.Any, 48002);
                oListener.Start();

                bServerRunning = true;
                Thread listeningThread = new Thread(this.ListeningWorker);
                listeningThread.Start();
                Thread receivingThread = new Thread(this.ReceivingWorker);
                receivingThread.Start();
            }
        }

        public void StopServer()
        {
            if (bServerRunning)
            {
                bServerRunning = false;

                oListener.Stop();
                lock (oClientSocketLock)
                    lClientSockets.Clear();
            }
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
                System.Threading.Thread.Sleep(100);
            }
        }

        //public void setN

        public void updateMesh(VertexC4ubV3f[] lVertices, int[] lTriangles)
        {
            lock (meshCopyLock)
            {
                lVerticesCloned = new List<VertexC4ubV3f>(lVertices);
                lTrianglesCloned = new List<int>(lTriangles);
            }
        }

        private void ReceivingWorker()
        {
            System.Timers.Timer checkConnectionTimer = new System.Timers.Timer();
            checkConnectionTimer.Interval = 1000;

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
                            if (buffer[0] == 0)
                            {
                               List<VertexC4ubV3f> lVerticesClonedLocal;
                               List<int> lTrianglesClonedLocal;

                                lock (meshCopyLock)
                                {
                                    lVerticesClonedLocal = new List<VertexC4ubV3f>(lVerticesCloned);
                                    lTrianglesClonedLocal = new List<int>(lTrianglesCloned);
                                }

                                if (lTrianglesClonedLocal.Count > 0)
                                {
                                    MeshChunks meshChunks = formMeshChunks(lVerticesClonedLocal, lTrianglesClonedLocal);
                                    Stopwatch sw = new Stopwatch();
                                    sw.Start(); 
                                    lClientSockets[i].SendFrame(meshChunks);
                                    sw.Stop(); 
                                    Console.WriteLine("Sending elapsed={0}", sw.ElapsedMilliseconds);

                                }
                                else if (lVerticesClonedLocal.Count > 0)
                                {
                                    MeshChunks meshChunks = formVerticesChunks(lVerticesClonedLocal);
                                    lClientSockets[i].SendFrame(meshChunks);

                                }
                            }

                            buffer = lClientSockets[i].Receive(1);
                        }
                    }
                }

                Thread.Sleep(10);
            }

            checkConnectionTimer.Stop();
        }

        private VertexC4ubV3f[] SubArray(VertexC4ubV3f[] data, int index, int length)
        {
            VertexC4ubV3f[] result = new VertexC4ubV3f[length];
            Array.Copy(data, index, result, 0, length);
            return result;
        }
        private MeshChunks formVerticesChunks(List<VertexC4ubV3f> lVertices)
        {
            int chunkSizeLimit = 65000 - 3;

            MeshChunks ret = new MeshChunks();
            int nVertices = lVertices.Count();
            List<int> trianglesInChunks = new List<int>();
            List<int> verticesInChunks = new List<int>();
            int currentVertex = 0;

            while (currentVertex < nVertices)
            {
                int size = Math.Min(chunkSizeLimit, nVertices - currentVertex);
                verticesInChunks.Add(size);
                trianglesInChunks.Add(0);
                currentVertex += size; 
            }

            ret.lTriangles = new List<int>();
            ret.lVertices = new List<VertexC4ubV3f>(lVertices);
            ret.trianglesChunkSizes = trianglesInChunks;
            ret.verticesChunkSizes = verticesInChunks;
            return ret;
        }


    private MeshChunks formMeshChunks(List<VertexC4ubV3f> lVertices, List<int> lTriangles)
        {
            int chunkSizeLimit = 65000 - 3;

            MeshChunks ret = new MeshChunks();
            int nVertices = lVertices.Count();
            int nTriangles = lTriangles.Count() / 3;
            List<int> trianglesInChunks = new List<int>();
            List<int> verticesInChunks = new List<int>(); 
            int[] chunkIndex = new int[nVertices];
            int[] verticesMap = new int[nVertices];
            VertexC4ubV3f[] newVertices = new VertexC4ubV3f[nTriangles * 3];
            int[] newTriangles = new int[nTriangles * 3];
            int trianglesChunkStart = 0;
            int currentChunkIndex = 0;
            int currentVertex = 0;

            for (int v = 0; v < nVertices; v++)
                chunkIndex[v] = -1;

            int verticesInCurrentChunk = 0;

            Stopwatch sw = new Stopwatch();

            sw.Start(); 

            for (int t = 0; t < nTriangles * 3; t++)
            {
                int val = lTriangles[t];
                if (chunkIndex[val] != currentChunkIndex)
                {
                    newVertices[currentVertex] = lVertices[val];
                    verticesMap[val] = verticesInCurrentChunk;
                    chunkIndex[val] = currentChunkIndex;
                    currentVertex++;
                    newTriangles[t] = verticesInCurrentChunk;
                    verticesInCurrentChunk++;
                }
                else
                {
                    newTriangles[t] = verticesMap[val];
                }
                 
                if (verticesInCurrentChunk >= chunkSizeLimit && (((t+1) % 3) == 0))
                {
                    currentChunkIndex++;
                    verticesInChunks.Add(verticesInCurrentChunk);
                    trianglesInChunks.Add((t - trianglesChunkStart) / 3);
                    verticesInCurrentChunk = 0;
                    trianglesChunkStart = t;
                }
            }

            if (verticesInCurrentChunk!=0)
            {
                verticesInChunks.Add(verticesInCurrentChunk);
                trianglesInChunks.Add((nTriangles * 3 - trianglesChunkStart) / 3);
            }

            ret.lTriangles = new List<int>(newTriangles);
            ret.lVertices = new List<VertexC4ubV3f>(SubArray(newVertices, 0, currentVertex));
            ret.trianglesChunkSizes = trianglesInChunks;
            ret.verticesChunkSizes = verticesInChunks;

            sw.Stop();

            Console.WriteLine("Forming elapsed={0}", sw.ElapsedMilliseconds);

            return ret; 
        }
    }
}
