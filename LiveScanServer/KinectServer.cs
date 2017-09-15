//   Copyright (C) 2015  Marek Kowalski (M.Kowalski@ire.pw.edu.pl), Jacek Naruniec (J.Naruniec@ire.pw.edu.pl)
//   License: MIT Software License   See LICENSE.txt for the full license.

//   If you use this software in your research, then please use the following citation:

//    Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
//    Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015

//    @INPROCEEDINGS{Kowalski15,
//        author={Kowalski, M. and Naruniec, J. and Daniluk, M.},
//        booktitle={3D Vision (3DV), 2015 International Conference on},
//        title={LiveScan3D: A Fast and Inexpensive 3D Data Acquisition System for Multiple Kinect v2 Sensors},
//        year={2015},
//    }
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using System.Runtime.InteropServices;

using System.Net.Sockets;
using System.Net;
using System.Diagnostics;
using System.ComponentModel;

namespace KinectServer
{
    public delegate void SocketListChangedHandler(List<KinectSocket> list);
    public class KinectServer
    {


        [DllImport("NativeUtils.dll")]
        static extern void generateMeshFromDepthMaps(int n_maps, byte[] depth_maps, byte[] depth_colors,
            int[] widths, int[] heights, float[] iparams, float[] tparams, ref Mesh out_mesh, bool bColorTransfer,
            float minX, float minY, float minZ, float maxX, float maxY,  float maxZ);

        [DllImport("NativeUtils.dll")]
        static extern void generateTrianglesWithColorsFromDepthMap(int n_maps, byte[] depth_maps, byte[] depth_colors,
            int[] widths, int[] heights, float[] iparams, float[] tparams, ref Mesh out_mesh, int depth_map_index);

        [DllImport("NativeUtils.dll")]
        static extern void depthMapAndColorSetRadialCorrection(int n_maps, byte[] depth_maps, byte[] depth_colors,
            int[] widths, int[] heights, float[] iparams);


        [DllImport("NativeUtils.dll")]
        static extern Mesh createMesh();

        [DllImport("NativeUtils.dll")]
        static extern void deleteMesh(ref Mesh mesh);

        Socket oServerSocket;

        bool bServerRunning = false;

        KinectSettings oSettings;
        object oClientSocketLock = new object();
        object oFrameRequestLock = new object();

        List<KinectSocket> lClientSockets = new List<KinectSocket>();

        public event SocketListChangedHandler eSocketListChanged;

        byte[] depthMaps;
        byte[] depthColors;
        int[] widths;
        int[] heights;
        float[] intrinsicsParams;
        float[] transformParams;
        int nFramesCopied = 0;

        List<List<Body>> lStoredBodies = new List<List<Body>>();

        public int nClientCount
        {
            get
            {
                int nClients;
                lock (oClientSocketLock)
                {
                    nClients = lClientSockets.Count;
                }
                return nClients;
            }
        }

        public List<AffineTransform> lCameraPoses
        {
            get
            {
                List<AffineTransform> cameraPoses = new List<AffineTransform>();
                lock (oClientSocketLock)
                {
                    for (int i = 0; i < lClientSockets.Count; i++)
                    {
                        cameraPoses.Add(lClientSockets[i].oCameraPose);
                    }
                }
                return cameraPoses;
            }
            set
            {
                lock (oClientSocketLock)
                {
                    for (int i = 0; i < lClientSockets.Count; i++)
                    {
                        lClientSockets[i].oCameraPose = value[i];
                    }
                }
            }
        }

        public List<AffineTransform> lWorldTransforms
        {
            get
            {
                List<AffineTransform> worldTransforms = new List<AffineTransform>();
                lock (oClientSocketLock)
                {
                    for (int i = 0; i < lClientSockets.Count; i++)
                    {
                        worldTransforms.Add(lClientSockets[i].oWorldTransform);
                    }
                }
                return worldTransforms;
            }

            set
            {
                lock (oClientSocketLock)
                {
                    for (int i = 0; i < lClientSockets.Count; i++)
                    {
                        lClientSockets[i].oWorldTransform = value[i];
                    }
                }
            }
        }

        public bool bAllCalibrated
        {
            get
            {
                bool allCalibrated = true;
                lock (oClientSocketLock)
                {
                    for (int i = 0; i < lClientSockets.Count; i++)
                    {
                        if (!lClientSockets[i].bCalibrated)
                        {
                            allCalibrated = false;
                            break;
                        }
                    }

                }
                return allCalibrated;
            }
        }

        public KinectServer(KinectSettings settings)
        {
            this.oSettings = settings;
        }

        private static T[] FromByteArray<T>(byte[] source) where T : struct
        {
            T[] destination = new T[source.Length / Marshal.SizeOf(typeof(T))];
            GCHandle handle = GCHandle.Alloc(destination, GCHandleType.Pinned);
            try
            {
                IntPtr pointer = handle.AddrOfPinnedObject();
                Marshal.Copy(source, 0, pointer, source.Length);
                return destination;
            }
            finally
            {
                if (handle.IsAllocated)
                    handle.Free();
            }
        }

        private void SocketListChanged()
        {
            if (eSocketListChanged != null)
            {
                eSocketListChanged(lClientSockets);
            }
        }

        public void StartServer()
        {
            if (!bServerRunning)
            {
                oServerSocket = new Socket(SocketType.Stream, ProtocolType.Tcp);
                oServerSocket.Blocking = false;

                IPEndPoint endPoint = new IPEndPoint(IPAddress.Any, 48001);
                oServerSocket.Bind(endPoint);
                oServerSocket.Listen(10);

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

                oServerSocket.Close();
                lock (oClientSocketLock)
                    lClientSockets.Clear();
            }
        }

        public void CaptureSynchronizedFrame()
        {
            lock (oClientSocketLock)
            {
                for (int i = 0; i < lClientSockets.Count; i++)
                    lClientSockets[i].CaptureFrame();
            }

            //Wait till frames captured
            bool allGathered = false;
            while (!allGathered)
            {
                allGathered = true;

                lock (oClientSocketLock)
                {
                    for (int i = 0; i < lClientSockets.Count; i++)
                    {
                        if (!lClientSockets[i].bFrameCaptured)
                        {
                            allGathered = false;
                            break;
                        }
                    }
                }
            }
        }

        public void Calibrate()
        {
            lock (oClientSocketLock)
            {
                for (int i = 0; i < lClientSockets.Count; i++)
                {
                    lClientSockets[i].Calibrate();
                }
            }
        }

        public void SendSettings()
        {
            lock (oClientSocketLock)
            {
                for (int i = 0; i < lClientSockets.Count; i++)
                {
                    lClientSockets[i].SendSettings(oSettings);
                }
            }
        }

        public void SendCalibrationData()
        {
            lock (oClientSocketLock)
            {
                for (int i = 0; i < lClientSockets.Count; i++)
                {
                    lClientSockets[i].SendCalibrationData();
                }
            }
        }

        public bool GetStoredFrame(List<VertexC4ubV3f> lVerticesWithColors, List<int> lFrameTriangles)
        {
            bool bNoMoreStoredFrames;
            lVerticesWithColors.Clear();
            lFrameTriangles.Clear();

            lock (oFrameRequestLock)
            {
                //Request frames
                lock (oClientSocketLock)
                {
                    for (int i = 0; i < lClientSockets.Count; i++)
                        lClientSockets[i].RequestStoredFrame();
                }

                //Wait till frames received
                bool allGathered = false;
                bNoMoreStoredFrames = false;
                while (!allGathered && !bNoMoreStoredFrames)
                {
                    allGathered = true;
                    lock (oClientSocketLock)
                    {
                        for (int i = 0; i < lClientSockets.Count; i++)
                        {
                            if (!lClientSockets[i].bStoredFrameReceived)                 
                                allGathered = false;
                            

                            if (lClientSockets[i].bNoMoreStoredFrames)
                                bNoMoreStoredFrames = true;
                        }
                    }
                }

                if (!bNoMoreStoredFrames)
                {
                    CopyLatestFrames();
                    CorrectRadialDistortionsForDepthMaps();
                    GenerateMesh(lVerticesWithColors, lFrameTriangles);
                }
            }

            if (bNoMoreStoredFrames)
                return false;
            else
                return true;
        }

        private List<int> CopyMeshToTrianglesList(Mesh mesh)
        {
            if (mesh.nTriangles == 0)
                return new List<int>();

            int[] triangles = new int[mesh.nTriangles * 3];

            Marshal.Copy(mesh.triangles, triangles, 0, mesh.nTriangles * 3);

            return triangles.ToList<int>();
        }

        public void GenerateMesh(List<VertexC4ubV3f> lVerticesWithColours, List<int> lFrameTriangles)
        {
            Mesh mesh = new Mesh();
            mesh.nVertices = 0;

            int nClients = nFramesCopied;

            lVerticesWithColours.Clear();
            lFrameTriangles.Clear();

            if (nClients == 0)
                return; 
            
            generateMeshFromDepthMaps(nClients, depthMaps, depthColors, widths, heights, intrinsicsParams, transformParams, ref mesh, oSettings.bColorTransfer,
                oSettings.aMinBounds[0], oSettings.aMinBounds[1], oSettings.aMinBounds[2],
                oSettings.aMaxBounds[0], oSettings.aMaxBounds[1], oSettings.aMaxBounds[2]);

            lVerticesWithColours.AddRange(CopyMeshToVerticesWithColours(mesh));
            lFrameTriangles.AddRange(CopyMeshToTrianglesList(mesh));
            deleteMesh(ref mesh);

        }

        private List<VertexC4ubV3f> CopyMeshToVerticesWithColours(Mesh mesh)
        {
            int nElements = mesh.nVertices;
            if (nElements == 0)
                return new List<VertexC4ubV3f>();

            byte[] byteVertices = new byte[nElements * 4 * sizeof(int)];
            Marshal.Copy(mesh.verticesWithColors, byteVertices, 0, nElements * 4 * sizeof(int));
            VertexC4ubV3f[] vertices = new VertexC4ubV3f[1];

            vertices = FromByteArray<VertexC4ubV3f>(byteVertices);

            return vertices.ToList<VertexC4ubV3f>();
        }

        public void RequestLastFrames()
        {
            lock (oFrameRequestLock)
            {
                lock (oClientSocketLock)
                {
                    for (int i = 0; i < lClientSockets.Count; i++)
                        lClientSockets[i].RequestLastFrame();
                }
            }
        }


        public void CopyLatestFrames()
        {
            lock (oFrameRequestLock)
            {
                //Wait till frames received
                bool allGathered = false;

                while (!allGathered)
                {
                    allGathered = true;

                    lock (oClientSocketLock)
                    {
                        for (int i = 0; i < lClientSockets.Count; i++)
                        {
                            if (!lClientSockets[i].bLatestFrameReceived)
                                if (lClientSockets[i].bWaitingForFrame)
                                {
                                    allGathered = false;
                                    break;
                                }
                        }
                    }

                }
            }

            lock (oClientSocketLock)
            {
                int nClients = lClientSockets.Count;
                int nClientsWithFrames = 0;
                int[] activeClients = new int[nClients];

                for (int i = 0; i < nClients; i++)
                {
                    if ((lClientSockets[i].iDepthFrameWidth != 0) && lClientSockets[i].SocketConnected())
                    {
                        activeClients[nClientsWithFrames] = i;
                        nClientsWithFrames++;
                    }
                }

                lStoredBodies.Clear();
                nFramesCopied = nClientsWithFrames;

                if (nClientsWithFrames == 0)
                    return;
                

                widths = new int[nClientsWithFrames];
                heights = new int[nClientsWithFrames];
                intrinsicsParams = new float[7 * nClientsWithFrames];
                transformParams = new float[(3 + 3 * 3) * nClientsWithFrames];

                int totalDepthBytes = 0;
                int totalColorBytes = 0;

                for (int i = 0; i < nClientsWithFrames; i++)
                    lStoredBodies.Add(new List<Body>(lClientSockets[activeClients[i]].lBodies));

                for (int c = 0; c < nClientsWithFrames; c++)
                {
                    int i = activeClients[c];
                    totalDepthBytes += lClientSockets[i].frameDepth.Count();
                    totalColorBytes += lClientSockets[i].frameRGB.Count();
                    widths[c] = lClientSockets[i].iDepthFrameWidth;
                    heights[c] = lClientSockets[i].iDepthFrameHeight;
                    intrinsicsParams[7 * c] = lClientSockets[i].oCameraIntrinsicParameters.cx;
                    intrinsicsParams[7 * c + 1] = lClientSockets[i].oCameraIntrinsicParameters.cy;
                    intrinsicsParams[7 * c + 2] = lClientSockets[i].oCameraIntrinsicParameters.fx;
                    intrinsicsParams[7 * c + 3] = lClientSockets[i].oCameraIntrinsicParameters.fy;
                    intrinsicsParams[7 * c + 4] = lClientSockets[i].oCameraIntrinsicParameters.r2;
                    intrinsicsParams[7 * c + 5] = lClientSockets[i].oCameraIntrinsicParameters.r4;
                    intrinsicsParams[7 * c + 6] = lClientSockets[i].oCameraIntrinsicParameters.r6;

                    for (int j = 0; j < 3; j++)
                        transformParams[j + 12 * c] = lClientSockets[i].oWorldTransform.t[j];

                    for (int m = 0; m < 3; m++)
                        for (int n = 0; n < 3; n++)
                            transformParams[3 + m * 3 + n + 12 * c] = lClientSockets[i].oWorldTransform.R[m, n];

                }
                depthMaps = new byte[totalDepthBytes];
                depthColors = new byte[totalColorBytes];
                int copiedDepth = 0;
                int copiedColors = 0;
                for (int c = 0; c < nClientsWithFrames; c++)
                {
                    int i = activeClients[c];
                    Array.Copy(lClientSockets[i].frameDepth, 0, depthMaps, copiedDepth, lClientSockets[i].frameDepth.Count());
                    copiedDepth += lClientSockets[i].frameDepth.Count();
                    Array.Copy(lClientSockets[i].frameRGB, 0, depthColors, copiedColors, lClientSockets[i].frameRGB.Count());
                    copiedColors += lClientSockets[i].frameRGB.Count();
                }
            }
        }

        // Function doesn't put request for frames, assume, that they alreade have been copied
        public void GenerateAndGetMesh(List<VertexC4ubV3f> lFrameVerts, List<List<Body>> lFramesBody, List<int> lFramesTriagles)
        {
            GenerateMesh(lFrameVerts, lFramesTriagles);
            lFramesBody = new List<List<Body>>(lStoredBodies);
        }

        public void GetLatestFrame(List<VertexC4ubV3f> lFrameVerts, List<List<Body>> lFramesBody, List<int> lFramesTriagles)
        {
            RequestLastFrames();
            CopyLatestFrames();
            CorrectRadialDistortionsForDepthMaps();
            GenerateMesh(lFrameVerts, lFramesTriagles);
            lFramesBody = new List<List<Body>>(lStoredBodies);
        }

        public void CorrectRadialDistortionsForDepthMaps()
        {
            int nClients = nFramesCopied;

            if (nClients == 0)
                return;
            depthMapAndColorSetRadialCorrection(nClients, depthMaps, depthColors, widths, heights, intrinsicsParams);
        }

        public void GetLatestFrameVerticesOnly(List<List<VertexC4ubV3f>> lFrameVerts)
        {
            RequestLastFrames();
            CopyLatestFrames();
            CorrectRadialDistortionsForDepthMaps();

            int nClients = nFramesCopied;
            lFrameVerts.Clear();
            
            if (nClients == 0)
                return;

            for (int i = 0; i < nClients; i++)
            {
                lFrameVerts.Add(new List<VertexC4ubV3f>());
                Mesh mesh = new Mesh();
                mesh.nVertices = 0;
                generateTrianglesWithColorsFromDepthMap(nClients, depthMaps, depthColors, widths, heights, intrinsicsParams, transformParams, ref mesh, i);
                lFrameVerts[i].AddRange(CopyMeshToVerticesWithColours(mesh));
                deleteMesh(ref mesh);
            }
        }

        public void ClearStoredFrames()
        {
            lock (oClientSocketLock)
            {
                for (int i = 0; i < lClientSockets.Count; i++)
                {
                    lClientSockets[i].ClearStoredFrames();
                }
            }
        }

        private void ListeningWorker()
        {
            while (bServerRunning)
            {
                try
                {
                    Socket newClient = oServerSocket.Accept();

                    //we do not want to add new clients while a frame is being requested
                    lock (oFrameRequestLock)
                    {
                        lock (oClientSocketLock)
                        {
                            Console.WriteLine(System.Threading.Thread.CurrentThread.ManagedThreadId);
                            lClientSockets.Add(new KinectSocket(newClient));
                            lClientSockets[lClientSockets.Count - 1].SendSettings(oSettings);
                            lClientSockets[lClientSockets.Count - 1].RequestCameraIntrinsicParameters();
                            lClientSockets[lClientSockets.Count - 1].eChanged += new SocketChangedHandler(SocketListChanged);
                            if (eSocketListChanged != null)
                            {
                                eSocketListChanged(lClientSockets);
                            }
                        }
                    }
                }
                catch (SocketException)
                {
                }
                System.Threading.Thread.Sleep(100);
            }

            if (eSocketListChanged != null)
            {
                eSocketListChanged(lClientSockets);
            }
        }

        private void ReceivingWorker()
        {
            System.Timers.Timer checkConnectionTimer = new System.Timers.Timer();
            checkConnectionTimer.Interval = 1000;

            checkConnectionTimer.Elapsed += delegate(object sender, System.Timers.ElapsedEventArgs e)
            {
                lock (oClientSocketLock)
                {
                    for (int i = 0; i < lClientSockets.Count; i++)
                    {
                        if (!lClientSockets[i].SocketConnected())
                        {
                            lClientSockets.RemoveAt(i);
                            if (eSocketListChanged != null)
                            {
                                eSocketListChanged(lClientSockets);
                            }
                            continue;
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
                                lClientSockets[i].bFrameCaptured = true;
                            }
                            else if (buffer[0] == 1)
                            {
                                lClientSockets[i].ReceiveCalibrationData();
                            }
                            //stored frame
                            else if (buffer[0] == 2)
                            {
                                lClientSockets[i].ReceiveFrame();
                                lClientSockets[i].bStoredFrameReceived = true;
                                lClientSockets[i].bWaitingForFrame = false;
                            }
                        
                            //last frame
                            else if (buffer[0] == 3)
                            {
                                lClientSockets[i].ReceiveFrame();
                                lClientSockets[i].bLatestFrameReceived = true;
                                lClientSockets[i].bWaitingForFrame = false;
                            }
                            // Camera intrinsic parameters
                            else if (buffer[0] == 4)
                            {
                                lClientSockets[i].ReceiveCameraIntrinsicParameters();
                            }
                            else if (buffer[0] == 5)
                            {
                                lClientSockets[i].bNoMoreStoredFrames = true;
                                lClientSockets[i].bWaitingForFrame = false;
                            }

                            buffer = lClientSockets[i].Receive(1);
                        }
                    }
                }

                Thread.Sleep(10);
            }

            checkConnectionTimer.Stop();
        }
    }
}
