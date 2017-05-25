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
            int[] widths, int[] heights, float []iparams, float []tparams, ref Mesh out_mesh);

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

        public bool GetStoredFrame(List<List<byte>> lFramesRGB, List<List<Single>> lFramesVerts)
        {
            bool bNoMoreStoredFrames;
            lFramesRGB.Clear();
            lFramesVerts.Clear();
            
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
                while (!allGathered)
                {
                    allGathered = true;                
                    lock (oClientSocketLock)
                    {
                        for (int i = 0; i < lClientSockets.Count; i++)
                        {
                            if (!lClientSockets[i].bStoredFrameReceived)
                            {
                                allGathered = false;
                                break;
                            }

                            if (lClientSockets[i].bNoMoreStoredFrames)
                                bNoMoreStoredFrames = true;
                        }
                    }
                }

                //Store received frames
                lock (oClientSocketLock)
                {
                    for (int i = 0; i < lClientSockets.Count; i++)
                    {
                        lFramesRGB.Add(new List<byte>(lClientSockets[i].lFrameRGB));
                        lFramesVerts.Add(new List<Single>(lClientSockets[i].lFrameVerts));
                    }
                }
            }

            if (bNoMoreStoredFrames)
                return false;
            else
                return true;
        }

        private List<int> copyMeshToTrianglesList(Mesh mesh)
        {
            if (mesh.nTriangles == 0)
                return new List<int>();

            int[] triangles = new int[mesh.nTriangles * 3];

            Marshal.Copy(mesh.triangles, triangles, 0, mesh.nTriangles * 3);

            return triangles.ToList<int>();
        }

        private List<Single> copyMeshToVerticesList(Mesh mesh)
        {
            int nElements = mesh.nVertices;

            if (nElements == 0)
                return new List<Single>();

            Single[] vertices = new Single[nElements * 3];
            Marshal.Copy(mesh.vertices, vertices, 0, nElements * 3);

            return vertices.ToList<Single>(); 
        }

        private List<byte> copyMeshToRGBList(Mesh mesh)
        {
            int nElements = mesh.nVertices;

            if (nElements == 0)
                return new List<byte>();

            byte[] rgb = new byte[nElements * 3];
            Marshal.Copy(mesh.verticesRGB, rgb, 0, nElements * 3);

            return rgb.ToList<byte>();
        }

        public void GenerateMesh(List<byte> lFrameRGB, List<Single> lFrameVerts, List<int> lFrameTriagles)
        {
            int nClients = lClientSockets.Count; 
            byte[] depthMaps;
            byte[] depthColors;
            int[] widths = new int[nClients];
            int[] heights = new int[nClients];
            float[] intrinsicsParams = new float[7 * nClients];
            float[] transformParams = new float[(3 + 3 * 3) * nClients];
            Mesh mesh = new Mesh();
            mesh.nVertices = 0;

            int totalDepthBytes = 0;
            int totalColorBytes = 0;
            lock (oClientSocketLock)
            {
                for (int i = 0; i < nClients; i++)
                {
                    totalDepthBytes += lClientSockets[i].frameDepth.Count();
                    totalColorBytes += lClientSockets[i].frameRGB.Count();
                    widths[i] = lClientSockets[i].iDepthFrameWidth;
                    heights[i] = lClientSockets[i].iDepthFrameHeight;
                    intrinsicsParams[7 * i] = lClientSockets[i].oCameraIntrinsicParameters.cx;
                    intrinsicsParams[7 * i + 1] = lClientSockets[i].oCameraIntrinsicParameters.cy;
                    intrinsicsParams[7 * i + 2] = lClientSockets[i].oCameraIntrinsicParameters.fx;
                    intrinsicsParams[7 * i + 3] = lClientSockets[i].oCameraIntrinsicParameters.fy;
                    intrinsicsParams[7 * i + 4] = lClientSockets[i].oCameraIntrinsicParameters.r2;
                    intrinsicsParams[7 * i + 5] = lClientSockets[i].oCameraIntrinsicParameters.r4;
                    intrinsicsParams[7 * i + 6] = lClientSockets[i].oCameraIntrinsicParameters.r6;

                    for (int j = 0; j < 3; j++)
                        transformParams[j + 12 * i] = lClientSockets[i].oWorldTransform.t[j];

                    for (int m = 0; m < 3; m++)
                        for (int n = 0; n < 3; n++)
                            transformParams[3 + m * 3 + n + 12 * i] = lClientSockets[i].oWorldTransform.R[m, n];

                }
                depthMaps = new byte[totalDepthBytes];
                depthColors = new byte[totalColorBytes];
                int copiedDepth = 0;
                int copiedColors = 0;
                for (int i = 0; i < nClients; i++)
                {
                    Array.Copy(lClientSockets[i].frameDepth, 0, depthMaps, copiedDepth, lClientSockets[i].frameDepth.Count());
                    copiedDepth += lClientSockets[i].frameDepth.Count();
                    Array.Copy(lClientSockets[i].frameRGB, 0, depthColors, copiedColors, lClientSockets[i].frameRGB.Count());
                    copiedColors += lClientSockets[i].frameRGB.Count();
                }
            }

            generateMeshFromDepthMaps(nClients, depthMaps, depthColors, widths, heights, intrinsicsParams, transformParams, ref mesh);

            lFrameRGB.AddRange(copyMeshToRGBList(mesh));
            lFrameVerts.AddRange(copyMeshToVerticesList(mesh));
            lFrameTriagles.AddRange(copyMeshToTrianglesList(mesh));

            deleteMesh(ref mesh);

        }
        
        public void GetLatestFrame(List<List<byte>> lFramesRGB, List<List<Single>> lFramesVerts, List<List<Body>> lFramesBody, List<List<int>> lFramesTriagles)
        {
            lFramesRGB.Clear();
            lFramesVerts.Clear();
            lFramesBody.Clear();
            lFramesTriagles.Clear();

            lock (oFrameRequestLock)
            {
                //Request frames
                lock (oClientSocketLock)
                {
                    for (int i = 0; i < lClientSockets.Count; i++)
                        lClientSockets[i].RequestLastFrame();
                }

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
                            {
                                allGathered = false;
                                break;
                            }
                        }
                    }

                }
            }

            List<byte> lFrameRGB = new List<byte>();
            List<Single> lFrameVerts = new List<Single>();
            List<int> lFrameTriagles = new List<int>();
            GenerateMesh(lFrameRGB, lFrameVerts, lFrameTriagles);

            //Store received frames
            lock (oClientSocketLock)
            {
                lFramesRGB.Add(new List<byte>(lFrameRGB));
                lFramesVerts.Add(new List<Single>(lFrameVerts));
                lFramesTriagles.Add(new List<int>(lFrameTriagles));
                for (int i = 0; i < lClientSockets.Count; i++)
                    lFramesBody.Add(new List<Body>(lClientSockets[i].lBodies));
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
                            }
                            //last frame
                            else if (buffer[0] == 3)
                            {
                                lClientSockets[i].ReceiveFrame();
                                lClientSockets[i].bLatestFrameReceived = true;
                            }
                            // Camera intrinsic parameters
                            else if (buffer[0] == 4)
                            {
                                lClientSockets[i].ReceiveCameraIntrinsicParameters();
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
