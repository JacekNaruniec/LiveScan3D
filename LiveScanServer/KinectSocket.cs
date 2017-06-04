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
using System.Net.Sockets;


namespace KinectServer
{
    public delegate void SocketChangedHandler();

    public class KinectSocket
    {
        Socket oSocket;
        byte[] byteToSend = new byte[1];
        public bool bFrameCaptured = false;
        public bool bLatestFrameReceived = false;
        public bool bStoredFrameReceived = false;
        public bool bNoMoreStoredFrames = true;
        public bool bWaitingForFrame = false;
        public bool bCalibrated = false;
        //The pose of the sensor in the scene (used by the OpenGLWindow to show the sensor)
        public AffineTransform oCameraPose = new AffineTransform();
        //The transform that maps the vertices in the sensor coordinate system to the world corrdinate system.
        public AffineTransform oWorldTransform = new AffineTransform();

        public IntrinsicCameraParameters oCameraIntrinsicParameters = new IntrinsicCameraParameters();

        public string sSocketState;

        public List<Body> lBodies = new List<Body>();
        public byte[] frameDepth;
        public byte[] frameRGB;

        public int iDepthFrameWidth = 0;
        public int iDepthFrameHeight = 0;

        // to delete
        public List<int> lTriangles = new List<int>();
        public List<byte> lFrameRGB = new List<byte>();
        public List<Single> lFrameVerts = new List<Single>();
        // ----------

        public event SocketChangedHandler eChanged;

        public KinectSocket(Socket clientSocket)
        {
            oSocket = clientSocket;
            sSocketState = oSocket.RemoteEndPoint.ToString() + " Calibrated = false";
        }

        public void CaptureFrame()
        {
            bFrameCaptured = false;
            byteToSend[0] = 0;
            SendByte();
        }

        public void Calibrate()
        {            
            bCalibrated = false;
            sSocketState = oSocket.RemoteEndPoint.ToString() + " Calibrated = false";

            byteToSend[0] = 1;
            SendByte();

            UpdateSocketState();
        }

        public void SendSettings(KinectSettings settings)
        {
            List<byte> lData = settings.ToByteList();

            byte[] bTemp = BitConverter.GetBytes(lData.Count);
            lData.InsertRange(0, bTemp);
            lData.Insert(0, 2);

            if (SocketConnected())
                oSocket.Send(lData.ToArray());
        }

        public void RequestStoredFrame()
        {
            byteToSend[0] = 3;
            SendByte();
            bNoMoreStoredFrames = false;
            bStoredFrameReceived = false;
            bWaitingForFrame = true;
        }

        public void RequestLastFrame()
        {
            byteToSend[0] = 4;
            SendByte();
            bLatestFrameReceived = false;
            bWaitingForFrame = true;
        }

        public void SendCalibrationData()
        {
            int size = 1 + (9 + 3) * sizeof(float);
            byte[] data = new byte[size];
            int i = 0;

            data[i] = 5;
            i++;

            Buffer.BlockCopy(oWorldTransform.R, 0, data, i, 9 * sizeof(float));
            i += 9 * sizeof(float);
            Buffer.BlockCopy(oWorldTransform.t, 0, data, i, 3 * sizeof(float));
            i += 3 * sizeof(float);

            if (SocketConnected())
                oSocket.Send(data);
        }

        public void ClearStoredFrames()
        {
            byteToSend[0] = 6;
            SendByte();
        }

        public void RequestCameraIntrinsicParameters()
        {
            byteToSend[0] = 7;
            SendByte();
        }
        public void Receive(byte[] buffer, int nToRead)
        {
            int nAlreadyRead = 0;

            while (nAlreadyRead != nToRead)
            {
                while (oSocket.Available == 0)
                {
                    if (!SocketConnected())
                        return;
                }

                nAlreadyRead += oSocket.Receive(buffer, nAlreadyRead, nToRead - nAlreadyRead, SocketFlags.None);
            }
        }

        public void ReceiveCameraIntrinsicParameters()
        {
            bCalibrated = true;

            byte[] buffer = new byte[7 * sizeof(Single)];
            Receive(buffer, 7 * sizeof(float));
            int pos = 0;
            oCameraIntrinsicParameters.cx = BitConverter.ToSingle(buffer, 0);
            pos += sizeof(float);
            oCameraIntrinsicParameters.cy = BitConverter.ToSingle(buffer, pos);
            pos += sizeof(float);
            oCameraIntrinsicParameters.fx = BitConverter.ToSingle(buffer, pos);
            pos += sizeof(float);
            oCameraIntrinsicParameters.fy = BitConverter.ToSingle(buffer, pos);
            pos += sizeof(float);
            oCameraIntrinsicParameters.r2 = BitConverter.ToSingle(buffer, pos);
            pos += sizeof(float);
            oCameraIntrinsicParameters.r4 = BitConverter.ToSingle(buffer, pos);
            pos += sizeof(float);
            oCameraIntrinsicParameters.r6 = BitConverter.ToSingle(buffer, pos);
        }

        public void ReceiveCalibrationData()
        {
            bCalibrated = true;

            byte[] buffer = new byte[sizeof(float) * 9];
            Receive(buffer, sizeof(int) * 1);
            //currently not used
            int markerId = BitConverter.ToInt32(buffer, 0);

            Receive(buffer, sizeof(float) * 9);
            Buffer.BlockCopy(buffer, 0, oWorldTransform.R, 0, sizeof(float) * 9);

            Receive(buffer, sizeof(float) * 3);
            Buffer.BlockCopy(buffer, 0, oWorldTransform.t, 0, sizeof(float) * 3);

            oCameraPose.R = oWorldTransform.R;
            for (int i = 0; i < 3; i++)
            {
                oCameraPose.t[i] = 0.0f;
                for (int j = 0; j < 3; j++)
                {
                    oCameraPose.t[i] += oWorldTransform.t[j] * oWorldTransform.R[i, j];
                }
            }

            UpdateSocketState();
        }

     

        public void ReceiveFrame()
        {
            lBodies.Clear();

            int nToRead = 16;
            byte[] buffer = new byte[nToRead];

            while (oSocket.Available == 0)
            {
                if (!SocketConnected())
                {
                    bNoMoreStoredFrames = true;
                    return;
                }
            }

            Receive(buffer, nToRead);

            //oSocket.Receive(buffer, 16, SocketFlags.None);
            nToRead = BitConverter.ToInt32(buffer, 0);

            if (nToRead <= 0)
            {
                bNoMoreStoredFrames = true;
                return;
            }

            int iCompressed = BitConverter.ToInt32(buffer, 4);
            int iDepthWidth = BitConverter.ToInt32(buffer, 8);
            int iDepthHeight = BitConverter.ToInt32(buffer, 12);
            iDepthFrameWidth = iDepthWidth;
            iDepthFrameHeight = iDepthHeight;

            buffer = new byte[nToRead];

            Receive(buffer, nToRead);

            if (iCompressed == 1)
                buffer = ZSTDDecompressor.Decompress(buffer);

            if (iDepthWidth == 0)
                iDepthWidth = 0;

            frameDepth = new byte[iDepthWidth * iDepthHeight * 2];
            frameRGB = new byte[iDepthWidth * iDepthHeight * 3];

            Array.Copy(buffer, frameDepth, iDepthWidth * iDepthHeight * 2);
            Array.Copy(buffer, iDepthWidth * iDepthHeight * 2, frameRGB, 0, iDepthWidth * iDepthHeight * 3);

            //Receive body data
            int startIdx = iDepthWidth * iDepthHeight * 5;

            //Receive body data
            int nBodies = BitConverter.ToInt32(buffer, startIdx);

            startIdx += 4;
            for (int i = 0; i < nBodies; i++)
            {
                Body tempBody = new Body();
                tempBody.bTracked = BitConverter.ToBoolean(buffer, startIdx++);
                int nJoints = BitConverter.ToInt32(buffer, startIdx);
                startIdx += 4;

                tempBody.lJoints = new List<Joint>(nJoints);
                tempBody.lJointsInColorSpace = new List<Point2f>(nJoints);

                for (int j = 0; j < nJoints; j++)
                {
                    Joint tempJoint = new Joint();
                    Point2f tempPoint = new Point2f();

                    tempJoint.jointType = (JointType)BitConverter.ToInt32(buffer, startIdx);
                    startIdx += 4;
                    tempJoint.trackingState = (TrackingState)BitConverter.ToInt32(buffer, startIdx);
                    startIdx += 4;
                    tempJoint.position.X = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;
                    tempJoint.position.Y = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;
                    tempJoint.position.Z = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;

                    tempPoint.X = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;
                    tempPoint.Y = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;

                    tempBody.lJoints.Add(tempJoint);
                    tempBody.lJointsInColorSpace.Add(tempPoint);
                }

                lBodies.Add(tempBody);
            }
        }


        
        public byte[] Receive(int nBytes)
        {
            byte[] buffer;
            if (oSocket.Available != 0)
            {
                buffer = new byte[Math.Min(nBytes, oSocket.Available)];
                oSocket.Receive(buffer, nBytes, SocketFlags.None);
            }
            else
                buffer = new byte[0];

            return buffer;
        }
        
        public bool SocketConnected()
        {
            bool part1 = oSocket.Poll(1000, SelectMode.SelectRead);
            bool part2 = (oSocket.Available == 0);

            if (part1 && part2)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        private void SendByte()
        {
            try
            {
                oSocket.Send(byteToSend);
            }
            catch(SocketException)
            {
                return;
            }
            catch(ObjectDisposedException)
            {
                return;
            }
        }

        public void UpdateSocketState()
        {
            if (bCalibrated)
                sSocketState = oSocket.RemoteEndPoint.ToString() + " Calibrated = true";
            else
                sSocketState = oSocket.RemoteEndPoint.ToString() + " Calibrated = false";

            if (eChanged != null)
                eChanged();
        }
    }
}
