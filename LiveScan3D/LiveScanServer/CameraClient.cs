/***************************************************************************\

Module Name:  CameraClient.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module is the C#-side client wrapper which imports all required functions
from the C++ LiveScanClient program to control cameras and retrieve their data.
It allows communication between the C++ and C# processes.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace LiveScanServer
{
    public class CameraClient
    {
        #region Server to client (outbound) call imports

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr CreateClient(int index);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void StartClient(IntPtr handle);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void StopClient(IntPtr handle);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void DestroyClient(IntPtr handle);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void StartFrameRecording(IntPtr handle);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void Calibrate(IntPtr handle);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void SetSettings(IntPtr handle, ref NativeCameraSettings settings);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void RequestRecordedFrame(IntPtr handle);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void RequestLatestFrame(IntPtr handle);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void ReceiveCalibration(IntPtr handle, ref NativeAffineTransform calibration);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void ClearRecordedFrames(IntPtr handle);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void EnableSync(IntPtr handle, int syncState, int syncOffset);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void DisableSync(IntPtr handle);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void StartMaster(IntPtr handle);

        #endregion

        #region Client to server (inbound) call imports
        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void SetSendSerialNumberCallback(IntPtr handle, SendSerialNumberCallback cb);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void SetConfirmRecordedCallback(IntPtr handle, ConfirmRecordedCallback cb);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void SetConfirmCalibratedCallback(IntPtr handle, ConfirmCalibratedCallback cb);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void SetSendLatestFrameCallback(IntPtr handle, SendLatestFrameCallback callback);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void SetSendRecordedFrameCallback(IntPtr handle, SendRecordedFrameCallback callback);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void SetConfirmSyncStateCallback(IntPtr handle, ConfirmSyncStateCallback callback);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void SetConfirmMasterRestartCallback(IntPtr handle, ConfirmMasterRestartCallback callback);

        [DllImport("LiveScanClient.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void SetSendDocumentCallback(IntPtr handle, SendDocumentCallback callback);

        // Callback definitions
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void SendSerialNumberCallback(int clientIndex, [MarshalAs(UnmanagedType.LPStr)] string serialNumber);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void ConfirmRecordedCallback(int clientIndex);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public unsafe delegate void ConfirmCalibratedCallback(int clientIndex, int markerId, float* R, float* t);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public unsafe delegate void SendLatestFrameCallback(int clientIndex, Point3s* vertices, RGB* colors, int count);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public unsafe delegate void SendRecordedFrameCallback(int clientIndex, Point3s* vertices, RGB* colors, int count, byte noMoreFrames);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void ConfirmSyncStateCallback(int clientIndex, int syncState);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void ConfirmMasterRestartCallback(int clientIndex);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public unsafe delegate void SendDocumentCallback(int clientIndex, byte* data, float score, short width, short height);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void SendDeviceSyncStateCallback(int clientIndex, int syncState);

        #endregion

        public bool IsFrameRecorded = false;
        public bool IsCalibrated = false;
        public bool IsLatestFrameReceived = false;
        public bool IsRecordedFrameReceived = false;
        public bool NoMoreRecordedFrames = true;
        public bool IsStarted = false;

        public string SerialNumber = "XXXXXXXXXXX";
        public string ClientState;

        public SyncState CurrentSyncState = SyncState.Standalone;

        // Pose of the camera in the scene (used by the OpenGLWindow to show the sensor)
        public AffineTransform CameraPose = new AffineTransform();

        // Transform that maps the vertices in the camera coordinate system to the world coordinate system
        public AffineTransform WorldTransform = new AffineTransform();

        public List<byte> FrameColors = new List<byte>();
        public List<float> FrameVertices = new List<float>();

        public List<byte> DocumentData = new List<byte>();
        public float DocumentScore = 0.0f;
        public short DocumentWidth = 0;
        public short DocumentHeight = 0;

        private int clientIndex;
        private IntPtr clientHandle;

        // Callbacks for client to server calls
        private SendSerialNumberCallback sendSerialNumberCallback;
        private ConfirmRecordedCallback confirmRecordedCallback;
        private ConfirmCalibratedCallback confirmCalibratedCallback;
        private SendLatestFrameCallback sendLatestFrameCallback;
        private SendRecordedFrameCallback sendRecordedFrameCallback;
        private ConfirmSyncStateCallback confirmSyncStateCallback;
        private ConfirmMasterRestartCallback confirmMasterRestartCallback;
        private SendDocumentCallback sendDocumentCallback;

        public CameraClient(int index)
        {
            clientHandle = CreateClient(index);
            clientIndex = index;
            ClientState = "[Client " + clientIndex.ToString() + "] Calibrated = false";

            UpdateSocketState();
        }

        public void Start() => StartClient(clientHandle);
       
        public void Stop() => StopClient(clientHandle);
        
        public void Dispose()
        {
            Stop();
            DestroyClient(clientHandle);
            clientHandle = IntPtr.Zero;
        }

        public void StartFrameRecording()
        {
            IsFrameRecorded = false;
            StartFrameRecording(clientHandle);
        }

        public void Calibrate()
        {
            IsCalibrated = false;
            UpdateSocketState();

            Calibrate(clientHandle);
        }

        public void SetSettings(CameraSettings settings)
        {
            var native = settings.ToNative(out GCHandle markerHandle);

            try
            {
                SetSettings(clientHandle, ref native);
            }
            finally
            {
                if (markerHandle.IsAllocated)
                    markerHandle.Free();
            }
        }

        public void RequestRecordedFrame() => RequestRecordedFrame(clientHandle);
        
        public void RequestLatestFrame() => RequestLatestFrame(clientHandle);

        public void ReceiveCalibration()
        {
            var native = WorldTransform.ToNative();
            ReceiveCalibration(clientHandle, ref native);
        }

        public void ClearRecordedFrames() => ClearRecordedFrames(clientHandle);

        public void EnableSync(int syncState, int syncOffset)
        {
            IsStarted = false;
            CurrentSyncState = SyncState.Unknown;
            EnableSync(clientHandle, syncState, syncOffset);
        }

        public void DisableSync()
        {
            IsStarted = false;
            CurrentSyncState = SyncState.Unknown;
            DisableSync(clientHandle);
        }

        public void StartMaster() => StartMaster(clientHandle);

        public void SetSendSerialNumberCallback(Action callback)
        {
            sendSerialNumberCallback = new SendSerialNumberCallback((int index, string serial) =>
            {
                SerialNumber = serial;
                UpdateSocketState();
                callback();
            });

            SetSendSerialNumberCallback(clientHandle, sendSerialNumberCallback);
        }

        public void SetConfirmRecordedCallback()
        {
            confirmRecordedCallback = new ConfirmRecordedCallback((int clientIndex) => 
            {
                IsFrameRecorded = true;
            });

            SetConfirmRecordedCallback(clientHandle, confirmRecordedCallback);
        }

        public unsafe void SetConfirmCalibratedCallback(Action callback)
        {
            confirmCalibratedCallback = new ConfirmCalibratedCallback((clientIndex, markerId, R, t) =>
            {
                float[] rotation = new float[9];
                float[] translation = new float[3];
                Marshal.Copy((IntPtr)R, rotation, 0, 9);
                Marshal.Copy((IntPtr)t, translation, 0, 3);

                WorldTransform = new AffineTransform
                {
                    R = new float[3, 3],
                    T = new float[3]
                };

                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        WorldTransform.R[i, j] = R[i * 3 + j];

                for (int i = 0; i < 3; i++)
                    WorldTransform.T[i] = t[i];

                CameraPose.R = WorldTransform.R;
                for (int i = 0; i < 3; i++)
                {
                    CameraPose.T[i] = 0.0f;
                    for (int j = 0; j < 3; j++)
                    {
                        CameraPose.T[i] += WorldTransform.T[j] * WorldTransform.R[i, j];
                    }
                }

                IsCalibrated = true;
                UpdateSocketState();
                callback();
            });

            SetConfirmCalibratedCallback(clientHandle, confirmCalibratedCallback);
        }

        public unsafe void SetSendLatestFrameCallback()
        {
            sendLatestFrameCallback = new SendLatestFrameCallback((int index, Point3s* vertices, RGB* colors, int count) =>
            {
                // Ensure list capacity
                if (FrameVertices.Capacity < count * 3)
                    FrameVertices.Capacity = count * 3;
                if (FrameColors.Capacity < count * 3)
                    FrameColors.Capacity = count * 3;

                FrameVertices.Clear();
                FrameColors.Clear();

                for (int i = 0; i < count; i++)
                {
                    // Divide vertex data by 1000 to convert to meters
                    FrameVertices.Add(vertices[i].X / 1000.0f);
                    FrameVertices.Add(vertices[i].Y / 1000.0f);
                    FrameVertices.Add(vertices[i].Z / 1000.0f);

                    FrameColors.Add(colors[i].Red);
                    FrameColors.Add(colors[i].Green);
                    FrameColors.Add(colors[i].Blue);
                }

                IsLatestFrameReceived = true;
            });

            SetSendLatestFrameCallback(clientHandle, sendLatestFrameCallback);
        }

        public unsafe void SetSendRecordedFrameCallback()
        {
            sendRecordedFrameCallback = new SendRecordedFrameCallback((int index, Point3s* vertices, RGB* colors, int count, byte noMoreFrames) =>
            {
                if (noMoreFrames != 0)
                {
                    NoMoreRecordedFrames = true;
                    IsRecordedFrameReceived = true;
                    return;
                }

                // Ensure list capacity
                if (FrameVertices.Capacity < count * 3)
                    FrameVertices.Capacity = count * 3;
                if (FrameColors.Capacity < count * 3)
                    FrameColors.Capacity = count * 3;

                FrameVertices.Clear();
                FrameColors.Clear();

                for (int i = 0; i < count; i++)
                {
                    // Divide vertex data by 1000 to convert to meters
                    FrameVertices.Add(vertices[i].X / 1000.0f);
                    FrameVertices.Add(vertices[i].Y / 1000.0f);
                    FrameVertices.Add(vertices[i].Z / 1000.0f);

                    FrameColors.Add(colors[i].Red);
                    FrameColors.Add(colors[i].Green);
                    FrameColors.Add(colors[i].Blue);
                }

                IsRecordedFrameReceived = true;
            });

            SetSendRecordedFrameCallback(clientHandle, sendRecordedFrameCallback);
        }

        public void SetConfirmSyncStateCallback(Action<int, SyncState> callback)
        {
            confirmSyncStateCallback = new ConfirmSyncStateCallback((int index, int state) =>
            {
               SyncState config = SyncState.Unknown;

               switch (state)
               {
                    case 0:
                        config = SyncState.Subordinate;
                        break;
                    case 1:
                        config = SyncState.Master;
                        break;
                    case 2:
                        config = SyncState.Standalone;
                        break;
                    default:
                        config = SyncState.Unknown;
                        break;
               }

               callback(index, config);
            });

            SetConfirmSyncStateCallback(clientHandle, confirmSyncStateCallback);
        }

        public void SetConfirmMasterRestartCallback(Action<int> callback)
        {
            confirmMasterRestartCallback = new ConfirmMasterRestartCallback((int index) =>
            {
                IsStarted = true;
                callback(index);
            });

            SetConfirmMasterRestartCallback(clientHandle, confirmMasterRestartCallback);
        }

        public unsafe void SetSendDocumentCallback(Action<int> callback)
        {
            sendDocumentCallback = new SendDocumentCallback((int index, byte* data, float score, short width, short height) =>
            {
                DocumentData.Clear();
                DocumentData.Capacity = width * height * 3; // pre-allocate for speed

                // Copy raw bytes into managed List<byte>
                for (int i = 0; i < width * height * 3; i++)
                {
                    DocumentData.Add(data[i]);
                }

                DocumentScore = score;
                DocumentWidth = width;
                DocumentHeight = height;

                callback(index);
            });

            SetSendDocumentCallback(clientHandle, sendDocumentCallback);
        }

        public void UpdateSocketState()
        {
            string syncMessage = "";

            switch (CurrentSyncState)
            {
                case SyncState.Master:
                    syncMessage = "[MASTER]";
                    break;
                case SyncState.Subordinate:
                    syncMessage = "[SUBORDINATE]";
                    break;
                case SyncState.Standalone:
                    syncMessage = "[STANDALONE]";
                    break;
                default:
                    break;
            }

            ClientState = "[Client " + clientIndex.ToString() + " ( " + SerialNumber +  ")] Calibrated = " + IsCalibrated + " " + syncMessage;
        }
    }
}
