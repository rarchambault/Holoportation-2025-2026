/***************************************************************************\

Module Name:  CameraServer.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module is the server used to launch client processes for each connected
camera and to communicate with them.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;

namespace LiveScanServer
{
    // Delegate to register any change in a list of clients
    public delegate void ClientListChangedHandler(List<CameraClient> list);

    public class CameraServer
    {
        public int ClientCount
        {
            get
            {
                int nClients;

                lock (clientLock)
                {
                    nClients = liveScanClients.Count;
                }

                return nClients;
            }
        }

        public bool AllCamerasCalibrated
        {
            get
            {
                bool allCalibrated = true;

                lock (clientLock)
                {
                    foreach (var client in liveScanClients)
                    {
                        if (!client.IsCalibrated)
                        {
                            allCalibrated = false;
                            break;
                        }
                    }
                }

                return allCalibrated;
            }
        }

        public List<AffineTransform> CameraPoses
        {
            get
            {
                List<AffineTransform> cameraPoses = new List<AffineTransform>();

                lock (clientLock)
                {
                    foreach (var client in liveScanClients)
                    {
                        cameraPoses.Add(client.CameraPose);
                    }
                }

                return cameraPoses;
            }
            set
            {
                lock (clientLock)
                {
                    for (int i = 0; i < liveScanClients.Count; i++)
                    {
                        liveScanClients[i].CameraPose = value[i];
                    }
                }
            }
        }

        public List<AffineTransform> WorldTransforms
        {
            get
            {
                List<AffineTransform> worldTransforms = new List<AffineTransform>();

                lock (clientLock)
                {
                    foreach (var client in liveScanClients)
                    {
                        worldTransforms.Add(client.WorldTransform);
                    }
                }

                return worldTransforms;
            }

            set
            {
                lock (clientLock)
                {
                    for (int i = 0; i < liveScanClients.Count; i++)
                    {
                        liveScanClients[i].WorldTransform = value[i];
                    }
                }
            }
        }

        public event ClientListChangedHandler OnClientListChanged;
        public DocumentInfo DocumentInfo = new DocumentInfo();

        private bool waitForSubordinateStart = false;

        // This is used to prevent enabling/disabling the Sync State while the cameras are in transition to another state
        // When starting the server, all cameras are already initialized, as the LiveScanClient can only connect once it is initialized
        private bool allDevicesInitialized = true; 

        private CameraSettings cameraSettings;
        private SettingsForm settingsForm;
        private List<CameraClient> liveScanClients = new List<CameraClient>();

        private object clientLock = new object();
        private object frameRequestLock = new object();
        private object documentDataLock = new object();

        private int counter = 0;

        public CameraServer(CameraSettings settings)
        {
            this.cameraSettings = settings;
        }

        public void SetSettingsForm(SettingsForm settings)
        {
            settingsForm = settings;
        }

        public SettingsForm GetSettingsForm()
        {
            return settingsForm;
        }

        public bool GetAllDevicesInitialized()
        {
            return allDevicesInitialized;
        }

        /// <summary>
        /// Launches <paramref name="count"/> camera client processes
        /// </summary>
        /// <param name="count">Number of camera client processes to start</param>
        public void LaunchClients(uint count)
        {
            // Start multiple instances of LiveScanClient
            for (int i = 0; i < count; i++)
            {
                var client = new CameraClient(i);
                liveScanClients.Add(client);

                // Set callbacks so the client can call server methods directly
                client.SetSendSerialNumberCallback(OnReceiveSerialNumber);
                client.SetConfirmRecordedCallback();
                client.SetConfirmCalibratedCallback(OnConfirmCalibrated);
                client.SetSendLatestFrameCallback();
                client.SetSendRecordedFrameCallback();
                client.SetConfirmSyncStateCallback(OnConfirmSyncState);
                client.SetConfirmMasterRestartCallback(OnConfirmMasterRestart);
                client.SetSendDocumentCallback(OnReceiveDocument);
                client.Start();
                
                // Send settings
                client.SetSettings(cameraSettings);
            }

            // Update client list in the main UI form
            ClientListChanged();
        }

        public void StopServer()
        {
            // Ensure all LiveScanClients are terminated
            foreach (var client in liveScanClients)
            {
                client.Stop();
            }
        }

        /// <summary>
        /// Starts frame recording for each connected camera client
        /// </summary>
        public void StartFrameRecording()
        {
            // Tell each connected client to start capturing frames
            lock (clientLock)
            {
                foreach (var client in liveScanClients)
                {
                    client.StartFrameRecording();
                }
            }

            // Wait for all frames to be recorded
            bool allGathered = false;
            while (!allGathered)
            {
                allGathered = true;

                lock (clientLock)
                {
                    foreach (var client in liveScanClients)
                    {
                        if (!client.IsFrameRecorded)
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
            lock (clientLock)
            {
                foreach (var client in liveScanClients)
                {
                    client.Calibrate();
                }
            }
        }

        public void SendSettings()
        {
            lock (clientLock)
            {
                foreach (var client in liveScanClients)
                {
                    client.SetSettings(cameraSettings);
                }
            }
        }

        public void SendCalibrationData()
        {
            lock (clientLock)
            {
                foreach (var client in liveScanClients)
                {
                    client.ReceiveCalibration();
                }
            }
        }

        /// <summary>
        /// Enables sync by assigning roles to each connected device
        /// </summary>
        public void EnableSync()
        {
            lock (clientLock)
            {
                allDevicesInitialized = false;
                waitForSubordinateStart = true;

                // Sort clients by their serial numbers (ascending)
                List<CameraClient> sortedClients = liveScanClients
                    .Where(c => !string.IsNullOrEmpty(c.SerialNumber))
                    .OrderBy(c => c.SerialNumber, StringComparer.Ordinal)
                    .ToList();

                if (sortedClients.Count == 0)
                    return;

                // First client becomes MASTER
                CameraClient masterClient = sortedClients[0];
                masterClient.EnableSync((int)SyncState.Master, 0);

                // All others become SUBORDINATE
                for (int i = 1; i < sortedClients.Count; i++)
                {
                    sortedClients[i].EnableSync((int)SyncState.Subordinate, i);
                }

                // Any clients not in sortedClients are set to STANDALONE
                var standaloneClients = liveScanClients.Except(sortedClients);
                foreach (var c in standaloneClients)
                {
                    c.EnableSync((int)SyncState.Standalone, 0);
                }
            }
        }

        public void DisableSync()
        {
            allDevicesInitialized = false;

            lock (clientLock)
            {
                foreach (var client in liveScanClients)
                {
                    client.DisableSync();
                }
            }
        }

        /// <summary>
        /// Tries to get a recorded frame from each connected camera client
        /// </summary>
        /// <param name="frameColors">List where to store the recorded frame's colors</param>
        /// <param name="framesVertices">List where to store the recorded frame's vertices</param>
        /// <returns>True if a recorded frame was succesfully retrieved, false if there are no more recorded frames to retrieve</returns>
        public bool TryGetRecordedFrame(ref List<List<byte>> frameColors, ref List<List<float>> framesVertices)
        {
            bool noMoreRecordedFrames;
            int count = frameColors.Count;

            // Ensure list capacity
            if (framesVertices.Capacity < count)
                framesVertices.Capacity = count;
            if (frameColors.Capacity < count)
                frameColors.Capacity = count;

            frameColors.Clear();
            framesVertices.Clear();
            
            lock (frameRequestLock)
            {
                // Request a recorded frame from each connected client
                lock (clientLock)
                {
                    foreach (var client in liveScanClients)
                    {
                        client.IsRecordedFrameReceived = false;
                        client.NoMoreRecordedFrames = false;
                        client.RequestRecordedFrame();
                    }
                }

                // Wait for all frames to be received
                bool allGathered = false;
                noMoreRecordedFrames = false;

                while (!allGathered)
                {
                    allGathered = true;                
                    lock (clientLock)
                    {
                        foreach (var client in liveScanClients)
                        {
                            if (!client.IsRecordedFrameReceived)
                            {
                                allGathered = false;
                                break;
                            }

                            if (client.NoMoreRecordedFrames)
                                noMoreRecordedFrames = true;
                        }
                    }
                }

                // Store received frame from each client in the provided lists
                lock (clientLock)
                {
                    foreach (var client in liveScanClients)
                    {
                        frameColors.Add(client.FrameColors);
                        framesVertices.Add(client.FrameVertices);
                    }
                }
            }

            if (noMoreRecordedFrames)
                return false;
            else
                return true;
        }

        /// <summary>
        /// Gets the latest frame processed by the connected camera clients
        /// </summary>
        /// <param name="frameColors">List where to store the frame's colors</param>
        /// <param name="framesVertices">List where to store the frame's vertices</param>
        public void GetLatestFrame(ref List<List<byte>> frameColors, ref List<List<float>> framesVertices)
        {
            int count = frameColors.Count;

            // Ensure list capacity
            if (framesVertices.Capacity < count)
                framesVertices.Capacity = count;
            if (frameColors.Capacity < count)
                frameColors.Capacity = count;

            frameColors.Clear();
            framesVertices.Clear();

            lock (frameRequestLock)
            {
                // Request the latest frame from each connected client
                lock (clientLock)
                {
                    foreach (var client in liveScanClients)
                    {
                        client.IsLatestFrameReceived = false;
                        client.RequestLatestFrame();
                    }
                }

                // Wait for all frames to be received
                bool allGathered = false;

                while (!allGathered)
                {
                    allGathered = true;

                    lock (clientLock)
                    {
                        foreach (var client in liveScanClients)
                        {
                            if (!client.IsLatestFrameReceived)
                            {
                                allGathered = false;
                                break;
                            }
                        }
                    }
                }

                // Store received frames from each client in the provided lists
                lock (clientLock)
                {
                    foreach(var client in liveScanClients)
                    {
                        frameColors.Add(client.FrameColors);
                        framesVertices.Add(client.FrameVertices);
                    }
                }
            }
        }

        /// <summary>
        /// Tells each connected client to clear its internal recorded frame lists
        /// </summary>
        public void ClearRecordedFrames()
        {
            lock (clientLock)
            {
                foreach (var client in liveScanClients)
                {
                    client.ClearRecordedFrames();
                }
            }
        }


        private void ConfirmSyncDisabled()
        {
            // Check that we are not still waiting for a subordinate camera to start
            if (waitForSubordinateStart)
                return;

            // Check that each client has the STANDALONE role
            lock (clientLock)
            {
                foreach (var client in liveScanClients)
                {
                    if (client.CurrentSyncState != SyncState.Standalone)
                    {
                        return;
                    }
                }
            }

            // Mark that all devices are now initialized
            allDevicesInitialized = true;
        }

        // Called when a subordinate client has started. Checks if all subordinate clients have already started and initializes the master if they have.
        private void StartMaster()
        {
            // Check that we are still waiting for some subordinate clients to start
            if (!waitForSubordinateStart)
            {
                return;
            }

            // Check if all subordinate clients have started now
            bool allSubsStarted = true;

            lock (clientLock)
            {
                foreach (var client in liveScanClients)
                {
                    if (!client.IsStarted && client.CurrentSyncState == SyncState.Subordinate)
                    {
                        allSubsStarted = false;
                        break;
                    }
                }

                waitForSubordinateStart = false;

                if (allSubsStarted)
                {
                    // Start the master client
                    foreach (var client in liveScanClients)
                    {
                        if (client.CurrentSyncState == SyncState.Master)
                        {
                            client.StartMaster();
                            return;
                        }
                    }
                }
            }
        }

        // Confirms that the master camera has restarted and therefore all cameras are initialized after enabling sync
        private void MasterSuccessfullyRestarted()
        {
            allDevicesInitialized = true;
        }

        private void OnConfirmCalibrated()
        {
            ClientListChanged();
        }

        private void OnReceiveSerialNumber()
        {
            ClientListChanged();
        }

        private void OnConfirmSyncState(int clientIndex, SyncState state)
        {
            // Set local client sync state
            CameraClient client = liveScanClients[clientIndex];
            client.CurrentSyncState = state;

            // Adjust local variables based on the state that was confirmed
            if (state == SyncState.Subordinate)
            {
                client.IsStarted = true;
                StartMaster();
            }
            else if (state == SyncState.Master)
            {
                client.IsStarted = false;
            }
            else if (state == SyncState.Standalone)
            {
                client.IsStarted = true;
                ConfirmSyncDisabled();
            }

            // Update client status
            client.UpdateSocketState();
            ClientListChanged();
        }

        private void OnConfirmMasterRestart(int clientIndex)
        {
            MasterSuccessfullyRestarted();
        }

        private void OnReceiveDocument(int clientIndex)
        {
            lock (documentDataLock)
            {
                CameraClient client = liveScanClients[clientIndex];

                if (client.DocumentData == null || client.DocumentData.Count == 0)
                {
                    return;
                }

                DocumentInfo.Data = new List<byte>(client.DocumentData);
                DocumentInfo.Score = client.DocumentScore; ;
                DocumentInfo.Width = client.DocumentWidth; ;
                DocumentInfo.Height = client.DocumentHeight; ;
                DocumentInfo.IsNew = true;
            }
        }

        // Calls event handler to modify the client list in the main window form
        private void ClientListChanged()
        {
            if (OnClientListChanged != null)
            {
                OnClientListChanged(liveScanClients);
            }
        }
    }
}
