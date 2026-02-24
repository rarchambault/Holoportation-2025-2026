/***************************************************************************\

Module Name:  MainWindowForm.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module is the logic behind the main UI form of the application. It
launches the CameraServer used to launch the different camera clients
and retrieves its data. It is also used to apply settings to the cameras and
visualize the output reconstruction.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Runtime.InteropServices;
using System.Runtime.Serialization;
using System.Threading;
using System.Windows.Forms;

namespace LiveScanServer
{
    public partial class MainWindowForm : Form
    {
        // DLL imports for some required methods from the Orbbec SDK
        [DllImport("OrbbecSDK.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr ob_create_context();

        [DllImport("OrbbecSDK.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr ob_query_device_list(IntPtr ctx);

        [DllImport("OrbbecSDK.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern UIntPtr ob_device_list_device_count(IntPtr devList);

        // DLL import for the ICP method (used for camera pose estimation)
        [DllImport("ICP.dll")]
        private static extern float ICP(IntPtr verts1, IntPtr verts2, int nVerts1, int nVerts2, float[] R, float[] t, int maxIter = 200);

        private bool isRecording = false;
        private bool isSaving = false;
        private bool isLiveViewRunning = false;

        // Camera server used to retrieve data from the connected cameras
        private CameraServer cameraServer;

        // Transfer server used to send the reconstruction to connected clients
        private TransferServer transferServer;

        private CameraSettings settings = new CameraSettings();
        private OpenGLWindow openGLWindow;
        private System.Timers.Timer statusBarTimer = new System.Timers.Timer();

        // Vertices from all of the cameras
        private List<float> vertices = new List<float>();

        // Vertices from each camera, separated in lists
        private List<List<float>> cameraVertices = new List<List<float>>();

        // Recorded frame vertices from each camera, separated in lists
        private List<List<float>> cameraRecordedVertices = new List<List<float>>();

        // Color data from all of the cameras
        private List<byte> colors = new List<byte>();

        // Color data from each camera, separated in lists
        private List<List<byte>> cameraColors = new List<List<byte>>();

        // Recorded frame color data from each camera, separated in lists
        private List<List<byte>> cameraRecordedColors = new List<List<byte>>();

        // Position from each camera
        private List<AffineTransform> cameraPoses = new List<AffineTransform>();

        public MainWindowForm()
        {
            // Tries to read the settings from "settings.bin". If it fails, the settings are set to default values.
            try
            {
                IFormatter formatter = new System.Runtime.Serialization.Formatters.Binary.BinaryFormatter();
                Stream stream = new FileStream("settings.bin", FileMode.Open, FileAccess.Read);
                settings = (CameraSettings)formatter.Deserialize(stream);
                stream.Close();
            }
            catch (Exception)
            {
            }

            // Create the servers
            cameraServer = new CameraServer(settings);
            cameraServer.OnClientListChanged += new ClientListChangedHandler(UpdateListView);

            transferServer = new TransferServer();

            // Set the transfer server to point to the same vertices and colors lists to avoid copying large arrays in memory
            transferServer.Vertices = vertices;
            transferServer.Colors = colors;

            transferServer.DocumentInfo = cameraServer.DocumentInfo;

            InitializeComponent();

            // Start the servers
            transferServer.StartPointCloudServer();
            transferServer.StartDocumentServer();

            // Find the number of connected cameras
            IntPtr ctx = ob_create_context();
            IntPtr devList = ob_query_device_list(ctx);
            uint count = ob_device_list_device_count(devList).ToUInt32();

            cameraServer.LaunchClients(count);
        }

        private void CloseForm(object sender, FormClosingEventArgs e)
        {
            // Cache current settings to a file for next launch
            IFormatter formatter = new System.Runtime.Serialization.Formatters.Binary.BinaryFormatter();

            Stream stream = new FileStream("settings.bin", FileMode.Create, FileAccess.Write);
            formatter.Serialize(stream, settings);
            stream.Close();

            // Stop servers
            cameraServer.StopServer();
            transferServer.StopPointCloudServer();
            transferServer.StopDocumentServer();
        }

        private void OpenSettingsForm(object sender, EventArgs e)
        {
            // Create settings form to show if it is not already open
            if (cameraServer.GetSettingsForm() == null)
            {
                SettingsForm form = new SettingsForm();
                form.settings = settings;
                form.server = cameraServer;
                form.Show();
                cameraServer.SetSettingsForm(form);
            }
        }

        // Records sychronized frames from camera clients. Frames are saved once recording is finished.
        private void RecordFrames(object sender, DoWorkEventArgs e)
        {
            cameraServer.ClearRecordedFrames();

            int numRecordedFrames = 0;
            BackgroundWorker worker = (BackgroundWorker)sender;
            while (!worker.CancellationPending)
            {
                cameraServer.StartFrameRecording();

                numRecordedFrames++;
                SetStatusBarOnTimer("Recorded frame " + numRecordedFrames.ToString() + ".", 5000);
            }
        }

        // Called after a recording has been terminated to save recorded frames
        private void SaveFrames(object sender, RunWorkerCompletedEventArgs e)
        {
            isSaving = true;
            
            btRecord.Text = "Stop saving";
            btRecord.Enabled = true;

            savingWorker.RunWorkerAsync();
        }

        private void OpenLiveViewWindow(object sender, DoWorkEventArgs e)
        {
            isLiveViewRunning = true;
            openGLWindow = new OpenGLWindow();

            // Set openGLWindow variables to point to local arrays to avoid copying large arrays in memory
            lock (vertices)
            {
                openGLWindow.Vertices = vertices;
                openGLWindow.Colors = colors;
                openGLWindow.CameraPoses = cameraPoses;
                openGLWindow.Settings = settings;
            }

            openGLWindow.Run();
        }

        private void CloseLiveViewWindow(object sender, RunWorkerCompletedEventArgs e)
        {
            isLiveViewRunning = false;
            updateWorker.CancelAsync();
        }

        private void SaveFrames(object sender, DoWorkEventArgs e)
        {
            int numFrames = 0;

            string outDir = "out" + "\\" + txtSeqName.Text + "\\";
            DirectoryInfo di = Directory.CreateDirectory(outDir);

            BackgroundWorker worker = (BackgroundWorker)sender;

            // This loop runs until it is either cancelled (using the btRecord button), or until there are no more recorded frames
            while (!worker.CancellationPending)
            {
                bool success = cameraServer.TryGetRecordedFrame(ref cameraRecordedColors, ref cameraRecordedVertices);

                // This indicates that there are no more recorded frames
                if (!success)
                    break;

                numFrames++;
                int numVerticesTotal = 0;
                for (int i = 0; i < cameraRecordedColors.Count; i++)
                {
                    numVerticesTotal += cameraRecordedVertices[i].Count;
                }

                List<byte> frameColors = new List<byte>();
                List<Single> frameVertices = new List<Single>();

                SetStatusBarOnTimer("Saving frame " + (numFrames).ToString() + ".", 5000);

                for (int i = 0; i < cameraRecordedColors.Count; i++)
                {                                 
                    frameColors.AddRange(cameraRecordedColors[i]);
                    frameVertices.AddRange(cameraRecordedVertices[i]);

                    if (!settings.MergeScansForSave)
                    {
                        // Place frames from each client in separate files if requested
                        string outputFilename = outDir + "\\" + numFrames.ToString().PadLeft(5, '0') + i.ToString() + ".ply";
                        Utils.saveToPly(outputFilename, cameraRecordedVertices[i], cameraRecordedColors[i], settings.SaveAsBinaryPLY);                        
                    }
                }

                // Place frames from all clients in a single file
                if (settings.MergeScansForSave)
                {
                    string outputFilename = outDir + "\\" + numFrames.ToString().PadLeft(5, '0') + ".ply";
                    Utils.saveToPly(outputFilename, frameVertices, frameColors, settings.SaveAsBinaryPLY);
                }
            }
        }

        private void FinishSavingFrames(object sender, RunWorkerCompletedEventArgs e)
        {
            cameraServer.ClearRecordedFrames();
            isSaving = false;

            // If the live view window is open, restart the UpdateWorker
            if (isLiveViewRunning)
                RestartUpdateWorker();

            btRecord.Enabled = true;
            btRecord.Text = "Start recording";
            btRefineCalib.Enabled = true;
            btCalibrate.Enabled = true;
        }

        // Continually requests frames that will be displayed in the live view window
        private void UpdateLatestFrame(object sender, DoWorkEventArgs e)
        {
            BackgroundWorker worker = (BackgroundWorker)sender;
            while (!worker.CancellationPending)
            {
                Thread.Sleep(1);

                // Check that all connected cameras are initialized
                if (!cameraServer.GetAllDevicesInitialized())
                {
                    continue;
                }

                // Request latest frame from each camera
                lock (cameraVertices)
                {
                    cameraServer.GetLatestFrame(ref cameraColors, ref cameraVertices);
                }

                // Update the local lists representing the latest frame
                lock (vertices)
                {
                    vertices.Clear();
                    colors.Clear();
                    cameraPoses.Clear();

                    // Add vertices and colors from each camera to the encompassing list
                    for (int i = 0; i < cameraColors.Count; i++)
                    {
                        vertices.AddRange(cameraVertices[i]);
                        colors.AddRange(cameraColors[i]);
                    }

                    cameraPoses.AddRange(cameraServer.CameraPoses);
                }
                
                if (openGLWindow != null)
                {
                    // Note that a new frame was obtained (this is used to estimate the FPS)
                    openGLWindow.IncreaseFrameCounter();
                }           
            }
        }

        private void RestartUpdateWorker()
        {
            if (!updateWorker.IsBusy)
                updateWorker.RunWorkerAsync();
        }

        // Performs ICP-based pose refinement
        private void RefineCameraPoses(object sender, DoWorkEventArgs e)
        {
            // Check that all connected cameras are already calibrated
            if (cameraServer.AllCamerasCalibrated == false)
            {
                SetStatusBarOnTimer("Not all of the devices are calibrated.", 5000);
                return;
            }

            // Retrieve a frame from each connected camera
            lock (cameraVertices)
            {
                cameraServer.GetLatestFrame(ref cameraColors, ref cameraVertices);
            }

            // Initialize containers for the poses
            List<float[]> Rs = new List<float[]>();
            List<float[]> Ts = new List<float[]>();

            for (int i = 0; i < cameraVertices.Count; i++)
            {
                float[] tempR = new float[9];
                float[] tempT = new float[3];
                for (int j = 0; j < 3; j++)
                {
                    tempT[j] = 0;
                    tempR[j + j * 3] = 1;
                }

                Rs.Add(tempR);
                Ts.Add(tempT);
            }

            // Use ICP to refine the sensor poses (see referenced research article for more detail)
            for (int refineIter = 0; refineIter < settings.NumRefineIterations; refineIter++)
            {
                for (int i = 0; i < cameraVertices.Count; i++)
                {
                    List<float> otherFramesVertices = new List<float>();
                    for (int j = 0; j < cameraVertices.Count; j++)
                    {
                        if (j == i)
                            continue;
                        otherFramesVertices.AddRange(cameraVertices[j]);
                    }

                    float[] verts1 = otherFramesVertices.ToArray();
                    float[] verts2 = cameraVertices[i].ToArray();

                    IntPtr pVerts1 = Marshal.AllocHGlobal(otherFramesVertices.Count * sizeof(float));
                    IntPtr pVerts2 = Marshal.AllocHGlobal(cameraVertices[i].Count * sizeof(float));

                    Marshal.Copy(verts1, 0, pVerts1, verts1.Length);
                    Marshal.Copy(verts2, 0, pVerts2, verts2.Length);

                    ICP(pVerts1, pVerts2, otherFramesVertices.Count / 3, cameraVertices[i].Count / 3, Rs[i], Ts[i], settings.NumICPIterations);

                    Marshal.Copy(pVerts2, verts2, 0, verts2.Length);
                    cameraVertices[i].Clear();
                    cameraVertices[i].AddRange(verts2);
                }
            }

            // Update calibration data for all connected cameras
            List<AffineTransform> worldTransforms = cameraServer.WorldTransforms;
            List<AffineTransform> cameraPoses = cameraServer.CameraPoses;

            for (int i = 0; i < worldTransforms.Count; i++)
            {
                float[] tempT = new float[3];
                float[,] tempR = new float[3, 3];
                for (int j = 0; j < 3; j++)
                {
                    for (int k = 0; k < 3; k++)
                    {
                        tempT[j] += Ts[i][k] * worldTransforms[i].R[k, j];
                    }

                    worldTransforms[i].T[j] += tempT[j];
                    cameraPoses[i].T[j] += Ts[i][j];
                }

                for (int j = 0; j < 3; j++)
                {
                    for (int k = 0; k < 3; k++)
                    {
                        for (int l = 0; l < 3; l++)
                        {
                            tempR[j, k] += Rs[i][l * 3 + j] * worldTransforms[i].R[l, k];
                        }

                        worldTransforms[i].R[j, k] = tempR[j, k];
                        cameraPoses[i].R[j, k] = tempR[j, k];
                    }
                }
            }

            cameraServer.WorldTransforms = worldTransforms;
            cameraServer.CameraPoses = cameraPoses;

            cameraServer.SendCalibrationData();
        }

        private void FinishRefiningCameraPoses(object sender, RunWorkerCompletedEventArgs e)
        {
            // Re-enable all affected buttons after refinement is done
            btRefineCalib.Enabled = true;
            btCalibrate.Enabled = true;
            btRecord.Enabled = true;
        }

        // This is used for starting/stopping the recording worker and stopping the saving worker
        private void OnRecordButtonClick(object sender, EventArgs e)
        {
            if (cameraServer.ClientCount < 1)
            {
                SetStatusBarOnTimer("At least one client needs to be connected for recording.", 5000);                
                return;
            }

            // If we are saving frames, this button stops saving
            if (isSaving)
            {
                btRecord.Enabled = false;
                savingWorker.CancelAsync();
                return;
            }

            // Otherwise, start or stop the recording worker based on current state
            if (!isRecording)
            {
                // Stop the update worker to reduce the network usage (provides better synchronization).
                updateWorker.CancelAsync();

                // Start the recording worker
                recordingWorker.RunWorkerAsync();
                btRecord.Text = "Stop recording";
                btRefineCalib.Enabled = false;
                btCalibrate.Enabled = false;
            }
            else 
            {
                // Stop the recording worker
                btRecord.Enabled = false;
                recordingWorker.CancelAsync();                
            }

            isRecording = !isRecording;
        }

        private void OnCalibrateButtonClick(object sender, EventArgs e)
        {
            cameraServer.Calibrate();
        }

        private void OnRefineCalibrationButtonClick(object sender, EventArgs e)
        {
            // Check that at least two cameras are connected
            if (cameraServer.ClientCount < 2)
            {
                SetStatusBarOnTimer("Refining calibration requires at least 2 connected devices.", 5000);
                return;
            }

            btRefineCalib.Enabled = false;
            btCalibrate.Enabled = false;
            btRecord.Enabled = false;

            // Start the refine worker
            refineWorker.RunWorkerAsync();
        }

        private void OnShowLiveButtonClick(object sender, EventArgs e)
        {            
            RestartUpdateWorker();

            // Open the live view window if it is not open yet
            if (!OpenGLWorker.IsBusy)
                OpenGLWorker.RunWorkerAsync();
        }

        // Sets a new message on the status bar for a given time
        public void SetStatusBarOnTimer(string message, int milliseconds)
        {
            statusLabel.Text = message;

            statusBarTimer.Stop();
            statusBarTimer = new System.Timers.Timer();

            statusBarTimer.Interval = milliseconds;
            statusBarTimer.Elapsed += delegate(object sender, System.Timers.ElapsedEventArgs e)
            {
                statusBarTimer.Stop();
                statusLabel.Text = "";
            };

            statusBarTimer.Start();
        }

        // Updates the ListBox contaning the connected clients. Called by events in the CameraServer.
        private void UpdateListView(List<CameraClient> socketList)
        {
            List<string> listBoxItems = new List<string>();

            for (int i = 0; i < socketList.Count; i++)
                listBoxItems.Add(socketList[i].ClientState);


            lClientListBox.DataSource = listBoxItems;
        }
    }
}
