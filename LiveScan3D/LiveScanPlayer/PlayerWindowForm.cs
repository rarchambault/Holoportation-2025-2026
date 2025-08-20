/***************************************************************************\

Module Name:  PlayerWindowForm.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module is the logic behind the main UI form of the application. It
allows selecting files for playback and controlling the playback itself.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using LiveScanServer;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Windows.Forms;
using System.Threading;
using System.Diagnostics;

namespace LiveScanPlayer
{
    public partial class PlayerWindowForm : Form
    {
        private bool isPlayerRunning = false;

        private BindingList<IFrameFileReader> frameFiles = new BindingList<IFrameFileReader>();
        private List<float> vertices = new List<float>();
        private List<byte> colors = new List<byte>();

        private TransferServer transferServer = new TransferServer();
        private AutoResetEvent onPlayFramesFinished = new AutoResetEvent(false);

        public PlayerWindowForm()
        {
            InitializeComponent();
           
            transferServer.Vertices = vertices;
            transferServer.Colors = colors;

            lFrameFilesListView.Columns.Add("Current frame", 75);
            lFrameFilesListView.Columns.Add("Filename", 300);
        }

        private void btSelect_Click(object sender, EventArgs e)
        {
            OpenFileDialog dialog = new OpenFileDialog();
            dialog.Multiselect = true;
            dialog.ShowDialog();

            lock (frameFiles)
            {
                for (int i = 0; i < dialog.FileNames.Length; i++)
                {
                    frameFiles.Add(new FrameFileReaderBin(dialog.FileNames[i]));

                    var item = new ListViewItem(new[] { "0", dialog.FileNames[i] });
                    lFrameFilesListView.Items.Add(item);
                }
            }
        }

        private void btnSelectPly_Click(object sender, EventArgs e)
        {
            OpenFileDialog dialog = new OpenFileDialog();
            dialog.Multiselect = true;
            dialog.ShowDialog();

            if (dialog.FileNames.Length == 0)
                return;

            lock (frameFiles)
            {
                frameFiles.Add(new FrameFileReaderPly(dialog.FileNames));

                var item = new ListViewItem(new[] { "0", Path.GetDirectoryName(dialog.FileNames[0]) });
                lFrameFilesListView.Items.Add(item);
            }
        }

        private void btStart_Click(object sender, EventArgs e)
        {
            isPlayerRunning = !isPlayerRunning;

            if (isPlayerRunning)
            {
                transferServer.StartPointCloudServer();
                transferServer.StartDocumentServer();
                updateWorker.RunWorkerAsync();
                btStart.Text = "Stop player";
            }
            else
            {
                transferServer.StopPointCloudServer();
                transferServer.StopDocumentServer();
                btStart.Text = "Start player";
                onPlayFramesFinished.WaitOne();
            }
        }

        private void btRemove_Click(object sender, EventArgs e)
        {
            if (lFrameFilesListView.SelectedIndices.Count == 0)
                return;

            lock (frameFiles)
            {
                int idx = lFrameFilesListView.SelectedIndices[0];
                lFrameFilesListView.Items.RemoveAt(idx);
                frameFiles.RemoveAt(idx);
            }
        }

        private void btRewind_Click(object sender, EventArgs e)
        {
            lock (frameFiles)
            {
                for (int i = 0; i < frameFiles.Count; i++)
                {
                    frameFiles[i].Rewind();
                    lFrameFilesListView.Items[i].Text = "0";
                }
            }
        }

        private void btShow_Click(object sender, EventArgs e)
        {
            if (!OpenGLWorker.IsBusy)
                OpenGLWorker.RunWorkerAsync();
        }

        private void lFrameFilesListView_DoubleClick(object sender, EventArgs e)
        {
            lFrameFilesListView.SelectedItems[0].BeginEdit();
        }

        private void lFrameFilesListView_AfterLabelEdit(object sender, LabelEditEventArgs e)
        {
            int fileIdx = lFrameFilesListView.SelectedIndices[0];
            int frameIdx;
            bool res = Int32.TryParse(e.Label, out frameIdx);

            if (!res)
            {
                e.CancelEdit = true;
                return;
            }

            lock (frameFiles)
            {
                frameFiles[fileIdx].JumpToFrame(frameIdx);
            }

        }

        private void CloseForm(object sender, FormClosingEventArgs e)
        {
            isPlayerRunning = false;
            transferServer.StopPointCloudServer();
            transferServer.StopDocumentServer();
        }

        private void PlayFrames(object sender, DoWorkEventArgs e)
        {
            int curFrameIdx = 0;
            string outDir = "outPlayer\\";
            DirectoryInfo di = Directory.CreateDirectory(outDir);

            while (isPlayerRunning)
            {
                Thread.Sleep(50);

                // Read frame into local variables
                List<float> tempVertices = new List<float>();
                List<byte> tempColors = new List<byte>();

                lock (frameFiles)
                {
                    for (int i = 0; i < frameFiles.Count; i++)
                    {
                        List<float> vertices = new List<float>();
                        List<byte> colors = new List<byte>();
                        frameFiles[i].ReadFrame(vertices, colors);

                        tempVertices.AddRange(vertices);                        
                        tempColors.AddRange(colors);
                    }
                }

                // Update frame indices in the UI
                Thread frameIdxUpdate = new Thread(() => this.Invoke((MethodInvoker)delegate { this.UpdateDisplayedFrameIndices(); }));
                frameIdxUpdate.Start();

                // Add the read frame to the global variables
                lock (vertices)
                {
                    vertices.Clear();
                    colors.Clear();
                    vertices.AddRange(tempVertices);
                    colors.AddRange(tempColors);
                }

                // Save the frame if requested
                if (chSaveFrames.Checked)
                    SaveCurrentFrameToFile(outDir, curFrameIdx);
                
                curFrameIdx++;
            }

            onPlayFramesFinished.Set();
        }

        private void OpenLiveViewWindowd(object sender, DoWorkEventArgs e)
        {
            OpenGLWindow openGLWindow = new OpenGLWindow();

            openGLWindow.Vertices = vertices;
            openGLWindow.Colors = colors;

            openGLWindow.Run();
        }

        private void UpdateDisplayedFrameIndices()
        {
            lock (frameFiles)
            {
                for (int i = 0; i < frameFiles.Count; i++)
                {
                    lFrameFilesListView.Items[i].SubItems[0].Text = frameFiles[i].FrameIdx.ToString();
                }
            }
        }

        private void SaveCurrentFrameToFile(string outDir, int frameIdx)
        {
            List<float> lVertices = new List<float>();
            List<byte> lColors = new List<byte>();

            lock (vertices)
            {
                lVertices.AddRange(vertices);
                lColors.AddRange(colors);
            }
            string outputFilename = outDir + frameIdx.ToString().PadLeft(5, '0') + ".ply";
            Utils.saveToPly(outputFilename, lVertices, lColors, true);
        }
    }
}
