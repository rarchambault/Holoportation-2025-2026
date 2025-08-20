/***************************************************************************\

Module Name:  SettingsForm.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module is the UI form from which users can change settings to modify
camera data capturing and the point cloud reconstruction in general.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System;
using System.Windows.Forms;
using System.Globalization;

namespace LiveScanServer
{
    public partial class SettingsForm : Form
    {
        public CameraSettings settings;
        public CameraServer server;

        private bool isFormLoaded = false;

        private Timer scrollTimer = null;

        public SettingsForm()
        {
            InitializeComponent();
        }

        private void LoadSettingsForm(object sender, EventArgs e)
        {
            txtMinX.Text = settings.MinBounds[0].ToString(CultureInfo.InvariantCulture);
            txtMinY.Text = settings.MinBounds[1].ToString(CultureInfo.InvariantCulture);
            txtMinZ.Text = settings.MinBounds[2].ToString(CultureInfo.InvariantCulture);

            txtMaxX.Text = settings.MaxBounds[0].ToString(CultureInfo.InvariantCulture);
            txtMaxY.Text = settings.MaxBounds[1].ToString(CultureInfo.InvariantCulture);
            txtMaxZ.Text = settings.MaxBounds[2].ToString(CultureInfo.InvariantCulture);

            chFilter.Checked = settings.Filter;
            txtFilterNeighbors.Text = settings.NumFilterNeighbors.ToString();
            txtFilterDistance.Text = settings.FilterThreshold.ToString(CultureInfo.InvariantCulture);

            lisMarkers.DataSource = settings.MarkerPoses;

            chMerge.Checked = settings.MergeScansForSave;
            txtICPIters.Text = settings.NumICPIterations.ToString();
            txtRefinIters.Text = settings.NumRefineIterations.ToString();

            btSyncEnable.Enabled = true;
            btSyncDisable.Enabled = false;

            chAutoExposureEnabled.Checked = settings.IsAutoExposureEnabled;

            trManualExposure.Value = settings.ExposureStep;

            if (settings.SaveAsBinaryPLY)
            {
                rBinaryPly.Checked = true;
                rAsciiPly.Checked = false;
            }
            else
            {
                rBinaryPly.Checked = false;
                rAsciiPly.Checked = true;
            }

            isFormLoaded = true;
        }

        void UpdateClients()
        {
            if (isFormLoaded)
            {
                server.SendSettings();
            }
        }

        void UpdateMarkerFields()
        {
            if (lisMarkers.SelectedIndex >= 0)
            {
                MarkerPose pose = settings.MarkerPoses[lisMarkers.SelectedIndex];

                float X, Y, Z;
                pose.GetOrientation(out X, out Y, out Z);

                txtOrientationX.Text = X.ToString(CultureInfo.InvariantCulture);
                txtOrientationY.Text = Y.ToString(CultureInfo.InvariantCulture);
                txtOrientationZ.Text = Z.ToString(CultureInfo.InvariantCulture);

                txtTranslationX.Text = pose.Pose.T[0].ToString(CultureInfo.InvariantCulture);
                txtTranslationY.Text = pose.Pose.T[1].ToString(CultureInfo.InvariantCulture);
                txtTranslationZ.Text = pose.Pose.T[2].ToString(CultureInfo.InvariantCulture);

                txtId.Text = pose.Id.ToString(CultureInfo.InvariantCulture);
            }
            else
            {
                txtOrientationX.Text = "";
                txtOrientationY.Text = "";
                txtOrientationZ.Text = "";

                txtTranslationX.Text = "";
                txtTranslationY.Text = "";
                txtTranslationZ.Text = "";

                txtId.Text = "";
            }
        }

        private void txtMinX_TextChanged(object sender, EventArgs e)
        {
            Single.TryParse(txtMinX.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out settings.MinBounds[0]);
            UpdateClients();
        }

        private void txtMinY_TextChanged(object sender, EventArgs e)
        {
            Single.TryParse(txtMinY.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out settings.MinBounds[1]);
            UpdateClients();
        }

        private void txtMinZ_TextChanged(object sender, EventArgs e)
        {
            Single.TryParse(txtMinZ.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out settings.MinBounds[2]);
            UpdateClients();
        }

        private void txtMaxX_TextChanged(object sender, EventArgs e)
        {
            Single.TryParse(txtMaxX.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out settings.MaxBounds[0]);
            UpdateClients();
        }

        private void txtMaxY_TextChanged(object sender, EventArgs e)
        {
            Single.TryParse(txtMaxY.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out settings.MaxBounds[1]);
            UpdateClients();
        }

        private void txtMaxZ_TextChanged(object sender, EventArgs e)
        {
            Single.TryParse(txtMaxZ.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out settings.MaxBounds[2]);
            UpdateClients();
        }

        private void chFilter_CheckedChanged(object sender, EventArgs e)
        {
            settings.Filter = chFilter.Checked;
            UpdateClients();
        }

        private void txtFilterNeighbors_TextChanged(object sender, EventArgs e)
        {
            Int32.TryParse(txtFilterNeighbors.Text, out settings.NumFilterNeighbors);
            UpdateClients();
        }

        private void txtFilterDistance_TextChanged(object sender, EventArgs e)
        {
            Single.TryParse(txtFilterDistance.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out settings.FilterThreshold);
            UpdateClients();
        }

        private void txtICPIters_TextChanged(object sender, EventArgs e)
        {
            Int32.TryParse(txtICPIters.Text, out settings.NumICPIterations);
        }

        private void txtRefinIters_TextChanged(object sender, EventArgs e)
        {
            Int32.TryParse(txtRefinIters.Text, out settings.NumRefineIterations);
        }

        private void chMerge_CheckedChanged(object sender, EventArgs e)
        {
            settings.MergeScansForSave = chMerge.Checked;
        }

        private void btAdd_Click(object sender, EventArgs e)
        {
            lock (settings)
                settings.MarkerPoses.Add(new MarkerPose());
            lisMarkers.SelectedIndex = settings.MarkerPoses.Count - 1;
            UpdateMarkerFields();
            UpdateClients();
        }

        private void btRemove_Click(object sender, EventArgs e)
        {
            if (settings.MarkerPoses.Count > 0)
            {
                settings.MarkerPoses.RemoveAt(lisMarkers.SelectedIndex);
                lisMarkers.SelectedIndex = settings.MarkerPoses.Count - 1;
                UpdateMarkerFields();
                UpdateClients();
            }
        }

        private void lisMarkers_SelectedIndexChanged(object sender, EventArgs e)
        {
            UpdateMarkerFields();
        }

        private void txtOrientationX_TextChanged(object sender, EventArgs e)
        {
            if (lisMarkers.SelectedIndex >= 0)
            {
                MarkerPose pose = settings.MarkerPoses[lisMarkers.SelectedIndex];
                float X, Y, Z;
                pose.GetOrientation(out X, out Y, out Z);
                Single.TryParse(txtOrientationX.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out X);

                pose.SetOrientation(X, Y, Z);
                UpdateClients();
            }
        }

        private void txtOrientationY_TextChanged(object sender, EventArgs e)
        {
            if (lisMarkers.SelectedIndex >= 0)
            {
                MarkerPose pose = settings.MarkerPoses[lisMarkers.SelectedIndex];
                float X, Y, Z;
                pose.GetOrientation(out X, out Y, out Z);
                Single.TryParse(txtOrientationY.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out Y);

                pose.SetOrientation(X, Y, Z);
                UpdateClients();
            }
        }

        private void txtOrientationZ_TextChanged(object sender, EventArgs e)
        {
            if (lisMarkers.SelectedIndex >= 0)
            {
                MarkerPose pose = settings.MarkerPoses[lisMarkers.SelectedIndex];
                float X, Y, Z;
                pose.GetOrientation(out X, out Y, out Z);
                Single.TryParse(txtOrientationZ.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out Z);

                pose.SetOrientation(X, Y, Z);
                UpdateClients();
            }
        }

        private void txtTranslationX_TextChanged(object sender, EventArgs e)
        {
            if (lisMarkers.SelectedIndex >= 0)
            {
                float X;
                MarkerPose pose = settings.MarkerPoses[lisMarkers.SelectedIndex];
                Single.TryParse(txtTranslationX.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out X);

                pose.Pose.T[0] = X;
                UpdateClients();
            }
        }

        private void txtTranslationY_TextChanged(object sender, EventArgs e)
        {
            if (lisMarkers.SelectedIndex >= 0)
            {
                float Y;
                MarkerPose pose = settings.MarkerPoses[lisMarkers.SelectedIndex];
                Single.TryParse(txtTranslationY.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out Y);

                pose.Pose.T[1] = Y;
                UpdateClients();
            }
        }

        private void txtTranslationZ_TextChanged(object sender, EventArgs e)
        {
            if (lisMarkers.SelectedIndex >= 0)
            {
                float Z;
                MarkerPose pose = settings.MarkerPoses[lisMarkers.SelectedIndex];
                Single.TryParse(txtTranslationZ.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out Z);

                pose.Pose.T[2] = Z;
                UpdateClients();
            }
        }

        private void txtId_TextChanged(object sender, EventArgs e)
        {
            if (lisMarkers.SelectedIndex >= 0)
            {
                int id;
                MarkerPose pose = settings.MarkerPoses[lisMarkers.SelectedIndex];
                Int32.TryParse(txtId.Text, out id);

                pose.Id = id;
                UpdateClients();
            }
        }

        private void PlyFormat_CheckedChanged(object sender, EventArgs e)
        {
            if (rAsciiPly.Checked)
            {
                settings.SaveAsBinaryPLY = false;
            }
            else
            {
                settings.SaveAsBinaryPLY = true;
            }
        }

        private void btSyncEnable_click(object sender, EventArgs e)
        {
            if (server.GetAllDevicesInitialized() && server.ClientCount > 1)
            {
                server.EnableSync();
                btSyncEnable.Enabled = false;
                btSyncDisable.Enabled = true;

                // Disable the Auto Exposure, as this could interfere with the temporal sync
                chAutoExposureEnabled.Enabled = false;
                trManualExposure.Enabled = true;
                chAutoExposureEnabled.CheckState = CheckState.Unchecked;
            }

        }

        private void btSyncDisable_click(object sender, EventArgs e)
        {
            if (server.GetAllDevicesInitialized())
            {
                server.DisableSync();
                btSyncEnable.Enabled = true;
                btSyncDisable.Enabled = false;
                chAutoExposureEnabled.Enabled = true;
            }
        }

        private void chAutoExposureEnabled_CheckedChanged(object sender, EventArgs e)
        {
            settings.IsAutoExposureEnabled = chAutoExposureEnabled.Checked;
            UpdateClients();

            trManualExposure.Enabled = !settings.IsAutoExposureEnabled;
        }


        /// <summary>
        /// When the user scrolls on the trackbar, we wait a short amount of time to check if the user has scrolled again.
        /// This prevents the Manual Exposure to be set too often, and only sets it when the user has stopped scrolling.
        /// Code taken from: https://stackoverflow.com/a/15687418
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void trManualExposure_Scroll(object sender, EventArgs e)
        {
            if (scrollTimer == null)
            {
                // Tick every 300 ms
                scrollTimer = new Timer()
                {
                    Enabled = false,
                    Interval = 300,
                    Tag = (sender as TrackBar).Value
                };

                scrollTimer.Tick += (s, ea) =>
                {
                    // Check to see if the value has changed since last tick
                    if (trManualExposure.Value == (int)scrollTimer.Tag)
                    {
                        scrollTimer.Stop();

                        // Clamp Exposure Step between 1 and 300 to fit Orbbec values
                        int exposureStep = trManualExposure.Value;
                        int exposureStepClamped = exposureStep < 1 ? 1 : exposureStep > 300 ? 300 : exposureStep;
                        settings.ExposureStep = exposureStepClamped;
                        UpdateClients();

                        scrollTimer.Dispose();
                        scrollTimer = null;
                    }
                    else
                    {
                        // Record the last value seen
                        scrollTimer.Tag = trManualExposure.Value;
                    }
                };

                scrollTimer.Start();
            }
        }

        private void SettingsForm_FormClosed(object sender, FormClosedEventArgs e)
        {
            server.SetSettingsForm(null);
        }
    }
}