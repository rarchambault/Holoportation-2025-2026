/***************************************************************************\

Module Name:  OpenGLWindow.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module is used to render the current point cloud reconstruction as well
as the marker and camera poses.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using OpenTK;
using OpenTK.Graphics;
using OpenTK.Input;
using System;
using System.Collections.Generic;

namespace LiveScanServer
{
    public class OpenGLWindow : GameWindow
    {
        public List<float> Vertices = new List<float>();
        public List<byte> Colors = new List<byte>();
        public List<AffineTransform> CameraPoses = new List<AffineTransform>();
        public CameraSettings Settings = new CameraSettings();

        private static float s_mouseOrbitSpeed = 0.30f;    // 0 = SLOWEST, 1 = FASTEST
        private static float s_mouseDollySpeed = 0.2f;     // Same as above but much more sensitive
        private static float s_mouseTrackSpeed = 0.003f;   // Same as above but much more sensitive
        private static float s_keyboardMoveSpeed = 0.01f;

        private const int InitialWindowWidth = 800;
        private const int InitialWindowHeight = 600;
        private const string WindowTitle = "LiveScan3D";

        private Vector2 previousMousePosition = new Vector2();
        private Vector2 currentMousePosition = new Vector2();
        private float[] cameraPosition = new float[3];
        private CameraMode cameraMode = CameraMode.None;

        private float headingAngle;
        private float pitchAngle;
        private float dx = 0.0f;
        private float dy = 0.0f;

        private bool isFullScreen = false;
        private byte brightnessModifier = 0;

        private bool drawMarkings = true;
        private uint vboHandle;
        private int pointCount;
        private int lineCount;
        private float pointSize = 0.0f;

        private VertexColorPosition[] vertexColorPositions;

        private DateTime lastFrameTime = DateTime.Now;
        private int frameCounter = 0;

        /// <summary>
        /// Creates a window with the specified title
        /// </summary>
        public OpenGLWindow()
            : base(InitialWindowWidth, InitialWindowHeight, GraphicsMode.Default, WindowTitle)
        {
            this.VSync = VSyncMode.Off;
            MouseUp += new EventHandler<MouseButtonEventArgs>(OnMouseButtonUp);
            MouseDown += new EventHandler<MouseButtonEventArgs>(OnMouseButtonDown);
            MouseMove += new EventHandler<MouseMoveEventArgs>(OnMouseMove);
            MouseWheel += new EventHandler<MouseWheelEventArgs>(OnMouseWheelChanged);

            KeyDown += new EventHandler<KeyboardKeyEventArgs>(OnKeyDown);
            
            cameraPosition[0] = 0;
            cameraPosition[1] = 0;
            cameraPosition[2] = 1.0f;
        }

        private enum CameraMode
        {
            None,
            Track,
            Dolly,
            Orbit
        }

        public void IncreaseFrameCounter()
        {
            frameCounter++;
        }

        private void ToggleFullscreen()
        {
            if (isFullScreen)
            {
                WindowBorder = WindowBorder.Resizable;
                WindowState = WindowState.Normal;
                ClientSize = new System.Drawing.Size(800, 600);
                CursorVisible = true;
            }
            else
            {
                CursorVisible = false;
                WindowBorder = WindowBorder.Hidden;
                WindowState = WindowState.Fullscreen;
            }

            isFullScreen = !isFullScreen;
        }

        private void OnKeyDown(object sender, KeyboardKeyEventArgs e)
        {
            var keyboard = e.Keyboard;

            // Exit window on Escape press
            if (keyboard[Key.Escape])
            {
                Exit();
            }

            // Increase point size on Plus press
            if (keyboard[Key.Plus])
            {
                pointSize += 0.1f;
                GL.PointSize(pointSize);
            }

            // Decrease point size on Minus press
            if (keyboard[Key.Minus])
            {
                if (pointSize != 0)
                    pointSize -= 0.1f;
                GL.PointSize(pointSize);
            }

            // Move camera position on WASD press
            if (keyboard[Key.W])
                cameraPosition[2] -= s_keyboardMoveSpeed;

            if (keyboard[Key.A])
                cameraPosition[0] -= s_keyboardMoveSpeed;

            if (keyboard[Key.S])
                cameraPosition[2] += s_keyboardMoveSpeed;

            if (keyboard[Key.D])
                cameraPosition[0] += s_keyboardMoveSpeed;

            // Toggle full screen mode on F press
            if (keyboard[Key.F])
                ToggleFullscreen();

            // Toggle marking drawings on M press
            if (keyboard[Key.M])
                drawMarkings = !drawMarkings;

            // Decrease brightness on O press
            if (keyboard[Key.O])
                brightnessModifier = (byte)Math.Max(0, brightnessModifier - 10);

            // Increase brightness on P press
            if (keyboard[Key.P])
                brightnessModifier = (byte)Math.Min(255, brightnessModifier + 10);
        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);

            // Check OpenGL version
            Version version = new Version(GL.GetString(StringName.Version).Substring(0, 3));
            Version target = new Version(1, 5);
            if (version < target)
            {
                throw new NotSupportedException(String.Format(
                    "OpenGL {0} is required (you only have {1}).", target, version));
            }

            GL.ClearColor(.1f, 0f, .1f, 0f);
            GL.Enable(EnableCap.DepthTest);

            // Setup parameters for Points
            GL.PointSize(pointSize);
            GL.Enable(EnableCap.PointSmooth);
            GL.Hint(HintTarget.PointSmoothHint, HintMode.Nicest);

            // Setup VBO state
            GL.EnableClientState(EnableCap.ColorArray);
            GL.EnableClientState(EnableCap.VertexArray);
            
            GL.GenBuffers(1, out vboHandle);

            // Setup VBO since there is only one in the window
            GL.BindBuffer(BufferTarget.ArrayBuffer, vboHandle);
            GL.ColorPointer(4, ColorPointerType.UnsignedByte, VertexColorPosition.s_sizeInBytes, (IntPtr)0);
            GL.VertexPointer(3, VertexPointerType.Float, VertexColorPosition.s_sizeInBytes, (IntPtr)(4 * sizeof(byte)));

            pointCount = 0;
            lineCount = 12;
            vertexColorPositions = new VertexColorPosition[pointCount + 2 * lineCount];
        }

        protected override void OnUnload(EventArgs e)
        {
            GL.DeleteBuffers(1, ref vboHandle);
        }

        protected override void OnResize(EventArgs e)
        {
            GL.Viewport(0, 0, Width, Height);

            GL.MatrixMode(MatrixMode.Projection);
            Matrix4 p = Matrix4.CreatePerspectiveFieldOfView(MathHelper.PiOver4, Width / (float)Height, 0.1f, 50.0f);
            GL.LoadMatrix(ref p);

            GL.MatrixMode(MatrixMode.Modelview);
            Matrix4 mv = Matrix4.LookAt(Vector3.UnitZ, Vector3.Zero, Vector3.UnitY);
            GL.LoadMatrix(ref mv);
        }

        void OnMouseWheelChanged(object sender, MouseWheelEventArgs e)
        {
            dy = e.Delta * s_mouseDollySpeed;

            cameraPosition[2] -= dy;
        }

        void OnMouseMove(object sender, MouseMoveEventArgs e)
        {
            // Retrieve new mouse position
            currentMousePosition.X = e.Mouse.X;
            currentMousePosition.Y = e.Mouse.Y;

            // Use difference with the previous position to move the camera
            switch (cameraMode)
            {
                case CameraMode.Track:
                    dx = currentMousePosition.X - previousMousePosition.X;
                    dx *= s_mouseTrackSpeed;

                    dy = currentMousePosition.Y - previousMousePosition.Y;
                    dy *= s_mouseTrackSpeed;

                    cameraPosition[0] -= dx;
                    cameraPosition[1] += dy;

                    break;

                case CameraMode.Dolly:
                    dy = currentMousePosition.Y - previousMousePosition.Y;
                    dy *= s_mouseDollySpeed;

                    cameraPosition[2] -= dy;

                    break;

                case CameraMode.Orbit:
                    dx = currentMousePosition.X - previousMousePosition.X;
                    dx *= s_mouseOrbitSpeed;

                    dy = currentMousePosition.Y - previousMousePosition.Y;
                    dy *= s_mouseOrbitSpeed;

                    headingAngle += dx;
                    pitchAngle += dy;

                    break;
            }

            previousMousePosition.X = currentMousePosition.X;
            previousMousePosition.Y = currentMousePosition.Y;
        }

        void OnMouseButtonUp(object sender, MouseButtonEventArgs e)
        {
            cameraMode = CameraMode.None;
        }

        void OnMouseButtonDown(object sender, MouseButtonEventArgs e)
        {
            switch (e.Button)
            {
                case MouseButton.Left:
                    cameraMode = CameraMode.Orbit;
                    break;
                case MouseButton.Middle:
                    cameraMode = CameraMode.Dolly;
                    break;
                case MouseButton.Right:
                    cameraMode = CameraMode.Track;
                    break;
            }

            previousMousePosition.X = Mouse.X;
            previousMousePosition.Y = Mouse.Y;
        }

        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            // Update FPS text
            if ((DateTime.Now - lastFrameTime).Seconds >= 1)
            {
                double FPS = frameCounter / (DateTime.Now - lastFrameTime).TotalSeconds;
                this.Title = "FPS: " + string.Format("{0:F}", FPS);

                lastFrameTime = DateTime.Now;
                frameCounter = 0;
            }

            lock (Vertices)
            {
                lock (Settings)
                {
                    pointCount = Vertices.Count / 3;
                    lineCount = 0;

                    if (drawMarkings)
                    {
                        // Bounding box
                        lineCount += 12;

                        // Markers
                        lineCount += Settings.MarkerPoses.Count * 3;

                        // Cameras
                        lineCount += CameraPoses.Count * 3;
                    }

                    vertexColorPositions = new VertexColorPosition[pointCount + 2 * lineCount];

                    for (int i = 0; i < pointCount; i++)
                    {
                        vertexColorPositions[i].R = (byte)Math.Max(0, Math.Min(255, (Colors[i * 3] + brightnessModifier)));
                        vertexColorPositions[i].G = (byte)Math.Max(0, Math.Min(255, (Colors[i * 3 + 1] + brightnessModifier)));
                        vertexColorPositions[i].B = (byte)Math.Max(0, Math.Min(255, (Colors[i * 3 + 2] + brightnessModifier)));
                        vertexColorPositions[i].A = 255;
                        vertexColorPositions[i].Position.X = Vertices[i * 3];
                        vertexColorPositions[i].Position.Y = Vertices[i * 3 + 1];
                        vertexColorPositions[i].Position.Z = Vertices[i * 3 + 2];
                    }

                    if (drawMarkings)
                    {
                        int iCurLineCount = 0;
                        iCurLineCount += AddBoundingBox(pointCount + 2 * iCurLineCount);
                        for (int i = 0; i < Settings.MarkerPoses.Count; i++)
                        {
                            iCurLineCount += AddMarker(pointCount + 2 * iCurLineCount, Settings.MarkerPoses[i].Pose);
                        }
                        for (int i = 0; i < CameraPoses.Count; i++)
                        {
                            iCurLineCount += AddCamera(pointCount + 2 * iCurLineCount, CameraPoses[i]);
                        }
                    }
                }
            }
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {        
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            GL.PushMatrix();

            GL.MatrixMode(MatrixMode.Modelview);
            GL.Translate(-cameraPosition[0], -cameraPosition[1], -cameraPosition[2]);
            GL.Rotate(pitchAngle, 1.0f, 0.0f, 0.0f);
            GL.Rotate(headingAngle, 0.0f, 1.0f, 0.0f);

            // Tell OpenGL to discard old VBO when done drawing it and reserve memory _now_ for a new buffer.
            // Without this, GL would wait until draw operations on old VBO are complete before writing to it
            GL.BufferData(BufferTarget.ArrayBuffer, (IntPtr)(VertexColorPosition.s_sizeInBytes * (pointCount + 2 * lineCount)), IntPtr.Zero, BufferUsageHint.StreamDraw);
            
            // Fill newly allocated buffer
            GL.BufferData(BufferTarget.ArrayBuffer, (IntPtr)(VertexColorPosition.s_sizeInBytes * (pointCount + 2 * lineCount)), vertexColorPositions, BufferUsageHint.StreamDraw);

            GL.DrawArrays(BeginMode.Points, 0, pointCount);
            GL.DrawArrays(BeginMode.Lines, pointCount, 2 * lineCount);

            GL.PopMatrix();

            SwapBuffers();
        }

        private int AddBoundingBox(int startIdx)
        {
            int nLinesBeingAdded = 12;

            // 2 points per line
            int nPointsToAdd = 2 * nLinesBeingAdded;

            for (int i = startIdx; i < startIdx + nPointsToAdd; i++)
            {
                vertexColorPositions[i].R = 255;
                vertexColorPositions[i].G = 255;
                vertexColorPositions[i].B = 0;
                vertexColorPositions[i].A = 0;
            }

            int n = 0;

            // Bottom vertices
            // First vertex
            AddLine(startIdx + n, Settings.MinBounds[0], Settings.MinBounds[1], Settings.MinBounds[2],
                Settings.MaxBounds[0], Settings.MinBounds[1], Settings.MinBounds[2]);
            n += 2;
            AddLine(startIdx + n, Settings.MinBounds[0], Settings.MinBounds[1], Settings.MinBounds[2],
                Settings.MinBounds[0], Settings.MaxBounds[1], Settings.MinBounds[2]);
            n += 2;
            AddLine(startIdx + n, Settings.MinBounds[0], Settings.MinBounds[1], Settings.MinBounds[2],
                Settings.MinBounds[0], Settings.MinBounds[1], Settings.MaxBounds[2]);
            n += 2;

            // Second vertex
            AddLine(startIdx + n, Settings.MaxBounds[0], Settings.MinBounds[1], Settings.MinBounds[2],
                Settings.MaxBounds[0], Settings.MaxBounds[1], Settings.MinBounds[2]);
            n += 2;
            AddLine(startIdx + n, Settings.MaxBounds[0], Settings.MinBounds[1], Settings.MinBounds[2],
                Settings.MaxBounds[0], Settings.MinBounds[1], Settings.MaxBounds[2]);
            n += 2;

            // Third vertex
            AddLine(startIdx + n, Settings.MaxBounds[0], Settings.MinBounds[1], Settings.MaxBounds[2],
                Settings.MaxBounds[0], Settings.MaxBounds[1], Settings.MaxBounds[2]);
            n += 2;
            AddLine(startIdx + n, Settings.MaxBounds[0], Settings.MinBounds[1], Settings.MaxBounds[2],
                Settings.MinBounds[0], Settings.MinBounds[1], Settings.MaxBounds[2]);
            n += 2;

            // Fourth vertex
            AddLine(startIdx + n, Settings.MinBounds[0], Settings.MinBounds[1], Settings.MaxBounds[2],
                Settings.MinBounds[0], Settings.MaxBounds[1], Settings.MaxBounds[2]);
            n += 2;

            // Top vertices
            // Fifth vertex 
            AddLine(startIdx + n, Settings.MinBounds[0], Settings.MaxBounds[1], Settings.MinBounds[2],
                Settings.MaxBounds[0], Settings.MaxBounds[1], Settings.MinBounds[2]);
            n += 2;
            AddLine(startIdx + n, Settings.MinBounds[0], Settings.MaxBounds[1], Settings.MinBounds[2],
                Settings.MinBounds[0], Settings.MaxBounds[1], Settings.MaxBounds[2]);
            n += 2;

            // Sixth vertex
            AddLine(startIdx + n, Settings.MaxBounds[0], Settings.MaxBounds[1], Settings.MaxBounds[2],
                Settings.MaxBounds[0], Settings.MaxBounds[1], Settings.MinBounds[2]);
            n += 2;
            AddLine(startIdx + n, Settings.MaxBounds[0], Settings.MaxBounds[1], Settings.MaxBounds[2],
                Settings.MinBounds[0], Settings.MaxBounds[1], Settings.MaxBounds[2]);
            n += 2;

            return nLinesBeingAdded;
        }

        private int AddMarker(int startIdx, AffineTransform pose)
        {
            int nLinesBeingAdded = 3;

            // 2 points per line
            int nPointsToAdd = 2 * nLinesBeingAdded;

            for (int i = startIdx; i < startIdx + nPointsToAdd; i++)
            {
                vertexColorPositions[i].R = 255;
                vertexColorPositions[i].G = 0;
                vertexColorPositions[i].B = 0;
                vertexColorPositions[i].A = 0;
            }

            int n = 0;

            float x0 = pose.T[0];
            float y0 = pose.T[1];
            float z0 = pose.T[2];

            float x1 = 0.1f;
            float y1 = 0.1f;
            float z1 = 0.1f;

            float x2 = pose.R[0, 0] * x1;
            float y2 = pose.R[1, 0] * x1;
            float z2 = pose.R[2, 0] * x1;

            x2 += pose.T[0];
            y2 += pose.T[1];
            z2 += pose.T[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            x2 = pose.R[0, 1] * y1;
            y2 = pose.R[1, 1] * y1;
            z2 = pose.R[2, 1] * y1;

            x2 += pose.T[0];
            y2 += pose.T[1];
            z2 += pose.T[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            x2 = pose.R[0, 2] * z1;
            y2 = pose.R[1, 2] * z1;
            z2 = pose.R[2, 2] * z1;

            x2 += pose.T[0];
            y2 += pose.T[1];
            z2 += pose.T[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            return nLinesBeingAdded;
        }

        private int AddCamera(int startIdx, AffineTransform pose)
        {
            int nLinesBeingAdded = 3;

            // 2 points per line
            int nPointsToAdd = 2 * nLinesBeingAdded;

            for (int i = startIdx; i < startIdx + nPointsToAdd; i++)
            {
                vertexColorPositions[i].R = 0;
                vertexColorPositions[i].G = 255;
                vertexColorPositions[i].B = 0;
                vertexColorPositions[i].A = 0;
            }

            int n = 0;

            float x0 = pose.T[0];
            float y0 = pose.T[1];
            float z0 = pose.T[2];

            float x1 = 0.1f;
            float y1 = 0.1f;
            float z1 = 0.1f;

            float x2 = pose.R[0, 0] * x1;
            float y2 = pose.R[1, 0] * x1;
            float z2 = pose.R[2, 0] * x1;

            x2 += pose.T[0];
            y2 += pose.T[1];
            z2 += pose.T[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            x2 = pose.R[0, 1] * y1;
            y2 = pose.R[1, 1] * y1;
            z2 = pose.R[2, 1] * y1;

            x2 += pose.T[0];
            y2 += pose.T[1];
            z2 += pose.T[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            x2 = pose.R[0, 2] * z1;
            y2 = pose.R[1, 2] * z1;
            z2 = pose.R[2, 2] * z1;

            x2 += pose.T[0];
            y2 += pose.T[1];
            z2 += pose.T[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            return nLinesBeingAdded;
        }

        private void AddLine(int startIdx, float x0, float y0, float z0, 
            float x1, float y1, float z1)
        {
            vertexColorPositions[startIdx].Position.X = x0;
            vertexColorPositions[startIdx].Position.Y = y0;
            vertexColorPositions[startIdx].Position.Z = z0;

            vertexColorPositions[startIdx + 1].Position.X = x1;
            vertexColorPositions[startIdx + 1].Position.Y = y1;
            vertexColorPositions[startIdx + 1].Position.Z = z1;
        }

        // Used for drawing
        struct VertexColorPosition
        {
            public static int s_sizeInBytes = 16;

            public byte R, G, B, A;
            public Vector3 Position;
        }
    }
}

