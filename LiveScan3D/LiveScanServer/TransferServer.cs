/***************************************************************************\

Module Name:  TransferServer.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module is the server used to listen for client connections through TCP
and to send them point cloud data at a high frequency

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using System.Threading;

namespace LiveScanServer
{
    public class TransferServer
    {
        public List<float> Vertices = new List<float>();
        public List<byte> Colors = new List<byte>();
        public DocumentInfo DocumentInfo = new DocumentInfo();

        private const int PointCloudPort = 48002;
        private const int DocumentPort = 48003;
        private const int CheckConnectionInterval = 1000;

        private TcpListener pointCloudListener;
        private System.Timers.Timer pointCloudConnectionTimer;
        private CancellationTokenSource pointCloudCancellationTokenSource;
        private List<PointCloudTransferSocket> pointCloudClients = new List<PointCloudTransferSocket>();
        private object pointCloudClientLock = new object();
        private bool isPointCloudServerRunning = false;

        private TcpListener documentListener;
        private System.Timers.Timer documentConnectionTimer;
        private CancellationTokenSource documentCancellationTokenSource;
        private List<DocumentTransferSocket> documentClients = new List<DocumentTransferSocket>();
        private object documentClientLock = new object();
        private bool isDocumentServerRunning = false;

        ~TransferServer()
        {
            StopPointCloudServer();
            StopDocumentServer();
        }

        /// <summary>
        /// Starts the TCP listener and Tasks for the point cloud server to listen for client connections and send them data
        /// </summary>
        public void StartPointCloudServer()
        {
            if (!isPointCloudServerRunning)
            {
                // Start TCP listener server
                pointCloudListener = new TcpListener(IPAddress.Any, PointCloudPort);
                pointCloudListener.Start();

                isPointCloudServerRunning = true;

                // Start tasks to listen for client connections and send data
                pointCloudCancellationTokenSource = new CancellationTokenSource();
                Task.Run(() => ConnectPointCloudClients(pointCloudCancellationTokenSource.Token));
                Task.Run(() => SendPointCloudToAllClients(pointCloudCancellationTokenSource.Token));

                // Start a timer to ping connected clients at a regular interval to ensure they are still connected
                pointCloudConnectionTimer = new System.Timers.Timer();
                pointCloudConnectionTimer.Interval = CheckConnectionInterval;

                pointCloudConnectionTimer.Elapsed += delegate (object sender, System.Timers.ElapsedEventArgs e)
                {
                    lock (pointCloudClientLock)
                    {
                        for (int i = 0; i < pointCloudClients.Count; i++)
                        {
                            if (!pointCloudClients[i].IsConnected())
                            {
                                pointCloudClients.RemoveAt(i);
                                i--;
                            }
                        }
                    }
                };

                pointCloudConnectionTimer.Start();
            }
        }

        /// <summary>
        /// Starts the TCP listener and Tasks for the document server to listen for client connections and send them data
        /// </summary>
        public void StartDocumentServer()
        {
            if (!isDocumentServerRunning)
            {
                // Start TCP listener server
                documentListener = new TcpListener(IPAddress.Any, DocumentPort);
                documentListener.Start();

                isDocumentServerRunning = true;

                // Start tasks to listen for client connections and send data
                documentCancellationTokenSource = new CancellationTokenSource();
                Task.Run(() => ConnectDocumentClients(documentCancellationTokenSource.Token));
                Task.Run(() => SendDocumentToAllClients(documentCancellationTokenSource.Token));

                // Start a timer to ping connected clients at a regular interval to ensure they are still connected
                documentConnectionTimer = new System.Timers.Timer();
                documentConnectionTimer.Interval = CheckConnectionInterval;

                documentConnectionTimer.Elapsed += delegate (object sender, System.Timers.ElapsedEventArgs e)
                {
                    lock (documentClientLock)
                    {
                        for (int i = 0; i < documentClients.Count; i++)
                        {
                            if (!documentClients[i].IsConnected())
                            {
                                documentClients.RemoveAt(i);
                                i--;
                            }
                        }
                    }
                };

                documentConnectionTimer.Start();
            }
        }

        /// <summary>
        /// Stops the point cloud server and all associated threads and connections
        /// </summary>
        public void StopPointCloudServer()
        {
            if (isPointCloudServerRunning)
            {
                isPointCloudServerRunning = false;

                // Stop checking client connections
                pointCloudConnectionTimer.Stop();

                // Explicitly stop Tasks
                pointCloudCancellationTokenSource.Cancel();

                // Stop each client socket and its threads
                foreach (PointCloudTransferSocket clientSocket in pointCloudClients)
                {
                    clientSocket.Stop();
                }

                // Stop the listener server
                pointCloudListener.Stop();

                lock (pointCloudClientLock)
                    pointCloudClients.Clear();
            }
        }

        /// <summary>
        /// Stops the document server and all associated threads and connections
        /// </summary>
        public void StopDocumentServer()
        {
            if (isDocumentServerRunning)
            {
                isDocumentServerRunning = false;

                // Stop checking client connections
                documentConnectionTimer.Stop();

                // Explicitly stop Tasks
                documentCancellationTokenSource.Cancel();

                // Stop each client socket and its threads
                foreach (DocumentTransferSocket clientSocket in documentClients)
                {
                    clientSocket.Stop();
                }

                // Stop the listener server
                documentListener.Stop();

                lock (documentClientLock)
                    documentClients.Clear();
            }
        }

        /// <summary>
        /// Listens for point cloud client connections in a loop
        /// </summary>
        /// <param name="token">Cancellation token to stop the task</param>
        /// <returns>Task representing the listener</returns>
        private async Task ConnectPointCloudClients(CancellationToken token)
        {
            while (isPointCloudServerRunning && !token.IsCancellationRequested)
            {
                try
                {
                    // Try to accept a new client
                    TcpClient newClient = pointCloudListener.AcceptTcpClient();

                    // Add the new client to the list
                    lock (pointCloudClientLock)
                    {
                        pointCloudClients.Add(new PointCloudTransferSocket(newClient));
                    }
                }
                catch (SocketException)
                {
                }

                await Task.Delay(100);
            }
        }

        /// <summary>
        /// Listens for document client connections in a loop
        /// </summary>
        /// <param name="token">Cancellation token to stop the task</param>
        /// <returns>Task representing the listener</returns>
        private async Task ConnectDocumentClients(CancellationToken token)
        {
            while (isDocumentServerRunning && !token.IsCancellationRequested)
            {
                try
                {
                    // Try to accept a new client
                    TcpClient newClient = documentListener.AcceptTcpClient();

                    // Add the new client to the list
                    lock (documentClientLock)
                    {
                        documentClients.Add(new DocumentTransferSocket(newClient));
                    }
                }
                catch (SocketException)
                {
                }

                await Task.Delay(100);
            }
        }

        /// <summary>
        /// Sends point cloud data to all connected clients at regular intervals
        /// </summary>
        /// <param name="token">Cancellation token to stop the Task</param>
        /// <returns>Task representing the sender</returns>
        private async Task SendPointCloudToAllClients(CancellationToken token)
        {
            while (isPointCloudServerRunning && !token.IsCancellationRequested)
            {
                // Send latest point cloud to all connected clients
                for (int i = 0; i < pointCloudClients.Count; i++)
                {
                    // Send a point cloud frame
                    lock (Vertices)
                    {
                        pointCloudClients[i].SendPointCloud(Vertices, Colors);
                    }
                }

                await Task.Delay(10);
            }
        }

        /// <summary>
        /// Sends document data to all connected clients at regular intervals
        /// </summary>
        /// <param name="token">Cancellation token to stop the Task</param>
        /// <returns>Task representing the sender</returns>
        private async Task SendDocumentToAllClients(CancellationToken token)
        {
            while (isDocumentServerRunning && !token.IsCancellationRequested)
            {
                if (DocumentInfo.IsNew)
                {
                    lock (DocumentInfo)
                    {
                        // Send latest document to all connected clients
                        for (int i = 0; i < documentClients.Count; i++)
                        {
                            documentClients[i].SendDocument(DocumentInfo.Data, DocumentInfo.Width, DocumentInfo.Height);
                        }
                    }

                    DocumentInfo.IsNew = false;
                }

                await Task.Delay(100);
            }
        }
    }
}
