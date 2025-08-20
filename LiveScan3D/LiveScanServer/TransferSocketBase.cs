/***************************************************************************\

Module Name:  TransferSocketBase.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module contains some base logic for modules which use a TCP socket to
send data to connected clients.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System;
using System.Net.Sockets;

namespace LiveScanServer
{
    public class TransferSocketBase
    {
        protected TcpClient socket;

        public TransferSocketBase(TcpClient clientSocket)
        {
            socket = clientSocket;
        }

        public void Stop()
        {
            if (IsConnected())
            {
                socket.Close();
            }
        }

        /// <summary>
        /// Checks if the client connection is still established. This is used to ping the clients at regular
        /// intervals to ensure they are still connected.
        /// </summary>
        /// <returns>True is the client connection is still valid, false otherwise.</returns>
        public bool IsConnected()
        {
            return socket.Connected;
        }

        protected byte[] Receive(int nBytes)
        {
            byte[] buffer;
            if (socket.Available != 0)
            {
                buffer = new byte[Math.Min(nBytes, socket.Available)];
                socket.GetStream().Read(buffer, 0, nBytes);
            }
            else
                buffer = new byte[0];

            return buffer;
        }

        protected void WriteInt(int val)
        {
            socket.GetStream().Write(BitConverter.GetBytes(val), 0, 4);
        }
    }
}
