/***************************************************************************\

Module Name:  Logger.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module is a helper for debugging which can be called from anywhere in 
LiveScanServer to print to a log file in a fixed location.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System;
using System.IO;

namespace LiveScanServer
{
    public static class Logger
    {
        private static readonly string s_logFilePath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Log/LiveScanServer_Log.txt");
        private static readonly object s_lockObj = new object();

        public static void Log(string message)
        {
            try
            {
                lock (s_lockObj)
                {
                    File.AppendAllText(s_logFilePath, $"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff} - {message}{Environment.NewLine}");
                }
            }
            catch
            {
            }
        }
    }
}
