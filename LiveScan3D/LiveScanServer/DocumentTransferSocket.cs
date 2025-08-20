/***************************************************************************\

Module Name:  DocumentTransferSocket.cs
Project:      LiveScan3D
Authors:      Roxanne Archambault
Copyright (c) Canadian Space Agency.

<Description>
This module is the socket used to send document data to connected clients.

This code was adapted from the following research: 
Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 
3D Data Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 
2015 International Conference on, Lyon, France, 2015

\***************************************************************************/

using System;
using System.Collections.Generic;
using System.Drawing.Imaging;
using System.Drawing;
using System.IO;
using System.Net.Sockets;
using System.Linq;

namespace LiveScanServer
{
    public class DocumentTransferSocket : TransferSocketBase
    {
        public DocumentTransferSocket(TcpClient clientSocket) : base(clientSocket) { }

        public void SendDocument(List<byte> data, float width, float height)
        {
            // Receive 1 byte to check that the receiver has requested a new frame
            byte[] requestBuffer = Receive(1);

            while (requestBuffer.Length != 0)
            {
                if (requestBuffer[0] == 0)
                {
                    try
                    {
                        if (data == null || data.Count == 0 || width == 0 || height == 0)
                        {
                            return;
                        }

                        // Encode document data
                        byte[] dataArray = EncodeToJpeg(data.ToArray(), (int)width, (int)height);

                        if (dataArray == null || dataArray.Length == 0)
                        {
                            return;
                        }

                        // Send width and height of document first
                        WriteInt((int)width);
                        WriteInt((int)height);

                        // Write data size
                        WriteInt(dataArray.Length);

                        // Write actual data
                        socket.GetStream().Write(dataArray, 0, dataArray.Length);
                    }
                    catch (Exception)
                    {
                    }
                }

                // Receive a new request byte to make sure the receiver is ready to receive
                requestBuffer = Receive(1);
            }
        }

        public static byte[] EncodeToJpeg(byte[] rawBgr, int width, int height, int quality = 90)
        {
            if (rawBgr == null || rawBgr.Length != width * height * 3 || width <= 0 || height <= 0)
            {
                // Invalid input for JPEG encoding
                return null;
            }

            // Create Bitmap
            using (Bitmap bmp = new Bitmap(width, height, PixelFormat.Format24bppRgb))
            {
                var rect = new Rectangle(0, 0, width, height);
                var data = bmp.LockBits(rect, ImageLockMode.WriteOnly, bmp.PixelFormat);

                try
                {
                    // Copy raw pixels into Bitmap
                    int srcStride = width * 3;
                    int dstStride = Math.Abs(data.Stride);

                    unsafe
                    {
                        byte* dstRow = (byte*)data.Scan0;
                        fixed (byte* pSrc = rawBgr)
                        {
                            byte* srcRow = pSrc;
                            for (int y = 0; y < height; y++)
                            {
                                Buffer.MemoryCopy(srcRow, dstRow, dstStride, srcStride);
                                srcRow += srcStride;
                                dstRow += data.Stride; // includes padding
                            }
                        }
                    }
                }
                finally
                {
                    // Unlock Bitmap bits
                    bmp.UnlockBits(data);
                }

                // Save Bitmap as JPEG
                using (MemoryStream ms = new MemoryStream())
                {
                    var encoder = ImageCodecInfo.GetImageEncoders().First(e => e.FormatID == ImageFormat.Jpeg.Guid);
                    using (EncoderParameters eps = new EncoderParameters(1))
                    {
                        eps.Param[0] = new EncoderParameter(System.Drawing.Imaging.Encoder.Quality, (long)quality);
                        bmp.Save(ms, encoder, eps);
                    }

                    return ms.ToArray();
                }
            }
        }
    }
}