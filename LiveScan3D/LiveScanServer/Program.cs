using System;
using System.Windows.Forms;
using System.Runtime.InteropServices;

namespace LiveScanServer
{
    static class Program
    {
        // Correct: place DllImport as a FIELD inside the class, not as an attribute
        [DllImport("kernel32.dll")]
        private static extern bool SetErrorMode(uint uMode);

        /// <summary>
        /// Main entry point for the application
        /// </summary>
        [STAThread]
        static void Main()
        {
            // Disable Windows debug output spam (Media Foundation, Orbbec, MF metadata, etc.)
            SetErrorMode(0x0001 | 0x0002 | 0x0004 | 0x8000);

            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new MainWindowForm());
        }
    }
}
