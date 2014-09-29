// Copyright (c) 2014, Freescale Semiconductor, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Freescale Semiconductor, Inc. nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL FREESCALE SEMICONDUCTOR, INC. BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;
using System.Threading;
using System.Drawing.Drawing2D;

namespace SensorFusion
{
    public partial class MainForm : Form
    {
        // general constants
        private const int X = 0;
        private const int Y = 1;
        private const int Z = 2;
        private const double DEGTORAD = 0.01745329251994;
        private const double RADTODEG = 57.2957795130823;

        // serial port constants
        // 124 bytes (total for all packets) at 25Hz = 3050 bytes/sec
        private const int SERIAL_PORT_MAX_ATTEMPTS = 3;
        private const int SERIAL_PORT_RX_BUFFER_SIZE = 256;
        private const int SERIAL_PORT_RX_CALLBACK_THRESHOLD = 100;

        // Kinetis coordinate system received
        private const int NEDCOORD = 0x00;
        private const int ANDROIDCOORD = 0x10;
        private const int WIN8COORD = 0x20;
        private static int Coordinates = ANDROIDCOORD;  // default to Android

        // Kinetis quaternion type received
        private const int Q3 = 0x01;
        private const int Q3M = 0x06;
        private const int Q3G = 0x03;
        private const int Q6MA = 0x02;
        private const int Q6AG = 0x04;
        private const int Q9 = 0x08;
        private static int QReceived = 0x00;   // default to invalid type
        private static int SyncQuaternionCountdown;

        // PCB constants
        private const int WIN8PCB = 0x00;
        private const int KL25ZPCB = 0x01;
        private const int K20PCB = 0x02;
        private const int KL26ZPCB = 0x04;
        private const int K64FPCB = 0x05;
        private const int KL16ZPCB = 0x06;
        private const int KL46ZPCB = 0x07;
        private const int KL46ZSTANDALONEPCB = 0x08;
        private static int Hardware = KL25ZPCB; // default to KL25Z board

        // global bluetooth packet decoding
        private static byte[] bluetoothInputBuffer = new byte[SERIAL_PORT_RX_BUFFER_SIZE];
        private static bool bluetoothInsidePacket;
        private static int bluetoothByteCount;
        private static int bluetoothPacketErrors;
        private static bool bluetoothDangling0x7D;
        private static bool bluetooth0x7ESyncDone;

        // serial port
        private static byte[] serialPortBuffer = new byte[SERIAL_PORT_RX_BUFFER_SIZE];
        private string[] comPortNames;

        // file I/O
        StreamWriter logFileStreamWriter;
        StreamWriter magBufferStreamWriter;

        // main packet data received
        private static Quaternion.fquaternion qRx;
        private static Quaternion.fquaternion qDisplay;
        private static double OnTime;
        private static double[] Acc = new double[3];
        private static double[] Mag = new double[3];
        private static double[] Gyro = new double[3];
        private static double[] AngVel = new double[3];

        // debug packet data received
        private static double Firmware;
        private static double ksystick;

        // magnetic packet data received
        private static int MagneticVariableID;
        private static double Delta;
        private static int BufferCount;
        private static double FitErrorpc;
        private static double B;
        private static double[] V = new double[3];
        private static double[,] invW = new double[3, 3];

        // magnetic buffer data received
        private const int MAGBUFFSIZE = 288;
        private static int[] MagBuffIndex = new int[MAGBUFFSIZE];
        private static double[,] MagBuff = new double[3, MAGBUFFSIZE];
        private static double[] MagBuffXData = new double[MAGBUFFSIZE];
        private static double[] MagBuffYData = new double[MAGBUFFSIZE];
        private static double[] MagBuffZData = new double[MAGBUFFSIZE];

        // roll, pitch, compass packet data received
        private static double Roll, Pitch, Compass;

        // altimeter packet data received
        private static double Altitude, Temperature;

        // kalman filter packet data received
        private static double[] ThErrPl = new double[3];
        private static double[] bPl = new double[3];
        private static double[] bErrPl = new double[3];

        // physical linear acceleration
        private static double[] Acceleration = new double[3];

        //  flags and timers
        private static bool LEDPacketFlag;
        private static bool LEDRecFlag;
        private static bool AltitudePacketsReceived;

        // Kinetis PCB graphics
        private const double PCBSCALING = 0.87;               
        private const int PCBWIDTH = 338;         
        private const int PCBHEIGHT = 512;                
        private static Bitmap K20D50MBottom;
        private static Bitmap KL25ZBottom;
        private static Bitmap KL26ZBottom;
        private static Bitmap KL46ZTop;
        private static Bitmap KL46ZBottom;
        private static Bitmap K64FBottom;
        private static Bitmap MULTIBTop;
        private static Bitmap Rev5Bottom;
        private static Bitmap Rev5Top;

        // magnetic calibration bar feedback graphics
        private static Bitmap Image1Bar;
        private static Bitmap Image2Bars;
        private static Bitmap Image3Bars;
        private static Bitmap Image4Bars;
        private static Bitmap Image5Bars;
        private static Bitmap ImageNoCal;

        // delegate class for updating the user interface
        private delegate void MakeUpdateGUIDelegate();
        MakeUpdateGUIDelegate UpdateGUIDelegate;

        //////////////////////////////////////////////////////////////////
        // Callback for main Form initialization and load
        /////////////////////////////////////////////////////////////////

        public MainForm()
        {
            InitializeComponent();
        }

        private void MainForm_Load(object sender, EventArgs e)
        {
            int i;           // loop counters         

            // remove the Altitude and Temperature tab until an MPL3115 packet is received
            AltitudePacketsReceived = false;
            tabControl.TabPages.Remove(tabPageAltitude);

            // point the web browser to the help page
            string curDir = Directory.GetCurrentDirectory();
            webBrowser.Url = new Uri(String.Format("file:///{0}/help.html", curDir));
            toolStripMenuItemBack.Visible = false;
            toolStripMenuItemHome.Visible = false;

            // create the plot update delegate
            UpdateGUIDelegate = new MakeUpdateGUIDelegate(UpdateGUI);

            // serial port parameters
            serialPort.ReceivedBytesThreshold = SERIAL_PORT_RX_CALLBACK_THRESHOLD;
            serialPort.BaudRate = 115200;
            serialPort.WriteTimeout = 500;

            // reset the bluetooth packet processing flags
            bluetoothInsidePacket = false;
            bluetoothByteCount = 0;
            bluetoothDangling0x7D = false;
            bluetooth0x7ESyncDone = false;
            bluetoothPacketErrors = 0;
            textBoxErrors.Text = bluetoothPacketErrors.ToString("F0");

            // get the list of active serial ports and fill the combo menu
            comPortNames = System.IO.Ports.SerialPort.GetPortNames();
            comboBoxPort.Items.Clear();
            for (i = 0; i < comPortNames.Length; i++)
                comboBoxPort.Items.Add(comPortNames[i]);

            // create the image bitmaps        
            K20D50MBottom = new Bitmap(global::SensorFusion.Properties.Resources.K20D50MBottom);
            KL25ZBottom = new Bitmap(global::SensorFusion.Properties.Resources.KL25ZBottom);
            KL26ZBottom = new Bitmap(global::SensorFusion.Properties.Resources.KL26ZBottom);
            KL46ZTop = new Bitmap(global::SensorFusion.Properties.Resources.KL46ZTop);
            KL46ZBottom = new Bitmap(global::SensorFusion.Properties.Resources.KL46ZBottom);
            K64FBottom = new Bitmap(global::SensorFusion.Properties.Resources.K64FBottom);
            MULTIBTop = new Bitmap(global::SensorFusion.Properties.Resources.MULTIBTop);
            Rev5Bottom = new Bitmap(global::SensorFusion.Properties.Resources.Rev5Bottom);
            Rev5Top = new Bitmap(global::SensorFusion.Properties.Resources.Rev5Top);

            // create the magnetic calibration bitmaps         
            Image1Bar = new Bitmap(global::SensorFusion.Properties.Resources.OneGreenBar);
            Image2Bars = new Bitmap(global::SensorFusion.Properties.Resources.TwoGreenBars);
            Image3Bars = new Bitmap(global::SensorFusion.Properties.Resources.ThreeGreenBars);
            Image4Bars = new Bitmap(global::SensorFusion.Properties.Resources.FourGreenBars);
            Image5Bars = new Bitmap(global::SensorFusion.Properties.Resources.FiveGreenBars);
            ImageNoCal = new Bitmap(global::SensorFusion.Properties.Resources.NoCal);
            FitErrorpc = 1000.0;

            // reset the quaternion for the alignment with the physical monitor
            qDisplay.q0 = 1.0;
            qDisplay.q1 = qDisplay.q2 = qDisplay.q3 = 0.0;

            // zero the contents of the magnetic buffer
            for (i = 0; i < MAGBUFFSIZE; i++)
                MagBuffIndex[i] = -1;

            // reset the remote PCB and the GUI
            ResetKinetisAndPCGUI();

            return;
        }

        //////////////////////////////////////////////////////////////////
        // Delegate function which updates user interface controls and
        // writes to the data logging file
        /////////////////////////////////////////////////////////////////

        private void UpdateGUI()
        {
            double[] tmp = new double[3];
            int i, j;                    // loop counters

            // set the view perspective label
            if (toolStripMenuItemForDisplayOrientation.Checked)
            {
                labelPerspective.Text = "Perspective corrected for display orientation.";
                pictureBoxCompass.Visible = false;
            }
            else
            {
                labelPerspective.Text = "Downwards perspective with North at top of screen";
                pictureBoxCompass.Visible = true;
            }

            // View and Dynamics: Euler angles
            textBoxRoll2.Text = textBoxRoll.Text = Roll.ToString("F1");
            waveformPlotRoll.PlotYAppend(Roll);
            textBoxPitch2.Text = textBoxPitch.Text = Pitch.ToString("F1");
            waveformPlotPitch.PlotYAppend(Pitch);
            textBoxCompass2.Text = textBoxCompass.Text = Compass.ToString("F1");
            waveformPlotCompass.PlotYAppend(Compass);

            // View: power on time
            textBoxOnTime.Text = OnTime.ToString("F2");

            // View: bluetooth errors
            textBoxErrors.Text = bluetoothPacketErrors.ToString("F0");

            // View: remote PCB hardware
            switch (Hardware)
            {
                case WIN8PCB:
                    textBoxHardware.Text = "Win8 Rev 0.5";
                    break;
                case KL25ZPCB:
                    textBoxHardware.Text = "FRDM-KL25Z";
                    break;
                case K20PCB:
                    textBoxHardware.Text = "FRDM-K20D50M";
                    break;
                case KL26ZPCB:
                    textBoxHardware.Text = "FRDM-KL26Z";
                    break;
                case K64FPCB:
                    textBoxHardware.Text = "FRDM-K64F";
                    break;
                case KL16ZPCB:
                    textBoxHardware.Text = "FRDM-KL16Z";
                    break;
                case KL46ZPCB:
                case KL46ZSTANDALONEPCB:
                    textBoxHardware.Text = "FRDM-KL46Z";
                    break;
                default:
                    textBoxHardware.Text = "Unknown";
                    break;
            }

            // View: Coordinate build
            switch (Coordinates)
            {
                case NEDCOORD:
                    textBoxCoordinates.Text = "Aerospace";
                    break;
                case ANDROIDCOORD:
                    textBoxCoordinates.Text = "Android";
                    break;
                case WIN8COORD:
                    textBoxCoordinates.Text = "Windows 8";
                    break;
                default:
                    textBoxCoordinates.Text = "Unknown";
                    break;
            }

            // View: Firmware version
            textBoxFirmware.Text = Firmware.ToString("F2");

            // View: SysTick count (already in units of k)
            textBoxsystick.Text = ksystick.ToString("F1");

            // Sensors: accelerometer data
            textBoxAccX.Text = Acc[X].ToString("F3");
            waveformPlotAccX.PlotYAppend(Acc[X]);
            textBoxAccY.Text = Acc[Y].ToString("F3");
            waveformPlotAccY.PlotYAppend(Acc[Y]);
            textBoxAccZ.Text = Acc[Z].ToString("F3");
            waveformPlotAccZ.PlotYAppend(Acc[Z]);

            // Sensors: magnetometer data
            textBoxMagX.Text = Mag[X].ToString("F1");
            waveformPlotMagX.PlotYAppend(Mag[X]);
            textBoxMagY.Text = Mag[Y].ToString("F1");
            waveformPlotMagY.PlotYAppend(Mag[Y]);
            textBoxMagZ.Text = Mag[Z].ToString("F1");
            waveformPlotMagZ.PlotYAppend(Mag[Z]);

            // Sensors gyroscope data
            textBoxGyroX.Text = Gyro[X].ToString("F2");
            waveformPlotGyroX.PlotYAppend(Gyro[X]);
            textBoxGyroY.Text = Gyro[Y].ToString("F2");
            waveformPlotGyroY.PlotYAppend(Gyro[Y]);
            textBoxGyroZ.Text = Gyro[Z].ToString("F2");
            waveformPlotGyroZ.PlotYAppend(Gyro[Z]);

            // Dynamics: quaternion values
            textBoxq0.Text = qRx.q0.ToString("F4");
            textBoxq1.Text = qRx.q1.ToString("F4");
            textBoxq2.Text = qRx.q2.ToString("F4");
            textBoxq3.Text = qRx.q3.ToString("F4");

            // Dynamics: angular velocity data
            textBoxAngVelX.Text = AngVel[X].ToString("F2");
            waveformPlotAngVelX.PlotYAppend(AngVel[X]);
            textBoxAngVelY.Text = AngVel[Y].ToString("F2");
            waveformPlotAngVelY.PlotYAppend(AngVel[Y]);
            textBoxAngVelZ.Text = AngVel[Z].ToString("F2");
            waveformPlotAngVelZ.PlotYAppend(AngVel[Z]);

            // calculate the linear acceleration from the accelerometer and orientation
            switch (Coordinates)
            {
                case NEDCOORD:
                    // NED is gravity positive                               
                    Acceleration[X] = -Acc[X] + 2.0 * qRx.q1 * qRx.q3 - 2.0 * qRx.q0 * qRx.q2; // R[X, Z]
                    Acceleration[Y] = -Acc[Y] + 2.0 * qRx.q2 * qRx.q3 + 2.0 * qRx.q0 * qRx.q1; // R[Y, Z]
                    Acceleration[Z] = -Acc[Z] + 2.0 * qRx.q0 * qRx.q0 + 2.0 * qRx.q3 * qRx.q3 - 1.0; // R[Z, Z]

                    break;
                case ANDROIDCOORD:
                    // Android is ENU and acceleration positive
                    Acceleration[X] = Acc[X] - 2.0 * qRx.q1 * qRx.q3 + 2.0 * qRx.q0 * qRx.q2; // R[X, Z]
                    Acceleration[Y] = Acc[Y] - 2.0 * qRx.q2 * qRx.q3 - 2.0 * qRx.q0 * qRx.q1; // R[Y, Z]
                    Acceleration[Z] = Acc[Z] - 2.0 * qRx.q0 * qRx.q0 - 2.0 * qRx.q3 * qRx.q3 + 1.0; // R[Z, Z]
                    break;
                case WIN8COORD:
                    // Windows 8 is ENU and gravity positive
                    Acceleration[X] = -Acc[X] - 2.0 * qRx.q1 * qRx.q3 + 2.0 * qRx.q0 * qRx.q2; // R[X, Z]
                    Acceleration[Y] = -Acc[Y] - 2.0 * qRx.q2 * qRx.q3 - 2.0 * qRx.q0 * qRx.q1; // R[Y, Z]
                    Acceleration[Z] = -Acc[Z] - 2.0 * qRx.q0 * qRx.q0 - 2.0 * qRx.q3 * qRx.q3 + 1.0; // R[Z, Z]         
                    break;
                default:
                    // default to Android in the event of error
                    Acceleration[X] = Acc[X] - 2.0 * qRx.q1 * qRx.q3 + 2.0 * qRx.q0 * qRx.q2; // R[X, Z]
                    Acceleration[Y] = Acc[Y] - 2.0 * qRx.q2 * qRx.q3 - 2.0 * qRx.q0 * qRx.q1; // R[Y, Z]
                    Acceleration[Z] = Acc[Z] - 2.0 * qRx.q0 * qRx.q0 - 2.0 * qRx.q3 * qRx.q3 + 1.0; // R[Z, Z]
                    break;
            }

            // Dynamics: acceleration data       
            textBoxAccelerationX.Text = Acceleration[X].ToString("F3");
            waveformPlotAccelerationX.PlotYAppend(Acceleration[X]);
            textBoxAccelerationY.Text = Acceleration[Y].ToString("F3");
            waveformPlotAccelerationY.PlotYAppend(Acceleration[Y]);
            textBoxAccelerationZ.Text = Acceleration[Z].ToString("F3");
            waveformPlotAccelerationZ.PlotYAppend(Acceleration[Z]);

            // Kalman: orientation error vector
            textBoxThErrPlX.Text = ThErrPl[X].ToString("F3");
            waveformPlotThErrX.PlotYAppend(ThErrPl[X]);
            textBoxThErrPlY.Text = ThErrPl[Y].ToString("F3");
            waveformPlotThErrY.PlotYAppend(ThErrPl[Y]);
            textBoxThErrPlZ.Text = ThErrPl[Z].ToString("F3");
            waveformPlotThErrZ.PlotYAppend(ThErrPl[Z]);

            // Kalman: gyro offset vector
            textBoxbPlX.Text = bPl[X].ToString("F3");
            waveformPlotbPlX.PlotYAppend(bPl[X]);
            textBoxbPlY.Text = bPl[Y].ToString("F3");
            waveformPlotbPlY.PlotYAppend(bPl[Y]);
            textBoxbPlZ.Text = bPl[Z].ToString("F3");
            waveformPlotbPlZ.PlotYAppend(bPl[Z]);

            // Kalman: gyro offset error vector
            textBoxberrPlX.Text = bErrPl[X].ToString("F4");
            waveformPlotbErrPlX.PlotYAppend(bErrPl[X]);
            textBoxberrPlY.Text = bErrPl[Y].ToString("F4");
            waveformPlotbErrPlY.PlotYAppend(bErrPl[Y]);
            textBoxberrPlZ.Text = bErrPl[Z].ToString("F4");
            waveformPlotbErrPlZ.PlotYAppend(bErrPl[Z]);

            // Altitude and Temperature
            textBoxAltitude.Text = Altitude.ToString("F3");
            waveformPlotAltitude.PlotYAppend(Altitude);
            textBoxTemperature.Text = Temperature.ToString("F2");
            waveformPlotTemperature.PlotYAppend(Temperature);

            // Sensor Fusion group LED    
            ledRecord.Value = LEDRecFlag;

            // disable all Algorithm group LEDs and text boxes
            ledActivityAcc.Visible = false;
            ledActivityMag.Visible = false;
            ledActivityGyro.Visible = false;
            ledActivityAccMag.Visible = false;
            ledActivityAccGyro.Visible = false;
            ledActivityAccMagGyro.Visible = false;
            textBoxSensorsAcc_Acc.Visible = textBoxSensorsAcc_Mag.Visible = textBoxSensorsAcc_Gyro.Visible = false;
            textBoxSensorsMag_Acc.Visible = textBoxSensorsMag_Mag.Visible = textBoxSensorsMag_Gyro.Visible = false;
            textBoxSensorsGyro_Acc.Visible = textBoxSensorsGyro_Mag.Visible = textBoxSensorsGyro_Gyro.Visible = false;
            textBoxSensorsAccMag_Acc.Visible = textBoxSensorsAccMag_Mag.Visible = textBoxSensorsAccMag_Gyro.Visible = false;
            textBoxSensorsAccGyro_Acc.Visible = textBoxSensorsAccGyro_Mag.Visible = textBoxSensorsAccGyro_Gyro.Visible = false;
            textBoxSensorsAccMagGyro_Acc.Visible = textBoxSensorsAccMagGyro_Mag.Visible = textBoxSensorsAccMagGyro_Gyro.Visible = false;

            // algorithm radio button re-synchronization following reset or power up
            if (SyncQuaternionCountdown == 0)
            {
                switch (QReceived)
                {
                    case Q3:
                        radioButtonAcc.Checked = true;                   
                        break;
                    case Q3M:
                        radioButtonMag.Checked = true;               
                        break;
                    case Q3G:
                        radioButtonGyro.Checked = true;                     
                        break;
                    case Q6MA:
                        radioButtonAccMag.Checked = true;                     
                        break;
                    case Q6AG:
                        radioButtonAccGyro.Checked = true;                    
                        break;
                    case Q9:
                        radioButtonAccMagGyro.Checked = true;                      
                        break;
                    default:
                        break;
                }
            }

            // ensure the relevant items are visible and flash the appropriate LED
            switch (QReceived)
            {
                case Q3:
                    ledActivityAcc.Visible = true;
                    textBoxSensorsAcc_Acc.Visible = textBoxSensorsAcc_Mag.Visible = textBoxSensorsAcc_Gyro.Visible = true;
                    ledActivityAcc.Value = LEDPacketFlag;
                    break;
                case Q3M:
                    ledActivityMag.Visible = true;
                    textBoxSensorsMag_Acc.Visible = textBoxSensorsMag_Mag.Visible = textBoxSensorsMag_Gyro.Visible = true;
                    ledActivityMag.Value = LEDPacketFlag;
                    break;
                case Q3G:
                    ledActivityGyro.Visible = true;
                    textBoxSensorsGyro_Acc.Visible = textBoxSensorsGyro_Mag.Visible = textBoxSensorsGyro_Gyro.Visible = true;
                    ledActivityGyro.Value = LEDPacketFlag;
                    break;
                case Q6MA:
                    ledActivityAccMag.Visible = true;
                    textBoxSensorsAccMag_Acc.Visible = textBoxSensorsAccMag_Mag.Visible = textBoxSensorsAccMag_Gyro.Visible = true;
                    ledActivityAccMag.Value = LEDPacketFlag;
                    break;
                case Q6AG:
                    ledActivityAccGyro.Visible = true;
                    textBoxSensorsAccGyro_Acc.Visible = textBoxSensorsAccGyro_Mag.Visible = textBoxSensorsAccGyro_Gyro.Visible = true;
                    ledActivityAccGyro.Value = LEDPacketFlag;
                    break;
                case Q9:
                    ledActivityAccMagGyro.Visible = true;
                    textBoxSensorsAccMagGyro_Acc.Visible = textBoxSensorsAccMagGyro_Mag.Visible = textBoxSensorsAccMagGyro_Gyro.Visible = true;
                    ledActivityAccMagGyro.Value = LEDPacketFlag;
                    break;
                default:
                    break;
            }

            // buffer count
            textBoxMeasurements.Text = BufferCount.ToString("F0");

            // calibration quality feedback
            textBoxFitErrorpc.Text = FitErrorpc.ToString("F1");
            if (FitErrorpc <= 15.0)
            {
                labelFeedback.Text = "Magnetometer is calibrated";
                labelFeedbackMain.Text = "Magnetometer is calibrated";
                labelFeedback.ForeColor = System.Drawing.Color.Black;
                labelFeedbackMain.ForeColor = System.Drawing.Color.Black;
            }
            else
            {
                labelFeedback.Text = "Rotate sensor board to calibrate magnetometer";
                labelFeedbackMain.Text = "Rotate sensor board to calibrate magnetometer";
                labelFeedback.ForeColor = System.Drawing.Color.Red;
                labelFeedbackMain.ForeColor = System.Drawing.Color.Red;
            }

            // 4% or less equals five bars
            if (FitErrorpc <= 4.0)
            {
                pictureBoxFitError.Image = Image5Bars;
                pictureBoxFitErrorMain.Image = Image5Bars;
            }
            // 6.5% or less equals four bars
            else if (FitErrorpc <= 6.5)
            {
                pictureBoxFitError.Image = Image4Bars;
                pictureBoxFitErrorMain.Image = Image4Bars;
            }
            // 8% or less equals three bars
            else if (FitErrorpc <= 8.0)
            {
                pictureBoxFitError.Image = Image3Bars;
                pictureBoxFitErrorMain.Image = Image3Bars;
            }
            // 11% or less equals two bars
            else if (FitErrorpc <= 11.0)
            {
                pictureBoxFitError.Image = Image2Bars;
                pictureBoxFitErrorMain.Image = Image2Bars;
            }
            // 15% or less equals one bar
            else if (FitErrorpc <= 15.0)
            {
                pictureBoxFitError.Image = Image1Bar;
                pictureBoxFitErrorMain.Image = Image1Bar;
            }
            // above 15% default to magnetic warning icon
            else
            {
                pictureBoxFitError.Image = ImageNoCal;
                pictureBoxFitErrorMain.Image = ImageNoCal;
            }

            // update low bandwidth magnetic packet data every 5 updates
            if (MagneticVariableID % 5 == 0)
            {
                textBoxInclination.Text = Delta.ToString("F1");
                textBoxVx.Text = V[X].ToString("F1");
                textBoxVy.Text = V[Y].ToString("F1");
                textBoxVz.Text = V[Z].ToString("F1");
                textBoxWxx.Text = invW[X, X].ToString("F3");
                textBoxWyy.Text = invW[Y, Y].ToString("F3");
                textBoxWzz.Text = invW[Z, Z].ToString("F3");
                textBoxWxy.Text = invW[X, Y].ToString("F3");
                textBoxWyx.Text = invW[Y, X].ToString("F3");
                textBoxWxz.Text = invW[X, Z].ToString("F3");
                textBoxWzx.Text = invW[Z, X].ToString("F3");
                textBoxWyz.Text = invW[Y, Z].ToString("F3");
                textBoxWzy.Text = invW[Z, Y].ToString("F3");
                textBoxMagnitudeB.Text = B.ToString("F1");

                // copy the active elements of the magnetic buffer into the display arrays                                 
                j = 0;
                // handle uncalibrated option
                if (!checkBoxCalibrated.Checked)
                {
                    // loop over all elements in the buffer
                    for (i = 0; i < MAGBUFFSIZE; i++)
                    {
                        // check for valid buffer entry
                        if (MagBuffIndex[i] != -1)
                        {
                            // store this entry in the plot arrays
                            MagBuffXData[j] = MagBuff[X, i];
                            MagBuffYData[j] = MagBuff[Y, i];
                            MagBuffZData[j] = MagBuff[Z, i];
                            j++;
                        } // end of test for valid entry
                    } // end of loop over buffer elements
                } // end of uncalibrated case

                // handle calibrated option
                else
                {
                    // loop over all elements in the buffer
                    for (i = 0; i < MAGBUFFSIZE; i++)
                    {
                        // check for valid buffer entry
                        if (MagBuffIndex[i] != -1)
                        {
                            // store this entry in the plot arrays
                            tmp[X] = MagBuff[X, i] - V[X];
                            tmp[Y] = MagBuff[Y, i] - V[Y];
                            tmp[Z] = MagBuff[Z, i] - V[Z];
                            MagBuffXData[j] = invW[X, X] * tmp[X] + invW[X, Y] * tmp[Y] + invW[X, Z] * tmp[Z];
                            MagBuffYData[j] = invW[Y, X] * tmp[X] + invW[Y, Y] * tmp[Y] + invW[Y, Z] * tmp[Z];
                            MagBuffZData[j] = invW[Z, X] * tmp[X] + invW[Z, Y] * tmp[Y] + invW[Z, Z] * tmp[Z];
                            j++;
                        } // end of test for valid entry
                    } // end of loop over buffer elements
                } // end of calibrated case

                // plot the data
                scatterPlotMagBufferXY.PlotXY(MagBuffXData, MagBuffYData, 0, j);
                scatterPlotMagBufferYZ.PlotXY(MagBuffYData, MagBuffZData, 0, j);
                scatterPlotMagBufferZX.PlotXY(MagBuffZData, MagBuffXData, 0, j);

            } // end of intermittent magnetic update section           

            // enable the Altitude tab once only when the first Altitude packet type comes in
            if ((AltitudePacketsReceived) && !tabControl.Contains(tabPageAltitude))
            {
                tabControl.TabPages.Remove(tabPageHelp);
                tabControl.TabPages.Add(tabPageAltitude);
                tabControl.TabPages.Add(tabPageHelp);
            }

            // reset the image to force a callback which then displays the rotated image
            pictureBox.Image = null;

            // write data to file if logging is enabled       
            if (logFileStreamWriter != null)
            {
                if (logFileStreamWriter.BaseStream != null)
                {
                    // View: time stamp
                    logFileStreamWriter.Write(OnTime.ToString("F3") + ",");

                    // View: algorithm selected
                    if (radioButtonAcc.Checked)
                        logFileStreamWriter.Write("Acc" + ",");
                    else if (radioButtonMag.Checked)
                        logFileStreamWriter.Write("Mag" + ",");
                    else if (radioButtonGyro.Checked)
                        logFileStreamWriter.Write("Gyro" + ",");
                    else if (radioButtonAccMag.Checked)
                        logFileStreamWriter.Write("Acc+Mag" + ",");
                    else if (radioButtonAccGyro.Checked)
                        logFileStreamWriter.Write("Acc+Gyro" + ",");
                    else
                        logFileStreamWriter.Write("Acc+Mag+Gyro" + ",");

                    // View: coordinates
                    logFileStreamWriter.Write(textBoxCoordinates.Text + ",");

                    // View: Roll, Pitch, Compass Euler angles
                    logFileStreamWriter.Write(textBoxRoll.Text + ",");
                    logFileStreamWriter.Write(textBoxPitch.Text + ",");
                    logFileStreamWriter.Write(textBoxCompass.Text + ",");

                    // Sensors: accelerometer
                    logFileStreamWriter.Write(textBoxAccX.Text + ",");
                    logFileStreamWriter.Write(textBoxAccY.Text + ",");
                    logFileStreamWriter.Write(textBoxAccZ.Text + ",");
                    // Sensors: calibrated magnetometer
                    logFileStreamWriter.Write(textBoxMagX.Text + ",");
                    logFileStreamWriter.Write(textBoxMagY.Text + ",");
                    logFileStreamWriter.Write(textBoxMagZ.Text + ",");
                    // Sensors: gyroscope
                    logFileStreamWriter.Write(textBoxGyroX.Text + ",");
                    logFileStreamWriter.Write(textBoxGyroY.Text + ",");
                    logFileStreamWriter.Write(textBoxGyroZ.Text + ",");

                    // Dynamics: quaternion
                    logFileStreamWriter.Write(textBoxq0.Text + ",");
                    logFileStreamWriter.Write(textBoxq1.Text + ",");
                    logFileStreamWriter.Write(textBoxq2.Text + ",");
                    logFileStreamWriter.Write(textBoxq3.Text + ",");

                    // Dynamics: angular velocity                  
                    logFileStreamWriter.Write(textBoxAngVelX.Text + ",");
                    logFileStreamWriter.Write(textBoxAngVelY.Text + ",");
                    logFileStreamWriter.Write(textBoxAngVelZ.Text + ",");

                    // Dynamics: acceleration                  
                    logFileStreamWriter.Write(textBoxAccelerationX.Text + ",");
                    logFileStreamWriter.Write(textBoxAccelerationY.Text + ",");
                    logFileStreamWriter.Write(textBoxAccelerationZ.Text + ",");

                    // Magnetics: hard iron vector                  
                    logFileStreamWriter.Write(textBoxVx.Text + ",");
                    logFileStreamWriter.Write(textBoxVy.Text + ",");
                    logFileStreamWriter.Write(textBoxVz.Text + ",");

                    // Magnetics: soft iron matrix                  
                    logFileStreamWriter.Write(textBoxWxx.Text + ",");
                    logFileStreamWriter.Write(textBoxWyy.Text + ",");
                    logFileStreamWriter.Write(textBoxWzz.Text + ",");
                    logFileStreamWriter.Write(textBoxWxy.Text + ",");
                    logFileStreamWriter.Write(textBoxWxz.Text + ",");
                    logFileStreamWriter.Write(textBoxWyz.Text + ",");

                    // Magnetics: various
                    logFileStreamWriter.Write(textBoxMeasurements.Text + ",");
                    logFileStreamWriter.Write(textBoxFitErrorpc.Text + ",");
                    logFileStreamWriter.Write(textBoxMagnitudeB.Text + ",");
                    logFileStreamWriter.Write(textBoxInclination.Text + ",");

                    // Kalman: orientation error vector                  
                    logFileStreamWriter.Write(textBoxThErrPlX.Text + ",");
                    logFileStreamWriter.Write(textBoxThErrPlY.Text + ",");
                    logFileStreamWriter.Write(textBoxThErrPlZ.Text + ",");

                    // Kalman: gyro offset vector                  
                    logFileStreamWriter.Write(textBoxbPlX.Text + ",");
                    logFileStreamWriter.Write(textBoxbPlY.Text + ",");
                    logFileStreamWriter.Write(textBoxbPlZ.Text + ",");

                    // Kalman: gyro offset error vector                  
                    logFileStreamWriter.Write(textBoxberrPlX.Text + ",");
                    logFileStreamWriter.Write(textBoxberrPlY.Text + ",");
                    logFileStreamWriter.Write(textBoxberrPlZ.Text + ",");

                    // Atimeter: altitude and temperature
                    logFileStreamWriter.Write(textBoxAltitude.Text + ",");
                    logFileStreamWriter.Write(textBoxTemperature.Text);

                    // start a new line
                    logFileStreamWriter.Write("\r");
                }
            }

            // close down the Streamwriter here if the logging toggle is now off
            if (logFileStreamWriter != null)
                if ((logFileStreamWriter.BaseStream != null) && (!checkBoxRecord.Checked))
                {
                    try
                    {
                        logFileStreamWriter.Flush();
                        logFileStreamWriter.Close();
                    }
                    catch (Exception ex)
                    {
                        MessageBox.Show("Error: " + ex.ToString(), "ERROR");
                    }
                }

            return;
        }

        //////////////////////////////////////////////////////////////////
        // Callback for update rotating PCB image
        /////////////////////////////////////////////////////////////////

        private void pictureBox_Paint(object sender, PaintEventArgs e)
        {
            Quaternion.fquaternion qENU;        // ENU quaternion
            Quaternion.fquaternion qWindows;    // Windows DirectX orientation quaternion
            double[,] R = new double[3, 3];     // rotation matrix
            float m11, m12, m21, m22;           // matrix elements of affine transformation
            float dx, dy;                       // vector elements of affine transformation
            Matrix transformMatrix;             // matrix for affine transformation      
            double tmp;                         // scratch

            // return with no action is there in no connection to a board
            if (!serialPort.IsOpen)
                return;

            // correct the orientation for the display orientation qDisplay    
            qENU = Quaternion.qconjgAxB(qDisplay, qRx);

            // convert NED quaternion to ENU quaternion before proceeding
            if (Coordinates == NEDCOORD)
            {
                // swap x and y vector components and negate z
                tmp = qENU.q1;
                qENU.q1 = qENU.q2;
                qENU.q2 = tmp;
                qENU.q3 = -qENU.q3;
            }

            // correct for Windows DirectX being a left handed system
            qWindows.q0 = qENU.q0;
            qWindows.q1 = qENU.q1;
            qWindows.q2 = -qENU.q2;
            qWindows.q3 = qENU.q3;

            // from here onwards, everything is done in the Windows coordinate frame

            // define the corners of the image as vector quaternions in the
            // Windows coordinate frame. the z component is set to zero and
            // the image centre is assumed to be at (0, 0)
            //                  ------> +ve x
            //                 | 
            //                 |
            //                 V   +ve y
            //          
            // calculate the 3x3 orientation matrix from the quaternion
            Quaternion.fRotationMatrixFromQuaternion(ref R, ref qWindows);

            // set the Windows transform matrix to the transpose of the the 2D matrix components
            m11 = (float)(R[X, X] * PCBSCALING);
            m12 = (float)(R[Y, X] * PCBSCALING);
            m21 = (float)(R[X, Y] * PCBSCALING);
            m22 = (float)(R[Y, Y] * PCBSCALING);

            // define the transform matrix offset components          
            dx = (pictureBox.Width - m11 * PCBWIDTH - m21 * PCBHEIGHT) * 0.5F;
            dy = (pictureBox.Height - m12 * PCBWIDTH - m22 * PCBHEIGHT) * 0.5F;

            // construct the new transform matrix from its elements and apply
            transformMatrix = new Matrix(m11, m12, m21, m22, dx, dy);
            e.Graphics.Transform = transformMatrix;

            // display the appropriate image for the PCB connected          
            if ((qWindows.q1 * qWindows.q1 + qWindows.q2 * qWindows.q2) <= 0.5)
            {
                // display upper surface
                switch (Hardware)
                {
                    case WIN8PCB:
                        e.Graphics.DrawImage(Rev5Top, 0.0F, 0.0F);
                        break;
                    // sensor shield boards
                    case KL25ZPCB:
                    case K20PCB:
                    case KL26ZPCB:
                    case K64FPCB:
                    case KL16ZPCB:
                    case KL46ZPCB:
                        e.Graphics.DrawImage(MULTIBTop, 0.0F, 0.0F);
                        break;
                    // standalone KL46Z board         
                    case KL46ZSTANDALONEPCB:
                        e.Graphics.DrawImage(KL46ZTop, 0.0F, 0.0F);
                        break;
                    default:
                        e.Graphics.DrawImage(MULTIBTop, 0.0F, 0.0F);
                        break;
                }
            }
            else
            {
                // display lower PCB surface
                switch (Hardware)
                {
                    case WIN8PCB:
                        e.Graphics.DrawImage(Rev5Bottom, 0.0F, 0.0F);
                        break;
                    case KL25ZPCB:
                        e.Graphics.DrawImage(KL25ZBottom, 0.0F, 0.0F);
                        break;
                    case K20PCB:
                        e.Graphics.DrawImage(K20D50MBottom, 0.0F, 0.0F);
                        break;
                    case KL26ZPCB:
                        e.Graphics.DrawImage(KL26ZBottom, 0.0F, 0.0F);
                        break;
                    case K64FPCB:
                        e.Graphics.DrawImage(K64FBottom, 0.0F, 0.0F);
                        break;
                    case KL16ZPCB:
                        e.Graphics.DrawImage(KL26ZBottom, 0.0F, 0.0F);
                        break;
                    case KL46ZPCB:
                    case KL46ZSTANDALONEPCB:
                        e.Graphics.DrawImage(KL46ZBottom, 0.0F, 0.0F);
                        break;
                    default:
                        e.Graphics.DrawImage(KL25ZBottom, 0.0F, 0.0F);
                        break;
                }
            }

            return;
        }

        //////////////////////////////////////////////////////////////////
        // Callback for serial port index changed.
        // this is executed at power up when the software automatically
        // sets the COM port and afterwards under manual control
        /////////////////////////////////////////////////////////////////

        private void comboBoxPort_SelectedIndexChanged(object sender, EventArgs e)
        {
            // disable connection GUI elements
            ledConnection.OnColor = System.Drawing.Color.Red;

            // close the current port if open
            if (serialPort.IsOpen)
            {
                try
                {
                    serialPort.DiscardInBuffer();
                    serialPort.Close();
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Error: " + ex.ToString(), "ERROR");
                }
            }

            // open the new COM port from the combo box selection
            serialPort.PortName = (String)comboBoxPort.SelectedItem;
            try
            {
                serialPort.Open();
            }
            catch
            {
                ;
                // low level bluetooth drivers can cause a semaphore exception
                // so do nothing              
            }

            // update the GUI for the new connection
            if (serialPort.IsOpen)
            {
                // update GUI elements and empty the buffer                           
                ledConnection.OnColor = System.Drawing.Color.DodgerBlue;
                serialPort.DiscardInBuffer();

                // reset both the remote board and the PC GUI
                ResetKinetisAndPCGUI();
            }

            // the opening and closing of the port will cause serial buffer glitches
            // so reset the start of packet synchronization flag
            bluetooth0x7ESyncDone = false;
            bluetoothPacketErrors = 0;

            return;
        }

        //////////////////////////////////////////////////////////////////
        // Callback for data logging record button
        /////////////////////////////////////////////////////////////////

        private void checkBoxRecord_CheckedChanged(object sender, EventArgs e)
        {
            // handle transition to Checked
            if (checkBoxRecord.Checked)
            {
                // initialize the File/Save dialog
                saveFileDialog.FileName = "sensorfusion.csv";
                saveFileDialog.Filter = "txt files (*.csv)|*.csv|All files (*.*)|*.*";
                saveFileDialog.FilterIndex = 2;
                saveFileDialog.RestoreDirectory = true;

                // obtain the requested filename             
                if (saveFileDialog.ShowDialog() == DialogResult.OK)
                {
                    // OK button: attempt to open the file and write the header
                    try
                    {
                        logFileStreamWriter = new StreamWriter(saveFileDialog.FileName, false);
                    }
                    catch
                    {
                        MessageBox.Show("Error: Cannot open the log file for writing.\n\n" +
                            "Try again using a different log file name.", "ERROR");
                        checkBoxRecord.Checked = false;
                    }

                    // write the column headings if there is a valid stream
                    if (logFileStreamWriter != null)
                    {
                        if (logFileStreamWriter.BaseStream != null)
                        {
                            // View tab                        
                            logFileStreamWriter.Write("On Time (s),");
                            logFileStreamWriter.Write("Algorithm,");
                            logFileStreamWriter.Write("Coordinates,");
                            logFileStreamWriter.Write("Roll (deg),");
                            logFileStreamWriter.Write("Pitch (deg),");
                            logFileStreamWriter.Write("Compass (deg),");
                            // Sensors tab
                            logFileStreamWriter.Write("Accelerometer (g) X,");
                            logFileStreamWriter.Write("Accelerometer (g) Y,");
                            logFileStreamWriter.Write("Accelerometer (g) Z,");
                            logFileStreamWriter.Write("Magnetometer (uT) X,");
                            logFileStreamWriter.Write("Magnetometer (uT) Y,");
                            logFileStreamWriter.Write("Magnetometer (uT) Z,");
                            logFileStreamWriter.Write("Gyroscope (deg/s) X,");
                            logFileStreamWriter.Write("Gyroscope (deg/s) Y,");
                            logFileStreamWriter.Write("Gyroscope (deg/s) Z,");
                            // Dynamics tab
                            logFileStreamWriter.Write("q0,");
                            logFileStreamWriter.Write("q1,");
                            logFileStreamWriter.Write("q2,");
                            logFileStreamWriter.Write("q3,");
                            logFileStreamWriter.Write("Ang Vel (deg/s) X,");
                            logFileStreamWriter.Write("Ang Vel (deg/s) Y,");
                            logFileStreamWriter.Write("Ang Vel (deg/s) Z,");
                            logFileStreamWriter.Write("Acceleration (g) X,");
                            logFileStreamWriter.Write("Acceleration (g) Y,");
                            logFileStreamWriter.Write("Acceleration (g) Z,");

                            // Magnetics tab: Hard iron
                            logFileStreamWriter.Write("Hard iron (uT) X,");
                            logFileStreamWriter.Write("Hard iron (uT) Y,");
                            logFileStreamWriter.Write("Hard iron (uT) Z,");
                            // Magnetics tab: Soft iron
                            logFileStreamWriter.Write("Soft iron XX,");
                            logFileStreamWriter.Write("Soft iron YY,");
                            logFileStreamWriter.Write("Soft iron ZZ,");
                            logFileStreamWriter.Write("Soft iron XY,");
                            logFileStreamWriter.Write("Soft iron XZ,");
                            logFileStreamWriter.Write("Soft iron YZ,");
                            // Magnetics tab: Various
                            logFileStreamWriter.Write("Measurements,");
                            logFileStreamWriter.Write("Fit Error %,");
                            logFileStreamWriter.Write("Geomagnetic field (uT),");
                            logFileStreamWriter.Write("Inclination (deg),");

                            // Kalman tab: orientation error vector
                            logFileStreamWriter.Write("Orientation error (deg) X,");
                            logFileStreamWriter.Write("Orientation error (deg) Y,");
                            logFileStreamWriter.Write("Orientation error (deg) Z,");

                            // Kalman tab: gyro offset vector
                            logFileStreamWriter.Write("Gyro offset (deg/s) X,");
                            logFileStreamWriter.Write("Gyro offset (deg/s) Y,");
                            logFileStreamWriter.Write("Gyro offset (deg/s) Z,");

                            // Kalman tab: gyro offset error vector
                            logFileStreamWriter.Write("Gyro offset error (deg/s) X,");
                            logFileStreamWriter.Write("Gyro offset error (deg/s) Y,");
                            logFileStreamWriter.Write("Gyro offset error (deg/s) Z,");

                            // Altimeter tab
                            logFileStreamWriter.Write("Altitude (m),");
                            logFileStreamWriter.Write("Temp (C)");

                            // start a new line
                            logFileStreamWriter.Write("\r");
                        }
                    }
                } // end of OK button actions  
                else
                {
                    // Cancel button: uncheck the check box
                    checkBoxRecord.Checked = false;
                } // end of Cancel button actions
            } // end of test for check box transition to ticked

            // no actions here for transition to unchecked
            // the packet handler will close the writer once the
            // next write is completed
        }

        //////////////////////////////////////////////////////////////////
        // Callback for tab control
        /////////////////////////////////////////////////////////////////

        private void tabControl_SelectedIndexChanged(object sender, EventArgs e)
        {
            // check for Help page web broser selected
            if (tabControl.SelectedTab == tabPageHelp)
            {
                // make the Back and Home buttons visible
                toolStripMenuItemBack.Visible = true;
                toolStripMenuItemHome.Visible = true;
            }
            else
            {
                // make the Back and Home buttons invisible
                toolStripMenuItemBack.Visible = false;
                toolStripMenuItemHome.Visible = false;
            }

            return;
        }

        //////////////////////////////////////////////////////////////////
        // Callback for Magnetics tab Save button
        /////////////////////////////////////////////////////////////////

        private void buttonSave_Click(object sender, EventArgs e)
        {
            int i, j;                               // loop counters
            double[] Cal = new double[3];           // calibrated magnetic reading
            double[,] A = new double[3, 3];         // ellipsoid matrix components
            bool firstline;                         // flag to denote first line of output

            // initialize the File/Save dialog
            saveFileDialog.FileName = "magnetics.txt";
            saveFileDialog.Filter = "txt files (*.txt)|*.txt|All files (*.*)|*.*";
            saveFileDialog.FilterIndex = 2;
            saveFileDialog.RestoreDirectory = true;

            // obtain the requested filename             
            if (saveFileDialog.ShowDialog() == DialogResult.OK)
            {
                // OK button: attempt to open the file and write the header
                try
                {
                    magBufferStreamWriter = new StreamWriter(saveFileDialog.FileName, false);
                }
                catch
                {
                    MessageBox.Show("Error: Cannot open the output file for writing.\n\n" +
                        "Try again using a different file name.", "ERROR");
                }

                // calculate the ellipsoid matrix A
                for (i = X; i <= Z; i++)
                    for (j = X; j <= Z; j++)
                        A[i, j] = invW[i, X] * invW[X, j] + invW[i, Y] * invW[Y, j] + invW[i, Z] * invW[Z, j];

                // write the column headings if there is a valid stream
                if (magBufferStreamWriter != null)
                {
                    if (magBufferStreamWriter.BaseStream != null)
                    {
                        // hard iron coefficients
                        magBufferStreamWriter.WriteLine("{0}\t{1}", "Hard iron V[X] (uT)", V[X].ToString("F2"));
                        magBufferStreamWriter.WriteLine("{0}\t{1}", "Hard iron V[Y] (uT)", V[Y].ToString("F2"));
                        magBufferStreamWriter.WriteLine("{0}\t{1}", "Hard iron V[Z] (uT)", V[Z].ToString("F2"));
                        // soft iron coefficients 
                        magBufferStreamWriter.WriteLine("{0}\t{1}\t{2}\t{3}", "Inverse Soft Iron invW[X][X] [X][Y] [X][Z]",
                            invW[X, X].ToString("F3"), invW[X, Y].ToString("F3"), invW[X, Z].ToString("F3"));
                        magBufferStreamWriter.WriteLine("{0}\t{1}\t{2}\t{3}", "Inverse Soft Iron invW[Y][X] [Y][Y] [Y][Z]",
                            invW[Y, X].ToString("F3"), invW[Y, Y].ToString("F3"), invW[Y, Z].ToString("F3"));
                        magBufferStreamWriter.WriteLine("{0}\t{1}\t{2}\t{3}", "Inverse Soft Iron invW[Z][X] [Z][Y] [Z][Z]",
                            invW[Z, X].ToString("F3"), invW[Z, Y].ToString("F3"), invW[Z, Z].ToString("F3"));

                        // ellipsoid matrix
                        magBufferStreamWriter.WriteLine("{0}\t{1}\t{2}\t{3}", "Ellipsoid matrix A[X][X] [X][Y] [X][Z]",
                            A[X, X].ToString("F3"), A[X, Y].ToString("F3"), A[X, Z].ToString("F3"));
                        magBufferStreamWriter.WriteLine("{0}\t{1}\t{2}\t{3}", "Ellipsoid matrix A[Y][X] [Y][Y] [Y][Z]",
                            A[Y, X].ToString("F3"), A[Y, Y].ToString("F3"), A[Y, Z].ToString("F3"));
                        magBufferStreamWriter.WriteLine("{0}\t{1}\t{2}\t{3}", "Ellipsoid matrix A[Z][X] [Z][Y] [Z][Z]",
                            A[Z, X].ToString("F3"), A[Z, Y].ToString("F3"), A[Z, Z].ToString("F3"));

                        // geomagnetic field strength
                        magBufferStreamWriter.WriteLine("{0}\t{1}", "Geomagnetic field B (uT)", B.ToString("F1"));
                        magBufferStreamWriter.WriteLine("{0}\t{1}", "Geomagnetic inclination (deg)", Delta.ToString("F1"));

                        // calibration fit
                        magBufferStreamWriter.WriteLine("{0}\t{1}", "Calibration fit error %", FitErrorpc.ToString("F2"));
                        
                        // number of measurements
                        magBufferStreamWriter.WriteLine("{0}\t{1}", "Measurements in buffer", BufferCount.ToString("F0"));

                        // write the column headings                                      
                        magBufferStreamWriter.WriteLine("X uT" + "\t" + "Y uT" + "\t" + "Z uT");

                        // write all valid entries in the magnetic buffer
                        for (i = 0; i < MAGBUFFSIZE; i++)
                        {
                            if (MagBuffIndex[i] == 1)
                            {
                                magBufferStreamWriter.Write(MagBuff[X, i] + "\t");
                                magBufferStreamWriter.Write(MagBuff[Y, i] + "\t");
                                magBufferStreamWriter.WriteLine(MagBuff[Z, i]);
                            }
                        }

                        // mathematica visualisation script
                        magBufferStreamWriter.WriteLine();
                        magBufferStreamWriter.WriteLine("------ Cut and paste text below into Mathematica for data visualization ------");

                        // write out raw magnetic buffer data
                        firstline = true;
                        magBufferStreamWriter.WriteLine("uncaldata={");
                        for (i = 0; i < MAGBUFFSIZE; i++)
                        {
                            if (MagBuffIndex[i] != -1)
                            {
                                if (firstline)
                                {
                                    magBufferStreamWriter.Write("{");
                                    firstline = false;
                                }
                                else
                                {
                                    magBufferStreamWriter.WriteLine(",");
                                    magBufferStreamWriter.Write("{");
                                }
                                magBufferStreamWriter.Write(MagBuff[X, i].ToString("F2") + ",");
                                magBufferStreamWriter.Write(MagBuff[Y, i].ToString("F2") + ",");
                                magBufferStreamWriter.Write(MagBuff[Z, i].ToString("F2"));
                                magBufferStreamWriter.Write("}");
                            }
                        }
                        magBufferStreamWriter.WriteLine("}");
                        magBufferStreamWriter.WriteLine();

                        // write out calibrated magnetic buffer data
                        firstline = true;
                        magBufferStreamWriter.WriteLine("caldata={");
                        for (i = 0; i < MAGBUFFSIZE; i++)
                        {
                            if (MagBuffIndex[i] != -1)
                            {
                                for (j = X; j <= Z; j++)
                                {
                                    Cal[j] = invW[j, X] * (MagBuff[X, i] - V[X]) +
                                        invW[j, Y] * (MagBuff[Y, i] - V[Y]) +
                                        invW[j, Z] * (MagBuff[Z, i] - V[Z]);
                                }
                                if (firstline)
                                {
                                    magBufferStreamWriter.Write("{");
                                    firstline = false;
                                }
                                else
                                {
                                    magBufferStreamWriter.WriteLine(",");
                                    magBufferStreamWriter.Write("{");
                                }
                                magBufferStreamWriter.Write(Cal[X].ToString("F2") + ",");
                                magBufferStreamWriter.Write(Cal[Y].ToString("F2") + ",");
                                magBufferStreamWriter.Write(Cal[Z].ToString("F2"));
                                magBufferStreamWriter.Write("}");
                            }
                        }
                        magBufferStreamWriter.WriteLine("}");
                        magBufferStreamWriter.WriteLine();

                        // mathematica plot 1 of raw magnetic data
                        magBufferStreamWriter.WriteLine("{0}", "plot1 = ListPointPlot3D[uncaldata, AxesLabel -> {Bx, By, Bz}, Axes -> True, BaseStyle -> {FontSize -> 14}, BoxRatios -> Automatic, PlotStyle -> Directive[Red, PointSize[0.02]]]");
                        magBufferStreamWriter.WriteLine("{0}{1}{2}{3}{4}{5}{6}", "a={{", A[X, X], ",", A[X, Y], ",", A[X, Z], "},");
                        magBufferStreamWriter.WriteLine("{0}{1}{2}{3}{4}{5}{6}", "{", A[Y, X], ",", A[Y, Y], ",", A[Y, Z], "},");
                        magBufferStreamWriter.WriteLine("{0}{1}{2}{3}{4}{5}{6}", "{", A[Z, X], ",", A[Z, Y], ",", A[Z, Z], "}}");
                        // mathematica plot 2 of fitted ellipsoid
                        magBufferStreamWriter.WriteLine("{0}{1}{2}{3}{4}{5}{6}", "plot2=RegionPlot3D[{x-(", V[X], "),y-(", V[Y], "),z-(", V[Z], ")}.a.");
                        magBufferStreamWriter.WriteLine("{0}{1}{2}{3}{4}{5}{6}{7}{8}", "{x-(", V[X], "),y-(", V[Y], "),z-(", V[Z], ")}<", B * B, ",");
                        magBufferStreamWriter.WriteLine("{0}{1}{2}{3}{4}{5}{6}{7}{8}{9}{10}{11}{12}", "{x,", V[X], "-120,", V[X], "+120}, {y,", V[Y], "-120,", V[Y], "+120}, {z,", V[Z], "-120,", V[Z], "+120},");
                        magBufferStreamWriter.WriteLine("{0}", "PlotPoints -> 100,  Mesh -> None, AxesLabel -> {Bx, By, Bz}, BaseStyle -> {FontSize -> 14}, AxesLabel -> Automatic, PlotStyle -> Opacity[0.5]]");
                        // mathematica plot 3 of calibrated data
                        magBufferStreamWriter.WriteLine("{0}", "plot3 = ListPointPlot3D[caldata, AxesLabel -> {Bx, By, Bz}, Axes -> True, BaseStyle -> {FontSize -> 14}, BoxRatios -> Automatic, PlotStyle -> Directive[Blue, PointSize[0.02]]]");
                        // mathematica plot 4 of fitted calibration sphere
                        magBufferStreamWriter.WriteLine("{0}{1}{2}", "plot4=RegionPlot3D[x*x+y*y+z*z<", B * B, ", {x,-100,100}, {y,-100,100}, {z,-100,100}, ");
                        magBufferStreamWriter.WriteLine("{0}", "PlotPoints -> 100,  Mesh -> None, AxesLabel -> {Bx, By, Bz}, BaseStyle -> {FontSize -> 14}, AxesLabel -> Automatic, PlotStyle -> Opacity[0.5]]");
                        // combine plots together
                        magBufferStreamWriter.WriteLine("{0}", "Show[plot1, plot2, PlotRange -> Automatic]");
                        magBufferStreamWriter.WriteLine("{0}", "Show[plot3, plot4, PlotRange -> Automatic]");
                        magBufferStreamWriter.WriteLine("{0}", "Show[plot2, plot4, PlotRange -> Automatic]");
                        magBufferStreamWriter.WriteLine("{0}", "Show[plot1, plot2, plot3, plot4, PlotRange -> Automatic]");

                        // close the file
                        magBufferStreamWriter.Flush();
                        magBufferStreamWriter.Close();
                    }
                }
            }

            return;
        }

        private void saveFileDialog_FileOk(object sender, CancelEventArgs e)
        {
            return;
        }

        //////////////////////////////////////////////////////////////////
        // Callback for Kinetis packet parsing
        /////////////////////////////////////////////////////////////////

        // this function is called when a complete packet (including the start
        // 0x7E byte but not the end 0x7E byte) is in the bluetooth buffer
        // Note: the decoding assumes that the least significant byte is transmitted
        // first since Kinetis ARM is little endian by default.
        // Note: this function is called by the serial port callback and is therefore also
        // in a separate thread from the main user interface.
        // it must therefore not change any UI elements including NI graphing controls
        public void ParseBluetoothPacket()
        {
            int PacketType;                             // packet type 
            byte PacketNumber;                          // packet number
            float[] tmpfloatArray = new float[1];       // scratch
            UInt32[] tmpUInt32Array = new UInt32[1];    // scratch
            Int32[] tmpInt32Array = new Int32[1];       // scratch
            UInt16[] tmpUInt16Array = new UInt16[1];    // scratch
            Int16[] tmpInt16Array = new Int16[1];       // scratch  
            int MagneticBufferIndex;                    // index of magnetic buffer transmitted
            bool BufferEntryValid;                      // flag to denote active magnetic buffer entry

            // processing common to all packet types
            // skip byte [0] (0x7E start byte)
            // byte [1]: packet type (all packet types)
            PacketType = bluetoothInputBuffer[1];
            // byte [2]: packet number (all packet types)
            PacketNumber = bluetoothInputBuffer[2];

            // process packet type 1 (main)
            if (PacketType == 1)
            {             
                // bytes [6: 3]: time stamp (use standard packet 1)          
                // extract the 1MHz clock ticks and convert to secs
                Buffer.BlockCopy(bluetoothInputBuffer, 3, tmpUInt32Array, 0, 4);
                OnTime = (double)tmpUInt32Array[0] / 1E6;

                // bytes [8, 7]: Accelerometer x data (1g = 8192 counts)
                // bytes [10, 9]: Accelerometer y data (1g = 8192 counts)
                // bytes [12, 11]: Accelerometer z data (1g = 8192 counts)
                Buffer.BlockCopy(bluetoothInputBuffer, 7, tmpInt16Array, 0, 2);
                Acc[X] = (double)tmpInt16Array[0] / 8192.0;                 
                Buffer.BlockCopy(bluetoothInputBuffer, 9, tmpInt16Array, 0, 2);
                Acc[Y] = (double)tmpInt16Array[0] / 8192.0;              
                Buffer.BlockCopy(bluetoothInputBuffer, 11, tmpInt16Array, 0, 2);
                Acc[Z] = (double)tmpInt16Array[0] / 8192.0;              

                // bytes [14, 13]: Magnetometer x data (1uT = 10 counts)
                // bytes [16, 15]: Magnetometer y data (1uT = 10 counts)
                // bytes [18, 17]: Magnetometer z data (1uT = 10 counts)
                Buffer.BlockCopy(bluetoothInputBuffer, 13, tmpInt16Array, 0, 2);
                Mag[X] = (double)tmpInt16Array[0] * 0.1;
                Buffer.BlockCopy(bluetoothInputBuffer, 15, tmpInt16Array, 0, 2);
                Mag[Y] = (double)tmpInt16Array[0] * 0.1;
                Buffer.BlockCopy(bluetoothInputBuffer, 17, tmpInt16Array, 0, 2);
                Mag[Z] = (double)tmpInt16Array[0] * 0.1;

                // bytes [20, 19]: Gyro x data (1 deg/sec = 20 counts)
                // bytes [22, 21]: Gyro y data (1 deg/sec = 20 counts)
                // bytes [24, 23]: Gyro z data (1 deg/sec = 20 counts)
                Buffer.BlockCopy(bluetoothInputBuffer, 19, tmpInt16Array, 0, 2);
                Gyro[X] = (double)tmpInt16Array[0] * 0.05;
                Buffer.BlockCopy(bluetoothInputBuffer, 21, tmpInt16Array, 0, 2);
                Gyro[Y] = (double)tmpInt16Array[0] * 0.05;
                Buffer.BlockCopy(bluetoothInputBuffer, 23, tmpInt16Array, 0, 2);
                Gyro[Z] = (double)tmpInt16Array[0] * 0.05;

                // skip [26, 25] (q0) since this will be re-computed to normalize
                // bytes [28, 27]: quaternion q1 (1.0F = 30,000 counts)          
                // bytes [30, 29]: quaternion q2 (1.0F = 30,000 counts)   
                // bytes [32, 31]: quaternion q3 (1.0F = 30,000 counts)
                // compute q9 to normalize the quaternion
                Buffer.BlockCopy(bluetoothInputBuffer, 27, tmpInt16Array, 0, 2);
                qRx.q1 = (float)tmpInt16Array[0] / 30000.0F;
                Buffer.BlockCopy(bluetoothInputBuffer, 29, tmpInt16Array, 0, 2);
                qRx.q2 = (float)tmpInt16Array[0] / 30000.0F;
                Buffer.BlockCopy(bluetoothInputBuffer, 31, tmpInt16Array, 0, 2);
                qRx.q3 = (float)tmpInt16Array[0] / 30000.0F;
                qRx.q0 = (float)Math.Sqrt(1.0F - qRx.q1 * qRx.q1 - qRx.q2 * qRx.q2 - qRx.q3 * qRx.q3);

                // byte [33] least significant nibble
                // flags for quaternion type
                // Q3:   coordinate nibble, 1
                // Q3M:	 coordinate nibble, 6
                // Q3G:	 coordinate nibble, 3
                // Q6MA: coordinate nibble, 2
                // Q6AG: coordinate nibble, 4
                // Q9:   coordinate nibble, 8
                QReceived = bluetoothInputBuffer[33] & 0x0F;

                // byte [33] most significant nibble
                // flags for coordinate system (n is arbitrary nibble)       
                // NED:       0, quaternion nibble    
                // Android:	  1, quaternion nibble
                // Windows 8: 2, quaternion nibble
                Coordinates = bluetoothInputBuffer[33] & 0x30;

                // byte [34]: remote PCB hardware
                Hardware = bluetoothInputBuffer[34];

                // byte[35]: reliability checking for terminating 0x7E
                if (bluetoothInputBuffer[35] != 0x7E)
                    bluetoothPacketErrors++;
            }

            // extract data from the byte fields for packet 2 (debug)
            else if (PacketType == 2)
            {
                // bytes [4, 3]: software build number
                Buffer.BlockCopy(bluetoothInputBuffer, 3, tmpUInt16Array, 0, 2);
                Firmware = ((double)tmpUInt16Array[0]) / 100.0;

                // bytes [6, 5]: systick / 20 then further divided by 50 to give k
                Buffer.BlockCopy(bluetoothInputBuffer, 5, tmpUInt16Array, 0, 2);
                ksystick = ((double)tmpUInt16Array[0]) / 50.0;

                // byte [7]: reliability checking for terminating 0x7E
                if (bluetoothInputBuffer[7] != 0x7E)
                    bluetoothPacketErrors++;
            }

            // extract data from the byte fields for packet 3 (angular velocity)
            else if (PacketType == 3)
            {
                // bytes [8, 7]: Angular velocity x data (1 deg/sec = 20 counts)
                // bytes [10, 9]: Angular velocity y data (1 deg/sec = 20 counts)
                // bytes [12, 11]: Angular velocity z data (1 deg/sec = 20 counts)
                Buffer.BlockCopy(bluetoothInputBuffer, 7, tmpInt16Array, 0, 2);
                AngVel[X] = (double)tmpInt16Array[0] * 0.05;
                Buffer.BlockCopy(bluetoothInputBuffer, 9, tmpInt16Array, 0, 2);
                AngVel[Y] = (double)tmpInt16Array[0] * 0.05;
                Buffer.BlockCopy(bluetoothInputBuffer, 11, tmpInt16Array, 0, 2);
                AngVel[Z] = (double)tmpInt16Array[0] * 0.05;

                // byte [13]: reliability checking for terminating 0x7E
                if (bluetoothInputBuffer[13] != 0x7E)
                    bluetoothPacketErrors++;
            }

            // extract data from the byte fields for packet 4 (angles)
            else if (PacketType == 4)
            {
                // bytes [8, 7]: roll angle (1 count = 0.1deg)
                // bytes [10, 9]: pitch angle (1 count = 0.1deg)
                // bytes [12, 11]: compass angle (1 count = 0.1deg)
                Buffer.BlockCopy(bluetoothInputBuffer, 7, tmpInt16Array, 0, 2);
                Roll = (double)tmpInt16Array[0] * 0.1;
                Buffer.BlockCopy(bluetoothInputBuffer, 9, tmpInt16Array, 0, 2);
                Pitch = (double)tmpInt16Array[0] * 0.1;
                Buffer.BlockCopy(bluetoothInputBuffer, 11, tmpInt16Array, 0, 2);
                Compass = (double)tmpInt16Array[0] * 0.1;

                // byte [13]: reliability checking for terminating 0x7E
                if (bluetoothInputBuffer[13] != 0x7E)
                    bluetoothPacketErrors++;

            }

            // extract data from the byte fields for packet 5 (pressure, temperature)
            else if (PacketType == 5)
            {
                // set the flag denoting that this packet is supported by the remote board
                AltitudePacketsReceived = true;

                // bytes [10: 7]: altitude (m) times 1000
                // bytes [12: 11]: temperature (C) times 100          
                Buffer.BlockCopy(bluetoothInputBuffer, 7, tmpInt32Array, 0, 4);
                Altitude = (double)tmpInt32Array[0] * 0.001;
                Buffer.BlockCopy(bluetoothInputBuffer, 11, tmpInt16Array, 0, 2);
                Temperature = (double)tmpInt16Array[0] * 0.01;

                // byte [13]: reliability checking for terminating 0x7E
                if (bluetoothInputBuffer[13] != 0x7E)
                    bluetoothPacketErrors++;
            }

            // extract data from the byte fields for packet 6 (magnetic)
            else if (PacketType == 6)
            {
                // bytes [4, 3]: number of active measurements in the magnetic buffer
                Buffer.BlockCopy(bluetoothInputBuffer, 3, tmpInt16Array, 0, 2);
                BufferCount = (int)tmpInt16Array[0];

                // bytes [6, 5]: fit error %
                Buffer.BlockCopy(bluetoothInputBuffer, 5, tmpInt16Array, 0, 2);
                FitErrorpc = ((double)tmpInt16Array[0]) / 10.0;

                // bytes [8, 7]: definition of the three variables being transmitted
                Buffer.BlockCopy(bluetoothInputBuffer, 7, tmpInt16Array, 0, 2);
                MagneticVariableID = (int)tmpInt16Array[0];

                // determine if the entry is negative (-10 or less to be precise) and invalid
                if (MagneticVariableID >= 0)
                {
                    BufferEntryValid = true;
                }
                else
                {
                    MagneticVariableID = -MagneticVariableID;
                    BufferEntryValid = false;
                }

                // bytes [10, 9]: variable 1
                // bytes [12, 11]: variable 2
                // bytes [14, 13]: variable 3
                switch (MagneticVariableID)
                {
                    case 0:
                        Buffer.BlockCopy(bluetoothInputBuffer, 9, tmpInt16Array, 0, 2);
                        // this slot is currently unused
                        Buffer.BlockCopy(bluetoothInputBuffer, 11, tmpInt16Array, 0, 2);
                        B = ((double)tmpInt16Array[0]) / 10.0;
                        Buffer.BlockCopy(bluetoothInputBuffer, 13, tmpInt16Array, 0, 2);
                        Delta = ((double)tmpInt16Array[0]) / 10.0;
                        break;
                    case 1:
                        // hard iron offset vector
                        Buffer.BlockCopy(bluetoothInputBuffer, 9, tmpInt16Array, 0, 2);
                        V[X] = ((double)tmpInt16Array[0]) / 10.0;
                        Buffer.BlockCopy(bluetoothInputBuffer, 11, tmpInt16Array, 0, 2);
                        V[Y] = ((double)tmpInt16Array[0]) / 10.0;
                        Buffer.BlockCopy(bluetoothInputBuffer, 13, tmpInt16Array, 0, 2);
                        V[Z] = ((double)tmpInt16Array[0]) / 10.0;
                        break;
                    case 2:
                        // diagonal elements of soft iron matrix
                        Buffer.BlockCopy(bluetoothInputBuffer, 9, tmpInt16Array, 0, 2);
                        invW[X, X] = ((double)tmpInt16Array[0]) / 1000.0;
                        Buffer.BlockCopy(bluetoothInputBuffer, 11, tmpInt16Array, 0, 2);
                        invW[Y, Y] = ((double)tmpInt16Array[0]) / 1000.0;
                        Buffer.BlockCopy(bluetoothInputBuffer, 13, tmpInt16Array, 0, 2);
                        invW[Z, Z] = ((double)tmpInt16Array[0]) / 1000.0;
                        break;
                    case 3:
                        // off-diagonal elements of soft iron matrix
                        Buffer.BlockCopy(bluetoothInputBuffer, 9, tmpInt16Array, 0, 2);
                        invW[Y, X] = invW[X, Y] = ((double)tmpInt16Array[0]) / 1000.0;
                        Buffer.BlockCopy(bluetoothInputBuffer, 11, tmpInt16Array, 0, 2);
                        invW[Z, X] = invW[X, Z] = ((double)tmpInt16Array[0]) / 1000.0;
                        Buffer.BlockCopy(bluetoothInputBuffer, 13, tmpInt16Array, 0, 2);
                        invW[Z, Y] = invW[Y, Z] = ((double)tmpInt16Array[0]) / 1000.0;
                        break;
                    case 4:
                        // for future expansion                     
                        break;
                    case 5:
                        // for future expansion
                        break;
                    case 6:
                        // for future expansion
                        break;
                    case 7:
                        // for future expansion
                        break;
                    case 8:
                        // for future expansion
                        break;
                    case 9:
                        // for future expansion
                        break;
                    default:
                        // 10 upwards: magnetic buffer contents
                        // correct for buffer elements being transmitted in slots 10 and up
                        MagneticBufferIndex = MagneticVariableID - 10;
                        // defensive check against bluetooth transmission errors
                        if ((MagneticBufferIndex >= 0) && (MagneticBufferIndex < MAGBUFFSIZE))
                        {
                            if (BufferEntryValid)
                            {
                                MagBuffIndex[MagneticBufferIndex] = 1;
                                Buffer.BlockCopy(bluetoothInputBuffer, 9, tmpInt16Array, 0, 2);
                                MagBuff[X, MagneticBufferIndex] = ((double)tmpInt16Array[0]) * 0.1;
                                Buffer.BlockCopy(bluetoothInputBuffer, 11, tmpInt16Array, 0, 2);
                                MagBuff[Y, MagneticBufferIndex] = ((double)tmpInt16Array[0]) * 0.1;
                                Buffer.BlockCopy(bluetoothInputBuffer, 13, tmpInt16Array, 0, 2);
                                MagBuff[Z, MagneticBufferIndex] = ((double)tmpInt16Array[0]) * 0.1;
                            }
                            else
                            {
                                // deactivate this element
                                MagBuffIndex[MagneticBufferIndex] = -1;
                            }
                        }
                        break;
                }

                // byte [15]: reliability checking for terminating 0x7E
                if (bluetoothInputBuffer[15] != 0x7E)
                    bluetoothPacketErrors++;
            }

            // kalman filter packet type
            else if (PacketType == 7)
            {
                // [4-3]: ThErrPl[X] resolution 0.001 deg
                // [6-5]: ThErrPl[Y] resolution 0.001 deg
                // [8-7]: ThErrPl[Z] resolution 0.001 deg
                Buffer.BlockCopy(bluetoothInputBuffer, 3, tmpInt16Array, 0, 2);
                ThErrPl[X] = ((double)tmpInt16Array[0]) * 0.001;
                Buffer.BlockCopy(bluetoothInputBuffer, 5, tmpInt16Array, 0, 2);
                ThErrPl[Y] = ((double)tmpInt16Array[0]) * 0.001;
                Buffer.BlockCopy(bluetoothInputBuffer, 7, tmpInt16Array, 0, 2);
                ThErrPl[Z] = ((double)tmpInt16Array[0]) * 0.001;

                // [10-9]: bPl[X] resolution 0.001 deg/sec
                // [12-11]: bPl[Y] resolution 0.001 deg/sec
                // [14-13]: bPl[Z] resolution 0.001 deg/sec
                Buffer.BlockCopy(bluetoothInputBuffer, 9, tmpInt16Array, 0, 2);
                bPl[X] = ((double)tmpInt16Array[0]) * 0.001;
                Buffer.BlockCopy(bluetoothInputBuffer, 11, tmpInt16Array, 0, 2);
                bPl[Y] = ((double)tmpInt16Array[0]) * 0.001;
                Buffer.BlockCopy(bluetoothInputBuffer, 13, tmpInt16Array, 0, 2);
                bPl[Z] = ((double)tmpInt16Array[0]) * 0.001;

                // [16-15]: bErrPl[X] resolution 0.0001deg/s
                // [18-17]: bErrPl[Y] resolution 0.0001deg/s
                // [20-19]: bErrPl[Z] resolution 0.0001deg/s
                Buffer.BlockCopy(bluetoothInputBuffer, 15, tmpInt16Array, 0, 2);
                bErrPl[X] = ((double)tmpInt16Array[0]) * 0.0001;
                Buffer.BlockCopy(bluetoothInputBuffer, 17, tmpInt16Array, 0, 2);
                bErrPl[Y] = ((double)tmpInt16Array[0]) * 0.0001;
                Buffer.BlockCopy(bluetoothInputBuffer, 19, tmpInt16Array, 0, 2);
                bErrPl[Z] = ((double)tmpInt16Array[0]) * 0.0001;

                // byte [21]: reliability checking for terminating 0x7E
                if (bluetoothInputBuffer[21] != 0x7E)
                    bluetoothPacketErrors++;
            }

            // invalid packet type (should never happen)
            else
            {
                bluetoothPacketErrors++;
            }


            // flash the disc logging LED if data logging in progress
            if (checkBoxRecord.Checked)
            {
                if ((PacketNumber & 63) == 0)
                    LEDRecFlag = !LEDRecFlag;
            }
            else
            {
                LEDRecFlag = false;
            }

            // flash the packet LED
            if ((PacketNumber & 63) == 0)
                LEDPacketFlag = !LEDPacketFlag;

            // reset the flags for the next packet
            bluetoothInsidePacket = false;
            bluetoothByteCount = 0;
            bluetoothDangling0x7D = false;
    
            // invoke the delegate to update the GUI and log to disc if enabled
            if (PacketType == 1)
            {
                // decrease the synchronization time for which 0 means sync and -1 is its stable value
                if (SyncQuaternionCountdown >= 0)
                {
                    SyncQuaternionCountdown--;
                }
                tabControl.BeginInvoke(UpdateGUIDelegate);
            }

            return;
        }

        //////////////////////////////////////////////////////////////////
        // Callback for serial port data received
        /////////////////////////////////////////////////////////////////

        // Note: .NET invokes this callback in a separate thread from the main user interface thread.
        // this function must therefore not change any UI elements including NI graphing controls
        private void serialPort_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            int i;              // loop counter
            int bytesRead;      // bytes read from the serial port

            // return if for some reason the serial port is not open
            if (!serialPort.IsOpen)
                return;

            // read the entire serial port buffer up to maximum of buffer size
            bytesRead = serialPort.Read(serialPortBuffer, 0, SERIAL_PORT_RX_BUFFER_SIZE);

            // process every byte in the buffer before leaving this callback
            for (i = 0; i < bytesRead; i++)
            {
                // 1: skip to the next 0x7E if we're not in a packet
                if (!bluetoothInsidePacket)
                {
                    // first byte after previous packet completion should be 0x7E
                    if (serialPortBuffer[i] == 0x7E)
                    {
                        // assume it's a start byte (and correct later if it's a stop)
                        bluetoothInputBuffer[bluetoothByteCount++] = 0x7E;
                        bluetoothInsidePacket = true;
                        bluetooth0x7ESyncDone = true;
                    }
                    // only increment the packet error count if 0x7E has previously been detected
                    else if (bluetooth0x7ESyncDone)
                    {
                        bluetoothPacketErrors++;
                    }
                }

                // from here on it is known that we are inside a packet

                // 2: first serial buffer byte and dangling 0x7D      
                else if ((i == 0) && bluetoothDangling0x7D)
                {
                    // 2a: check for 0x7D 0x5E sequence = 0x7E (data) split across callbacks
                    if (serialPortBuffer[i] == 0x5E)
                    {
                        bluetoothInputBuffer[bluetoothByteCount++] = 0x7E;
                        bluetoothDangling0x7D = false;
                    }
                    // 2b: check for 0x7D 0x5D sequence = 0x7D split across callbacks
                    else if (serialPortBuffer[i] == 0x5D)
                    {
                        bluetoothInputBuffer[bluetoothByteCount++] = 0x7D;
                        bluetoothDangling0x7D = false;
                    }
                    // 2c: check for 0x7D 0x7E sequence = 0x7D 0x7E (end) split across callbacks
                    else if (serialPortBuffer[i] == 0x7E)
                    {
                        // process the dangling 0x7D and 0x7E and process the completed packet
                        bluetoothInputBuffer[bluetoothByteCount++] = 0x7D;
                        bluetoothInputBuffer[bluetoothByteCount++] = 0x7E;
                        bluetoothDangling0x7D = false;
                        ParseBluetoothPacket();
                    }
                    // 2d: default case with nothing special
                    else
                    {
                        // process the dangling 0x7D and add the current byte
                        bluetoothInputBuffer[bluetoothByteCount++] = 0x7D;
                        bluetoothInputBuffer[bluetoothByteCount++] = serialPortBuffer[i];
                        bluetoothDangling0x7D = false;
                    }
                }

                // at this point any existing dangling 0x7D condition has been handled

                // 3: serial port byte is 0x7E (end or possibly start)
                else if (serialPortBuffer[i] == 0x7E)
                {
                    // check for initial power-up sync accidentally done on end 0x7E (quite likely)
                    if (bluetoothByteCount == 1)
                    {
                        // 3a: place start 0x7E at beginning of the buffer
                        bluetoothByteCount = 0;
                        bluetoothInputBuffer[bluetoothByteCount++] = 0x7E;
                    }
                    else
                    {
                        // 3b: it's a valid end 0x7E so process the complete bluetooth packet
                        bluetoothInputBuffer[bluetoothByteCount++] = 0x7E;
                        ParseBluetoothPacket();
                    }
                }

                // 4: end of serial buffer
                else if (i == (bytesRead - 1))
                {
                    // check for new dangling 0x7D condition
                    if (serialPortBuffer[i] == 0x7D)
                    {
                        // 4a: set the dangling flag ready for the next serial buffer read callback
                        bluetoothDangling0x7D = true;
                    }
                    else
                    {
                        // 4b: add the serial byte to the bluetooth packet
                        // the 0x7E end of packet case has already been detected and handled
                        bluetoothInputBuffer[bluetoothByteCount++] = serialPortBuffer[i];
                    }
                }

                // from here on, it is known that this is not the last byte in the buffer

                // 5: handle the escape sequences 0x7D 0x5E and 0x7D 0x5D
                else if (serialPortBuffer[i] == 0x7D)
                {
                    if (serialPortBuffer[i + 1] == 0x5E)
                    {
                        // 5a: replace 0x7D 0x5E with 0x7E (data)
                        bluetoothInputBuffer[bluetoothByteCount++] = 0x7E;
                        i++;
                    }
                    else if (serialPortBuffer[i + 1] == 0x5D)
                    {
                        // 5b: replace 0x7D 0x5D with 0x7D (data
                        bluetoothInputBuffer[bluetoothByteCount++] = 0x7D;
                        i++;
                    }
                    else
                    {
                        // 5c: just 0x7D and not an escape sequence
                        bluetoothInputBuffer[bluetoothByteCount++] = 0x7D;
                    }
                }

                // 6: all other cases
                else
                {
                    // simply add the serial byte to the bluetooth buffer
                    bluetoothInputBuffer[bluetoothByteCount++] = serialPortBuffer[i];
                }

                // error condition: reset the bluetooth buffer if it's now full
                // which means that a 0x7E packet limiter was lost somewhere
                if (bluetoothByteCount >= SERIAL_PORT_RX_BUFFER_SIZE)
                {
                    // reset the bluetooth flags
                    bluetoothInsidePacket = false;
                    bluetoothByteCount = 0;
                    bluetoothDangling0x7D = false;
                    bluetoothPacketErrors++;
                }
            } // end of loop over serial port buffer contents

            return;
        }

        //////////////////////////////////////////////////////////////////
        // Callbacks for Kinetis algorithm selection radio buttons
        /////////////////////////////////////////////////////////////////

        private void radioButtonAcc_CheckedChanged(object sender, EventArgs e)
        {
            // check for a transition to checked and valid open port
            if (radioButtonAcc.Checked && serialPort.IsOpen)
            {
                try
                {
                    // transmit the command via the serial port
                    serialPort.Write(Encoding.ASCII.GetBytes("Q3  "), 0, 4);
                }
                catch
                {
                    // do nothing on timeout error
                    ;
                }
            }

            return;
        }

        private void radioButtonMag_CheckedChanged(object sender, EventArgs e)
        {
            // check for a transition to checked and valid open port
            if (radioButtonMag.Checked && serialPort.IsOpen)
            {
                try
                {
                    // transmit the command via the serial port
                    serialPort.Write(Encoding.ASCII.GetBytes("Q3M "), 0, 4);
                }
                catch
                {
                    // do nothing on timeout error
                    ;
                }
            }

            return;
        }

        private void radioButtonGyro_CheckedChanged(object sender, EventArgs e)
        {
            // check for a transition to checked and valid open port
            if (radioButtonGyro.Checked && serialPort.IsOpen)
            {
                try
                {
                    // transmit the command via the serial port
                    serialPort.Write(Encoding.ASCII.GetBytes("Q3G "), 0, 4);
                }
                catch
                {
                    // do nothing on timeout error
                    ;
                }
            }

            return;
        }

        private void radioButtonAccMag_CheckedChanged(object sender, EventArgs e)
        {
            // check for a transition to checked and valid open port
            if (radioButtonAccMag.Checked && serialPort.IsOpen)
            {
                try
                {
                    // transmit the command via the serial port
                    serialPort.Write(Encoding.ASCII.GetBytes("Q6MA"), 0, 4);
                }
                catch
                {
                    // do nothing on timeout error
                    ;
                }
            }

            return;
        }

        private void radioButtonAccGyro_CheckedChanged(object sender, EventArgs e)
        {
            // check for a transition to checked and valid open port
            if (radioButtonAccGyro.Checked && serialPort.IsOpen)
            {
                try
                {
                    // transmit the command via the serial port
                    serialPort.Write(Encoding.ASCII.GetBytes("Q6AG"), 0, 4);
                }
                catch
                {
                    // do nothing on timeout error
                    ;
                }
            }

            return;
        }

        private void radioButtonAccMagGyro_CheckedChanged(object sender, EventArgs e)
        {
            // check for a transition to checked and valid open port
            if (radioButtonAccMagGyro.Checked && serialPort.IsOpen)
            {
                try
                {
                    // transmit the command via the serial port
                    serialPort.Write(Encoding.ASCII.GetBytes("Q9  "), 0, 4);
                }
                catch
                {
                    // do nothing on timeout error
                    ;
                }
            }

            return;
        }

        //////////////////////////////////////////////////////////////////
        // Callbacks for Auto Detect (detects COM port of Kinetis board)
        /////////////////////////////////////////////////////////////////

        private void buttonAutoDetect_Click(object sender, EventArgs e)
        {
            int i, j;           // loop counters
            bool PortFound;     // local flag to denote a serial port has been found

            // make the progress bar visible
            progressBar.Visible = true;

            // loop a maximum of SERIAL_PORT_MAX_ATTEMPTS attempts to find the serial port
            PortFound = false;
            for (j = 0; (j < SERIAL_PORT_MAX_ATTEMPTS) && !PortFound; j++)
            {
                // loop over all available ports to find the one required
                for (i = 0; (i < comboBoxPort.Items.Count) && !PortFound; i++)
                {
                    // update the progress bar
                    progressBar.Value = (int)(100.0 * (double)(i + j * comboBoxPort.Items.Count) / (double)(comboBoxPort.Items.Count * SERIAL_PORT_MAX_ATTEMPTS));

                    // close the port if it's open
                    if (serialPort.IsOpen)
                    {
                        try
                        {
                            serialPort.Close();
                        }
                        catch (Exception ex)
                        {
                            MessageBox.Show("Error: " + ex.ToString(), "ERROR");
                        }
                    }

                    // assign the new port name (COM45 for example) and try to open it
                    serialPort.PortName = (String)comboBoxPort.Items[i];
                    try
                    {
                        serialPort.Open();
                    }
                    catch
                    {
                        ;
                        // low level bluetooth drivers can cause a semaphone exception
                        // so disable an explicit error message
                        // MessageBox.Show("Error: " + ex.ToString(), "ERROR");
                    }

                    // look for activity on this port
                    if (serialPort.IsOpen)
                    {
                        // clear the input buffer and sleep (units of ms)
                        serialPort.DiscardInBuffer();
                        Thread.Sleep(100);
                        // interpret any bytes read as coming from the correct port
                        if (serialPort.BytesToRead > 0)
                        {
                            // assume this is the required port and set GUI to match
                            comboBoxPort.SelectedIndex = i;
                            ledConnection.OnColor = System.Drawing.Color.DodgerBlue;
                            PortFound = true;
                        }
                    }

                } // end of loop over candidate COM ports
            } // end of outer loop over maximum number of attempts

            // initialize the PC and Kinetis application if a port is found
            if (PortFound)
            {
                ResetKinetisAndPCGUI();
            }
            else
            {
                // make the progress bar disappear
                progressBar.Visible = false;

                // display the Help tab page
                tabControl.SelectedTab = tabPageHelp;

                // display an error message  
                MessageBox.Show("Cannot open a port to the sensor board. \n\n" +
                 "Check that the sensor board is switched on.\n" +
                 "If the sensor board has never been paired with this PC then\n" +
                 "use Control Panel / Devices and Printer / Add a device \n" +
                 "to add the sensor board and then re-start this application.\n\n" +
                 "If the board has been paired then try explicitly setting the\n" +
                 "Port from the menu on the application's Device View page.\n\n" +
                 "More details can be found on the Help page.", "ERROR");
            }

            return;
        }

        //////////////////////////////////////////////////////////////////
        // Callbacks File/Reset (Kinetis and user interface reset)
        /////////////////////////////////////////////////////////////////

        private void ResetKinetisAndPCGUI()
        {
            int i, j;      // loop counters

            // reset GUI settings         
            LEDPacketFlag = LEDRecFlag = false;
            checkBoxRecord.Checked = false;
            ledRecord.Value = false;
            progressBar.Visible = false;
            pictureBoxFitError.Image = ImageNoCal;
            pictureBoxFitErrorMain.Image = ImageNoCal;
            pictureBox.Image = null;
            toolStripMenuItemForDisplayOrientation.Checked = false;
            labelPerspective.Text = "";
            ledActivityAcc.Value = ledActivityAcc.Visible = false;
            ledActivityMag.Value = ledActivityMag.Visible = false;
            ledActivityGyro.Value = ledActivityGyro.Visible = false;
            ledActivityAccMag.Value = ledActivityAccMag.Visible = false;
            ledActivityAccGyro.Value = ledActivityAccGyro.Visible = false;
            ledActivityAccMagGyro.Value = ledActivityAccMagGyro.Visible = false;
            textBoxSensorsAcc_Acc.Visible = textBoxSensorsAcc_Mag.Visible = textBoxSensorsAcc_Gyro.Visible = false;
            textBoxSensorsMag_Acc.Visible = textBoxSensorsMag_Mag.Visible = textBoxSensorsMag_Gyro.Visible = false;
            textBoxSensorsGyro_Acc.Visible = textBoxSensorsGyro_Mag.Visible = textBoxSensorsGyro_Gyro.Visible = false;
            textBoxSensorsAccMag_Acc.Visible = textBoxSensorsAccMag_Mag.Visible = textBoxSensorsAccMag_Gyro.Visible = false;
            textBoxSensorsAccGyro_Acc.Visible = textBoxSensorsAccGyro_Mag.Visible = textBoxSensorsAccGyro_Gyro.Visible = false;
            textBoxSensorsAccMagGyro_Acc.Visible = textBoxSensorsAccMagGyro_Mag.Visible = textBoxSensorsAccMagGyro_Gyro.Visible = false;
            textBoxErrors.Visible = false;
            labelFeedbackMain.Text = "";

            // no valid quaternion received yet
            SyncQuaternionCountdown = 5;
            QReceived = 0x00;

            // reset the fit error, geomagnetic field strength and inclination angle
            FitErrorpc = 1000.0;
            B = 50.0;
            Delta = 0.0;

            // reset the hard and soft iron calibration parameters displayed
            for (i = X; i <= Z; i++)
            {
                for (j = X; j <= Z; j++)
                {
                    invW[i, j] = 0.0;
                }
                invW[i, i] = 1.0;
                V[i] = 0.0;
            }

            // clear the magnetic buffer
            for (i = 0; i < MAGBUFFSIZE; i++)
                MagBuffIndex[i] = -1;

            // reset the alignment quaternion              
            qDisplay.q0 = 1.0;
            qDisplay.q1 = qDisplay.q2 = qDisplay.q3 = 0.0;

            // always reset the remote board
            if (serialPort.IsOpen)
            {
                try
                {
                    serialPort.DiscardOutBuffer();
                    serialPort.Write(Encoding.ASCII.GetBytes("RST "), 0, 4);
                    serialPort.Write(Encoding.ASCII.GetBytes("VG+ "), 0, 4);
                    serialPort.Write(Encoding.ASCII.GetBytes("RPC+"), 0, 4);
                    serialPort.Write(Encoding.ASCII.GetBytes("DB+ "), 0, 4);
                    serialPort.Write(Encoding.ASCII.GetBytes("ALT+"), 0, 4);             
                }
                catch
                {
                    ;
                    // probably a write timeout to invalid port so simply ignore
                }
            }

            return;
        }
        //////////////////////////////////////////////////////////////////
        // Callbacks for File/Reset (resets Kinetis and this application)
        /////////////////////////////////////////////////////////////////

        private void menuItemReset_Click(object sender, EventArgs e)
        {
            // reset remote PCB and GUI
            ResetKinetisAndPCGUI();

            return;
        }

        //////////////////////////////////////////////////////////////////
        // Callbacks for File/Flash (Kinetis board re-flash)
        /////////////////////////////////////////////////////////////////

        private void FlashKinetis(string Filename)
        {
            // display the message box with instructions
            MessageBox.Show("Plug the Kinetis board into your PC using its right hand USB connector (viewed from above) " +
            "except for the FRDM-K64F board where the left hand USB connector should be used.\n" +
            "The Kinetis board should appear to your Windows PC as a removable drive.\n" +
            "Click the OK button to dismiss this message and, when the File/Save dialog appears, select the Kinetis drive " +
            "as destination for the hex file, click Save and then wait approximately 10s for confirmation.",
            "Instructions");

            // initialize the File/Save dialog
            saveFileDialog.FileName = Filename;
            saveFileDialog.Filter = "hex files (*.hex)|*.hex|All files (*.*)|*.*";
            saveFileDialog.FilterIndex = 2;
            saveFileDialog.RestoreDirectory = true;

            // obtain the requested filename             
            if (saveFileDialog.ShowDialog() == DialogResult.OK)
            {
                // OK button: attempt to copy the binary
                try
                {
                    File.Copy(Filename, saveFileDialog.FileName, true);
                    MessageBox.Show("Kinetis board flashed OK.\n" +
                    "Unplug the Kinetis board and power cycle it off and on again.",
                        "SUCCESS");
                }
                catch
                {
                    MessageBox.Show("Error: Kinetis flash program operation failed.\n\n" +
                        "Check that the Kinetis board is plugged in and try again.", "ERROR");
                }
            }

            return;
        }

        private void toolStripMenuItemFRDMKL25ZGaussLeonNEDAerospace_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL25Z-GL-NED_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL25ZGaussLeonAndroid_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL25Z-GL-AND_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL25ZGaussLeonWindows8_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL25Z-GL-WIN8_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL25ZNewtonMaxwellLeonNEDAerospace_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL25Z-NML-NED_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL25ZNewtonMaxwellLeonAndroid_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL25Z-NML-AND_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL25ZNewtonMaxwellLeonWindows8_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL25Z-NML-WIN8_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL26ZGaussLeonNEDAerospace_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL26Z-GL-NED_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL26ZGaussLeonAndroid_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL26Z-GL-AND_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL26ZGaussLeonWindows8_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL26Z-GL-WIN8_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL26ZNewtonMaxwellLeonNEDAerospace_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL26Z-NML-NED_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL26ZNewtonMaxwellLeonAndroid_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL26Z-NML-AND_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL26ZNewtonMaxwellLeonWindows8_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL26Z-NML-WIN8_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMK20D50MGaussLeonNEDAerospace_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-K20D50M-GL-NED_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMK20D50MGaussLeonAndroid_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-K20D50M-GL-AND_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMK20D50MGaussLeonWindows8_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-K20D50M-GL-WIN8_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMK64FGaussLeonNEDAerospace_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-K64F-GL-NED_MQXL.bin");
            return;
        }

        private void toolStripMenuItemFRDMK64FGaussLeonAndroid_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-K64F-GL-AND_MQXL.bin");
            return;
        }

        private void toolStripMenuItemFRDMK64FGaussLeonWindows8_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-K64F-GL-WIN8_MQXL.bin");
            return;
        }

        private void toolStripMenuItemFRDMK64FNewtonMaxwellLeonNEDAerospace_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-K64F-NML-NED_MQXL.bin");
            return;
        }

        private void toolStripMenuItemFRDMK64FNewtonMaxwellLeonAndroid_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-K64F-NML-AND_MQXL.bin");
            return;
        }

        private void toolStripMenuItemFRDMK64FNewtonMaxwellLeonWindows8_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-K64F-NML-WIN8_MQXL.bin");
            return;
        }

        private void toolStripMenuItemFRDMKL46ZGaussLeonNEDAerospace_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL46Z-GL-NED_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL46ZGaussLeonAndroid_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL46Z-GL-AND_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL46ZGaussLeonWindows8_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL46Z-GL-WIN8_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL46ZNewtonMaxwellLeonNEDAerospace_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL46Z-NML-NED_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL46ZNewtonMaxwellLeonAndroid_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL46Z-NML-AND_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL46ZNewtonMaxwellLeonWindows8_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL46Z-NML-WIN8_MQXL.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL46ZVeyronMaxwellNEDAerospace_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL46Z-VM-NED.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL46ZVeyronMaxwellAndroid_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL46Z-VM-AND.hex");
            return;
        }

        private void toolStripMenuItemFRDMKL46ZVeyronMaxwellWindows8_Click(object sender, EventArgs e)
        {
            FlashKinetis("FRDM-KL46Z-VM-WIN8.hex");
            return;
        }

        //////////////////////////////////////////////////////////////////
        // Callbacks for File/Exit (exits this application)
        /////////////////////////////////////////////////////////////////

        private void menuItemExit_Click(object sender, EventArgs e)
        {
            // close the log file stream writer if it exists
            if (logFileStreamWriter != null)
                if (logFileStreamWriter.BaseStream != null)
                {
                    logFileStreamWriter.Flush();
                    logFileStreamWriter.Close();
                }

            // close the magnetic buffer writer if still open                 
            if (magBufferStreamWriter != null)
                if (magBufferStreamWriter.BaseStream != null)
                {
                    magBufferStreamWriter.Flush();
                    magBufferStreamWriter.Close();
                }

            // close the current serial port
            if (serialPort.IsOpen)
            {
                try
                {
                    serialPort.Close();
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Error: " + ex.ToString(), "ERROR");
                }
            }

            // terminate the application
            Application.Exit();
        }

        //////////////////////////////////////////////////////////////////
        // Callbacks for Align/With Display menu command 
        /////////////////////////////////////////////////////////////////

        private void toolStripMenuItemWithDisplay_Click(object sender, EventArgs e)
        {
            // check which transition is to be handled
            if (toolStripMenuItemForDisplayOrientation.Checked)
            {
                // transition to unchecked: remove the tick
                toolStripMenuItemForDisplayOrientation.Checked = false;

                // reset the quaternion for the physical display monitor
                qDisplay.q0 = 1.0;
                qDisplay.q1 = qDisplay.q2 = qDisplay.q3 = 0.0;
            }
            else
            {
                // transition to ticked
                toolStripMenuItemForDisplayOrientation.Checked = true;

                // store the current ENU quaternion assumed aligned with the display
                qDisplay = qRx;
            }

            return;
        }

        //////////////////////////////////////////////////////////////////
        // Callback for About menu command
        /////////////////////////////////////////////////////////////////

        private void toolStripMenuItemAbout_Click(object sender, EventArgs e)
        {
            MessageBox.Show("Freescale Sensor Fusion Toolbox\n\n" +
             "Build date 29 Aug 2014", "About");

            return;
        }

        //////////////////////////////////////////////////////////////////
        // Callbacks for web browsing Back and Home commands
        /////////////////////////////////////////////////////////////////

        private void toolStripMenuItemBack_Click(object sender, EventArgs e)
        {
            // go back to the previous page
            webBrowser.GoBack();

            return;
        }

        private void toolStripMenuItemHome_Click(object sender, EventArgs e)
        {
            // return Home
            string curDir = Directory.GetCurrentDirectory();
            webBrowser.Url = new Uri(String.Format("file:///{0}/help.html", curDir));

            return;
        }
   
    } // end of class Form
} // end of namespace SensorFusion

