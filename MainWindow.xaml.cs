/*
 * KinectOSC
 * Copyright 2013 - Alan Chatham
 * Extended from the work of Andre Hayter
 * 
 * This code reads in multiple Kinect sensors, and fuses the skeleton data
 *  into a unified list of all skeletons the sensors see, reported in a global
 *  coordinate frame
 *
 *  This code is released to the public under the GNU Public License, version 3
 *  The full licence can be read in the included gpl-3.0.txt file, or at
 *  http://www.gnu.org/licenses/gpl-3.0.txt
 */
using System.Collections.Generic;
using System.Linq;
using System.Windows.Input;
using System.Security.Cryptography;
using System.Text;

namespace KinectOSC
{
    using Ventuz.OSC;
    using System;
    using System.Globalization;
    using System.IO;
    using System.Net;
    using System.Threading;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using MathNet.Numerics.LinearAlgebra.Double;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Global Settings

        float[] kinectXPositions = { 0.0f, 7.88f, 5.6f, 3.28f, 1.0f }; // X is positive left, if looking at the screen
        float[] kinectYPositions = { 0, 0, 0, 0 }; // Y is positive up
        float[] kinectZPositions = { 0, 0, 0, 0 }; // Z is positive towards the screen, so offsets will usually be positive
        float[] kinectAngles = { 00, 0, 0, 0 };

       
        /// <summary>
        /// Store the OSC On checkbox value for quick access
        /// </summary>
        private bool oscOn = true;

        /// <summary>
        /// Store the Show OSC Data checkbox value for quick access
        /// </summary>
        private bool showOscData = false;

        /// <summary>
        /// Flag to choose to send data specifically in a format that Animata will appreciate
        /// </summary>
        private bool sendAnimataData = true;

        /// <summary>
        /// If this is true then each variable of each of the joints will be sent separately (each osc element will have one float)
        /// </summary>
        private string oscAddress = "";

        private List<VisualKinectUnit> visualKinectUnitList;

        private VisualKinectGroup kinectGroup;

        // OSC 
        private static UdpWriter oscWriter;
        private static string[] oscArgs = new string[2];

        private static UdpWriter deltaToscWriter;

        #endregion

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
        }


        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e) {
            // Setup osc sender
            oscArgs[0] = OscAddress.Text;
            oscArgs[1] = OscPort.Text;
            oscWriter = new UdpWriter(oscArgs[0], Convert.ToInt32(oscArgs[1]));

            deltaToscWriter = new UdpWriter(oscArgs[0], 7114);
            // Initialize Data viewer
            oscViewer.Text = "\nData will be shown here\nwhen there is a skeleton\nbeing tracked.";  

            kinectGroup = new VisualKinectGroup();
            AngleRadius.Text = kinectGroup.sameSkeletonAngle.ToString();

            KinectViewport[] viewports = { TestViewport0, TestViewport1, TestViewport2, TestViewport3 };

            string leftKinect = "A00365";
            string centerKinect = "A00363A";
            string rightKinect = "A003638";

            ThreadPool.SetMaxThreads(1, 1);

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit
            int numberOfKinects = 0;
            foreach (var potentialSensor in KinectSensor.KinectSensors) {
                if (numberOfKinects < viewports.Length)
                {
                    if (potentialSensor.Status == KinectStatus.Connected)
                    {
                        // Start the sensor!
                        try
                        {
                            potentialSensor.Start();
                            
                            // Good to go, so count this one as connected!
                            // So let's set up some environment for this...

                            //  LocatedSensor sensor = new LocatedSensor(potentialSensor, kinectXPositions[numberOfKinects],
                            //                                                            kinectYPositions[numberOfKinects],
                            //                                                           kinectZPositions[numberOfKinects],
                            //                                                         kinectAngles[numberOfKinects]);
                            LocatedSensor sensor = new LocatedSensor(potentialSensor, 0, 0, 0, 0, 0, 0);

                            int viewportIdx = numberOfKinects;
                            if (sensor.sensor.UniqueKinectId.Contains(leftKinect))
                            {
                                viewportIdx = 0;
                            }
                            else if (sensor.sensor.UniqueKinectId.Contains(centerKinect))
                            {
                                viewportIdx = 1;
                            }
                            else if (sensor.sensor.UniqueKinectId.Contains(rightKinect))
                            {
                                viewportIdx = 2;
                            }
                            VisualKinectUnit newSensor = new VisualKinectUnit(sensor, viewports[viewportIdx].skeletonDrawingImage, viewports[viewportIdx].colorImage, viewports[viewportIdx]);
                            kinectGroup.AddVisualKinectUnit(newSensor);

                            // This function sends out skeleton data as OSC
                            newSensor.locatedSensor.sensor.SkeletonFrameReady += sendOSCAsAnimataData;


                            numberOfKinects++;
                            Console.WriteLine("Number of Kinects : " + numberOfKinects);
                        }
                        catch (IOException)
                        {
                            Console.WriteLine("Couldn't start one of the Kinect sensors...");
                        }
                    }
                }
            }

            sendOscTimer = new Timer(sendOscCallback, null, 0, 30);
        }

        private readonly object sendOscSyncLock = new object();
        private Timer sendOscTimer;
        void sendOscCallback(object state)
        {
            return;

            lock (sendOscSyncLock)
            {
                Dictionary<String, Skeleton> d = new Dictionary<String, Skeleton>();
                foreach (KeyValuePair<KinectSensor, List<Skeleton>> skels in foundSkeletons)
                {
                    foreach (Skeleton skel in skels.Value)
                    {
                        double x = skel.Joints[JointType.Spine].Position.X - skel.Joints[JointType.HipCenter].Position.X;
                        double y = skel.Joints[JointType.Spine].Position.Y - skel.Joints[JointType.HipCenter].Position.Y;
                        double z = skel.Joints[JointType.Spine].Position.Z - skel.Joints[JointType.HipCenter].Position.Z;

                        double dist = (Math.Sqrt(x * x + y * y + z * z));

                        string skelId = string.Format("{0:0.00}", dist);
                        d[skelId] = skel;
                    }
                }

                oscViewer.Dispatcher.BeginInvoke((Action)(() => 
                {
                    oscViewer.Text = "skeleton count: " + d.Count.ToString() + "\n";
                    foreach (KeyValuePair<String, Skeleton> skel in d)
                    {
                        sendOneOSCAnimataSkeleton(skel.Value, skel.Value.TrackingId.ToString(), skel.Key);
                    }
                }));

                foundSkeletons.Clear();
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            foreach (VisualKinectUnit unit in kinectGroup.visualKinectUnits){
                unit.Stop();
            }
        }

        private int sendOSC(int counter, Skeleton skel) {
            var elements = new List<OscElement>();
            var oscText = "";
            if (sendAnimataData){
                //oscText = sendOSCHeadOnly(counter, skel, oscText);
            }
            
            if (showOscData) {
                oscViewer.Text = oscText;
            }
            counter++;
            return counter;
        }


        Dictionary<KinectSensor, List<Skeleton>> foundSkeletons = new Dictionary<KinectSensor, List<Skeleton>>();

        List<int> knownSkeletons = new List<int>();
        Dictionary<String, String> knownSkeletonsKinect = new Dictionary<String, String>();


        string toMd5(string password)
        {
            // byte array representation of that string
            byte[] encodedPassword = new UTF8Encoding().GetBytes(password);

            // need MD5 to calculate the hash
            byte[] hash = ((HashAlgorithm)CryptoConfig.CreateFromName("MD5")).ComputeHash(encodedPassword);

            // string representation (similar to UNIX format)
            string encoded = BitConverter.ToString(hash)
                // without dashes
               .Replace("-", string.Empty)
                // make lowercase
               .ToLower();

            // encoded contains the hash you are wanting
            return encoded;
        }

        // This function will send out an entire skeleton data
        private void sendOSCAsAnimataData(object sender, SkeletonFrameReadyEventArgs e) {
            lock (sendOscSyncLock)
            {
                if (kinectGroup != null)
                {
                    skeletonFoundLabel.Content = "Skeleton found: 0";
                    oscViewer.Text = "skeleton count: 0";

                    List<Skeleton> skeletons = kinectGroup.getGlobalSkeletons();
                    if (skeletons.Count > 0)
                    {
                        skeletonFoundLabel.Content = "Skeleton found: " + skeletons.Count.ToString();
                        oscViewer.Text = "skeleton count: " + skeletons.Count.ToString() + "\n";
                        foreach (Skeleton skel in skeletons)
                        {
                            sendOneOSCAnimataSkeleton(skel, skel.TrackingId.ToString(), null);
                        }
                    }


                    foreach (VisualKinectUnit unit in kinectGroup.visualKinectUnits)
                    {
                        foreach (Skeleton skel in unit.locatedSensor.globalSkeletons)
                        {
                            if (skel.TrackingState == SkeletonTrackingState.Tracked)
                                sendOneOSCAnimataSkeleton(skel, skel.TrackingId.ToString(), unit.locatedSensor.sensor.UniqueKinectId);
                        }
                    }

                    List<int> toRemove = new List<int>();
                    foreach (int i in knownSkeletons)
                    {
                        bool found = false;
                        foreach (VisualKinectUnit unit in kinectGroup.visualKinectUnits)
                        {
                            foreach (Skeleton skel in unit.locatedSensor.globalSkeletons)
                            {
                                if (skel.TrackingId == i)
                                {
                                    found = true;
                                    break;
                                }
                            }
                        }
                        if (!found)
                        {
                            var jointElement = new List<OscElement>();
                            OscElement elem;
                            if (knownSkeletonsKinect.ContainsKey(i.ToString()))
                            {
                                elem = new OscElement("/enduserWithKinect", toMd5(knownSkeletonsKinect[i.ToString()]), i.ToString());
                            }
                            else
                            {
                                elem = new OscElement("/enduser", i.ToString());
                            }
                            
                            jointElement.Add(elem);
                            oscWriter.Send(new OscBundle(DateTime.Now, jointElement.ToArray()));
                            toRemove.Add(i);

                            Console.WriteLine(elem.ToString());
                        }
                    }
                    foreach (int i in toRemove)
                    {
                        knownSkeletonsKinect.Remove(i.ToString());
                        knownSkeletons.Remove(i);
                    }
                }
            }
        }

        // Measurements for one screen
        static int SCREEN_WIDTH_PX = 640;
        static float SCREEN_WIDTH_METERS = 1.89f;
        static int SCREEN_HEIGHT_PX = 720;
        static float SCREEN_HEIGHT_METERS = 1.52f;

        static float SCREEN_WIDTH_PX_M_RATIO = SCREEN_WIDTH_PX / SCREEN_WIDTH_METERS;
        static float SCREEN_HEIGHT_PX_M_RATIO = SCREEN_HEIGHT_PX / SCREEN_HEIGHT_METERS;

        // Send out one skeleton data via OSC in Animata-friendly format
        private void sendOneOSCAnimataSkeleton(Skeleton skel, String skeletonName, String kinectName)
        {
            double playerHeight = skeletonHeight(skel);
            // joints bundled individually as 2 floats (x, y)
            string oscText = "";
            var jointElement = new List<OscElement>();

            double x = skel.Joints[JointType.Spine].Position.X - skel.Joints[JointType.HipCenter].Position.X;
            double y = skel.Joints[JointType.Spine].Position.Y - skel.Joints[JointType.HipCenter].Position.Y;
            double z = skel.Joints[JointType.Spine].Position.Z - skel.Joints[JointType.HipCenter].Position.Z;

            double dist = ( Math.Sqrt(x * x + y * y + z * z));


            if (knownSkeletons.Contains(skel.TrackingId) == false)
            {
                knownSkeletons.Add(skel.TrackingId);

                OscElement elem;
                if (kinectName == null)
                {
                    elem = new OscElement("/newuser", skeletonName);
                }
                else
                {
                    elem = new OscElement("/newuserWithKinect", toMd5(kinectName), skeletonName);
                    knownSkeletonsKinect.Add(skeletonName, kinectName);
                }
                jointElement.Add(elem);
                if (showOscData)
                {
                    oscText += elem.ToString();
                }
                Console.WriteLine(elem.ToString());
            }
            
            if (knownSkeletons.Contains(skel.TrackingId) && knownSkeletonsKinect.ContainsKey(skeletonName) == false && kinectName != null)
            {
                OscElement elem = new OscElement("/newuserWithKinect", toMd5(kinectName), skeletonName);
                knownSkeletonsKinect.Add(skeletonName, kinectName);

                jointElement.Add(elem);
                if (showOscData)
                {
                    oscText += elem.ToString();
                }
                Console.WriteLine(elem.ToString());
            }
            


            foreach (Joint joint in skel.Joints)
            {
                OscElement elem = null;
                if (kinectName == null)
                {
                    elem = new OscElement(
                                         "/joint", skeletonName, joint.JointType.ToString(),
                                         joint.Position.X, joint.Position.Y, joint.Position.Z);
                }
                else
                {
                    elem = new OscElement(
                                        "/jointWithKinect", toMd5(kinectName), skeletonName, joint.JointType.ToString(),
                                        joint.Position.X, joint.Position.Y, joint.Position.Z);
                }
                jointElement.Add(elem);
                if (showOscData)
                {
                    oscText += elem.ToString();
                }
            }

            ThreadPool.QueueUserWorkItem(_ =>
            {
                oscWriter.Send(new OscBundle(DateTime.Now, jointElement.ToArray()));
            }
            );

            
            if (showOscData)
            {
                oscViewer.Text +=  oscText;
            }
            return;
        }

        private float sanitizeTextToFloat(string input) {
            float output = 1.0f;
            if (!float.TryParse(input, out output)){
                    output  = 1.0f;
                }
            return output;
        }

/*        private void sendOSCHeadOnly(object sender, SkeletonFrameReadyEventArgs e)
        {
            // If there isn't a skeleton we want to send, we don't send anything
            if (leadSkeletonIDs.Count == 0)
                return;
            Skeleton headTrackedSkeleton = new Skeleton();
            headTrackedSkeleton = null;
            // Get the skeleton we want to send
            foreach (Skeleton s in masterSkeletonList) {
                if (s.TrackingId == leadSkeletonIDs[0]) {
                    headTrackedSkeleton = s;
                }
            }
            // Just in case, if we didn't find a skeleton, get out of here
            if (headTrackedSkeleton == null) {
                Console.WriteLine("Had skeletons in our leadSkeletonIDs list, but couldn't find the appropriate skeleton... How odd.");
                return;
            }
            double playerHeight = skeletonHeight(headTrackedSkeleton);
            // joints bundled individually as 2 floats (x, y)
            Joint headJoint = headTrackedSkeleton.Joints[JointType.Head];

           var jointElement = new List<OscElement>();
                
            // Joint positions are returned in meters, so we'll assume a 2 meter tall person
            // and scale that range to pixels for the animation

           var jointX = headJoint.Position.X;
           var jointY = headJoint.Position.Y;
           var jointZ = headJoint.Position.Z;
            jointElement.Add(new OscElement(
                                    "/head",
                                    (float)Math.Round(jointX, 4), (float)Math.Round(jointY, 4),
                                    (float)Math.Round(jointZ, 4)));
            oscWriter.Send(new OscBundle(DateTime.Now, jointElement.ToArray()));
            deltaToscWriter.Send(new OscBundle(DateTime.Now, jointElement.ToArray()));
                
            if (showOscData)
            {
                string oscText = "\n\n/head " +
                           (float)Math.Round(jointX, 2) + " " +
                           (float)Math.Round(jointY, 2) + " " +
                                    (float)Math.Round(jointZ, 2);
                oscViewer.Text =  oscText;
            }

            // If there isn't a skeleton we want to send, we don't send anything
            if (prunedSkeletonList.Count == 0)
                return;
            int i = 0;
            foreach (Skeleton skel in prunedSkeletonList) {
                if (skel.TrackingId != headTrackedSkeleton.TrackingId) {
                    sendOneOSCHand(skel.Joints[JointType.HandLeft], i);
                    i++;
                    sendOneOSCHand(skel.Joints[JointType.HandRight], i);
                    i++;
                }
            }
        }

        private void sendOSCHands(object sender, SkeletonFrameReadyEventArgs e) {
            // If there isn't a skeleton we want to send, we don't send anything
            if (prunedSkeletonList.Count == 0)
                return;
            int i = 0;
            foreach (Skeleton skel in prunedSkeletonList) {

                sendOneOSCHand(skel.Joints[JointType.HandLeft],i);
                i++;
                sendOneOSCHand(skel.Joints[JointType.HandRight], i);
                i++;
            }
        }
        */
        private void sendOneOSCHand(Joint joint, int i) {
            var jointElement = new List<OscElement>();
            var jointX = joint.Position.X;
            var jointY = joint.Position.Y;
            var jointZ = joint.Position.Z;
            jointElement.Add(new OscElement(
                                    "/hand",
                                    (float)Math.Round(jointX, 4), (float)Math.Round(jointY, 4),
                                    (float)Math.Round(jointZ, 4), i));
            oscWriter.Send(new OscBundle(DateTime.Now, jointElement.ToArray()));

            if (showOscData) {
                string oscText = "\n\n/hand " +
                            (float)Math.Round(jointX, 2) + " " +
                            (float)Math.Round(jointY, 2) + " " +
                                    (float)Math.Round(jointZ, 2);
                oscViewer.Text = oscText;
            }
        }
        /*
        private void sendOSCForearms(object sender, SkeletonFrameReadyEventArgs e) {
            // If there isn't a skeleton we want to send, we don't send anything
            if (prunedSkeletonList.Count == 0)
                return;
            int i = 0;
            foreach (Skeleton skel in prunedSkeletonList) {

                sendOneOSCForearm(skel.Joints[JointType.HandLeft], skel.Joints[JointType.ElbowLeft],i);
                i++;
                sendOneOSCForearm(skel.Joints[JointType.HandRight], skel.Joints[JointType.ElbowRight], i);
                i++;
            }
        }
        */
        private void sendOneOSCForearm(Joint startJoint, Joint endJoint, int i) {
            var jointElement = new List<OscElement>();
           jointElement.Add(new OscElement(
                                    "/forearm", 
                                    i,
                                    (float)Math.Round(startJoint.Position.X, 4),
                                    (float)Math.Round(startJoint.Position.Y, 4),
                                    (float)Math.Round(startJoint.Position.Z, 4), 
                                    (float)Math.Round(endJoint.Position.X, 4),
                                    (float)Math.Round(endJoint.Position.Y, 4),
                                    (float)Math.Round(endJoint.Position.Z, 4)
                                    ));
            deltaToscWriter.Send(new OscBundle(DateTime.Now, jointElement.ToArray()));

            if (showOscData) {
                string oscText = "\n\n/hand " + i + " " +
                                    (float)Math.Round(startJoint.Position.X, 2) + " " +
                                    (float)Math.Round(startJoint.Position.Y, 2) + " " +
                                    (float)Math.Round(startJoint.Position.Z, 2) + "\n" +
                                    (float)Math.Round(endJoint.Position.X, 2) + " " +
                                    (float)Math.Round(endJoint.Position.Y, 2) + " " +
                                    (float)Math.Round(endJoint.Position.Z, 2);
                oscViewer.Text = oscText;
            }
        }




        private String GenerateOscDataDump(int counter, string jointName, SkeletonPoint jointPoint)
        {
            var dataDump = "";
            if (oscAddress != "")
            {
                dataDump += oscAddress + (counter - 3) + "/" + Math.Round(jointPoint.X, 4) + "\n";
                dataDump += oscAddress + (counter - 2) + "/" + Math.Round(jointPoint.Y, 4) + "\n";
                dataDump += oscAddress + (counter - 1) + "/" + Math.Round(jointPoint.Z, 4) + "\n";
            }
            else
            {
                dataDump += "/skeleton" + counter + "/" + jointName + "/x" + Math.Round(jointPoint.X, 4) + "\n";
                dataDump += "/skeleton" + counter + "/" + jointName + "/y" + Math.Round(jointPoint.Y, 4) + "\n";
                dataDump += "/skeleton" + counter + "/" + jointName + "/z" + Math.Round(jointPoint.Z, 4) + "\n";
            }
            return dataDump;
        }

        private String GenerateOscDataDump(int counter, string jointName, SkeletonPoint jointPoint, Point jointPosition)
        {
            var dataDump = "";
            if (oscAddress != "")
            {
                dataDump += oscAddress + (counter - 3) + "/" + Math.Round(jointPosition.X, 3) + "\n";
                dataDump += oscAddress + (counter - 2) + "/" + Math.Round(jointPosition.Y, 3) + "\n";
                dataDump += oscAddress + (counter - 1) + "/" + Math.Round(jointPoint.Z, 3) + "\n";
            }
            else
            {
                dataDump += "/skeleton" + counter + "/" + jointName + "/x" + Math.Round(jointPosition.X, 3) + "\n";
                dataDump += "/skeleton" + counter + "/" + jointName + "/y" + Math.Round(jointPosition.Y, 3) + "\n";
                dataDump += "/skeleton" + counter + "/" + jointName + "/z" + Math.Round(jointPoint.Z, 3) + "\n";
            }
            return dataDump;
        }


        public static long GetTimestamp()
        {
            long ticks = DateTime.UtcNow.Ticks - DateTime.Parse("01/01/1970 00:00:00").Ticks;
            ticks /= 10000;
            return ticks;
        }

        private void CheckBoxOscOnChanged(object sender, RoutedEventArgs e)
        {
            oscOn = this.checkBoxOscOn.IsChecked.GetValueOrDefault();
        }
        private void CheckBoxShowOscDataChanged(object sender, RoutedEventArgs e)
        {
            showOscData = this.checkBoxShowOscData.IsChecked.GetValueOrDefault();
            if (showOscData)
            {
                oscViewer.Visibility = Visibility.Visible;
                CloseOscViewer.Visibility = Visibility.Visible;
            }
            else
            {
                oscViewer.Visibility = Visibility.Collapsed;
                CloseOscViewer.Visibility = Visibility.Collapsed;
            }
        }

        private void ChangePortClicked(object sender, RoutedEventArgs e)
        {
            oscAddress = OscAddress.Text;
            oscArgs[0] = OscAddress.Text;
            oscArgs[1] = OscPort.Text;
            oscWriter = new UdpWriter(oscAddress, Convert.ToInt32(OscPort.Text));
        }

        private void CloseOscViewerClicked(object sender, RoutedEventArgs e)
        {
            oscViewer.Visibility = Visibility.Collapsed;
            CloseOscViewer.Visibility = Visibility.Collapsed;
            checkBoxShowOscData.IsChecked = false;
        }

        private void ChangeAddressClicked(object sender, RoutedEventArgs e)
        {
            UpdateOscAddress();
        }

        private void OscAddressKeyUp(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                UpdateOscAddress();
            }
        }

        private void UpdateOscAddress()
        {
            oscAddress = OscAddress.Text;
            if (!String.IsNullOrEmpty(oscAddress))
            {
                OscAddress.Text = oscAddress;
                ChangeAddress.Focus();
            }

            oscArgs[0] = OscAddress.Text;
            oscArgs[1] = OscPort.Text;
            oscWriter = new UdpWriter(oscAddress, Convert.ToInt32(OscPort.Text));

        }

        private void OscPortKeyUp(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                oscArgs[1] = OscPort.Text;
                oscWriter = new UdpWriter(oscArgs[0], Convert.ToInt32(oscArgs[1]));
            }
        }

        private void CheckBoxSendAnimataData(object sender, RoutedEventArgs e)
        {
            sendAnimataData = this.checkBoxSendAnimataData.IsChecked.GetValueOrDefault();
        }



        private double Length(Joint p1, Joint p2)
        {
            return Math.Sqrt(
                Math.Pow(p1.Position.X - p2.Position.X, 2) +
                Math.Pow(p1.Position.Y - p2.Position.Y, 2) +
                Math.Pow(p1.Position.Z - p2.Position.Z, 2));
        }

        private double Length(params Joint[] joints)
        {
            double length = 0;
            for (int index = 0; index < joints.Length - 1; index++) {
                length += Length(joints[index], joints[index + 1]);
            }
            return length;
        }

        private int NumberOfTrackedJoints(params Joint[] joints)
        {
            int trackedJoints = 0;
            foreach (var joint in joints)
            {
                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    trackedJoints++;
                }
            }
            return trackedJoints;
        }

        private double skeletonHeight(Skeleton skeleton)
        {
            const double HEAD_DIVERGENCE = 0.1;

            var head = skeleton.Joints[JointType.Head];
            var neck = skeleton.Joints[JointType.ShoulderCenter];
            var spine = skeleton.Joints[JointType.Spine];
            var waist = skeleton.Joints[JointType.HipCenter];
            var hipLeft = skeleton.Joints[JointType.HipLeft];
            var hipRight = skeleton.Joints[JointType.HipRight];
            var kneeLeft = skeleton.Joints[JointType.KneeLeft];
            var kneeRight = skeleton.Joints[JointType.KneeRight];
            var ankleLeft = skeleton.Joints[JointType.AnkleLeft];
            var ankleRight = skeleton.Joints[JointType.AnkleRight];
            var footLeft = skeleton.Joints[JointType.FootLeft];
            var footRight = skeleton.Joints[JointType.FootRight];

            // Find which leg is tracked more accurately.
            int legLeftTrackedJoints = NumberOfTrackedJoints(hipLeft, kneeLeft, ankleLeft, footLeft);
            int legRightTrackedJoints = NumberOfTrackedJoints(hipRight, kneeRight, ankleRight, footRight);
            double legLength = legLeftTrackedJoints > legRightTrackedJoints ? Length(hipLeft, kneeLeft, ankleLeft, footLeft) : Length(hipRight, kneeRight, ankleRight, footRight);

            return Length(head, neck, spine, waist) + legLength + HEAD_DIVERGENCE;
        }

        private void AngleRadius_TextChanged(object sender, System.Windows.Controls.TextChangedEventArgs e)
        {
            double o;
            if (double.TryParse( AngleRadius.Text, out o))
            {
                kinectGroup.sameSkeletonAngle = o;
            }
        }
    }
}