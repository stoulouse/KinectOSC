using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect;
using MathNet.Numerics.LinearAlgebra.Double;

namespace KinectOSC
{
    public class LocatedSensor
    {
        public KinectSensor sensor { get; set; }
        public float xOffset { get; set; }
        public float yOffset { get; set; }
        public float zOffset { get; set; }
        public DenseMatrix rotationMatrix { get; set; }
        private double pitchAngle { get; set; }
        private double yawAngle { get; set; }
        private double rollAngle { get; set; }
        private DenseMatrix rotationMatrixPitch { get; set; }
        private DenseMatrix rotationMatrixYaw { get; set; }
        private DenseMatrix rotationMatrixRoll { get; set; }
        
        /// <summary>
        /// A List of skeletons, with joint positions in relative orientation to the sensor
        /// </summary>
        public List<Skeleton> relativeSkeletons;
        /// <summary>
        /// A List of skeletons, with joint positions in a global orientation
        /// </summary>
        public List<Skeleton> globalSkeletons;

        public LocatedSensor() { }

        /// <summary>
        /// Create a new sensor with location and orientation parameters
        /// </summary>
        /// <param name="sensor">A given Kinect sensor</param>
        /// <param name="x">X, in global coordinates</param>
        /// <param name="y">Y, in global coordinates</param>
        /// <param name="z">Z, in global coordinates</param>
        /// <param name="theta">Rotation around the vertical axis, </param>
        public LocatedSensor(KinectSensor sensor, float x, float y, float z,
            // Was going 
            //double rotationVectorX, double rotationVectorY, double rotationVectorZ,
                              double thetaPitch, double thetaYaw, double thetaRoll) {
            this.sensor = sensor;
            this.xOffset = x;
            this.yOffset = y;
            this.zOffset = z;

            // Set up the rotation matrix
            /*        this.rotationMatrix = new DenseMatrix(4, 4);
                    double u = rotationVectorX;
                    double u2 = Math.Pow(u, 2);
                    double v = rotationVectorY;
                    double v2 = Math.Pow(v, 2);
                    double w = rotationVectorZ;
                    double w2 = Math.Pow(w, 2);
                    double thta = theta * Math.PI / 180; // Converted to radians
                    rotationMatrix[0, 0] = u2 + (1 - u2) * Math.Cos(thta);
                    rotationMatrix[0, 1] = u * v * (1 - Math.Cos(thta)) - w * Math.Sin(thta);
                    rotationMatrix[0, 2] = u * w * (1 - Math.Cos(thta)) - v * Math.Sin(thta);
                    rotationMatrix[0, 3] = 0;
                    rotationMatrix[1, 0] = u * v * (1 - Math.Cos(thta)) - w * Math.Sin(thta);
                    rotationMatrix[1, 1] = v2 + (1 - v2) * Math.Cos(thta);
                    rotationMatrix[1, 2] = v * w * (1 - Math.Cos(thta)) - u * Math.Sin(thta);
                    rotationMatrix[1, 3] = 0;
                    rotationMatrix[2, 0] = u * w * (1 - Math.Cos(thta)) - v * Math.Sin(thta);
                    rotationMatrix[2, 2] = v * w * (1 - Math.Cos(thta)) - u * Math.Sin(thta);
                    rotationMatrix[2, 3] = w2 + (1 - w2) * Math.Cos(thta);
                    rotationMatrix[2, 3] = 0;
                    rotationMatrix[3, 0] = 0;
                    rotationMatrix[3, 1] = 0;
                    rotationMatrix[3, 2] = 0;
                    rotationMatrix[3, 3] = 1;
              */
            setRotationMatrix(thetaPitch, thetaRoll, thetaYaw);

            this.relativeSkeletons = new List<Skeleton>();
            this.globalSkeletons = new List<Skeleton>();

            //Register an event to update the internal skeleton lists when we get fresh skeleton data
            sensor.SkeletonFrameReady += this.refreshSkeletonPositions;
        }

        public void setPitch(double angle)
        {
            this.pitchAngle = angle;
            setRotationMatrix(pitchAngle, rollAngle, yawAngle);
        }
        public void setRoll(double angle)
        {
            this.rollAngle = angle;
            setRotationMatrix(pitchAngle, rollAngle, yawAngle);
        }
        public void setYaw(double angle)
        {
            this.yawAngle = angle;
            setRotationMatrix(pitchAngle, rollAngle, yawAngle);
        }


        private void setRotationMatrix(double thetaPitch, double thetaRoll, double thetaYaw)
        {

            thetaPitch = thetaPitch * Math.PI / 180; // Converted to radians
            thetaYaw = thetaYaw * Math.PI / 180;
            thetaRoll = thetaRoll * Math.PI / 180;

            //
            rotationMatrixRoll = new DenseMatrix(3, 3);
            rotationMatrixRoll[0, 0] = Math.Cos(thetaRoll);  rotationMatrixRoll[0, 1] = -Math.Sin(thetaRoll); rotationMatrixRoll[0, 2] = 0;
            rotationMatrixRoll[1, 0] = Math.Sin(thetaRoll); rotationMatrixRoll[1, 1] = Math.Cos(thetaRoll); rotationMatrixRoll[1, 2] = 0;
            rotationMatrixRoll[2, 0] = 0;                    rotationMatrixRoll[2, 1] = 0;                   rotationMatrixRoll[2, 2] = 1;

            rotationMatrixYaw = new DenseMatrix(3, 3);
            rotationMatrixYaw[0, 0] = Math.Cos(thetaYaw); rotationMatrixYaw[0, 1] = 0; rotationMatrixYaw[0, 2] = Math.Sin(thetaYaw);
            rotationMatrixYaw[1, 0] = 0; rotationMatrixYaw[1, 1] = 1; rotationMatrixYaw[1, 2] = 0;
            rotationMatrixYaw[2, 0] = -Math.Sin(thetaYaw); rotationMatrixYaw[2, 1] = 0; rotationMatrixYaw[2, 2] = Math.Cos(thetaYaw);

            rotationMatrixPitch = new DenseMatrix(3, 3);
            rotationMatrixPitch[0, 0] = 1;   rotationMatrixPitch[0, 1] = 0;                      rotationMatrixPitch[0, 2] = 0;
            rotationMatrixPitch[1, 0] = 0;   rotationMatrixPitch[1, 1] = Math.Cos(thetaPitch);   rotationMatrixPitch[1, 2] = -Math.Sin(thetaPitch);
            rotationMatrixPitch[2, 0] = 0;   rotationMatrixPitch[2, 1] = Math.Sin(thetaPitch);   rotationMatrixPitch[2, 2] = Math.Cos(thetaPitch);

            rotationMatrix = rotationMatrixYaw;
            rotationMatrix = (DenseMatrix)rotationMatrix.Multiply(rotationMatrixPitch);
            rotationMatrix = (DenseMatrix)rotationMatrix.Multiply(rotationMatrixRoll);
        }

        /// <summary>
        /// SkeletonFrameReady gets fired every skeleton frame update, and refreshes the LocatedSensor's
        ///  global and relative skeleton maps
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void refreshSkeletonPositions(object sender, SkeletonFrameReadyEventArgs e) {
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame()) {
                if (skeletonFrame != null) {
                    // First, get the relative skeletons - easy peasy
                    Skeleton[] skeletonsR = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletonsR);
                    this.relativeSkeletons = skeletonsR.ToList<Skeleton>();

                    // Now global skeletons...
                    // First, clear our global skeletons list.
                    //  We'll be building this back up from scratch here
                    this.globalSkeletons.Clear();
                    // Next, iterate through all the skeletons, applying a rotation and translation
                    //  to get us into global coordinates
                    foreach (Skeleton skel in this.relativeSkeletons) {
                        // Add a temporary skeleton object to store transformed
                        //  data into
                        Skeleton tempSkel = skel;

                        foreach (Joint j in skel.Joints) {
                            // Make a new joint, then put it into our temporary joint
                            //  collection
                            JointType type = j.JointType;
                            Joint tempJoint = tempSkel.Joints[type];
                            // Copy the current joint state
                            JointTrackingState tracking = j.TrackingState;
                            tempJoint.TrackingState = tracking;

                            // However, we transform the position of the joint at least
                            SkeletonPoint shiftedPoint = new SkeletonPoint();
                            // Rotate the points
                            DenseMatrix point = new DenseMatrix(1, 3);
                            point[0, 0] = j.Position.X;
                            point[0, 1] = j.Position.Y;
                            point[0, 2] = j.Position.Z;
                            var rotatedPoint = point.Multiply(this.rotationMatrix);

                            // Then shift them by the global coordinates.
                            shiftedPoint.X = (float)rotatedPoint[0, 0] + this.xOffset;
                            shiftedPoint.Y = (float)rotatedPoint[0, 1] + this.yOffset;
                            shiftedPoint.Z = (float)rotatedPoint[0, 2] + this.zOffset;
                            tempJoint.Position = shiftedPoint;

                            tempSkel.Joints[type] = tempJoint;
                        }
                        // Next, alter the higher-level parameters of our skeleton
                        SkeletonPoint shiftedPosition = new SkeletonPoint();
                        // Rotate
                        DenseMatrix p = new DenseMatrix(1, 3);
                        p[0, 0] = tempSkel.Position.X;
                        p[0, 1] = tempSkel.Position.Y;
                        p[0, 2] = tempSkel.Position.Z;
                        var rPoint = p.Multiply(this.rotationMatrixPitch);

                        // Then shift them by the global coordinates.
                        shiftedPosition.X = (float)rPoint[0, 0] + this.xOffset;
                        shiftedPosition.Y = (float)rPoint[0, 1] + this.yOffset;
                        shiftedPosition.Z = (float)rPoint[0, 2] + this.zOffset;

                        tempSkel.Position = shiftedPosition;

                        // Now add that skeleton to our global skeleton list
                        this.globalSkeletons.Add(tempSkel);
                    }
                }
            }
        }
    }
}
