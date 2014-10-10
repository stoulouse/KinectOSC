using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Microsoft.Kinect;

namespace KinectOSC
{
    /// <summary>
    /// VisualKinectGroups hold one or more VisualKinectUnits, and provide combined skeleton
    ///  tracking, unifying all the skeleton data that the VisualKinectUnits provide
    ///  in one global coordinate frame
    /// </summary>
    class VisualKinectGroup
    {
        public List<VisualKinectUnit> visualKinectUnits;

        /// <summary>
        /// This list holds all the skeletons seen by all sensors,
        ///  with position data in the global coordinate frame
        /// We trust that there won't be a conflict between random
        ///  IDs assigned by kinect sensors
        /// </summary>
        public List<Skeleton> masterSkeletonList;
        /// <summary>
        /// Skeletons within this radius of each other will be assumed
        ///  to be the same skeleton
        /// </summary>
        private float sameSkeletonRadius = 1.0f;

        private List<Skeleton> prunedSkeletonList;

        private List<int> leadSkeletonIDs;

        /// <summary>
        /// Default Constructor
        /// </summary>
        public VisualKinectGroup()
        {
            //Initialize some variables, woo!
            visualKinectUnits = new List<VisualKinectUnit>();
            masterSkeletonList = new List<Skeleton>();
            leadSkeletonIDs = new List<int>();
            prunedSkeletonList = new List<Skeleton>();
        }

        public void AddVisualKinectUnit(VisualKinectUnit unit){
            unit.locatedSensor.sensor.SkeletonFrameReady += updateSkeletons;
            visualKinectUnits.Add(unit);
        }
        

        public List<Skeleton> getGlobalSkeletons()
        {
            return prunedSkeletonList;
        }


        Dictionary<String, List<Skeleton>> allSkeletons = new Dictionary<String, List<Skeleton>>();
        Dictionary<int, int> associatedSkeletons = new Dictionary<int, int>();

        List<Skeleton> trackedSkeletons = new List<Skeleton>();

        private String findSkeletonKinect(Skeleton skel)
        {
            String result = null;
            foreach (KeyValuePair<String, List<Skeleton>> skels in allSkeletons)
            {
                if (skels.Value.Contains(skel))
                {
                    result = skels.Key;
                    break;
                }
            }
            return result;
        }

        private void updateSkeletons(object sender, SkeletonFrameReadyEventArgs e)
        {
            List<Skeleton> currentFrameSkeletons = new List<Skeleton>();

            allSkeletons.Clear();
            List<Skeleton> lastTrackedSkeletons = new List<Skeleton>( trackedSkeletons);
            trackedSkeletons.Clear();

            foreach (VisualKinectUnit kinect in this.visualKinectUnits)
            {
                foreach (Skeleton skel in kinect.locatedSensor.globalSkeletons)
                {
                    if (skel.TrackingState == SkeletonTrackingState.Tracked)
                    {
                        if (allSkeletons.ContainsKey(kinect.locatedSensor.sensor.UniqueKinectId) == false)
                        {
                            allSkeletons.Add(kinect.locatedSensor.sensor.UniqueKinectId, new List<Skeleton>());
                        }
                        allSkeletons[kinect.locatedSensor.sensor.UniqueKinectId].Add(skel);
                        currentFrameSkeletons.Add(skel);
                    }
                }
            }

            List<int> toRemove = new List<int>();
            foreach (KeyValuePair<int, int> a in associatedSkeletons) 
            {
                //currentFrameSkeletons.Exists()
            }
            

            foreach (KeyValuePair<String, List<Skeleton>> skels in allSkeletons)
            {
                foreach (Skeleton skel in skels.Value)
                {
                    bool presentLastFrame = lastTrackedSkeletons.Exists(x => x.TrackingId == skel.TrackingId);
                    if (!presentLastFrame && associatedSkeletons.ContainsKey(skel.TrackingId) == false && associatedSkeletons.ContainsValue(skel.TrackingId) == false)
                    {
                        String kinectId = findSkeletonKinect(skel);
                        Console.WriteLine("new skeleton named " + skel.TrackingId.ToString());
                        foreach (Skeleton s in currentFrameSkeletons)
                        {
                            String sKinectId = findSkeletonKinect(s);
                            if (sKinectId.Equals(kinectId) == false)
                            {
                                if (isTheSameSkeleton(skel, s))
                                {
                                    Console.WriteLine("similar to " + s.TrackingId.ToString());
                                    associatedSkeletons.Add(skel.TrackingId, s.TrackingId);
                                    break;
                                }
                            }
                        }

                        Console.WriteLine("tracking " + skel.TrackingId.ToString());
                        trackedSkeletons.Add(skel);
                    }
                }
            }

            foreach (KeyValuePair<String, List<Skeleton>> skels in allSkeletons)
            {
                foreach (Skeleton skel in skels.Value)
                {
                    bool presentLastFrame = lastTrackedSkeletons.Exists(x => x.TrackingId == skel.TrackingId);
                    if (presentLastFrame)
                    {
                        if (associatedSkeletons.ContainsValue(skel.TrackingId) == false)
                        {
                            //Console.WriteLine("tracking " + skel.TrackingId.ToString());
                            trackedSkeletons.Add(skel);
                        }
                    }
                }
            }

            
            foreach (KeyValuePair<String, List<Skeleton>> skels in allSkeletons)
            {
                foreach (Skeleton skel in skels.Value)
                {

                    if (trackedSkeletons.Exists(x => x.TrackingId == skel.TrackingId) == false)
                    {
                        if (associatedSkeletons.ContainsValue(skel.TrackingId) == true)
                        {
                            try
                            {
                                KeyValuePair<int, int> i = associatedSkeletons.FirstOrDefault(x => x.Value == skel.TrackingId);
                                if (trackedSkeletons.Exists(x => x.TrackingId == i.Key) == false)
                                {
                                    Console.WriteLine("tracking orphan " + skel.TrackingId.ToString());
                                    trackedSkeletons.Add(skel);
                                    associatedSkeletons.Remove(i.Key);
                                    associatedSkeletons.Remove(skel.TrackingId);
                                    associatedSkeletons.Add(skel.TrackingId, i.Key);
                                }
                            }
                            catch (InvalidOperationException ex)
                            {

                            }
                        }
                    }
                        
                    
                }
            }
            
            prunedSkeletonList = new List<Skeleton>(trackedSkeletons);

            foreach (VisualKinectUnit kinect in this.visualKinectUnits)
            {
                lock (kinect.locatedSensor.trackedSkeletonsLock)
                {
                    kinect.locatedSensor.trackedSkeletons.Clear();
                    foreach (Skeleton skel in trackedSkeletons)
                    {
                        String k = findSkeletonKinect(skel);
                        if (k.Equals(kinect.locatedSensor.sensor.UniqueKinectId))
                        {
                            kinect.locatedSensor.trackedSkeletons.Add(skel);
                        }
                    }
                }
            }

            return;

            Console.WriteLine("x:");
            masterSkeletonList = new List<Skeleton>();
            List<int> currentSkeletonIDs = new List<int>();
            // From each of our kinect sensors...
            foreach (VisualKinectUnit kinect in this.visualKinectUnits)
            {
                // Read all our skeleton data
                foreach (Skeleton skel in kinect.locatedSensor.globalSkeletons)
                {
                    // And if the skeleton is being tracked...
                    if (skel.TrackingState == SkeletonTrackingState.Tracked)
                    {
                        currentSkeletonIDs.Add(skel.TrackingId);
                        bool isInMasterList = false;
                        // if it's in our master list already, 
                        for (int i = 0; i < masterSkeletonList.Count; i++)
                        {
                            // update the skeleton to the fresh data
                            if (skel.TrackingId == masterSkeletonList[i].TrackingId)
                            {
                                masterSkeletonList[i] = skel;
                                isInMasterList = true;
                                break;
                            }
                        }
                        if (!isInMasterList)
                        {
                            masterSkeletonList.Add(skel);
                        }
                    }
                }
            }
            // Now, make sure we remove extra IDs of skeletons that aren't in our view anymore
            for (int i = leadSkeletonIDs.Count - 1; i >= 0; i--)
            {
                if (currentSkeletonIDs.Find(item => item == leadSkeletonIDs[i]) == 0)
                {
                    leadSkeletonIDs.RemoveAt(i);
                }
            }

            // Now let's pick a skeleton to persistently follow if we're not following one
            if (leadSkeletonIDs.Count == 0 && currentSkeletonIDs.Count > 0)
            {
                leadSkeletonIDs.Add(currentSkeletonIDs[0]);
            }

            // And let's find duplicate skeletons that are our lead skeletons
            if (leadSkeletonIDs.Count > 0)
            {
                Skeleton trackedSkeleton = new Skeleton();
                // Find our tracked skeleton
                foreach (Skeleton skel in masterSkeletonList)
                {
                    if ((skel.TrackingState == SkeletonTrackingState.Tracked) && (currentSkeletonIDs.Find(id => id == skel.TrackingId) != 0))
                    {
                        trackedSkeleton = skel;
                        break;
                    }
                }

                // Iterate through it agian, since we might have missed it the first time
                foreach (Skeleton skel in masterSkeletonList)
                {
                    if ((skel.TrackingState == SkeletonTrackingState.Tracked) &&
                        skel.Position.X < trackedSkeleton.Position.X + sameSkeletonRadius &&
                        skel.Position.X > trackedSkeleton.Position.X - sameSkeletonRadius &&
                        skel.Position.Z < trackedSkeleton.Position.Z + sameSkeletonRadius &&
                        skel.Position.Z > trackedSkeleton.Position.Z - sameSkeletonRadius &&
                        skel != trackedSkeleton)
                    {
                        if (leadSkeletonIDs.Find(item => item == skel.TrackingId) == 0)
                            leadSkeletonIDs.Add(skel.TrackingId);
                    }
                }

            }

            // Now let's prune our skeleton list to remove the duplicates
            prunedSkeletonList = new List<Skeleton>();

            foreach (Skeleton skel in masterSkeletonList)
            {
                if (skel.TrackingState != SkeletonTrackingState.NotTracked)
                {
                    Boolean isUnique = true;
                    for (int i = 0; i < prunedSkeletonList.Count; i++)
                    {
                        if (isTheSameSkeleton(skel, prunedSkeletonList[i]))
                        {
                            isUnique = false;
                            
                        }
                    }
                    if (isUnique)
                    {
                        prunedSkeletonList.Add(skel);
                    }
                }
            }
        }

        public double sameSkeletonAngle = 0.6;
        Boolean isTheSameSkeleton(Skeleton a, Skeleton b)
        {
            if (a.TrackingState == SkeletonTrackingState.Tracked && b.TrackingState == SkeletonTrackingState.Tracked)
            {
                Vector4 aq = a.BoneOrientations[JointType.Spine].HierarchicalRotation.Quaternion;
                Vector4 bq = b.BoneOrientations[JointType.Spine].HierarchicalRotation.Quaternion;
                return
                    aq.X < bq.X + sameSkeletonAngle &&
                    aq.X > bq.X - sameSkeletonAngle &&

                    aq.Y < bq.Y + sameSkeletonAngle &&
                    aq.Y > bq.Y - sameSkeletonAngle &&

                    aq.Z < bq.Z + sameSkeletonAngle &&
                    aq.Z > bq.Z - sameSkeletonAngle &&

                    aq.W < bq.W + sameSkeletonAngle &&
                    aq.W > bq.W - sameSkeletonAngle;

            }
            return false;



            if ((a.TrackingState == SkeletonTrackingState.Tracked) &&
                        a.Position.X < b.Position.X + sameSkeletonRadius &&
                        a.Position.X > b.Position.X - sameSkeletonRadius &&
                        a.Position.Z < b.Position.Z + sameSkeletonRadius &&
                        a.Position.Z > b.Position.Z - sameSkeletonRadius)
            {
                return true;
            }
            else
                return false;
        }
    }
}
