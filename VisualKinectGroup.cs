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
        private List<Skeleton> masterSkeletonList;
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

        private void updateSkeletons(object sender, SkeletonFrameReadyEventArgs e)
        {
            Console.WriteLine("x");
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

        Boolean isTheSameSkeleton(Skeleton a, Skeleton b)
        {
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
