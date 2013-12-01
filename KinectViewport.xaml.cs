using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace KinectOSC
{
    /// <summary>
    /// Interaction logic for KinectViewport.xaml
    /// </summary>
    public partial class KinectViewport : UserControl
    {
        private VisualKinectUnit attachedKinectUnit;

        public void AttachVisualKinect(VisualKinectUnit unit){
            attachedKinectUnit = unit;
        }

        public void DetachVisualKinectUnit()
        {
            attachedKinectUnit = null;
        }

        public KinectViewport()
        {
            InitializeComponent();
        }

        private float sanitizeTextToFloat(string input)
        {
            float output = 0.0f;
            if (!float.TryParse(input, out output))
            {
                output = 0.0f;
            }
            return output;
        }

        private void changeXOffset(object sender, TextChangedEventArgs e)
        {
            if (attachedKinectUnit != null)
            {
                attachedKinectUnit.locatedSensor.xOffset = sanitizeTextToFloat(this.xOffset.Text);
            }
        }

        private void changeYOffset(object sender, TextChangedEventArgs e)
        {
            if (attachedKinectUnit != null)
            {
                attachedKinectUnit.locatedSensor.yOffset = sanitizeTextToFloat(this.yOffset.Text);
            }
        }

        private void changeZOffset(object sender, TextChangedEventArgs e)
        {
            if (attachedKinectUnit != null)
            {
                attachedKinectUnit.locatedSensor.zOffset = sanitizeTextToFloat(this.zOffset.Text);
            }
        }

        private void changePitchAngle(object sender, TextChangedEventArgs e)
        {
            if (attachedKinectUnit != null)
            {
                attachedKinectUnit.locatedSensor.setPitch(sanitizeTextToFloat(this.pitchAngle.Text));
            }
        }

        private void changeYawAngle(object sender, TextChangedEventArgs e)
        {
            if (attachedKinectUnit != null)
            {
                attachedKinectUnit.locatedSensor.setYaw(sanitizeTextToFloat(this.yawAngle.Text));
            }
        }

        private void changeRollAngle(object sender, TextChangedEventArgs e)
        {
            if (attachedKinectUnit != null)
            {
                attachedKinectUnit.locatedSensor.setRoll(sanitizeTextToFloat(this.rollAngle.Text));
            }
        }
    }
}
