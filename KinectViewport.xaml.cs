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
        public bool globalCheckbox;

        public void AttachVisualKinect(VisualKinectUnit unit)
        {
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
    }
}
