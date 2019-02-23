using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Xml;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.Storage;
using Windows.Storage.Pickers;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using Svg;
using Svg.Pathing;

// The Blank Page item template is documented at https://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace LaunchPad
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        public MainPage()
        {
            this.InitializeComponent();
            filePicker = new FileOpenPicker();
        }

        StorageFile imageFile;
        FileOpenPicker filePicker;

        private async void LoadSVGButton_Click(object sender, RoutedEventArgs e)
        {
            filePicker.FileTypeFilter.Add(".svg");
            filePicker.SuggestedStartLocation = PickerLocationId.Desktop;
            imageFile = await filePicker.PickSingleFileAsync();
            if(imageFile != null) {
                string imageFileString = await FileIO.ReadTextAsync(imageFile);
                FileTextBlock.Text = imageFileString;
                FileTextScrollView.Visibility = Visibility.Visible;
                FileTextScrollTitleBlock.Visibility = Visibility.Visible;

                XmlDocument doc = new XmlDocument();
                doc.LoadXml(imageFileString);
                XmlNodeList Paths = doc.GetElementsByTagName("path");
                
                foreach(XmlElement path in Paths)
                {
                    string pathData = path.GetAttribute("d");
                    SvgPathSegmentList segments = SvgPathBuilder.Parse(pathData);
                    foreach(SvgPathSegment segment in segments)
                    {
                        if(segment.GetType() == typeof(SvgMoveToSegment))
                        {
                            var seg = (SvgMoveToSegment)segment;

                        }
                        else if (segment.GetType() == typeof(SvgLineSegment))
                        {
                            var seg = (SvgQuadraticCurveSegment)segment;

                        }
                        else if (segment.GetType() == typeof(SvgQuadraticCurveSegment))
                        {
                            var seg = (SvgQuadraticCurveSegment)segment;

                        }
                        else if (segment.GetType() == typeof(SvgCubicCurveSegment))
                        {
                            var seg = (SvgQuadraticCurveSegment)segment;

                        }
                        else if (segment.GetType() == typeof(SvgArcSegment))
                        {
                            var seg = (SvgQuadraticCurveSegment)segment;

                        }
                        else if (segment.GetType() == typeof(SvgClosePathSegment))
                        {
                            var seg = (SvgQuadraticCurveSegment)segment;

                        }
                    }
                }
            }
        }
    }
}
