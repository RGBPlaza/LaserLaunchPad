using System;
using System.Collections.Generic;
using System.Linq;
using System.Xml;
using Windows.Storage;
using Windows.Storage.Pickers;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Media;
using Svg;
using Svg.Pathing;
using System.Drawing;
using Windows.Devices.SerialCommunication;
using Windows.Devices.Enumeration;
using System.Text;
using Windows.Graphics.Imaging;
using Windows.Storage.Streams;
using SerialPortLib;
using Newtonsoft.Json;
using Windows.UI.Xaml.Navigation;
using Windows.ApplicationModel.Activation;

namespace LaunchPad
{
    public sealed partial class MainPage : Page
    {
        public MainPage()
        {
            this.InitializeComponent();
            filePicker = new FileOpenPicker();
            filePicker.SuggestedStartLocation = PickerLocationId.Desktop;
            fileSavePicker = new FileSavePicker();
            fileSavePicker.FileTypeChoices.Add("Laser LaunchPad Project", new string[] { ".llpp" });
            fileSavePicker.SuggestedStartLocation = PickerLocationId.Desktop;
            designs = new List<Design>();
            cutterSerial = new SerialPortInput();
            //instructionTimer = new Timer() { Interval = 64, AutoReset = true };
            //instructionTimer.Elapsed += InstructionTimer_Elapsed;
            laserCircle = new Windows.UI.Xaml.Shapes.Ellipse() { Width = 6, Height = 6, Fill = new SolidColorBrush(Windows.UI.Colors.BlueViolet) };
            Canvas.SetLeft(laserCircle, -3);
            Canvas.SetTop(laserCircle, -3);
            Canvas.SetZIndex(laserCircle, 10);
            PreviewCanvas.Children.Add(laserCircle);

            cutterSerial.MessageReceived += CutterSerial_MessageReceived;
        }

        private StorageFile imageFile;
        private readonly FileOpenPicker filePicker;
        private readonly FileSavePicker fileSavePicker;
        private readonly List<Design> designs;
        private Design selectedDesign;
        private static SerialPortInput cutterSerial;
        //private readonly Timer instructionTimer;
        private readonly Windows.UI.Xaml.Shapes.Ellipse laserCircle;

        private async void LoadRasterButton_Click(object sender, RoutedEventArgs e)
        {
            filePicker.FileTypeFilter.Clear();
            filePicker.FileTypeFilter.Add(".bmp");
            filePicker.FileTypeFilter.Add(".png");
            filePicker.FileTypeFilter.Add(".jpg");
            imageFile = await filePicker.PickSingleFileAsync();
            if (imageFile != null)
            {
                using (IRandomAccessStream stream = await imageFile.OpenAsync(FileAccessMode.Read))
                {
                    BitmapDecoder decoder = await BitmapDecoder.CreateAsync(stream);
                    if (decoder.BitmapPixelFormat == BitmapPixelFormat.Bgra8)
                    {
                        selectedDesign = new BitmapDesign(await decoder.GetPixelDataAsync(), decoder);
                        designs.Add(selectedDesign);
                        UpdatePreview();
                        PropertiesScrollView.Visibility = Visibility.Visible;
                        PropertiesTitleBlock.Visibility = Visibility.Visible;

                        PrintButton.IsEnabled = true;
                    }
                }
            }
        }

        private async void LoadSVGButton_Click(object sender, RoutedEventArgs e)
        {
            filePicker.FileTypeFilter.Clear();
            filePicker.FileTypeFilter.Add(".svg");
            imageFile = await filePicker.PickSingleFileAsync();
            if (imageFile != null)
            {
                string imageFileString = await FileIO.ReadTextAsync(imageFile);
                XmlDocument xDoc = new XmlDocument();
                xDoc.LoadXml(imageFileString);
                selectedDesign = new SVGDesign(xDoc);
                designs.Add(selectedDesign);
                UpdatePreview();
                PropertiesScrollView.Visibility = Visibility.Visible;
                PropertiesTitleBlock.Visibility = Visibility.Visible;

                PrintButton.IsEnabled = true;
            }
        }

        private void UpdatePreview()
        {
            PreviewCanvas.Children.Clear();
            foreach (Design design in designs)
            {
                Border border = design.GetPreviewElement();
                border.BorderBrush.Opacity = Equals(design, selectedDesign) ? 1 : 0;
                Canvas.SetLeft(border, design.X * Design.previewScale);
                Canvas.SetTop(border, design.Y * Design.previewScale);
                border.Tag = designs.IndexOf(design);
                ((Canvas)border.Child).Tag = designs.IndexOf(design);
                border.Tapped += PreviewBorder_Tapped;
                border.Child.Tapped += PreviewCanvas_Tapped;
                PreviewCanvas.Children.Add(border);
            }
            PreviewCanvas.Children.Add(laserCircle);

            XPosTextBox.Text = Math.Round(selectedDesign.X,4).ToString();
            YPosTextBox.Text = Math.Round(selectedDesign.Y,4).ToString();
            WidthTextBox.Text = Math.Round(selectedDesign.Width,4).ToString();
            HeightTextBox.Text = Math.Round(selectedDesign.Height,4).ToString();
            ScaleTextBox.Text = Math.Round(selectedDesign.Scale,4).ToString();
            LaserPowerSlider.Value = selectedDesign.LaserPower;

            if (selectedDesign.GetType() == typeof(SVGDesign))
            {
                SmoothnessSlider.Value = ((SVGDesign)selectedDesign).Smoothness;
                PassesSlider.Value = ((SVGDesign)selectedDesign).Passes;
                VectorPropertiesGrid.Visibility = Visibility.Visible;
                RasterPropertiesGrid.Visibility = Visibility.Collapsed;
            }

            if (selectedDesign.GetType() == typeof(BitmapDesign))
            {
                ThresholdSlider.Value = ((BitmapDesign)selectedDesign).Threshold;
                ShadesSlider.Value = ((BitmapDesign)selectedDesign).Shades;
                NegateSwitch.IsOn = ((BitmapDesign)selectedDesign).IsNegative;
                VectorPropertiesGrid.Visibility = Visibility.Collapsed;
                RasterPropertiesGrid.Visibility = Visibility.Visible;
            }
        }

        private void PreviewCanvas_Tapped(object sender, Windows.UI.Xaml.Input.TappedRoutedEventArgs e)
        {
            selectedDesign = designs[(int)((Canvas)sender).Tag];
            UpdatePreview();
        }

        private void PreviewBorder_Tapped(object sender, Windows.UI.Xaml.Input.TappedRoutedEventArgs e)
        {
            selectedDesign = designs[(int)((Border)sender).Tag];
            UpdatePreview();
        }



        // Cut-Time Variables
        private double destX, destY = 0;

        private List<CutterInstruction> instructions = new List<CutterInstruction>();

        private void PrintButton_Click(object sender, RoutedEventArgs e)
        {
            if (instructions != null && cutterSerial != null)
            {
                instructions.Clear();
                foreach (Design design in designs)
                {
                    instructions.AddRange(design.GetArduinoInstructions());
                }
                instructions.Add(CutterInstruction.PenUpInstruction);
                instructions.Add(CutterInstruction.ReturnToOriginInstruction);

                if (!cutterSerial.IsConnected)
                    cutterSerial.Connect();

                //InstructionsRequested = true;
                //instructionTimer.Start();
                SendNextInstruction();
                StopButton.IsEnabled = true;
                NextButton.IsEnabled = true;

            }
        }

        /*private void CutterSerial_MessageReceived(object sender, MessageReceivedEventArgs e)
        {
            string resp = Encoding.ASCII.GetString(e.Data);
            if (resp == ":)")
                InstructionsRequested = true;
            else if (resp == ":(")
                InstructionsRequested = false;
            else if (resp == "(:")
            {
                instructionTimer.Stop();
                InstructionsRequested = false;
                System.Diagnostics.Debug.WriteLine("Print Complete");
            }

        }*/
        private void CutterSerial_MessageReceived(object sender, MessageReceivedEventArgs e)
        {
            string resp = Encoding.ASCII.GetString(e.Data);
            if (resp == "+")
                SendNextInstruction();
            else
                System.Diagnostics.Debug.WriteLine(resp);
        }


        private async void SendNextInstruction()
        {
            if (instructions.Any())
            {
                string msg;
                CutterInstruction currentInstruction;
                do
                {
                    currentInstruction = instructions.First();
                    if (currentInstruction.type == CutterInstruction.InstructionType.velocityTime)
                    {
                        msg = currentInstruction.GetArduinoMessage(destX, destY);
                        (destX, destY) = (currentInstruction.Point.X, currentInstruction.Point.Y);
                    }
                    else if (currentInstruction.type == CutterInstruction.InstructionType.printLine)
                    {
                        msg = currentInstruction.GetArduinoMessage(destX);
                        destX = currentInstruction.Point.X;
                    }
                    else 
                    {
                        msg = currentInstruction.GetArduinoMessage();
                    }
                    instructions.Remove(currentInstruction);
                } while (string.IsNullOrWhiteSpace(msg) && instructions.Any());
                System.Diagnostics.Debug.WriteLine(msg);
                cutterSerial.SendMessage(Encoding.ASCII.GetBytes(msg));

            }
            else
            {
                cutterSerial.SendMessage(Encoding.ASCII.GetBytes(CutterInstruction.StopMessage));
                await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
                {
                    StopButton.IsEnabled = false;
                    NextButton.IsEnabled = false;
                });
            }
        }

        private async void PortComboBox_DropDownOpened(object sender, object e)
        {
            string aqsFilter = SerialDevice.GetDeviceSelector();
            var devices = await DeviceInformation.FindAllAsync(aqsFilter);
            if (devices.Any())
                PortComboBox.ItemsSource = devices.Where(x => x.Name.Contains("COM"));
        }

        private void LaserPowerSlider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            if (selectedDesign != null)
            {
                if ((int)e.NewValue != selectedDesign.LaserPower)
                {
                    selectedDesign.LaserPower = (int)e.NewValue;
                    UpdatePreview();
                }
            }
        }

        private void SmoothnessSlider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            if (selectedDesign != null)
            {
                if ((int)e.NewValue != ((SVGDesign)selectedDesign).Smoothness)
                {
                    ((SVGDesign)selectedDesign).Smoothness = (int)e.NewValue;
                    UpdatePreview();
                }
            }
        }

        private void PassesSlider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            if (selectedDesign != null)
            {
                if ((int)e.NewValue != ((SVGDesign)selectedDesign).Passes)
                {
                    ((SVGDesign)selectedDesign).Passes = (int)e.NewValue;
                    UpdatePreview();
                }
            }
        }

        private void ThresholdSlider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            if (selectedDesign != null)
            {
                if ((uint)e.NewValue != ((BitmapDesign)selectedDesign).Threshold)
                {
                    ((BitmapDesign)selectedDesign).Threshold = (int)e.NewValue;
                    UpdatePreview();
                }
            }
        }

        private void DeleteButton_Click(object sender, RoutedEventArgs e)
        {
            designs.Remove(selectedDesign);
            selectedDesign = null;
            if (designs.Count == 0)
            {
                PreviewCanvas.Children.Clear();
                PreviewCanvas.Children.Add(laserCircle);
                PropertiesScrollView.Visibility = Visibility.Collapsed;
                PropertiesTitleBlock.Visibility = Visibility.Collapsed;
                PrintButton.IsEnabled = false;
            }
            else
            {
                selectedDesign = designs.Last();
                UpdatePreview();
            }
        }

        private void NegateSwitch_Toggled(object sender, RoutedEventArgs e)
        {
            if (selectedDesign != null)
            {
                if (NegateSwitch.IsOn != ((BitmapDesign)selectedDesign).IsNegative)
                {
                    ((BitmapDesign)selectedDesign).IsNegative = NegateSwitch.IsOn;
                    UpdatePreview();
                }
            }
        }

        private void ShadesSlider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            if (selectedDesign != null)
            {
                if ((uint)e.NewValue != ((BitmapDesign)selectedDesign).Shades)
                {
                    ((BitmapDesign)selectedDesign).Shades = (int)e.NewValue;
                    UpdatePreview();
                }
            }
        }

        private void StopButton_Click(object sender, RoutedEventArgs e)
        {
            instructions.Clear();
            cutterSerial.SendMessage(Encoding.ASCII.GetBytes(CutterInstruction.StopMessage));
            StopButton.IsEnabled = false;
            NextButton.IsEnabled = false;
            (destX, destY) = (0, 0);
        }

        private void NextButton_Click(object sender, RoutedEventArgs e)
        {
            SendNextInstruction();
        }

        private void ScaleTextBox_KeyUp(object sender, Windows.UI.Xaml.Input.KeyRoutedEventArgs e)
        {
            if (double.TryParse(ScaleTextBox.Text, out double scale))
            {
                if (scale != selectedDesign.Scale && scale > 0)
                {
                    selectedDesign.Scale = scale;
                    UpdatePreview();
                }
            }
        }

        private void HeightTextBox_KeyUp(object sender, Windows.UI.Xaml.Input.KeyRoutedEventArgs e)
        {
            if (double.TryParse(HeightTextBox.Text, out double height))
            {
                if (height != selectedDesign.Height && height > 0)
                {
                    selectedDesign.Height = height;
                    UpdatePreview();
                }
            }
        }

        private void WidthTextBox_KeyUp(object sender, Windows.UI.Xaml.Input.KeyRoutedEventArgs e)
        {
            if (double.TryParse(WidthTextBox.Text, out double width))
            {
                if (width != selectedDesign.Width && width > 0)
                {
                    selectedDesign.Width = width;
                    UpdatePreview();
                }
            }
        }

        private void YPosTextBox_KeyUp(object sender, Windows.UI.Xaml.Input.KeyRoutedEventArgs e)
        {
            if (double.TryParse(YPosTextBox.Text, out double yPos))
            {
                if (yPos != selectedDesign.Y)
                {
                    selectedDesign.Y = yPos;
                    UpdatePreview();
                }
            }
        }

        private void XPosTextBox_KeyUp(object sender, Windows.UI.Xaml.Input.KeyRoutedEventArgs e)
        {
            if (double.TryParse(XPosTextBox.Text, out double xPos))
            {
                if (xPos != selectedDesign.X)
                {
                    selectedDesign.X = xPos;
                    UpdatePreview();
                }
            }
        }

        private async void LoadProjectButton_Click(object sender, RoutedEventArgs e)
        {
            filePicker.FileTypeFilter.Clear();
            filePicker.FileTypeFilter.Add(".llpp");
            imageFile = await filePicker.PickSingleFileAsync();
            if (imageFile != null)
            {
                string projectFileString = await FileIO.ReadTextAsync(imageFile);
                var proj = JsonConvert.DeserializeObject<LaserLaunchPadProject>(projectFileString);

                designs.Clear();
                foreach(SVGProjectObj svgObj in proj.SVGProjectObjs)
                    designs.Add(new SVGDesign(svgObj));
                foreach (BitmapProjectObj bmpObj in proj.BitmapProjectObjs)
                    designs.Add(new BitmapDesign(bmpObj));

                selectedDesign = designs[0];
                UpdatePreview();
                PropertiesScrollView.Visibility = Visibility.Visible;
                PropertiesTitleBlock.Visibility = Visibility.Visible;

                PrintButton.IsEnabled = true;
            }
        }

        private async void SaveProjectButton_Click(object sender, RoutedEventArgs e)
        {
            imageFile = await fileSavePicker.PickSaveFileAsync();
            if(imageFile != null)
            {
                var proj = new LaserLaunchPadProject();
                foreach(Design design in designs)
                {
                    if (design.GetType() == typeof(SVGDesign))
                        proj.SVGProjectObjs.Add(((SVGDesign)design).GetProjectObj());
                    else if (design.GetType() == typeof(BitmapDesign))
                        proj.BitmapProjectObjs.Add(((BitmapDesign)design).GetProjectObj());
                }
                string projectFileString = JsonConvert.SerializeObject(proj);
                await FileIO.WriteTextAsync(imageFile, projectFileString);
            }
        }

        protected override async void OnNavigatedTo(NavigationEventArgs e)
        {
            base.OnNavigatedTo(e);
            if (e.Parameter.GetType() == typeof(FileActivatedEventArgs))
            {
                var args = (FileActivatedEventArgs)e.Parameter;
                imageFile = (StorageFile)args.Files[0];
                string projectFileString = await FileIO.ReadTextAsync(imageFile);
                var proj = JsonConvert.DeserializeObject<LaserLaunchPadProject>(projectFileString);

                designs.Clear();
                foreach (SVGProjectObj svgObj in proj.SVGProjectObjs)
                    designs.Add(new SVGDesign(svgObj));
                foreach (BitmapProjectObj bmpObj in proj.BitmapProjectObjs)
                    designs.Add(new BitmapDesign(bmpObj));

                selectedDesign = designs[0];
                UpdatePreview();
                PropertiesScrollView.Visibility = Visibility.Visible;
                PropertiesTitleBlock.Visibility = Visibility.Visible;

                PrintButton.IsEnabled = true;
            }
        }

        private void PortComboBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (PortComboBox.SelectedItem != null)
            {
                DeviceInformation info = (DeviceInformation)PortComboBox.SelectedItem;
                string portName = info.Name.Substring(info.Name.IndexOf("COM"), 5);
                cutterSerial.SetPort(portName, 2000000);
            }
        }
    }

    public class CenterParameterizedArcData
    {
        public float cx;
        public float cy;
        public float rx;
        public float ry;
        public float theta0;
        public float dTheta;
        public float angle;

        public CenterParameterizedArcData(SvgArcSegment seg)
        {
            float x0 = seg.Start.X;
            float y0 = seg.Start.Y;
            float x1 = seg.End.X;
            float y1 = seg.End.Y;
            float dX = (x0 - x1) / 2;
            float dY = (y0 - y1) / 2;
            float midX = (x0 + x1) / 2;
            float midY = (y0 + y1) / 2;

            angle = seg.Angle;

            float sinA = MathF.Sin(angle);
            float cosA = MathF.Cos(angle);

            // Intermediate Start Values
            float x0Prime = cosA * dX + sinA * dY;
            float y0Prime = cosA * dY - sinA * dX;

            rx = seg.RadiusX;
            ry = seg.RadiusY;

            float radius_check = (MathF.Pow(x0Prime, 2) / MathF.Pow(rx, 2)) + (MathF.Pow(y0Prime, 2) / MathF.Pow(ry, 2));
            if (radius_check > 1)
            {
                rx *= MathF.Sqrt(radius_check);
                ry *= MathF.Sqrt(radius_check);
            }

            // Intermediate Centre Values
            float temp0 = MathF.Pow(rx, 2) * MathF.Pow(ry, 2);
            float temp1 = MathF.Pow(rx, 2) * MathF.Pow(y0Prime, 2);
            float temp2 = MathF.Pow(ry, 2) * MathF.Pow(x0Prime, 2);

            float cPrime = MathF.Sqrt(MathF.Abs((temp0 - temp1 - temp2) / (temp1 + temp2))) * (Equals(seg.Size == SvgArcSize.Large, seg.Sweep == SvgArcSweep.Positive) ? -1 : 1);
            float cxPrime = cPrime * rx * y0Prime / ry;
            float cyPrime = -cPrime * ry * x0Prime / rx;

            // Calculate Centre
            cx = cosA * cxPrime - sinA * cyPrime + midX;
            cy = sinA * cxPrime + cosA * cyPrime + midY;

            // Calculate Angles
            float ux = (x0Prime - cxPrime) / rx;
            float uy = (y0Prime - cyPrime) / ry;
            float vx = (-x0Prime - cxPrime) / rx;
            float vy = (-y0Prime - cyPrime) / ry;

            // Calculate Starting Angle
            float n0 = MathF.Sqrt(MathF.Pow(ux, 2) + MathF.Pow(uy, 2));
            theta0 = MathF.Acos(ux / n0) * (uy > 0 ? 1 : -1);

            // Calculate dTheta
            float dn = MathF.Sqrt(MathF.Pow(ux, 2) + MathF.Pow(uy, 2)) * MathF.Sqrt(MathF.Pow(vx, 2) + MathF.Pow(vy, 2));
            float dp = ux * vx + uy * vy;
            float d = dp / dn;
            d = (d > 0) ? MathF.Min(d, 1) : MathF.Max(-1, d);
            dTheta = (ux * vy - vx * uy > 0) ? MathF.Acos(d) : -MathF.Acos(d);
            dTheta %= (2 * MathF.PI);
            if (seg.Sweep == SvgArcSweep.Negative)
                dTheta -= 2 * MathF.PI;

        }
        // guidance: www.w3.org/TR/SVG/implnote.html#ArcSyntax
        // there is no easy way to do this except convert to centre parameterization lol 
    }

    public static class PointFExtensions
    {
        public static Windows.Foundation.Point ToFoundationPoint(this PointF point, double scale)
        {
            return new Windows.Foundation.Point(point.X * scale, point.Y * scale);
        }
    }

    public class BitmapPixel
    {
        public uint R;
        public uint G;
        public uint B;
        public uint A;

        public BitmapPixel(uint r, uint g, uint b, uint a)
        {
            R = r;
            G = g;
            B = b;
            A = a;
        }

    }

    public class BitmapEditor
    {
        public readonly int Width;
        public readonly int Height;
        private readonly List<BitmapPixel> pixels;

        public BitmapEditor(byte[] pixelData, double width, double height)
        {
            Width = (int)width;
            Height = (int)height;
            pixels = new List<BitmapPixel>();
            for (int i = 0; i < pixelData.Length; i += 4)
            {
                pixels.Add(new BitmapPixel(pixelData[i], pixelData[i + 1], pixelData[i + 2], pixelData[i + 3]));
            }
        }

        public BitmapPixel GetPixel(int x, int y)
        {
            int pixelIndex = (y * Width) + x;
            return pixels[pixelIndex];
        }

        public void SetPixel(int x, int y, BitmapPixel pixel)
        {
            int pixelIndex = (y * Width) + (x * 4);
            pixels[pixelIndex] = pixel;
        }

        public byte[,] GetShadeMap(int threshold, int shades, bool printNegative)
        {
            byte[,] shadeMap = new byte[Width, Height];
            double interval = 1d / (shades - 1);
            for (int y = 0; y < Height; y++)
            {
                for (int x = 0; x < Width; x++)
                {
                    BitmapPixel px = GetPixel(x, y);
                    byte shade;
                    double mean = 255 - ((px.R + px.G + px.B) / 3);
                    double proportion = mean / threshold;
                    if (proportion >= 1)
                        shade = 255;
                    else
                    {
                        double shadeClass = Math.Floor(proportion / interval);
                        shade = (byte)(255 * shadeClass / shades);
                    }
                    shadeMap[x, y] = printNegative ? (byte)(255 - shade) : shade;
                }
            }
            return shadeMap;
        }
    }

    public abstract class Design
    {
        public const double previewScale = 3;

        protected double originalWidth;
        protected double originalHeight;

        protected double width;
        protected double height;
        protected double scale = 1;
        protected double x = 0;
        protected double y = 0;

        public double X { get => x; set => x = value; }
        public double Y { get => y; set => y = value; }
        public double Scale { get => scale; set { scale = value; width = originalWidth * scale; height = originalHeight * scale; } }
        public double Width { get => width; set { width = value; scale = width / originalWidth; height = originalHeight * scale; } }
        public double Height { get => height; set { height = value; scale = height / originalHeight; width = originalWidth * scale; } }
        public int LaserPower { get; set; } = 128;
        public double OriginalWidth { get => originalWidth; }
        public double OriginalHeight { get => originalHeight; }

        public abstract List<CutterInstruction> GetArduinoInstructions();

        public abstract Border GetPreviewElement();
    }

    public class SVGDesign : Design
    {
        public SVGDesign(XmlDocument xDoc)
        {
            paths = new List<SvgPathSegmentList>();
            points = new List<PointCollection>();
            previewPoints = new List<PointCollection>();
            _svgPathData = new List<string>();
            XmlNodeList xmlPaths = xDoc.GetElementsByTagName("path");
            foreach (XmlElement path in xmlPaths)
            {
                string pathData = path.GetAttribute("d");
                _svgPathData.Add(pathData);
                SvgPathSegmentList segments = SvgPathBuilder.Parse(pathData);
                paths.Add(segments);
            }
            var dims = ((XmlElement)xDoc.GetElementsByTagName("svg")[0]).GetAttribute("viewBox").Split(' ');
            (originalWidth, originalHeight) = (double.Parse(dims[2]), double.Parse(dims[3]));
            (width, height) = (originalWidth, originalHeight);
            UpdatePoints();
        }

        public SVGDesign(SVGProjectObj projectObj)
        {
            paths = new List<SvgPathSegmentList>();
            points = new List<PointCollection>();
            previewPoints = new List<PointCollection>();
            _svgPathData = new List<string>();
            LoadValues(projectObj); 
            foreach (string pathData in _svgPathData)
            {
                SvgPathSegmentList segments = SvgPathBuilder.Parse(pathData);
                paths.Add(segments);
            }
            UpdatePoints();
        }

        private void LoadValues(SVGProjectObj projectObj)
        {
            x = projectObj.X;
            y = projectObj.Y;
            originalWidth = projectObj.OriginalWidth;
            originalHeight = projectObj.OriginalHeight;
            width = projectObj.Width;
            height = projectObj.Height;
            scale = width / originalWidth;
            LaserPower = projectObj.LaserPower;
            Smoothness = projectObj.Smoothness;
            Passes = projectObj.Passes;
            _svgPathData = projectObj.PathData;
        }

        public SVGProjectObj GetProjectObj() => new SVGProjectObj(X, Y, OriginalWidth, OriginalHeight, Width, Height, LaserPower, Smoothness, Passes, _svgPathData);

        private float step { get => MathF.Pow(1.2f, -Smoothness); }
        public int Smoothness { get; set; } = 1;
        public int Passes { get; set; } = 1;

        private List<string> _svgPathData;
        private readonly List<SvgPathSegmentList> paths;
        private readonly List<PointCollection> points;
        private readonly List<PointCollection> previewPoints;

        private void UpdatePoints()
        {
            double scalingFactor = scale * previewScale;
            points.Clear();
            previewPoints.Clear();
            foreach (SvgPathSegmentList path in paths)
            {
                foreach (SvgPathSegment segment in path)
                {
                    if (segment.GetType() == typeof(SvgMoveToSegment))
                    {
                        var seg = (SvgMoveToSegment)segment;
                        points.Add(GetMoveToPointCollection(seg, scale));
                        previewPoints.Add(GetMoveToPointCollection(seg, scalingFactor));
                    }
                    else if (segment.GetType() == typeof(SvgLineSegment))
                    {
                        var seg = (SvgLineSegment)segment;
                        points.Last().Add(GetLineToPoint(seg).ToFoundationPoint(scale));
                        previewPoints.Last().Add(GetLineToPoint(seg).ToFoundationPoint(scalingFactor));

                    }
                    else if (segment.GetType() == typeof(SvgQuadraticCurveSegment))
                    {
                        var seg = (SvgQuadraticCurveSegment)segment;
                        for (float t = 0; t < 1; t += step)
                        {
                            points.Last().Add(GetQuadraticPoint(seg, t).ToFoundationPoint(scale));
                            previewPoints.Last().Add(GetQuadraticPoint(seg, t).ToFoundationPoint(scalingFactor));
                        }
                        points.Last().Add(seg.End.ToFoundationPoint(scale));
                        previewPoints.Last().Add(seg.End.ToFoundationPoint(scalingFactor));
                    }
                    else if (segment.GetType() == typeof(SvgCubicCurveSegment))
                    {
                        var seg = (SvgCubicCurveSegment)segment;
                        for (float t = 0; t < 1; t += step)
                        {
                            points.Last().Add(GetCubicPoint(seg, t).ToFoundationPoint(scale));
                            previewPoints.Last().Add(GetCubicPoint(seg, t).ToFoundationPoint(scalingFactor));
                        }
                        points.Last().Add(seg.End.ToFoundationPoint(scale));
                        previewPoints.Last().Add(seg.End.ToFoundationPoint(scalingFactor));

                    }
                    else if (segment.GetType() == typeof(SvgArcSegment))
                    {
                        var seg = (SvgArcSegment)segment;
                        var arcData = new CenterParameterizedArcData(seg);
                        // Get Points for Polyline Approximation
                        for (float t = 0; t < 1; t += step)
                        {
                            points.Last().Add(GetArcPoint(arcData, t).ToFoundationPoint(scale));
                            previewPoints.Last().Add(GetArcPoint(arcData, t).ToFoundationPoint(scalingFactor));
                        }
                        points.Last().Add(seg.End.ToFoundationPoint(scale));
                        previewPoints.Last().Add(seg.End.ToFoundationPoint(scalingFactor));

                    }
                    else if (segment.GetType() == typeof(SvgClosePathSegment))
                    {
                        var seg = (SvgClosePathSegment)segment;

                        if (GetClosePathPoint() != seg.Start) // if length back to start is not zero
                        {
                            points.Last().Add(GetClosePathPoint().ToFoundationPoint(1));
                            previewPoints.Last().Add(GetClosePathPoint().ToFoundationPoint(previewScale));
                        }
                    }
                }
            }
        }

        public override List<CutterInstruction> GetArduinoInstructions()
        {
            UpdatePoints();
            List<CutterInstruction> instructions = new List<CutterInstruction> { CutterInstruction.PenUpInstruction };
            foreach (PointCollection pointCollection in points)
            {
                for (int pass = 0; pass < Passes; pass++)
                {
                    instructions.Add(new CutterInstruction(pointCollection.First(),X,Y));
                    instructions.Add(new CutterInstruction(LaserPower));
                    for (int i = 1; i < pointCollection.Count; i++)
                        instructions.Add(new CutterInstruction(pointCollection[i],X,Y));
                    instructions.Add(CutterInstruction.PenUpInstruction);
                }
            }
            return instructions;
        }

        public override Border GetPreviewElement()
        {
            UpdatePoints();
            Canvas canvas = new Canvas();

            foreach(PointCollection pointCollection in previewPoints)
            {
                canvas.Children.Add(new Windows.UI.Xaml.Shapes.Polyline()
                {
                    Stroke = new SolidColorBrush(Windows.UI.Colors.Black),
                    StrokeThickness = 2,
                    Points = pointCollection
                });
            }
            return new Border() { Child = canvas, Width = width * previewScale + 4, Height = height * previewScale + 4, Margin = new Thickness(-2, -2, 0, 0), BorderBrush = new SolidColorBrush(Windows.UI.Colors.Blue), BorderThickness = new Thickness(2) };
        }

        private PointF ColinearAtTime(PointF A, PointF B, float t)
        {
            float x = ((1 - t) * A.X) + (t * B.X);
            float y = ((1 - t) * A.Y) + (t * B.Y);
            return new PointF(x, y);
        }

        private PointCollection GetMoveToPointCollection(SvgMoveToSegment seg, double scale)
        {
            return new PointCollection() { seg.End.ToFoundationPoint(scale) };
        }

        private PointF GetLineToPoint(SvgLineSegment seg)
        {
            return seg.End;
        }

        private PointF GetQuadraticPoint(SvgQuadraticCurveSegment seg, float t)
        {
            PointF SC = ColinearAtTime(seg.Start, seg.ControlPoint, t);
            PointF CE = ColinearAtTime(seg.ControlPoint, seg.End, t);
            PointF point = ColinearAtTime(SC, CE, t);

            return point;
        }

        private PointF GetCubicPoint(SvgCubicCurveSegment seg, float t)
        {
            // Intermediate Points
            PointF SC1 = ColinearAtTime(seg.Start, seg.FirstControlPoint, t);
            PointF C1C2 = ColinearAtTime(seg.FirstControlPoint, seg.SecondControlPoint, t);
            PointF C2E = ColinearAtTime(seg.SecondControlPoint, seg.End, t);

            // Resolve Intermediate Points (Like Quadratic)
            PointF A = ColinearAtTime(SC1, C1C2, t);
            PointF B = ColinearAtTime(C1C2, C2E, t);
            PointF point = ColinearAtTime(A, B, t);

            return point;
        }

        private PointF GetArcPoint(CenterParameterizedArcData arcData, float t)
        {
            float sinA = MathF.Sin(arcData.angle);
            float cosA = MathF.Cos(arcData.angle);
            float theta = arcData.theta0 + (t * arcData.dTheta);
            float sinT = MathF.Sin(theta);
            float cosT = MathF.Cos(theta);
            float x = (cosA * arcData.rx * cosT) - (sinA * arcData.ry * sinT) + arcData.cx;
            float y = (sinA * arcData.rx * cosT) + (cosA * arcData.ry * sinT) + arcData.cy;

            return new PointF(x, y);
        }

        private PointF GetClosePathPoint()
        {
            var p = points.Last().First();
            return new PointF((float)p.X, (float)p.Y);
        }
    }

    public class BitmapDesign : Design
    {
        public BitmapDesign(PixelDataProvider pixelDataProvider, BitmapDecoder decoder)
        {
            _pixelData = pixelDataProvider.DetachPixelData();
            bitmapEditor = new BitmapEditor(_pixelData, decoder.PixelWidth, decoder.PixelHeight);
            (originalWidth, originalHeight) = (decoder.PixelWidth, decoder.PixelHeight);
            (width, height) = (originalWidth, originalHeight);
            UpdateBWMap();
        }

        public BitmapDesign(BitmapProjectObj projectObj)
        {
            LoadValues(projectObj);
            bitmapEditor = new BitmapEditor(_pixelData, originalWidth, originalHeight);
            UpdateBWMap();
        }

        public BitmapProjectObj GetProjectObj() => new BitmapProjectObj(X, Y, OriginalWidth, OriginalHeight, Width, Height, LaserPower, Shades, Threshold, IsNegative, _pixelData);

        private readonly BitmapEditor bitmapEditor;
        private byte[] _pixelData;
        private byte[,] shadeMap;
        
        private void LoadValues(BitmapProjectObj projectObj)
        {
            x = projectObj.X;
            y = projectObj.Y;
            originalWidth = projectObj.OriginalWidth;
            originalHeight = projectObj.OriginalHeight;
            width = projectObj.Width;
            height = projectObj.Height;
            scale = width / originalWidth;
            LaserPower = projectObj.LaserPower;
            Threshold = projectObj.Threshold;
            Shades = projectObj.Shades;
            IsNegative = projectObj.IsNegative;
            _pixelData = projectObj.PixelData;
        }

        public int Threshold { get; set; } = 128;
        public int Shades { get; set; } = 2;
        public bool IsNegative { get; set; } = false;

        private void UpdateBWMap() => shadeMap = bitmapEditor.GetShadeMap(Threshold, Shades, IsNegative);

        public override List<CutterInstruction> GetArduinoInstructions()
        {
            UpdateBWMap();
            List<CutterInstruction> instructions = new List<CutterInstruction>();
            int width = shadeMap.GetLength(0);
            int height = shadeMap.GetLength(1);
            byte previousShade;
            int startX;
            StringBuilder stringBuilder = new StringBuilder();
            for (int y = 0; y < height; y++)
            {
                stringBuilder.Clear();
                instructions.Add(CutterInstruction.PenUpInstruction);
                if (y % 2 == 0)
                {
                    startX = 0;
                    previousShade = shadeMap[0, y];
                    instructions.Add(new CutterInstruction(new Windows.Foundation.Point(0, y * scale), X, Y));
                    instructions.Add(new CutterInstruction(width * scale, width, X));

                    for (int x = 0; x < width; x += 1)
                    {
                        if (x == width - 1)
                        {
                            instructions.Add(new CutterInstruction(Math.Floor(previousShade / 255d * LaserPower), (width - startX)));
                        }
                        else if (shadeMap[x, y] != previousShade)
                        {
                            instructions.Add(new CutterInstruction(Math.Floor(previousShade / 255d * LaserPower), (x - startX)));
                            startX = x;
                            previousShade = shadeMap[x, y];
                        }
                    }
                }
                else
                {
                    instructions.Add(new CutterInstruction(new Windows.Foundation.Point(width * scale, y * scale), X, Y));
                    instructions.Add(new CutterInstruction(0, width, X));
                    startX = width - 1;
                    previousShade = shadeMap[width - 1, y];
                    for (int x = width - 1; x >= 0; x--)
                    {
                        if (x == 0)
                        {
                            instructions.Add(new CutterInstruction(Math.Floor(previousShade / 255d * LaserPower), (startX + 1)));

                        }
                        else if (shadeMap[x, y] != previousShade)
                        {
                            instructions.Add(new CutterInstruction(Math.Floor(previousShade / 255d * LaserPower), (startX - x)));
                            startX = x;
                            previousShade = shadeMap[x, y];
                        }
                    }
                }
            }
            return instructions;
        }

        public override Border GetPreviewElement()
        {
            UpdateBWMap();
            double scalingFactor = scale * previewScale;
            Canvas canvas = new Canvas();
            int width = shadeMap.GetLength(0);
            int height = shadeMap.GetLength(1);
            byte previousShade;
            int startX;
            for (int y = 0; y < height; y += 1)
            {
                if (y % 2 == 0)
                {
                    startX = 0;
                    previousShade = shadeMap[0, y];
                    for (int x = 0; x < width; x += 1)
                    {
                        if (x == width - 1)
                        {
                            Windows.UI.Xaml.Shapes.Rectangle rect = new Windows.UI.Xaml.Shapes.Rectangle() { Width = (width - startX) * scalingFactor, Height = scalingFactor, Fill = new SolidColorBrush(Windows.UI.Color.FromArgb(previousShade, 32, 8, 0)) };
                            Canvas.SetLeft(rect, startX * scalingFactor);
                            Canvas.SetTop(rect, y * scalingFactor);
                            canvas.Children.Add(rect);
                        }
                        else if (shadeMap[x, y] != previousShade)
                        {
                            Windows.UI.Xaml.Shapes.Rectangle rect = new Windows.UI.Xaml.Shapes.Rectangle() { Width = (x - startX) * scalingFactor, Height = scalingFactor, Fill = new SolidColorBrush(Windows.UI.Color.FromArgb(previousShade, 32, 8, 0)) };
                            Canvas.SetLeft(rect, startX * scalingFactor);
                            Canvas.SetTop(rect, y * scalingFactor);
                            canvas.Children.Add(rect);

                            startX = x;
                            previousShade = shadeMap[x, y];
                        }
                    }
                }
                else
                {
                    startX = width - 1;
                    previousShade = shadeMap[width - 1, y];
                    for (int x = width - 1; x >= 0; x--)
                    {
                        if (x == 0)
                        {
                            Windows.UI.Xaml.Shapes.Rectangle rect = new Windows.UI.Xaml.Shapes.Rectangle() { Width = (startX + 1) * scalingFactor, Height = scalingFactor, Fill = new SolidColorBrush(Windows.UI.Color.FromArgb(previousShade, 32, 8, 0)) };
                            Canvas.SetLeft(rect, 0);
                            Canvas.SetTop(rect, y * scalingFactor);
                            canvas.Children.Add(rect);
                        }
                        else if (shadeMap[x, y] != previousShade)
                        {
                            Windows.UI.Xaml.Shapes.Rectangle rect = new Windows.UI.Xaml.Shapes.Rectangle() { Width = (startX - x) * scalingFactor, Height = scalingFactor, Fill = new SolidColorBrush(Windows.UI.Color.FromArgb(previousShade, 32, 8, 0)) };
                            Canvas.SetLeft(rect, (x + 1) * scalingFactor);
                            Canvas.SetTop(rect, y * scalingFactor);
                            canvas.Children.Add(rect);

                            startX = x;
                            previousShade = shadeMap[x, y];
                        }
                    }
                }

            }
            return new Border() { Child = canvas, Width = base.width * previewScale + 4, Height = base.height * previewScale + 4, Margin = new Thickness(-2, -2, 0, 0), BorderBrush = new SolidColorBrush(Windows.UI.Colors.Blue), BorderThickness = new Thickness(2) }; 
        }
    }

    public class CutterInstruction
    {
        const double scaleConst = 336;
        public enum InstructionType
        {
            velocityTime,
            penPower,
            printLine,
            printLineSegment
        }

        public readonly Windows.Foundation.Point Point;
        public readonly InstructionType type;
        public readonly int LaserPower;
        public int NominalWidth;

        public CutterInstruction(Windows.Foundation.Point point, double offsetX = 0, double offsetY = 0)
        {
            type = InstructionType.velocityTime;
            point.X += offsetX;
            point.Y += offsetY;
            Point = point;
        }

        public CutterInstruction(double laserPower)
        {
            type = InstructionType.penPower;
            LaserPower = (int)laserPower;
        }

        public CutterInstruction(double targetX, int nominalWidth,  double offestX = 0)
        {
            type = InstructionType.printLine;
            Point = new Windows.Foundation.Point(targetX + offestX, 0);
            NominalWidth = nominalWidth;
        }

        public CutterInstruction(double laserPower, int count)
        {
            type = InstructionType.printLineSegment;
            LaserPower = (int)laserPower;
            NominalWidth = count;
        }

        private double diffX, diffY, theta, vX, vY, t;
        public string GetArduinoMessage(double currentX = 0, double currentY = 0)
        {
            string msg;
            if (type == InstructionType.velocityTime)
            {
                (vX, vY) = GetComponentVelocities(currentX, currentY);
                t = GetTime(currentX, currentY);
                
                if (GetTime(currentX, currentY) != 0)
                    msg = $"({vX},{vY},{t});";
                else
                    return null;
            }
            else if (type == InstructionType.penPower)
            {
                msg = $"SetPower({LaserPower});";
            }
            else if (type == InstructionType.printLineSegment)
            {
                msg = $"{LaserPower}:{NominalWidth}!";
            }
            else
            {
                diffX = Point.X - currentX;
                t = Math.Round(diffX * scaleConst, 2);
                msg = $"PrintLine({t},{NominalWidth});";
            }
            return msg;
        }

        public (double, double) GetComponentVelocities(double currentX, double currentY)
        {
            diffX = Point.X - currentX;
            diffY = Point.Y - currentY;
            if (diffX != 0 || diffY != 0)
            {
                theta = Math.Atan2(diffY, diffX);
                vX = Math.Round(Math.Cos(theta),4);
                vY = Math.Round(Math.Sin(theta),4);
            }
            else  // Same Location
            {
                (vX, vY) = (0, 0);
            }
            return (vX, vY);
        }

        public double GetTime(double currentX, double currentY)
        {
            diffX = Point.X - currentX;
            diffY = Point.Y - currentY;
            double displacement = Math.Sqrt(Math.Pow(diffX, 2) + Math.Pow(diffY, 2));
            return Math.Round(displacement * scaleConst,2);
        }

        public static readonly CutterInstruction PenUpInstruction = new CutterInstruction(0);
        public static readonly CutterInstruction ReturnToOriginInstruction = new CutterInstruction(new Windows.Foundation.Point(0, 0), 0, 0);
        public static string StopMessage = "STOP;";
    }

    public abstract class ProjectObj 
    {
        public double X;
        public double Y;
        public double OriginalWidth;
        public double OriginalHeight;
        public double Width;
        public double Height;
        public int LaserPower;
    }

    public class SVGProjectObj : ProjectObj
    {
        public SVGProjectObj(double x, double y, double oW, double oH, double w, double h, int lP, int s, int p, List<string> pathData)
        {
            X = x;
            Y = y;
            OriginalWidth = oW;
            OriginalHeight = oH;
            Width = w;
            Height = h;
            LaserPower = lP;
            Smoothness = s;
            Passes = p;
            PathData = pathData;
        }

        public List<string> PathData;
        public int Smoothness;
        public int Passes;
    }

    public class BitmapProjectObj : ProjectObj
    {
        public BitmapProjectObj(double x, double y, double oW, double oH, double w, double h, int lP, int s, int t, bool iN, byte[] pixelData)
        {
            X = x;
            Y = y;
            OriginalWidth = oW;
            OriginalHeight = oH;
            Width = w;
            Height = h;
            LaserPower = lP;
            Shades = s;
            Threshold = t;
            IsNegative = iN;
            PixelData = pixelData;
        }

        public byte[] PixelData;
        public int Threshold;
        public int Shades;
        public bool IsNegative;
    }

    public class LaserLaunchPadProject
    {
        public LaserLaunchPadProject()
        {
            SVGProjectObjs = new List<SVGProjectObj>();
            BitmapProjectObjs = new List<BitmapProjectObj>();
        }

        public List<SVGProjectObj> SVGProjectObjs;
        public List<BitmapProjectObj> BitmapProjectObjs;
    }

}