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
using System.Timers;

namespace LaunchPad
{
    public sealed partial class MainPage : Page
    {
        public MainPage()
        {
            this.InitializeComponent();
            filePicker = new FileOpenPicker();
            designs = new List<Design>();
            cutterSerial = new SerialPortInput();
            cutterSerial.MessageReceived += CutterSerial_MessageReceived;
        }

        private StorageFile imageFile;
        private readonly FileOpenPicker filePicker;
        private readonly List<Design> designs;
        private Design selectedDesign;
        private static SerialPortInput cutterSerial;

        private async void LoadRasterButton_Click(object sender, RoutedEventArgs e)
        {
            filePicker.FileTypeFilter.Clear();
            filePicker.FileTypeFilter.Add(".bmp");
            filePicker.FileTypeFilter.Add(".png");
            filePicker.FileTypeFilter.Add(".jpg");
            imageFile = await filePicker.PickSingleFileAsync();
            if (imageFile != null)
            {
                using (IRandomAccessStream stream = await imageFile.OpenAsync(FileAccessMode.ReadWrite))
                {
                    BitmapDecoder decoder = await BitmapDecoder.CreateAsync(stream);
                    if (decoder.BitmapPixelFormat == BitmapPixelFormat.Bgra8)
                    {
                        selectedDesign = new BitmapDesign(decoder, await decoder.GetPixelDataAsync());
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
            filePicker.SuggestedStartLocation = PickerLocationId.Desktop;
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
                Canvas.SetLeft(border, design.X);
                Canvas.SetTop(border, design.Y);
                border.Tag = designs.IndexOf(design);
                ((Canvas)border.Child).Tag = designs.IndexOf(design);
                border.Tapped += PreviewBorder_Tapped;
                border.Child.Tapped += PreviewCanvas_Tapped;
                PreviewCanvas.Children.Add(border);
            }

            XPosTextBox.Text = selectedDesign.X.ToString();
            YPosTextBox.Text = selectedDesign.Y.ToString();
            WidthTextBox.Text = selectedDesign.Width.ToString();
            HeightTextBox.Text = selectedDesign.Height.ToString();
            ScaleTextBox.Text = selectedDesign.Scale.ToString();
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

        private List<CutterInstruction> instructions = new List<CutterInstruction>();
        private void PrintButton_Click(object sender, RoutedEventArgs e)
        {
            if (instructions != null && cutterSerial != null)
            {
                instructions.Clear();
                if (!cutterSerial.IsConnected)
                {
                    foreach (Design design in designs)
                    {
                        instructions.AddRange(design.GetArduinoInstructions());
                    }
                    instructions.Add(CutterInstruction.PenUpInstruction);
                    instructions.Add(CutterInstruction.ReturnToOriginInstruction);
                    cutterSerial.Connect();
                }
            }
        }


        // Cut-Time Variables
        private double laserX = 0;
        private double laserY = 0;
        private bool laserOn = false;
        private double vX = 0;
        private double vY = 0;
        private double destX, destY = 0;

        private void CutterSerial_MessageReceived(object sender, MessageReceivedEventArgs e)
        {
            string resp = Encoding.ASCII.GetString(e.Data);
            if (resp == ":)")
            {
                if (instructions.Any())
                {
                    string msg;
                    CutterInstruction currentInstruction;
                    do
                    {
                        currentInstruction = instructions.First();
                        if (currentInstruction.IsCoord)
                        {
                            msg = currentInstruction.GetArduinoMessage(destX, destY);
                            (vX, vY) = currentInstruction.GetComponentVelocities(destX, destY);
                            (laserX, laserY) = (destX, destY);
                            (destX, destY) = (currentInstruction.Point.X, currentInstruction.Point.Y);
                        }
                        else
                        {
                            msg = currentInstruction.GetArduinoMessage();
                            laserOn = currentInstruction.LaserPower > 0;
                            (vX, vY) = (0, 0);
                        }
                        instructions.Remove(currentInstruction);
                    } while (string.IsNullOrWhiteSpace(msg) && instructions.Any());

                    System.Diagnostics.Debug.WriteLine(msg);
                    cutterSerial.SendMessage(Encoding.ASCII.GetBytes(msg));
                }
                else
                {
                    //cutterSerial.SendMessage(Encoding.ASCII.GetBytes(CutterInstruction.ZeroString)); // Send Speed of Zero
                    System.Diagnostics.Debug.WriteLine("Print Complete");
                    (vX, vY) = (0, 0);
                }
            }
            else
            {
                System.Diagnostics.Debug.WriteLine(resp);
            }
        }

        private async void PortComboBox_DropDownOpened(object sender, object e)
        {
            string aqsFilter = SerialDevice.GetDeviceSelector();
            var devices = await DeviceInformation.FindAllAsync(aqsFilter);
            if (devices.Any())
                PortComboBox.ItemsSource = devices;
        }

        private void XPosTextBox_TextChanged(object sender, TextChangedEventArgs e)
        {
            if(double.TryParse(XPosTextBox.Text,out double xPos))
            {
                if (xPos != selectedDesign.X)
                {
                    selectedDesign.X = xPos;
                    UpdatePreview();
                }
            }
        }

        private void YPosTextBox_TextChanged(object sender, TextChangedEventArgs e)
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

        private void WidthTextBox_TextChanged(object sender, TextChangedEventArgs e)
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

        private void HeightTextBox_TextChanged(object sender, TextChangedEventArgs e)
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

        private void ScaleTextBox_TextChanged(object sender, TextChangedEventArgs e)
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
                    ((BitmapDesign)selectedDesign).Threshold = (uint)e.NewValue;
                    UpdatePreview();
                }
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

        private void PortComboBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (PortComboBox.SelectedItem != null)
            {
                DeviceInformation info = (DeviceInformation)PortComboBox.SelectedItem;
                string portName = info.Name.Substring(info.Name.IndexOf("COM"), 4);
                cutterSerial.SetPort(portName, 9600);
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
        private readonly bool ignoreAlpha;
        private readonly List<BitmapPixel> pixels;

        public BitmapEditor(PixelDataProvider pixelDataProvider, uint width, uint height, bool ignoreAlpha)
        {
            byte[] pixelData = pixelDataProvider.DetachPixelData();
            Width = (int)width;
            Height = (int)height;
            this.ignoreAlpha = ignoreAlpha;
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

        public bool[,] GetBWMap(uint threshold, bool printNegative)
        {
            bool[,] bwMap = new bool[Width, Height];
            for (int y = 0; y < Height; y++)
            {
                for (int x = 0; x < Width; x++)
                {
                    BitmapPixel px = GetPixel(x, y);
                    uint mean = 255 - ((px.R + px.G + px.B) / 3);
                    //mean = printNegative ? 255 - mean : mean;
                    //System.Diagnostics.Debug.WriteLine($"({px.R},{px.G},{px.B},{px.A}):{mean}");
                    bool isWhite = mean >= threshold && (ignoreAlpha || px.A >= threshold);
                    bwMap[x, y] = (isWhite != printNegative);
                }
            }
            return bwMap;
        }
    }

    public abstract class Design
    {
        public double X { get; set; } = 0;
        public double Y { get; set; } = 0;

        protected double originalWidth;
        protected double originalHeight;

        protected double width;
        protected double height;
        protected double scale = 1;

        public double Scale { get => scale; set { scale = value; width = originalWidth * scale; height = originalHeight * scale; } }
        public double Width { get => width; set { width = value; scale = width / originalWidth; height = originalHeight * scale; } }
        public double Height { get => height; set { height = value; scale = height / originalHeight; width = originalWidth * scale; } }
        public int LaserPower { get; set; } = 128;

        public abstract List<CutterInstruction> GetArduinoInstructions();

        public abstract Border GetPreviewElement();
    }

    public class SVGDesign : Design
    {
        public SVGDesign(XmlDocument xDoc)
        {
            paths = new List<SvgPathSegmentList>();
            points = new List<PointCollection>();
            XmlNodeList Paths = xDoc.GetElementsByTagName("path");
            foreach (XmlElement path in Paths)
            {
                string pathData = path.GetAttribute("d");
                SvgPathSegmentList segments = SvgPathBuilder.Parse(pathData);
                paths.Add(segments);
            }
            var dims = ((XmlElement)xDoc.GetElementsByTagName("svg")[0]).GetAttribute("viewBox").Split(' ');
            (originalWidth, originalHeight) = (double.Parse(dims[2]), double.Parse(dims[3]));
            (width, height) = (originalWidth, originalHeight);
            UpdatePoints();
        }

        private float step { get => MathF.Pow(1.2f, -Smoothness); }

        public int Smoothness { get; set; } = 1;
        public int Passes { get; set; } = 1;

        private List<SvgPathSegmentList> paths;
        private List<PointCollection> points;

        private void UpdatePoints()
        {
            points.Clear();
            foreach (SvgPathSegmentList path in paths)
            {
                foreach (SvgPathSegment segment in path)
                {
                    if (segment.GetType() == typeof(SvgMoveToSegment))
                    {
                        var seg = (SvgMoveToSegment)segment;
                        points.Add(GetMoveToPointCollection(seg));
                    }
                    else if (segment.GetType() == typeof(SvgLineSegment))
                    {
                        var seg = (SvgLineSegment)segment;
                        points.Last().Add(GetLineToPoint(seg).ToFoundationPoint(scale));
                    }
                    else if (segment.GetType() == typeof(SvgQuadraticCurveSegment))
                    {
                        var seg = (SvgQuadraticCurveSegment)segment;
                        for (float t = 0; t < 1; t += step)
                        {
                            points.Last().Add(GetQuadraticPoint(seg, t).ToFoundationPoint(scale));
                        }
                        points.Last().Add(seg.End.ToFoundationPoint(scale));
                    }
                    else if (segment.GetType() == typeof(SvgCubicCurveSegment))
                    {
                        var seg = (SvgCubicCurveSegment)segment;
                        for (float t = 0; t < 1; t += step)
                        {
                            points.Last().Add(GetCubicPoint(seg, t).ToFoundationPoint(scale));
                        }
                        points.Last().Add(seg.End.ToFoundationPoint(scale));
                    }
                    else if (segment.GetType() == typeof(SvgArcSegment))
                    {
                        var seg = (SvgArcSegment)segment;
                        var arcData = new CenterParameterizedArcData(seg);
                        // Get Points for Polyline Approximation
                        for (float t = 0; t < 1; t += step)
                        {
                            points.Last().Add(GetArcPoint(arcData, t).ToFoundationPoint(scale));
                        }
                        points.Last().Add(seg.End.ToFoundationPoint(scale));
                    }
                    else if (segment.GetType() == typeof(SvgClosePathSegment))
                    {
                        var seg = (SvgClosePathSegment)segment;

                        if (points.Last().First() != seg.Start.ToFoundationPoint(scale)) // if length back to start is not zero
                        {
                            points.Last().Add(GetClosePathPoint().ToFoundationPoint(1));
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

            foreach(PointCollection pointCollection in points)
            {
                canvas.Children.Add(new Windows.UI.Xaml.Shapes.Polyline()
                {
                    Stroke = new SolidColorBrush(Windows.UI.Colors.Black),
                    StrokeThickness = 2,
                    Points = pointCollection
                });
            }
            return new Border() { Child = canvas, Width = width + 4, Height = height + 4, Margin = new Thickness(-2, -2, 0, 0), BorderBrush = new SolidColorBrush(Windows.UI.Colors.Blue), BorderThickness = new Thickness(2) };
        }

        private PointF ColinearAtTime(PointF A, PointF B, float t)
        {
            float x = ((1 - t) * A.X) + (t * B.X);
            float y = ((1 - t) * A.Y) + (t * B.Y);
            return new PointF(x, y);
        }

        private PointCollection GetMoveToPointCollection(SvgMoveToSegment seg)
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
        public BitmapDesign(BitmapDecoder decoder, PixelDataProvider pixelData)
        {
            bitmapEditor = new BitmapEditor(pixelData, decoder.PixelWidth, decoder.PixelHeight, decoder.BitmapAlphaMode == BitmapAlphaMode.Ignore);
            (originalWidth, originalHeight) = (decoder.PixelWidth, decoder.PixelHeight);
            (width, height) = (originalWidth, originalHeight);
            UpdateBWMap();
        }
        private readonly BitmapEditor bitmapEditor;
        private bool[,] bwMap;

        public uint Threshold { get; set; } = 128;
        public bool IsNegative { get; set; } = false;

        private void UpdateBWMap() => bwMap = bitmapEditor.GetBWMap(Threshold, IsNegative);

        public override List<CutterInstruction> GetArduinoInstructions()
        {
            UpdateBWMap();
            List<CutterInstruction> instructions = new List<CutterInstruction>();
            int width = bwMap.GetLength(0);
            int height = bwMap.GetLength(1);
            bool penDown = false;
            for (int y = 0; y < height; y++)
            {
                if (y % 2 == 0)
                {
                    for (int x = 0; x < width; x++)
                    {
                        if (bwMap[x, y] && !penDown)
                        {
                            instructions.Add(new CutterInstruction(new Windows.Foundation.Point(x * scale, y * scale), X, Y));
                            instructions.Add(new CutterInstruction(LaserPower));
                            penDown = true;
                        }
                        else if (x == width - 1 && penDown)
                        {
                            instructions.Add(new CutterInstruction(new Windows.Foundation.Point((width - 1) * scale, y * scale), X, Y));
                            instructions.Add(CutterInstruction.PenUpInstruction);
                            penDown = false;
                        }
                        else if (!bwMap[x, y] && penDown)
                        {
                            instructions.Add(new CutterInstruction(new Windows.Foundation.Point((x - 1) * scale, y * scale), X, Y));
                            instructions.Add(CutterInstruction.PenUpInstruction);
                            penDown = false;
                        }
                    }
                }
                else
                {
                    for (int x = width - 1; x >= 0; x--)
                    {
                        if (bwMap[x, y] && !penDown)
                        {
                            instructions.Add(new CutterInstruction(new Windows.Foundation.Point(x * scale, y * scale), X, Y));
                            instructions.Add(new CutterInstruction(LaserPower));
                            penDown = true;
                        }
                        else if (x == 0 && penDown)
                        {
                            instructions.Add(new CutterInstruction(new Windows.Foundation.Point(0, y * scale), X, Y));
                            instructions.Add(CutterInstruction.PenUpInstruction);
                            penDown = false;
                        }
                        else if (!bwMap[x, y] && penDown)
                        {
                            instructions.Add(new CutterInstruction(new Windows.Foundation.Point((x + 1) * scale, y * scale), X, Y));
                            instructions.Add(CutterInstruction.PenUpInstruction);
                            penDown = false;
                        }
                    }
                }
            }
            return instructions;
        }

        public override Border GetPreviewElement()
        {
            UpdateBWMap();
            Canvas canvas = new Canvas();
            int width = bwMap.GetLength(0);
            int height = bwMap.GetLength(1);
            bool penDown;
            int startX = 0;
            for (int y = 0; y < height; y++)
            {
                penDown = false;
                if (y % 2 == 0)
                {
                    for (int x = 0; x < width; x++)
                    {
                        if (bwMap[x, y] && !penDown)
                        {
                            startX = x;
                            penDown = true;
                        }
                        else if (x == width - 1 && penDown)
                        {
                            Windows.UI.Xaml.Shapes.Rectangle rect = new Windows.UI.Xaml.Shapes.Rectangle() { Width = (width - startX) * scale, Height = scale, Fill = new SolidColorBrush(Windows.UI.Colors.Black) };
                            Canvas.SetLeft(rect, startX * scale);
                            Canvas.SetTop(rect, y * scale);
                            canvas.Children.Add(rect);
                            penDown = false;
                        }
                        else if (!bwMap[x, y] && penDown)
                        {
                            Windows.UI.Xaml.Shapes.Rectangle rect = new Windows.UI.Xaml.Shapes.Rectangle() { Width = ((x - 1) - startX) * scale, Height = scale, Fill = new SolidColorBrush(Windows.UI.Colors.Black) };
                            Canvas.SetLeft(rect, startX * scale);
                            Canvas.SetTop(rect, y * scale);
                            canvas.Children.Add(rect);
                            penDown = false;
                        }
                    }
                }
                else
                {
                    for (int x = width - 1; x >= 0; x--)
                    {
                        if (bwMap[x, y] && !penDown)
                        {
                            startX = x;
                            penDown = true;
                        }
                        else if (x == 0 && penDown)
                        {
                            Windows.UI.Xaml.Shapes.Rectangle rect = new Windows.UI.Xaml.Shapes.Rectangle() { Width = startX * scale, Height = scale, Fill = new SolidColorBrush(Windows.UI.Colors.Black) };
                            Canvas.SetTop(rect, y * scale);
                            canvas.Children.Add(rect);
                            penDown = false;
                        }
                        else if (!bwMap[x, y] && penDown)
                        {
                            Windows.UI.Xaml.Shapes.Rectangle rect = new Windows.UI.Xaml.Shapes.Rectangle() { Width = (startX - (x + 1)) * scale, Height = scale, Fill = new SolidColorBrush(Windows.UI.Colors.Black) };
                            Canvas.SetLeft(rect, (x + 1) * scale);
                            Canvas.SetTop(rect, y * scale);
                            canvas.Children.Add(rect);
                            penDown = false;
                        }
                    }
                }

            }
            return new Border() { Child = canvas, Width = base.width + 4, Height = base.height + 4, Margin = new Thickness(-2, -2, 0, 0), BorderBrush = new SolidColorBrush(Windows.UI.Colors.Blue), BorderThickness = new Thickness(2) }; 
        }
    }

    public class CutterInstruction
    {
        const double velocityConstant = 1800;

        public readonly Windows.Foundation.Point Point;
        public readonly bool IsCoord;
        public readonly int LaserPower;

        public CutterInstruction(Windows.Foundation.Point point, double offsetX, double offsetY)
        {
            IsCoord = true;
            point.X += offsetX;
            point.Y += offsetY;
            Point = point;
        }

        public CutterInstruction(int laserPower)
        {
            IsCoord = false;
            LaserPower = laserPower;
        }

        private double diffX, diffY, theta, vX, vY, t;
        public string GetArduinoMessage(double currentX = 0, double currentY = 0)
        {
            string msg;
            if (IsCoord)
            {
                (vX, vY) = GetComponentVelocities(currentX, currentY);
                t = GetTime(currentX, currentY);
                if (t != 0)
                    msg = $"({vX},{vY},{t});";
                else
                    return null;
            }
            else
            {
                msg = $"SetPower({LaserPower});";
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
                vX = Math.Cos(theta);
                vY = Math.Sin(theta);
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
            return displacement * velocityConstant;
        }

        public static readonly CutterInstruction PenUpInstruction = new CutterInstruction(0);
        public static readonly CutterInstruction ReturnToOriginInstruction = new CutterInstruction(new Windows.Foundation.Point(0, 0), 0, 0);
    }
}