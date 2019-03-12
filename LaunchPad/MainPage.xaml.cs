﻿using System;
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

namespace LaunchPad
{
    public sealed partial class MainPage : Page
    {
        // Constants
        const string PEN_UP_MESSAGE = "PenUp();";
        const string PEN_DOWN_MESSAGE = "PenDown();";
        const string NEXT_INSTRUCTION_MESSAGE = ":)";
        const string FINISH_DRAWING_MESSAGE = "That will do, cheers bud :);";
        const string DRAWING_FINISHED_MESSAGE = "(:";

        public MainPage()
        {
            this.InitializeComponent();
            filePicker = new FileOpenPicker();
            cutterSerial = new SerialPortInput();
            cutterSerial.MessageReceived += CutterSerial_MessageReceived;
        }

        StorageFile imageFile;
        FileOpenPicker filePicker;
        XmlDocument xDoc = new XmlDocument();
        PointF currentStartPoint;
        float Step { get { return 1 / (float)SmoothSlider.Value; } }
        List<string> instructions;
        BitmapEditor bitmapEditor;
        double currentScale = 1;
        SerialPortInput cutterSerial;

        PointF ColinearAtTime(PointF A, PointF B, float t)
        {
            float x = ((1 - t) * A.X) + (t * B.X);
            float y = ((1 - t) * A.Y) + (t * B.Y);
            return new PointF(x, y);
        }

        List<SvgPathSegmentList> GetPathsFromSvg()
        {
            List<SvgPathSegmentList> paths = new List<SvgPathSegmentList>();
            XmlNodeList Paths = xDoc.GetElementsByTagName("path");
            foreach (XmlElement path in Paths)
            {
                string pathData = path.GetAttribute("d");
                SvgPathSegmentList segments = SvgPathBuilder.Parse(pathData);
                paths.Add(segments);
            }
            return paths;
        }

        List<string> GetArduinoInstructions(List<SvgPathSegmentList> paths)
        {
            List<string> instructions = new List<string>();
            foreach (SvgPathSegmentList path in paths)
            {
                foreach (SvgPathSegment segment in path)
                {
                    if (segment.GetType() == typeof(SvgMoveToSegment))
                    {
                        var seg = (SvgMoveToSegment)segment;
                        instructions.Add(PEN_UP_MESSAGE);
                        instructions.Add(seg.End.ToCoordString(currentScale));
                        instructions.Add(PEN_DOWN_MESSAGE);
                        currentStartPoint = seg.End;

                    }
                    else if (segment.GetType() == typeof(SvgLineSegment))
                    {
                        var seg = (SvgLineSegment)segment;
                        instructions.Add(GetLineToPoint(seg).ToCoordString(currentScale));
                    }
                    else if (segment.GetType() == typeof(SvgQuadraticCurveSegment))
                    {
                        var seg = (SvgQuadraticCurveSegment)segment;
                        for (float t = 0; t <= 1; t += Step)
                        {
                            instructions.Add(GetQuadraticPoint(seg, t).ToCoordString(currentScale));
                        }
                        instructions.Add(seg.End.ToCoordString(currentScale));
                    }
                    else if (segment.GetType() == typeof(SvgCubicCurveSegment))
                    {
                        var seg = (SvgCubicCurveSegment)segment;
                        for (float t = 0; t <= 1; t += Step)
                        {
                            instructions.Add(GetCubicPoint(seg, t).ToCoordString(currentScale));
                        }
                        instructions.Add(seg.End.ToCoordString(currentScale));
                    }
                    else if (segment.GetType() == typeof(SvgArcSegment))
                    {
                        var seg = (SvgArcSegment)segment;

                        var arcData = new CenterParameterizedArcData(seg);
                        for (float t = 0; t <= 1; t += Step)
                        {
                            instructions.Add(GetArcPoint(arcData, t).ToCoordString(currentScale));
                        }
                        instructions.Add(seg.End.ToCoordString(currentScale));
                    }
                    else if (segment.GetType() == typeof(SvgClosePathSegment))
                    {
                        var seg = (SvgClosePathSegment)segment;

                        if (currentStartPoint != seg.Start) // if length back to start is not zero
                        {
                            instructions.Add(GetClosePathPoint().ToCoordString(currentScale));
                        }
                        instructions.Add(PEN_UP_MESSAGE);

                    }
                }
            }
            return instructions;
        }

        List<PointCollection> GetPreviewPoints(List<SvgPathSegmentList> paths)
        {
            List<PointCollection> PreviewPoints = new List<PointCollection>();
            foreach (SvgPathSegmentList path in paths)
            {
                foreach (SvgPathSegment segment in path)
                {
                    if (segment.GetType() == typeof(SvgMoveToSegment))
                    {
                        var seg = (SvgMoveToSegment)segment;
                        currentStartPoint = seg.End;
                        PreviewPoints.Add(GetMoveToPointCollection(seg));

                    }
                    else if (segment.GetType() == typeof(SvgLineSegment))
                    {
                        var seg = (SvgLineSegment)segment;
                        PreviewPoints.Last().Add(GetLineToPoint(seg).ToFoundationPoint(currentScale));
                    }
                    else if (segment.GetType() == typeof(SvgQuadraticCurveSegment))
                    {
                        var seg = (SvgQuadraticCurveSegment)segment;
                        for (float t = 0; t < 1; t += Step)
                        {
                            PreviewPoints.Last().Add(GetQuadraticPoint(seg, t).ToFoundationPoint(currentScale));
                        }
                        PreviewPoints.Last().Add(seg.End.ToFoundationPoint(currentScale));
                    }
                    else if (segment.GetType() == typeof(SvgCubicCurveSegment))
                    {
                        var seg = (SvgCubicCurveSegment)segment;
                        for (float t = 0; t < 1; t += Step)
                        {
                            PreviewPoints.Last().Add(GetCubicPoint(seg, t).ToFoundationPoint(currentScale));
                        }
                        PreviewPoints.Last().Add(seg.End.ToFoundationPoint(currentScale));
                    }
                    else if (segment.GetType() == typeof(SvgArcSegment))
                    {
                        var seg = (SvgArcSegment)segment;
                        var arcData = new CenterParameterizedArcData(seg);
                        System.Diagnostics.Debug.WriteLine(seg.Size);
                        System.Diagnostics.Debug.WriteLine($"Centre ({arcData.cx},{arcData.cy}), Radius ({arcData.rx},{arcData.ry})");
                        // Get Points for Polyline Approximation
                        for (float t = 0; t < 1; t += Step)
                        {
                            PreviewPoints.Last().Add(GetArcPoint(arcData, t).ToFoundationPoint(currentScale));
                        }
                        PreviewPoints.Last().Add(seg.End.ToFoundationPoint(currentScale));
                    }
                    else if (segment.GetType() == typeof(SvgClosePathSegment))
                    {
                        var seg = (SvgClosePathSegment)segment;

                        if (currentStartPoint != seg.Start) // if length back to start is not zero
                        {
                            PreviewPoints.Last().Add(GetClosePathPoint().ToFoundationPoint(currentScale));
                        }
                    }
                }
            }
            return PreviewPoints;
        }

        PointCollection GetMoveToPointCollection(SvgMoveToSegment seg)
        {
            return new PointCollection() { seg.End.ToFoundationPoint(currentScale) };
        }

        PointF GetLineToPoint(SvgLineSegment seg)
        {
            return seg.End;
        }

        PointF GetQuadraticPoint(SvgQuadraticCurveSegment seg, float t)
        {
            PointF SC = ColinearAtTime(seg.Start, seg.ControlPoint, t);
            PointF CE = ColinearAtTime(seg.ControlPoint, seg.End, t);
            PointF point = ColinearAtTime(SC, CE, t);

            return point;
        }

        PointF GetCubicPoint(SvgCubicCurveSegment seg, float t)
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


        PointF GetArcPoint(CenterParameterizedArcData arcData, float t)
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

        PointF GetClosePathPoint()
        {
            return currentStartPoint;
        }

        private void DisplayPreview(List<PointCollection> previewPoints)
        {
            PreviewCanvas.Children.Clear();
            foreach (PointCollection points in previewPoints)
            {
                PreviewCanvas.Children.Add(new Windows.UI.Xaml.Shapes.Polyline()
                {
                    Stroke = new SolidColorBrush(Windows.UI.Colors.Black),
                    StrokeThickness = 2,
                    Width = 512,
                    Height = 512,
                    Points = points
                });
            }
        }

        private void DisplayPreview(bool[,] bwMap)
        {
            List<Windows.UI.Xaml.Shapes.Rectangle> prevLines = GetPreviewRects(bwMap, (int)LaminationSlider.Value);
            PreviewCanvas.Children.Clear();
            foreach (var rect in prevLines)
            {
                rect.Fill = new SolidColorBrush(Windows.UI.Colors.Black);
                rect.StrokeThickness = 0;
                PreviewCanvas.Children.Add(rect);
            }
        }

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
                        FileTextBlock.Text = string.Empty;
                        FileTextScrollView.Visibility = Visibility.Collapsed;
                        FileTextScrollTitleBlock.Visibility = Visibility.Collapsed;
                        SmoothSlider.Visibility = Visibility.Collapsed;
                        PassCountSlider.Visibility = Visibility.Collapsed;
                        ThresholdSlider.Visibility = Visibility.Visible;
                        LaminationSlider.Visibility = Visibility.Visible;
                        NegativeSwitch.Visibility = Visibility.Visible;
                        PrintButton.IsEnabled = true;

                        System.Diagnostics.Debug.WriteLine(decoder.BitmapAlphaMode);
                        PixelDataProvider pixelData = await decoder.GetPixelDataAsync();
                        bitmapEditor = new BitmapEditor(pixelData, decoder.PixelWidth, decoder.PixelHeight, decoder.BitmapAlphaMode == BitmapAlphaMode.Ignore);
                        bool[,] bwMap = bitmapEditor.GetBWMap((uint)ThresholdSlider.Value, NegativeSwitch.IsOn);
                        DisplayPreview(bwMap);
                        instructions = GetArduinoInstructions(bwMap, (int)LaminationSlider.Value);
                    }
                }
            }
        }

        public List<Windows.UI.Xaml.Shapes.Rectangle> GetPreviewRects(bool[,] bwMap, int lamination)
        {
            List<Windows.UI.Xaml.Shapes.Rectangle> rects = new List<Windows.UI.Xaml.Shapes.Rectangle>();
            int width = bwMap.GetLength(0);
            int height = bwMap.GetLength(1);
            int lineStart;                    // Equals width when a line hasn't been started
            for (int y = 0; y < height; y += lamination)
            {
                lineStart = width;
                for (int x = 0; x < width; x += 1)
                {
                    if (bwMap[x, y])
                    {
                        if(lineStart == width) // Not currently drawing
                        {
                            lineStart = x;
                        }
                        else if(x == width - 1)
                        {
                            var rect = new Windows.UI.Xaml.Shapes.Rectangle() { Width = (x - 1 - lineStart) * currentScale, Height = lamination * currentScale };
                            Canvas.SetTop(rect, y * currentScale);
                            Canvas.SetLeft(rect, lineStart * currentScale);
                            rects.Add(rect);
                        }
                    }
                    else
                    {
                        if(lineStart != width) // Currently drawing
                        {
                            var rect = new Windows.UI.Xaml.Shapes.Rectangle() { Width = (x - 1 - lineStart) * currentScale, Height = lamination * currentScale };
                            Canvas.SetTop(rect, y * currentScale);
                            Canvas.SetLeft(rect, lineStart * currentScale);
                            rects.Add(rect);

                            lineStart = width;
                        }
                    }
                }
            }
            return rects;
        }

        List<string> GetArduinoInstructions(bool[,] bwMap, int lamination)
        {
            List<string> instructions = new List<string>();
            int width = bwMap.GetLength(0);
            int height = bwMap.GetLength(1);
            int lineStart;                    // Equals width when a line hasn't been started
            int startingX;
            int stepX;
            instructions.Add(PEN_UP_MESSAGE);
            instructions.Add("(0,0);");
            for (int y = 0; y < height; y += lamination)
            {
                lineStart = width;
                startingX = (y % 2 == 0) ? 0 : width - 1;
                stepX = (y % 2 == 0) ? 1 : -1;
                for (int x = startingX; (y % 2 == 0) ? x < width : x > 0; x += stepX)
                {
                    if (bwMap[x, y])
                    {
                        if (lineStart == width) // Not currenntly drawing
                        {
                            lineStart = x;
                            instructions.Add($"({x * currentScale},{y * currentScale});");
                            instructions.Add(PEN_DOWN_MESSAGE);
                        }
                        else if (x == width - (startingX + 1))
                        {
                            instructions.Add($"({x * currentScale},{y * currentScale});");
                            instructions.Add(PEN_UP_MESSAGE);
                        }
                    }
                    else
                    {
                        if (lineStart != width) // Currently drawing
                        {
                            instructions.Add($"({(x - stepX) * currentScale},{y * currentScale});");
                            instructions.Add(PEN_UP_MESSAGE);

                            lineStart = width;
                        }
                    }
                }
            }
            return instructions;
        }

        private async void LoadSVGButton_Click(object sender, RoutedEventArgs e)
        {
            filePicker.FileTypeFilter.Clear();
            filePicker.FileTypeFilter.Add(".svg");
            filePicker.SuggestedStartLocation = PickerLocationId.Desktop;
            imageFile = await filePicker.PickSingleFileAsync();
            if (imageFile != null) {
                string imageFileString = await FileIO.ReadTextAsync(imageFile);
                FileTextBlock.Text = imageFileString;
                ThresholdSlider.Visibility = Visibility.Collapsed;
                LaminationSlider.Visibility = Visibility.Collapsed;
                NegativeSwitch.Visibility = Visibility.Collapsed;
                FileTextScrollView.Visibility = Visibility.Visible;
                FileTextScrollTitleBlock.Visibility = Visibility.Visible;
                SmoothSlider.Visibility = Visibility.Visible;
                PassCountSlider.Visibility = Visibility.Visible;
                PrintButton.IsEnabled = true;

                xDoc.LoadXml(imageFileString);
                var paths = GetPathsFromSvg();
                DisplayPreview(GetPreviewPoints(paths));
                instructions = GetArduinoInstructions(paths);
            }
        }

        private void SmoothSlider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            if (imageFile != null)
            {
                var paths = GetPathsFromSvg();
                DisplayPreview(GetPreviewPoints(paths));
                instructions = GetArduinoInstructions(paths);
            }
        }

        List<string> fullInstructions = new List<string>();
        private void PrintButton_Click(object sender, RoutedEventArgs e)
        {
            if (instructions != null && cutterSerial != null)
            {
                fullInstructions.Clear();
                for (int pass = 0; pass < PassCountSlider.Value; pass++)
                {
                    foreach (string instruction in instructions)
                    {
                        fullInstructions.Add(instruction);
                    }
                }
                if (!cutterSerial.IsConnected)
                {
                    cutterSerial.Connect();
                }
                else
                {
                    cutterSerial.Disconnect();
                    fullInstructions.Clear();
                }
            }
        }

        private void CutterSerial_MessageReceived(object sender, MessageReceivedEventArgs e)
        {
            string resp = Encoding.ASCII.GetString(e.Data);
            System.Diagnostics.Debug.WriteLine(resp);
            if (resp == NEXT_INSTRUCTION_MESSAGE)
            {
                if (fullInstructions.Any())
                {
                    System.Diagnostics.Debug.WriteLine(fullInstructions.First());
                    byte[] ins = Encoding.ASCII.GetBytes(fullInstructions.First());
                    cutterSerial.SendMessage(ins);
                    fullInstructions.RemoveAt(0);
                }
                else
                {
                    byte[] msg = Encoding.ASCII.GetBytes(FINISH_DRAWING_MESSAGE);
                    cutterSerial.SendMessage(msg);
                }
            }
            else if (resp == DRAWING_FINISHED_MESSAGE)
            {
                System.Diagnostics.Debug.WriteLine("Print Complete");
                try
                {
                    cutterSerial.Disconnect();
                }
                catch { }
            }
            else
            {
                System.Diagnostics.Debug.WriteLine("Say What: " + resp);
            }
        }

        private async void PortComboBox_DropDownOpened(object sender, object e)
        {
            //var serialDevices = GXSerial.GetPortNames();
            //if (serialDevices.Any())
            //    PortComboBox.ItemsSource = serialDevices;
            string aqsFilter = SerialDevice.GetDeviceSelector();
            var devices = await DeviceInformation.FindAllAsync(aqsFilter);
            if (devices.Any())
                PortComboBox.ItemsSource = devices;
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

        private void ThresholdSlider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            if (bitmapEditor != null)
            {
                bool[,] bwMap = bitmapEditor.GetBWMap((uint)ThresholdSlider.Value, NegativeSwitch.IsOn);
                DisplayPreview(bwMap);
                instructions = GetArduinoInstructions(bwMap, (int)LaminationSlider.Value);
            }
        }

        private void NegativeSwitch_Toggled(object sender, RoutedEventArgs e)
        {
            if (bitmapEditor != null)
            {
                bool[,] bwMap = bitmapEditor.GetBWMap((uint)ThresholdSlider.Value, NegativeSwitch.IsOn);
                DisplayPreview(bwMap);
                instructions = GetArduinoInstructions(bwMap, (int)LaminationSlider.Value);
            }
        }

        private void LaminationSlider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            if (bitmapEditor != null)
            {
                bool[,] bwMap = bitmapEditor.GetBWMap((uint)ThresholdSlider.Value, NegativeSwitch.IsOn);
                DisplayPreview(bwMap);
                instructions = GetArduinoInstructions(bwMap, (int)LaminationSlider.Value);
            }
        }

        private void ScaleTextBox_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (double.TryParse(ScaleTextBox.Text.Replace("%",""), out double newScale))
            {
                if (newScale > 0)
                {
                    currentScale = newScale / 100;
                    if (imageFile != null)
                    {
                        if (PassCountSlider.Visibility == Visibility.Visible)
                        {
                            var paths = GetPathsFromSvg();
                            DisplayPreview(GetPreviewPoints(paths));
                            instructions = GetArduinoInstructions(paths);
                        }
                        else
                        {
                            bool[,] bwMap = bitmapEditor.GetBWMap((uint)ThresholdSlider.Value, NegativeSwitch.IsOn);
                            DisplayPreview(bwMap);
                            instructions = GetArduinoInstructions(bwMap, (int)LaminationSlider.Value);
                        }
                    }
                }
            }
        }

        private void ScaleUpButton_Click(object sender, RoutedEventArgs e)
        {
            currentScale += 0.1;
            ScaleTextBox.Text = (100 * currentScale).ToString() + "%";
        }

        private void ScaleDownButton_Click(object sender, RoutedEventArgs e)
        {
            if (currentScale > 0.1)
            {
                currentScale -= 0.1;
                ScaleTextBox.Text = (100 * currentScale).ToString() + "%";
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
            if (radius_check > 1) {
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
            dTheta = dTheta % (2 * MathF.PI);
            if (seg.Sweep == SvgArcSweep.Negative)
                dTheta -= 2 * MathF.PI;

        }
        // guidance: www.w3.org/TR/SVG/implnote.html#ArcSyntax
        // there is no easy way to do this except convert to centre parameterization lol 
    }

    public static class Extensions
    { 

        public static Windows.Foundation.Point ToFoundationPoint(this PointF point, double scale)
        {
            return new Windows.Foundation.Point(point.X * scale, point.Y * scale);
        }

        public static string ToCoordString(this PointF point, double scale)
        {
            return $"({point.X * scale},{point.Y * scale});";
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
            for(int i = 0; i < pixelData.Length; i += 4)
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
                    uint mean = ((px.R + px.G + px.B) / 3);
                    mean = printNegative ? 255 - mean : mean;
                    //System.Diagnostics.Debug.WriteLine($"({px.R},{px.G},{px.B},{px.A}):{mean}");
                    bool isWhite = mean >= threshold && (ignoreAlpha || px.A >= threshold);
                    bwMap[x,y] = isWhite;
                }
            }
            return bwMap;
        }
    }

}
