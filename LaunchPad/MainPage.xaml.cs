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
            cutterSerial = new SerialPortInput();
            cutterSerial.MessageReceived += CutterSerial_MessageReceived;
            LaserCircle = new Windows.UI.Xaml.Shapes.Ellipse() { Fill = null, Stroke = LaserCircleBrush, Width = 8, Height = 8, StrokeThickness = 2 };
            LaserCircle.Transitions.Add(new Windows.UI.Xaml.Media.Animation.RepositionThemeTransition());
            LaserLocationTimer = new Timer() { AutoReset = true, Interval = 10 };
            LaserLocationTimer.Elapsed += LaserLocationTimer_Elapsed;
            PreviewCanvas.Children.Add(LaserCircle);
            Canvas.SetLeft(LaserCircle, -4);
            Canvas.SetTop(LaserCircle, -4);
        }

        private async void LaserLocationTimer_Elapsed(object sender, ElapsedEventArgs e)
        {
            LaserX = 10 * VX;
            LaserY = 10 * VY;
            await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Low, () => {
                LaserCircle.Fill = LaserOn ? LaserCircleBrush : null;
                Canvas.SetLeft(LaserCircle, LaserX - 4);
                Canvas.SetTop(LaserCircle, LaserY - 4);
            });
        }

        StorageFile imageFile;
        FileOpenPicker filePicker;
        XmlDocument xDoc = new XmlDocument();
        PointF currentStartPoint;
        float Step { get { return 1 / (float)SmoothSlider.Value; } }
        List<CutterInstruction> instructions;
        BitmapEditor bitmapEditor;
        double currentScale = 1;
        static SerialPortInput cutterSerial;
        Windows.UI.Xaml.Shapes.Ellipse LaserCircle;
        SolidColorBrush LaserCircleBrush = new SolidColorBrush(Windows.UI.Colors.BlueViolet);
        Timer LaserLocationTimer;
        double LaserX = 0;
        double LaserY = 0;
        bool LaserOn = false;
        double VX = 0;
        double VY = 0;

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

        List<CutterInstruction> GetArduinoInstructions(List<SvgPathSegmentList> paths)
        {
            List<CutterInstruction> instructions = new List<CutterInstruction>();
            foreach (SvgPathSegmentList path in paths)
            {
                foreach (SvgPathSegment segment in path)
                {
                    if (segment.GetType() == typeof(SvgMoveToSegment))
                    {
                        var seg = (SvgMoveToSegment)segment;
                        instructions.Add(CutterInstruction.PenUpInstruction);
                        instructions.Add(new CutterInstruction(seg.End.ToFoundationPoint(currentScale)));
                        instructions.Add(CutterInstruction.PenDownInstruction);
                        currentStartPoint = seg.End;

                    }
                    else if (segment.GetType() == typeof(SvgLineSegment))
                    {
                        var seg = (SvgLineSegment)segment;
                        instructions.Add(new CutterInstruction(GetLineToPoint(seg).ToFoundationPoint(currentScale)));
                    }
                    else if (segment.GetType() == typeof(SvgQuadraticCurveSegment))
                    {
                        var seg = (SvgQuadraticCurveSegment)segment;
                        for (float t = 0; t <= 1; t += Step)
                        {
                            instructions.Add(new CutterInstruction(GetQuadraticPoint(seg, t).ToFoundationPoint(currentScale)));
                        }
                        instructions.Add(new CutterInstruction(seg.End.ToFoundationPoint(currentScale)));
                    }
                    else if (segment.GetType() == typeof(SvgCubicCurveSegment))
                    {
                        var seg = (SvgCubicCurveSegment)segment;
                        for (float t = 0; t <= 1; t += Step)
                        {
                            instructions.Add(new CutterInstruction(GetCubicPoint(seg, t).ToFoundationPoint(currentScale)));
                        }
                        instructions.Add(new CutterInstruction(seg.End.ToFoundationPoint(currentScale)));
                    }
                    else if (segment.GetType() == typeof(SvgArcSegment))
                    {
                        var seg = (SvgArcSegment)segment;
                        var arcData = new CenterParameterizedArcData(seg);
                        for (float t = 0; t <= 1; t += Step)
                        {
                            instructions.Add(new CutterInstruction(GetArcPoint(arcData, t).ToFoundationPoint(currentScale)));
                        }
                        instructions.Add(new CutterInstruction(seg.End.ToFoundationPoint(currentScale)));
                    }
                    else if (segment.GetType() == typeof(SvgClosePathSegment))
                    {
                        var seg = (SvgClosePathSegment)segment;

                        if (currentStartPoint != seg.Start) // if length back to start is not zero
                        {
                            instructions.Add(new CutterInstruction(GetClosePathPoint().ToFoundationPoint(currentScale)));
                        }
                        instructions.Add(CutterInstruction.PenUpInstruction);
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
            PreviewCanvas.Children.Add(LaserCircle);
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
                        if (lineStart == width) // Not currently drawing
                        {
                            lineStart = x;
                        }
                        else if (x == width - 1)
                        {
                            var rect = new Windows.UI.Xaml.Shapes.Rectangle() { Width = (x - 1 - lineStart) * currentScale, Height = lamination * currentScale };
                            Canvas.SetTop(rect, y * currentScale);
                            Canvas.SetLeft(rect, lineStart * currentScale);
                            rects.Add(rect);
                        }
                    }
                    else
                    {
                        if (lineStart != width) // Currently drawing
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

        List<CutterInstruction> GetArduinoInstructions(bool[,] bwMap, int lamination)
        {
            List<CutterInstruction> instructions = new List<CutterInstruction>();
            int width = bwMap.GetLength(0);
            int height = bwMap.GetLength(1);
            bool drawing;                    // Equals width when a line hasn't been started
            int startingX;
            int stepX;
            instructions.Add(CutterInstruction.PenUpInstruction);
            instructions.Add(new CutterInstruction(new Windows.Foundation.Point(0, 0)));
            for (int y = 0; y < height; y += lamination)
            {
                drawing = false;
                startingX = (y % 2 == 0) ? 0 : width - 1;
                stepX = (y % 2 == 0) ? 1 : -1;
                for (int x = startingX; (y % 2 == 0) ? x < width : x > 0; x += stepX)
                {
                    if (bwMap[x, y])
                    {
                        if (!drawing) // Not currenntly drawing
                        {
                            instructions.Add(new CutterInstruction(new Windows.Foundation.Point(x * currentScale, y * currentScale)));
                            instructions.Add(CutterInstruction.PenDownInstruction);
                            drawing = true;
                        }
                        else if (x == width - (startingX + 1))
                        {
                            instructions.Add(new CutterInstruction(new Windows.Foundation.Point(x * currentScale, y * currentScale)));
                            instructions.Add(CutterInstruction.PenUpInstruction);
                        }
                    }
                    else
                    {
                        if (drawing) // Currently drawing
                        {
                            instructions.Add(new CutterInstruction(new Windows.Foundation.Point(x * currentScale, y * currentScale)));
                            instructions.Add(CutterInstruction.PenUpInstruction);
                            drawing = false;
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
            if (imageFile != null)
            {
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

        List<CutterInstruction> fullInstructions = new List<CutterInstruction>();
        private void PrintButton_Click(object sender, RoutedEventArgs e)
        {
            if (instructions != null && cutterSerial != null)
            {
                fullInstructions.Clear();
                if (!cutterSerial.IsConnected)
                {
                    for (int pass = 0; pass < PassCountSlider.Value; pass++)
                    {
                        foreach (CutterInstruction instruction in instructions)
                        {
                            fullInstructions.Add(instruction);
                        }
                    }
                    fullInstructions.Add(CutterInstruction.PenUpInstruction);
                    fullInstructions.Add(CutterInstruction.ReturnToOriginInstruction);
                    cutterSerial.Connect();
                    LaserLocationTimer.Start();
                    //instructionTimer.Interval = 1;
                    //instructionTimer.Start();
                }
                else
                {
                    LaserLocationTimer.Stop();
                    //instructionTimer.Stop();
                    cutterSerial.Disconnect();
                }
            }
        }

        double destX, destY = 0;

        /*private void InstructionTimer_Elapsed(object sender, ElapsedEventArgs e)
        {
            if (fullInstructions.Any())
            {
                string msg = string.Empty;
                double time;
                CutterInstruction currentInstruction;
                do
                {
                    currentInstruction = fullInstructions.First();
                    if (currentInstruction.IsCoord)
                    {
                        msg = currentInstruction.GetArduinoMessage(destX, destY);
                        time = currentInstruction.GetTime(destX, destY);
                        (destX, destY) = (currentInstruction.Point.X, currentInstruction.Point.Y);
                    }
                    else
                    {
                        msg = currentInstruction.GetArduinoMessage();
                        time = 1;
                    }
                    fullInstructions.Remove(currentInstruction);
                } while (string.IsNullOrWhiteSpace(msg) && fullInstructions.Any());

                System.Diagnostics.Debug.WriteLine(msg);
                cutterSerial.SendMessage(Encoding.ASCII.GetBytes(msg));
                instructionTimer.Interval = time;
                instructionTimer.Start();
            }
            else
            {
                cutterSerial.SendMessage(Encoding.ASCII.GetBytes(CutterInstruction.ZeroString)); // Send Speed of Zero
                System.Diagnostics.Debug.WriteLine("Print Complete");
                //PreviewCanvas.Children.Remove(LaserCircle);
                try
                {
                    cutterSerial.Disconnect();
                }
                catch { }
            }
        }*/

        private void CutterSerial_MessageReceived(object sender, MessageReceivedEventArgs e)
        {
            string resp = Encoding.ASCII.GetString(e.Data);
            if (resp == ":)")
            {
                if (fullInstructions.Any())
                {
                    string msg;
                    CutterInstruction currentInstruction;
                    do
                    {
                        currentInstruction = fullInstructions.First();
                        if (currentInstruction.IsCoord)
                        {
                            msg = currentInstruction.GetArduinoMessage(destX, destY);
                            (VX, VY) = currentInstruction.GetComponentVelocities(destX, destY);
                            (LaserX, LaserY) = (destX, destY);
                            (destX, destY) = (currentInstruction.Point.X, currentInstruction.Point.Y);
                        }
                        else
                        {
                            msg = currentInstruction.GetArduinoMessage();
                            LaserOn = currentInstruction.PenDown;
                            (VX, VY) = (0, 0);
                        }
                        fullInstructions.Remove(currentInstruction);
                    } while (string.IsNullOrWhiteSpace(msg) && fullInstructions.Any());

                    System.Diagnostics.Debug.WriteLine(msg);
                    cutterSerial.SendMessage(Encoding.ASCII.GetBytes(msg));
                }
                else
                {
                    //cutterSerial.SendMessage(Encoding.ASCII.GetBytes(CutterInstruction.ZeroString)); // Send Speed of Zero
                    System.Diagnostics.Debug.WriteLine("Print Complete");
                    (VX, VY) = (0, 0);
                    LaserLocationTimer.Stop();
                    //PreviewCanvas.Children.Remove(LaserCircle);
                    try
                    {
                        cutterSerial.Disconnect();
                    }
                    catch { }
                }
            }
            else
            {
                System.Diagnostics.Debug.WriteLine(resp);
            }
        }

        /*private async void CutterSerial_MessageReceived(object sender, MessageReceivedEventArgs e)
        {
            try
            {
                string resp = Encoding.ASCII.GetString(e.Data);
                System.Diagnostics.Debug.WriteLine(resp);

                var coord = resp.Substring(1, resp.Length - 2).Split(',');
                double posX = double.Parse(coord[0]);
                double posY = double.Parse(coord[1]);
                bool penDown = int.Parse(coord[2]) == 1;

                if (fullInstructions.Any())
                {
                    // Update Laser Circle's Position
                    await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
                    {
                        LaserCircle.StrokeThickness = penDown ? 0 : 1;
                        Canvas.SetLeft(LaserCircle, posX - 5 - LaserCircle.StrokeThickness);
                        Canvas.SetTop(LaserCircle, posY - 5 - LaserCircle.StrokeThickness);
                        LaserCircle.Fill = penDown ? LaserCircleFillBrush : null;
                        if (!PreviewCanvas.Children.Contains(LaserCircle))
                            PreviewCanvas.Children.Add(LaserCircle);
                    });

                    // Check if cutter has reached destination
                    bool xArrived = (posX >= destX && !double.IsNegative(vX)) || (posX <= destX && double.IsNegative(vX));
                    bool yArrived = (posY >= destY && !double.IsNegative(vY)) || (posY <= destY && double.IsNegative(vY));

                    string msg = string.Empty;
                    CutterInstruction currentInstruction = fullInstructions.First();
                    if (xArrived && yArrived) // We must send new instruction
                    {
                        do
                        {
                            if (currentInstruction.IsCoord)
                            {
                                msg = currentInstruction.GetArduinoMessage(destX, destY, !penDown);
                                (vX, vY) = currentInstruction.GetComponentVelocities(destX, destY, !penDown);
                                System.Diagnostics.Debug.WriteLine($"Velocities: ({vX},{vY})");
                                (destX, destY) = (currentInstruction.Point.X, currentInstruction.Point.Y);
                            }
                            else
                            {
                                msg = currentInstruction.GetArduinoMessage();
                            }
                            fullInstructions.Remove(currentInstruction);
                            currentInstruction = fullInstructions.First();
                        } while (string.IsNullOrWhiteSpace(msg) && fullInstructions.Any());
                    }
                else if (xArrived && currentInstruction.IsCoord)
                {
                    msg = currentInstruction.GetArduinoMessage(destX, posY, !penDown);
                }
                else if(yArrived && currentInstruction.IsCoord)
                {
                    msg = currentInstruction.GetArduinoMessage(posX, destY, !penDown);
                }
                    cutterSerial.SendMessage(Encoding.ASCII.GetBytes(msg));
                }
                else
                {
                    byte[] msg = Encoding.ASCII.GetBytes(CutterInstruction.PenUpInstruction.GetArduinoMessage());
                    cutterSerial.SendMessage(msg); // PenUp
                    msg = Encoding.ASCII.GetBytes(new CutterInstruction(new Windows.Foundation.Point(0, 0)).GetArduinoMessage(posX, posY, true));
                    cutterSerial.SendMessage(msg); // Return to origin
                    System.Diagnostics.Debug.WriteLine("Print Complete");
                    PreviewCanvas.Children.Remove(LaserCircle);
                    try
                    {
                        cutterSerial.Disconnect();
                    }
                    catch { }
                }
            }
            catch { }
        }*/

        /*
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
        }*/

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
            if (double.TryParse(ScaleTextBox.Text.Replace("%", ""), out double newScale))
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

    public class CutterInstruction
    {
        const string PEN_UP_MESSAGE = "PenUp();";
        const string PEN_DOWN_MESSAGE = "PenDown();";
        const double velocityConstant = 1000;

        public readonly Windows.Foundation.Point Point;
        public readonly bool IsCoord;
        public readonly bool PenDown;

        public CutterInstruction(Windows.Foundation.Point point)
        {
            IsCoord = true;
            Point = point;
        }

        public CutterInstruction(bool penDown)
        {
            IsCoord = false;
            PenDown = penDown;
        }

        private double diffX, diffY, theta, vX, vY, t;
        public string GetArduinoMessage(double currentX = 0, double currentY = 0)
        {
            string msg = string.Empty;
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
                msg = PenDown ? PEN_DOWN_MESSAGE : PEN_UP_MESSAGE;
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

        public static readonly CutterInstruction PenUpInstruction = new CutterInstruction(false);
        public static readonly CutterInstruction PenDownInstruction = new CutterInstruction(true);
        public static readonly CutterInstruction ReturnToOriginInstruction = new CutterInstruction(new Windows.Foundation.Point(0, 0));
        public const string ZeroString = "(0,0,0);";
    }

}