﻿using System;
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
using System.Drawing;

namespace LaunchPad
{
    public sealed partial class MainPage : Page
    {
        // Constants
        const string PEN_UP_MESSAGE = "PenUp();";
        const string PEN_DOWN_MESSAGE = "PenDown();";
        const string NEXT_INSTRUCTION_MESSAGE = "Next Please :)";
        const string FINISH_DRAWING_MESSAGE = "That will do, cheers bud :);";
        const string DRAWING_FINISHED_MESSAGE = "All done! :)";

        public MainPage()
        {
            this.InitializeComponent();
            filePicker = new FileOpenPicker();
        }

        StorageFile imageFile;
        FileOpenPicker filePicker;
        
        PointF currentStartPoint;

        PointF ColinearAtTime(PointF A, PointF B, float t)
        {
            float x = ((1 - t) * A.X) + (t * B.X);
            float y = ((1 - t) * A.Y) + (t * B.Y);
            return new PointF(x, y);
        }

        List<SvgPathSegmentList> GetPathsFromSvg(XmlDocument xDoc)
        {

        }

        List<string> GetArduinoInstructions(List<SvgPathSegmentList> paths, double smoothness)
        {
            
        }

        List<PointCollect> GetPreviewPoints(List<SvgPathSegmentList> paths, double smoothness)
        {

        }

        PointCollection GetMoveToPointCollection(SvgMoveToSegment seg)
        {
            return new PointCollection() { new Windows.Foundation.Point(seg.End.X, seg.End.Y) };
        }

        Windows.Foundation.Point GetLineToPoint(SvgLineSegment seg)
        {
            return new Windows.Foundation.Point(seg.End.X, seg.End.Y);
        }

        Windows.Foundation.Point GetQuadraticPoint(SvgQuadraticCurveSegment seg, float t)
        {
            PointF SC = ColinearAtTime(seg.Start, seg.ControlPoint, t);
            PointF CE = ColinearAtTime(seg.ControlPoint, seg.End, t);
            PointF point = ColinearAtTime(SC, CE, t);

            return new Windows.Foundation.Point(point.X, point.Y);
        }

        Windows.Foundation.Point GetCubicPoint(SvgCubicCurveSegment seg, float t)
        {
            // Intermediate Points
            PointF SC1 = ColinearAtTime(seg.Start, seg.FirstControlPoint, t);
            PointF C1C2 = ColinearAtTime(seg.FirstControlPoint, seg.SecondControlPoint, t);
            PointF C2E = ColinearAtTime(seg.SecondControlPoint, seg.End, t);

            // Resolve Intermediate Points (Like Quadratic)
            PointF A = ColinearAtTime(SC1, C1C2, t);
            PointF B = ColinearAtTime(C1C2, C2E, t);
            PointF point = ColinearAtTime(A, B, t);

            return new Windows.Foundation.Point(point.X, point.Y);
        }

        Windows.Foundation.Point GetArcPoint(CenterParameterizedArcData arcData, float t)
        {

        }

        Windows.Foundation.Point GetClosePathPoint(SvgClosePathSegment seg)
        {
            return new Windows.Foundation.Point(currentStartPoint.X, currentStartPoint.Y);
        }



        List<PointCollection> GetPointsFromSvg(, double smoothness) {
            List<PointCollection> PreviewPoints = new List<PointCollection>();

            XmlNodeList Paths = xDoc.GetElementsByTagName("path");

            foreach (XmlElement path in Paths)
            {
                string pathData = path.GetAttribute("d");
                SvgPathSegmentList segments = SvgPathBuilder.Parse(pathData);
                foreach (SvgPathSegment segment in segments)
                {
                    if (segment.GetType() == typeof(SvgMoveToSegment))
                    {
                        var seg = (SvgMoveToSegment)segment;
                        // PenUp
                        // Coord
                        // PenDown

                        currentStartPoint = seg.End;

                        // Preview
                        PreviewPoints.Add();

                    }
                    else if (segment.GetType() == typeof(SvgLineSegment))
                    {
                        var seg = (SvgLineSegment)segment;
                        // Coord

                        // Preview                            
                        PreviewPoints.Last().Add();
                    }
                    else if (segment.GetType() == typeof(SvgQuadraticCurveSegment))
                    {
                        var seg = (SvgQuadraticCurveSegment)segment;
                        for (float t = 0; t <= 1; t += 0.05f)
                        {

                            // Preview
                            PreviewPoints.Last().Add();
                        }

                    }
                    else if (segment.GetType() == typeof(SvgCubicCurveSegment))
                    {
                        var seg = (SvgCubicCurveSegment)segment;
                        for (float t = 0; t <= 1; t += 0.05f)
                        {
                            // Preview
                            PreviewPoints.Last().Add(new Windows.Foundation.Point(point.X, point.Y));
                        }

                    }
                    else if (segment.GetType() == typeof(SvgArcSegment))
                    {
                        // guidance: www.w3.org/TR/SVG/implnote.html#ArcSyntax
                        // there is no easy way to do this except convert to centre parameterization lol 
                        var seg = (SvgArcSegment)segment;
                        
                        //System.Diagnostics.Debug.WriteLine($"cx: {cx} cy: {cy}");

                        // Get Points for Polyline Approximation
                        for (float t = theta0; t <= theta0 + dTheta; t += dTheta / 50)
                        {
                            float sinT = MathF.Sin(t);
                            float cosT = MathF.Cos(t);
                            float x = (cosA * rx * cosT) - (sinA * ry * sinT) + cx;
                            float y = (sinA * rx * cosT) + (cosA * ry * sinT) + cy;

                            // Preview
                            PreviewPoints.Last().Add(new Windows.Foundation.Point(x, y));

                        }

                    }
                    else if (segment.GetType() == typeof(SvgClosePathSegment))
                    {
                        var seg = (SvgClosePathSegment)segment;

                        if (currentStartPoint != seg.Start) // if length back to start is not zero
                        {
                            // Coord 
                            // PenUp

                            // Preview
                            PreviewPoints.Last().Add();
                        }

                    }
                }
            }
            return PreviewPoints;
        }

        private async void LoadSVGButton_Click(object sender, RoutedEventArgs e)
        {
            PreviewCanvas.Children.Clear();

            filePicker.FileTypeFilter.Add(".svg");
            filePicker.SuggestedStartLocation = PickerLocationId.Desktop;
            imageFile = await filePicker.PickSingleFileAsync();
            if(imageFile != null) {
                string imageFileString = await FileIO.ReadTextAsync(imageFile);
                FileTextBlock.Text = imageFileString;
                FileTextScrollView.Visibility = Visibility.Visible;
                FileTextScrollTitleBlock.Visibility = Visibility.Visible;

                XmlDocument xDoc = new XmlDocument();
                xDoc.LoadXml(imageFileString);

                var previewPoints = GetPointsFromSvg(xDoc, SmoothSlider.Value);
                foreach(PointCollection points in previewPoints)
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
        }

        private void SmoothSlider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {

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
            cy = seg.RadiusY;

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
            theta0 = MathF.Acos(ux / n0);

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

    }

}
