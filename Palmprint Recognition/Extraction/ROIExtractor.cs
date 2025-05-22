using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Face;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System.Diagnostics;
using System.Globalization;
using System.Net;
using System.Runtime.InteropServices;

namespace Palmprint_Recognition.Extraction
{
    internal class ROIExtractor
    {
        /// <summary>
        /// Tries to extract a consistent palm ROI: first via valley-defect method; on failure, falls back to color+DT refine.
        /// </summary>
        /// <param name="inputBgr"></param>
        /// <param name="palmROI"></param>
        /// <param name="dtMask"></param>
        /// <returns></returns>
        public bool TryExtract(Mat inputBgr, out Mat palmROI, out Mat dtMask)
        {
            palmROI = null;
            dtMask = null;

            if (!IsValidInput(inputBgr))
                return false;

            // 1) Preprocess: normalize lighting and binarize
            LightingNormalize(inputBgr);
            Mat binaryImg = BinaryImage(inputBgr);
            ShowBinaryImage(binaryImg, new Size(600, 600)); // Optional: for debugging
            binaryImg = AddBorder(binaryImg, 5);

            Mat proc = binaryImg.Clone();

            using var ker = CvInvoke.GetStructuringElement(ElementShape.Rectangle, new Size(5, 5), new Point(-1, -1));
            CvInvoke.Erode(proc, proc, ker, new Point(-1, -1), 5, BorderType.Constant, new MCvScalar(0));

            // 2) Find hand contour
            VectorOfPoint? contour = FindLargestContour(proc);
            ShowContour(inputBgr, contour, new Size(600, 600)); // Optional: for debugging
            //ShowBoundingBox(inputBgr, contour, new Size(300, 300)); // Optional: for debugging

            if (contour == null)
                return false;

            // 3) Detect valley points
            List<Point> valleys = GetValleyPoints(contour);

            if (valleys.Count >= 3 && valleys.Count < 5)
            {
                // 4) Determine handedness and order valleys
                bool isLeftHand = IsLeftHand(contour, valleys);
                List<Point> orderedValleys = OrderValleys(valleys, isLeftHand);
                DrawValleyPoints(inputBgr, orderedValleys, new Size(600, 600)); // Optional: for debugging
                // 5) Compute source polygon points for ROI
                Point[] srcPoints = ComputeSourcePoints(orderedValleys, isLeftHand, 150);
                DrawROIRectangle(inputBgr, srcPoints, new Size(600, 600)); // Optional: for debugging
                // 6) Extract raw ROI and mask
                (Mat rawROI, Mat rawMask) = ExtractRawROI(inputBgr, srcPoints);
                // 7) Compute rotation angle and align
                double theta = ComputeRotationAngle(orderedValleys, isLeftHand);
                AlignROI(rawROI, rawMask, theta, out palmROI, out dtMask);

                // 8 Crop align roi
                (palmROI, _) = CropToMask(palmROI, dtMask);
                
                dtMask = binaryImg.Clone();
                return true;
            }
            else
            {
                return ExtractByDistanceTransform(inputBgr, out palmROI, out dtMask);
            }
        }

        private bool IsValidInput(Mat input)
        {
            return input != null && !input.IsEmpty;
        }

        private List<Point> GetValleyPoints(VectorOfPoint contour)
        {
            // Compute convex hull indices
            var hullIdx = new VectorOfInt();
            CvInvoke.ConvexHull(contour, hullIdx, false, false);

            // Calculate convexity defects
            var defectMat = new Mat();
            CvInvoke.ConvexityDefects(contour, hullIdx, defectMat);
            int[,,]? d = defectMat.GetData() as int[,,];

            var valleys = new List<Point>();
            if (d != null)
            {
                // Bounding box to filter by vertical position
                var bb = CvInvoke.BoundingRectangle(contour);
                double topThresh = bb.Y + bb.Height * 0.2; // üst %15’i atla
                double bottomThresh = bb.Y + bb.Height * 0.75; // alt %25’i atla
                //double yThresh = bb.Y + bb.Height * 0.2;
                int minDepth = Math.Max(10, (int)(bb.Height * 0.1));

                for (int i = 0; i < d.GetLength(0); i++)
                {
                    int startIdx = d[i, 0, 0];
                    int endIdx = d[i, 0, 1];
                    int farIdx = d[i, 0, 2];
                    int depth = d[i, 0, 3];

                    // Depth threshold
                    if (depth < minDepth)
                        continue;

                    Point pt = contour[farIdx];
                    // Exclude defects too high (finger tips)
                    if (pt.Y < topThresh || pt.Y > bottomThresh)
                        continue;

                    // Angle between start→far and end→far vectors
                    var v1 = new PointF(contour[startIdx].X - pt.X, contour[startIdx].Y - pt.Y);
                    var v2 = new PointF(contour[endIdx].X - pt.X, contour[endIdx].Y - pt.Y);
                    double dot = v1.X * v2.X + v1.Y * v2.Y;
                    double ang = Math.Acos(dot / (
                        Math.Sqrt(v1.X * v1.X + v1.Y * v1.Y) *
                        Math.Sqrt(v2.X * v2.X + v2.Y * v2.Y)
                    )) * 180.0 / Math.PI;

                    if (ang < 20 || ang > 120)
                        continue;

                    valleys.Add(pt);
                }
            }

            return valleys;
        }
        //private bool IsLeftHand(VectorOfPoint contour, List<Point> valleys)
        //{
        //    // 1) El konturunun centroid X’i (daha stabil bir merkez)
        //    double centerX = contour.ToArray().Average(p => p.X);

        //    // 2) Kontur bounding‐box’unun Y aralığını alıp gerçek parmak diplerine odaklan
        //    var bb = CvInvoke.BoundingRectangle(contour);
        //    double minY = bb.Y + bb.Height * 0.2;  // en üstten %20 aşağı
        //    double maxY = bb.Y + bb.Height * 0.9;  // en üstten %90 aşağı (bilek altını atla)

        //    // 3) Bu Y aralığında kalan valley’leri al
        //    var filtered = valleys
        //        .Where(p => p.Y >= minY && p.Y <= maxY)
        //        .ToList();

        //    // 4) Eğer hiç kalmadıysa orijinal listeye dön
        //    var candidates = filtered.Count > 0 ? filtered : valleys;

        //    // 5) Merkezden en uzak X farkına sahip valley’ı thumb olarak seç
        //    var thumbValley = candidates
        //        .OrderByDescending(p => Math.Abs(p.X - centerX))
        //        .First();

        //    // 6) Thumb, centroid’in sağındaysa sol el (mirror!), solundaysa sağ el
        //    bool isLeft = (thumbValley.X > centerX);

        //    Debug.WriteLine($"IsLeftHand? centroidX={centerX:F1}, thumbX={thumbValley.X}, isLeft={isLeft}");
        //    return isLeft;
        //}

        private bool IsLeftHand(VectorOfPoint contour, List<Point> valleys)
        {
            var bb = CvInvoke.BoundingRectangle(contour);
            int centerX = bb.X + bb.Width / 2;
            var thumbValley = valleys.OrderByDescending(p => Math.Abs(p.X - centerX)).First();
            Debug.WriteLine(thumbValley.X > centerX);
            return thumbValley.X > centerX;
        }
        private bool IsLeftHandPca(VectorOfPoint contour, List<Point> valleys)
        {
            // 1) Kontur noktalarını double[,] dizisine taşı
            int N = contour.Size;
            var data = new Matrix<double>(N, 2);
            for (int i = 0; i < N; i++)
            {
                data[i, 0] = contour[i].X;
                data[i, 1] = contour[i].Y;
            }

            // 2) Ortalama al ve merkezi veri oluştur
            double meanX = CvInvoke.Mean(data.GetCol(0)).V0;
            double meanY = CvInvoke.Mean(data.GetCol(1)).V0;
            for (int i = 0; i < N; i++)
            {
                data[i, 0] -= meanX;
                data[i, 1] -= meanY;
            }

            // 3) Kovaryans matrisi
            var cov = new Matrix<double>(2, 2);
            CvInvoke.Gemm(data, data, 1.0 / N, null, 0, cov, GemmType.Src2Transpose);

            // 4) Eigen decomposition
            Matrix<double> evals = new Matrix<double>(2, 1);
            Matrix<double> evecs = new Matrix<double>(2, 2);
            CvInvoke.Eigen(cov, evals, evecs);

            // 5) Birinci bileşen vektörü (en büyük eigenvector)
            double vx = evecs[0, 0], vy = evecs[0, 1];

            // 6) Valley noktalarını bu vektöre projekte et, en büyük projeksiyonu bulun
            double maxProj = double.MinValue;
            Point thumbValley = valleys[0];
            foreach (var p in valleys)
            {
                double dx = p.X - meanX, dy = p.Y - meanY;
                double proj = dx * vx + dy * vy;
                if (proj > maxProj)
                {
                    maxProj = proj;
                    thumbValley = p;
                }
            }
            Debug.WriteLine(maxProj > 0);
            // 7) Pozitif projeksiyon sol elde, negatif sağ elde diyelim
            return maxProj > 0;
        }

        private List<Point> OrderValleys(List<Point> valleys, bool isLeftHand)
        {
            return isLeftHand
                ? valleys.OrderBy(p => p.X).ToList()
                : valleys.OrderByDescending(p => p.X).ToList();
        }

        private Point[] ComputeSourcePoints(List<Point> ordered, bool isLeftHand, int offset = 100)
        {
            // Calculate p1 and p2 based on count
            Point p1, p2;
            if (ordered.Count >= 4)
            {
                p1 = ordered[1];
                p2 = ordered[3];
            }
            else if (ordered.Count == 3)
            {
                p1 = ordered[1];
                p2 = ordered[2];
            }
            else
            {
                p1 = ordered[0];
                p2 = ordered[1];
            }

            // Compute vector and normals
            double dx = p2.X - p1.X;
            double dy = p2.Y - p1.Y;
            double side = Math.Sqrt(dx * dx + dy * dy);

            double nx = dy / side;
            double ny = -dx / side;

            // 4) Sol elde normali ters çevir
            if (isLeftHand)
            {
                nx = -nx;
                ny = -ny;
            }

            Point down1 = new Point(
                p1.X + (int)Math.Round(nx * side),
                p1.Y + (int)Math.Round(ny * side)
            );
            Point down2 = new Point(
                p2.X + (int)Math.Round(nx * side),
                p2.Y + (int)Math.Round(ny * side)
            );

            Point p1s = new Point(p1.X, p1.Y + offset);
            Point p2s = new Point(p2.X, p2.Y + offset);
            Point d1s = new Point(down1.X, down1.Y + offset);
            Point d2s = new Point(down2.X, down2.Y + offset);

            return new[] { p1s, p2s, d2s, d1s };
        }

        private (Mat rawROI, Mat rawMask) ExtractRawROI(Mat input, Point[] src)
        {
            using var polyMask = Mat.Zeros(input.Size.Height, input.Size.Width, DepthType.Cv8U, 1);
            CvInvoke.FillConvexPoly(polyMask, new VectorOfPoint(src), new MCvScalar(255));

            Mat maskedPalm = new Mat();
            CvInvoke.BitwiseAnd(input, input, maskedPalm, polyMask);

            var bb = CvInvoke.BoundingRectangle(new VectorOfPoint(src));
            bb.Intersect(new Rectangle(Point.Empty, input.Size));

            return (new Mat(maskedPalm, bb), new Mat(polyMask, bb));
        }

        private double ComputeRotationAngle(List<Point> ordered, bool isLeftHand)
        {
            Point p1, p2;
            if (ordered.Count >= 4)
            {
                p1 = ordered[1];
                p2 = ordered[3];
            }
            else if (ordered.Count == 3)
            {
                p1 = ordered[1];
                p2 = ordered[2];
            }
            else
            {
                p1 = ordered[0];
                p2 = ordered[1];
            }
            if (isLeftHand)
            {
                return Math.Atan2(p2.Y - p1.Y, p2.X - p1.X) * 180 / Math.PI;
            }
            else
            {
                return Math.Atan2(p1.Y - p2.Y, p1.X - p2.X) * 180 / Math.PI;
            }

        }

        private bool AlignROI(Mat rawROI, Mat rawMask, double theta, out Mat finalROI, out Mat finalMask)
        {
            PointF center = new PointF(rawROI.Width / 2f, rawROI.Height / 2f);
            Mat rotMat = new Mat();
            CvInvoke.GetRotationMatrix2D(center, theta, 1.0, rotMat);

            finalROI = new Mat();
            finalMask = new Mat();
            CvInvoke.WarpAffine(rawROI, finalROI, rotMat, rawROI.Size, Inter.Linear, Warp.Default, BorderType.Constant, new MCvScalar(0));
            CvInvoke.WarpAffine(rawMask, finalMask, rotMat, rawMask.Size, Inter.Nearest, Warp.Default, BorderType.Constant, new MCvScalar(0));

            return true;
        }
        private (Mat croppedROI, Mat croppedMask) CropToMask(Mat roi, Mat mask)
        {
            // 1) Maskede sıfırdan farklı (non-zero) noktaları bul
            using var nonZero = new VectorOfPoint();
            CvInvoke.FindNonZero(mask, nonZero);

            // 2) Bu noktaların bounding rectangle’ini al
            var rect = CvInvoke.BoundingRectangle(nonZero);

            // 3) ROI ve maskeyi o rectangle ile kırp
            var croppedROI = new Mat(roi, rect);
            var croppedMask = new Mat(mask, rect);

            return (croppedROI, croppedMask);
        }

        private void LightingNormalize(Mat image)
        {
            using var lab = new Mat();
            CvInvoke.CvtColor(image, lab, ColorConversion.Bgr2Lab);
            Mat[] channels = lab.Split();
            CvInvoke.CLAHE(channels[0], 2.0, new Size(8, 8), channels[0]);
            using var merged = new Mat();
            CvInvoke.Merge(new VectorOfMat(channels), merged);
            CvInvoke.CvtColor(merged, image, ColorConversion.Lab2Bgr);
            CvInvoke.GaussianBlur(image, image, new Size(3, 3), 0);
        }

        private Mat BinaryImage(Mat image)
        {
            Mat repaired = RemoveSpecularHighlights(image);

            using var ycc = new Mat();
            CvInvoke.CvtColor(repaired, ycc, ColorConversion.Bgr2YCrCb);
            var mask = new Mat();
            CvInvoke.InRange(ycc,
                new ScalarArray(new MCvScalar(0, 140, 80)),
                new ScalarArray(new MCvScalar(255, 180, 130)),
                mask);
            using var kernel = CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(7, 7), new Point(-1, -1));
            CvInvoke.MorphologyEx(mask, mask, MorphOp.Open, kernel, Point.Empty, 3, BorderType.Default, new MCvScalar());
            CvInvoke.MorphologyEx(mask, mask, MorphOp.Close, kernel, Point.Empty, 1, BorderType.Default, new MCvScalar());

            // Ufak delikleri doldur
            FillSmallHoles(mask, maxHoleArea: 1500);

            return mask;
        }
        private Mat RemoveSpecularHighlights(Mat color)
        {
            // a) HSV'ye çevir, V kanalını çıkar
            Mat hsv = new Mat();
            CvInvoke.CvtColor(color, hsv, ColorConversion.Bgr2Hsv);
            Mat vch = new Mat();
            CvInvoke.ExtractChannel(hsv, vch, 2);

            // b) Çok parlakları tespit et (>230)
            Mat brightMask = new Mat();
            CvInvoke.Threshold(vch, brightMask, 230, 255, ThresholdType.Binary);

            // c) Ufak gürültüyü açma ile temizle
            Mat k3 = CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(3, 3), Point.Empty);
            CvInvoke.MorphologyEx(brightMask, brightMask, MorphOp.Open, k3, Point.Empty, 1, BorderType.Default, new MCvScalar());

            // d) Inpaint: Telea yöntemi ile çok parlak pikselleri onar
            Mat repaired = new Mat();
            CvInvoke.Inpaint(color, brightMask, repaired, 3, InpaintType.Telea);

            return repaired;
        }
        private void FillSmallHoles(Mat mask, double maxHoleArea = 1000)
        {
            // 1) Ters maskede konturları bul (delikler)
            Mat inv = new Mat();
            CvInvoke.BitwiseNot(mask, inv);
            var holes = new VectorOfVectorOfPoint();
            CvInvoke.FindContours(inv, holes, null, RetrType.External, ChainApproxMethod.ChainApproxSimple);

            // 2) Her bir iç kontur için, alan küçükse doldur
            Mat filled = mask.Clone();
            for (int i = 0; i < holes.Size; i++)
            {
                double area = CvInvoke.ContourArea(holes[i]);
                // Sadece maxHoleArea’den küçük delikleri kapat
                if (area > 0 && area < maxHoleArea)
                {
                    CvInvoke.DrawContours(filled, holes, i, new MCvScalar(255), -1);
                }
            }

            // 3) Güncel maskeyi replace et
            filled.CopyTo(mask);
        }
        private VectorOfPoint? FindLargestContour(Mat mask)
        {
            var contours = new VectorOfVectorOfPoint();
            CvInvoke.FindContours(mask, contours, null, RetrType.External, ChainApproxMethod.ChainApproxSimple);
            if (contours.Size == 0) return null;
            int idx = Enumerable.Range(0, contours.Size)
                         .OrderByDescending(i => CvInvoke.ContourArea(contours[i]))
                         .First();
            return contours[idx];
        }
        private Mat AddBorder(Mat src, int pad)
        {
            Mat dst = new Mat();
            CvInvoke.CopyMakeBorder(src, dst, pad, pad, pad, pad, BorderType.Constant, new MCvScalar(0));
            return dst;
        }

        /// <summary>
        /// Distance‐Transform tabanlı fallback ROI çıkarma.
        /// </summary>
        /// <param name="inputBgr">Orijinal renkli BGR görüntü</param>
        /// <param name="palmROI">Çıkarılan palm ROI (renkli)</param>
        /// <param name="dtMask">Çıkarılan palm ROI maskesi (binary)</param>
        /// <returns>Başarılıysa true; el bulunamazsa false</returns>
        private bool ExtractByDistanceTransform(Mat inputBgr, out Mat palmROI, out Mat dtMask)
        {
            palmROI = null;
            dtMask = null;
            if (inputBgr == null || inputBgr.IsEmpty)
                return false;

            // 1) Normalize & binarize (AdaptiveThreshold veya sizin NormalizeAndBinarize)
            LightingNormalize(inputBgr);
            Mat binary = BinaryImage(inputBgr);

            // 2) En büyük el konturunu bulup doldur
            VectorOfPoint? contour = FindLargestContour(binary);
            if (contour == null)
                return false;
            Mat handMask = new Mat(binary.Size, DepthType.Cv8U, 1);
            CvInvoke.DrawContours(handMask, new VectorOfVectorOfPoint(contour), 0, new MCvScalar(255), -1);

            // 3) Morfolojik close ile ufak boşlukları kapatın (opsiyonel ama tavsiye ederim)
            var kernel = CvInvoke.GetStructuringElement(
                ElementShape.Ellipse, new Size(15, 15), new Point(-1, -1)
            );
            CvInvoke.MorphologyEx(
                handMask, handMask,
                MorphOp.Close, kernel,
                new Point(-1, -1), 1,
                BorderType.Constant, new MCvScalar(0)
            );

            // 4) Distance Transform
            Mat dist = new Mat();
            CvInvoke.DistanceTransform(handMask, dist, null, DistType.L2, 5);

            double minVal = 0, maxVal = 0;
            Point minLoc = new Point();
            Point maxLoc = new Point();

            // 5) En derin noktanın koordinatını al
            CvInvoke.MinMaxLoc(
                dist,       // input distance-transform matrisi
                ref minVal, // en küçük değer (kullanmasak da tanımlı olmak zorunda)
                ref maxVal, // en büyük değer
                ref minLoc, // minLoc
                ref maxLoc  // maxLoc: avuç içinin en iç noktası
            );
            Point center = maxLoc;

            // 6) El bounding‐box’ından sabit boyut belirleyin (oransal)
            var bb = CvInvoke.BoundingRectangle(contour);
            int w = (int)(bb.Width * 0.35);
            int h = (int)(bb.Height * 0.35);

            // 7) ROI dikdörtgenini center etrafında oluşturun, sınırlar içinde kalacak şekilde clamp edin
            int x = Math.Max(0, Math.Min(handMask.Cols - w, center.X - w / 2));
            int y = Math.Max(0, Math.Min(handMask.Rows - h, center.Y - h / 2));
            var roiRect = new Rectangle(x, y, w, h);

            // 8) Renkli görüntüden ve maskeden kırp
            palmROI = new Mat(inputBgr, roiRect);
            dtMask = new Mat(handMask, roiRect);
            return true;
        }

        #region Debug Methods
        private void DrawValleyPoints(Mat input, List<Point> valleys, Size windowSize)
        {
            Mat debug = input.Clone();

            for (int i = 0; i < valleys.Count; i++)
            {
                var p = valleys[i];
                string label1 = $"P{i + 1}";
                CvInvoke.Circle(debug, p, 8, new MCvScalar(0, 0, 255), 2);
                CvInvoke.PutText(debug, label1, new Point(p.X + 10, p.Y + 5),
                                 FontFace.HersheySimplex, 0.7, new MCvScalar(0, 0, 255), 2);
            }
            CvInvoke.ResizeForFrame(debug, debug, windowSize);
            CvInvoke.Imshow("Valley points", debug);
        }
        private void DrawROIRectangle(Mat input, Point[] corners, Size windowSize, int thickness = 2)
        {
            if (corners == null || corners.Length != 4)
                throw new ArgumentException("4 corner point bekleniyor", nameof(corners));

            Mat debug = input.Clone();
            // 5) Bu dik kenar çizgilerini maviyle çiz
            // 4 kenarı sırayla çiz
            for (int i = 0; i < 4; i++)
            {
                Point pA = corners[i];
                Point pB = corners[(i + 1) % 4];
                CvInvoke.Line(debug, pA, pB, new MCvScalar(255, 0, 0), thickness);
            }
            float cx = (corners[0].X + corners[2].X) / 2f;
            float cy = (corners[0].Y + corners[2].Y) / 2f;
            CvInvoke.Circle(debug, new Point((int)cx, (int)cy), 5, new MCvScalar(0, 0, 255), thickness);

            CvInvoke.ResizeForFrame(debug, debug, windowSize);
            CvInvoke.Imshow("ROI Rectangle", debug);
        }

        /// <summary>
        /// Contour’a ait bounding‐box’u orijinal görüntüye çizip PictureBox’a bastırır.
        /// </summary>
        private void ShowBoundingBox(Mat originalBgr, VectorOfPoint contour, Size windowSize)
        {
            // Orijinalin bir kopyasını alın, üzerine çizim yapacağız
            using var vis = originalBgr.Clone();

            // 1) Konturun bounding rect’ini bulun
            var bb = CvInvoke.BoundingRectangle(contour);

            // 2) Rect’i yeşil olarak çiz (BGR: 0,255,0) kalınlık 2
            CvInvoke.Rectangle(vis, bb, new MCvScalar(0, 255, 0), 2);

            CvInvoke.ResizeForFrame(vis, vis, windowSize);
            CvInvoke.Imshow("Bounding Box", vis);
        }
        /// <summary>
        /// Show countour in a separate window.
        /// </summary>
        /// <param name="originalBgr"></param>
        /// <param name="contour"></param>
        /// <param name="windowSize"></param>
        private void ShowContour(Mat originalBgr, VectorOfPoint contour, Size windowSize)
        {
            // Orijinalin bir kopyasını alın, üzerine çizim yapacağız
            using var vis = originalBgr.Clone();
            // 1) Konturu mavi olarak çiz (BGR: 255,0,0) kalınlık 2
            CvInvoke.DrawContours(vis, new VectorOfVectorOfPoint(contour), -1, new MCvScalar(255, 0, 0), 2);
            CvInvoke.ResizeForFrame(vis, vis, windowSize);
            CvInvoke.Imshow("Contour", vis);
        }
        /// <summary>
        /// Show binary image in a separate window.
        /// </summary>
        /// <param name="binary"></param>
        /// <param name="windowSize"></param>
        private void ShowBinaryImage(Mat binary, Size windowSize)
        {
            Mat mat = binary.Clone();
            CvInvoke.ResizeForFrame(mat, mat, windowSize);
            CvInvoke.Imshow("Binary Image", mat);
        }
        #endregion
    }
}
///// <summary>
///// Tries to extract a consistent palm ROI: first via valley-defect method; on failure, falls back to color+DT refine.
///// </summary>
//public bool TryExtract(Mat inputBgr, out Mat palmROI, out Mat dtMask)
//{
//    palmROI = null;
//    dtMask = null;
//    if (inputBgr == null || inputBgr.IsEmpty)
//        return false;

//    //1) Normalize lighting + binary image
//    LightingNormalize(inputBgr);
//    Mat binaryImg = BinaryImage(inputBgr);

//    // 2) En büyük konturu bulup doldur
//    var contour = FindLargestContour(binaryImg);
//    if (contour == null) return false;
//    Mat handMask = new Mat(binaryImg.Size, DepthType.Cv8U, 1);
//    CvInvoke.DrawContours(handMask, new VectorOfVectorOfPoint(contour), 0, new MCvScalar(255), -1);

//    // 3) ConvexityDefects ile valley noktalarını al
//    var hullIdx = new VectorOfInt();
//    CvInvoke.ConvexHull(contour, hullIdx, false, false);

//    Mat defectMat = new Mat();
//    CvInvoke.ConvexityDefects(contour, hullIdx, defectMat);
//    int[,,]? d = defectMat.GetData() as int[,,];

//    // Konturun bounding‐box'unu al
//    var bb = CvInvoke.BoundingRectangle(contour);
//    double yThresh = bb.Y + bb.Height * 0.2;    // en üstten %20 kadar aşağıdaki noktalar
//    double minDepth = 20;                       // derinlik eşiği (örneğin 20 piksel)

//    var valleys = new List<Point>();
//    if (d != null)
//    {
//        for (int i = 0; i < d.GetLength(0); i++)
//        {
//            int startIdx = d[i, 0, 0], endIdx = d[i, 0, 1], farIdx = d[i, 0, 2];
//            int depth = d[i, 0, 3];

//            // 1) Derinlik eşiğini geçsin
//            if (depth < minDepth)
//                continue;

//            // 2) Konum: çok yukarıda (finger tip civarı) olanları ele
//            var pt = contour[farIdx];
//            if (pt.Y < yThresh)
//                continue;

//            // 3) Dar açılı defect’leri el opsu değil parmak dipine uygun açılarla süz
//            //    Vektörler: v1 = start→far, v2 = end→far
//            var v1 = new PointF(contour[startIdx].X - pt.X, contour[startIdx].Y - pt.Y);
//            var v2 = new PointF(contour[endIdx].X - pt.X, contour[endIdx].Y - pt.Y);
//            double dot = v1.X * v2.X + v1.Y * v2.Y;
//            double ang = Math.Acos(dot / (
//                            Math.Sqrt(v1.X * v1.X + v1.Y * v1.Y) *
//                            Math.Sqrt(v2.X * v2.X + v2.Y * v2.Y)
//                          )) * 180.0 / Math.PI;
//            if (ang < 20 || ang > 120)
//                continue;

//            valleys.Add(pt);
//        }
//    }
//    if (valleys.Count < 2)
//        return false;
//    // 1) Elin orta dikey eksenini al
//    bb = CvInvoke.BoundingRectangle(contour);
//    int centerX = bb.X + bb.Width / 2;

//    // 2) Thumb–Index vadisini bul (merkezden en uzak X farkına sahip nokta)
//    Point thumbValley = valleys
//        .OrderByDescending(p => Math.Abs(p.X - centerX))
//        .First();

//    // 3) Handedness: thumbValley merkezX’in sağındaysa Sol el, solundaysa Sağ el
//    bool isLeftHand = thumbValley.X > centerX;

//    // 4) Vadileri soldan sağa (sağ el) veya sağdan sola (sol el) sırala
//    var ordered = isLeftHand
//        ? valleys.OrderBy(p => p.X).ToList()
//        : valleys.OrderByDescending(p => p.X).ToList();

//    Mat debug = inputBgr.Clone();  // veya alignedColor.Clone()
//    for (int i = 0; i < ordered.Count; i++)
//    {
//        var p = ordered[i];
//        string label1 = $"P{i + 1}";
//        CvInvoke.Circle(debug, p, 8, new MCvScalar(0, 0, 255), 2);
//        CvInvoke.PutText(debug, label1, new Point(p.X + 10, p.Y + 5),
//                         FontFace.HersheySimplex, 0.7, new MCvScalar(0, 0, 255), 2);
//    }

//    //CvInvoke.ResizeForFrame(debug, debug, new Size(600, 600));
//    //CvInvoke.Imshow("Valley Points", debug);


//    Point p1, p2;
//    if (ordered.Count >= 4)
//    {
//        p1 = ordered[1];  // index–middle arası
//        p2 = ordered[3];  // ring–little arası
//    }
//    else if (ordered.Count == 3)
//    {
//        p1 = ordered[1];  // index–middle arası
//        p2 = ordered[2];  // middle–ring  arası
//    }
//    else // sorted.Count == 2
//    {
//        p1 = ordered[0];
//        p2 = ordered[1];
//    }

//    // 2) Valley vektörü ve uzunluğu
//    double dx = p2.X - p1.X;
//    double dy = p2.Y - p1.Y;
//    double side = Math.Sqrt(dx * dx + dy * dy);

//    // 3) Valley vektörüne dik birim normal
//    //    n = (−dy, dx) / ||(dx,dy)|| 
//    double nx = dy / side;
//    double ny = -dx / side;

//    // 4) p1 ve p2’den dik çizginin uç noktaları
//    Point down1 = new Point(
//        p1.X + (int)Math.Round(nx * side),
//        p1.Y + (int)Math.Round(ny * side)
//    );
//    Point down2 = new Point(
//        p2.X + (int)Math.Round(nx * side),
//        p2.Y + (int)Math.Round(ny * side)
//    );
//    int offsetY = 100;
//    Point p1s = new Point(p1.X, p1.Y + offsetY);
//    Point p2s = new Point(p2.X, p2.Y + offsetY);
//    Point d1s = new Point(down1.X, down1.Y + offsetY);
//    Point d2s = new Point(down2.X, down2.Y + offsetY);

//    // Orta nokta ve açıyı hesapla
//    float centerXX = (p1.X + p2.X) / 2f;
//    float centerY = (p1.Y + p2.Y) / 2f;
//    Point center = new Point((int)centerXX, (int)centerY);

//    Mat debugLine = inputBgr.Clone();
//    // 5) Bu dik kenar çizgilerini maviyle çiz
//    CvInvoke.Line(debugLine, p1s, p2s, new MCvScalar(255, 0, 0), 2);
//    CvInvoke.Line(debugLine, p1s, d1s, new MCvScalar(255, 0, 0), 2);
//    CvInvoke.Line(debugLine, p2s, d2s, new MCvScalar(255, 0, 0), 2);
//    CvInvoke.Line(debugLine, d1s, d2s, new MCvScalar(255, 0, 0), 2);

//    string label = $"C";
//    CvInvoke.Circle(debugLine, center, 8, new MCvScalar(0, 0, 255), 2);
//    CvInvoke.PutText(debugLine, label, new Point((int)center.X + 10, (int)center.Y + 5),
//                     FontFace.HersheySimplex, 0.7, new MCvScalar(0, 0, 255), 2);

//    //CvInvoke.ResizeForFrame(debugLine, debugLine, new Size(600, 600));
//    //CvInvoke.Imshow("Valley Line", debugLine);

//    Mat debugLine2 = inputBgr.Clone();

//    CvInvoke.Line(debugLine2, p1s, p2s, new MCvScalar(255, 0, 0), 2);
//    CvInvoke.Line(debugLine2, p1s, d1s, new MCvScalar(255, 0, 0), 2);
//    CvInvoke.Line(debugLine2, p2s, d2s, new MCvScalar(255, 0, 0), 2);
//    CvInvoke.Line(debugLine2, d1s, d2s, new MCvScalar(255, 0, 0), 2);
//    //CvInvoke.ResizeForFrame(debugLine2, debugLine2, new Size(600, 600));
//    //CvInvoke.Imshow("Valley Line 2", debugLine2);

//    // 1) Kaynak köşeler (Point[]):
//    var src = new[] { p1s, p2s, d2s, d1s };

//    using var polyMask = Mat.Zeros(inputBgr.Size.Height,inputBgr.Size.Width, DepthType.Cv8U, 1);
//    CvInvoke.FillConvexPoly(polyMask, new VectorOfPoint(src), new MCvScalar(255));

//    var m = polyMask.Clone();
//    //CvInvoke.ResizeForFrame(m, m, new Size(600, 600));
//    //CvInvoke.Imshow("Valley Line", m);

//    Mat maskedPalm = new Mat();
//    CvInvoke.BitwiseAnd(inputBgr, inputBgr, maskedPalm, polyMask);

//    var mp = maskedPalm.Clone();
//    //CvInvoke.ResizeForFrame(mp, mp, new Size(600, 600));
//    //CvInvoke.Imshow("Valley Line", mp);

//    bb = CvInvoke.BoundingRectangle(new VectorOfPoint(src));
//    bb.Intersect(new Rectangle(Point.Empty, inputBgr.Size));
//    if (bb.Width <= 0 || bb.Height <= 0) return false;

//    Mat rawROI = new Mat(maskedPalm, bb);
//    Mat rawMaskROI = new Mat(polyMask, bb);

//    float cx = rawROI.Width / 2f;
//    float cy = rawROI.Height / 2f;
//    PointF centerr = new PointF(cx, cy);

//    double theta = Math.Atan2(p1.Y - p2.Y, p1.X - p2.X) * 180 / Math.PI;
//    Debug.WriteLine($"Aradaki açı: {theta}");

//    // Rotasyon matrixi hesapla
//    Mat rotMat = new Mat();
//    CvInvoke.GetRotationMatrix2D(centerr, theta, 1.0, rotMat);

//    Mat finalROI = new();
//    Mat finalMask = new Mat();
//    CvInvoke.WarpAffine(
//        rawROI, finalROI, rotMat, rawROI.Size,
//        Inter.Linear, Warp.Default,
//        BorderType.Constant, new MCvScalar(0)
//    );
//    CvInvoke.WarpAffine(
//        rawMaskROI, finalMask, rotMat, rawMaskROI.Size,
//        Inter.Nearest, Warp.Default,
//        BorderType.Constant, new MCvScalar(0)
//    );
//    palmROI = finalROI;
//    dtMask = finalMask;

//    return true;
//}