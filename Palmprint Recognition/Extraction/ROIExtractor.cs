using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Face;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using Emgu.CV.XImgproc;
using System.Diagnostics;
using System.Globalization;
using System.Net;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics.X86;

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
            palmROI = new();
            dtMask = new();

            if (!IsValidInput(inputBgr))
                return false;

            // 1) Preprocess: normalize lighting and binarize
            Mat noGlare = RemoveSpecularHighlights(inputBgr);
            LightingNormalize(noGlare);

            // 2) Binarize and prepare mask
            Mat binaryImg = BinaryImage(noGlare);
            binaryImg = AddBorder(binaryImg, 5);
            Mat proc = binaryImg.Clone();
            
            using var ker = CvInvoke.GetStructuringElement(ElementShape.Rectangle, new Size(5, 5), new Point(-1, -1));
            CvInvoke.Erode(proc, proc, ker, new Point(-1, -1), 3, BorderType.Constant, new MCvScalar(0));
            ShowBinaryImage(proc, new Size(600, 600)); // Optional: for debugging

            // 3) Find hand contour
            VectorOfPoint? rawContour = FindLargestContour(proc);
            if (rawContour == null || rawContour.Size < 3)
                return false;
            using var contour = rawContour;

            // 4) Detect valley points
            List<Point> valleys = GetValleyPointsFlexible(contour);
            Debug.WriteLine($"Valley count: {valleys.Count}");

            if (valleys.Count < 3) return false;

            // 5) Determine handedness and order valleys
            bool isLeftHand = IsLeftHandCombined(contour, valleys, 20);
            List<Point> orderedValleys = SortValleys(valleys, isLeftHand);

            // 6) Compute source polygon and extract raw ROI
            Point[] srcPoints = ComputeSourcePointsScaledOffset(orderedValleys, isLeftHand);
            (Mat rawROI, Mat rawMask) = ExtractRawROI(inputBgr, srcPoints);
            DrawHandFeatures(inputBgr, contour, orderedValleys, srcPoints, new Size(600, 600), 6);

            // 7) Compute rotation angle and align
            double theta = ComputeRotationAngle(orderedValleys, isLeftHand);
            AlignROI(rawROI, rawMask, theta, out palmROI, out dtMask);

            // 8 Crop align roi
            (palmROI, _) = CropToMask(palmROI, dtMask);

            LightingNormalize(palmROI);
            dtMask = binaryImg.Clone();
            return true;

        }
        private bool IsValidInput(Mat input)
        {
            return input != null && !input.IsEmpty;
        }
        private List<Point> GetValleyPoints(VectorOfPoint contour)
        {
            // 0) (Opsiyonel) Konturu yumuşatmak istersen buraya koyabilirsin:
            double eps = 0.01 * CvInvoke.ArcLength(contour, true);
            var smooth = new VectorOfPoint();
            CvInvoke.ApproxPolyDP(contour, smooth, eps, true);
            contour = smooth;

            // 1) Convex hull indeksleri
            using var hullIdx = new VectorOfInt();
            CvInvoke.ConvexHull(contour, hullIdx, true, false);

            // 2) Convexity defects
            using var defectMat = new Mat();
            CvInvoke.ConvexityDefects(contour, hullIdx, defectMat);

            var valleys = new List<Point>();
            if (defectMat.GetData() is int[,,] d)
            {
                // ───────────────────────────────────────────────────────────────
                // Burada eşik değerlerini ayarla:
                var bb = CvInvoke.BoundingRectangle(contour);

                // Derinlik eşiği: el yüksekliğinin %7'si (20 piksel yerine dinamik)
                int minDepth = Math.Max(10, (int)(bb.Height * 0.07));

                // Y‐ekseni bandı: sadece palm dip bölgeleri (%15–%80 arası)
                double topThresh = bb.Y + bb.Height * 0.05;
                double bottomThresh = bb.Y + bb.Height * 0.80;
                // ───────────────────────────────────────────────────────────────

                for (int i = 0; i < d.GetLength(0); i++)
                {
                    int startIdx = d[i, 0, 0];
                    int endIdx = d[i, 0, 1];
                    int farIdx = d[i, 0, 2];
                    int depth = d[i, 0, 3];

                    int pixDepth = depth >> 8;
                    Point pt = contour[farIdx];
                    Debug.WriteLine($"defect[{i}] depth={pixDepth} at {pt} Y‐band? {pt.Y} top={topThresh:F1} bot={bottomThresh:F1}");

                    // 3) Depth filtresi
                    if (pixDepth < minDepth)
                        continue;
                    // 4) Dikey bant filtresi
                    if (pt.Y < topThresh || pt.Y > bottomThresh)
                        continue;

                    // 5) Açı filtresi (opsiyonel)
                    var v1 = new PointF(contour[startIdx].X - pt.X, contour[startIdx].Y - pt.Y);
                    var v2 = new PointF(contour[endIdx].X - pt.X, contour[endIdx].Y - pt.Y);
                    double dot = v1.X * v2.X + v1.Y * v2.Y;
                    double ang = Math.Acos(dot /
                                (Math.Sqrt(v1.X * v1.X + v1.Y * v1.Y) * Math.Sqrt(v2.X * v2.X + v2.Y * v2.Y)))
                                 * 180.0 / Math.PI;
                    if (ang < 10 || ang > 120)
                        continue;

                    valleys.Add(pt);
                }
            }
            return valleys;
        }
        private List<Point> GetValleyPointsFlexible(VectorOfPoint contour)
        {
            // 1) Hull ve defects
            using var hullIdx = new VectorOfInt();
            CvInvoke.ConvexHull(contour, hullIdx, false, false);
            using var defectMat = new Mat();
            CvInvoke.ConvexityDefects(contour, hullIdx, defectMat);

            var bb = CvInvoke.BoundingRectangle(contour);
            double topY = bb.Y + bb.Height * 0.05;
            double botY = bb.Y + bb.Height * 0.80;

            var list = new List<(int farIdx, int depthPix, double angle)>();

            if (defectMat.GetData() is int[,,] d)
            {
                for (int i = 0; i < d.GetLength(0); i++)
                {
                    int s = d[i, 0, 0], e = d[i, 0, 1], f = d[i, 0, 2], depthRaw = d[i, 0, 3];
                    int depthPix = depthRaw >> 8;       // 256 sabit-nokta düzeltmesi
                    var pt = contour[f];

                    // 1) Y-band filtresi:
                    if (pt.Y < topY || pt.Y > botY)
                        continue;

                    // 2) Açı filtresi (opsiyonel ama faydalı)
                    var v1 = new PointF(contour[s].X - pt.X, contour[s].Y - pt.Y);
                    var v2 = new PointF(contour[e].X - pt.X, contour[e].Y - pt.Y);
                    double dot = v1.X * v2.X + v1.Y * v2.Y;
                    double ang = Math.Acos(dot / (
                                 Math.Sqrt(v1.X * v1.X + v1.Y * v1.Y) *
                                 Math.Sqrt(v2.X * v2.X + v2.Y * v2.Y)
                               )) * 180.0 / Math.PI;
                    if (ang < 15 || ang > 120)
                        continue;

                    list.Add((f, depthPix, ang));
                }
            }

            // 3) Derinliğe göre en derin 5 farIdx seç
            var valleys = list
              .OrderByDescending(x => x.depthPix)
              .Take(4)
              .Select(x => contour[x.farIdx])
              .OrderBy(p => p.X)    // soldan sağa
              .ToList();

            return valleys;
        }





        //private List<Point> GetValleyPoints(VectorOfPoint contour)
        //{
        //    // Compute convex hull indices
        //    using var hullIdx = new VectorOfInt();
        //    CvInvoke.ConvexHull(contour, hullIdx, false, false);

        //    // Calculate convexity defects
        //    using var defectMat = new Mat();
        //    CvInvoke.ConvexityDefects(contour, hullIdx, defectMat);

        //    var valleys = new List<Point>();
        //    if (defectMat.GetData() is int[,,] d)
        //    {
        //        // Bounding box to filter by vertical position
        //        var bb = CvInvoke.BoundingRectangle(contour);
        //        double topThresh = bb.Y + bb.Height * 0.2; // üst %15’i atla
        //        double bottomThresh = bb.Y + bb.Height * 0.75; // alt %25’i atla
        //        //double yThresh = bb.Y + bb.Height * 0.2;
        //        int minDepth = Math.Max(20, (int)(bb.Height * 0.1));

        //        for (int i = 0; i < d.GetLength(0); i++)
        //        {
        //            int startIdx = d[i, 0, 0];
        //            int endIdx = d[i, 0, 1];
        //            int farIdx = d[i, 0, 2];
        //            int depth = d[i, 0, 3];

        //            // Depth threshold
        //            if (depth < minDepth)
        //                continue;

        //            Point pt = contour[farIdx];
        //            // Exclude defects too high (finger tips)
        //            if (pt.Y < topThresh || pt.Y > bottomThresh)
        //                continue;

        //            // Angle between start→far and end→far vectors
        //            var v1 = new PointF(contour[startIdx].X - pt.X, contour[startIdx].Y - pt.Y);
        //            var v2 = new PointF(contour[endIdx].X - pt.X, contour[endIdx].Y - pt.Y);
        //            double dot = v1.X * v2.X + v1.Y * v2.Y;
        //            double ang = Math.Acos(dot / (
        //                Math.Sqrt(v1.X * v1.X + v1.Y * v1.Y) *
        //                Math.Sqrt(v2.X * v2.X + v2.Y * v2.Y)
        //            )) * 180.0 / Math.PI;

        //            if (ang < 20 || ang > 120)
        //                continue;

        //            valleys.Add(pt);
        //        }
        //    }

        //    return valleys;
        //}
        private bool IsLeftHandCombined1(
            VectorOfPoint contour,
            List<Point> valleys,
            double ellipseEpsilon = 5.0     // ellipse projeksi­yon eşiği
        )   
        {
            // 1) Fallback‐öncesi: thumb–Y metodu
            bool valleyDecision = IsLeftHandByThumbValleyY(contour, valleys);

            // 2) Valley‐metodunun güvenirliği: 
            //    en az 2 vadi, ve merkezden X farkı belli bir eşiğin üzerinde olmalı
            //    (30 px örnek, ya da bb.Width * 0.1 gibi oransal)
            var bb = CvInvoke.BoundingRectangle(contour);
            int centerX = bb.X + bb.Width / 2;

            // thumb–Y ile seçilen vadinin X uzaklığı
            Point thumbVal = valleys
                .OrderByDescending(p => p.Y)
                .First();
            int distX = Math.Abs(thumbVal.X - centerX);

            bool valleyReliable = valleys.Count >= 2
                                  && distX >= Math.Max(10, bb.Width / 10);

            if (valleyReliable)
            {
                Debug.WriteLine($"Handedness by thumb‐Y : {valleyDecision}");
                return valleyDecision;
            }

            // 3) Buraya geldiyse valley‐metodu belirsiz kaldı → ellipse‐projeksiyon
            //    (orijinal IsLeftHandCombined kodundan kopya)
            RotatedRect ellipse = CvInvoke.FitEllipse(contour);
            PointF eCenter = ellipse.Center;
            double angRad = ellipse.Angle * Math.PI / 180.0;
            double ux = Math.Cos(angRad), uy = Math.Sin(angRad);
            double nx = -uy, ny = ux;

            var m = CvInvoke.Moments(contour);
            double cx = m.M10 / m.M00, cy = m.M01 / m.M00;
            double proj = (cx - eCenter.X) * nx + (cy - eCenter.Y) * ny;

            bool ellipseDecision = proj > 0;
            Debug.WriteLine($"Handedness by ellipse fallback: {ellipseDecision} (proj={proj:F1})");

            return Math.Abs(proj) >= ellipseEpsilon
                 ? ellipseDecision
                 : valleyDecision;   // eğer ellipse de çok belirsizse,  
                                     // yine valley kararına dön
        }
        /// <summary>
        /// Birincil yöntem: el konturuna fitEllipse → centroid projeksiyonu.
        /// Eğer |proj| < epsilon ise (belirsiz bölge), fallback olarak
        /// valley‐angle yöntemini kullanır.
        /// </summary>
        private bool IsLeftHandCombined(
            VectorOfPoint contour,
            List<Point> valleys,
            double epsilon = 5.0   // piksel cinsinden eşik
        )
        {
            // --- 1) Elips‐projeksiyon yöntemi ---
            // Uygun kontur uzunluğu kontrolü
            if (contour == null || contour.Size < 5)
                throw new ArgumentException("En az 5 noktalı contour gerekli.");

            //// 1.1) Ellipse fit
            //RotatedRect ellipse = CvInvoke.FitEllipse(contour);
            //PointF eCenter = ellipse.Center;
            //double angRad = ellipse.Angle * Math.PI / 180.0;

            //// 1.2) Ellips minor-ekseni normali
            //double ux = Math.Cos(angRad), uy = Math.Sin(angRad);
            //double nx = -uy, ny = ux;

            //// 1.3) Contour centroid
            //var m = CvInvoke.Moments(contour);
            //double cx = m.M10 / m.M00, cy = m.M01 / m.M00;

            //// 1.4) Projeksiyon
            //double dx = cx - eCenter.X;
            //double dy = cy - eCenter.Y;
            //double proj = dx * nx + dy * ny;

            //// Eğer yeterince büyükse (güvenli karar)
            //if (Math.Abs(proj) >= epsilon)
            //{
            //    // proj>0 → centroid, normal yönünde → left hand
            //    Debug.WriteLine($"Elips method: {proj > 0}");
            //    return proj > 0;
            //}

            // --- 2) Fallback: valley‐angle yöntemi ---
            return IsLeftHandByThumbValleyY(contour, valleys);

        }

        /// <summary>
        /// Fallback: en küçük Y’ye (en yukarıdaki) valley’i thumb-index çukuru
        /// kabul edip onun bounding-box merkezine göre sol/sağ karar verir.
        /// </summary>
        private bool IsLeftHandByThumbValleyY(VectorOfPoint contour, List<Point> valleys)
        {
            // 1) El konturunun bounding-box ve merkezi
            var bb = CvInvoke.BoundingRectangle(contour);
            int centerX = bb.X + bb.Width / 2;
            int centerY = bb.Y + bb.Height / 2;

            // 2) En yukarıdaki (min Y) valley’i al
            //    bu valley thumb-index arasındaki çukurdur varsayıyoruz
            Point thumbValley = valleys
                .OrderByDescending(p => p.Y)
                .First();
            for (int i = 0; i < valleys.Count; i++)
            {
                Debug.WriteLine($"Valley-{i} loc: ({valleys[i].X},{valleys[i].Y})");
            }
            Debug.WriteLine($"Thumb valley at X={thumbValley.X}, centerX={centerX}");
            Debug.WriteLine($"Thumb valley method: {thumbValley.X > centerX}");

            // 3) Eğer o nokta centerX’in sağındaysa sol el, solundaysa sağ el
            return thumbValley.X < centerX;
        }

        private bool IsLeftHandByFurthestValley(VectorOfPoint contour, List<Point> valleys)
        {
            var bb = CvInvoke.BoundingRectangle(contour);
            int centerX = bb.X + bb.Width / 2;
            var thumbValley = valleys.OrderByDescending(p => Math.Abs(p.X - centerX)).First();
            Debug.WriteLine(thumbValley.X > centerX);
            return thumbValley.X > centerX;
        }
        private List<Point> SortValleys(List<Point> valleys, bool isLeftHand)
        {
            return isLeftHand
                ? valleys.OrderBy(p => p.X).ToList()
                : valleys.OrderByDescending(p => p.X).ToList();
        }

        /// <summary>
        /// İki valley arası mesafeyi d olarak alır,
        /// - yatayda scale·d büyütür (scale ≥ 1),
        /// - valley çizgisinden palm içerisine doğru offsetDown·d öteler (offsetDown ≥ 0),
        /// ve döndürülmüş dörtgen köşelerini döner.
        /// </summary>
        private Point[] ComputeSourcePointsScaledOffset(
            List<Point> orderedValleys,
            bool isLeftHand,
            double scale = 1.4,
            double offsetDown = 0.2
        )
        {
            // 1) p1, p2 seçimi
            Point p1, p2;
            if (orderedValleys.Count >= 4)
            {
                p1 = orderedValleys[1];
                p2 = orderedValleys[3];
            }
            else if (orderedValleys.Count == 3)
            {
                p1 = orderedValleys[1];
                p2 = orderedValleys[2];
            }
            else
            {
                p1 = orderedValleys[0];
                p2 = orderedValleys[1];
            }

            // 2) d ve birim valley→valley vektörü û
            double dx = p2.X - p1.X, dy = p2.Y - p1.Y;
            double d = Math.Sqrt(dx * dx + dy * dy);
            double ux = dx / d, uy = dy / d;

            // 3) içeri bakan normal (avuç içi yönü)
            double nx = dy / d;
            double ny = -dx / d;
            if (isLeftHand)
            {
                nx = -nx;
                ny = -ny;
            }

            // 4) Yatayda pad = (scale-1)*d/2
            double padH = (scale - 1) * d * 0.5;

            // 5) valley line üzeri scaled top-köşeler
            var p1e = new Point(
                (int)Math.Round(p1.X - ux * padH),
                (int)Math.Round(p1.Y - uy * padH)
            );
            var p2e = new Point(
                (int)Math.Round(p2.X + ux * padH),
                (int)Math.Round(p2.Y + uy * padH)
            );

            // 6) Aşağı offset = offsetDown * d
            double padV = offsetDown * d;

            // 7) Ölçekli kenar boyu = scale*d
            double newD = scale * d;

            // 8) ROI’nin dört köşesi:
            //    - Üst sol: p1e + normal*padV
            //    - Üst sağ: p2e + normal*padV
            //    - Alt sağ: p2e + normal*(padV + newD)
            //    - Alt sol: p1e + normal*(padV + newD)
            Point topLeft = new Point(
                (int)Math.Round(p1e.X + nx * padV),
                (int)Math.Round(p1e.Y + ny * padV)
            );
            Point topRight = new Point(
                (int)Math.Round(p2e.X + nx * padV),
                (int)Math.Round(p2e.Y + ny * padV)
            );
            Point bottomRight = new Point(
                (int)Math.Round(p2e.X + nx * (padV + newD)),
                (int)Math.Round(p2e.Y + ny * (padV + newD))
            );
            Point bottomLeft = new Point(
                (int)Math.Round(p1e.X + nx * (padV + newD)),
                (int)Math.Round(p1e.Y + ny * (padV + newD))
            );

            return [topLeft, topRight, bottomRight, bottomLeft];
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
            try
            {
                CvInvoke.CLAHE(channels[0], 2.0, new Size(8, 8), channels[0]);
                using var merged = new Mat();
                CvInvoke.Merge(new VectorOfMat(channels), merged);
                CvInvoke.CvtColor(merged, image, ColorConversion.Lab2Bgr);
                //CvInvoke.GaussianBlur(image, image, new Size(3, 3), 0);
                using var denoised = new Mat();
                CvInvoke.BilateralFilter(
                    image,      // src: orijinal
                    denoised,   // dst: sonuç için boş Mat
                    9,          // diameter
                    75,         // sigmaColor
                    75          // sigmaSpace
                );
                denoised.CopyTo(image);
            }
            finally
            {
                foreach (var ch in channels)
                {
                    ch.Dispose();
                }
            }
        }

        private Mat BinaryImage(Mat image)
        {
            using var ycc = new Mat();
            CvInvoke.CvtColor(image, ycc, ColorConversion.Bgr2YCrCb);
            var mask = new Mat();
            CvInvoke.InRange(ycc,
                new ScalarArray(new MCvScalar(0, 140, 75)),
                new ScalarArray(new MCvScalar(255, 180, 135)),
                mask);

            using var kernel = CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(7, 7), new Point(-1, -1));
            CvInvoke.MorphologyEx(mask, mask, MorphOp.Open, kernel, Point.Empty, 3, BorderType.Default, new MCvScalar());
            CvInvoke.MorphologyEx(mask, mask, MorphOp.Close, kernel, Point.Empty, 1, BorderType.Default, new MCvScalar());

            // Ufak delikleri doldur
            //FillSmallHoles(mask, maxHoleArea: 200);

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

            //CvInvoke.GaussianBlur(repaired, repaired, new Size(5, 5), sigmaX: 1.5);
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

        #region Debug Methods
        private void DrawHandFeatures(Mat originalBgr, VectorOfPoint contour,
            List<Point> valleys, Point[] corners,
            Size windowSize, int thickness = 6,
            bool drawBoundingBox = true, bool drawCountour = true, bool drawValleyPts = true, bool drawROIArea = true)
        {
            Mat debug = originalBgr.Clone();

            // Draw contour
            if (drawCountour)
            {
                CvInvoke.DrawContours(debug, new VectorOfVectorOfPoint(contour), -1, new MCvScalar(255, 0, 0), thickness);
            }
            // Draw valley points
            if (drawValleyPts)
            {
                for (int i = 0; i < valleys.Count; i++)
                {
                    var p = valleys[i];
                    string label1 = $"P{i + 1}";
                    CvInvoke.Circle(debug, p, 16, new MCvScalar(0, 0, 255), thickness);
                    CvInvoke.PutText(debug, label1, new Point(p.X + 10, p.Y + 5),
                                     FontFace.HersheySimplex, 1.5, new MCvScalar(0, 0, 255), thickness);
                }
            }
            // Draw bounding box and center
            if (drawBoundingBox)
            {
                var bb = CvInvoke.BoundingRectangle(contour);
                int centerX = bb.X + bb.Width / 2;
                int centerY = bb.Y + bb.Height / 2;
                CvInvoke.Rectangle(debug, bb, new MCvScalar(0, 255, 0), thickness);
                CvInvoke.Circle(debug, new Point(centerX, centerY), 15, new MCvScalar(255, 255, 0), thickness);
            }
            // Draw ROI Rectangle
            if (drawROIArea)
            {
                for (int i = 0; i < 4; i++)
                {
                    Point pA = corners[i];
                    Point pB = corners[(i + 1) % 4];
                    CvInvoke.Line(debug, pA, pB, new MCvScalar(255, 0, 0), thickness);
                }
                float cx = (corners[0].X + corners[2].X) / 2f;
                float cy = (corners[0].Y + corners[2].Y) / 2f;
                CvInvoke.Circle(debug, new Point((int)cx, (int)cy), 5, new MCvScalar(0, 0, 255), thickness);
            }

            CvInvoke.ResizeForFrame(debug, debug, windowSize);
            CvInvoke.Imshow("Hand Features", debug);
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