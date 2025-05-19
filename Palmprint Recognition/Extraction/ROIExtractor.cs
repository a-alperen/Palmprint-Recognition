using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Face;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System.Runtime.InteropServices;

namespace Palmprint_Recognition.Extraction
{
    internal class ROIExtractor
    {
        /// <summary>
        /// Tries to extract a consistent palm ROI: first via valley-defect method; on failure, falls back to color+DT refine.
        /// </summary>
        public bool TryExtract(Mat inputBgr, out Mat palmROI, out Mat dtMask)
        {
            palmROI = null;
            dtMask = null;
            if (inputBgr == null || inputBgr.IsEmpty)
                return false;

            //1) Normalize lighting + binary image
            LightingNormalize(inputBgr);
            Mat binaryImg = BinaryImage(inputBgr);

            // 2) En büyük konturu bulup doldur
            var contour = FindLargestContour(binaryImg);
            if (contour == null) return false;
            Mat handMask = new Mat(binaryImg.Size, DepthType.Cv8U, 1);
            CvInvoke.DrawContours(handMask, new VectorOfVectorOfPoint(contour), 0, new MCvScalar(255), -1);

            // 3) ConvexityDefects ile valley noktalarını al
            var hullIdx = new VectorOfInt();
            CvInvoke.ConvexHull(contour, hullIdx, false, false);

            Mat defectMat = new Mat();
            CvInvoke.ConvexityDefects(contour, hullIdx, defectMat);
            var d = defectMat.GetData() as int[,,];

            // Konturun bounding‐box'unu al
            var bb = CvInvoke.BoundingRectangle(contour);
            double yThresh = bb.Y + bb.Height * 0.2;    // en üstten %20 kadar aşağıdaki noktalar
            double minDepth = 20;                       // derinlik eşiği (örneğin 20 piksel)

            var valleys = new List<Point>();
            if (d != null)
            {
                for (int i = 0; i < d.GetLength(0); i++)
                {
                    int startIdx = d[i, 0, 0], endIdx = d[i, 0, 1], farIdx = d[i, 0, 2];
                    int depth = d[i, 0, 3];

                    // 1) Derinlik eşiğini geçsin
                    if (depth < minDepth)
                        continue;

                    // 2) Konum: çok yukarıda (finger tip civarı) olanları ele
                    var pt = contour[farIdx];
                    if (pt.Y < yThresh)
                        continue;

                    // 3) (Opsiyonel) Dar açılı defect’leri el opsu değil parmak dipine uygun açılarla süz
                    //    Vektörler: v1 = start→far, v2 = end→far
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

            // Şimdi valleys listesi yalnızca gerçek parmak arası vadileri içeriyor
            if (valleys.Count < 2)
                return false;
            Mat debug = inputBgr.Clone();  // veya alignedColor.Clone()
            foreach (var p in valleys)
            {
                CvInvoke.Circle(debug, p, 8, new MCvScalar(0, 0, 255), 3);
            }
            CvInvoke.ResizeForFrame(debug,debug,new Size(400,400));
            CvInvoke.Imshow("Valley Points", debug);
            // 4) En üst iki valley → valleyY ve midX
            valleys = valleys.OrderBy(p => p.Y).Take(2).ToList();
            int valleyY = Math.Min(valleys[0].Y, valleys[1].Y);
            int midX = (valleys[0].X + valleys[1].X) / 2;

            // 5) Valley çizgisinin altından kare ROI oluştur
            int h = inputBgr.Height - valleyY;
            int w = h;
            int x = Math.Max(0, midX - w / 2);
            var roiRect = new Rectangle(x, valleyY, w, h);
            roiRect.Intersect(new Rectangle(0, 0, inputBgr.Width, inputBgr.Height));
            if (roiRect.Width <= 0 || roiRect.Height <= 0)
                return false;

            // 5) Sonuç: renkli palmROI
            palmROI = new Mat(inputBgr, roiRect);
            dtMask = handMask.Clone();
            return true;
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
            CvInvoke.MorphologyEx(mask, mask, MorphOp.Open, kernel, Point.Empty, 1, BorderType.Default, new MCvScalar());
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
        private VectorOfPoint FindLargestContour(Mat mask)
        {
            var contours = new VectorOfVectorOfPoint();
            CvInvoke.FindContours(mask, contours, null, RetrType.External, ChainApproxMethod.ChainApproxSimple);
            if (contours.Size == 0) return null;
            int idx = Enumerable.Range(0, contours.Size)
                         .OrderByDescending(i => CvInvoke.ContourArea(contours[i]))
                         .First();
            return contours[idx];
        }
        /// <summary>
        /// Finds the two deepest “valley” points (finger interdigital clefts) on the hand contour
        /// using OpenCV’s ConvexityDefects, then computes an affine transform that aligns
        /// the hand so those two points lie on a horizontal line.
        /// </summary>
        /// <param name="handContour">The filled hand contour (VectorOfPoint)</param>
        /// <param name="alignedColor">Output: the color image warped so valley line is horizontal</param>
        /// <param name="alignedMask">Output: the binary mask warped the same way</param>
        /// <param name="srcColor">Input color image (Mat)</param>
        /// <param name="srcMask">Input mask image (Mat)</param>
        private void AlignByValleyDefects(
            VectorOfPoint handContour,
            Mat srcColor,
            Mat srcMask,
            out Mat alignedColor,
            out Mat alignedMask)
        {
            // 1) Compute convex hull indices
            var hullIdx = new VectorOfInt();
            CvInvoke.ConvexHull(handContour, hullIdx, false, false);

            // 2) Compute convexity defects
            Mat defectMat = new Mat();
            CvInvoke.ConvexityDefects(handContour, hullIdx, defectMat);
            int[,,] data = defectMat.GetData() as int[,,];

            // 3) Gather all (farPoint, depth) pairs
            var defects = new List<(Point pt, int depth)>();
            if (data != null)
            {
                for (int i = 0; i < data.GetLength(0); i++)
                {
                    int farIdx = data[i, 0, 2];
                    int depth = data[i, 0, 3];
                    defects.Add((handContour[farIdx], depth));
                }
            }

            if (defects.Count < 2)
            {
                // Fallback: no defect alignment
                alignedColor = srcColor.Clone();
                alignedMask = srcMask.Clone();
                return;
            }

            // 4) Select the two defects with greatest depth
            var top2 = defects
                .OrderByDescending(d => d.depth)
                .Take(2)
                .Select(d => d.pt)
                .ToArray();
            Point p1 = top2[0];
            Point p2 = top2[1];

            // 5) Compute angle between them
            double theta = Math.Atan2(p2.Y - p1.Y, p2.X - p1.X) * 180.0 / Math.PI;

            // 6) Build affine warp matrix around midpoint
            PointF center = new PointF((p1.X + p2.X) / 2f, (p1.Y + p2.Y) / 2f);
            Mat M = new();
            CvInvoke.GetRotationMatrix2D(center, theta, 1.0, M);

            // 7) Apply to both color and mask
            alignedColor = new Mat();
            alignedMask = new Mat();
            CvInvoke.WarpAffine(srcColor, alignedColor, M, srcColor.Size,
                                Inter.Linear, Warp.Default, BorderType.Constant, new MCvScalar(0));
            CvInvoke.WarpAffine(srcMask, alignedMask, M, srcMask.Size,
                                Inter.Nearest, Warp.Default, BorderType.Constant, new MCvScalar(0));
        }

        private Mat ComputeRotationMatrix(VectorOfPoint contour)
        {
            var ptsF = contour.ToArray().Select(p => new PointF(p.X, p.Y)).ToArray();
            var box = CvInvoke.MinAreaRect(ptsF);
            float angle = (box.Size.Width < box.Size.Height ? box.Angle + 90f : box.Angle) - 90f;
            if (Math.Abs(angle) < 10f) angle = 0f;
            var M = new Mat();
            CvInvoke.GetRotationMatrix2D(box.Center, -angle, 1.0, M);
            return M;
        }

        private void WarpAlign(Mat image, Mat mask, Mat M, Mat dstImage, Mat dstMask)
        {
            CvInvoke.WarpAffine(image, dstImage, M, image.Size, Inter.Linear, Warp.Default, BorderType.Constant, new MCvScalar(0));
            CvInvoke.WarpAffine(mask, dstMask, M, mask.Size, Inter.Nearest, Warp.Default, BorderType.Constant, new MCvScalar(0));
        }

        private Rectangle GetBoundingBox(Mat mask, int pad, Size clampSize)
        {
            var contours = new VectorOfVectorOfPoint();
            CvInvoke.FindContours(mask, contours, null, RetrType.External, ChainApproxMethod.ChainApproxSimple);
            if (contours.Size == 0) return Rectangle.Empty;
            int idx = Enumerable.Range(0, contours.Size)
                         .OrderByDescending(i => CvInvoke.ContourArea(contours[i]))
                         .First();
            var box = CvInvoke.BoundingRectangle(contours[idx]);
            box.Inflate(pad, pad);
            box.Intersect(new Rectangle(Point.Empty, clampSize));
            return box;
        }

        private Mat RefinePalmInterior(Mat colorROI, Mat maskROI, out Mat dtMask)
        {
            using var ycc = new Mat();
            CvInvoke.CvtColor(colorROI, ycc, ColorConversion.Bgr2YCrCb);
            var channels = ycc.Split();
            CvInvoke.GaussianBlur(channels[1], channels[1], new Size(5, 5), 0);
            var crMask = new Mat();
            CvInvoke.Threshold(channels[1], crMask, 0, 255, ThresholdType.Binary | ThresholdType.Otsu);

            using var hsv = new Mat();
            CvInvoke.CvtColor(colorROI, hsv, ColorConversion.Bgr2Hsv);
            var hsvMask = new Mat();
            CvInvoke.InRange(hsv, new ScalarArray(new MCvScalar(0, 30, 60)), new ScalarArray(new MCvScalar(50, 150, 255)), hsvMask);

            var mask = new Mat();
            CvInvoke.BitwiseAnd(maskROI, hsvMask, mask);
            CvInvoke.BitwiseAnd(mask, crMask, mask);
            using var kernel = CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(5, 5), new Point(-1, -1));
            CvInvoke.MorphologyEx(mask, mask, MorphOp.Open, kernel, new Point(-1, -1), 1, BorderType.Default, new MCvScalar());
            CvInvoke.MorphologyEx(mask, mask, MorphOp.Close, kernel, new Point(-1, -1), 2, BorderType.Default, new MCvScalar());

            var noSmall = new Mat(mask.Size, DepthType.Cv8U, 1);
            var contours = new VectorOfVectorOfPoint();
            CvInvoke.FindContours(mask, contours, null, RetrType.External, ChainApproxMethod.ChainApproxSimple);
            for (int i = 0; i < contours.Size; i++)
                if (CvInvoke.ContourArea(contours[i]) > 2000)
                    CvInvoke.DrawContours(noSmall, contours, i, new MCvScalar(255), -1);
            noSmall.CopyTo(mask);

            CvInvoke.MedianBlur(mask, mask, 7);

            // 1) Distance transform
            using var dist = new Mat();
            CvInvoke.DistanceTransform(mask, dist, null, DistType.L2, 5);

            // 2) Min/Max değerler ve lokasyonları için değişkenler:
            double minVal = 0, maxVal = 0;
            Point minLoc = new Point(), maxLoc = new Point();

            // 3) Çağrı: mask=null diyerek yalnızca görüntü üzerinde çalışıyoruz
            CvInvoke.MinMaxLoc(
                dist,
                ref minVal,
                ref maxVal,
                ref minLoc,
                ref maxLoc,
                null
            );

            // 4) Şimdi maxVal gerçek en yüksek mesafe değeri
            float thresh = (float)(maxVal * 0.5);

            // 5) Normalize + eşikleme
            CvInvoke.Normalize(dist, dist, 0, 1.0, NormType.MinMax, DepthType.Cv32F);
            dtMask = new Mat();
            CvInvoke.Threshold(dist, dtMask, thresh, 1.0, ThresholdType.Binary);
            dtMask.ConvertTo(dtMask, DepthType.Cv8U, 255);


            var palmRect = GetBoundingBox(dtMask, 0, colorROI.Size);
            return palmRect == Rectangle.Empty
                ? colorROI.Clone()
                : new Mat(colorROI, palmRect);
        }
        private bool IsQualityOK(Mat maskROI)
        {
            var contours = new VectorOfVectorOfPoint();
            CvInvoke.FindContours(maskROI, contours, null, RetrType.External, ChainApproxMethod.ChainApproxSimple);
            if (contours.Size == 0) return false;
            int idx = Enumerable.Range(0, contours.Size)
                         .OrderByDescending(i => CvInvoke.ContourArea(contours[i]))
                         .First();
            var cnt = contours[idx];
            double area = CvInvoke.ContourArea(cnt);
            using var hull = new VectorOfPoint();
            CvInvoke.ConvexHull(cnt, hull, true);
            double solidity = area / CvInvoke.ContourArea(hull);
            var bbox = CvInvoke.BoundingRectangle(cnt);
            double extent = area / (bbox.Width * bbox.Height);
            const double MIN_SOLIDITY = 0.7, MIN_EXTENT = 0.3;
            return solidity >= MIN_SOLIDITY || extent >= MIN_EXTENT;
        }

    }
}
///// <summary>
///// Extracts the palm ROI and mask using valley-based alignment,
///// selecting the two deepest convexity defects for valley detection.
///// </summary>
//public bool TryExtract(Mat inputBgr, out Mat palmRoi, out Mat roiMask)
//{
//    palmRoi = new Mat();
//    roiMask = new Mat();
//    if (inputBgr == null || inputBgr.IsEmpty)
//        return false;

//    // 1) Normalize lighting (YCrCb)
//    Mat ycrcb = new Mat();
//    CvInvoke.CvtColor(inputBgr, ycrcb, ColorConversion.Bgr2YCrCb);
//    var ycc = new VectorOfMat();
//    CvInvoke.Split(ycrcb, ycc);
//    CvInvoke.EqualizeHist(ycc[0], ycc[0]);
//    CvInvoke.Merge(ycc, ycrcb);
//    Mat normalized = new Mat();
//    CvInvoke.CvtColor(ycrcb, normalized, ColorConversion.YCrCb2Bgr);

//    // 2) Skin segmentation in HSV
//    Mat hsv = new Mat();
//    CvInvoke.CvtColor(normalized, hsv, ColorConversion.Bgr2Hsv);
//    Mat mask = new Mat();
//    CvInvoke.InRange(hsv,
//        new ScalarArray(new MCvScalar(0, 15, 30)),
//        new ScalarArray(new MCvScalar(25, 255, 255)),
//        mask);

//    // 3) Morphological cleaning
//    Mat kernel = CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(7, 7), Point.Empty);
//    CvInvoke.MorphologyEx(mask, mask, MorphOp.Open, kernel, Point.Empty, 2, BorderType.Default, new MCvScalar());
//    CvInvoke.MorphologyEx(mask, mask, MorphOp.Close, kernel, Point.Empty, 3, BorderType.Default, new MCvScalar());

//    // 4) Extract largest contour (hand)
//    var contours = new VectorOfVectorOfPoint();
//    CvInvoke.FindContours(mask, contours, null, RetrType.External, ChainApproxMethod.ChainApproxSimple);
//    int bestIdx = -1;
//    double bestArea = 0;
//    for (int i = 0; i < contours.Size; i++)
//    {
//        double area = CvInvoke.ContourArea(contours[i]);
//        if (area > bestArea) { bestArea = area; bestIdx = i; }
//    }
//    if (bestIdx < 0) return false;
//    var handContour = contours[bestIdx];
//    Mat handMask = Mat.Zeros(mask.Size.Height, mask.Size.Width, DepthType.Cv8U, 1);
//    CvInvoke.DrawContours(handMask, contours, bestIdx, new MCvScalar(255), -1);

//    // 5) Convexity defects: select two deepest valleys
//    var hull = new VectorOfInt();
//    CvInvoke.ConvexHull(handContour, hull, false, false);
//    Mat defectMat = new Mat();
//    CvInvoke.ConvexityDefects(handContour, hull, defectMat);

//    var allDefects = new List<(Point pt, int depth)>();
//    int[,,] data = defectMat.GetData() as int[,,];
//    if (data != null)
//    {
//        int rows = data.GetLength(0);
//        for (int r = 0; r < rows; r++)
//        {
//            int farIdx = data[r, 0, 2];
//            int depth = data[r, 0, 3];
//            allDefects.Add((handContour[farIdx], depth));
//        }
//    }
//    if (allDefects.Count < 2) return false;
//    var top2 = allDefects
//        .OrderByDescending(d => d.depth)
//        .Take(2)
//        .ToArray();
//    Point p1 = top2[0].pt;
//    Point p2 = top2[1].pt;

//    // Debug: draw the selected valley points on normalized image
//    CvInvoke.Circle(normalized, p1, 5, new MCvScalar(0, 0, 255), 2);
//    CvInvoke.Circle(normalized, p2, 5, new MCvScalar(0, 0, 255), 2);

//    // 6) Valley-based alignment
//    double theta = Math.Atan2(p2.Y - p1.Y, p2.X - p1.X) * 180.0 / Math.PI;
//    Mat alignedImg = normalized.Clone();
//    Mat alignedMask = handMask.Clone();
//    if (Math.Abs(theta) > 5)
//    {
//        var center = new PointF((p1.X + p2.X) * 0.5f, (p1.Y + p2.Y) * 0.5f);
//        Mat rotMat = new();
//        CvInvoke.GetRotationMatrix2D(center, theta, 1.0, rotMat);
//        CvInvoke.WarpAffine(normalized, alignedImg, rotMat, normalized.Size);
//        CvInvoke.WarpAffine(handMask, alignedMask, rotMat, handMask.Size,
//            Inter.Linear, Warp.Default, BorderType.Constant, new MCvScalar(0));
//    }

//    // 7) Extract largest contour on aligned mask
//    var alignedContours = new VectorOfVectorOfPoint();
//    CvInvoke.FindContours(alignedMask, alignedContours, null, RetrType.External, ChainApproxMethod.ChainApproxSimple);
//    bestIdx = -1; bestArea = 0;
//    for (int i = 0; i < alignedContours.Size; i++)
//    {
//        double area = CvInvoke.ContourArea(alignedContours[i]);
//        if (area > bestArea) { bestArea = area; bestIdx = i; }
//    }
//    if (bestIdx < 0) return false;
//    handContour = alignedContours[bestIdx];

//    // 8) Define ROI cropping above valley line
//    int valleyY = Math.Min(p1.Y, p2.Y);
//    int midX = (p1.X + p2.X) / 2;
//    int height = Math.Max(1, normalized.Height - valleyY);
//    int width = height;
//    int x = Math.Max(0, midX - width / 2);
//    var roiRect = new Rectangle(x, valleyY, width, height);
//    roiRect.Intersect(new Rectangle(0, 0, alignedImg.Width, alignedImg.Height));

//    palmRoi = new Mat(alignedImg, roiRect);
//    roiMask = new Mat(alignedMask, roiRect);
//    return true;
//}

///// <summary>
///// (Optional) Straight-line path sampling with obstacle avoidance
///// </summary>
//private List<Point> ComputeAgentPath(Mat mask, Point a, Point b)
//{
//    var img = mask.ToImage<Gray, byte>();
//    var pts = new List<Point>();
//    int steps = (int)Math.Max(Math.Abs(b.X - a.X), Math.Abs(b.Y - a.Y));
//    for (int i = 0; i <= steps; i++)
//    {
//        int x = (int)(a.X + i * (b.X - a.X) / (double)steps);
//        int y = (int)(a.Y + i * (b.Y - a.Y) / (double)steps);
//        if (x >= 0 && y >= 0 && x < img.Width && y < img.Height && img.Data[y, x, 0] > 0)
//            pts.Add(new Point(x, y));
//    }
//    return pts;
//}


//public bool TryExtractUnified(Mat inputBgr, out Mat palmRoi, out Mat mask)
//{
//    palmRoi = null; mask = null;
//    if (inputBgr == null || inputBgr.IsEmpty) return false;

//    // 1) Gri + blur + Otsu BinaryInv ile el maskesi
//    var gray = new Mat(); CvInvoke.CvtColor(inputBgr, gray, ColorConversion.Bgr2Gray);
//    CvInvoke.GaussianBlur(gray, gray, new Size(5, 5), 0);
//    var bw = new Mat();
//    CvInvoke.Threshold(gray, bw, 0, 255, ThresholdType.Otsu | ThresholdType.BinaryInv);
//    var k = CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(7, 7), new Point(-1, -1));
//    CvInvoke.MorphologyEx(bw, bw, MorphOp.Close, k, new Point(-1, -1), 2, BorderType.Default, new MCvScalar());
//    CvInvoke.MorphologyEx(bw, bw, MorphOp.Open, k, new Point(-1, -1), 2, BorderType.Default, new MCvScalar());

//    // 2) El siluetinin kaba dikdörtgeni
//    var nz = new VectorOfPoint(); CvInvoke.FindNonZero(bw, nz);
//    if (nz.Size == 0) return false;
//    var handRect = CvInvoke.BoundingRectangle(nz);

//    // 3) Genişlik profili çıkar
//    int y0 = handRect.Y, y1 = handRect.Bottom, h = y1 - y0;
//    var widths = new int[h];
//    int maxW = 0;
//    for (int i = 0; i < h; i++)
//    {
//        using var row = bw.Row(y0 + i);
//        int w = CvInvoke.CountNonZero(row);
//        widths[i] = w;
//        maxW = Math.Max(maxW, w);
//    }

//    // 4) Bilek çizgisini arayın (varsa)
//    int wristLine = -1;
//    int thr = (int)(maxW * 0.5);  // genişliğin yarısına düşüş eşik
//    for (int i = h - 1; i >= 0; i--)
//    {
//        if (widths[i] > thr)
//        {
//            wristLine = y0 + i;
//            break;
//        }
//    }

//    Rectangle crop;
//    if (wristLine > 0 && wristLine - handRect.Y < handRect.Height * 0.9)
//    {
//        // bilek bulundu: sadece yukarıdaki kısmı al
//        crop = new Rectangle(
//            handRect.X,
//            handRect.Y,
//            handRect.Width,
//            wristLine - handRect.Y
//        );
//    }
//    else
//    {
//        // bilek yok gibi: tüm el bölgesini kare yaparak al
//        int side = Math.Max(handRect.Width, handRect.Height);
//        int cx = handRect.X + handRect.Width / 2, cy = handRect.Y + handRect.Height / 2;
//        int x0 = Math.Max(0, cx - side / 2), y0c = Math.Max(0, cy - side / 2);
//        side = Math.Min(side, Math.Min(inputBgr.Width, inputBgr.Height));
//        crop = new Rectangle(x0, y0c, side, side);
//    }

//    // sınır içinde kal
//    crop = Rectangle.Intersect(crop, new Rectangle(0, 0, inputBgr.Width, inputBgr.Height));
//    if (crop.Width <= 0 || crop.Height <= 0) return false;

//    palmRoi = new Mat(inputBgr, crop);
//    mask = new Mat(bw, crop);
//    return true;
//}


///// <summary>
///// Axis-aligned kare ROI
///// </summary>
///// <param name="colorROI"></param>
///// <param name="dtMask"></param>
///// <returns></returns>
///// <exception cref="InvalidOperationException"></exception>
//private Mat CropCentralSquare(Mat colorROI, Mat dtMask)
//{
//    var m = CvInvoke.Moments(dtMask, true);
//    if (m.M00 < 1e-6) throw new InvalidOperationException("dtMask içinde yeterli nokta yok.");
//    PointF center = new PointF((float)(m.M10 / m.M00), (float)(m.M01 / m.M00));
//    float side = Math.Min(colorROI.Width, colorROI.Height) * 0.6f;
//    var rect = new Rectangle(
//        (int)(center.X - side / 2),
//        (int)(center.Y - side / 2),
//        (int)side,
//        (int)side
//    );
//    rect.Intersect(new Rectangle(Point.Empty, colorROI.Size));
//    return new Mat(colorROI, rect);
//}

///// <summary>
///// Rotated kare ROI
///// </summary>
///// <param name="colorROI"></param>
///// <param name="dtMask"></param>
///// <returns></returns>
///// <exception cref="InvalidOperationException"></exception>
//private Mat CropRotatedSquare(Mat colorROI, Mat dtMask)
//{
//    var m = CvInvoke.Moments(dtMask, true);
//    if (m.M00 < 1e-6) throw new InvalidOperationException("dtMask boş");
//    PointF center = new PointF((float)(m.M10 / m.M00), (float)(m.M01 / m.M00));

//    // PCA ile ana eksen
//    var pts = new VectorOfPoint();
//    CvInvoke.FindNonZero(dtMask, pts);
//    var data = new Matrix<float>(pts.Size, 2);
//    for (int i = 0; i < pts.Size; i++)
//    {
//        data[i, 0] = pts[i].X; data[i, 1] = pts[i].Y;
//    }
//    var mean = new Matrix<float>(1, 2);
//    var eigenVecs = new Matrix<float>(2, 2);
//    CvInvoke.PCACompute(data, mean, eigenVecs, 1);
//    float vx = eigenVecs[0, 0], vy = eigenVecs[0, 1];
//    float angle = (float)(Math.Atan2(vy, vx) * 180 / Math.PI);

//    float side = Math.Min(colorROI.Width, colorROI.Height) * 0.6f;
//    var M = new Mat();
//    CvInvoke.GetRotationMatrix2D(center, angle, 1.0, M);
//    var rotated = new Mat();
//    CvInvoke.WarpAffine(colorROI, rotated, M, colorROI.Size);

//    var rect = new Rectangle(
//        (int)(center.X - side / 2),
//        (int)(center.Y - side / 2),
//        (int)side,
//        (int)side
//    );
//    rect.Intersect(new Rectangle(Point.Empty, rotated.Size));
//    return new Mat(rotated, rect);
//}
