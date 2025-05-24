using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;

namespace Palmprint_Recognition.Extraction
{
    internal class FeatureExtractor
    {
        /// <summary>
        /// Compute block-size first DCT coefficients, then L2-normalize.
        /// ROI must be at least blockSize×blockSize.
        /// </summary>
        public float[] ComputeDctFeatures(Mat roi, int blockSize)
        {
            // 1) Convert to grayscale and float matrix
            using var gray = new Mat();
            CvInvoke.CvtColor(roi, gray, ColorConversion.Bgr2Gray);
            // Convert Mat to Matrix<float>
            using var floatMat = new Matrix<float>(gray.Rows, gray.Cols);
            gray.ConvertTo(floatMat, DepthType.Cv32F);

            // 2) Forward DCT into a new Matrix<float>
            using var dctMat = new Matrix<float>(gray.Rows, gray.Cols);
            CvInvoke.Dct(floatMat, dctMat, DctType.Forward);

            // 3) Extract top-left blockSize×blockSize coefficients (raw)
            if (dctMat.Rows < blockSize || dctMat.Cols < blockSize)
                throw new ArgumentException("ROI smaller than blockSize");
            int N = blockSize;
            var feats = new float[N * N];
            int idx = 0;
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < N; j++)
                {
                    feats[idx++] = dctMat[i, j];
                }
            }
            return feats;
        }
        /// <summary>
        /// Standard L2 normalize a vector to unit length.
        /// </summary>
        public float[] L2Normalize(float[] vec)
        {
            float sumSq = 0f;
            for (int i = 0; i < vec.Length; i++)
                sumSq += vec[i] * vec[i];

            float norm = (float)Math.Sqrt(sumSq);
            if (norm > 1e-6f)
            {
                for (int i = 0; i < vec.Length; i++)
                    vec[i] /= norm;
            }
            return vec;
        }
        
        ///// <summary>
        ///// Select top features by discriminant power (between-class / within-class).
        ///// </summary>
        //public int[] SelectTopIndices(float[][] allFeatures, int[] labels, int selectCount)
        //{
        //    int N = allFeatures.Length;
        //    int D = allFeatures[0].Length;
        //    var classes = labels.Distinct().ToArray();

        //    // Compute global mean
        //    double[] globalMean = new double[D];
        //    for (int i = 0; i < N; i++)
        //        for (int j = 0; j < D; j++)
        //            globalMean[j] += allFeatures[i][j];
        //    for (int j = 0; j < D; j++)
        //        globalMean[j] /= N;

        //    // Class statistics
        //    var classMeans = new Dictionary<int, double[]>();
        //    var classVars = new Dictionary<int, double[]>();
        //    foreach (int c in classes)
        //    {
        //        var members = allFeatures.Where((_, i) => labels[i] == c).ToArray();
        //        var mu = new double[D];
        //        var varc = new double[D];
        //        int M = members.Length;
        //        for (int j = 0; j < D; j++)
        //        {
        //            double sum = 0, sum2 = 0;
        //            foreach (var vec in members)
        //            {
        //                double v = vec[j]; sum += v; sum2 += v * v;
        //            }
        //            mu[j] = sum / M;
        //            varc[j] = sum2 / M - mu[j] * mu[j];
        //        }
        //        classMeans[c] = mu;
        //        classVars[c] = varc;
        //    }

        //    // Discriminant power
        //    double[] dp = new double[D];
        //    for (int j = 0; j < D; j++)
        //    {
        //        double between = 0, within = 0;
        //        foreach (int c in classes)
        //        {
        //            between += Math.Pow(classMeans[c][j] - globalMean[j], 2);
        //            within += classVars[c][j];
        //        }
        //        dp[j] = within > 1e-6 ? between / within : 0.0;
        //    }

        //    // Top indices
        //    return Enumerable.Range(0, D)
        //                     .OrderByDescending(i => dp[i])
        //                     .Take(selectCount)
        //                     .ToArray();
        //}


        ///// <summary>
        ///// Pick features by index list.
        ///// </summary>
        //public float[] SelectFeaturesByIndex(float[] feats, int[] idx)
        //    => idx.Select(i => feats[i]).ToArray();

        ///// <summary>
        ///// Compute per-dimension global min and max over all feature vectors in one pass.
        ///// </summary>
        //public void ComputeGlobalMinMax(IList<float[]> allFeatures, out float[] globalMin, out float[] globalMax)
        //{
        //    if (allFeatures == null || allFeatures.Count == 0)
        //        throw new ArgumentException("Feature list cannot be empty.");

        //    int D = allFeatures[0].Length;
        //    globalMin = new float[D];
        //    globalMax = new float[D];
        //    for (int j = 0; j < D; j++)
        //    {
        //        globalMin[j] = float.MaxValue;
        //        globalMax[j] = float.MinValue;
        //    }

        //    foreach (var f in allFeatures)
        //    {
        //        if (f.Length != D)
        //            throw new InvalidOperationException("All feature vectors must have same length.");
        //        for (int j = 0; j < D; j++)
        //        {
        //            float v = f[j];
        //            if (v < globalMin[j]) globalMin[j] = v;
        //            if (v > globalMax[j]) globalMax[j] = v;
        //        }
        //    }
        //}

        ///// <summary>
        ///// Min-max normalize a single feature vector given global min/max.
        ///// </summary>
        //public float[] MinMaxNormalize(float[] feats, float[] minVals, float[] maxVals)
        //{
        //    int D = feats.Length;
        //    var outF = new float[D];
        //    for (int j = 0; j < D; j++)
        //    {
        //        float range = maxVals[j] - minVals[j];
        //        outF[j] = range > 1e-6f
        //            ? (feats[j] - minVals[j]) / range
        //            : 0f;
        //    }
        //    return outF;
        //}
    }
}
