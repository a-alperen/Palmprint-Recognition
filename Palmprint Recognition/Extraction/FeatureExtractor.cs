using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;

namespace Palmprint_Recognition.Extraction
{
    internal class FeatureExtractor
    {
        public float[] ComputeDctFeatures(Mat roi, int blockSize)
        {
            var gray = new Mat();
            CvInvoke.CvtColor(roi, gray, ColorConversion.Bgr2Gray);
            var fgray = new Mat();
            gray.ConvertTo(fgray, DepthType.Cv32F);

            var dctMat = new Matrix<float>(fgray.Rows, fgray.Cols);
            CvInvoke.Dct(fgray, dctMat, DctType.Forward);

            int N = blockSize;
            var feats = new float[N * N];
            int idx = 0;
            for (int i = 0; i < N; i++)
                for (int j = 0; j < N; j++)
                    feats[idx++] = dctMat[i, j];

            var norm = (float)Math.Sqrt(feats.Sum(x => x * x));
            if (norm > 1e-6f) for (int k = 0; k < feats.Length; k++) feats[k] /= norm;
            return feats;
        }

        public void ComputeGlobalMinMax(IList<float[]> allFeatures, out float[] globalMin, out float[] globalMax)
        {
            if (allFeatures == null || allFeatures.Count == 0)
                throw new ArgumentException("Özellik listesi boş olamaz.");

            int D = allFeatures[0].Length;
            globalMin = Enumerable.Repeat(float.PositiveInfinity, D).ToArray();
            globalMax = Enumerable.Repeat(float.NegativeInfinity, D).ToArray();
            foreach (var f in allFeatures)
            {
                if (f.Length != D)
                    throw new InvalidOperationException("Tüm vektörler aynı uzunlukta olmalı.");
                for (int j = 0; j < D; j++)
                {
                    if (f[j] < globalMin[j]) globalMin[j] = f[j];
                    if (f[j] > globalMax[j]) globalMax[j] = f[j];
                }
            }
        }

        public float[] MinMaxNormalize(float[] feats, float[] minVals, float[] maxVals)
        {
            int D = feats.Length;
            var outF = new float[D];
            for (int j = 0; j < D; j++)
            {
                var range = maxVals[j] - minVals[j];
                outF[j] = range > 1e-6f
                    ? (feats[j] - minVals[j]) / range
                    : 0f;
            }
            return outF;
        }

        public int[] SelectTopIndices(float[][] allNorm, int[] labels, int selectCount)
        {
            int N = allNorm.Length;
            int D = allNorm[0].Length;
            var classes = labels.Distinct().ToArray();

            // Global ortalama
            double[] globalMean = new double[D];
            for (int i = 0; i < N; i++)
                for (int j = 0; j < D; j++)
                    globalMean[j] += allNorm[i][j];
            for (int j = 0; j < D; j++)
                globalMean[j] /= N;

            // Sınıf bazlı ortalama ve varyans
            var classMeans = new Dictionary<int, double[]>();
            var classVars = new Dictionary<int, double[]>();
            foreach (int c in classes)
            {
                var members = allNorm.Where((f, i) => labels[i] == c).ToArray();
                var mu = new double[D];
                var varc = new double[D];
                for (int j = 0; j < D; j++) mu[j] = members.Average(f => f[j]);
                for (int j = 0; j < D; j++) varc[j] = members.Average(f => (f[j] - mu[j]) * (f[j] - mu[j]));
                classMeans[c] = mu;
                classVars[c] = varc;
            }

            // Discriminant power hesapla
            double[] dp = new double[D];
            for (int j = 0; j < D; j++)
            {
                double between = 0, within = 0;
                foreach (int c in classes)
                {
                    between += Math.Pow(classMeans[c][j] - globalMean[j], 2);
                    within += classVars[c][j];
                }
                dp[j] = within > 1e-6 ? between / within : 0.0;
            }

            // En yüksek skorlu M indeksi seç
            return Enumerable.Range(0, D)
                .Select(i => (Score: dp[i], Index: i))
                .OrderByDescending(pair => pair.Score)
                .Take(selectCount)
                .Select(pair => pair.Index)
                .ToArray();
        }


        public float[] SelectFeaturesByIndex(float[] feats, int[] idx)
            => idx.Select(i => feats[i]).ToArray();

        public float[] L2Normalize(float[] vec)
        {
            var norm = (float)Math.Sqrt(vec.Sum(v => v * v));
            return norm > 1e-6f ? vec.Select(v => v / norm).ToArray() : vec;
        }
    }
}
