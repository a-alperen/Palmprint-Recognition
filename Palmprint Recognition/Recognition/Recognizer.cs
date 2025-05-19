using Emgu.CV;
using Emgu.CV.CvEnum;
using Palmprint_Recognition.Data;
using Palmprint_Recognition.Extraction;
using System.Diagnostics;

namespace Palmprint_Recognition.Recognition
{
    internal class Recognizer
    {
        private readonly DatabaseManager _dbManager;
        private readonly RawFeatureManager _rawFtManager;
        private readonly FeatureExtractor _featExt;
        private readonly int _M;
        private readonly List<float[]> _allDctFeatures = new();
        private float[] _globalMin;
        private float[] _globalMax;
        private int[] _topIdx;

        public Recognizer(DatabaseManager dbManager,RawFeatureManager rawMgr, FeatureExtractor featExt, int M = 30)
        {
            _dbManager = dbManager;
            _rawFtManager = rawMgr;
            _featExt = featExt;
            _M = M;
        }

        public void Initialize(IEnumerable<float[]> rawFeatures)
        {
            _allDctFeatures.Clear();
            _allDctFeatures.AddRange(rawFeatures);
            if (_allDctFeatures.Count > 0)
                _featExt.ComputeGlobalMinMax(_allDctFeatures, out _globalMin, out _globalMax);
            if (_allDctFeatures.Count >= _M)
            {
                var norm = _allDctFeatures.Select(f => _featExt.MinMaxNormalize(f, _globalMin, _globalMax)).ToArray();
                _topIdx = _featExt.SelectTopIndices(norm, Enumerable.Range(0, norm.Length).ToArray(), _M);
            }
        }

        public void Enroll(string id, Mat colorROI)
        {
            var stdROI = new Mat();
            CvInvoke.Resize(colorROI, stdROI, new Size(128, 128), 0, 0, Inter.Linear);
            float[] feats = _featExt.ComputeDctFeatures(stdROI, 8);

            _allDctFeatures.Add(feats);
            _rawFtManager.SaveNew(id, feats);

            if (_allDctFeatures.Count == 1)
            {
                // Global min/max’i yine feats üzerinden ayarla, topIdx’i de basitçe ilk M indekse set et
                _globalMin = (float[])feats.Clone();
                _globalMax = (float[])feats.Clone();
                _topIdx = Enumerable.Range(0, feats.Length)
                                     .Take(_M)
                                     .ToArray();

                // finalVec = orijinal DCT özelliklerinin L2 normalize hali
                float[] finalVec1 = _featExt.L2Normalize(feats);
                _dbManager.Save(id, finalVec1);
                return;
            }
            else
            {
                _featExt.ComputeGlobalMinMax(_allDctFeatures, out _globalMin, out _globalMax);
            }
            var normG = _featExt.MinMaxNormalize(feats, _globalMin, _globalMax);
            _topIdx = _featExt.SelectTopIndices(_allDctFeatures.Select(f => _featExt.MinMaxNormalize(f, _globalMin, _globalMax)).ToArray(), Enumerable.Range(0, _allDctFeatures.Count).ToArray(), _M);
            var reduced = _featExt.SelectFeaturesByIndex(normG, _topIdx);
            var finalVec = _featExt.L2Normalize(reduced);
            Debug.WriteLine(finalVec);
            _dbManager.Save(id, finalVec);
        }

        public string? Recognize(Mat colorROI, float threshold)
        {
            var stdROI = new Mat();
            CvInvoke.Resize(colorROI, stdROI, new Size(128, 128), 0, 0, Inter.Linear);
            var feats = _featExt.ComputeDctFeatures(stdROI, 8);
            var normG = _featExt.MinMaxNormalize(feats, _globalMin, _globalMax);
            var reduced = _featExt.SelectFeaturesByIndex(normG, _topIdx);
            if (reduced == null || reduced.Length == 0) return null;
            var minv = reduced.Min(); var maxv = reduced.Max(); var range = maxv - minv;
            var normS = reduced.Select(v => range > 1e-6f ? (v - minv) / range : 0f).ToArray();
            var queryVec = _featExt.L2Normalize(normS);
            string bestId = null; float bestDist = float.MaxValue;
            foreach (var kv in _dbManager.Records)
            {
                var dbVec = kv.Value; float dist = 0;
                for (int i = 0; i < queryVec.Length; i++) dist += (queryVec[i] - dbVec[i]) * (queryVec[i] - dbVec[i]);
                dist = (float)Math.Sqrt(dist);
                if (dist < bestDist) { bestDist = dist; bestId = kv.Key; }
            }
            return (bestId != null && bestDist <= threshold) ? bestId : null;
        }

    }
}
