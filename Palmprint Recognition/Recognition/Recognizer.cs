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

        private readonly int _blockSize;
        public Recognizer(DatabaseManager dbManager, RawFeatureManager rawMgr, FeatureExtractor featExt, int blockSize)
        {
            _dbManager = dbManager;
            _rawFtManager = rawMgr;
            _featExt = featExt;
            _blockSize = blockSize;
        }

        /// <summary>
        /// Enroll a new user:
        ///   1) Extract raw DCT features and save to raw database
        ///   2) L2-normalize features and save to recognition database
        /// </summary>
        public void Enroll(string id, Mat colorROI)
        {
            using var stdROI = new Mat();
            CvInvoke.Resize(colorROI, stdROI, new Size(128, 128), 0, 0, Inter.Linear);

            // 1) Raw DCT
            float[] rawFeats = _featExt.ComputeDctFeatures(stdROI, _blockSize);
            _rawFtManager.SaveNew(id, rawFeats);

            // 2) Normalise and save
            float[] normed = _featExt.L2Normalize((float[])rawFeats.Clone());
            _dbManager.Save(id, normed);
        }

        /// <summary>
        /// En yakın ID’yi ve cosine benzerlik skorunu döner.
        /// </summary>
        /// <param name="colorROI">Giriş ROI</param>
        /// <param name="threshold">Euclidean eşiği (isteğe bağlı, mesela 0.6f)</param>
        /// <returns>(ID ve skoru) eşiğin altındaysa ID, değilse null ve 0</returns>
        public (string? Id, float Similarity) Recognize(Mat colorROI, float distanceThreshold, float similarityThreshold)
        {
            // 1) Standardize ROI size
            using var stdROI = new Mat();
            CvInvoke.Resize(colorROI, stdROI, new Size(128, 128), 0, 0, Inter.Linear);

            // 2) Extract DCT features and L2-normalize
            float[] rawFeats = _featExt.ComputeDctFeatures(stdROI, _blockSize);
            float[] queryVec = _featExt.L2Normalize(rawFeats);

            // 3) Compute Euclidean distances to all database vectors
            var distances = new List<(string Id, float Dist)>();
            foreach (var kv in _dbManager.Records)
            {
                var dbVec = kv.Value;
                float distSq = 0f;
                for (int i = 0; i < queryVec.Length; i++)
                {
                    float d = queryVec[i] - dbVec[i];
                    distSq += d * d;
                }
                float dist = (float)Math.Sqrt(distSq);
                distances.Add((kv.Key, dist));
            }
            // Log all distances for debugging
            distances.Sort((a, b) => a.Dist.CompareTo(b.Dist));
            foreach (var (Id, Dist) in distances)
                Debug.WriteLine($"[Recognize] ID={Id}, Distance={Dist:F6}");

            // 4) Filter out everyone beyond the distanceThreshold,
            //    then take top-3 closest candidates
            var topCandidates = distances
                .Where(x => x.Dist <= distanceThreshold)
                .OrderBy(x => x.Dist)
                .Take(3)
                .ToList();

            // If nobody is within the distance bound, reject immediately
            if (topCandidates.Count == 0)
                return (null, 0f);
            
            // 5) For each of the top candidates compute cosine similarity,
            //    and accept the first one above similarityThreshold
            foreach (var candidate in topCandidates)
            {
                float[] dbVec = _dbManager.Records[candidate.Id];
                float dot = 0f;
                for (int i = 0; i < queryVec.Length; i++)
                    dot += queryVec[i] * dbVec[i];

                // dot is in [0,1] after L2-normalization,
                // multiply by 100 to get percentage
                float similarity = dot * 100f;
                Debug.WriteLine($"[Recognize] Candidate={candidate.Id}, EuclidDist={candidate.Dist:F4}, CosineSim={similarity:F2}%");

                if (similarity >= similarityThreshold)
                    return (candidate.Id, similarity);
            }

            // 6) None of the top candidates passed the cosine check → reject
            return (null, 0f);
        }


    }
}
