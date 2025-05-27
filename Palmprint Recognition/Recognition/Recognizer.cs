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
        /// Enrolls a new palmprint by saving its raw and normalized features.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="rawFeats"></param>
        /// <param name="normFeats"></param>
        public void Enroll(string id, float[] rawFeats, float[] normFeats)
        {
            // Save database "palmdb.csv" for normalized features and "rawdb.csv" for raw features
            _rawFtManager.SaveNew(id, rawFeats);
            _dbManager.Save(id, normFeats);
        }

        /// <summary>
        /// Recognizes a palmprint by comparing its features against the database with euclidean distance and cosine similarity.
        /// </summary>
        /// <param name="feats"></param>
        /// <param name="distanceThreshold"></param>
        /// <param name="similarityThreshold"></param>
        /// <returns></returns>
        //public (string? Id, float Similarity) Recognize1(float[] feats, float distanceThreshold, float similarityThreshold)
        //{

        //    // Compute Euclidean distances to all database vectors
        //    var distances = new List<(string Id, float Dist)>();
        //    foreach (var kv in _dbManager.Records)
        //    {
        //        var dbVec = kv.Value;
        //        float distSq = 0f;
        //        for (int i = 0; i < feats.Length; i++)
        //        {
        //            float d = feats[i] - dbVec[i];
        //            distSq += d * d;
        //        }
        //        float dist = (float)Math.Sqrt(distSq);
        //        distances.Add((kv.Key, dist));
        //    }
        //    // Log all distances for debugging
        //    //distances.Sort((a, b) => a.Dist.CompareTo(b.Dist));
        //    //foreach (var (Id, Dist) in distances)
        //    //    Debug.WriteLine($"[Recognize] ID={Id}, Distance={Dist:F6}");

        //    // 4) Filter out everyone beyond the distanceThreshold,
        //    //    then take top-3 closest candidates
        //    var topCandidates = distances
        //        .Where(x => x.Dist <= distanceThreshold)
        //        .OrderBy(x => x.Dist)
        //        .Take(3)
        //        .ToList();

        //    // If nobody is within the distance bound, reject immediately
        //    if (topCandidates.Count == 0)
        //        return (null, 0f);

        //    // 5) For each of the top candidates compute cosine similarity,
        //    //    and accept the first one above similarityThreshold
        //    foreach (var candidate in topCandidates)
        //    {
        //        float[] dbVec = _dbManager.Records[candidate.Id];
        //        float dot = 0f;
        //        for (int i = 0; i < feats.Length; i++)
        //            dot += feats[i] * dbVec[i];

        //        // dot is in [0,1] after L2-normalization,
        //        // multiply by 100 to get percentage
        //        float similarity = dot * 100f;
        //        //Debug.WriteLine($"[Recognize] Candidate={candidate.Id}, EuclidDist={candidate.Dist:F4}, CosineSim={similarity:F2}%");

        //        if (similarity >= similarityThreshold)
        //            return (candidate.Id, similarity);
        //    }

        //    // 6) None of the top candidates passed the cosine check → reject
        //    return (null, 0f);
        //}
        public (string? Id, float Similarity) Recognize(
        float[] feat,
        float distThreshold,
        float simThreshold)
        {
            // 1) Tüm kayıtlı vektörler için (ID, vec, euclidDist) listesi oluştur
            var dists = new List<(string Id, float[] Vec, float Dist)>();
            foreach (var kv in _dbManager.Records)
            {
                string id = kv.Key;
                foreach (var dbVec in kv.Value)
                {
                    // Öklid mesafesi
                    float sum = 0f;
                    for (int i = 0; i < feat.Length; i++)
                    {
                        float d = feat[i] - dbVec[i];
                        sum += d * d;
                    }
                    float dist = (float)Math.Sqrt(sum);
                    dists.Add((id, dbVec, dist));
                }
            }

            // Log all distances for debugging
            dists.Sort((a, b) => a.Dist.CompareTo(b.Dist));
            foreach (var (Id, Vec, Dist) in dists)
                Debug.WriteLine($"[Recognize] ID={Id}, Distance={Dist:F6}");

            // 2) Mesafe eşiğine takılanların içinden en yakın 3’ünü seç
            var top3 = dists
                .Where(x => x.Dist <= distThreshold)
                .OrderBy(x => x.Dist)
                .Take(3)
                .ToList();
            //if (top3.Count < 3)
            //    return (null, 0f);

            // 3) Oylama: en çok tekrar eden ID’yi bul
            var winnerGroup = top3
                .GroupBy(x => x.Id)
                .OrderByDescending(g => g.Count())
                .First();
            string winnerId = winnerGroup.Key;

            if (!winnerGroup.Any())
                return (null, 0f);

            // 4) Kosinüs benzerliğini hesapla (feat ve dbVec’ler L2-normalize edildiği varsayılarak)
            float bestCos = 0f;
            foreach (var candidate in top3.Where(x => x.Id == winnerId))
            {
                float dot = 0f;
                for (int i = 0; i < feat.Length; i++)
                    dot += feat[i] * candidate.Vec[i];
                bestCos = Math.Max(bestCos, dot * 100f);  // % cinsine çevirmek için 100×
            }

            // 5) Benzerlik eşiğini kontrol et
            if (bestCos < simThreshold)
                return (null, 0f);

            // Kabul ediliyor
            return (winnerId, bestCos);
        }

        public (string? Id, float Similarity) Recognize1(
        float[] feats,
        float distanceThreshold,
        float similarityThreshold)
        {
            // 1) Tüm kayıtlı vektörler için (ID, vektör, mesafe) üçlüsü oluştur
            var distances = new List<(string Id, float[] Vec, float Dist)>();
            foreach (var kv in _dbManager.Records)
            {
                string id = kv.Key;
                foreach (var dbVec in kv.Value)
                {
                    float distSq = 0f;
                    for (int i = 0; i < feats.Length; i++)
                    {
                        float d = feats[i] - dbVec[i];
                        distSq += d * d;
                    }
                    float dist = (float)Math.Sqrt(distSq);
                    distances.Add((id, dbVec, dist));
                }
            }

            // 2) Eşiği aşmayanların en yakın 3 tanesini seç
            var topCandidates = distances
                .Where(x => x.Dist <= distanceThreshold)
                .OrderBy(x => x.Dist)
                .Take(3)
                .ToList();

            if (topCandidates.Count == 0)
                return (null, 0f);

            // 3) Ön elemeden geçen her aday vektör ile kosinüs benzerliğini hesapla
            foreach (var candidate in topCandidates)
            {
                float dot = 0f;
                for (int i = 0; i < feats.Length; i++)
                    dot += feats[i] * candidate.Vec[i];

                float similarity = dot * 100f; // % cinsinden

                if (similarity >= similarityThreshold)
                    return (candidate.Id, similarity);
            }

            // 4) Hiçbiri eşik geçemedi → reddet
            return (null, 0f);
        }


    }
}
