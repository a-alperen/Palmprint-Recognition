using Emgu.CV.CvEnum;
using Emgu.CV;
using Palmprint_Recognition.Data;
using Palmprint_Recognition.Extraction;
using Palmprint_Recognition.Recognition;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using System.Collections.Concurrent;

namespace Palmprint_Recognition
{
    internal class EvaluationResult
    {
        public float Threshold { get; set; }
        public float FAR { get; set; }
        public float FRR { get; set; }
        public override string ToString() =>
            $"T={Threshold:F2} → FAR={FAR:F3}, FRR={FRR:F3}";
    }

    internal static class EvaluationService
    {

        public static List<EvaluationResult> Run(
            string datasetPath,
            double trainRatio = 0.5,
            int blockSize = 8,
            float distThreshold = 0.15f,
            float simThreshold = 0.85f)
        {
            // 1) Split
            DataSplitter.SplitRandom(
                baseDir: datasetPath,
                trainRatio: trainRatio,
                out var trainSet,
                out var testSet);

            // 2) Helper’ları hazırla
            var roiExt = new ROIExtractor();
            var featComp = new FeatureExtractor();
            var dbMan = new DatabaseManager("palmdb.csv");
            var rawMan = new RawFeatureManager("rawdb.csv");
            var recog = new Recognizer(dbMan, rawMan, featComp, blockSize);

            // 3a) Her kişi-el için yalnızca ilk dosyayı alacak şekilde enrollItems oluştur
            var enrollItems = trainSet
              .SelectMany(kv => kv.Value.Take(3)
              .Select(path => (Id: kv.Key, Path: path)))
              .AsParallel()
              .WithDegreeOfParallelism(Environment.ProcessorCount)
              .Select(item =>
              {
                  using var img = CvInvoke.Imread(item.Path, ImreadModes.Color);
                  var raw = featComp.ComputeDctFeatures(img, blockSize);
                  var norm = featComp.L2Normalize(raw);
                  return (item.Id, Raw: raw, Norm: norm, Path: item.Path);
              })
              .Where(x => x.Raw != null)
              .ToList();

            // 3b) Tek iş parçacığında enroll etme (dosya I/O çakışmasını önler)
            foreach (var e in enrollItems)
            {
                recog.Enroll(e.Id, e.Raw, e.Norm);
                Debug.WriteLine($"Enrolled: {e.Id} from {e.Path}");
            }

            // 4) Test için ROI + feature extract’ı paralel oluşturma
            var testFeatures = testSet.ToDictionary(
                kv => kv.Key,  // "031_L", "031_R" vb.
                kv => kv.Value
                    .AsParallel()
                    .WithDegreeOfParallelism(Environment.ProcessorCount)
                    .Select(file =>
                    {
                        using var img = CvInvoke.Imread(file, ImreadModes.Color);
                        //if (!roiExt.TryExtract(img, out Mat roi, out _))
                        //    return null;

                        //using var droi = roi;
                        var rawFeat = featComp.ComputeDctFeatures(img, blockSize);
                        var normedFeat = featComp.L2Normalize(rawFeat);
                        return normedFeat;
                    })
                    .Where(feat => feat != null)              // null’ları çıkar
                    .Select(feat => feat!)                    // feat kesin non-null
                    .ToList()
            );

            // 5) FAR/FRR hesapla
            //  thresholds: 0.05 ile 0.50 arası 0.01 adımlı liste
            //var thresholds = Enumerable.Range(5, 16).Select(i => i / 100f).ToList();
            

            // tüm testFeatures’ı (ID, feat) düz bir liste olarak al
            var allTests = testFeatures
                .SelectMany(kv => kv.Value.Select(feat => (TrueId: kv.Key, Feat: feat)))
                .ToList();

            // ——— Genuine mesafe dağılımını ölç ve yazdır ———
            var genuineDists = new List<float>();
            foreach (var (trueId, feat) in allTests)
            {
                // o TrueId'nin enroll edilmiş vektörleri arasından en küçük mesafe
                var minDist = dbMan.Records[trueId]
                    .Min(dbVec => {
                        float sum = 0;
                        for (int i = 0; i < feat.Length; i++)
                            sum += (feat[i] - dbVec[i]) * (feat[i] - dbVec[i]);
                        return (float)Math.Sqrt(sum);
                    });
                genuineDists.Add(minDist);
            }
            float gMin = genuineDists.Min();
            float gMax = genuineDists.Max();
            Console.WriteLine($"Genuine distances → Min={gMin:F4}, Max={gMax:F4}, Mean={genuineDists.Average():F4}");

            // 5b) Eşik listesini genuine dağılımına göre oluştur
            // örn. min’in 1 adım altından, max’in 1 adım üstünden
            int start = Math.Max(1, (int)(gMin * 1000) - 1);
            int end = (int)(gMax * 1000) + 1;
            var thresholds = Enumerable.Range(start, end - start + 1)
                .Select(i => i / 1000f)
                .ToList();

            var resultsBag = new ConcurrentBag<EvaluationResult>();
            Parallel.ForEach(thresholds, thr =>
            {
                int FA = 0, FR = 0;
                int genuineTrials = 0, impostorTrials = 0;

                // tüm test örnekleri için
                Parallel.ForEach(allTests, pair =>
                {
                    var (predId, sim) = recog.Recognize(pair.Feat, thr, simThreshold);
                    bool accepted = predId != null;

                    // her çift mutlaka genuine denemedir
                    Interlocked.Increment(ref genuineTrials);
                    if (pair.TrueId != predId)
                    {
                        // genuine olan bir örnek reddedildiyse
                        Interlocked.Increment(ref FR);
                    }

                    // aynı anda her çift aynı zamanda bir impostor denemesidir
                    // (doğru kabul genuine, yanlış kabul impostor olarak sayılır)
                    Interlocked.Increment(ref impostorTrials);
                    if (accepted && pair.TrueId != predId)
                    {
                        // impostor örnek yanlışlıkla kabul edildiyse
                        Interlocked.Increment(ref FA);
                    }
                });

                float FAR = impostorTrials > 0 ? FA / (float)impostorTrials : 0f;
                float FRR = genuineTrials > 0 ? FR / (float)genuineTrials : 0f;

                resultsBag.Add(new EvaluationResult
                {
                    Threshold = thr,
                    FAR = FAR,
                    FRR = FRR
                });
            });

            // Sonuçları eşik sırasına göre sırala
            var results = resultsBag
                .OrderBy(r => r.Threshold)
                .ToList();

            return results;
        }
    }
}
