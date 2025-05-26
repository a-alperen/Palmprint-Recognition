using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Palmprint_Recognition.Data
{
    internal static class DataSplitter
    {
        /// <summary>
        /// Her kişi klasöründe:
        /// - Dosya adı: 031_F_L_1.png gibi → parts[2] = "L" veya "R"
        /// - session1Count: session 1'deki örnek sayısı (6)
        /// 
        /// Çıktı:
        ///   trainSet: key = "031_L" veya "031_R"
        ///   testSet:  aynı key’ler, kalan görüntüler
        /// </summary>
        public static void SplitByHandSession(
            string baseDir,
            int session1Count,
            out Dictionary<string, List<string>> trainSet,
            out Dictionary<string, List<string>> testSet)
        {
            trainSet = new Dictionary<string, List<string>>();
            testSet = new Dictionary<string, List<string>>();

            foreach (var personDir in Directory.GetDirectories(baseDir))
            {
                string personId = Path.GetFileName(personDir);

                // Tüm resimleri topla
                var files = Directory.GetFiles(personDir)
                    .Where(f => f.EndsWith(".png", StringComparison.OrdinalIgnoreCase)
                             || f.EndsWith(".bmp", StringComparison.OrdinalIgnoreCase)
                             || f.EndsWith(".jpg", StringComparison.OrdinalIgnoreCase))
                    .ToList();

                // El bazlı grupla: key = "031_L" veya "031_R"
                var handGroups = files
                    .GroupBy(f =>
                    {
                        var nameParts = Path.GetFileNameWithoutExtension(f).Split('_');
                        var hand = nameParts[2];            // örn: "L" veya "R"
                        return $"{personId}_{hand}";
                    });

                foreach (var grp in handGroups)
                {
                    // Gruplardaki dosyaları index’e göre sırala:
                    // son parça sayı → int index
                    var sorted = grp.OrderBy(f =>
                    {
                        var parts = Path.GetFileNameWithoutExtension(f).Split('_');
                        return int.Parse(parts.Last());
                    }).ToList();

                    // İlk session1Count train, kalan test
                    trainSet[grp.Key] = sorted.Take(session1Count).ToList();
                    testSet[grp.Key] = sorted.Skip(session1Count).ToList();
                }
            }
        }
        /// <summary>
        /// Rastgele train/test split:
        /// - baseDir: kişi klasörlerinin ana dizini
        /// - trainRatio: örn. 0.5 => %50 train, %50 test
        /// Çıktı key'leri: "031_L", "031_R" vb.
        /// </summary>
        public static void SplitRandom(
            string baseDir,
            double trainRatio,
            out Dictionary<string, List<string>> trainSet,
            out Dictionary<string, List<string>> testSet)
        {
            var rnd = new Random();
            trainSet = new Dictionary<string, List<string>>();
            testSet = new Dictionary<string, List<string>>();

            foreach (var personDir in Directory.GetDirectories(baseDir))
            {
                string personId = Path.GetFileName(personDir);

                // 1) El bazlı grupla
                var handGroups = Directory.GetFiles(personDir)
                    .Where(f => f.EndsWith(".png", StringComparison.OrdinalIgnoreCase)
                             || f.EndsWith(".bmp", StringComparison.OrdinalIgnoreCase)
                             || f.EndsWith(".jpg", StringComparison.OrdinalIgnoreCase))
                    .GroupBy(f =>
                    {
                        var parts = Path.GetFileNameWithoutExtension(f).Split('_');
                        var hand = parts[2];  // "L" veya "R"
                        return $"{personId}_{hand}";
                    });

                // 2) Her grup için shuffle + split
                foreach (var grp in handGroups)
                {
                    var all = grp.ToList()
                                 .OrderBy(_ => rnd.Next())  // karıştır
                                 .ToList();

                    int splitPoint = (int)(all.Count * trainRatio);
                    trainSet[grp.Key] = all.Take(splitPoint).ToList();
                    testSet[grp.Key] = all.Skip(splitPoint).ToList();
                }
            }
        }

    }
}
