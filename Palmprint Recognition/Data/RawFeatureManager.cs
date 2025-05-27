using System.Globalization;
using System.Text;

namespace Palmprint_Recognition.Data
{
    using System;
    using System.Collections.Generic;
    using System.Globalization;
    using System.IO;
    using System.Linq;
    using System.Text;

    internal class RawFeatureManager
    {
        private readonly string _filePath;
        private readonly Dictionary<string, List<float[]>> _raw = new();

        public IReadOnlyDictionary<string, List<float[]>> Records => _raw;

        public RawFeatureManager(string filePath)
        {
            _filePath = filePath;
            EnsureExists();
            Load();
        }

        private void EnsureExists()
        {
            var dir = Path.GetDirectoryName(_filePath);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                Directory.CreateDirectory(dir);
            if (!File.Exists(_filePath))
                File.WriteAllText(_filePath, string.Empty, Encoding.UTF8);
        }

        private void Load()
        {
            _raw.Clear();
            foreach (var line in File.ReadLines(_filePath))
            {
                if (string.IsNullOrWhiteSpace(line)) continue;
                var parts = line.Split(',');
                var id = parts[0];
                var vec = parts
                    .Skip(1)
                    .Select(tok => float.Parse(tok, CultureInfo.InvariantCulture))
                    .ToArray();

                if (!_raw.ContainsKey(id))
                    _raw[id] = new List<float[]>();
                _raw[id].Add(vec);
            }
        }

        /// <summary>
        /// Yeni bir raw vektörü kayıt eder. 
        /// Aynı ID için belleğe ekler ve dosyaya append yazar.
        /// </summary>
        public void SaveNew(string id, float[] feats)
        {
            // 1) Belleğe ekle
            if (!_raw.ContainsKey(id))
                _raw[id] = new List<float[]>();
            _raw[id].Add(feats);

            // 2) Dosyaya append et
            using var sw = new StreamWriter(_filePath, append: true, encoding: Encoding.UTF8);
            var line = id + "," +
                       string.Join(",", feats
                         .Select(f => f.ToString("G6", CultureInfo.InvariantCulture)));
            sw.WriteLine(line);
        }
    }

}
