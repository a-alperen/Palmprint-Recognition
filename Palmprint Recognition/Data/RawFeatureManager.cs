using System.Globalization;
using System.Text;

namespace Palmprint_Recognition.Data
{
    internal class RawFeatureManager
    {
        private readonly string _filePath;
        private readonly List<float[]> _raw = new();

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
                File.WriteAllText(_filePath, "");
        }

        private void Load()
        {
            _raw.Clear();
            foreach (var line in File.ReadLines(_filePath))
            {
                if (string.IsNullOrWhiteSpace(line)) continue;
                var parts = line.Split(',');
                var id = parts[0];
                // Ondalık ayırıcı olarak nokta kullanan InvariantCulture ile parse ediyoruz
                var vec = parts
                    .Skip(1)
                    .Select(tok => float.Parse(tok, CultureInfo.InvariantCulture))
                    .ToArray();
                _raw.Add(vec);
            }
        }

        public void SaveNew(string id, float[] feats)
        {
            // Listeye ekle (isteğe bağlı)
            _raw.Add(feats);

            // Dosyayı baştan yaz: böylece eski raw satırlar kalmaz
            using var sw = new StreamWriter(_filePath, append: false, encoding: Encoding.UTF8);
            foreach (var kv in _raw)
            {
                string line = id + "," +
                              string.Join(",", kv.Select(f => f.ToString("G6", CultureInfo.InvariantCulture)));
                sw.WriteLine(line);
            }
        }

        public IReadOnlyList<float[]> AllRaw => _raw;
    }
}
