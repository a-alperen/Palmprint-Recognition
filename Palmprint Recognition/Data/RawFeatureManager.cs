using System.Globalization;

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
                // ilk eleman: ID; gerisi raw DCT feature
                var feats = parts.Skip(1).Select(float.Parse).ToArray();
                _raw.Add(feats);
            }
        }

        public void SaveNew(string id, float[] feats)
        {
            // Listeye ekle (isteğe bağlı)
            _raw.Add(feats);

            // Dosyaya tek satır olarak ekle
            var line = id + "," +
                string.Join(",",
                    feats.Select(f =>
                        f.ToString("G6", CultureInfo.InvariantCulture)
                    )
                );
            File.AppendAllText(_filePath, line + Environment.NewLine);
        }

        public IReadOnlyList<float[]> AllRaw => _raw;
    }
}
