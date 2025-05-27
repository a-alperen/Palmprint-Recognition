using System.Globalization;
using System.Text;

namespace Palmprint_Recognition.Data
{
    internal class DatabaseManager
    {
        private readonly string _filePath;
        private readonly Dictionary<string, List<float[]>> _db = new();

        // Artık her ID'nin birden çok feature listesi var
        public IReadOnlyDictionary<string, List<float[]>> Records => _db;

        public DatabaseManager(string filePath)
        {
            _filePath = filePath;
            EnsureFileExists();
            Load();
        }

        private void EnsureFileExists()
        {
            var dir = Path.GetDirectoryName(_filePath);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                Directory.CreateDirectory(dir);
            if (!File.Exists(_filePath))
                File.WriteAllText(_filePath, string.Empty, Encoding.UTF8);
        }

        public void Load()
        {
            _db.Clear();
            foreach (var line in File.ReadLines(_filePath))
            {
                if (string.IsNullOrWhiteSpace(line)) continue;
                var parts = line.Split(',');
                var id = parts[0];
                var vec = parts
                    .Skip(1)
                    .Select(tok => float.Parse(tok, CultureInfo.InvariantCulture))
                    .ToArray();

                if (!_db.ContainsKey(id))
                    _db[id] = new List<float[]>();
                _db[id].Add(vec);
            }
        }

        /// <summary>
        /// Yeni bir raw veya normalize edilmiş vektörü kayıt eder.
        /// Aynı ID için birden çok satır dosyaya append edilir.
        /// Bellekte de listeye eklenir.
        /// </summary>
        public void Save(string id, float[] features)
        {
            // 1) Belleğe ekle
            if (!_db.ContainsKey(id))
                _db[id] = new List<float[]>();
            _db[id].Add(features);

            // 2) Dosyaya append et
            using var sw = new StreamWriter(_filePath, append: true, encoding: Encoding.UTF8);
            var line = id + "," +
                       string.Join(",", features
                           .Select(f => f.ToString("G6", CultureInfo.InvariantCulture)));
            sw.WriteLine(line);
        }
    }
}
