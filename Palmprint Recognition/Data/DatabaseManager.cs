using System.Globalization;
using System.Text;

namespace Palmprint_Recognition.Data
{
    internal class DatabaseManager
    {
        private readonly string _filePath;
        private readonly Dictionary<string, float[]> _db = new();

        public IReadOnlyDictionary<string, float[]> Records => _db;

        public DatabaseManager(string filePath)
        {
            _filePath = filePath;
            EnsureFileExists();
        }

        private void EnsureFileExists()
        {
            var dir = Path.GetDirectoryName(_filePath);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                Directory.CreateDirectory(dir);
            if (!File.Exists(_filePath))
                File.WriteAllText(_filePath, string.Empty);
        }

        public void Load()
        {
            _db.Clear();
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
                _db[id] = vec;
            }
        }

        public void Save(string id, float[] features)
        {
            // Belleğe ekle ya da güncelle
            _db[id] = features;

            // Dosyayı baştan yaz: böylece eski raw satırlar kalmaz
            using var sw = new StreamWriter(_filePath, append: false, encoding: Encoding.UTF8);
            foreach (var kv in _db)
            {
                string line = kv.Key + "," +
                              string.Join(",", kv.Value
                                                .Select(f => f.ToString("G6", CultureInfo.InvariantCulture)));
                sw.WriteLine(line);
            }
        }
    }
}
