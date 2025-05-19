using System.Globalization;

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
                var vec = parts.Skip(1).Select(float.Parse).ToArray();
                _db[id] = vec;
            }
        }

        public void Save(string id, float[] features)
        {
            if (_db.ContainsKey(id))
                throw new InvalidOperationException($"ID '{id}' already exists.");

            // Belleğe ekle
            _db[id] = features;

            // Dosyaya sadece bu kaydı ekle (append: true)
            var line = id + "," +
            string.Join(",",
                features.Select(f =>
                    f.ToString("G6", CultureInfo.InvariantCulture)
                )
            );

            File.AppendAllText(_filePath, line + Environment.NewLine);
        }
    }
}
