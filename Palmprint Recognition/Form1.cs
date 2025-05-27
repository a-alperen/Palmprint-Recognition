using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Diagnostics;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Util;
using Emgu.CV.Structure;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Window;
using System.Xml.Linq;
using System.Threading.Tasks;
using Palmprint_Recognition.Data;
using Palmprint_Recognition.Extraction;
using Palmprint_Recognition.Recognition;
using Emgu.CV.ML;

namespace Palmprint_Recognition
{
    public partial class Form1 : Form
    {
        private readonly DatabaseManager _db;
        private readonly RawFeatureManager _rawFtManager;
        private readonly ROIExtractor _roiExt;
        private readonly FeatureExtractor _featExt;
        private readonly Recognizer _recognizer;

        private Mat? _inputMat;
        private Mat? _loginMat;
        private Mat? _roi;
        private Mat? _roiLogin;

        private readonly int _blockSize = 8;
        private readonly float _distanceThreshold = 0.15f;
        private readonly float _similarityThreshold = 0.85f;
        private string _datasetPath = string.Empty;
        public Form1()
        {
            InitializeComponent();
            _db = new DatabaseManager("palmdb.csv");
            _rawFtManager = new RawFeatureManager("rawdb.csv");
            _roiExt = new ROIExtractor();
            _featExt = new FeatureExtractor();
            _recognizer = new Recognizer(_db, _rawFtManager, _featExt, _blockSize);

        }
        private void Form1_Load(object sender, EventArgs e)
        {
            _db.Load();
            RefreshIdList();
        }

        /// <summary>
        /// Upload image from file for enrollment
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button1_Click(object sender, EventArgs e)
        {
            using var dlg = new OpenFileDialog { Filter = "Image Files|*.jpg;*.png;*.bmp" };
            if (dlg.ShowDialog() != DialogResult.OK) return;

            _inputMat?.Dispose();
            pictureBox1.Image = null;
            _roi?.Dispose();
            RegisterROIPictureBox.Image = null;

            using var tmp = CvInvoke.Imread(dlg.FileName, ImreadModes.Color);
            _inputMat = tmp.Clone();
            pictureBox1.Image = _inputMat.ToBitmap();

            if (!_roiExt.TryExtract(_inputMat, out _roi, out _)) { MessageBox.Show("ROI alınamadı."); return; }

            RegisterROIPictureBox.Image = _roi.ToBitmap();


        }
        /// <summary>
        /// Upload image from file for recognition
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void LoginImageButton_Click(object sender, EventArgs e)
        {
            using var dlg = new OpenFileDialog { Filter = "Image Files|*.jpg;*.png;*.bmp" };
            if (dlg.ShowDialog() != DialogResult.OK) return;

            _loginMat?.Dispose();
            pictureBox2.Image = null;
            _roiLogin?.Dispose();
            LoginROIPictureBox.Image = null;

            using var tmp = CvInvoke.Imread(dlg.FileName, ImreadModes.Color);
            _loginMat = tmp.Clone();
            pictureBox2.Image = _loginMat.ToBitmap();

            if (!_roiExt.TryExtract(_loginMat, out _roiLogin, out _)) { MessageBox.Show("ROI alınamadı."); return; }

            LoginROIPictureBox.Image = _roiLogin.ToBitmap();

        }
        /// <summary>
        /// Recognition button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button2_Click(object sender, EventArgs e)
        {
            if (_roiLogin == null || _loginMat == null) { MessageBox.Show("Önce giriş görüntüsü yükleyin."); return; }
            var raw = _featExt.ComputeHybridFeatures(_roiLogin, _blockSize);
            var feat = _featExt.L2Normalize(raw);
            var (id, percentage) = _recognizer.Recognize(feat, _distanceThreshold, _similarityThreshold);
            if (id == null)
            {
                MessageBox.Show("Kayıt bulunamadı.");
            }
            else
            {
                MessageBox.Show($"Hoş geldiniz, {id}!");
                RecognitionLabel.Text = $"{id} ile benzerlik yüzdesi %{percentage}";
            }

        }
        /// <summary>
        /// Enroll
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void enrollButton_Click(object sender, EventArgs e)
        {
            if (_inputMat == null) { MessageBox.Show("Önce görüntü yükleyin."); return; }
            var id = userIDTextBox.Text.Trim();
            if (string.IsNullOrEmpty(id)) { MessageBox.Show("ID girin."); return; }
            if (_roi == null) return;

            var raw = _featExt.ComputeHybridFeatures(_roi, _blockSize);
            var feat = _featExt.L2Normalize(raw);

            try { _recognizer.Enroll(id, raw, feat); MessageBox.Show($"'{id}' başarıyla kaydedildi."); RefreshIdList(); }
            catch (Emgu.CV.Util.CvException cvEx) { MessageBox.Show($"OpenCV hatası: {cvEx.Message}"); }
            catch (Exception ex) { MessageBox.Show(ex.Message); }
        }
        private void EvaluateButton_Click(object sender, EventArgs e)
        {
            using var fbd = new FolderBrowserDialog();

            if (fbd.ShowDialog() == DialogResult.OK)
            {
                // Seçilen klasörün tam yolu:
                _datasetPath = fbd.SelectedPath;

                var results = EvaluationService.Run(
                    datasetPath: _datasetPath,
                    trainRatio: 0.5,
                    blockSize: 8,
                    distThreshold: 0.15f,
                    simThreshold: 0.85f);

                // 1) CSV dosyasının tam yolunu oluştur
                var csvPath = Path.Combine(_datasetPath, "far_frr.csv");

                // 2) Dosya varsa üzerine yazılıp silinsin
                if (File.Exists(csvPath))
                    File.Delete(csvPath);

                // 3) StreamWriter ile başlık satırını ve verileri yaz
                using (var sw = new StreamWriter(csvPath))
                {
                    sw.WriteLine("Threshold;FAR;FRR");
                    foreach (var r in results)
                        sw.WriteLine($"{r.Threshold:F2};{r.FAR:F3};{r.FRR:F3}");
                }

                // 4) Bilgilendirme
                MessageBox.Show($"FAR/FRR değerleri '{csvPath}' dosyasına kaydedildi.");

                // Alt klasörleri alıp örnek logla:
                //var subDirs = Directory.GetDirectories(_datasetPath);
                //foreach (var dir in subDirs)
                //    Debug.WriteLine(dir);

            }
        }
        private void ExtractROIButton_Click(object sender, EventArgs e)
        {
            using var fbd = new FolderBrowserDialog();

            if(fbd.ShowDialog() == DialogResult.OK)
            {
                var root = fbd.SelectedPath;

                if (string.IsNullOrEmpty(root) || !Directory.Exists(root))
                {
                    MessageBox.Show("Lütfen geçerli bir dataset klasörü seçin.");
                    return;
                }

                var roiRoot = Path.Combine(root, "ROI");
                if (Directory.Exists(roiRoot))
                    Directory.Delete(roiRoot, recursive: true);

                var roiExt = new ROIExtractor();

                // 1) Tüm resim dosyalarını topla
                var files = Directory.GetFiles(root, "*.*", SearchOption.AllDirectories)
                    .Where(f => {
                        var ext = Path.GetExtension(f).ToLower();
                        return ext == ".png" || ext == ".bmp" || ext == ".jpg";
                    })
                    .ToList();

                // 2) Paralel olarak ROI çıkarma ve kaydetme
                Parallel.ForEach(files,
                    new ParallelOptions { MaxDegreeOfParallelism = Environment.ProcessorCount },
                    file =>
                    {
                        using var img = CvInvoke.Imread(file, ImreadModes.Color);
                        if (!roiExt.TryExtract(img, out Mat roi, out _))
                            return;

                        // Çıkışı kaydederken orijinal klasör yapısını koru
                        var relPath = Path.GetRelativePath(root, file);
                        var outPath = Path.Combine(roiRoot, relPath);
                        Directory.CreateDirectory(Path.GetDirectoryName(outPath)!);

                        CvInvoke.Imwrite(outPath, roi);
                        roi.Dispose();
                    });

                MessageBox.Show($"Tüm ROI’ler önbelleğe alındı:\n{roiRoot}");
            }
            
        }
        private void RefreshIdList() { listBoxIds.Items.Clear(); listBoxIds.Items.AddRange(_db.Records.Keys.ToArray()); }

        private void pictureBox1_Paint(object sender, PaintEventArgs e)
        {

        }

        private void pictureBox1_Click(object sender, EventArgs e)
        {

        }

        private void grayScalePictureBox_Click(object sender, EventArgs e)
        {

        }

        private void contourPictureBox_Click(object sender, EventArgs e)
        {

        }

        
    }
}
