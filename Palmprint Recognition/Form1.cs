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

namespace Palmprint_Recognition
{
    public partial class Form1 : Form
    {
        private readonly DatabaseManager _db;
        private readonly RawFeatureManager _rawFtManager;
        private readonly ROIExtractor _roiExt;
        private readonly FeatureExtractor _featExt;
        private readonly Recognizer _recognizer;
        private Mat _inputMat;
        private Mat _loginMat;
        private Mat _roi;
        private Mat _roiLogin;
        public Form1()
        {
            InitializeComponent();
            _db = new DatabaseManager("palmdb.csv");
            _rawFtManager = new RawFeatureManager("rawdb.csv");
            _roiExt = new ROIExtractor();
            _featExt = new FeatureExtractor();
            _recognizer = new Recognizer(_db, _rawFtManager, _featExt);
        }
        private void Form1_Load(object sender, EventArgs e)
        {
            _db.Load();
            _recognizer.Initialize(_rawFtManager.AllRaw);
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
            _inputMat?.Dispose(); pictureBox1.Image?.Dispose();_roi?.Dispose(); RegisterROIPictureBox.Image?.Dispose();
            using var tmp = CvInvoke.Imread(dlg.FileName, ImreadModes.Color);
            _inputMat = tmp.Clone(); pictureBox1.Image = _inputMat.ToBitmap();
            if (!_roiExt.TryExtract(_inputMat, out _roi, out var _mask)) { MessageBox.Show("ROI alınamadı."); return; }
            RegisterROIPictureBox.Image = _roi.ToBitmap();
            LoginROIPictureBox.Image = _mask.ToBitmap();
            
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
            _loginMat?.Dispose(); pictureBox2.Image?.Dispose(); _roiLogin?.Dispose(); LoginROIPictureBox.Image?.Dispose();
            using var tmp = CvInvoke.Imread(dlg.FileName, ImreadModes.Color);
            _loginMat = tmp.Clone(); pictureBox2.Image = _loginMat.ToBitmap();
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
            if (_loginMat == null) { MessageBox.Show("Önce giriş görüntüsü yükleyin."); return; }
            var res = _recognizer.Recognize(_roiLogin, 0.6f);
            MessageBox.Show(res != null ? $"Hoşgeldiniz, {res}!" : "Kayıt bulunamadı.");
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
            try { _recognizer.Enroll(id, _roi); MessageBox.Show($"'{id}' başarıyla kaydedildi."); RefreshIdList(); }
            catch (Emgu.CV.Util.CvException cvEx) { MessageBox.Show($"OpenCV hatası: {cvEx.Message}"); }
            catch (Exception ex) { MessageBox.Show(ex.Message); }
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
