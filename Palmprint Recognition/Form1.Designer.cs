namespace Palmprint_Recognition
{
    partial class Form1
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            pictureBox1 = new PictureBox();
            openFileDialog1 = new OpenFileDialog();
            uploadImageButton = new Button();
            recognitionButton = new Button();
            RegisterROIPictureBox = new PictureBox();
            enrollButton = new Button();
            userIDTextBox = new TextBox();
            label1 = new Label();
            listBoxIds = new ListBox();
            label2 = new Label();
            pictureBox2 = new PictureBox();
            LoginROIPictureBox = new PictureBox();
            RecognitionLabel = new Label();
            LoginImageButton = new Button();
            panel1 = new Panel();
            label3 = new Label();
            panel2 = new Panel();
            label4 = new Label();
            EvaluateButton = new Button();
            ExtractROIButton = new Button();
            ((System.ComponentModel.ISupportInitialize)pictureBox1).BeginInit();
            ((System.ComponentModel.ISupportInitialize)RegisterROIPictureBox).BeginInit();
            ((System.ComponentModel.ISupportInitialize)pictureBox2).BeginInit();
            ((System.ComponentModel.ISupportInitialize)LoginROIPictureBox).BeginInit();
            panel1.SuspendLayout();
            panel2.SuspendLayout();
            SuspendLayout();
            // 
            // pictureBox1
            // 
            pictureBox1.BorderStyle = BorderStyle.FixedSingle;
            pictureBox1.Location = new Point(33, 49);
            pictureBox1.Name = "pictureBox1";
            pictureBox1.Size = new Size(300, 300);
            pictureBox1.SizeMode = PictureBoxSizeMode.StretchImage;
            pictureBox1.TabIndex = 0;
            pictureBox1.TabStop = false;
            pictureBox1.Click += pictureBox1_Click;
            pictureBox1.Paint += pictureBox1_Paint;
            // 
            // openFileDialog1
            // 
            openFileDialog1.FileName = "openFileDialog1";
            // 
            // uploadImageButton
            // 
            uploadImageButton.Location = new Point(193, 3);
            uploadImageButton.Name = "uploadImageButton";
            uploadImageButton.Size = new Size(140, 40);
            uploadImageButton.TabIndex = 1;
            uploadImageButton.Text = "Resim Yükle";
            uploadImageButton.UseVisualStyleBackColor = true;
            uploadImageButton.Click += button1_Click;
            // 
            // recognitionButton
            // 
            recognitionButton.Location = new Point(30, 361);
            recognitionButton.Name = "recognitionButton";
            recognitionButton.Size = new Size(140, 40);
            recognitionButton.TabIndex = 2;
            recognitionButton.Text = "Giriş Yap";
            recognitionButton.UseVisualStyleBackColor = true;
            recognitionButton.Click += button2_Click;
            // 
            // RegisterROIPictureBox
            // 
            RegisterROIPictureBox.BorderStyle = BorderStyle.FixedSingle;
            RegisterROIPictureBox.Location = new Point(33, 410);
            RegisterROIPictureBox.Name = "RegisterROIPictureBox";
            RegisterROIPictureBox.Size = new Size(300, 300);
            RegisterROIPictureBox.SizeMode = PictureBoxSizeMode.StretchImage;
            RegisterROIPictureBox.TabIndex = 6;
            RegisterROIPictureBox.TabStop = false;
            // 
            // enrollButton
            // 
            enrollButton.Location = new Point(234, 361);
            enrollButton.Name = "enrollButton";
            enrollButton.Size = new Size(99, 43);
            enrollButton.TabIndex = 7;
            enrollButton.Text = "Kayıt Yap";
            enrollButton.UseVisualStyleBackColor = true;
            enrollButton.Click += enrollButton_Click;
            // 
            // userIDTextBox
            // 
            userIDTextBox.Font = new Font("Segoe UI", 12F, FontStyle.Regular, GraphicsUnit.Point, 0);
            userIDTextBox.Location = new Point(72, 365);
            userIDTextBox.Name = "userIDTextBox";
            userIDTextBox.Size = new Size(156, 34);
            userIDTextBox.TabIndex = 8;
            // 
            // label1
            // 
            label1.AutoSize = true;
            label1.Font = new Font("Segoe UI", 12F, FontStyle.Regular, GraphicsUnit.Point, 0);
            label1.Location = new Point(33, 368);
            label1.Name = "label1";
            label1.Size = new Size(33, 28);
            label1.TabIndex = 9;
            label1.Text = "Id:";
            // 
            // listBoxIds
            // 
            listBoxIds.FormattingEnabled = true;
            listBoxIds.Location = new Point(470, 55);
            listBoxIds.Name = "listBoxIds";
            listBoxIds.Size = new Size(200, 304);
            listBoxIds.TabIndex = 10;
            // 
            // label2
            // 
            label2.AutoSize = true;
            label2.Font = new Font("Segoe UI", 12F);
            label2.Location = new Point(465, 24);
            label2.Name = "label2";
            label2.Size = new Size(211, 28);
            label2.TabIndex = 11;
            label2.Text = "Sisteme Kayıtlı Olanlar:";
            // 
            // pictureBox2
            // 
            pictureBox2.BorderStyle = BorderStyle.FixedSingle;
            pictureBox2.Location = new Point(30, 55);
            pictureBox2.Name = "pictureBox2";
            pictureBox2.Size = new Size(300, 300);
            pictureBox2.SizeMode = PictureBoxSizeMode.StretchImage;
            pictureBox2.TabIndex = 12;
            pictureBox2.TabStop = false;
            // 
            // LoginROIPictureBox
            // 
            LoginROIPictureBox.BorderStyle = BorderStyle.FixedSingle;
            LoginROIPictureBox.Location = new Point(30, 406);
            LoginROIPictureBox.Name = "LoginROIPictureBox";
            LoginROIPictureBox.Size = new Size(300, 300);
            LoginROIPictureBox.SizeMode = PictureBoxSizeMode.StretchImage;
            LoginROIPictureBox.TabIndex = 13;
            LoginROIPictureBox.TabStop = false;
            // 
            // RecognitionLabel
            // 
            RecognitionLabel.AutoSize = true;
            RecognitionLabel.Font = new Font("Segoe UI", 12F, FontStyle.Regular, GraphicsUnit.Point, 0);
            RecognitionLabel.Location = new Point(392, 681);
            RecognitionLabel.Name = "RecognitionLabel";
            RecognitionLabel.Size = new Size(354, 28);
            RecognitionLabel.TabIndex = 14;
            RecognitionLabel.Text = "Öklid Mesafesine Göre Tanıma Yüzdesi: ";
            // 
            // LoginImageButton
            // 
            LoginImageButton.Location = new Point(190, 9);
            LoginImageButton.Name = "LoginImageButton";
            LoginImageButton.Size = new Size(140, 40);
            LoginImageButton.TabIndex = 15;
            LoginImageButton.Text = "Resim Yükle";
            LoginImageButton.UseVisualStyleBackColor = true;
            LoginImageButton.Click += LoginImageButton_Click;
            // 
            // panel1
            // 
            panel1.BorderStyle = BorderStyle.FixedSingle;
            panel1.Controls.Add(uploadImageButton);
            panel1.Controls.Add(pictureBox1);
            panel1.Controls.Add(label1);
            panel1.Controls.Add(enrollButton);
            panel1.Controls.Add(userIDTextBox);
            panel1.Controls.Add(RegisterROIPictureBox);
            panel1.Location = new Point(12, 24);
            panel1.Name = "panel1";
            panel1.Size = new Size(364, 717);
            panel1.TabIndex = 16;
            panel1.Tag = "";
            // 
            // label3
            // 
            label3.AutoSize = true;
            label3.Font = new Font("Segoe UI", 12F);
            label3.Location = new Point(16, 9);
            label3.Name = "label3";
            label3.Size = new Size(129, 28);
            label3.TabIndex = 17;
            label3.Text = "Sisteme Kayıt";
            // 
            // panel2
            // 
            panel2.BorderStyle = BorderStyle.FixedSingle;
            panel2.Controls.Add(LoginImageButton);
            panel2.Controls.Add(recognitionButton);
            panel2.Controls.Add(pictureBox2);
            panel2.Controls.Add(LoginROIPictureBox);
            panel2.Location = new Point(762, 24);
            panel2.Name = "panel2";
            panel2.Size = new Size(364, 717);
            panel2.TabIndex = 18;
            panel2.Tag = "";
            // 
            // label4
            // 
            label4.AutoSize = true;
            label4.Font = new Font("Segoe UI", 12F);
            label4.Location = new Point(766, 9);
            label4.Name = "label4";
            label4.Size = new Size(124, 28);
            label4.TabIndex = 19;
            label4.Text = "Sisteme Giriş";
            // 
            // EvaluateButton
            // 
            EvaluateButton.Location = new Point(498, 386);
            EvaluateButton.Name = "EvaluateButton";
            EvaluateButton.Size = new Size(140, 40);
            EvaluateButton.TabIndex = 20;
            EvaluateButton.Text = "Dosya Seç";
            EvaluateButton.UseVisualStyleBackColor = true;
            EvaluateButton.Visible = false;
            EvaluateButton.Click += EvaluateButton_Click;
            // 
            // ExtractROIButton
            // 
            ExtractROIButton.Location = new Point(498, 435);
            ExtractROIButton.Name = "ExtractROIButton";
            ExtractROIButton.Size = new Size(140, 40);
            ExtractROIButton.TabIndex = 21;
            ExtractROIButton.Text = "ROI Çıkart";
            ExtractROIButton.UseVisualStyleBackColor = true;
            ExtractROIButton.Visible = false;
            ExtractROIButton.Click += ExtractROIButton_Click;
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(8F, 20F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1141, 753);
            Controls.Add(ExtractROIButton);
            Controls.Add(EvaluateButton);
            Controls.Add(label4);
            Controls.Add(panel2);
            Controls.Add(label3);
            Controls.Add(panel1);
            Controls.Add(RecognitionLabel);
            Controls.Add(label2);
            Controls.Add(listBoxIds);
            Name = "Form1";
            StartPosition = FormStartPosition.CenterScreen;
            Text = "Palmprint Recognition";
            Load += Form1_Load;
            ((System.ComponentModel.ISupportInitialize)pictureBox1).EndInit();
            ((System.ComponentModel.ISupportInitialize)RegisterROIPictureBox).EndInit();
            ((System.ComponentModel.ISupportInitialize)pictureBox2).EndInit();
            ((System.ComponentModel.ISupportInitialize)LoginROIPictureBox).EndInit();
            panel1.ResumeLayout(false);
            panel1.PerformLayout();
            panel2.ResumeLayout(false);
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private PictureBox pictureBox1;
        private OpenFileDialog openFileDialog1;
        private Button uploadImageButton;
        private Button recognitionButton;
        private PictureBox RegisterROIPictureBox;
        private Button enrollButton;
        private TextBox userIDTextBox;
        private Label label1;
        private ListBox listBoxIds;
        private Label label2;
        private PictureBox pictureBox2;
        private PictureBox LoginROIPictureBox;
        private Label RecognitionLabel;
        private Button LoginImageButton;
        private Panel panel1;
        private Label label3;
        private Panel panel2;
        private Label label4;
        private Button EvaluateButton;
        private Button ExtractROIButton;
    }
}
