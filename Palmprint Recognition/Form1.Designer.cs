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
            label3 = new Label();
            LoginImageButton = new Button();
            ((System.ComponentModel.ISupportInitialize)pictureBox1).BeginInit();
            ((System.ComponentModel.ISupportInitialize)RegisterROIPictureBox).BeginInit();
            ((System.ComponentModel.ISupportInitialize)pictureBox2).BeginInit();
            ((System.ComponentModel.ISupportInitialize)LoginROIPictureBox).BeginInit();
            SuspendLayout();
            // 
            // pictureBox1
            // 
            pictureBox1.BorderStyle = BorderStyle.FixedSingle;
            pictureBox1.Location = new Point(12, 58);
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
            uploadImageButton.Location = new Point(12, 12);
            uploadImageButton.Name = "uploadImageButton";
            uploadImageButton.Size = new Size(140, 40);
            uploadImageButton.TabIndex = 1;
            uploadImageButton.Text = "Resim Yükle";
            uploadImageButton.UseVisualStyleBackColor = true;
            uploadImageButton.Click += button1_Click;
            // 
            // recognitionButton
            // 
            recognitionButton.Location = new Point(786, 381);
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
            RegisterROIPictureBox.Location = new Point(12, 441);
            RegisterROIPictureBox.Name = "RegisterROIPictureBox";
            RegisterROIPictureBox.Size = new Size(300, 300);
            RegisterROIPictureBox.SizeMode = PictureBoxSizeMode.StretchImage;
            RegisterROIPictureBox.TabIndex = 6;
            RegisterROIPictureBox.TabStop = false;
            // 
            // enrollButton
            // 
            enrollButton.Location = new Point(213, 383);
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
            userIDTextBox.Location = new Point(51, 387);
            userIDTextBox.Name = "userIDTextBox";
            userIDTextBox.Size = new Size(156, 34);
            userIDTextBox.TabIndex = 8;
            // 
            // label1
            // 
            label1.AutoSize = true;
            label1.Font = new Font("Segoe UI", 12F, FontStyle.Regular, GraphicsUnit.Point, 0);
            label1.Location = new Point(12, 390);
            label1.Name = "label1";
            label1.Size = new Size(33, 28);
            label1.TabIndex = 9;
            label1.Text = "Id:";
            // 
            // listBoxIds
            // 
            listBoxIds.FormattingEnabled = true;
            listBoxIds.Location = new Point(402, 58);
            listBoxIds.Name = "listBoxIds";
            listBoxIds.Size = new Size(200, 304);
            listBoxIds.TabIndex = 10;
            // 
            // label2
            // 
            label2.AutoSize = true;
            label2.Font = new Font("Segoe UI", 12F);
            label2.Location = new Point(398, 24);
            label2.Name = "label2";
            label2.Size = new Size(211, 28);
            label2.TabIndex = 11;
            label2.Text = "Sisteme Kayıtlı Olanlar:";
            // 
            // pictureBox2
            // 
            pictureBox2.BorderStyle = BorderStyle.FixedSingle;
            pictureBox2.Location = new Point(694, 58);
            pictureBox2.Name = "pictureBox2";
            pictureBox2.Size = new Size(300, 300);
            pictureBox2.SizeMode = PictureBoxSizeMode.StretchImage;
            pictureBox2.TabIndex = 12;
            pictureBox2.TabStop = false;
            // 
            // LoginROIPictureBox
            // 
            LoginROIPictureBox.BorderStyle = BorderStyle.FixedSingle;
            LoginROIPictureBox.Location = new Point(694, 441);
            LoginROIPictureBox.Name = "LoginROIPictureBox";
            LoginROIPictureBox.Size = new Size(300, 300);
            LoginROIPictureBox.SizeMode = PictureBoxSizeMode.StretchImage;
            LoginROIPictureBox.TabIndex = 13;
            LoginROIPictureBox.TabStop = false;
            // 
            // label3
            // 
            label3.AutoSize = true;
            label3.Font = new Font("Segoe UI", 12F, FontStyle.Regular, GraphicsUnit.Point, 0);
            label3.Location = new Point(334, 688);
            label3.Name = "label3";
            label3.Size = new Size(354, 28);
            label3.TabIndex = 14;
            label3.Text = "Öklid Mesafesine Göre Tanıma Yüzdesi: ";
            // 
            // LoginImageButton
            // 
            LoginImageButton.Location = new Point(854, 12);
            LoginImageButton.Name = "LoginImageButton";
            LoginImageButton.Size = new Size(140, 40);
            LoginImageButton.TabIndex = 15;
            LoginImageButton.Text = "Resim Yükle";
            LoginImageButton.UseVisualStyleBackColor = true;
            LoginImageButton.Click += LoginImageButton_Click;
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(8F, 20F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1006, 753);
            Controls.Add(LoginImageButton);
            Controls.Add(label3);
            Controls.Add(LoginROIPictureBox);
            Controls.Add(pictureBox2);
            Controls.Add(label2);
            Controls.Add(listBoxIds);
            Controls.Add(label1);
            Controls.Add(userIDTextBox);
            Controls.Add(enrollButton);
            Controls.Add(RegisterROIPictureBox);
            Controls.Add(recognitionButton);
            Controls.Add(uploadImageButton);
            Controls.Add(pictureBox1);
            Name = "Form1";
            StartPosition = FormStartPosition.CenterScreen;
            Text = "Palmprint Recognition";
            Load += Form1_Load;
            ((System.ComponentModel.ISupportInitialize)pictureBox1).EndInit();
            ((System.ComponentModel.ISupportInitialize)RegisterROIPictureBox).EndInit();
            ((System.ComponentModel.ISupportInitialize)pictureBox2).EndInit();
            ((System.ComponentModel.ISupportInitialize)LoginROIPictureBox).EndInit();
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
        private Label label3;
        private Button LoginImageButton;
    }
}
