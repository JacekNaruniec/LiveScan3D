namespace KinectServer
{
    partial class AudioSettingsForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
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
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.btApply = new System.Windows.Forms.Button();
            this.label1 = new System.Windows.Forms.Label();
            this.cbDevices = new System.Windows.Forms.ComboBox();
            this.cbAudioEnabled = new System.Windows.Forms.CheckBox();
            this.tbSamplingRate = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.btApply);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.cbDevices);
            this.groupBox1.Controls.Add(this.cbAudioEnabled);
            this.groupBox1.Location = new System.Drawing.Point(5, 11);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(515, 211);
            this.groupBox1.TabIndex = 0;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Audio settings";
            // 
            // btApply
            // 
            this.btApply.Location = new System.Drawing.Point(295, 140);
            this.btApply.Name = "btApply";
            this.btApply.Size = new System.Drawing.Size(207, 39);
            this.btApply.TabIndex = 2;
            this.btApply.Text = "Apply";
            this.btApply.UseVisualStyleBackColor = true;
            this.btApply.Click += new System.EventHandler(this.btApply_Click);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(9, 34);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(99, 20);
            this.label1.TabIndex = 1;
            this.label1.Text = "Input device:";
            // 
            // cbDevices
            // 
            this.cbDevices.FormattingEnabled = true;
            this.cbDevices.Location = new System.Drawing.Point(13, 66);
            this.cbDevices.Name = "cbDevices";
            this.cbDevices.Size = new System.Drawing.Size(489, 28);
            this.cbDevices.TabIndex = 0;
            this.cbDevices.SelectedIndexChanged += new System.EventHandler(this.cbDevices_SelectedIndexChanged);
            // 
            // cbAudioEnabled
            // 
            this.cbAudioEnabled.AutoSize = true;
            this.cbAudioEnabled.Checked = true;
            this.cbAudioEnabled.CheckState = System.Windows.Forms.CheckState.Checked;
            this.cbAudioEnabled.Location = new System.Drawing.Point(13, 181);
            this.cbAudioEnabled.Name = "cbAudioEnabled";
            this.cbAudioEnabled.Size = new System.Drawing.Size(137, 24);
            this.cbAudioEnabled.TabIndex = 1;
            this.cbAudioEnabled.Text = "Audio enabled";
            this.cbAudioEnabled.UseVisualStyleBackColor = true;
            this.cbAudioEnabled.CheckedChanged += new System.EventHandler(this.cbAudioEnabled_CheckedChanged);
            // 
            // tbSamplingRate
            // 
            this.tbSamplingRate.Location = new System.Drawing.Point(18, 151);
            this.tbSamplingRate.Name = "tbSamplingRate";
            this.tbSamplingRate.Size = new System.Drawing.Size(206, 26);
            this.tbSamplingRate.TabIndex = 2;
            this.tbSamplingRate.Text = "44100";
            this.tbSamplingRate.TextChanged += new System.EventHandler(this.tbSamplingRate_TextChanged);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(14, 119);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(111, 20);
            this.label2.TabIndex = 3;
            this.label2.Text = "Sampling rate:";
            // 
            // AudioSettingsForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(532, 227);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.tbSamplingRate);
            this.Controls.Add(this.groupBox1);
            this.Name = "AudioSettingsForm";
            this.Text = "Audio settings";
            this.Load += new System.EventHandler(this.AudioSettingsForm_Load);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.ComboBox cbDevices;
        private System.Windows.Forms.CheckBox cbAudioEnabled;
        private System.Windows.Forms.TextBox tbSamplingRate;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Button btApply;
    }
}