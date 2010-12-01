namespace ArduPilotConfigTool
{
    partial class options_form
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
            this.reverse_Roll = new System.Windows.Forms.CheckBox();
            this.reverse_Elevator = new System.Windows.Forms.CheckBox();
            this.sensorZ = new System.Windows.Forms.CheckBox();
            this.txState = new System.Windows.Forms.CheckBox();
            this.txDefaultAction = new System.Windows.Forms.ComboBox();
            this.button_set = new System.Windows.Forms.Button();
            this.trimR = new System.Windows.Forms.TextBox();
            this.trimP = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.SuspendLayout();
            // 
            // reverse_Roll
            // 
            this.reverse_Roll.AutoSize = true;
            this.reverse_Roll.Location = new System.Drawing.Point(12, 12);
            this.reverse_Roll.Name = "reverse_Roll";
            this.reverse_Roll.Size = new System.Drawing.Size(93, 17);
            this.reverse_Roll.TabIndex = 0;
            this.reverse_Roll.Text = "Roll Reverse?";
            this.reverse_Roll.UseVisualStyleBackColor = true;
            this.reverse_Roll.Visible = false;
            this.reverse_Roll.CheckedChanged += new System.EventHandler(this.checkBox1_CheckedChanged);
            // 
            // reverse_Elevator
            // 
            this.reverse_Elevator.AutoSize = true;
            this.reverse_Elevator.Location = new System.Drawing.Point(12, 35);
            this.reverse_Elevator.Name = "reverse_Elevator";
            this.reverse_Elevator.Size = new System.Drawing.Size(114, 17);
            this.reverse_Elevator.TabIndex = 1;
            this.reverse_Elevator.Text = "Elevator Reverse?";
            this.reverse_Elevator.UseVisualStyleBackColor = true;
            this.reverse_Elevator.Visible = false;
            // 
            // sensorZ
            // 
            this.sensorZ.AutoSize = true;
            this.sensorZ.Location = new System.Drawing.Point(12, 58);
            this.sensorZ.Name = "sensorZ";
            this.sensorZ.Size = new System.Drawing.Size(111, 17);
            this.sensorZ.TabIndex = 2;
            this.sensorZ.Text = "Sensor Z Enable?";
            this.sensorZ.UseVisualStyleBackColor = true;
            this.sensorZ.Visible = false;
            // 
            // txState
            // 
            this.txState.AutoSize = true;
            this.txState.Location = new System.Drawing.Point(12, 81);
            this.txState.Name = "txState";
            this.txState.Size = new System.Drawing.Size(148, 17);
            this.txState.TabIndex = 3;
            this.txState.Text = "Radio has 2 state switch?";
            this.txState.UseVisualStyleBackColor = true;
            this.txState.Visible = false;
            this.txState.CheckedChanged += new System.EventHandler(this.txState_CheckedChanged);
            // 
            // txDefaultAction
            // 
            this.txDefaultAction.Enabled = false;
            this.txDefaultAction.FormattingEnabled = true;
            this.txDefaultAction.Items.AddRange(new object[] {
            "RTL",
            "Waypoints"});
            this.txDefaultAction.Location = new System.Drawing.Point(12, 104);
            this.txDefaultAction.Name = "txDefaultAction";
            this.txDefaultAction.Size = new System.Drawing.Size(148, 21);
            this.txDefaultAction.TabIndex = 4;
            this.txDefaultAction.Text = "Choose Default Action";
            this.txDefaultAction.Visible = false;
            this.txDefaultAction.SelectedIndexChanged += new System.EventHandler(this.txDefaultAction_SelectedIndexChanged);
            // 
            // button_set
            // 
            this.button_set.Location = new System.Drawing.Point(50, 189);
            this.button_set.Name = "button_set";
            this.button_set.Size = new System.Drawing.Size(110, 28);
            this.button_set.TabIndex = 5;
            this.button_set.Text = "Set";
            this.button_set.UseVisualStyleBackColor = true;
            this.button_set.Click += new System.EventHandler(this.button_set_Click);
            // 
            // trimR
            // 
            this.trimR.Location = new System.Drawing.Point(12, 131);
            this.trimR.Name = "trimR";
            this.trimR.Size = new System.Drawing.Size(45, 20);
            this.trimR.TabIndex = 6;
            this.trimR.Text = "0";
            // 
            // trimP
            // 
            this.trimP.Location = new System.Drawing.Point(12, 157);
            this.trimP.Name = "trimP";
            this.trimP.Size = new System.Drawing.Size(45, 20);
            this.trimP.TabIndex = 7;
            this.trimP.Text = "0";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(72, 134);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(48, 13);
            this.label1.TabIndex = 8;
            this.label1.Text = "Roll Trim";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(72, 157);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(69, 13);
            this.label2.TabIndex = 9;
            this.label2.Text = "Elevator Trim";
            // 
            // options_form
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(205, 229);
            this.ControlBox = false;
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.trimP);
            this.Controls.Add(this.trimR);
            this.Controls.Add(this.button_set);
            this.Controls.Add(this.txDefaultAction);
            this.Controls.Add(this.txState);
            this.Controls.Add(this.sensorZ);
            this.Controls.Add(this.reverse_Elevator);
            this.Controls.Add(this.reverse_Roll);
            this.Name = "options_form";
            this.Text = "AP Options";
            this.Load += new System.EventHandler(this.options_form_Load);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.CheckBox reverse_Roll;
        private System.Windows.Forms.CheckBox reverse_Elevator;
        private System.Windows.Forms.CheckBox sensorZ;
        private System.Windows.Forms.CheckBox txState;
        private System.Windows.Forms.ComboBox txDefaultAction;
        private System.Windows.Forms.Button button_set;
        private System.Windows.Forms.TextBox trimR;
        private System.Windows.Forms.TextBox trimP;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
    }
}