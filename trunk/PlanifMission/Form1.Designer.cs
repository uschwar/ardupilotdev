namespace ArduPilotConfigTool
{
    partial class Form1
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
            this.components = new System.ComponentModel.Container();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle1 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle2 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle3 = new System.Windows.Forms.DataGridViewCellStyle();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Form1));
            this.missionGrid = new System.Windows.Forms.DataGridView();
            this.Number = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Lat = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Lon = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Altitude = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Hidden_Alt = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.SeaLevelAlt = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.launchLat = new System.Windows.Forms.TextBox();
            this.launchLon = new System.Windows.Forms.TextBox();
            this.launchManually = new System.Windows.Forms.CheckBox();
            this.mnuMain = new System.Windows.Forms.MenuStrip();
            this.mnuFiles = new System.Windows.Forms.ToolStripMenuItem();
            this.mnuLoadMission = new System.Windows.Forms.ToolStripMenuItem();
            this.mnuSaveMission = new System.Windows.Forms.ToolStripMenuItem();
            this.mnuSaveMissionAs = new System.Windows.Forms.ToolStripMenuItem();
            this.toolStripSeparator5 = new System.Windows.Forms.ToolStripSeparator();
            this.cboComSelect = new System.Windows.Forms.ToolStripComboBox();
            this.cboBoardType = new System.Windows.Forms.ToolStripComboBox();
            this.cboMetersFeet = new System.Windows.Forms.ToolStripComboBox();
            this.toolStripSeparator6 = new System.Windows.Forms.ToolStripSeparator();
            this.mnuRead = new System.Windows.Forms.ToolStripMenuItem();
            this.mnuWrite = new System.Windows.Forms.ToolStripMenuItem();
            this.toolStripSeparator7 = new System.Windows.Forms.ToolStripSeparator();
            this.mnuExit = new System.Windows.Forms.ToolStripMenuItem();
            this.moreToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.optionsToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.restartMaxAltitutdeToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.dIYdronescomToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.launchAlt_ref = new System.Windows.Forms.TextBox();
            this.Launch_paramters = new System.Windows.Forms.GroupBox();
            this.chkLookupAlt = new System.Windows.Forms.CheckBox();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.launchAlt_hold = new System.Windows.Forms.TextBox();
            this.label8 = new System.Windows.Forms.Label();
            this.webBrowser1 = new System.Windows.Forms.WebBrowser();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.fraStatus = new System.Windows.Forms.GroupBox();
            this.cmdWrite = new System.Windows.Forms.Button();
            this.cmdRead = new System.Windows.Forms.Button();
            this.progressBar1 = new System.Windows.Forms.ProgressBar();
            this.statusLabel = new System.Windows.Forms.Label();
            this.lblClickMap = new System.Windows.Forms.Label();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.label7 = new System.Windows.Forms.Label();
            this.radius = new System.Windows.Forms.TextBox();
            this.lblHomeAltLabel = new System.Windows.Forms.Label();
            this.lblHomeAlt = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.maxSpd = new System.Windows.Forms.Label();
            this.maxAlt = new System.Windows.Forms.Label();
            this.wp_number = new System.Windows.Forms.Label();
            this.grpOffline = new System.Windows.Forms.GroupBox();
            this.cmdAddOffWaypoint = new System.Windows.Forms.Button();
            this.cmdClearOffWaypoints = new System.Windows.Forms.Button();
            this.cmdRemoveOffWaypoint = new System.Windows.Forms.Button();
            this.picMap = new System.Windows.Forms.PictureBox();
            this.grpOnline = new System.Windows.Forms.GroupBox();
            this.cmdCenter = new System.Windows.Forms.Button();
            this.cmdClear = new System.Windows.Forms.Button();
            this.optWaypoint = new System.Windows.Forms.RadioButton();
            this.optHome = new System.Windows.Forms.RadioButton();
            this.cmdSearch = new System.Windows.Forms.Button();
            this.lblAddress = new System.Windows.Forms.Label();
            this.txtAddress = new System.Windows.Forms.TextBox();
            this.label9 = new System.Windows.Forms.Label();
            this.numericMaps = new System.Windows.Forms.NumericUpDown();
            this.RemoveRow = new System.Windows.Forms.Button();
            this.lblOfflineLabel = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.missionGrid)).BeginInit();
            this.mnuMain.SuspendLayout();
            this.Launch_paramters.SuspendLayout();
            this.fraStatus.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.grpOffline.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.picMap)).BeginInit();
            this.grpOnline.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericMaps)).BeginInit();
            this.SuspendLayout();
            // 
            // missionGrid
            // 
            this.missionGrid.AllowUserToAddRows = false;
            this.missionGrid.AllowUserToDeleteRows = false;
            this.missionGrid.AllowUserToResizeColumns = false;
            this.missionGrid.AllowUserToResizeRows = false;
            this.missionGrid.BackgroundColor = System.Drawing.Color.White;
            this.missionGrid.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.missionGrid.Columns.AddRange(new System.Windows.Forms.DataGridViewColumn[] {
            this.Number,
            this.Lat,
            this.Lon,
            this.Altitude,
            this.Hidden_Alt,
            this.SeaLevelAlt});
            this.missionGrid.Location = new System.Drawing.Point(117, 31);
            this.missionGrid.Name = "missionGrid";
            this.missionGrid.RowHeadersVisible = false;
            this.missionGrid.RowHeadersWidthSizeMode = System.Windows.Forms.DataGridViewRowHeadersWidthSizeMode.DisableResizing;
            this.missionGrid.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.missionGrid.Size = new System.Drawing.Size(346, 293);
            this.missionGrid.TabIndex = 1;
            this.missionGrid.Enter += new System.EventHandler(this.missionGrid_Enter);
            this.missionGrid.Leave += new System.EventHandler(this.missionGrid_Leave);
            this.missionGrid.CellEndEdit += new System.Windows.Forms.DataGridViewCellEventHandler(this.missionGrid_CellEndEdit);
            this.missionGrid.CellContentClick += new System.Windows.Forms.DataGridViewCellEventHandler(this.missionGrid_CellContentClick);
            // 
            // Number
            // 
            this.Number.Frozen = true;
            this.Number.HeaderText = "#";
            this.Number.Name = "Number";
            this.Number.ReadOnly = true;
            this.Number.Resizable = System.Windows.Forms.DataGridViewTriState.False;
            this.Number.SortMode = System.Windows.Forms.DataGridViewColumnSortMode.NotSortable;
            this.Number.Width = 30;
            // 
            // Lat
            // 
            this.Lat.HeaderText = "Latitude";
            this.Lat.Name = "Lat";
            this.Lat.SortMode = System.Windows.Forms.DataGridViewColumnSortMode.NotSortable;
            this.Lat.Width = 70;
            // 
            // Lon
            // 
            this.Lon.HeaderText = "Longitude";
            this.Lon.Name = "Lon";
            this.Lon.SortMode = System.Windows.Forms.DataGridViewColumnSortMode.NotSortable;
            this.Lon.Width = 70;
            // 
            // Altitude
            // 
            dataGridViewCellStyle1.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            dataGridViewCellStyle1.Format = "N0";
            dataGridViewCellStyle1.NullValue = null;
            this.Altitude.DefaultCellStyle = dataGridViewCellStyle1;
            this.Altitude.HeaderText = "Altitude (QNH)";
            this.Altitude.Name = "Altitude";
            this.Altitude.ReadOnly = true;
            this.Altitude.SortMode = System.Windows.Forms.DataGridViewColumnSortMode.NotSortable;
            this.Altitude.Width = 50;
            // 
            // Hidden_Alt
            // 
            dataGridViewCellStyle2.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            dataGridViewCellStyle2.NullValue = null;
            this.Hidden_Alt.DefaultCellStyle = dataGridViewCellStyle2;
            this.Hidden_Alt.HeaderText = "Altitude AGL";
            this.Hidden_Alt.Name = "Hidden_Alt";
            this.Hidden_Alt.Width = 50;
            // 
            // SeaLevelAlt
            // 
            dataGridViewCellStyle3.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            dataGridViewCellStyle3.NullValue = null;
            this.SeaLevelAlt.DefaultCellStyle = dataGridViewCellStyle3;
            this.SeaLevelAlt.HeaderText = "Alt du sol (QFE)";
            this.SeaLevelAlt.Name = "SeaLevelAlt";
            this.SeaLevelAlt.ReadOnly = true;
            this.SeaLevelAlt.Width = 60;
            // 
            // launchLat
            // 
            this.launchLat.Enabled = false;
            this.launchLat.Location = new System.Drawing.Point(6, 32);
            this.launchLat.Name = "launchLat";
            this.launchLat.Size = new System.Drawing.Size(79, 20);
            this.launchLat.TabIndex = 3;
            this.launchLat.Text = "0.000000";
            this.launchLat.TextChanged += new System.EventHandler(this.launchLat_TextChanged);
            this.launchLat.Leave += new System.EventHandler(this.launchLat_Leave);
            this.launchLat.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.launchLat_KeyPress);
            // 
            // launchLon
            // 
            this.launchLon.Enabled = false;
            this.launchLon.Location = new System.Drawing.Point(6, 71);
            this.launchLon.Name = "launchLon";
            this.launchLon.Size = new System.Drawing.Size(79, 20);
            this.launchLon.TabIndex = 4;
            this.launchLon.Text = "0.000000";
            this.launchLon.TextChanged += new System.EventHandler(this.launchLon_TextChanged);
            this.launchLon.Leave += new System.EventHandler(this.launchLon_Leave);
            this.launchLon.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.launchLon_KeyPress);
            // 
            // launchManually
            // 
            this.launchManually.AutoSize = true;
            this.launchManually.Location = new System.Drawing.Point(6, 175);
            this.launchManually.Name = "launchManually";
            this.launchManually.Size = new System.Drawing.Size(100, 17);
            this.launchManually.TabIndex = 6;
            this.launchManually.Text = "Saisie Manuelle";
            this.launchManually.UseVisualStyleBackColor = true;
            this.launchManually.CheckedChanged += new System.EventHandler(this.launchManually_CheckedChanged);
            // 
            // mnuMain
            // 
            this.mnuMain.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.mnuFiles,
            this.moreToolStripMenuItem,
            this.dIYdronescomToolStripMenuItem});
            this.mnuMain.Location = new System.Drawing.Point(0, 0);
            this.mnuMain.Name = "mnuMain";
            this.mnuMain.Size = new System.Drawing.Size(998, 24);
            this.mnuMain.TabIndex = 13;
            this.mnuMain.Text = "menuStrip1";
            this.mnuMain.ItemClicked += new System.Windows.Forms.ToolStripItemClickedEventHandler(this.mnuMain_ItemClicked);
            // 
            // mnuFiles
            // 
            this.mnuFiles.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.mnuLoadMission,
            this.mnuSaveMission,
            this.mnuSaveMissionAs,
            this.toolStripSeparator5,
            this.cboComSelect,
            this.cboBoardType,
            this.cboMetersFeet,
            this.toolStripSeparator6,
            this.mnuRead,
            this.mnuWrite,
            this.toolStripSeparator7,
            this.mnuExit});
            this.mnuFiles.Name = "mnuFiles";
            this.mnuFiles.Size = new System.Drawing.Size(54, 20);
            this.mnuFiles.Text = "Fichier";
            this.mnuFiles.Click += new System.EventHandler(this.toolStripMenuItem8_Click);
            // 
            // mnuLoadMission
            // 
            this.mnuLoadMission.Name = "mnuLoadMission";
            this.mnuLoadMission.Size = new System.Drawing.Size(201, 22);
            this.mnuLoadMission.Text = "Charger une Mission";
            this.mnuLoadMission.Click += new System.EventHandler(this.mnuLoadMission_Click);
            // 
            // mnuSaveMission
            // 
            this.mnuSaveMission.Name = "mnuSaveMission";
            this.mnuSaveMission.Size = new System.Drawing.Size(201, 22);
            this.mnuSaveMission.Text = "Sauver la Mission";
            this.mnuSaveMission.Click += new System.EventHandler(this.mnuSaveMission_Click);
            // 
            // mnuSaveMissionAs
            // 
            this.mnuSaveMissionAs.Name = "mnuSaveMissionAs";
            this.mnuSaveMissionAs.Size = new System.Drawing.Size(201, 22);
            this.mnuSaveMissionAs.Text = "Sauver la Mission sous...";
            this.mnuSaveMissionAs.Visible = false;
            // 
            // toolStripSeparator5
            // 
            this.toolStripSeparator5.Name = "toolStripSeparator5";
            this.toolStripSeparator5.Size = new System.Drawing.Size(198, 6);
            // 
            // cboComSelect
            // 
            this.cboComSelect.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cboComSelect.DropDownWidth = 100;
            this.cboComSelect.Name = "cboComSelect";
            this.cboComSelect.Size = new System.Drawing.Size(140, 23);
            this.cboComSelect.SelectedIndexChanged += new System.EventHandler(this.cboComSelect_SelectedIndexChanged);
            // 
            // cboBoardType
            // 
            this.cboBoardType.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cboBoardType.Items.AddRange(new object[] {
            "AT168",
            "AT328"});
            this.cboBoardType.Name = "cboBoardType";
            this.cboBoardType.Size = new System.Drawing.Size(140, 23);
            this.cboBoardType.SelectedIndexChanged += new System.EventHandler(this.cboBoardType_SelectedIndexChanged);
            // 
            // cboMetersFeet
            // 
            this.cboMetersFeet.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cboMetersFeet.Items.AddRange(new object[] {
            "Feet",
            "Meters"});
            this.cboMetersFeet.Name = "cboMetersFeet";
            this.cboMetersFeet.Size = new System.Drawing.Size(140, 23);
            this.cboMetersFeet.SelectedIndexChanged += new System.EventHandler(this.cboMetersFeet_SelectedIndexChanged);
            this.cboMetersFeet.Click += new System.EventHandler(this.cboMetersFeet_Click);
            // 
            // toolStripSeparator6
            // 
            this.toolStripSeparator6.Name = "toolStripSeparator6";
            this.toolStripSeparator6.Size = new System.Drawing.Size(198, 6);
            // 
            // mnuRead
            // 
            this.mnuRead.Name = "mnuRead";
            this.mnuRead.Size = new System.Drawing.Size(201, 22);
            this.mnuRead.Text = "Lire";
            this.mnuRead.Click += new System.EventHandler(this.mnuRead_Click);
            // 
            // mnuWrite
            // 
            this.mnuWrite.Name = "mnuWrite";
            this.mnuWrite.Size = new System.Drawing.Size(201, 22);
            this.mnuWrite.Text = "Ecrire";
            this.mnuWrite.Click += new System.EventHandler(this.mnuWrite_Click);
            // 
            // toolStripSeparator7
            // 
            this.toolStripSeparator7.Name = "toolStripSeparator7";
            this.toolStripSeparator7.Size = new System.Drawing.Size(198, 6);
            // 
            // mnuExit
            // 
            this.mnuExit.Name = "mnuExit";
            this.mnuExit.Size = new System.Drawing.Size(201, 22);
            this.mnuExit.Text = "Exit";
            this.mnuExit.Click += new System.EventHandler(this.toolStripMenuItem14_Click);
            // 
            // moreToolStripMenuItem
            // 
            this.moreToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.optionsToolStripMenuItem,
            this.restartMaxAltitutdeToolStripMenuItem});
            this.moreToolStripMenuItem.Name = "moreToolStripMenuItem";
            this.moreToolStripMenuItem.Size = new System.Drawing.Size(48, 20);
            this.moreToolStripMenuItem.Text = "Autre";
            // 
            // optionsToolStripMenuItem
            // 
            this.optionsToolStripMenuItem.Name = "optionsToolStripMenuItem";
            this.optionsToolStripMenuItem.Size = new System.Drawing.Size(154, 22);
            this.optionsToolStripMenuItem.Text = "Options";
            this.optionsToolStripMenuItem.Click += new System.EventHandler(this.optionsToolStripMenuItem_Click);
            // 
            // restartMaxAltitutdeToolStripMenuItem
            // 
            this.restartMaxAltitutdeToolStripMenuItem.Name = "restartMaxAltitutdeToolStripMenuItem";
            this.restartMaxAltitutdeToolStripMenuItem.Size = new System.Drawing.Size(154, 22);
            this.restartMaxAltitutdeToolStripMenuItem.Text = "Raz Alt/Vit max";
            this.restartMaxAltitutdeToolStripMenuItem.Click += new System.EventHandler(this.restartMaxAltitutdeToolStripMenuItem_Click);
            // 
            // dIYdronescomToolStripMenuItem
            // 
            this.dIYdronescomToolStripMenuItem.Alignment = System.Windows.Forms.ToolStripItemAlignment.Right;
            this.dIYdronescomToolStripMenuItem.Name = "dIYdronescomToolStripMenuItem";
            this.dIYdronescomToolStripMenuItem.Size = new System.Drawing.Size(100, 20);
            this.dIYdronescomToolStripMenuItem.Text = "DIYdrones.com";
            this.dIYdronescomToolStripMenuItem.Click += new System.EventHandler(this.dIYdronescomToolStripMenuItem_Click);
            // 
            // launchAlt_ref
            // 
            this.launchAlt_ref.Location = new System.Drawing.Point(6, 149);
            this.launchAlt_ref.Name = "launchAlt_ref";
            this.launchAlt_ref.ReadOnly = true;
            this.launchAlt_ref.Size = new System.Drawing.Size(79, 20);
            this.launchAlt_ref.TabIndex = 23;
            this.launchAlt_ref.Text = "0";
            // 
            // Launch_paramters
            // 
            this.Launch_paramters.Controls.Add(this.chkLookupAlt);
            this.Launch_paramters.Controls.Add(this.label4);
            this.Launch_paramters.Controls.Add(this.label3);
            this.Launch_paramters.Controls.Add(this.label2);
            this.Launch_paramters.Controls.Add(this.label1);
            this.Launch_paramters.Controls.Add(this.launchAlt_hold);
            this.Launch_paramters.Controls.Add(this.launchAlt_ref);
            this.Launch_paramters.Controls.Add(this.launchLat);
            this.Launch_paramters.Controls.Add(this.launchLon);
            this.Launch_paramters.Controls.Add(this.launchManually);
            this.Launch_paramters.Location = new System.Drawing.Point(6, 27);
            this.Launch_paramters.Name = "Launch_paramters";
            this.Launch_paramters.Size = new System.Drawing.Size(105, 227);
            this.Launch_paramters.TabIndex = 24;
            this.Launch_paramters.TabStop = false;
            this.Launch_paramters.Text = "Position terrain";
            this.Launch_paramters.Enter += new System.EventHandler(this.Launch_paramters_Enter);
            // 
            // chkLookupAlt
            // 
            this.chkLookupAlt.AutoSize = true;
            this.chkLookupAlt.Location = new System.Drawing.Point(6, 198);
            this.chkLookupAlt.Name = "chkLookupAlt";
            this.chkLookupAlt.Size = new System.Drawing.Size(65, 17);
            this.chkLookupAlt.TabIndex = 30;
            this.chkLookupAlt.Text = "Alt QNH";
            this.chkLookupAlt.UseVisualStyleBackColor = true;
            this.chkLookupAlt.CheckedChanged += new System.EventHandler(this.chkLookupAlt_CheckedChanged);
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(6, 133);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(74, 13);
            this.label4.TabIndex = 29;
            this.label4.Text = "Altitude de vol";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(6, 94);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(83, 13);
            this.label3.TabIndex = 28;
            this.label3.Text = "Alt sol (maintien)";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(6, 55);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(54, 13);
            this.label2.TabIndex = 27;
            this.label2.Text = "Longitude";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(6, 16);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(45, 13);
            this.label1.TabIndex = 26;
            this.label1.Text = "Latitude";
            // 
            // launchAlt_hold
            // 
            this.launchAlt_hold.Location = new System.Drawing.Point(6, 110);
            this.launchAlt_hold.Name = "launchAlt_hold";
            this.launchAlt_hold.Size = new System.Drawing.Size(79, 20);
            this.launchAlt_hold.TabIndex = 25;
            this.launchAlt_hold.Text = "150";
            this.launchAlt_hold.TextChanged += new System.EventHandler(this.launchAlt_hold_TextChanged);
            this.launchAlt_hold.Leave += new System.EventHandler(this.launchAlt_hold_Leave);
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Font = new System.Drawing.Font("Microsoft Sans Serif", 6F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label8.Location = new System.Drawing.Point(797, 508);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(185, 9);
            this.label8.TabIndex = 28;
            this.label8.Text = "Par Jordi Muño, HappyKillmore et Jean-Louis Naudin";
            // 
            // webBrowser1
            // 
            this.webBrowser1.Location = new System.Drawing.Point(469, 27);
            this.webBrowser1.MinimumSize = new System.Drawing.Size(20, 20);
            this.webBrowser1.Name = "webBrowser1";
            this.webBrowser1.ScrollBarsEnabled = false;
            this.webBrowser1.Size = new System.Drawing.Size(522, 476);
            this.webBrowser1.TabIndex = 0;
            this.webBrowser1.WebBrowserShortcutsEnabled = false;
            // 
            // timer1
            // 
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // fraStatus
            // 
            this.fraStatus.Controls.Add(this.cmdWrite);
            this.fraStatus.Controls.Add(this.cmdRead);
            this.fraStatus.Controls.Add(this.progressBar1);
            this.fraStatus.Controls.Add(this.statusLabel);
            this.fraStatus.Location = new System.Drawing.Point(6, 431);
            this.fraStatus.Name = "fraStatus";
            this.fraStatus.Size = new System.Drawing.Size(457, 86);
            this.fraStatus.TabIndex = 43;
            this.fraStatus.TabStop = false;
            this.fraStatus.Text = "Status";
            // 
            // cmdWrite
            // 
            this.cmdWrite.Location = new System.Drawing.Point(350, 10);
            this.cmdWrite.Name = "cmdWrite";
            this.cmdWrite.Size = new System.Drawing.Size(97, 22);
            this.cmdWrite.TabIndex = 20;
            this.cmdWrite.Text = "Ecrire";
            this.cmdWrite.UseVisualStyleBackColor = true;
            this.cmdWrite.Click += new System.EventHandler(this.cmdWrite_Click);
            // 
            // cmdRead
            // 
            this.cmdRead.Location = new System.Drawing.Point(242, 10);
            this.cmdRead.Name = "cmdRead";
            this.cmdRead.Size = new System.Drawing.Size(97, 22);
            this.cmdRead.TabIndex = 19;
            this.cmdRead.Text = "Lire";
            this.cmdRead.UseVisualStyleBackColor = true;
            this.cmdRead.Click += new System.EventHandler(this.cmdRead_Click);
            // 
            // progressBar1
            // 
            this.progressBar1.Location = new System.Drawing.Point(6, 19);
            this.progressBar1.Name = "progressBar1";
            this.progressBar1.Size = new System.Drawing.Size(230, 10);
            this.progressBar1.TabIndex = 18;
            // 
            // statusLabel
            // 
            this.statusLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.statusLabel.Location = new System.Drawing.Point(8, 35);
            this.statusLabel.Name = "statusLabel";
            this.statusLabel.Size = new System.Drawing.Size(439, 48);
            this.statusLabel.TabIndex = 17;
            this.statusLabel.Text = "Planificateur de Mission... Prêt !";
            this.statusLabel.UseCompatibleTextRendering = true;
            this.statusLabel.Click += new System.EventHandler(this.statusLabel_Click);
            // 
            // lblClickMap
            // 
            this.lblClickMap.AutoSize = true;
            this.lblClickMap.BackColor = System.Drawing.Color.White;
            this.lblClickMap.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.lblClickMap.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblClickMap.ForeColor = System.Drawing.Color.Blue;
            this.lblClickMap.Location = new System.Drawing.Point(150, 178);
            this.lblClickMap.Name = "lblClickMap";
            this.lblClickMap.Size = new System.Drawing.Size(277, 18);
            this.lblClickMap.TabIndex = 46;
            this.lblClickMap.Text = "Cliquez sur la carte pour ajouter un WP";
            this.lblClickMap.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.lblClickMap.Click += new System.EventHandler(this.lblClickMap_Click);
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.label7);
            this.groupBox2.Controls.Add(this.radius);
            this.groupBox2.Controls.Add(this.lblHomeAltLabel);
            this.groupBox2.Controls.Add(this.lblHomeAlt);
            this.groupBox2.Controls.Add(this.label6);
            this.groupBox2.Controls.Add(this.label5);
            this.groupBox2.Controls.Add(this.maxSpd);
            this.groupBox2.Controls.Add(this.maxAlt);
            this.groupBox2.Controls.Add(this.wp_number);
            this.groupBox2.Location = new System.Drawing.Point(6, 256);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(105, 170);
            this.groupBox2.TabIndex = 48;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Autres réglages";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(2, 16);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(88, 13);
            this.label7.TabIndex = 54;
            this.label7.Text = "Rayon de virage:";
            // 
            // radius
            // 
            this.radius.Location = new System.Drawing.Point(5, 32);
            this.radius.Name = "radius";
            this.radius.Size = new System.Drawing.Size(60, 20);
            this.radius.TabIndex = 53;
            this.radius.Text = "60";
            this.radius.TextChanged += new System.EventHandler(this.radius_TextChanged);
            this.radius.Leave += new System.EventHandler(this.radius_Leave);
            // 
            // lblHomeAltLabel
            // 
            this.lblHomeAltLabel.AutoSize = true;
            this.lblHomeAltLabel.Location = new System.Drawing.Point(3, 108);
            this.lblHomeAltLabel.Name = "lblHomeAltLabel";
            this.lblHomeAltLabel.Size = new System.Drawing.Size(77, 13);
            this.lblHomeAltLabel.TabIndex = 52;
            this.lblHomeAltLabel.Text = "Altitude terrain:";
            // 
            // lblHomeAlt
            // 
            this.lblHomeAlt.Location = new System.Drawing.Point(61, 108);
            this.lblHomeAlt.Name = "lblHomeAlt";
            this.lblHomeAlt.Size = new System.Drawing.Size(40, 13);
            this.lblHomeAlt.TabIndex = 51;
            this.lblHomeAlt.Text = "0";
            this.lblHomeAlt.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(2, 89);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(68, 13);
            this.label6.TabIndex = 50;
            this.label6.Text = "Altitude Max:";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(2, 70);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(67, 13);
            this.label5.TabIndex = 49;
            this.label5.Text = "Vitesse Max:";
            // 
            // maxSpd
            // 
            this.maxSpd.Location = new System.Drawing.Point(67, 70);
            this.maxSpd.Name = "maxSpd";
            this.maxSpd.Size = new System.Drawing.Size(34, 19);
            this.maxSpd.TabIndex = 48;
            this.maxSpd.Text = "0";
            this.maxSpd.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // maxAlt
            // 
            this.maxAlt.Location = new System.Drawing.Point(64, 90);
            this.maxAlt.Name = "maxAlt";
            this.maxAlt.Size = new System.Drawing.Size(37, 13);
            this.maxAlt.TabIndex = 47;
            this.maxAlt.Text = "0";
            this.maxAlt.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // wp_number
            // 
            this.wp_number.AutoSize = true;
            this.wp_number.Location = new System.Drawing.Point(76, 179);
            this.wp_number.Name = "wp_number";
            this.wp_number.Size = new System.Drawing.Size(13, 13);
            this.wp_number.TabIndex = 46;
            this.wp_number.Text = "0";
            this.wp_number.Visible = false;
            // 
            // grpOffline
            // 
            this.grpOffline.Controls.Add(this.cmdAddOffWaypoint);
            this.grpOffline.Controls.Add(this.cmdClearOffWaypoints);
            this.grpOffline.Controls.Add(this.cmdRemoveOffWaypoint);
            this.grpOffline.Location = new System.Drawing.Point(117, 330);
            this.grpOffline.Name = "grpOffline";
            this.grpOffline.Size = new System.Drawing.Size(346, 96);
            this.grpOffline.TabIndex = 49;
            this.grpOffline.TabStop = false;
            this.grpOffline.Text = "Waypoint Setup (Offline Mode)";
            // 
            // cmdAddOffWaypoint
            // 
            this.cmdAddOffWaypoint.Location = new System.Drawing.Point(6, 19);
            this.cmdAddOffWaypoint.Name = "cmdAddOffWaypoint";
            this.cmdAddOffWaypoint.Size = new System.Drawing.Size(100, 23);
            this.cmdAddOffWaypoint.TabIndex = 54;
            this.cmdAddOffWaypoint.Text = "Add Waypoint";
            this.cmdAddOffWaypoint.UseVisualStyleBackColor = true;
            this.cmdAddOffWaypoint.Click += new System.EventHandler(this.cmdAddOffWaypoint_Click);
            // 
            // cmdClearOffWaypoints
            // 
            this.cmdClearOffWaypoints.Enabled = false;
            this.cmdClearOffWaypoints.Location = new System.Drawing.Point(222, 19);
            this.cmdClearOffWaypoints.Name = "cmdClearOffWaypoints";
            this.cmdClearOffWaypoints.Size = new System.Drawing.Size(114, 23);
            this.cmdClearOffWaypoints.TabIndex = 53;
            this.cmdClearOffWaypoints.Text = "Clear All Waypoints";
            this.cmdClearOffWaypoints.UseVisualStyleBackColor = true;
            this.cmdClearOffWaypoints.Click += new System.EventHandler(this.cmdClearOffWaypoints_Click);
            // 
            // cmdRemoveOffWaypoint
            // 
            this.cmdRemoveOffWaypoint.Enabled = false;
            this.cmdRemoveOffWaypoint.Location = new System.Drawing.Point(112, 19);
            this.cmdRemoveOffWaypoint.Name = "cmdRemoveOffWaypoint";
            this.cmdRemoveOffWaypoint.Size = new System.Drawing.Size(104, 23);
            this.cmdRemoveOffWaypoint.TabIndex = 52;
            this.cmdRemoveOffWaypoint.Text = "Remove Waypoint";
            this.cmdRemoveOffWaypoint.UseVisualStyleBackColor = true;
            this.cmdRemoveOffWaypoint.Click += new System.EventHandler(this.cmdRemoveOffWaypoint_Click);
            // 
            // picMap
            // 
            this.picMap.Location = new System.Drawing.Point(469, 26);
            this.picMap.Name = "picMap";
            this.picMap.Size = new System.Drawing.Size(522, 477);
            this.picMap.TabIndex = 50;
            this.picMap.TabStop = false;
            this.picMap.Click += new System.EventHandler(this.picMap_Click);
            // 
            // grpOnline
            // 
            this.grpOnline.Controls.Add(this.cmdCenter);
            this.grpOnline.Controls.Add(this.cmdClear);
            this.grpOnline.Controls.Add(this.optWaypoint);
            this.grpOnline.Controls.Add(this.optHome);
            this.grpOnline.Controls.Add(this.cmdSearch);
            this.grpOnline.Controls.Add(this.lblAddress);
            this.grpOnline.Controls.Add(this.txtAddress);
            this.grpOnline.Controls.Add(this.label9);
            this.grpOnline.Controls.Add(this.numericMaps);
            this.grpOnline.Controls.Add(this.RemoveRow);
            this.grpOnline.FlatStyle = System.Windows.Forms.FlatStyle.System;
            this.grpOnline.Location = new System.Drawing.Point(118, 330);
            this.grpOnline.Name = "grpOnline";
            this.grpOnline.Size = new System.Drawing.Size(345, 95);
            this.grpOnline.TabIndex = 51;
            this.grpOnline.TabStop = false;
            this.grpOnline.Text = "Paramétrage Waypoint (Mode en ligne)";
            this.grpOnline.Enter += new System.EventHandler(this.grpOnline_Enter);
            // 
            // cmdCenter
            // 
            this.cmdCenter.Enabled = false;
            this.cmdCenter.Location = new System.Drawing.Point(138, 16);
            this.cmdCenter.Name = "cmdCenter";
            this.cmdCenter.Size = new System.Drawing.Size(81, 23);
            this.cmdCenter.TabIndex = 52;
            this.cmdCenter.Text = "Centrer";
            this.cmdCenter.UseVisualStyleBackColor = true;
            this.cmdCenter.Click += new System.EventHandler(this.cmdCenter_Click);
            // 
            // cmdClear
            // 
            this.cmdClear.Enabled = false;
            this.cmdClear.Location = new System.Drawing.Point(225, 16);
            this.cmdClear.Name = "cmdClear";
            this.cmdClear.Size = new System.Drawing.Size(114, 23);
            this.cmdClear.TabIndex = 51;
            this.cmdClear.Text = "Effacer les Wp";
            this.cmdClear.UseVisualStyleBackColor = true;
            this.cmdClear.Click += new System.EventHandler(this.cmdClear_Click);
            // 
            // optWaypoint
            // 
            this.optWaypoint.AutoSize = true;
            this.optWaypoint.Location = new System.Drawing.Point(166, 43);
            this.optWaypoint.Name = "optWaypoint";
            this.optWaypoint.Size = new System.Drawing.Size(70, 17);
            this.optWaypoint.TabIndex = 50;
            this.optWaypoint.Text = "Waypoint";
            this.optWaypoint.UseVisualStyleBackColor = true;
            // 
            // optHome
            // 
            this.optHome.AutoSize = true;
            this.optHome.Checked = true;
            this.optHome.Location = new System.Drawing.Point(107, 43);
            this.optHome.Name = "optHome";
            this.optHome.Size = new System.Drawing.Size(58, 17);
            this.optHome.TabIndex = 49;
            this.optHome.TabStop = true;
            this.optHome.Text = "Terrain";
            this.optHome.UseVisualStyleBackColor = true;
            // 
            // cmdSearch
            // 
            this.cmdSearch.Location = new System.Drawing.Point(180, 65);
            this.cmdSearch.Name = "cmdSearch";
            this.cmdSearch.Size = new System.Drawing.Size(70, 23);
            this.cmdSearch.TabIndex = 48;
            this.cmdSearch.Text = "Recherche";
            this.cmdSearch.UseVisualStyleBackColor = true;
            this.cmdSearch.Click += new System.EventHandler(this.cmdSearch_Click);
            // 
            // lblAddress
            // 
            this.lblAddress.AutoSize = true;
            this.lblAddress.Location = new System.Drawing.Point(6, 47);
            this.lblAddress.Name = "lblAddress";
            this.lblAddress.Size = new System.Drawing.Size(100, 13);
            this.lblAddress.TabIndex = 47;
            this.lblAddress.Text = "Rechercher un lieu:";
            // 
            // txtAddress
            // 
            this.txtAddress.Location = new System.Drawing.Point(4, 68);
            this.txtAddress.Name = "txtAddress";
            this.txtAddress.Size = new System.Drawing.Size(170, 20);
            this.txtAddress.TabIndex = 46;
            this.txtAddress.TextChanged += new System.EventHandler(this.txtAddress_TextChanged);
            this.txtAddress.Enter += new System.EventHandler(this.txtAddress_Enter);
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(250, 43);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(87, 13);
            this.label9.TabIndex = 45;
            this.label9.Text = "Zoom de la carte";
            // 
            // numericMaps
            // 
            this.numericMaps.Location = new System.Drawing.Point(261, 65);
            this.numericMaps.Maximum = new decimal(new int[] {
            20,
            0,
            0,
            0});
            this.numericMaps.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.numericMaps.Name = "numericMaps";
            this.numericMaps.Size = new System.Drawing.Size(76, 20);
            this.numericMaps.TabIndex = 44;
            this.numericMaps.Value = new decimal(new int[] {
            15,
            0,
            0,
            0});
            this.numericMaps.ValueChanged += new System.EventHandler(this.numericMaps_ValueChanged);
            // 
            // RemoveRow
            // 
            this.RemoveRow.Enabled = false;
            this.RemoveRow.Location = new System.Drawing.Point(6, 16);
            this.RemoveRow.Name = "RemoveRow";
            this.RemoveRow.Size = new System.Drawing.Size(126, 23);
            this.RemoveRow.TabIndex = 43;
            this.RemoveRow.Text = "Retirer un Waypoint";
            this.RemoveRow.UseVisualStyleBackColor = true;
            this.RemoveRow.Click += new System.EventHandler(this.RemoveRow_Click);
            // 
            // lblOfflineLabel
            // 
            this.lblOfflineLabel.AutoSize = true;
            this.lblOfflineLabel.BackColor = System.Drawing.Color.White;
            this.lblOfflineLabel.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.lblOfflineLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblOfflineLabel.ForeColor = System.Drawing.Color.Red;
            this.lblOfflineLabel.Location = new System.Drawing.Point(753, 466);
            this.lblOfflineLabel.Name = "lblOfflineLabel";
            this.lblOfflineLabel.Size = new System.Drawing.Size(238, 18);
            this.lblOfflineLabel.TabIndex = 53;
            this.lblOfflineLabel.Text = "Image locale (Mode déconnecté)";
            this.lblOfflineLabel.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.lblOfflineLabel.Click += new System.EventHandler(this.lblOfflineLabel_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSize = true;
            this.ClientSize = new System.Drawing.Size(998, 523);
            this.Controls.Add(this.lblOfflineLabel);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.lblClickMap);
            this.Controls.Add(this.fraStatus);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.missionGrid);
            this.Controls.Add(this.mnuMain);
            this.Controls.Add(this.Launch_paramters);
            this.Controls.Add(this.picMap);
            this.Controls.Add(this.webBrowser1);
            this.Controls.Add(this.grpOnline);
            this.Controls.Add(this.grpOffline);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.Fixed3D;
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.MainMenuStrip = this.mnuMain;
            this.MaximizeBox = false;
            this.Name = "Form1";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "Planificateur de Mission";
            this.Load += new System.EventHandler(this.Form1_Load);
            ((System.ComponentModel.ISupportInitialize)(this.missionGrid)).EndInit();
            this.mnuMain.ResumeLayout(false);
            this.mnuMain.PerformLayout();
            this.Launch_paramters.ResumeLayout(false);
            this.Launch_paramters.PerformLayout();
            this.fraStatus.ResumeLayout(false);
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.grpOffline.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.picMap)).EndInit();
            this.grpOnline.ResumeLayout(false);
            this.grpOnline.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericMaps)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.DataGridView missionGrid;
        private System.Windows.Forms.TextBox launchLat;
        private System.Windows.Forms.TextBox launchLon;
        private System.Windows.Forms.CheckBox launchManually;
        private System.Windows.Forms.MenuStrip mnuMain;
        private System.Windows.Forms.ToolStripMenuItem moreToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem optionsToolStripMenuItem;
        private System.Windows.Forms.TextBox launchAlt_ref;
        private System.Windows.Forms.GroupBox Launch_paramters;
        private System.Windows.Forms.ToolStripMenuItem restartMaxAltitutdeToolStripMenuItem;
        private System.Windows.Forms.TextBox launchAlt_hold;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.WebBrowser webBrowser1;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.GroupBox fraStatus;
        private System.Windows.Forms.ProgressBar progressBar1;
        private System.Windows.Forms.Label statusLabel;
        private System.Windows.Forms.ToolStripMenuItem dIYdronescomToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem mnuFiles;
        private System.Windows.Forms.ToolStripMenuItem mnuLoadMission;
        private System.Windows.Forms.ToolStripMenuItem mnuSaveMission;
        private System.Windows.Forms.ToolStripMenuItem mnuSaveMissionAs;
        private System.Windows.Forms.ToolStripSeparator toolStripSeparator5;
        private System.Windows.Forms.ToolStripComboBox cboComSelect;
        private System.Windows.Forms.ToolStripComboBox cboBoardType;
        private System.Windows.Forms.ToolStripComboBox cboMetersFeet;
        private System.Windows.Forms.ToolStripSeparator toolStripSeparator6;
        private System.Windows.Forms.ToolStripMenuItem mnuRead;
        private System.Windows.Forms.ToolStripMenuItem mnuWrite;
        private System.Windows.Forms.ToolStripSeparator toolStripSeparator7;
        private System.Windows.Forms.ToolStripMenuItem mnuExit;
        private System.Windows.Forms.CheckBox chkLookupAlt;
        private System.Windows.Forms.Button cmdWrite;
        private System.Windows.Forms.Button cmdRead;
        private System.Windows.Forms.Label lblClickMap;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox radius;
        private System.Windows.Forms.Label lblHomeAltLabel;
        private System.Windows.Forms.Label lblHomeAlt;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label maxSpd;
        private System.Windows.Forms.Label maxAlt;
        private System.Windows.Forms.Label wp_number;
        private System.Windows.Forms.GroupBox grpOffline;
        private System.Windows.Forms.Button cmdAddOffWaypoint;
        private System.Windows.Forms.Button cmdClearOffWaypoints;
        private System.Windows.Forms.Button cmdRemoveOffWaypoint;
        private System.Windows.Forms.PictureBox picMap;
        private System.Windows.Forms.GroupBox grpOnline;
        private System.Windows.Forms.Button cmdCenter;
        private System.Windows.Forms.Button cmdClear;
        private System.Windows.Forms.RadioButton optWaypoint;
        private System.Windows.Forms.RadioButton optHome;
        private System.Windows.Forms.Button cmdSearch;
        private System.Windows.Forms.Label lblAddress;
        private System.Windows.Forms.TextBox txtAddress;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.NumericUpDown numericMaps;
        private System.Windows.Forms.Button RemoveRow;
        private System.Windows.Forms.Label lblOfflineLabel;
        private System.Windows.Forms.DataGridViewTextBoxColumn Number;
        private System.Windows.Forms.DataGridViewTextBoxColumn Lat;
        private System.Windows.Forms.DataGridViewTextBoxColumn Lon;
        private System.Windows.Forms.DataGridViewTextBoxColumn Altitude;
        private System.Windows.Forms.DataGridViewTextBoxColumn Hidden_Alt;
        private System.Windows.Forms.DataGridViewTextBoxColumn SeaLevelAlt;
    }
}

