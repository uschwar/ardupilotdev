using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.Threading;
using Utility.ModifyRegistry;
using System.IO;
using System.Text.RegularExpressions;
using System.Net;


namespace ArduPilotConfigTool
{
    public partial class Form1 : Form
    {
        portControl portControl1 = new portControl();

        options_form options_form = new options_form();
        

        //private options_form test; 
        bool bStartup;
        Double nMult = 1;
        double nAltHold = 150;
        double nRadius = 60;
        double nHomeAlt;
        int nLastMetersFeet;
        string sLastLatLng;
        int g_MaxWaypoints = 48;
        string sLastLoaded;
        bool bOnlineMode = true;
        bool bCompatibleMode = true;
        bool bOverTerrainMode = false;
        ModifyRegistry myRegistry = new ModifyRegistry();
        string separatorFormat = System.Globalization.CultureInfo.CurrentCulture.NumberFormat.NumberDecimalSeparator;

        //string separatorFormat = new System.Globalization.NumberFormatInfo().NumberDecimalSeparator;
        //separatorFormat = ",";

        public Form1()
        {
            bStartup = true;
            //Declarando un objecto que creara un evento en la forma options_form...
            options_form.buttonn_change += new EventHandler(options_form_buttonn_change);
            portControl1.write_succesfull +=new EventHandler(portControl1_write_succesfull);
            portControl1.read_succesfull +=new EventHandler(portControl1_read_succesfull);
            portControl1.write_wrong += new EventHandler(portControl1_write_wrong);
            portControl1.read_wrong += new EventHandler(portControl1_read_wrong);

            InitializeComponent();

            System.IO.Directory.CreateDirectory(Application.StartupPath + "\\Missions");

            bOnlineMode = true;
            if (checkOnlineMode() == true)
            {
                webBrowser1.Navigate(new Uri(System.AppDomain.CurrentDomain.BaseDirectory + "Maps.html"));
                {
                    do
                    {
                        Application.DoEvents();
                    } while (webBrowser1.ReadyState != WebBrowserReadyState.Complete);
                }
            }
            if (checkGCompatibleMode() == false)
            {
                webBrowser1.Navigate("http://maps.google.com/support/bin/topic.py?topic=10781");
            }

            myRegistry.SubKey = "SOFTWARE\\ArduPilotConfigTool\\Settings";

            string sComPort;
            sComPort = myRegistry.Read("ComPort", "");
            cboComSelect.Items.Clear(); //Leyendo puertos disponibles
            cboComSelect.Items.Add("Select COM Port");
            foreach (string x in portControl1.update_available_ports())
            {
                cboComSelect.Items.Add(x);
                if (x == sComPort)
                {
                    cboComSelect.SelectedIndex = cboComSelect.Items.Count - 1;
                }
            }
            if (cboComSelect.SelectedIndex < 0)
            {
                cboComSelect.SelectedIndex = 0;
            }

            txtAddress.Text = myRegistry.Read("SearchAddress","");
            if (myRegistry.Read("SearchType","true") == "false")
            {
                optWaypoint.Checked = true;
            }
            else
            {
                optHome.Checked = true;
            }


            nAltHold = Convert.ToDouble(myRegistry.Read("AltHold", "150"));
            nRadius = Convert.ToDouble(myRegistry.Read("Radius", "60"));

            cboBoardType.SelectedIndex = Convert.ToInt32(myRegistry.Read("BoardType", "1"));
            cboBoardType_SelectedIndexChanged(null, null);
            cboMetersFeet.SelectedIndex = Convert.ToInt32(myRegistry.Read("FeetMeters", "0"));
            nLastMetersFeet = cboMetersFeet.SelectedIndex;
            cboMetersFeet_SelectedIndexChanged(null, null);
            chkLookupAlt.Checked = Convert.ToBoolean(myRegistry.Read("OverTerrain", "False"));
            launchManually.Checked = Convert.ToBoolean(myRegistry.Read("SetManual", "False"));

            Version vrs = new Version(Application.ProductVersion);
            this.Text = this.Text + " v" + vrs.Major + "." + vrs.Minor + "." + vrs.Build;

            launchAlt_hold.Text = string.Format("{0:n1}", nAltHold * nMult);
            radius.Text = string.Format("{0:n1}",nRadius * nMult);

            if (bOnlineMode == true && bCompatibleMode == true)
            {
                double nLat;
                double nLng;
                double.TryParse(myRegistry.Read("HomeLat", "48.334286"), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nLat);
                double.TryParse(myRegistry.Read("HomeLong", "2.873869"), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nLng);

                launchLat.Text = Convert.ToString(nLat);
                launchLon.Text = Convert.ToString(nLng);
                setGoogleMapHome(launchLat.Text, launchLon.Text, true);

                webBrowser1.Document.GetElementById("homeLat").SetAttribute("value", Regex.Replace(launchLat.Text, ",", "."));
                webBrowser1.Document.GetElementById("homeLng").SetAttribute("value", Regex.Replace(launchLon.Text, ",", "."));
                webBrowser1.Document.GetElementById("centerMapHomeButton").InvokeMember("click");
                webBrowser1.Document.GetElementById("lat").SetAttribute("value", Regex.Replace(launchLat.Text, ",", "."));
                webBrowser1.Document.GetElementById("lng").SetAttribute("value", Regex.Replace(launchLon.Text, ",", "."));
                webBrowser1.Document.GetElementById("index").SetAttribute("value", "Home");
            }
            //webBrowser1.Document.GetElementById("homeLat").SetAttribute("value", launchLat.Text);
            //webBrowser1.Document.GetElementById("homeLng").SetAttribute("value", launchLon.Text);
            //webBrowser1.Document.GetElementById("setHomeLatLngButton").InvokeMember("click");

            //changeOnlineMode(bOnlineMode);
            bStartup = false;
            timer1.Enabled = true;
        }

        /************************************************************/
        private void Form1_Load(object sender, EventArgs e)
        {
        }
        /************************************************************/
        private void add_data_to_grind()
        {
            //Agregando una linea
            missionGrid.Rows.Add(1);
            //Assignamos un numero a la linea
            missionGrid[0, missionGrid.Rows.Count - 1].Value = Convert.ToString(missionGrid.Rows.Count);
            //Cambia la equica con el numero de waypoints

            //wp_number.Text = Convert.ToString(missionGrid.Rows.Count); 
            portControl1.number_Waypoints = Convert.ToByte(missionGrid.Rows.Count);
            //Enviamos el numero de waypoints a la variable... 
           
        }

        /************************************************************/
        /*Launch Home manual, Activando o desactivando casillas*/
        private void launchManually_CheckedChanged(object sender, EventArgs e)
        {
            //sbyte temp = 0;

                myRegistry.Write("SetManual", Convert.ToString(launchManually.Checked));
                launchLat.Enabled = launchManually.Checked;
                launchLon.Enabled = launchManually.Checked;
                if (launchManually.Checked == true)
                {
                    portControl1.options |= 0x08; 
                }
                else
                {
                    portControl1.options &= 0xF7;
                }
        }
        /************************************************************/
        //private void AddRow_Click(object sender, EventArgs e)
        //{
        //    add_data_to_grind();
        //    portControl1.number_Waypoints = Convert.ToByte(missionGrid.Rows.Count);
        //    missionGrid[1, missionGrid.Rows.Count - 1].Value = launchLat.Text;
        //    missionGrid[2, missionGrid.Rows.Count - 1].Value = launchLon.Text;
        //    missionGrid[3, missionGrid.Rows.Count - 1].Value = string.Format("{0:n0}", Convert.ToDouble(launchAlt_hold.Text) * nMult);
        //    missionGrid[4, missionGrid.Rows.Count - 1].Value = string.Format("{0:n0}", nAltHold);
        //}
        /************************************************************/
        /************************************************************/
        private void downloadToolStripMenuItem_Click(object sender, EventArgs e)
        {
        }
        /************************************************************/
        /**Esta funcion sirve para subir los datos al ardupilot***/
        private void uploadToolStripMenuItem_Click(object sender, EventArgs e)
        {
        }
        /************************************************************/

        //Options menu...
        private void optionsToolStripMenuItem_Click(object sender, EventArgs e)
        {
            options_form.Show();
            //options_form.reverse_roll = (portControl1.Options&(byte)0x01)==0x01;
            //options_form.reverse_elevator = (portControl1.Options & (byte)0x02) == 0x02;
            //options_form.sernsor_z = (portControl1.Options & (byte)0x04) == 0x04;
        }

        private void options_form_buttonn_change(object sender, EventArgs e)
        {
            options_form.Hide();

            /************Reverse Roll************************/
            if (options_form.reverse_roll == true)
                portControl1.options |= 0x01;
            else
                portControl1.options &= 0xFE;
            /************Reverse Elevator********************/
            if (options_form.reverse_elevator == true)
                portControl1.options |= 0x02;
            else
                portControl1.options &= 0xFD;
            /************Sensor z********************************/
            if (options_form.sernsor_z == true)
                portControl1.options |= 0x04;
            else
                portControl1.options &= 0xFB;
            /************tx states********************************/
            if (options_form.txstates == true)
                portControl1.options |= 0x10;
            else
                portControl1.options &= 0xEF;
            /************action********************************/
            if (options_form.txdefault == true)
                portControl1.options |= 0x20;
            else
                portControl1.options &= 0xDF;
            /************Updating trims*******************************/
            portControl1.rtrim = (byte)options_form.trim_r;
            portControl1.ptrim = (byte) options_form.trim_p;

        }

        private void progressBar1_Click(object sender, EventArgs e)
        {
            
        }

        private void restartMaxAltitutdeToolStripMenuItem_Click(object sender, EventArgs e)
        {
            DialogResult result;
            result = MessageBox.Show("Etes-vous sur?", "Reset alt/vit Max", MessageBoxButtons.YesNo, MessageBoxIcon.Question);
            if (result == System.Windows.Forms.DialogResult.Yes)
            {
                portControl1.clear_max_altspd();
                ardupilot_write();

                ardupilot_read();
            }
        }

        void ardupilot_read()
        {
            string sError="";
            double nConversion;

            progressBar1.Value = 0;
            if (bOnlineMode == true && bCompatibleMode == true)
            {
                cmdClear_Click(null,null);
            }
            statusLabel.Text = "Lecture des données...";
            progressBar1.Maximum = 120;
            progressBar1.Value = 10;
            this.Refresh();
            //try
           // {
            if (cboMetersFeet.Text == "Feet")
            {
                nConversion = 3.2808399;
            }
            else
            {
                nConversion = 1;
            }
 
               //for (int x = 0; x < 100; x+=1)
               // {
               //     progressBar1.Value = x;
               //     //Thread.Sleep(30);
               //     progressBar1.Refresh();
               // }
                if (Convert.ToString(cboBoardType.SelectedItem) == "AT328")
                {
                    portControl1.set_port(Convert.ToString(cboComSelect.SelectedItem), 57600, ref sError);
                }
                else
                {
                    portControl1.set_port(Convert.ToString(cboComSelect.SelectedItem), 19200, ref sError);
                }
                if (sError != "")
                {
                    statusLabel.Text = sError;
                    return;
                }
                else
                {
                    portControl1.download_Ardupilot();
                }
            /*}
            catch (Exception er)
            {
                MessageBox.Show(Convert.ToString(er.Message));
            }*/


                /*Now Updating data*/
                options_form.reverse_roll = (portControl1.options & (byte)0x01) == 0x01;
                options_form.reverse_elevator = (portControl1.options & (byte)0x02) == 0x02;
                options_form.sernsor_z = (portControl1.options & (byte)0x04) == 0x04;
                launchManually.Checked = (portControl1.options & (byte)0x08) == 0x08;
                options_form.txstates = (portControl1.options & (byte)0x10) == 0x10;
                options_form.txdefault = (portControl1.options & (byte)0x20) == 0x20;

                options_form.trim_r = (sbyte)portControl1.rtrim;
                options_form.trim_p = (sbyte)portControl1.ptrim;

                maxAlt.Text = Convert.ToString(portControl1.maxalt);
                maxSpd.Text = Convert.ToString(portControl1.maxspd);

                radius.Text = string.Format("{0:n0}", portControl1.Radius * nConversion);

                double nLat;
                double nLng;

                progressBar1.Value = progressBar1.Value + 10;

                double.TryParse(Convert.ToString((double)portControl1.launch_Latitude / 1000000), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nLat);
                double.TryParse(Convert.ToString((double)portControl1.launch_Longitude / 1000000), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nLng);

                if (nLat != 0)
                {
                    launchLat.Text = nLat.ToString();
                }
                if (nLng != 0)
                {
                    launchLon.Text = nLng.ToString();
                }
                launchLat_Leave(null,null);
                launchLon_Leave(null, null);

                launchAlt_ref.Text = Convert.ToString(portControl1.launch_Altitude_ref);
                launchAlt_hold.Text = string.Format("{0:n2}", portControl1.launch_Altitude_hold * nConversion);

                missionGrid.Rows.Clear();
                for (int x = 0; x < portControl1.number_Waypoints; x++)
                {
                    progressBar1.Value = progressBar1.Value + 100 / portControl1.number_Waypoints;

                    nLat = (double)portControl1.get_mission_lat(x) / 1000000;
                    nLng = (double)portControl1.get_mission_lon(x) / 1000000;
                    lblClickMap.Visible = false;
                    missionGrid.Rows.Add();
                    missionGrid[0, x].Value = Convert.ToString(x+1);
                    missionGrid[1, x].Value = Convert.ToString((double)portControl1.get_mission_lat(x) / 1000000);
                    missionGrid[2, x].Value = Convert.ToString((double)portControl1.get_mission_lon(x) / 1000000);
                    missionGrid[5, x].Value = Convert.ToString(getAltitudeData(nLat,nLng));
                    if (chkLookupAlt.Checked == true)
                    {
                        missionGrid[3, x].Value = string.Format("{0:n2}", portControl1.get_mission_alt(x) * nConversion );
                        missionGrid[4, x].Value = string.Format("{0:n2}", (nHomeAlt - Convert.ToDouble(missionGrid[5, x].Value) + Convert.ToDouble(missionGrid[3, x].Value)));
                    }
                    else
                    {
                        missionGrid[4, x].Value = string.Format("{0:n2}", portControl1.get_mission_alt(x) * nConversion);
                        missionGrid[3, x].Value = string.Format("{0:n2}", Convert.ToDouble(missionGrid[4, x].Value));
                    }

                    setWaypointValues(missionGrid.Rows.Count, missionGrid[1, x].Value.ToString(), missionGrid[2, x].Value.ToString(), true);
                }
                if (bOnlineMode == true && bCompatibleMode == true)
                {
                    webBrowser1.Document.GetElementById("refreshButton").InvokeMember("click");
                }
                progressBar1.Value = progressBar1.Maximum;

        }

        private void setWaypointValues(object index, string nlat, string nlng, Boolean addWaypointClick)
        {
            if (bOnlineMode == true && bCompatibleMode == true)
            {
                webBrowser1.Document.GetElementById("wpIndex").SetAttribute("value", index.ToString());
                webBrowser1.Document.GetElementById("wpLat").SetAttribute("value", Regex.Replace(nlat, ",", "."));
                webBrowser1.Document.GetElementById("wpLng").SetAttribute("value", Regex.Replace(nlng, ",", "."));
                if (addWaypointClick == true)
                {
                    webBrowser1.Document.GetElementById("addWaypointLatLngButton").InvokeMember("click");
                }
                //Application.DoEvents();
            }
        }

        private void portControl1_read_succesfull(object sender, EventArgs e)
        {
            statusLabel.Text = "Lecture réussie!";
            //update_maps();
            this.Refresh();
            //webBrowser1.Document.GetElementById("homeLat").SetAttribute("value", latLngAlt[0]);
            //webBrowser1.Document.GetElementById("homeLng").SetAttribute("value", latLngAlt[1]);
            //webBrowser1.Document.GetElementById("setHomeLatLngButton").InvokeMember("click");
            //webBrowser1.Document.GetElementById("centerMapHomeButton").InvokeMember("click");
            //nAltHold = Convert.ToDouble(latLngAlt[2]);
            nHomeAlt = getAltitudeData(Convert.ToDouble(launchLat.Text), Convert.ToDouble(launchLon.Text));
            lblHomeAlt.Text = string.Format("{0:n0}", nHomeAlt);
            //launchAlt_hold.Text = Convert.ToString(nAltHold * nMult);
            //launchLat.Text = Convert.ToString(nLat);
            //launchLon.Text = Convert.ToString(nLng);
            
            //webBrowser1.Document.GetElementById("wpIndex").SetAttribute("value", Convert.ToString(missionGrid.Rows.Count));
            //webBrowser1.Document.GetElementById("wpLat").SetAttribute("value", latLngAlt[0]);
            //webBrowser1.Document.GetElementById("wpLng").SetAttribute("value", latLngAlt[1]);
            //webBrowser1.Document.GetElementById("addWaypointLatLngButton").InvokeMember("click");

        
        }

        private void portControl1_read_wrong(object sender, EventArgs e)
        {
            statusLabel.Text = "Lecture impossible!";
            this.Refresh();
        }
        
        void ardupilot_write()
        {
            double nConversion;
            string sError = "";

            progressBar1.Value = 0;
            statusLabel.Text = "Ecriture des données...";
            this.Refresh();

            if (cboMetersFeet.Text == "Pieds")
            {
                nConversion = 3.2808399;
            }
            else
            {
                nConversion = 1;
            }
            try
            {
                myRegistry.Write("LastSaved", sLastLoaded);

                //COnverting general vairables
                portControl1.Radius = Convert.ToByte(Convert.ToDouble(radius.Text) / nConversion);
                portControl1.launch_Altitude_hold = Convert.ToInt16(Convert.ToDouble(launchAlt_hold.Text) / nConversion);

                //Verfiying if we going to setup home position manually
                if (launchManually.Checked)
                {
                    portControl1.launch_Latitude = (int)(Convert.ToDouble(launchLat.Text) * (double)1000000);
                    portControl1.launch_Longitude = (int)(Convert.ToDouble(launchLon.Text) * (double)1000000);
                    portControl1.options |= 0x08;
                }
                else
                    portControl1.options &= 0xF7;


                //Checking how many mission we have.. 
                portControl1.number_Waypoints = (byte)missionGrid.Rows.Count;

                //Extracting missions.... 
                progressBar1.Minimum = 0;
                progressBar1.Maximum = progressBar1.Value + Convert.ToInt32(portControl1.number_Waypoints)+20;
                for (int x = 0; x < portControl1.number_Waypoints; x++)
                {
                    progressBar1.Value = progressBar1.Value + 1;
                    portControl1.set_mission_lat(x, Convert.ToInt32((Convert.ToDouble(missionGrid[1, x].Value))*(double)1000000));
                    portControl1.set_mission_lon(x, Convert.ToInt32((Convert.ToDouble(missionGrid[2, x].Value))*(double)1000000));
                    portControl1.set_mission_alt(x, Convert.ToInt32(Convert.ToDouble(missionGrid[3, x].Value) / nConversion));
                    //progressBar1.Value = 10 + (x * 2);
                    progressBar1.Refresh();
                }

                //for (int x = 0; x < 100; x += 2)
                //{
                //    progressBar1.Value = progressBar1.Value + 1;
                //    Thread.Sleep(25);
                //}

                
                if (Convert.ToString(cboBoardType.SelectedItem) == "AT328")
                {
                    portControl1.set_port(Convert.ToString(cboComSelect.SelectedItem), 57600, ref sError);
                }
                else
                {
                    portControl1.set_port(Convert.ToString(cboComSelect.SelectedItem), 19200, ref sError);
                }
                progressBar1.Value = progressBar1.Value + 10;
                if (sError != "")
                {
                    statusLabel.Text = sError;
                }
                else
                {
                    portControl1.upload_Ardupilot(ref sError);
                    progressBar1.Value = progressBar1.Maximum;
                }
            }
            catch (Exception er)
            {
                //MessageBox.Show(Convert.ToString(er.Message));
                statusLabel.Text = Convert.ToString(er.Message);
            }
        }

        private double getAltitudeData(double latitude,double longitude)
        {
            try
            {
                if (bOverTerrainMode == false)
                {
                    return 0;
                }

                double nDoubleReturn;
                // Create a 'WebRequest' object with the specified url. 
                WebRequest myWebRequest = WebRequest.Create("http://gisdata.usgs.gov/XMLWebServices2/Elevation_Service.asmx/getElevation?X_Value=" + Regex.Replace(longitude.ToString(), ",", ".") + "&Y_Value=" + Regex.Replace(latitude.ToString(), ",", ".") + "&Elevation_Units=FEET&Source_Layer=-1&Elevation_Only=True");

                // Send the 'WebRequest' and wait for response. 
                WebResponse myWebResponse = myWebRequest.GetResponse();

                setOnlineMode(true);

                // Obtain a 'Stream' object associated with the response object. 
                Stream ReceiveStream = myWebResponse.GetResponseStream();

                Encoding encode = System.Text.Encoding.GetEncoding("utf-8");

                // Pipe the stream to a higher level stream reader with the required encoding format. 
                StreamReader readStream = new StreamReader(ReceiveStream, encode);

                string strResponse = readStream.ReadToEnd();

                //System.Diagnostics.Debug.WriteLine(strResponse);
                //Console.WriteLine( strResponse);

                readStream.Close();

                // Release the resources of response object. 
                myWebResponse.Close();

                strResponse = strResponse.Substring(strResponse.IndexOf(">") + 1);
                strResponse = strResponse.Substring(strResponse.IndexOf(">") + 1);
                strResponse = strResponse.Substring(0, strResponse.IndexOf("<"));
                //System.Diagnostics.Debug.WriteLine(strResponse);
                double.TryParse(strResponse, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nDoubleReturn);

                return Convert.ToDouble(string.Format("{0:n2}", nDoubleReturn * nMult));
            }
            catch
            {
                setOnlineMode(false);
                return 0;
            }
        }

        private void setOnlineMode(Boolean isOnline)
        {
            if (isOnline == true && bOverTerrainMode == true)
            {
                missionGrid.Columns[1].Width = 65;
                missionGrid.Columns[2].Width = 65;
                missionGrid.Columns[3].Width = 60;
                missionGrid.Columns[3].ReadOnly = true;
                missionGrid.Columns[3].HeaderText = "Alt vol  (QNH)";
                missionGrid.Columns[3].DefaultCellStyle.Alignment = DataGridViewContentAlignment.MiddleCenter;
                missionGrid.Columns[4].Visible = true;
                missionGrid.Columns[5].Visible = true;
                lblHomeAltLabel.Visible = true;
                lblHomeAlt.Visible = true;
            }
            else
            {
                missionGrid.Columns[1].Width = 100;
                missionGrid.Columns[2].Width = 100;
                missionGrid.Columns[3].Width = 100;
                missionGrid.Columns[3].HeaderText = "Altitude QFE";
                missionGrid.Columns[3].DefaultCellStyle.Alignment = DataGridViewContentAlignment.MiddleLeft;
                missionGrid.Columns[3].ReadOnly = false;
                missionGrid.Columns[4].Visible = false;
                missionGrid.Columns[5].Visible = false;
                lblHomeAltLabel.Visible = false;
                lblHomeAlt.Visible = false;
            }
            if (isOnline == false)
            {
                bOnlineMode = false;
            }
            grpOnline.Visible = isOnline;
            webBrowser1.Visible = isOnline;

            grpOffline.Visible = !isOnline;
            picMap.Visible = !isOnline;

            if (missionGrid.Rows.Count == 0)
            {
                lblClickMap.Visible = isOnline;
                lblOfflineLabel.Visible = !isOnline;
            }
            else
            {
                lblClickMap.Visible = false;
                lblOfflineLabel.Visible = !isOnline;
            }
        }

        private Boolean checkOnlineMode()
        {
            try
            {
                WebRequest myWebRequest = WebRequest.Create("http://www.google.com");
                WebResponse myWebResponse = myWebRequest.GetResponse();
                setOnlineMode(true);
                return true;
            }
            catch
            {
                setOnlineMode(false);
                return false;
            }
        }

        private Boolean checkGCompatibleMode()
        {
            try
            {
                if (webBrowser1.Document.GetElementById("status").InnerHtml == "Not Compatible")
                {
                    bCompatibleMode = false;
                    return false;
                }
                else
                    bCompatibleMode = true;
                    return true;
            }
            catch
            {
                bCompatibleMode = false;
                return false;
            }
        }

        private void portControl1_write_succesfull(object sender, EventArgs e)
        {
            statusLabel.Text = "Ecriture réussie!";
            this.Refresh();
        }

        private void portControl1_write_wrong(object sender, EventArgs e)
        {
            statusLabel.Text = "Quelque chose n'a pas marché!!!";
            this.Refresh();
        }


        private void numericMaps_ValueChanged(object sender, EventArgs e)
        {
            if (bOnlineMode == true)
            {
                if (bStartup == false)
                {
                    webBrowser1.Document.GetElementById("zoomLevel").SetAttribute("value", Convert.ToString(numericMaps.Value));
                    webBrowser1.Document.GetElementById("setZoomButton").InvokeMember("click");
                }
            }
        }

        //void update_maps()
        //{
        //    string mapkey = "key=ABQIAAAAH46Q8C1zHBd5VTkIsOdp0hRvp662xBiIsv0PCKJ9NqEpqiVFAxR7YfzagPM27B9Rkf5IJMCLYhCO0A";
        //    string url;
        //    string lat = launchLat.Text;
        //    string lon = launchLon.Text;
        //    string makers = "";
        //    string path = "&path=rgba:0xff0000ff,weight:2%7C";

        //    try
        //    {
        //        //Creating home position icon. 
        //        makers = "markers=" + lat + "," + lon + ",midgreenh%7C";
        //        path += lat + "," + lon + "%7C";
        //        if (portControl1.number_Waypoints > 0)
        //        {
        //            for (int x = 0; x < portControl1.number_Waypoints; x++)
        //            {
        //                makers += missionGrid[1, x].Value + "," + missionGrid[2, x].Value + ",midred" +
        //                    Convert.ToString(x+1) + "%7C";
        //                path += missionGrid[1, x].Value + "," + missionGrid[2, x].Value + "%7C";
        //            }
        //            path += "&";
        //        }
        //        else
        //        {
        //            path = "";
        //        }
        //        makers += "&";

        //    url = "http://www.maps.google.com/staticmap?center=" + lat + "," + lon + path+
        //        "&format=jpg&zoom=" + Convert.ToString(numericMaps.Value) + "&size=400x300&maptype=" 
        //        + mapType.Text + "&" + makers + mapkey;
            
        //    //System.Diagnostics.Process.Start(url);
        //    webBrowser1.Navigate(new Uri(url));
        //    }
        //    catch (Exception er)
        //    {
        //        //MessageBox.Show(Convert.ToString(er.Message));
        //        statusLabel.Text = Convert.ToString(er.Message);
        //    }
        //    do
        //    {
        //        Application.DoEvents();
        //    } while (webBrowser1.ReadyState != WebBrowserReadyState.Complete);
        //}

        //private void button_updatesMaps_Click(object sender, EventArgs e)
        //{
        //    update_maps();
        //}

        //private void mapType_SelectedIndexChanged(object sender, EventArgs e)
        //{
        //    update_maps();
        //}

        private void linkLabel1_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
        }

        private void updateAltColumns()
        {
            for (int nCount = 0; nCount < missionGrid.Rows.Count; nCount++)
            {
                if (chkLookupAlt.Checked == true)
                {
                    missionGrid[5, nCount].Value = string.Format("{0:n2}", getAltitudeData(Convert.ToDouble(missionGrid[1, nCount].Value),Convert.ToDouble(missionGrid[2, nCount].Value)));
                    missionGrid[3, nCount].Value = string.Format("{0:n2}", (Convert.ToDouble(missionGrid[4, nCount].Value) + Convert.ToDouble(missionGrid[5, nCount].Value) - nHomeAlt));
                }
                else
                {
                    missionGrid[3, nCount].Value = string.Format("{0:n2}", Convert.ToDouble(missionGrid[4, nCount].Value));
                }
            }
        }

        private void setGoogleMapHome(string latitude, string longitude, Boolean centerMap)
        {
            if (bOnlineMode == true && bCompatibleMode == true)
            {
                webBrowser1.Document.GetElementById("homeLat").SetAttribute("value", Regex.Replace(latitude, ",", "."));
                webBrowser1.Document.GetElementById("homeLng").SetAttribute("value", Regex.Replace(longitude, ",", "."));
                webBrowser1.Document.GetElementById("setHomeLatLngButton").InvokeMember("click");
                if (centerMap == true)
                {
                    webBrowser1.Document.GetElementById("centerMapHomeButton").InvokeMember("click");
                }
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            int waypointIndex;
            string sWaypointIndex;
            string waypointLat;
            string waypointLong;
            string sZoomLevel = "15";
            double nWaypointLat;
            double nWaypointLong;
            double nCurrentAlt;

            if (bOnlineMode == false || bCompatibleMode == false)
            {
                return;
            }

            do
            {
                Application.DoEvents();
            } while (webBrowser1.ReadyState != WebBrowserReadyState.Complete);

            bStartup = true;
            try
            {
                sZoomLevel = webBrowser1.Document.GetElementById("zoomLevel").GetAttribute("value");
                sWaypointIndex = webBrowser1.Document.GetElementById("index").GetAttribute("value");
                waypointLat = webBrowser1.Document.GetElementById("lat").GetAttribute("value");
                waypointLong = webBrowser1.Document.GetElementById("lng").GetAttribute("value");

                waypointIndex = -1;

                double.TryParse(waypointLat, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nWaypointLat);
                double.TryParse(waypointLong, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nWaypointLong);

                if (txtAddress.Text.Trim() == "")
                {
                    cmdSearch.Enabled = false;
                }
                else
                {
                    cmdSearch.Enabled = true;
                }

                if (sLastLatLng != waypointLat + "," + waypointLong)
                {
                    sLastLatLng = waypointLat + "," + waypointLong;
                    if (sWaypointIndex == "Home")
                    {
                        //bStartup = false;
                        launchLat.Text = string.Format("{0:n6}", nWaypointLat);
                        launchLon.Text = string.Format("{0:n6}", nWaypointLong);

                        //if (missionGrid.Rows.Count > 0 || nHomeAlt == 0)
                        //{
                        nHomeAlt = getAltitudeData(nWaypointLat, nWaypointLong);
                        lblHomeAlt.Text = string.Format("{0:n0}", nHomeAlt);
                        //System.Diagnostics.Debug.WriteLine(nHomeAlt);
                        //}

                        updateAltColumns();
                        //launchLat.Text = waypointLat;
                        //launchLon.Text = waypointLong;
                        bStartup = true;
                    }
                    else
                    {
                        waypointIndex = Convert.ToInt32(sWaypointIndex);
                        if (waypointIndex > g_MaxWaypoints)
                        {
                            MessageBox.Show("Too many waypoints selected.\nArduPilot has 1K of EEPROM memory and currently can handle up to " + g_MaxWaypoints.ToString() + " waypoints.", "Max waypoints", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                            webBrowser1.Document.GetElementById("dwIndex").SetAttribute("value", waypointIndex.ToString());
                            webBrowser1.Document.GetElementById("deleteWaypointButton").InvokeMember("click");
                            return;
                        }
                    }

                    double nDownloadedAlt;

                    if (waypointIndex > missionGrid.Rows.Count)
                    {
                        nDownloadedAlt = getAltitudeData(nWaypointLat, nWaypointLong);

                        missionGrid.Rows.Add(1);
                        missionGrid[0, missionGrid.Rows.Count - 1].Value = Convert.ToString(missionGrid.Rows.Count);
                        missionGrid[1, missionGrid.Rows.Count - 1].Value = string.Format("{0:n6}", nWaypointLat);
                        missionGrid[2, missionGrid.Rows.Count - 1].Value = string.Format("{0:n6}", nWaypointLong);
                        if (chkLookupAlt.Checked == true)
                        {
                            missionGrid[3, missionGrid.Rows.Count - 1].Value = string.Format("{0:n2}", (nDownloadedAlt + nAltHold * nMult - nHomeAlt));
                        }
                        else
                        {
                            missionGrid[3, missionGrid.Rows.Count - 1].Value = string.Format("{0:n2}", nAltHold * nMult);
                        }
                        missionGrid[4, missionGrid.Rows.Count - 1].Value = string.Format("{0:n2}", nAltHold * nMult);
                        missionGrid[5, missionGrid.Rows.Count - 1].Value = string.Format("{0:n2}", nDownloadedAlt);
                        //wp_number.Text = Convert.ToString(missionGrid.Rows.Count);
                        missionGrid.CurrentCell.Selected = false;
                        missionGrid.Rows[missionGrid.Rows.Count - 1].Cells[0].Selected = true;
                        missionGrid.CurrentCell = missionGrid.SelectedCells[0];
                    }
                    else if (waypointIndex != -1)
                    {
                        nDownloadedAlt = getAltitudeData(nWaypointLat, nWaypointLong);
                        nCurrentAlt = Convert.ToDouble(missionGrid[4, missionGrid.Rows.Count - 1].Value);

                        missionGrid[1, waypointIndex - 1].Value = string.Format("{0:n6}", nWaypointLat);
                        missionGrid[2, waypointIndex - 1].Value = string.Format("{0:n6}", nWaypointLong);
                        if (chkLookupAlt.Checked == true)
                        {
                            missionGrid[3, waypointIndex - 1].Value = string.Format("{0:n2}", (nDownloadedAlt + nCurrentAlt - nHomeAlt));
                        }
                        else
                        {
                            missionGrid[3, waypointIndex - 1].Value = string.Format("{0:n2}", nCurrentAlt);
                        }
                        missionGrid[4, waypointIndex - 1].Value = string.Format("{0:n2}", nCurrentAlt);
                        missionGrid[5, waypointIndex - 1].Value = string.Format("{0:n2}", nDownloadedAlt);
                        missionGrid.CurrentCell.Selected = false;
                        missionGrid.Rows[waypointIndex - 1].Cells[0].Selected = true;
                        missionGrid.CurrentCell = missionGrid.SelectedCells[0];
                    }
                    portControl1.number_Waypoints = Convert.ToByte(missionGrid.Rows.Count);
                }
                if (sZoomLevel != "")
                {
                    numericMaps.Value = int.Parse(sZoomLevel);
                }
            }
            catch (Exception)
            {
            }
            if (missionGrid.Rows.Count > 0)
            {
                RemoveRow.Enabled = true;
                cmdClear.Enabled = true;
                cmdCenter.Enabled = true;

                cmdRemoveOffWaypoint.Enabled = true;
                cmdClearOffWaypoints.Enabled = true;

                lblClickMap.Visible = false;
            }
            else
            {
                RemoveRow.Enabled = false;
                cmdClear.Enabled = false;
                cmdCenter.Enabled = false;

                cmdRemoveOffWaypoint.Enabled = false;
                cmdClearOffWaypoints.Enabled = false;

                if (bOnlineMode == true && bCompatibleMode == true)
                {
                    lblClickMap.Text = "Cliquez sur la carte pour ajouter un Wp";
                }
                else
                {
                    lblClickMap.Text = "Click pour ajouter un Wp";
                }
                lblClickMap.Visible = true;
            }
            bStartup = false;
        }

        private void optLocation_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void cmdSearch_Click(object sender, EventArgs e)
        {
            if (optHome.Checked == true)
            {
                webBrowser1.Document.GetElementById("address").SetAttribute("value", txtAddress.Text);
                webBrowser1.Document.GetElementById("setHomeAddressButton").InvokeMember("click");
                webBrowser1.Document.GetElementById("centerMapHomeButton").InvokeMember("click");
            }
            else
            {
                webBrowser1.Document.GetElementById("wpAddress").SetAttribute("value", txtAddress.Text);
                webBrowser1.Document.GetElementById("addWaypointButton").InvokeMember("click");
                webBrowser1.Document.GetElementById("centerMapWaypointButton").InvokeMember("click");
            }
            myRegistry.Write("SearchAddress", txtAddress.Text);
            myRegistry.Write("SearchType", Convert.ToString(optHome.Checked));
        }

        private void cmdClear_Click(object sender, EventArgs e)
        {
            webBrowser1.Document.GetElementById("ClearButton").InvokeMember("click");
            missionGrid.Rows.Clear();
            portControl1.number_Waypoints = 0;
        }

        private void txtAddress_TextChanged(object sender, EventArgs e)
        {
            //Registry.CreateSubKey("Software\\Remzibi OSD\\ArduPilot\\Settings");

        }

        private void missionGrid_CellEndEdit(object sender, DataGridViewCellEventArgs e)
        {
            setWaypointValues(missionGrid.CurrentRow.Cells[0].Value, missionGrid.CurrentRow.Cells[1].Value.ToString(), missionGrid.CurrentRow.Cells[2].Value.ToString(), false);

            if (bOnlineMode == true && bCompatibleMode == true)
            {
                webBrowser1.Document.GetElementById("moveWaypointButton").InvokeMember("click");
            }
            int nCurrentRow = Convert.ToInt32(missionGrid.CurrentRow.Cells[0].Value) - 1;
            if (chkLookupAlt.Checked == true)
            {
                missionGrid[3, nCurrentRow].Value = string.Format("{0:n2}", Convert.ToDouble(missionGrid[5, nCurrentRow].Value) - nHomeAlt + Convert.ToDouble(missionGrid[4, nCurrentRow].Value));
                missionGrid[4, nCurrentRow].Value = string.Format("{0:n2}", Convert.ToDouble(missionGrid[4, nCurrentRow].Value));
            }
            else
            {
                //System.Diagnostics.Debug.WriteLine(Convert.ToInt32(missionGrid.CurrentRow.Cells[0].Value) - 1);
                missionGrid[3, nCurrentRow].Value = string.Format("{0:n2}", Convert.ToDouble(missionGrid[3, nCurrentRow].Value));
                missionGrid[4, nCurrentRow].Value = string.Format("{0:n2}", Convert.ToDouble(missionGrid[4, nCurrentRow].Value));
            }
            //missionGrid[4,Convert.ToInt32( missionGrid.CurrentRow.Cells[0].Value)-1].Value = string.Format("{0:n0}", Convert.ToDouble(missionGrid[3, Convert.ToInt32(missionGrid.CurrentRow.Cells[0].Value)-1].Value));
        }

        private void launchLat_TextChanged(object sender, EventArgs e)
        {
        }

        private void txtAddress_Enter(object sender, EventArgs e)
        {
            this.AcceptButton = cmdSearch;
        }

        private void launchLon_TextChanged(object sender, EventArgs e)
        {
        }

        private void label9_Click(object sender, EventArgs e)
        {

        }

        private void missionGrid_Enter(object sender, EventArgs e)
        {
            timer1.Enabled = false;
        }

        private void missionGrid_Leave(object sender, EventArgs e)
        {
            timer1.Enabled = true;
        }

        private void launchAlt_hold_TextChanged(object sender, EventArgs e)
        {
            double outValue;
            if (double.TryParse(launchAlt_hold.Text, out outValue))
            {
                nAltHold = Convert.ToDouble(outValue / nMult);
            }
            else
            {
                nAltHold = 50;
            }
        }

        private void radius_TextChanged(object sender, EventArgs e)
        {
            double outValue;
            if (double.TryParse(radius.Text, out outValue))
            {
                nRadius = Convert.ToDouble(outValue / nMult);
            }
            else
            {
                nRadius = 20;
            }
        }

        private void launchAlt_hold_Leave(object sender, EventArgs e)
        {
            double outValue;
            if (double.TryParse(launchAlt_hold.Text, out outValue))
            {
                nAltHold = outValue / nMult;
            }
            else
            {
                nAltHold = 50 / nMult;
            }
            launchAlt_hold.Text = string.Format("{0:n1}",outValue);
            myRegistry.Write("AltHold", Convert.ToString(nAltHold));

        }

        private void radius_Leave(object sender, EventArgs e)
        {
            double outValue;
            if (double.TryParse(radius.Text, out outValue))
            {
                nRadius = outValue / nMult;
            }
            else
            {
                nRadius = 20 / nMult;
            }
            radius.Text = string.Format("{0:n1}", outValue);
            myRegistry.Write("Radius", Convert.ToString(nRadius));
        }


        private void cmdCenter_Click(object sender, EventArgs e)
        {
            if (missionGrid.Rows.Count != 0)
            {
                timer1.Enabled = false;
                webBrowser1.Document.GetElementById("index").SetAttribute("value", Convert.ToString(missionGrid.CurrentRow.Cells[0].Value));
                webBrowser1.Document.GetElementById("lat").SetAttribute("value", Regex.Replace(Convert.ToString(missionGrid.CurrentRow.Cells[1].Value), ",", "."));
                webBrowser1.Document.GetElementById("lng").SetAttribute("value", Regex.Replace(Convert.ToString(missionGrid.CurrentRow.Cells[2].Value), ",", "."));
                webBrowser1.Document.GetElementById("centerMapWaypointButton").InvokeMember("click");
                timer1.Enabled = true;
            }
        }

        private void dIYdronescomToolStripMenuItem_Click(object sender, EventArgs e)
        {
            System.Diagnostics.Process.Start("http://diydrones.com/profile/JeanLouisNaudin");
        }

        private void RemoveRow_Click(object sender, EventArgs e)
        {
            int nCount;
            string nSelectedColumn;
            if (missionGrid.Rows.Count != 0)
            {
                //missionGrid.CurrentCell = missionGrid.Rows[index].Cells[0];
                nSelectedColumn = Convert.ToString(missionGrid.CurrentRow.Cells[0].Value);
                missionGrid.Rows.RemoveAt(Convert.ToInt32(nSelectedColumn)-1);
                webBrowser1.Document.GetElementById("dwIndex").SetAttribute("value", nSelectedColumn);
                webBrowser1.Document.GetElementById("deleteWaypointButton").InvokeMember("click");

                for (nCount = 1; nCount <= missionGrid.Rows.Count; nCount++)
                {
                    missionGrid[0, nCount-1].Value = Convert.ToString(nCount);
                }
            }
            portControl1.number_Waypoints = Convert.ToByte(missionGrid.Rows.Count);
        }

        private void missionGrid_CellContentClick(object sender, DataGridViewCellEventArgs e)
        {
            try
            {
                if (bOnlineMode == true && bCompatibleMode == true)
                {
                    webBrowser1.Document.GetElementById("index").SetAttribute("value", Convert.ToString(missionGrid.CurrentRow.Cells[0].Value));
                    webBrowser1.Document.GetElementById("lat").SetAttribute("value", Regex.Replace(Convert.ToString(missionGrid.CurrentRow.Cells[1].Value), ",", "."));
                    webBrowser1.Document.GetElementById("lng").SetAttribute("value", Regex.Replace(Convert.ToString(missionGrid.CurrentRow.Cells[2].Value), ",", "."));
                }
            }
            catch { }
        }

        private void toolStripMenuItem14_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void toolStripMenuItem8_Click(object sender, EventArgs e)
        {

        }

        private void mnuSaveMission_Click(object sender, EventArgs e)
        {
            int nCount;

            SaveFileDialog dlg = new SaveFileDialog();

            dlg.DefaultExt = "txt";
            dlg.InitialDirectory = Application.StartupPath + "\\Missions";
            dlg.Title = "Charger un fichier Mission";
            dlg.AddExtension = true;
            dlg.Filter = "Fichiers Mission (*.txt)|*.txt|All files (*.*)|*.*";

            if (dlg.ShowDialog() == DialogResult.OK)
            {
                System.IO.FileStream fs = new System.IO.FileStream(dlg.FileName, System.IO.FileMode.Create);
                System.IO.StreamWriter sw = new System.IO.StreamWriter(fs, System.Text.Encoding.ASCII);

                sw.WriteLine("OPTIONS:" + Convert.ToInt16(Convert.ToDouble(radius.Text) / nMult) + "," + launchManually.Checked + "," + chkLookupAlt.Checked + "," + cboMetersFeet.Text + "," + numericMaps.Value.ToString());
                sw.WriteLine("HOME:" + Regex.Replace(Convert.ToString(launchLat.Text), ",", ".") + "," + Regex.Replace(Convert.ToString(launchLon.Text), ",", ".") + "," + Convert.ToInt16(Convert.ToDouble(launchAlt_hold.Text) / nMult));
                for (nCount = 1; nCount <= missionGrid.Rows.Count; nCount++)
                {
                    //if (chkLookupAlt.Checked == false)
                    //{
                        sw.WriteLine(Regex.Replace(Convert.ToString(missionGrid[1, nCount - 1].Value), ",", ".") + "," + Regex.Replace(Convert.ToString(missionGrid[2, nCount - 1].Value), ",", ".") + "," + Convert.ToInt32(Convert.ToDouble(missionGrid[4, nCount - 1].Value) / nMult));
                    //}
                    //else
                    //{
                    //    sw.WriteLine(Regex.Replace(Convert.ToString(missionGrid[1, nCount - 1].Value), ",", ".") + "," + Regex.Replace(Convert.ToString(missionGrid[2, nCount - 1].Value), ",", ".") + "," + Convert.ToInt32(Convert.ToDouble(missionGrid[4, nCount - 1].Value)) / nMult);
                    //}
                }
                sw.Flush();
                sw.Close(); fs.Close();

                Application.DoEvents();
                Bitmap b = new Bitmap(webBrowser1.ClientSize.Width, webBrowser1.ClientSize.Height);
                Graphics g = Graphics.FromImage(b);
                g.CopyFromScreen(webBrowser1.Parent.PointToScreen(webBrowser1.Location), new Point(0, 0), webBrowser1.ClientSize);
                b.Save(dlg.FileName.Substring(0, dlg.FileName.Length - 3) + "png");

                myRegistry.Write("LastSaved", dlg.FileName);
            }
        }

        private void mnuLoadMission_Click(object sender, EventArgs e)
        {
            string sLinesFromFile;
            string sValue;
            string sLastMetersFeet;

            timer1.Enabled = false;
            OpenFileDialog dlg = new OpenFileDialog();

            dlg.DefaultExt = "txt";
            dlg.InitialDirectory = Application.StartupPath + "\\Missions";
            dlg.Multiselect = false;
            dlg.CheckFileExists = true;
            dlg.Title = "Load Mission File";
            dlg.AddExtension = true;
            dlg.Filter = "Mission Files (*.txt)|*.txt|All files (*.*)|*.*";

            if (dlg.ShowDialog() == DialogResult.OK)
            {
                if (checkOnlineMode() == true)
                {
                    webBrowser1.Document.GetElementById("ClearButton").InvokeMember("click");
                }
                else
                {
                    System.Diagnostics.Debug.WriteLine(dlg.FileName.Substring(0, dlg.FileName.Length - 3) + "png");
                    if (File.Exists(dlg.FileName.Substring(0, dlg.FileName.Length - 3) + "png") == true)
                    {
                        picMap.Load(dlg.FileName.Substring(0, dlg.FileName.Length - 3) + "png");
                        lblOfflineLabel.Visible = true;
                        picMap.Visible = true;
                        //this.Width = picMap.Width + picMap.Left + 20;
                        //this.ResizeRedraw();
                    }
                    else
                    {
                        picMap.Image = null;
                        picMap.Invalidate();
                        //picMap.Image.Dispose();
                        lblOfflineLabel.Visible = false;
                        //this.Width = fraStatus.Width + fraStatus.Left + 20;
                        //this.ResizeRedraw();
                    }
                }
                missionGrid.Rows.Clear();

                FileStream fileStream = new FileStream(dlg.FileName, FileMode.Open);
                try
                {
                    sLastLoaded = dlg.FileName;
                    System.Text.ASCIIEncoding enc = new System.Text.ASCIIEncoding();
                    byte[] bytes = new byte[fileStream.Length];
                    int numBytesToRead = (int)fileStream.Length;
                    int numBytesRead = 0;
                    while (numBytesToRead > 0)
                    {
                        // Read may return anything from 0 to numBytesToRead.
                        int n = fileStream.Read(bytes, numBytesRead, numBytesToRead);

                        // Break when the end of the file is reached.
                        if (n == 0)
                            break;

                        numBytesRead += n;
                        numBytesToRead -= n;
                    }
                    numBytesToRead = bytes.Length;

                    sLinesFromFile = enc.GetString(bytes);
                    //System.Diagnostics.Debug.WriteLine(sLinesFromFile);
                    char[] sep ={ '\n' };
                    string[] values = sLinesFromFile.Split(sep);
                    string sObjType;
                    double nLat;
                    double nLng;

                    for (int nCount = 0; nCount < values.Length - 1; nCount++)
                    {
                        sValue = values[nCount].Substring(values[nCount].IndexOf(":") + 1);
                        sValue = sValue.Substring(0, sValue.Length - 1);

                        char[] sep2 ={ ',' };
                        string[] latLngAlt = sValue.Split(sep2);
                        sObjType = "";
                        if (values[nCount].IndexOf(":") != -1)
                        {
                            sObjType = values[nCount].Substring(0, values[nCount].IndexOf(":"));
                        }
                        double.TryParse(latLngAlt[0], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nLat);
                        double.TryParse(latLngAlt[1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nLng);
                        if (nCount % 1 == 0)
                        {
                            Application.DoEvents();
                        }
                        //System.Diagnostics.Debug.WriteLine(latLngAlt[0] + " , " + nLat);

                        //System.Diagnostics.Debug.WriteLine(values[nCount]);
                        //System.Diagnostics.Debug.WriteLine(values[nCount].Substring(0, values[nCount].IndexOf(":")));

                        switch (sObjType)
                        {
                            case "OPTIONS":
                                sLastMetersFeet = cboMetersFeet.Text;
                                cboMetersFeet.Text = latLngAlt[3];
                                if (sLastMetersFeet != cboMetersFeet.Text)
                                {
                                    cboMetersFeet_SelectedIndexChanged(null,null);
                                }
                                radius.Text = string.Format("{0:n1}", Convert.ToDouble(latLngAlt[0]) * nMult);
                                launchManually.Checked = Convert.ToBoolean(latLngAlt[1]);
                                chkLookupAlt.Checked = Convert.ToBoolean(latLngAlt[2]);
                                if (latLngAlt.GetUpperBound(0) >= 4)
                                {
                                    numericMaps.Value = Convert.ToInt32(latLngAlt[4]);
                                }
                                break;
                            case "HOME":
                                if (bOnlineMode == true && bCompatibleMode == true)
                                {
                                    webBrowser1.Document.GetElementById("homeLat").SetAttribute("value", latLngAlt[0]);
                                    webBrowser1.Document.GetElementById("homeLng").SetAttribute("value", latLngAlt[1]);
                                    webBrowser1.Document.GetElementById("setHomeLatLngButton").InvokeMember("click");
                                    webBrowser1.Document.GetElementById("centerMapHomeButton").InvokeMember("click");
                                }
                                nAltHold = Convert.ToDouble(latLngAlt[2]);
                                nHomeAlt = getAltitudeData(nLat, nLng);
                                lblHomeAlt.Text = string.Format("{0:n1}", nHomeAlt);
                                launchAlt_hold.Text = string.Format("{0:n1}", nAltHold * nMult);
                                
                                launchLat.Text = Convert.ToString(nLat);
                                launchLon.Text = Convert.ToString(nLng);
                                //System.Diagnostics.Debug.WriteLine(values[nCount].Substring(values[nCount].IndexOf(":")+1));
                                break;
                            default:
                                if (missionGrid.Rows.Count > g_MaxWaypoints-1)
                                {
                                    MessageBox.Show("Too many waypoints selected.\nArduPilot has 1K of EEPROM memory and currently can handle up to " + g_MaxWaypoints.ToString() + " waypoints.", "Max waypoints", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                                    timer1.Enabled = true;
                                    return;
                                }
                                lblClickMap.Visible = false;
                                missionGrid.Rows.Add();
                                missionGrid[0, missionGrid.Rows.Count - 1].Value = missionGrid.Rows.Count;
                                missionGrid[1, missionGrid.Rows.Count - 1].Value = Convert.ToString(nLat);
                                missionGrid[2, missionGrid.Rows.Count - 1].Value = Convert.ToString(nLng);

                                setWaypointValues(missionGrid.Rows.Count, latLngAlt[0], latLngAlt[1], true);
                                if (latLngAlt.Length > 2)
                                {
                                    missionGrid[4, missionGrid.Rows.Count - 1].Value = string.Format("{0:n2}", Convert.ToDouble(latLngAlt[2]) * nMult);
                                }
                                else
                                {
                                    missionGrid[4, missionGrid.Rows.Count - 1].Value = string.Format("{0:n2}", nAltHold * nMult);
                                }
                                missionGrid[5, missionGrid.Rows.Count - 1].Value = Convert.ToString(getAltitudeData(nLat, nLng));
                                if (chkLookupAlt.Checked == true)
                                {
                                    missionGrid[3, missionGrid.Rows.Count - 1].Value = string.Format("{0:n2}", (Convert.ToDouble(missionGrid[4, missionGrid.Rows.Count - 1].Value) + Convert.ToDouble(missionGrid[5, missionGrid.Rows.Count - 1].Value) - nHomeAlt));
                                }
                                else
                                {
                                    missionGrid[3, missionGrid.Rows.Count - 1].Value = string.Format("{0:n2}", Convert.ToDouble(missionGrid[4, missionGrid.Rows.Count - 1].Value));
                                }

                                break;
                        }
                    }
                }
                finally
                {
                    if (bOnlineMode == true && bCompatibleMode == true)
                    {
                        webBrowser1.Document.GetElementById("refreshButton").InvokeMember("click");
                    }
                    timer1_Tick(sender, e);
                    fileStream.Close();
                    portControl1.number_Waypoints = Convert.ToByte(missionGrid.Rows.Count);
                }
            }
            timer1.Enabled = true;


        }

        private void cboMetersFeet_SelectedIndexChanged(object sender, EventArgs e)
        {
            try
            {
                int nCount;
                double nSelectionChanged;
                if (cboMetersFeet.SelectedIndex == 0)
                {
                    if (bOnlineMode == true && bCompatibleMode == true)
                    {
                        webBrowser1.Document.GetElementById("metersFeet").SetAttribute("value", "0");
                    }
                    nMult = 1;
                    nSelectionChanged = 3.2808399;
                }
                else
                {
                    if (bOnlineMode == true && bCompatibleMode == true)
                    {
                        webBrowser1.Document.GetElementById("metersFeet").SetAttribute("value", "1");
                    }
                    nMult = 0.3048;
                    nSelectionChanged = 0.3048;
                }

                lblHomeAlt.Text = string.Format("{0:n0}", nHomeAlt * nMult);

                if (nLastMetersFeet != cboMetersFeet.SelectedIndex)
                {
                    //nHomeAlt = nHomeAlt * nSelectionChanged;
                    //nAltHold = nAltHold * nSelectionChanged;

                    nLastMetersFeet = cboMetersFeet.SelectedIndex;

                    for (nCount = 0; nCount < missionGrid.Rows.Count; nCount++)
                    {
                        missionGrid[3, nCount].Value = string.Format("{0:n2}", Convert.ToDouble(Convert.ToDouble(missionGrid[3, nCount].Value) * nSelectionChanged));
                        missionGrid[4, nCount].Value = string.Format("{0:n2}", Convert.ToDouble(Convert.ToDouble(missionGrid[4, nCount].Value) * nSelectionChanged));
                        missionGrid[5, nCount].Value = string.Format("{0:n2}", Convert.ToDouble(Convert.ToDouble(missionGrid[5, nCount].Value) * nSelectionChanged));
                    }
                }

                //webBrowser1.Document.GetElementById("zoomLevel").SetAttribute("value", Convert.ToString(numericMaps.Value));
                //webBrowser1.Document.GetElementById("setZoomButton").InvokeMember("click");

                if (bStartup == false)
                {
                    launchAlt_hold.Text = string.Format("{0:n2}", nAltHold * nMult);
                    radius.Text = string.Format("{0:n0}", nRadius * nMult);
                }
                //for (nCount = 1; nCount <= missionGrid.Rows.Count; nCount++)
                //{
                //    missionGrid[3, nCount - 1].Value = string.Format("{0:n0}", Convert.ToDouble(missionGrid[4, nCount - 1].Value) * nMult);
                //}
                if (bOnlineMode == true && bCompatibleMode == true)
                {
                    webBrowser1.Document.GetElementById("refreshButton").InvokeMember("click");
                }
                mnuFiles.HideDropDown();
                myRegistry.Write("FeetMeters", Convert.ToString(cboMetersFeet.SelectedIndex));
            }
            catch 
            {
                System.Diagnostics.Debug.WriteLine("HERE");

            }
        }

        private void cboBoardType_SelectedIndexChanged(object sender, EventArgs e)
        {
            mnuFiles.HideDropDown();
            myRegistry.Write("BoardType", Convert.ToString(cboBoardType.SelectedIndex));
        }

        private void cboComSelect_SelectedIndexChanged(object sender, EventArgs e)
        {
            mnuFiles.HideDropDown();
            myRegistry.Write("ComPort", Convert.ToString(cboComSelect.Text));
        }

        private void mnuRead_Click(object sender, EventArgs e)
        {
            timer1.Enabled = false;
            ardupilot_read();
            timer1.Enabled = true;
        }

        private void mnuWrite_Click(object sender, EventArgs e)
        {
            timer1.Enabled = false;
            ardupilot_write();
            timer1.Enabled = true;
        }

        private void chkLookupAlt_CheckedChanged(object sender, EventArgs e)
        {
            Cursor.Current = Cursors.WaitCursor; 
            bOverTerrainMode = chkLookupAlt.Checked;
            setOnlineMode(bOnlineMode);
            updateAltColumns();
            myRegistry.Write("OverTerrain", Convert.ToString(chkLookupAlt.Checked));
            
            if (bOverTerrainMode == true)
            {
                //System.Diagnostics.Debug.WriteLine(launchLat.Text.Replace(".",separatorFormat));
                //System.Diagnostics.Debug.WriteLine(separatorFormat);
                nHomeAlt = getAltitudeData(Convert.ToDouble(launchLat.Text.Replace(".",separatorFormat)), Convert.ToDouble(launchLon.Text.Replace(".",separatorFormat)));
                lblHomeAlt.Text = string.Format("{0:n0}", nHomeAlt);
            }
            Cursor.Current = Cursors.Default; 
        }

        private void cmdRead_Click(object sender, EventArgs e)
        {
            mnuRead_Click(null, null);
        }

        private void cmdWrite_Click(object sender, EventArgs e)
        {
            mnuWrite_Click(null,null);
        }

        private void cmdClearOffWaypoints_Click(object sender, EventArgs e)
        {
            missionGrid.Rows.Clear();
        }

        private void cmdRemoveOffWaypoint_Click(object sender, EventArgs e)
        {
            int nCount;
            string nSelectedColumn;
            if (missionGrid.Rows.Count != 0)
            {
                //missionGrid.CurrentCell = missionGrid.Rows[index].Cells[0];
                nSelectedColumn = Convert.ToString(missionGrid.CurrentRow.Cells[0].Value);
                missionGrid.Rows.RemoveAt(Convert.ToInt32(nSelectedColumn) - 1);

                for (nCount = 1; nCount <= missionGrid.Rows.Count; nCount++)
                {
                    missionGrid[0, nCount - 1].Value = Convert.ToString(nCount);
                }
            }
            portControl1.number_Waypoints = Convert.ToByte(missionGrid.Rows.Count);

        }

        private void cmdAddOffWaypoint_Click(object sender, EventArgs e)
        {
            missionGrid.Rows.Add();
            portControl1.number_Waypoints = Convert.ToByte(missionGrid.Rows.Count);
            missionGrid[0, missionGrid.Rows.Count - 1].Value = missionGrid.Rows.Count.ToString();
            missionGrid[1, missionGrid.Rows.Count - 1].Value = "00.000000";
            missionGrid[2, missionGrid.Rows.Count - 1].Value = "000.000000";
            missionGrid[3, missionGrid.Rows.Count - 1].Value = launchAlt_hold.Text;

        }

        private void launchLat_Leave(object sender, EventArgs e)
        {
            double outValue;
            double.TryParse(launchLat.Text, out outValue);
            if (outValue != 0)
            {
                if (bStartup == false && bOnlineMode == true && bCompatibleMode == true)
                {
                    setGoogleMapHome(launchLat.Text, launchLon.Text, true);
                }
                myRegistry.Write("HomeLat", Regex.Replace(launchLat.Text, ",", "."));
            }

        }

        private void launchLon_Leave(object sender, EventArgs e)
        {
            double outValue;
            double.TryParse(launchLon.Text, out outValue);
            if (outValue != 0)
            {
                if (bStartup == false && bOnlineMode == true && bCompatibleMode == true)
                {
                    setGoogleMapHome(launchLat.Text, outValue.ToString(), true);
                }
                myRegistry.Write("HomeLong", Regex.Replace(launchLon.Text, ",", "."));
            }

        }

        private void launchLon_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar.ToString() == "\r")
            {
                launchLon_Leave (sender, e);
            }
        }

        private void launchLat_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar.ToString() == "\r")
            {
                launchLat_Leave (sender, e);
            }

        }

        private void cboMetersFeet_Click(object sender, EventArgs e)
        {

        }

        private void Launch_paramters_Enter(object sender, EventArgs e)
        {

        }

        private void grpOnline_Enter(object sender, EventArgs e)
        {

        }

        private void lblClickMap_Click(object sender, EventArgs e)
        {

        }

        private void statusLabel_Click(object sender, EventArgs e)
        {

        }

        private void picMap_Click(object sender, EventArgs e)
        {

        }

        private void lblOfflineLabel_Click(object sender, EventArgs e)
        {

        }

        private void mnuMain_ItemClicked(object sender, ToolStripItemClickedEventArgs e)
        {

        }

    }

    /************************************************************/

    /************************************************************/

    public class portControl
    {
        public event EventHandler write_succesfull;
        public event EventHandler write_wrong;
        public event EventHandler read_succesfull;
        public event EventHandler read_wrong;
        SerialPort serialPort1 = new SerialPort();
        byte[] Array = new byte[512];
        byte[] Array_temp = new byte[512];
        private const byte start_byte = 24;

        /************************************************************/
        public string[] update_available_ports()
        {
            return SerialPort.GetPortNames();
        }
        /************************************************************/
        public void set_port(string portSelected, int speed, ref string errorMessage)
        {
            try
            {
                errorMessage = "";
                serialPort1.PortName = portSelected;
                serialPort1.ReadBufferSize = 1024;
                serialPort1.BaudRate = speed;
                serialPort1.Open();
                serialPort1.Close();
            }
            catch (Exception er)
            {
                //MessageBox.Show(Convert.ToString(er.Message));
                errorMessage = Convert.ToString(er.Message);
            }

        }
        /************************************************************/
        private void restart_Ardupilot(ref string errorMessage)
        {
            try
            {
                errorMessage = "";
                serialPort1.Open();
                serialPort1.DtrEnable = true; //Restarts ardupilot
                serialPort1.RtsEnable = true;
                Thread.Sleep(1000);
                serialPort1.DtrEnable = false;
                serialPort1.RtsEnable = false;
                serialPort1.Close();
            }
            catch (Exception er)
            {
                //MessageBox.Show(Convert.ToString(er.Message));
                errorMessage = Convert.ToString(er.Message);
            }
        }
        /************************************************************/
        public void change_address_Ardupilot(ref string errorMessage)
        {
            try
            {
                errorMessage = "";
                restart_Ardupilot(ref errorMessage);
                serialPort1.Open();
                anybodyThere(); //Hello Anybody there???? 
            }
            catch (Exception er)
            {
                //MessageBox.Show(Convert.ToString(er.Message));
                errorMessage = Convert.ToString(er.Message);
            }
        }
        /************************************************************/
        bool arduino_reponse()
        {
            byte one = 0;
            byte two = 0;
            int counter = 0;
            bool pass = true;

            do
            {
                if (serialPort1.BytesToRead >= 2)
                {
                    one = (byte)serialPort1.ReadByte();
                    two = (byte)serialPort1.ReadByte();
                }
                counter++;
                if (counter > 200000)
                {

                    pass = false;
                    break;

                }
            }
            while ((one != 0x14) & (two != 0x10));

            return pass;
        }
        /************************************************************/
        public void upload_Ardupilot(ref string errorMessage)
        {
            int counter = 0;
            bool pass = true;
            try
            {
                restart_Ardupilot(ref errorMessage);
                serialPort1.Open();
                anybodyThere(); //Hello Anybody there???? 

                /*Sending first 256 bytes... */
                serialPort1.Write(new byte[] { 0x64, 0x01, 0x00, 0x45 }, 0, 4); //I have change this to fix 0x64,0x00,0xFF,0x45
                serialPort1.Write(Array, 0, 0x0100); //I have changed this from 0x00FF to 0x0100
                serialPort1.Write(new byte[] { 0x20 }, 0, 1);

                /*Waiting for reponse.. */
                pass = arduino_reponse();

                if (pass)
                {
                    /*Changing EEPROM position... */
                    serialPort1.Write(new byte[] { 0x55, 0x80, 0x00, 0x20 }, 0, 4);

                    pass = arduino_reponse();
                    if (pass)
                    {
                        /*Sending the last 256 bytes .. */
                        serialPort1.Write(new byte[] { 0x64, 0x01, 0x00, 0x45 }, 0, 4);
                        serialPort1.Write(Array, 0x0100, 0x0100);
                        serialPort1.Write(new byte[] { 0x20 }, 0, 1);

                        while (serialPort1.BytesToRead == 0) //Waiting for bytes to arrive..
                        {
                            counter++;
                            if (counter > 1000000)
                            {
                                pass = false;
                                break;
                            }
                        }
                        if (pass)
                        {
                            pass = arduino_reponse();//waiting for reponse
                            if (pass)//verifying reponse...
                            {
                                Onwrite_succesfull(EventArgs.Empty);
                            }
                        }
                    }
                }
                if (!pass)
                {
                    Onwrite_wrong(EventArgs.Empty);
                }
                serialPort1.Close();
            }
            catch (Exception er)
            {
                errorMessage = Convert.ToString(er.Message);
            }
        }
        public void Onwrite_succesfull(EventArgs e)
        {
            write_succesfull(this, e);
        }
        public void Onwrite_wrong(EventArgs e)
        {
            write_wrong(this, e);
        }
        /************************************************************/
        public void download_Ardupilot()
        {
            int counter = 0;
            bool answer = true;
            byte one = 0;
            string sError = "";


            restart_Ardupilot(ref sError);
            serialPort1.Open();
            anybodyThere(); //Hello Anybody there???? 
            /************************************************************/

            do
            {
                //Seing commands
                serialPort1.Write(new byte[] { 0x74, 0x02, 0x00, 0x45, 0x20 }, 0, 5);
                counter++;
                if (counter > 1000)
                {
                    answer = false;
                    break;
                }

                if (serialPort1.BytesToRead > 0)
                {
                    one = (byte)serialPort1.ReadByte();
                }
            }
            while (one != 0x14);

            /************************************************************/
            if (answer == true)
            {
                for (int x = 0; x < 512; x++)
                {
                    counter = 0;
                    while (serialPort1.BytesToRead == 0)
                    {
                        counter++;
                        if (counter > 10000)
                        {
                            Application.DoEvents();
                        }
                        else if (counter > 20000)
                        {
                            answer = false;
                            break;
                        }
                    }
                    Array_temp[x] = (byte)serialPort1.ReadByte(); //Cambiado... 
                }


                while (serialPort1.BytesToRead == 0)
                {
                    counter++;
                    if (counter > 100000)
                    {
                        answer = false;
                        break;
                    }
                }
                /*********************************/
                //Now Writing
                if (answer)
                {
                    Application.DoEvents();
                    //System.Diagnostics.Debug.WriteLine(serialPort1.ReadByte() + " Byte=" + Convert.ToString(0x10));

                    if (serialPort1.ReadByte() == 0x10)
                    {
                        Onread_succesfull(EventArgs.Empty);
                        //MessageBox.Show("Download ran succesfully!", "YEAHH!!");
                        for (int y = 0; y < 512; y++)
                        {
                            Array[y] = Array_temp[y];
                        }

                    }
                    else
                    {
                        answer = false;
                    }
                }
            }
            if (!answer)
            {
                Onread_wrong(EventArgs.Empty);
            }
            // Close the port
            serialPort1.Close();
        }
        public void Onread_succesfull(EventArgs e)
        {
            read_succesfull(this, e);
        }
        public void Onread_wrong(EventArgs e)
        {
            read_wrong(this, e);
        }
        /************************************************************/
        private void anybodyThere()
        {
            int counter = 0;
            int counter2 = 0;
            int counter3 = 0;
            byte one = 0;
            byte two = 0;
            do
            {
                serialPort1.Write(new byte[] { 0x30, 0x20 }, 0, 2);
                //serialPort1.Write(new byte[] { 0x30 }, 0, 2);
                while (serialPort1.BytesToRead < 2)
                {
                    counter++;
                    if (counter > 200000)
                    {
                        serialPort1.Write(new byte[] { 0x30, 0x20 }, 0, 2);

                        if (counter3 > 5)
                        {
                            break;
                        }
                        else
                        {
                            counter3++;
                            counter = 0;
                        }
                    }
                }
                if (serialPort1.BytesToRead >= 2)
                {
                    one = (byte)serialPort1.ReadByte();
                    two = (byte)serialPort1.ReadByte();
                }
                counter2++;
                if (counter2 > 5)
                {
                    break;
                }
            }
            while ((one != 0x14) & (two != 0x10));
        }
        /************************************************************/
        public int get_mission_lat(int position)
        {
            int ber = 0;
            int real_position;
            real_position = start_byte + (position * 10);
            ber = (int)concat_4(real_position);
            return ber;
        }
        /************************************************************/
        public void set_mission_lat(int position, int data)
        {
            int real_position;
            real_position = start_byte + (position * 10);
            split_4(real_position, data);
        }
        /************************************************************/
        public int get_mission_lon(int position)
        {
            int dist = 0;
            int real_position;
            real_position = start_byte + (position * 10);
            dist = (int)concat_4(real_position + 4);
            return dist;
        }
        /************************************************************/
        public void set_mission_lon(int position, int data)
        {
            int real_position;
            real_position = start_byte + (position * 10);
            split_4(real_position + 4, data);
        }
        /************************************************************/
        public int get_mission_alt(int position)
        {
            int alt = 0;
            int real_position;
            real_position = start_byte + (position * 10);
            alt = (int)concat_2(real_position + 8);
            return alt;
        }
        /************************************************************/
        public void set_mission_alt(int position, int data)
        {
            int real_position;
            real_position = start_byte + (position * 10);
            split_2(real_position + 8, data);
        }
        /************************************************************/

        //Options byte
        public byte options
        {
            get
            {
                return Array[0];
            }
            set
            {
                Array[0] = value;
            }

        }

        //Roll Trim
        public byte rtrim
        {

            get
            {
                return Array[3];
            }
            set
            {
                Array[3] = value;
            }
        }

        //Elevator Trim
        public byte ptrim
        {

            get
            {
                return Array[4];
            }
            set
            {
                Array[4] = value;
            }
        }

        //Max altitude
        public Int16 maxalt
        {
            get
            {
                return (short)concat_2(5);
            }
        }

        //Max Speed
        public Int16 maxspd
        {
            get
            {
                return (short)concat_2(7);
            }

        }

        public void clear_max_altspd()
        {
            Array[5] = 0;
            Array[6] = 0;
            Array[7] = 0;
            Array[8] = 0;
        }

        //Radius for waypoint clearence
        public byte Radius
        {
            get
            {
                return Array[11];
            }
            set
            {
                Array[11] = value;
            }

        }

        //Reference altitude
        public Int16 launch_Altitude_ref
        {
            get
            {
                return (Int16)concat_4(12);
            }
        }

        //launch Latitude
        public int launch_Latitude
        {
            get
            {
                return concat_4(14);
            }
            set
            {
                split_4(14, value);
            }
        }

        //Launch Longitude
        public long launch_Longitude
        {
            get
            {
                return concat_4(18);
            }
            set
            {
                split_4(18, value);
            }
        }

        //Altitude to hold
        public Int16 launch_Altitude_hold
        {
            get
            {
                return (Int16)concat_4(22);
            }
            set
            {
                split_2(22, (int)value);
            }
        }

        //Number of waypoints
        public byte number_Waypoints
        {
            set
            {
                Array[9] = value;
            }
            get
            {
                return Array[9]; //number_waypoints;
            }
        }

        //Number of waypoints
        public byte current_Waypoints
        {
            get
            {
                return Array[10]; //number_waypoints;
            }
        }

        //Some services
        private void split_2(int startByte, int data)
        {
            Array[startByte + 1] = (byte)(data >> 8);
            Array[startByte] = (byte)(data);
        }

        private void split_4(int startByte, long data)
        {
            Array[startByte + 3] = (byte)(data >> 24);
            Array[startByte + 2] = (byte)(data >> 16);
            Array[startByte + 1] = (byte)(data >> 8);
            Array[startByte] = (byte)(data);

            //System.Diagnostics.Debug.WriteLine((int)(Array[startByte + 3] << 24));
        }

        private int concat_2(int startByte)
        {
            int temp;
            temp = ((int)Array[startByte + 1] << 8);
            temp |= (int)Array[startByte];
            return temp;
        }

        private int concat_4(int startByte)
        {
            int temp;
            temp = (int)(Array[startByte + 3] << 24);
            temp |= ((int)Array[startByte + 2] << 16);
            temp |= ((int)Array[startByte + 1] << 8);
            temp |= (int)Array[startByte];
            return temp;
        }

    }
}