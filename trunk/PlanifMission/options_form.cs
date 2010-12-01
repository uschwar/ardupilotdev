using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;

namespace ArduPilotConfigTool
{
    public partial class options_form : Form
    {
        public event EventHandler buttonn_change;

        public options_form()
        {
            InitializeComponent();
        }

        private void checkBox1_CheckedChanged(object sender, EventArgs e)
        {

        }
        /************************************************************/
        public bool reverse_roll
        {
            get
            {
                return reverse_Roll.Checked;
            }
            set
            {
                reverse_Roll.Checked = value;
            }
        }
        /************************************************************/
        public bool reverse_elevator
        {
            get
            {
                return reverse_Elevator.Checked;
            }
            set
            {
                reverse_Elevator.Checked = value;
            }
        }
        /************************************************************/
        public bool sernsor_z
        {
            get
            {
                return sensorZ.Checked;
            }
            set
            {
                sensorZ.Checked = value;
            }
        }

        public bool txstates
        {
            get
            {
                return txState.Checked;
            }
            set
            {
                txState.Checked = value;
            }
        }

        public bool txdefault
        {
            get
            {
                if (Convert.ToString(txDefaultAction.SelectedItem) == "RTL")
                    return false;
                else
                    return true;
            }
            set
            {
                if (value)
                {
                    txDefaultAction.SelectedIndex=1;
                }
                else
                {
                    txDefaultAction.SelectedIndex = 0;
                }
            }
        }

        public sbyte trim_r
        {
            get
            {
                return Convert.ToSByte(trimR.Text); 
            }
            set
            {
                trimR.Text = Convert.ToString(value);
            }

        }

        public sbyte trim_p
        {
            get
            {
                return Convert.ToSByte(trimP.Text);
            }
            set
            {
                trimP.Text = Convert.ToString(value);
            }

        }

        private void txState_CheckedChanged(object sender, EventArgs e)
        {
            txDefaultAction.Enabled = txState.Checked;
        }

        public void button_set_Click(object sender, EventArgs e)
        {
            Onbuttonn_change(EventArgs.Empty);
        }
        /************************************************************/
        public void Onbuttonn_change(EventArgs e)
        {
            buttonn_change(this, e);
        }

        private void options_form_Load(object sender, EventArgs e)
        {

        }

        private void txDefaultAction_SelectedIndexChanged(object sender, EventArgs e)
        {

        }
    }
}