/*
 * ARDUPILOT CONTROLLER DATALINK
 * FOR THE FLIGHT GEAR SIMULATOR
 *
 *   by Jean-Louis Naudin - Nov-Dec 2009
 *
     This is an Open Source software.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

// last update : 12-08-09 - JLN

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <conio.h>
#include <string.h>

#include <SDL/SDL.h>
#include <SDL/SDL_ttf.h>

//pour Windows
#include <SDL/SDL_Thread.h>
#include <winsock2.h>

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Value_Output.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Return_Button.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_Text_Display.H>
#include <FL/Fl_Text_Editor.H>
#include <FL/Fl_Int_Input.H>
#include <cstdio>

#include "net_fdm.hxx"
#include "net_ctrls.hxx"

#define FAILURE 0
#define SUCCESS !FAILURE

#define 		pi 			3.14159265
#define 		FOURTHPI 	0.7853981634 //pi/4
#define 		deg2rad 	0.0174532925
#define 		pi2     6.283184  //pi*2
#define         pi05	1.570796  //pi/2
#define 		rad2deg 57.29578
#define         kts2ms  0.514

using namespace std;
int output_state(int ch, int state);

// RS232 Routines
HANDLE rs_initialise (const long int BaudRate,
                      const char parity, const char data);
void rs_flush(void);
void rs_terminate(void);
char rs_getch(void);
int rs_getline(char line[], clock_t timeout);
void rs_putch(int txchar);
void rs_putstr(const char *string);

// delay_routines
void delay_ms(clock_t millis);

HANDLE hCom;           //handle for serial port I/O
int io_port;

const int SCREEN_WIDTH = 400;
const int SCREEN_HEIGHT = 200;
const int SCREEN_BPP = 32;

//Les surfaces
SDL_Surface *ecran = NULL;
int screenw, screenh;

FILE *fr;
clock_t timeout;

struct sockaddr_in echoclient,from, ctrlclient, ctrlinclient;
int sock, sock2, ret, ret2, echolen, clientlen, received=0,
    count, num_chars, res, mode=0;

unsigned long ip;
float ground_speed,dst,qfu=0;
int rll,pch,gps,cwp,bto,trll,tpch,gas,gcs,thr, ack,datain=0;
int set_roll=rll, set_pitch=pch, set_throttle=thr;
bool continuer = true;

SDL_Thread *th_routerFGS, *th_Ardulinker;
SDL_mutex* mut_ardupilot;    // Mutex for accessing to ardupilot data
SDL_mutex* mut_flightgear;   // Mutex for accessing to flightgear data

// setup var
int com, logfile;
double gphi, gtheta, gpsi, gthr,gphidot,ailtrim, rudtrim, ref_qfe;
int pitrim, rdtrim, rltrim, gstrim;

typedef signed int       int32_t;
typedef unsigned int     uint32_t;

WSADATA	wsaData ;

double ltime = 0.0;

struct known_state {
	double longitude;
	double latitude;
	double altitude;

	float phidot;
	float thetadot;
	float psidot;
	float vcas;
	float v_north;
	float v_east;
	float v_down;
	float phi;
	float theta;
	float psi;
	float alpha;
	float beta;

	float A_X_pilot;
	float A_Y_pilot;
	float A_Z_pilot;
};

struct known_state state;
struct known_state old_state;

struct packet {
  struct known_state known_state;
};

// Event structure
SDL_Event event;
SDL_Rect rect;

// used Font
TTF_Font *font;

// Data displayed on the screen
struct Fl_raw
{
  Fl_Value_Output *phi,*theta,*psi, *alpha, *beta;
  Fl_Value_Output *Pt, *gsp, *cwp, *dst, *bto, *gcs;
  Fl_Value_Output *rll, *pch, *thr, *mod, *com;
  Fl_Value_Output *lat,*lon,*alt, *ail, *ele;
  Fl_Value_Output *gps, *ack;
};

#define htoni(x) \
    ( x << 24 ) & 0xFF000000 | \
 	( x << 8  ) & 0x00FF0000 | \
 	( x >> 8  ) & 0x0000FF00 | \
 	( x >> 24 ) & 0x000000FF

#define ntohi  htoni

float ntohf(const float f)
 {
     int i = ntohi( *(int *)&f ) ;
     return *(float *)&i ;
 }

double ntohd  (double  d)
{        assert(sizeof(d) == 8);

         double tmp;
         char* src = (char*) &d;
         char* dst = (char*) &tmp;

         dst[0] = src[7];
         dst[1] = src[6];
         dst[2] = src[5];
         dst[3] = src[4];
         dst[4] = src[3];
         dst[5] = src[2];
         dst[6] = src[1];
         dst[7] = src[0];

         return tmp;
         }

static void htond (double &x)
{
        int    *Double_Overlay;
        int     Holding_Buffer;

        Double_Overlay = (int *) &x;
        Holding_Buffer = Double_Overlay [0];

        Double_Overlay [0] = htonl (Double_Overlay [1]);
        Double_Overlay [1] = htonl (Holding_Buffer);
}

// Float version
static void htonf (float &x)
{
        int    *Float_Overlay;
        int     Holding_Buffer;

        Float_Overlay = (int *) &x;
        Holding_Buffer = Float_Overlay [0];

        Float_Overlay [0] = htonl (Holding_Buffer);
}

// Update the property tree from the FGNetCtrls structure.
void FGNetCtrls2Props( FGNetCtrls *net )
{ 	int i = 0;

	// convert to network byte order
	net->version = ntohl(net->version);
	net->aileron=ntohd(net->aileron);
	net->elevator=ntohd(net->elevator);
	net->rudder=ntohd(net->rudder);
	net->aileron_trim=ntohd(net->aileron_trim);
	net->elevator_trim=ntohd(net->elevator_trim);
	net->rudder_trim=ntohd(net->rudder_trim);
	net->flaps=ntohd(net->flaps);
	net->flaps_power = ntohl(net->flaps_power);
	net->flap_motor_ok = ntohl(net->flap_motor_ok);

	net->num_engines = ntohl(net->num_engines);
	for ( i = 0; i < FGNetCtrls::FG_MAX_ENGINES; ++i ) {
		net->master_bat[i] = ntohl(net->master_bat[i]);
		net->master_alt[i] = ntohl(net->master_alt[i]);
		net->magnetos[i] = ntohl(net->magnetos[i]);
		net->starter_power[i] = ntohl(net->starter_power[i]);
		net->throttle[i]=ntohd(net->throttle[i]);
		net->mixture[i]=ntohd(net->mixture[i]);
		net->fuel_pump_power[i] = ntohl(net->fuel_pump_power[i]);
		net->prop_advance[i]=ntohd(net->prop_advance[i]);
		net->condition[i]=ntohd(net->condition[i]);
		net->engine_ok[i] = ntohl(net->engine_ok[i]);
		net->mag_left_ok[i] = ntohl(net->mag_left_ok[i]);
		net->mag_right_ok[i] = ntohl(net->mag_right_ok[i]);
		net->spark_plugs_ok[i] = ntohl(net->spark_plugs_ok[i]);
		net->oil_press_status[i] = ntohl(net->oil_press_status[i]);
		net->fuel_pump_ok[i] = ntohl(net->fuel_pump_ok[i]);
	}

	net->num_tanks = ntohl(net->num_tanks);
	for ( i = 0; i < FGNetCtrls::FG_MAX_TANKS; ++i ) {
		net->fuel_selector[i] = ntohl(net->fuel_selector[i]);
	}

	net->cross_feed = ntohl(net->cross_feed);
	net->brake_left=ntohd(net->brake_left);
	net->brake_right=ntohd(net->brake_right);
	net->copilot_brake_left=ntohd(net->copilot_brake_left);
	net->copilot_brake_right=ntohd(net->copilot_brake_right);
	net->brake_parking=ntohd(net->brake_parking);
	net->gear_handle = ntohl(net->gear_handle);
	net->master_avionics = ntohl(net->master_avionics);
	net->wind_speed_kt=ntohd(net->wind_speed_kt);
	net->wind_dir_deg=ntohd(net->wind_dir_deg);
	net->turbulence_norm=ntohd(net->turbulence_norm);
	net->temp_c=ntohd(net->temp_c);
	net->press_inhg=ntohd(net->press_inhg);
	net->hground=ntohd(net->hground);
	net->magvar=ntohd(net->magvar);
	net->icing = ntohl(net->icing);
	net->speedup = ntohl(net->speedup);
	net->freeze = ntohl(net->freeze);
}

void FGProps2NetCtrls( FGNetCtrls *net)
{	int i = 0;
	// convert to network byte order
	net->version = htonl(net->version);
	htond(net->aileron);
	htond(net->elevator);
	htond(net->rudder);
	htond(net->aileron_trim);
	htond(net->elevator_trim);
	htond(net->rudder_trim);
	htond(net->flaps);
	net->flaps_power = htonl(net->flaps_power);
	net->flap_motor_ok = htonl(net->flap_motor_ok);

	net->num_engines = htonl(net->num_engines);
	for ( i = 0; i < FGNetCtrls::FG_MAX_ENGINES; ++i ) {
		net->master_bat[i] = htonl(net->master_bat[i]);
		net->master_alt[i] = htonl(net->master_alt[i]);
		net->magnetos[i] = htonl(net->magnetos[i]);
		net->starter_power[i] = htonl(net->starter_power[i]);
		htond(net->throttle[i]);
		htond(net->mixture[i]);
		net->fuel_pump_power[i] = htonl(net->fuel_pump_power[i]);
		htond(net->prop_advance[i]);
		htond(net->condition[i]);
		net->engine_ok[i] = htonl(net->engine_ok[i]);
		net->mag_left_ok[i] = htonl(net->mag_left_ok[i]);
		net->mag_right_ok[i] = htonl(net->mag_right_ok[i]);
		net->spark_plugs_ok[i] = htonl(net->spark_plugs_ok[i]);
		net->oil_press_status[i] = htonl(net->oil_press_status[i]);
		net->fuel_pump_ok[i] = htonl(net->fuel_pump_ok[i]);
	}

	net->num_tanks = htonl(net->num_tanks);
	for ( i = 0; i < FGNetCtrls::FG_MAX_TANKS; ++i ) {
		net->fuel_selector[i] = htonl(net->fuel_selector[i]);
	}

	net->cross_feed = htonl(net->cross_feed);
	htond(net->brake_left);
	htond(net->brake_right);
	htond(net->copilot_brake_left);
	htond(net->copilot_brake_right);
	htond(net->brake_parking);
	net->gear_handle = htonl(net->gear_handle);
	net->master_avionics = htonl(net->master_avionics);
	htond(net->wind_speed_kt);
	htond(net->wind_dir_deg);
	htond(net->turbulence_norm);
	htond(net->temp_c);
	htond(net->press_inhg);
	htond(net->hground);
	htond(net->magvar);
	net->icing = htonl(net->icing);
	net->speedup = htonl(net->speedup);
	net->freeze = htonl(net->freeze);
}

void FGNetFDM2Props( FGNetFDM *net ) {
    unsigned int i;

// Convert to the net buffer from network format
        net->version = ntohl(net->version);

        htond(net->longitude);  // geodetic (radians)
        htond(net->latitude);   // geodetic (radians)
        htond(net->altitude);   // above sea level (meters)
        htonf(net->agl);        // above ground level (meters)
        htonf(net->phi);        // roll (radians)
        htonf(net->theta);      // pitch (radians)
        htonf(net->psi);        // yaw (radians)
        htonf(net->alpha);      // angle of attack (radians)
        htonf(net->beta);       // side slip angle (radians)

        htonf(net->phidot);		// roll rate (radians/sec)
        htonf(net->thetadot);	// pitch rate (radians/sec)
        htonf(net->psidot); 	// yaw rate (radians/sec)
        htonf(net->vcas);       // calibrated airspeed (knots)
        htonf(net->climb_rate);
        htonf(net->v_north);
        htonf(net->v_east);
        htonf(net->v_down);
        htonf(net->v_wind_body_north);
        htonf(net->v_wind_body_east);
        htonf(net->v_wind_body_down);

        htonf(net->A_X_pilot);	// X accel in body frame ft/sec^2
        htonf(net->A_Y_pilot);  // Y accel in body frame ft/sec^2
        htonf(net->A_Z_pilot);	// Z accel in body frame ft/sec^2

        htonf(net->stall_warning);
        htonf(net->slip_deg);
}

FGNetFDM buf;
char* buffer = (char*)&buf;

FGNetCtrls bufctrl;
char* buffer_ctrl = (char*)&bufctrl;

/// Tools ////

void delay_ms(clock_t millis)
{
   clock_t endtime;
   endtime = millis + clock();
   while( endtime > clock() );
}

float Search_Float(char buffer[],char looking[])
{
  char *BufferPointer=buffer;
  char *SearchString;
  SearchString= strstr (BufferPointer,looking);
  return atof(SearchString+4);
}

int Search_Int(char buffer[],char looking[])
{
  char *BufferPointer=buffer;
  char *SearchString;
  SearchString= strstr (BufferPointer,looking);
  return atoi(SearchString+4);
}

long Search_Long(char buffer[],char looking[])
{
  char *BufferPointer=buffer;
  char *SearchString;
  SearchString= strstr (BufferPointer,looking);
  return atol(SearchString+4);
}

// About window
int aide_ardufgs()
{    Fl_Window *win = new Fl_Window(640, 480);
     Fl_Text_Buffer *buff = new Fl_Text_Buffer();
     Fl_Text_Display *disp = new Fl_Text_Display(20, 20, 640-40, 480-40, "ArduPilot Control");
     disp->buffer(buff);
     win->resizable(*disp);
     win->show();
     buff->text("\n\nArduPilot Control for the Flight Gear Simulator by Jean-louis Naudin\n"
                "---------------------------------------------------------------------\n"
                "          Version 1.2 beta under developpment - September 2010\n\n\n"
                "Web Blog : http://diydrones.com/profile/JeanLouisNaudin\n\n"
                "Open Source software under the terms of the GNU General Public License\n\n"
                "FOR MORE INFO : jlnaudin@gmail.com");
     return(Fl::run());
 }


void aide(Fl_Widget*, void*) {
    aide_ardufgs();
}
// Loading initial setup
int readsetup()
{	char line[256]="";
    char mesg[3];
    char end=0;
    int i;
	FILE * fpid;

   //read file
	fpid = fopen("afgcinit.txt","r+");

       while(fgets(line, 256, fpid) != NULL)
       {
               sscanf(line,"%s",mesg);
               if(!strcmp(mesg,"COM"))
                    sscanf(line,"COM %d",&com);
               else if(!strcmp(mesg,"LOG"))
                    sscanf(line,"LOG %d",&logfile);
               else if(!strcmp(mesg,"EFF"))
                    sscanf(line,"EFF %d %d %d",&pitrim,&rltrim,&gstrim);
               else if(!strcmp(mesg,"SET"))
                    sscanf(line,"SET %lf %lf %lf %lf %lf %lf %lf",&gphi,&gtheta,&gpsi,&gthr,&gphidot,&ailtrim,&rudtrim);

       };
   fclose(fpid);

   printf("Setup file:\nCOM:%d\nLOG:%d\nEFF:%d %d %d\nSET:%lf %lf %lf %lf %lf %lf %lf\n\n",
      com,logfile,pitrim,rltrim,gstrim,gphi,gtheta,gpsi,gthr,gphidot,ailtrim,rudtrim);

}

void Quit_CB(Fl_Widget *, void *) {
	if(com>0)
	{
     rs_terminate();
     }
    SDL_DestroyMutex(mut_ardupilot);
    SDL_DestroyMutex(mut_flightgear);

    SDL_Quit();
    exit(0);
}

void mode_manual(Fl_Widget *, void *) {
    mode=0;
}

void mode_autostab(Fl_Widget *, void *) {
    mode=1;
}

void mode_wp(Fl_Widget *, void *) {
    mode=2;
}

void mode_rth(Fl_Widget *, void *) {
    mode=3;
}

void mode_NextWp(Fl_Widget *, void *) {
    mode=4;
}

void mode_PrevWp(Fl_Widget *, void *) {
    mode=5;
}

void reload(Fl_Widget *, void *) {
    readsetup();
    mode=6;
}
////////////////////
// RS232 routines //
////////////////////
// init the com port

HANDLE rs_initialise (const long int BaudRate, const char parity, const char data)
{   BOOL bPortReady;
    DCB dcb;
	COMMTIMEOUTS CommTimeouts;

    char ComPortName[5]="COM ";
    ComPortName[3]='0'+ io_port;
    hCom = CreateFile(ComPortName, GENERIC_READ | GENERIC_WRITE,
                                  0,            // exclusive access
                                  NULL,         // no security
                                  OPEN_EXISTING,
                                  0,            // no overlapped I/O
                                  NULL);        // null template

   if ((int)hCom <= 0)
    {   printf("serial port COM%d connect fail %s error %d\n\r", io_port, ComPortName, GetLastError());
        return 0;
    }

    bPortReady = SetupComm(hCom, 128, 128); // set buffer sizes
    if (!bPortReady )
    {   printf("serial port COM%d SetupComm fail  %d\n\r", io_port,  GetLastError());
        return 0;
    }

    bPortReady = GetCommState(hCom, &dcb);
    if (!bPortReady )
    {   printf("serial port COM%d  GetCommState fail  %d\n\r", io_port,  GetLastError());
        return 0;
    }
    dcb.BaudRate = BaudRate;
    if( data == '7') dcb.ByteSize = 7;
    else             dcb.ByteSize = 8;
    if( parity == 'E') dcb.Parity = EVENPARITY;
    if( parity == 'O') dcb.Parity = ODDPARITY;
    else               dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fAbortOnError = TRUE;

    // set XON/XOFF
    dcb.fOutX = FALSE;                       // XON/XOFF off for transmit
    dcb.fInX = FALSE;                        // XON/XOFF off for receive
    // set RTSCTS
    dcb.fOutxCtsFlow = FALSE;               // turn off CTS flow control
    dcb.fRtsControl = FALSE;                // RTS_CONTROL_HANDSHAKE; //
    // set DSRDTR
    dcb.fOutxDsrFlow = FALSE;               // turn off DSR flow control
    //dcb.fDtrControl = DTR_CONTROL_ENABLE; // DTR handshake
    dcb.fDtrControl = DTR_CONTROL_DISABLE;  //
    // dcb.fDtrControl = DTR_CONTROL_HANDSHAKE; //

    bPortReady = SetCommState(hCom, &dcb);
    if (!bPortReady )
    {
        printf("serial port COM%d  SetCommState fail  %d\n\r", io_port,  GetLastError());
        return 0;
    }
    // Communication timeouts
    //COMMTIMEOUTS CommTimeouts;
    bPortReady = GetCommTimeouts (hCom, &CommTimeouts);
    CommTimeouts.ReadIntervalTimeout = 5 ;
    CommTimeouts.ReadTotalTimeoutConstant = 5 ;
    CommTimeouts.ReadTotalTimeoutMultiplier = 1 ;
    CommTimeouts.WriteTotalTimeoutConstant = 5 ;
    CommTimeouts.WriteTotalTimeoutMultiplier = 1 ;
    bPortReady = SetCommTimeouts (hCom, &CommTimeouts);
    if (!bPortReady )
    {  printf("serial port COM%d SetCommTimeouts fail  %d\n\r", io_port,  GetLastError());
       return 0;
    }
    else
    {  printf(" serial port COM%d connect OK \n\r", io_port);
    }

    return(hCom);
}

// Close the com port
void rs_terminate(void)
{
   CloseHandle(hCom);
}

// receive one char from the com port
char rs_getch(void)
{
    char recchar;
    BOOL bReadRC;
    static DWORD iBytesRead;
    bReadRC = ReadFile(hCom, &recchar, 1, &iBytesRead, NULL);
    if (iBytesRead)
    {   //printf("%d ", recchar);
        return recchar;
    }
    else
    {
        return 0;         // return 0 if no character read
    }
}

// flush the buffer of the com port
void rs_flush(void)
{
    while(rs_getch()!=0)   ;
}

// get data uplink line (start=!  end= *)
int rs_getline(char line[], clock_t timeout)
{
    int num_chars = 0, stline=0;
    char ch;
    clock_t endtime;
    endtime = timeout + clock();

    while(endtime > clock())
    {
       //printf("!");
        ch = rs_getch();
        //printf("%c", ch);
        //ch=0;
        if (ch != 0)
        { if((ch == 33) && (stline==0)) stline=1;  // waiting the ! start of data
          if(stline==1)
          {
            //printf("%c", ch);
            if ((ch == 42) || (ch == 13)) // waiting the * end of data
               {
                line[num_chars-1] = '\0'; // terminate the string
                if (num_chars < 10) return(0);
                else return(num_chars);
                }
            else
                {
                line[num_chars] = ch;
                ++num_chars;
                }
            }
        }

    } // end of while
    line[num_chars-1] = '\0';

    return(-1);  // timeout
}

// send one char to the com port
void rs_putch(int txchar)
{
    BOOL bWriteRC;
    static DWORD iBytesWritten;
    bWriteRC = WriteFile(hCom, &txchar, 1, &iBytesWritten,NULL);
    return;
}

// send one line of text to the com port
void rs_putstr(const char *string)
{
if (logfile==1)
{
    printf("COM=>%s\n", string);
}
    while (*string != '\0')
    {
        delay_ms(5);
        rs_putch(*string++);
    }
}

// Opening com port with the ArduPilot
void InitDatalink()
{
	io_port = com;
	if (com>0)
	{
    if(!rs_initialise(9600, '8', 'N')) // open the COM port
      {
         printf("Opening Port Failed\n");
         delay_ms(5000);
         com=0;  // If no com port detected don't use the Ardupilot comlink
         }
     }
 }

//////////////////////
// Network routines //
//////////////////////
// Init network UDP communication with flight gear
int initFG()
{
    res = WSAStartup(MAKEWORD(2 ,0), &wsaData) ;

	srand ( time(NULL) );
	memset(&state,		0, sizeof(state));
	memset(&old_state,	0, sizeof(state));

	printf("UDP sockets creation\n");

	/* Create the UDP socket */
	if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		std::cerr << "Failed to create socket" << std::endl;
		printf("socket sock creation error");
		return -1;
	}
	if ((sock2 = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		std::cerr << "Failed to create socket" << std::endl;
		printf("socket sock2 creation error");
		return -1;
	}

   ip=inet_addr("127.0.0.1");       // Flightgear Ip

	/* Create the UDP socket */
	if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		std::cerr << "Failed to create socket" << std::endl;
		perror("socket");
		return -1;
	}

	if ((sock2 = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		std::cerr << "Failed to create socket" << std::endl;
		perror("socket");
		return -1;
	}

	/// the port where net fdm is received
	memset(&echoclient, 0, sizeof(echoclient));             /* Clear struct */
	echoclient.sin_family = AF_INET;                        /* Internet/IP */
	echoclient.sin_addr.s_addr = inet_addr("127.0.0.1");    /* IP address */
	echoclient.sin_port = htons(5500);                      /* server port */

	/// the port where the modified net ctrl is sent to flightgear
	memset(&ctrlclient, 0, sizeof(ctrlclient));              /* Clear struct */
	ctrlclient.sin_family = AF_INET;                         /* Internet/IP */
	ctrlclient.sin_addr.s_addr = inet_addr("127.0.0.1");     /* IP address */
	ctrlclient.sin_port = htons(5600);                       /* server port */

	/// this is the port where the net ctrl comes from flight gear
	memset(&ctrlinclient, 0, sizeof(ctrlinclient));          /* Clear struct */
	ctrlinclient.sin_family = AF_INET;                       /* Internet/IP */
	ctrlinclient.sin_addr.s_addr = inet_addr("127.0.0.1");   /* IP address */
	ctrlinclient.sin_port = htons(5700);                     /* server port */

	int length = sizeof(buf);

	std::cout << "packet length should be " << length << " bytes" << std::endl;

	echolen = length;

	char errorbuf[100];

	/// need this?
	ret  = bind(sock,  (struct sockaddr*)&echoclient, sizeof(echoclient) );
	printf("bind sock ret=%i\n",ret);

	ret2 = bind(sock2, (struct sockaddr*)&ctrlinclient, sizeof(ctrlinclient) );
    printf("bind sock2 ret2=%i\n",ret);

    return 0;
}

// Refresh the displayed data
void Refresh(void* data)
{ struct Fl_raw* disp=(struct Fl_raw*)data;

  (disp->phi)->value(state.phi);
  (disp->theta)->value(state.theta);
  (disp->gcs)->value((int)state.psi);

  (disp->mod)->value(mode);

  (disp->lat)->value(state.latitude);
  (disp->lon)->value(state.longitude);
  (disp->alt)->value((int)state.altitude);
  (disp->com)->value(com);

  (disp->Pt)->value((int)state.vcas);
  (disp->gsp)->value((int)ground_speed);
  if(com>0)
  {
  (disp->rll)->value(rll);
  (disp->pch)->value(pch);
  (disp->cwp)->value(cwp);
  (disp->dst)->value(dst);
  (disp->bto)->value(bto);
  (disp->thr)->value(thr);
  (disp->gps)->value(gps);
  (disp->ack)->value(ack);
}
  Fl::add_timeout(0.1,Refresh,data);
}

////////////////// THREADS ////////////////

//////////////////////////////////////////////////////////////////
// Thread to receive data from the ArduPilot through the Com port
//////////////////////////////////////////////////////////////////
int Ardulinker(void *data)
{  char s[1024], line[1024];
   int old_wp;

  while(1) {
   SDL_mutexP ( mut_ardupilot );  // Lock variable before accessing to ardupilot datalink

   if (com>0)
   {

   if ((num_chars = rs_getline(line, 100)) == 0)
    {
        return(FAILURE);
    }

 if (logfile == 1)
    {
    printf("COM<=%s\n", line);
    }

   if (strncmp (line,"!!!",3) == 0) //Looks for the !!! datalink header
      {
        rll=Search_Int(line,"RLL:");   // Roll command
        pch=Search_Int(line,"PCH:");   // Picth command

        thr=Search_Int(line,"THR:");   // Throttle command

        gps=Search_Int(line,"GPS:");   // GPS fix status
        cwp=Search_Int(line,"WPN:");   // Current waypoint
        dst=Search_Int(line,"DST:");   // Distance from the waypoint
        bto=Search_Int(line,"BER:");   // Bearing to
        ack=Search_Int(line,"ACK:");   // Acknowledge WP change
        datain=1;

        }

   rs_flush();  // flush buffer
   delay_ms(50);

   if((ack>2)) // Send the switching mode command (RTH, Next_Wp, Prev_Wp) then return to wp mode
   { mode=2;
   }

   sprintf(s, "&&&LAT:%ld,LON:%ld,ALT:%d,ASP:%d,GSP:%d,PHI:%f,THE:%f,GCS:%d,MOD:%d+++\n",
   (long)(state.latitude*100000),(long)(state.longitude*100000),(int)state.altitude,(int)(state.vcas),(int)ground_speed,
   state.phi,state.theta,(int)state.psi,mode);
   rs_putstr(s);

   delay_ms(50);

  }
   SDL_mutexV ( mut_ardupilot );  // unlock variable datas received

   set_roll=rll;
   set_pitch=pch;
   set_throttle=thr;
   delay_ms(100);
   ltime += 0.10;
  }
}
/////////////////////////////////////////////////////////////////////////
// Thread to Receive UDP data from Flight gear and send control commands
/////////////////////////////////////////////////////////////////////////
int ReceiveFG(void *data)
{
    while(1) {

   SDL_mutexP ( mut_flightgear );  // Lock Flight Gear variable before accessing network
	// Receive the Datalink FDM from Flight Gear
	received = recvfrom(sock, buffer, sizeof(buf), 0,
					(struct sockaddr *) &from, &echolen);

		FGNetFDM2Props( &buf);

    // Positions
		state.longitude = buf.longitude*rad2deg; // geodetic (deg)
		state.latitude = buf.latitude*rad2deg;   // geodetic (deg)
		state.altitude = buf.altitude;   // above sea level (meters)

		state.phi = buf.phi*rad2deg;            // roll (deg)
		state.theta = buf.theta*rad2deg;        // pitch (deg)
		state.psi = buf.psi*rad2deg;            // yaw or true heading (deg)
		state.alpha = buf.alpha*rad2deg;        // angle of attack (deg)
		state.beta = buf.beta*rad2deg;		    // side slip angle (deg)

    // Velocities
		state.phidot = buf.phidot*rad2deg;       // roll rate (deg/sec)
		state.thetadot = buf.thetadot*rad2deg;   // pitch rate (deg/sec)
		state.psidot = buf.psidot*rad2deg;       // yaw rate (deg/sec)

     	state.vcas = buf.vcas*kts2ms;           // calibrated airspeed (m/s)

     	state.v_north = buf.v_north;     // north velocity in local/body frame
     	state.v_east = buf.v_east;       // east velocity in local/body frame

    // Accelerations
		state.A_X_pilot = buf.A_X_pilot; // X accel in body frame ft/sec^2
		state.A_Y_pilot = buf.A_Y_pilot; // Y accel in body frame ft/sec^2
		state.A_Z_pilot = buf.A_Z_pilot; // Z accel in body frame ft/sec^2

		if(qfu==0) // Runway heading lock
			{ qfu=state.psi;
              ref_qfe=state.altitude;
            }

		if (received > 0) {

       // Receive the control commands from Flight Gear
			received = recvfrom(sock2, buffer_ctrl, sizeof(bufctrl), 0,
						(struct sockaddr *) &ctrlinclient,
						&echolen);

			FGProps2NetCtrls(&bufctrl);

          switch(mode)
          {
            case 0:   // manual
                 break;
            case 1:   // autostab
            // "PID gphi gtheta gpsi gthr
            // "EFF pitrim rltrim gstrim
       			 bufctrl.aileron = (-state.phi*gphi)+(state.phidot*gphidot)+ailtrim;
  		         bufctrl.elevator = state.theta*gtheta-rudtrim;
       	 	     bufctrl.rudder = state.psidot*gpsi;
                 bufctrl.throttle[0] = gthr;
		 	     break;
            case 2:   // wp
                 bufctrl.aileron = (-state.phi*gphi)+(state.phidot*gphidot)+ailtrim;
                 bufctrl.elevator = state.theta*gtheta-rudtrim;
       	 	     bufctrl.rudder = -state.psidot*gpsi;
               //  bufctrl.rudder_trim = ((float)set_roll/rdtrim);
		 	     bufctrl.elevator_trim = -((float)set_pitch/pitrim);
		 	     bufctrl.aileron_trim = ((float)set_roll/rltrim);
		 	     bufctrl.throttle[0] = ((float)set_throttle)/gstrim;
                 break;
            case 3:   // rth
                 bufctrl.aileron = (-state.phi*gphi)+(state.phidot*gphidot)+ailtrim;
                 bufctrl.elevator = state.theta*gtheta-rudtrim;
       	 	     bufctrl.rudder = -state.psidot*gpsi;
             //    bufctrl.rudder_trim = ((float)set_roll/rdtrim);
		 	     bufctrl.elevator_trim = (((float)set_pitch*deg2rad)/pitrim);
		 	     bufctrl.aileron_trim = (((float)set_roll*deg2rad)/rltrim);
		 	     bufctrl.throttle[0] = ((float)set_throttle)/gstrim;
                 break;
          }
       if ((mode > 0) && ((datain==1)||(com==1)) )
       {
 	      // send the control commands to Flight Gear
           FGNetCtrls2Props(&bufctrl);

		 	received = sendto(sock, buffer_ctrl, sizeof(bufctrl), 0,
 		   (struct sockaddr *) &ctrlclient, sizeof(ctrlclient));

			if (received <0) {
				printf("sendto FAIL\n");
			}
			datain=0;
      }
//    ground_speed = sqrt((state.v_north*state.v_north) + (state.v_east*state.v_east));
      ground_speed = state.vcas;

   SDL_mutexV ( mut_flightgear );  // Unlock Flightgear datas

    delay_ms(20);
			ltime += 0.020;
		}
	}
}

///////////////////////////////////////////////////////
////////////////////////  MAIN ////////////////////////
///////////////////////////////////////////////////////

int main(int argc, char *argv[])
{    struct Fl_raw packet;

  SDL_Init(SDL_INIT_VIDEO);

      //create mutexes for accessing datalink and display
   mut_ardupilot = SDL_CreateMutex();
   mut_flightgear = SDL_CreateMutex();

    if ( mut_ardupilot == NULL )
    	printf( "Failed to create mut_ardupilot!\n");
    if ( mut_flightgear == NULL )
    	printf( "Failed to create mut_flightgear!\n" );

   readsetup();         // Read the setup file "afgcinit.txt"
   initFG();            // Init the network links
   InitDatalink();      // Init the serial com links
   delay_ms(1000);
   rs_flush();  // flush buffer
   delay_ms(500);
   // create threads
   th_routerFGS=SDL_CreateThread(ReceiveFG,NULL);
   th_Ardulinker=SDL_CreateThread(Ardulinker,NULL);

  Fl_Window *window = new Fl_Window(400,220, "ArduFlighGearControl by Jean-Louis Naudin");
  Fl_Menu_Bar menubar(0,0,400,18);
//  menubar.add("&Prev Wp", 0, mode_PrevWp);
  menubar.add("&Reload", 0, reload);
  menubar.add("&About", 0, aide);
  menubar.add("&Quit", 0, Quit_CB);

  Fl_Button Manual(40,110,60,25,"Manual");
  Manual.callback (mode_manual);
  Fl_Button ModeWp(110,110,60,25,"AP Run");
  ModeWp.callback (mode_wp);
  Fl_Button NextWp(180,110,60,25,"Next wp");
  NextWp.callback (mode_NextWp);
  Fl_Button ModeRTH(250,110,60,25,"RTH");
  ModeRTH.callback (mode_rth);

  Fl_Value_Output *lat= new Fl_Value_Output(40,25,70,20,"Lat:");
  Fl_Value_Output *lon= new Fl_Value_Output(155,25,70,20,"Lon:");
  Fl_Value_Output *alt= new Fl_Value_Output(260,25,35,20,"Alt:");
  Fl_Value_Output *com= new Fl_Value_Output(350,25,12,20,"COM:");

  Fl_Value_Output *Pt= new Fl_Value_Output(40,50,30,20,"ASP:");
  Fl_Value_Output *gsp= new Fl_Value_Output(112,50,30,20,"GSP:");
  Fl_Value_Output *gcs= new Fl_Value_Output(185,50,30,20,"CRS:");
  Fl_Value_Output *mod= new Fl_Value_Output(260,50,12,20,"MOD:");

  Fl_Value_Output *phi= new Fl_Value_Output(40,75,65,20,"phi:");
  Fl_Value_Output *the= new Fl_Value_Output(155,75,65,20,"the:");

  if(com>0)
  { Fl_Value_Output *ack= new Fl_Value_Output(350,75,12,20,"Ack:");
    Fl_Value_Output *gps= new Fl_Value_Output(350,50,12,20,"gps:");

    Fl_Value_Output *cwp= new Fl_Value_Output(45,150,70,20,"CWp:");
    Fl_Value_Output *dst= new Fl_Value_Output(160,150,70,20,"DST:");
    Fl_Value_Output *bto= new Fl_Value_Output(300,150,70,20,"Bto:");

    Fl_Value_Output *rll= new Fl_Value_Output(45,185,70,20,"Rll:");
    Fl_Value_Output *pch= new Fl_Value_Output(160,185,70,20,"Pch:");
    Fl_Value_Output *thr= new Fl_Value_Output(300,185,70,20,"Thr:");

    packet.rll=rll;
    packet.pch=pch;
    packet.thr=thr;
    packet.gps=gps;
    packet.ack=ack;
    packet.cwp=cwp;
    packet.dst=dst;
    packet.bto=bto;
  }
  packet.phi=phi;
  packet.theta=the;

  packet.gcs=gcs;
  packet.mod=mod;
  packet.com=com;
  packet.lat=lat;
  packet.lon=lon;
  packet.Pt=Pt;
  packet.alt=alt;
  packet.gsp=gsp;

  Fl::add_timeout(1.0,Refresh,(void*)&packet);  // Refresh the displayed datas

  window->end();
  window->show();

  return Fl::run();
}
