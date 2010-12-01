/*****        SELF-LEARNING MODE WITH THE ARDUPILOT        *****/
/***** Firmware version 2.4.5 or more by Jean-louis Naudin *****/
/*                    November 13, 2009                        */
/***************************************************************/

Hello ArduPilots,

The Self-Learning mode works only on the version 2.4.5 (or more) and the JLN version ONLY. 
Now, I explain you how to record IN FLIGHT a flight plan to your ArduPilot UAV. 
This new feature is very usefull in the field, because you don't need to have an internet connection (google map) or a PC computer to create yourself a customized waypoints list, your ArduPilot will be able to Self-Learn its own flight plan in flight !!!

A) LEANING MODE :
-----------------
PRE-LAUNCH:
----------
A-1) The transmitter must be switched ON and the control button set in MANUAL MODE,
A-2) The "Remove Before Flight" (RBF) jumper must be connected,
A-3) Connect the Lipo battery to the ESC of your plane,
A-4) Wait the GPS-Fix (the blue led goes fix) and the Ailerons/Rudder moves quickly,
A-5) Remove the RBF, 
A-6) Then RECONNECT THE RBF.

Your ArduPilot is now ready to Self-Learn your flight plan.

B)Just before the launch : 
-------------------------
The previous reset procedure in the v2.4.4 is now automatic and not required before the launch. This is done when the first wp is recorded in flight.

C) RECORDING THE FLIGHT PLAN :
-----------------------------
C-1) The control button must be set in MANUAL MODE. Launch you plane and fly, when you want to record a waypoint, you simply need to turn the control button of your transmitter from MANUAL MODE TO WAYPOINT MODE (the middle position) then quickly back to the MANUAL MODE again. The plan will move quickly the ailerons to point you that the WP has been recorded...

So, you may record all the 3D Waypoints (lat, lon, alt) that you desire...

When all the flight plan has been recorded during your flight, land your plane.

D) REPLAYING YOUR FLIGHT PLAN :
-----------------------------
D-1) Disconnect the Lipo Battery
D-2) The RBF jumper is still in place on the ArduPilot and the transmitter ON and set in Manual mode,
D-3) Connect a new charged Lipo Battery
D-4) Wait for the GPS fix (the blue led fix and the quick motion of the servos)
D-5) REMOVE THE RBF jumper,
D-6) Launch your plane and now, you may run the recorded flight plan when you want by only turn the control button to WAYPOINT MODE....


Enjoy,
Best Regards,
Jean-Louis Naudin
http://diydrones.com/profile/JeanLouisNaudin
Email: JeanLouisN777@diydrones.com
Opensource project : http://code.google.com/p/ardupilotdev/

