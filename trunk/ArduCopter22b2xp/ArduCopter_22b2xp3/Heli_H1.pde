/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// JLN update for Helicopter type H-1 (i.e. Raptor, Vario Benzin) and HIL tested with AeroSIM-RC v3.81
// very ALPHA version must be used only on simulator in HIL mode - for explorers only

#if FRAME_CONFIG ==	HELI_FRAME_H1

static int16_t  coll_curve[5] = {415, 515, 625, 720, 835};         // collective curve (from Heli 30 AeroSIM-RC)
static int16_t  throttle_curve[5] = {0, 250, 500, 750, 1000};      // throttle curve

static void init_motors_out()
{

}

//
// heli_move_swash - moves swash plate to attitude of parameters passed in
//                 - expected ranges:
//                       roll :     -4500 ~ 4500
//                       pitch:     -4500 ~ 4500
//                       throttle:      0 ~ 1000
//                       collective:    0 ~ 1000
//                       yaw:       -4500 ~ 4500
//

static void output_motors_armed()
{       
        int coll_out;
	int out_min = g.rc_3.radio_min;
	int out_max = g.rc_3.radio_max;

	// Throttle is 0 to 1000 only
	g.rc_3.servo_out 	= constrain(g.rc_3.servo_out, 0, MAXIMUM_THROTTLE);

        int throttle_out = g.rc_3.servo_out;
       
	if(g.rc_3.servo_out > 0)
		out_min = g.rc_3.radio_min + MINIMUM_THROTTLE;

	g.rc_1.calc_pwm();
	g.rc_2.calc_pwm();
	g.rc_3.calc_pwm();
	g.rc_4.calc_pwm();

	int roll_out 		= (float)g.rc_1.pwm_out * 6;
	int pitch_out 		= (float)g.rc_2.pwm_out * 6;
	int yaw_out 		= (float)g.rc_4.pwm_out * 6;
	int thr_out 		= g.rc_3.pwm_out;

        if (thr_out < 250) {    // manage the throttle -> collective curve for the H1 swash plate
             coll_out = ((((coll_curve[1] - coll_curve[0] )/ 250) * (thr_out - throttle_curve[0])) + coll_curve[0]);
          } else if ((thr_out > 250) && (thr_out <= 500)) {
             coll_out = ((((coll_curve[2] - coll_curve[1]) / 250) * (thr_out - throttle_curve[1])) + coll_curve[1]);
          } else if ((thr_out > 500) && (thr_out <= 750)) {            
             coll_out = ((((coll_curve[3] - coll_curve[2]) / 250) * (thr_out - throttle_curve[2])) + coll_curve[2]);
          } else if ((thr_out > 750) && (thr_out <= 1000)) {            
             coll_out = ((((coll_curve[4] - coll_curve[3]) / 250) * (thr_out - throttle_curve[3])) + coll_curve[3]);
            }  
        
        // ensure values are acceptable:
	roll_out = constrain(roll_out, -4500, 4500);
	pitch_out = constrain(pitch_out, -4500, 4500);
	thr_out = constrain(thr_out, 0, 1000);
	coll_out = constrain(coll_out, 0, 1000);
	yaw_out = constrain(yaw_out, -4500, 4500);
        
        //Serial.printf("roll_out: %d   pitch_out: %d   throttle_out%d    yaw_out: %d\n", roll_out, pitch_out, throttle_out, yaw_out);

	APM_RC.OutputCh(MOT_1, roll_out);
	APM_RC.OutputCh(MOT_2, pitch_out);
	APM_RC.OutputCh(MOT_3, thr_out);
	APM_RC.OutputCh(MOT_4, yaw_out);
	APM_RC.OutputCh(MOT_5, coll_out);
}

static void output_motors_disarmed()
{
	if(g.rc_3.control_in > 0){
		// we have pushed up the throttle
		// remove safety
		motor_auto_armed = true;
	}

	// fill the motor_out[] array for HIL use
        motor_out[0] = 1500;                  // servo roll
        motor_out[1] = 1500;                  // servo pitch
        motor_out[2] = g.rc_3.radio_min;      // servo throttle
        motor_out[3] = 1500;                  // servo yaw
        motor_out[5] = 1100;                  // servo collective
        motor_out[6] = 1100;
        motor_out[7] = 1100;        
}

static void output_motor_test()
{

}

#endif
