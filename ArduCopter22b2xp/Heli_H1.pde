/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// JLN update for Helicopter type H-1 (i.e. Raptor, Vario Benzin) and HIL tested with AeroSIM-RC v3.81
// very ALPHA version must be used only on simulator in HIL mode - for explorers only

#if FRAME_CONFIG ==	HELI_FRAME_H1

static int16_t  coll_curve[5] = {0, 25, 50, 75, 100};
static int16_t  throttle_curve[5] = {0, 25, 50, 75, 100};

static void init_motors_out()
{

}

//
// heli_move_swash - moves swash plate to attitude of parameters passed in
//                 - expected ranges:
//                       roll : -4500 ~ 4500
//                       pitch: -4500 ~ 4500
//                       collective: 0 ~ 1000
//                       yaw:   -4500 ~ 4500
//

static void output_motors_armed()
{
	int out_min = g.rc_3.radio_min;
	int out_max = g.rc_3.radio_max;

	// Throttle is 0 to 1000 only
	g.rc_3.servo_out 	= constrain(g.rc_3.servo_out, 0, MAXIMUM_THROTTLE);

	if(g.rc_3.servo_out > 0)
		out_min = g.rc_3.radio_min + MINIMUM_THROTTLE;

	g.rc_1.calc_pwm();
	g.rc_2.calc_pwm();
	g.rc_3.calc_pwm();
	g.rc_4.calc_pwm();

	int roll_out 		= (float)g.rc_1.pwm_out * 6;
	int pitch_out 		= (float)g.rc_2.pwm_out * 6;
	int yaw_out 		= (float)g.rc_4.pwm_out * 6;
        int coll_out            = g.rc_3.pwm_out;
        
        // ensure values are acceptable:
	roll_out = constrain(roll_out, -4500, 4500);
	pitch_out = constrain(pitch_out, -4500, 4500);
	coll_out = constrain(coll_out, 0, 1000);
	yaw_out = constrain(yaw_out, -4500, 4500);
        
        //Serial.printf("roll_out: %d   pitch_out: %d   throttle_out%d    yaw_out: %d\n", roll_out, pitch_out, throttle_out, yaw_out);

	APM_RC.OutputCh(MOT_1, roll_out);
	APM_RC.OutputCh(MOT_2, pitch_out);
	APM_RC.OutputCh(MOT_3, coll_out);
	APM_RC.OutputCh(MOT_4, yaw_out);
}

static void output_motors_disarmed()
{
	if(g.rc_3.control_in > 0){
		// we have pushed up the throttle
		// remove safety
		motor_auto_armed = true;
	}

	// fill the motor_out[] array for HIL use
        motor_out[0] = 1500;
        motor_out[1] = 1500;
        motor_out[2] = g.rc_3.radio_min;
        motor_out[3] = 1500;
        motor_out[5] = 1100;
        motor_out[6] = 1100;
        motor_out[7] = 1100;        
}

static void output_motor_test()
{

}

#endif
