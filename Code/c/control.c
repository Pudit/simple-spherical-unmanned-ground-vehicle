/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */

#include <stdio.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <math.h>

#include <unistd.h> // for isatty()
#include <getopt.h>
#include <stdlib.h> // for atoi
//io
#define motor_enable	1,25
#define motor_dir		1,17
#define I2C_BUS 		2



//control
#define PRINTF_HZ		50
#define SAMPLE_RATE_HZ		100
#define DT			0.01			// 1/sample_rate
#define D3_KP	0.351*4
#define	D3_KI	0.172*4
#define D3_KD	0.178*4
// #define D3_KP 0
// #define D3_KI 0
// #define D3_KD 0

#define D4_KP	150 //75
#define	D4_KI	0
#define D4_KD	0

//maxon
#define max_motor_speed		2000
#define max_motor_torque	30
#define gearbox 		15.1666666666666
#define radius_robot	0.2
#define encoder_count	2000 
#define encoder_offset	0.36
#define roll_offset 1.3// -1.6 floor 9 -1.1 m
//math
#define M_PI 3.14159265358979323846

//etc
#define V_NOMINAL		7.4
#define BATTERY_CHECK_HZ	5
// function declarations
int ss = 0 ;
char chL = 'B' ;
char chR = 'A' ;
int f = 50 ;


typedef struct core_state_t{
	double roll;			///< roll angle of vehicle [rad]
	double pitch;			///< pitch angle of vehicle [rad]
	double yaw;			    ///< yaw angle of vehicle [rad]
	double roll_dot;		///< roll angle rate of vehicle [rad/s]
	double pitch_dot;		///< pitch angle rate of vehicle [rad/s]
	double vBatt;			///< voltage of battery at DC jack
	double d3_u;			///< result of roll error that generate by PID function
	double d4_u;			///< result of distance error that generate by PID function
	double duty;			///< duty cycle for control maxon motor
	double roll_servo;		///< roll angle servo
	double motor_speed;		///< read value of maxon motor speed by motor drive
	double motor_position;	///< read value of maxon motor position by encoder
	int checkpoint;			///< use in x,y command check condition
	double lenght; 		    ///< length of straight forward/backward direction
	double distance;		///< distance of turn left/right direction
	
} core_state_t;


typedef struct setpoint_t{
	double roll;			///< roll angle of vehicle == of servo 
	double omega;		    ///< angular velocity of vehicle
	double distance; 		///< forward/backward distance
	double radius_c; 		///< radius of curvature
	double round_n ; 		///< number of turning round
	double x_pos ; 		    ///< position x
	double y_pos ;			///< position y
	double theta_h ; 		///< yaw angle of vehicle at (x,y)
	
}setpoint_t;

typedef struct coordinat_t{
	double x;				///< position at x
	double y;				///< position at y
	double cum_theta;		///< cumulation of theta angel
	double motor_pos_tmp;	///< previos motor position
}coordinat_t;

///Global variable
///static functions
static setpoint_t setpoint;
static core_state_t cstate;
static rc_mpu_data_t data;
static coordinat_t coor;
static rc_filter_t D3 = RC_FILTER_INITIALIZER;
static rc_filter_t D4 = RC_FILTER_INITIALIZER;
FILE *fptr;

static 	double dis ;  ///distance of travel [n] round at [radius_c]
static	double moom ; ///roll angel at [radius_c]
static 	double pitch_prev;	
	
	
static	double l ;          ///straight travel lenght in [x,y] 
static	double r_l ;        ///turning travel lenght in [x,y] 
static	double theta_r ;    ///setpoint.theta_h
static	int sign_R ;        ///sign of r_l
static	double l_dis;       ///motor position afer travel [l]

static int check_steady;    ///count time to make sure vehicle steady before start moving


void on_pause_press();      ///pause botton function
void on_pause_release();    ///pause botton function
void update_coor();			///update coordination

static void __balance_controller(void);		///< mpu interrupt routine
static void* __printf_loop(void* ptr);		///< background thread
static void* __battery_checker(void* ptr);	///< background thread

static void __print_usage(void)
{
	printf("\n");
	printf(" Options\n");
	printf(" -w {angular velocity of vehicle}	range [0,2,000](rpm)\n");
	printf(" -r {roll angel of vehicle}	        range [-45,45] (DEG)\n");
    printf(" -D {forward/backward distance}	    	range [-inf,inf] (m)\n");
    printf(" -R {turning raduis}	            	range [1,inf] (m)\n");
    printf(" -n {number of turning round}	    	range [1,inf] \n");
    printf(" -x {x position}	                range [-inf,inf]-{0} (m)\n");
    printf(" -y {y position}	                range [-inf,inf]-{0} (m)\n");
    printf(" -t {yaw angle of vehicle at (x,y)}	range [-pi,pi] (rad)\n");
	printf(" -h             Print this help messege \n\n");
	printf("sample use \n");
	printf("pd -r 0 -w 300 \n");
    printf("pd -D -2 \n");
    printf("pd -R 1 -n 2 \n");
    printf("pd -x 1 -y 2 -t 1 \n");
}



/**
 * This template contains these critical components
 * - ensure no existing instances are running and make new PID file
 * - start the signal handler
 * - initialize subsystems you wish to use
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 *
 * @return     0 during normal operation, -1 on error
 */
 
int main(int argc, char *argv[])
{
	int c;
	pthread_t printf_thread = 0;
	pthread_t battery_thread = 0;
	bool adc_ok = true;
	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "r:w:D:R:n:x:y:t:")) != -1){
		switch (c){
		case 'r': 
			 setpoint.roll = atof(optarg);
			break;
		case 'w':
			setpoint.omega = atof(optarg);
			break;
		case 'D':
			setpoint.distance = atof(optarg);
			break;
		case 'R' :
			setpoint.radius_c = atof(optarg) ;
			break ;
		case 'n' :
			setpoint.round_n = atof(optarg) ;
			break ; 
		case 'x' :
			setpoint.x_pos = atof(optarg) ;
			break ;
		case 'y' :
			setpoint.y_pos = atof(optarg) ;
			break ;
		case 't' :
			setpoint.theta_h = atof(optarg) ;
			break ; 
		case 'h'://help
			__print_usage();
			return -1;
			break;
		default:
			__print_usage();
			return -1;
			break;
		}
	}
	
	
	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}
	////init
	rc_pinmux_set(GPS_HEADER_PIN_3, PINMUX_PWM);
	rc_pinmux_set(GPS_HEADER_PIN_4, PINMUX_PWM);
	rc_gpio_init(motor_enable, GPIOHANDLE_REQUEST_OUTPUT);
	rc_gpio_init(motor_dir, GPIOHANDLE_REQUEST_OUTPUT);
	rc_pwm_init(ss, f);
	rc_servo_init();
	
	if(rc_adc_init()==-1){
		fprintf(stderr, "failed to initialize adc\n");
		adc_ok = false;
	}
	rc_encoder_init();

	 rc_mpu_config_t mpu_config = rc_mpu_default_config();
	 mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
     mpu_config.i2c_bus = I2C_BUS;
     if(rc_mpu_initialize(&data, mpu_config)){
     	fprintf(stderr,"rc_mpu_initialize_failed\n");
     	return -1;
     }
     
     
     
     if(rc_mpu_initialize_dmp(&data, mpu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
		rc_led_blink(RC_LED_RED, 5, 5);
		return -1;
	}

    // if gyro isn't calibrated, run the calibration routine
	if(!rc_mpu_is_gyro_calibrated()){
		printf("Gyro not calibrated, automatically starting calibration routine\n");
		printf("Let your MiP sit still on a firm surface\n");
		rc_mpu_calibrate_gyro_routine(mpu_config);
	}
	// initialize pause button
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)==-1){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}

	if(rc_pthread_create(&printf_thread, __printf_loop, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start battery thread\n");
		return -1;
	}
	

	if (printf_thread) rc_pthread_timed_join(printf_thread, NULL, 1.5);
	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);
	
	// set up D3 roll controller
	if(rc_filter_pid(&D3, D3_KP, D3_KI, D3_KD, 4*DT, DT)){
		fprintf(stderr,"ERROR in rc_balance, failed to make steering controller\n");
		return -1;
	}
	// set up D3 distance controller
	if(rc_filter_pid(&D4, D4_KP, D4_KI, D4_KD, 4*DT, DT)){
		fprintf(stderr,"ERROR in rc_balance, failed to make steering controller\n");
		return -1;
	}
	
	
	if (adc_ok) {
		if(rc_pthread_create(&battery_thread, __battery_checker, (void*) NULL, SCHED_OTHER, 0)){
			fprintf(stderr, "failed to start battery thread\n");
			return -1;
		}
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();
	rc_mpu_set_dmp_callback(&__balance_controller);


	//printf("\nPress and release pause button to turn green LED on and off\n");
	//printf("hold pause button down for 2 seconds to exit\n");

	// Keep looping until state changes to EXITING
	//write txt

	fptr = fopen("/var/lib/cloud9/pd/data.txt","w");
	fprintf(fptr,"motor_position,roll,servo_position,motor_speed,pitch,yaw,x,y\n");
	
	
	rc_set_state(RUNNING);
	rc_gpio_set_value(motor_enable,0);
	rc_gpio_set_value(motor_dir,0);
	rc_servo_send_pulse_us (8,2000);
	cstate.checkpoint = 1;
	check_steady=0;
	
	rc_usleep(10000000);
	while(rc_get_state()!=EXITING){
		// do things based on the state
		if(rc_get_state()==RUNNING){
			
		rc_usleep(10000000);
		}
		
	}

	// turn off LEDs and close file descriptors
	printf("\nend\n");
	
	if (battery_thread) rc_pthread_timed_join(battery_thread, NULL, 1.5);
	if (printf_thread) rc_pthread_timed_join(printf_thread, NULL, 1.5);
	
	
	rc_mpu_power_off();
	//rc_pwm_set_duty(ss,chL,1500/20000);
	//rc_pwm_set_duty(ss,chR,1500/20000);
	fclose(fptr);


	//clean up
	rc_filter_reset(&D3);
	rc_filter_free(&D3);
	rc_filter_reset(&D4);
	rc_filter_free(&D4);
	rc_gpio_set_value(motor_dir,0);
	rc_gpio_set_value(motor_enable,0);
	rc_gpio_cleanup(motor_enable);
	rc_gpio_cleanup(motor_dir);
	//rc_pinmux_set_default();
	rc_encoder_eqep_cleanup();
	rc_adc_cleanup();
	rc_servo_cleanup();
	//rc_pwm_set_duty(ss,ch,0.1);
	rc_pwm_cleanup(ss);
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	
	return 0;


}

/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
	if(rc_get_state()==RUNNING)	rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/**
* If the user holds the pause button for 2 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/
void on_pause_press()
{
	int i;
	const int samples = 100; // check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}


static void* __printf_loop(__attribute__ ((unused)) void* ptr)
{
	rc_state_t last_rc_state, new_rc_state; // keep track of last state
	last_rc_state = rc_get_state();
	while(rc_get_state()!=EXITING){
		new_rc_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_rc_state==RUNNING && last_rc_state!=RUNNING){
			printf("  roll   |");
			printf("  pitch  |");
			printf("  yaw    |");
			printf("  vBatt  |");
			printf("  d3_u  |");
			printf("  d4_u  |");
			printf("  duty  |");
			printf("  pitch_dot  |");
			// printf(" motor_speed  |");
			printf("motor_pos|");
			printf("servo_pos|");
			printf("set_roll |");
			printf("set_omega|");
		//	printf(" lenght  |");
		//	printf("distance |");
			printf("checkpoint|");
			printf(" x  |");
			printf(" y  |");
			// printf(" m_temp  |");
			printf("\n");
		}
		else if(new_rc_state==PAUSED && last_rc_state!=PAUSED){
			printf("\nPAUSED: press pause again to start.\n");
		}
		last_rc_state = new_rc_state;

		// decide what to print or exit
		if(new_rc_state == RUNNING){
			
			printf("\r");
			printf("%7.3f  |", cstate.roll*RAD_TO_DEG);
			printf("%7.3f  |", cstate.pitch*RAD_TO_DEG);
			printf("%7.3f  |", cstate.yaw*RAD_TO_DEG);
			printf("%7.3f  |", cstate.vBatt);
			printf("%7.3f |", cstate.d3_u*RAD_TO_DEG);
			printf("%7.3f |", cstate.d4_u);
			printf("%7.3f |", cstate.duty);
			printf("%7.3f |", cstate.pitch_dot);
			// printf("%7.3f  |", cstate.motor_speed);
			printf("%7.3f  |", cstate.motor_position);
			printf("%7.3f  |", cstate.roll_servo);
			printf("%7.3f  |", setpoint.roll);
			printf("%7.3f  |", setpoint.omega);
			// printf("%7.3f  |", cstate.lenght);
			// printf("%7.3f  |", cstate.distance);
			printf("%d  |", cstate.checkpoint);
			printf("%7.3f  |", coor.x);
			printf("%7.3f  |", coor.y);
			// printf("%7.3f  |", coor.motor_pos_tmp);
			
			
			fflush(stdout);
		}
		rc_usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
}

static void __balance_controller(void){
	double dutyR,dutyL;
	double error;
	double d_error;
	// double pitch_dot_offset;
	
	rc_servo_power_rail_en(1);

	//delay 5 s
	if (check_steady < 500){
		check_steady++;
	}
	if (check_steady == 500){
		rc_gpio_set_value(motor_enable,1);
	}

	///read data from sensors
	cstate.roll = data.dmp_TaitBryan[1];		///[rad]
	cstate.pitch = data.dmp_TaitBryan[0];		///[rad]
	cstate.yaw = data.dmp_TaitBryan[2];			///[rad]
	cstate.roll_dot = data.raw_gyro[1]; 			///[deg/s]
	cstate.pitch_dot = (cstate.pitch-pitch_prev)/0.01;			///[rad/s]
	pitch_prev = cstate.pitch;
	// pitch_dot_offset = cstate.pitch_dot*gearbox*(30/M_PI); ///RPM
	//maxon
	cstate.motor_position = (rc_encoder_read(1)*2*M_PI*radius_robot)/(encoder_count*gearbox);
	cstate.motor_speed = rc_adc_read_raw(1);
	
	
	
	//check distance 
	if (fabs(setpoint.distance) > 0.001 ){
		if ((fabs(cstate.motor_position)+0.3) < fabs(setpoint.distance)){
			setpoint.roll = roll_offset + cstate.d4_u  ;
			setpoint.omega = 350; //+ pitch_dot_offset;
		}
		else{
			setpoint.roll = roll_offset;
			setpoint.omega = 0;
		}
		
		
		/// check move forward or backward
		if (setpoint.distance > 0){
		rc_gpio_set_value(motor_dir,0);
	}
		else{
		rc_gpio_set_value(motor_dir,1);
	}
	d_error = -coor.x;
	}
	
	
	//check radians and round
	
	if (fabs(setpoint.radius_c) > 0 ){
		moom = atan(.2/setpoint.radius_c) ; 
		// dis = setpoint.round_n * 2 * M_PI * .2 / sin(moom) *setpoint.radius_c;
		// dis = setpoint.round_n * 2 *M_PI ;
		dis = setpoint.round_n * 2 * M_PI /cos(moom) ;
		if (fabs(moom) < 20){
			if (fabs(dis) > 0.001 ){
			 			if (fabs(cstate.motor_position) < fabs(dis)){
			 				setpoint.roll = moom*180/M_PI + roll_offset;
			 				setpoint.omega = 350;// + pitch_dot_offset;
			 		}
			 			else {
			 				setpoint.roll = roll_offset;
			 				setpoint.omega = 0;
			 		}
			 			/// check move forward or backward
			 			if (moom > 0){
			 			rc_gpio_set_value(motor_dir,0);
			 		}
			 			else{
			 			rc_gpio_set_value(motor_dir,1);
			 		}
			 	}	
			 } }

	// x y position



	if ((fabs(setpoint.x_pos) > 0) && (fabs(setpoint.y_pos) > 0) && cstate.checkpoint == 1) {
		theta_r = setpoint.theta_h ;
		r_l  = setpoint.x_pos/(1-cos(theta_r)) ;
		

		if ((0 <= r_l) && (r_l < 0.5495)) {
		    r_l = 0.5495 ;
		    theta_r = acos(1 - (setpoint.x_pos)/r_l) ;
		}
		else if((0 > r_l) && (r_l > -0.5495)){
		    r_l = -0.5495 ;
		    theta_r = acos(1 - (setpoint.x_pos)/r_l) ;
		}
		   
		// sign_R = sign(R) ;
		if(r_l > 0) {
			sign_R = 1 ;
		}
		else {
			sign_R = -1 ; 
		}
		r_l = fabs(r_l) ; 

		l = setpoint.y_pos - r_l*sin(theta_r) ; 
		cstate.lenght = l;
		
		if (fabs(l) > 0.001 ){
		if (fabs(cstate.motor_position) < fabs(l)){
			setpoint.roll = roll_offset;
			setpoint.omega = 350;// + pitch_dot_offset;
		}
		else {
			setpoint.roll = 0;
			setpoint.omega = 0;
			cstate.checkpoint = 0;
			l_dis = cstate.motor_position;
			moom  = atan(.2/r_l*sign_R) ; 
			cstate.distance = r_l * sign_R * theta_r;

		}
		/// check move forward or backward
		if (l > 0){
		rc_gpio_set_value(motor_dir,0);
	}
		else{
		rc_gpio_set_value(motor_dir,1);
	}
		d_error = -coor.x;
	}	
		

	}
	if (fabs(moom) < 20 && cstate.checkpoint == 0 ){
			if (fabs(cstate.distance) > 0.001 ){
			 			if (fabs(cstate.motor_position - l_dis ) < fabs(cstate.distance)){
			 				setpoint.roll = round(moom*1000*180/M_PI)/1000 + roll_offset;
			 				setpoint.omega = 350;// + pitch_dot_offset;
			 				//printf("hello");
			 		}
			 			else {
			 				setpoint.roll = roll_offset;
			 				setpoint.omega = 0;
			 				//printf("gg");
			 		}
			 			/// check move forward or backward
			 			rc_gpio_set_value(motor_dir,0);
			}
			d_error = sqrt((coor.y-cstate.lenght)*(coor.y-cstate.lenght)+(coor.x-r_l)*(coor.x-r_l))-r_l;	
	}

	///calculate commanded speed of motor in duty
	cstate.duty = 0.1 + (0.8*setpoint.omega/max_motor_speed);
	if (cstate.duty > 0.9){
		cstate.duty = 0.9;
	}
	if (cstate.duty < 0.1){
		cstate.duty = 0.1;
	}
	/**********************************************************
	* distance controller D4
	* move the 
	***********************************************************/
	cstate.d4_u = rc_filter_march(&D4,d_error);
	

	
	/**********************************************************
	* gama (steering) controller D3
	* move the setpoint gamma based on user input like phi
	***********************************************************/
	error = (setpoint.roll*DEG_TO_RAD)- cstate.roll;
	cstate.d3_u = rc_filter_march(&D3,error);
	
	
	
	/**********************************************************
	* Send signal to motors
	***********************************************************/
	int offset = 20 ;
	double k_e = 0.88;
	// cstate.roll_servo = (setpoint.roll + cstate.d3_u*RAD_TO_DEG);
	// dutyL = 1500 + ((setpoint.roll*DEG_TO_RAD) + cstate.d3_u)*10*RAD_TO_DEG;
	// dutyR = 1500 - (k_e*((setpoint.roll*DEG_TO_RAD) + cstate.d3_u)*10*RAD_TO_DEG) + offset;
	
	//new
	cstate.roll_servo = (setpoint.roll + cstate.d3_u*RAD_TO_DEG );
	dutyL = 1500 + (cstate.roll_servo*10);
	dutyR = 1500 - (k_e*cstate.roll_servo*10) + offset;
	
	
	///limit at 30 deg
	if (dutyL > 1800){
		dutyL = 1800;
	}
	if (dutyL < 1200){
		dutyL = 1200;
	}
	if(dutyR > 1784) {dutyR = 1784 ;}
	if(dutyR < 1256) {dutyR = 1256 ;}
	

	
	
	//motor maxon
	
	
	rc_servo_send_pulse_us(8,cstate.duty*10000);
	//servo
	rc_pwm_set_duty(ss,chL,dutyL/20000);
	rc_pwm_set_duty(ss,chR,dutyR/20000);
	
	update_coor();
	//write data
	fprintf(fptr,"%f,%f,%f,%f,%f,%f\n",cstate.motor_position,cstate.roll*RAD_TO_DEG,cstate.roll_servo,setpoint.omega,cstate.pitch,cstate.yaw,coor.x,coor.y);
	
	return;
	
}

/**
 * Slow loop checking battery voltage. Also changes the D1 saturation limit
 * since that is dependent on the battery voltage.
 *
 * @return     nothing, NULL poitner
 */
static void* __battery_checker(__attribute__ ((unused)) void* ptr)
{
	double new_v;
	while(rc_get_state()!=EXITING){
		new_v = rc_adc_dc_jack();
		// if the value doesn't make sense, use nominal voltage
		if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL;
		cstate.vBatt = new_v;
		rc_usleep(1000000 / BATTERY_CHECK_HZ);
	}
	return NULL;
}

/// update coordinate
void update_coor(){
	double R_real ; 
	double theta_real ;   
	double dx ; 
	double dy ; 
	R_real = 0.2/tan(cstate.roll);
	theta_real = (cstate.motor_position - coor.motor_pos_tmp)/R_real ;
	dx = R_real*(1-cos(theta_real)) ; 
	dy = R_real*sin(theta_real) ;
	coor.x = coor.x + dx*cos(coor.cum_theta) + dy*sin(coor.cum_theta) ;
	coor.y = coor.y - dx*sin(coor.cum_theta) + dy*cos(coor.cum_theta) ;
	coor.cum_theta = coor.cum_theta + theta_real ; 
	coor.motor_pos_tmp = cstate.motor_position;
}