/* @title ALC.c
 *
 * @brief      Actuated landing control (ALC)--Use a servo to rotate the rover until it's upright.
 *
 * @author     Olivia Wong, parts taken from rc_test_mpu example
 *
 * @date       12/29/2018
 * 1. make loop to PRINT DOWN ACCEL
 * 2. keep
 * 3.
 */

#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <getopt.h>
#include <rc/adc.h>
#include <rc/mpu.h>
#include <rc/time.h>
#include <rc/servo.h>
#include <unistd.h>

#define G 9.8 // Gravity m/s^2
#define ACCEL_READS 3 // # of times to read accelerometer before averaging
#define ACCEL_READ_INTERVAL 500000 // Microseconds
#define X 0
#define Y 1
#define Z 2
#define SERVO_POS 1.0
#define NUM_PULSES 5
#define PULSE_FREQ_HZ 50
#define PULSE_WIDTH_US 12 // Microseconds. idk if this changes anything
#define SERVO_CH 8


static int running = 0;
static rc_mpu_data_t mpu_data; // Struct to hold new MPU data

// Interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy) {
	running = 0;
	return;
}
int run_motor(double direction) {
	// Init servo
	// Read adc to make sure battery is connected
	if(rc_adc_init()) {
		fprintf(stderr, "ERROR: failed to run rc_adc_init()\n");
		return -1;
	}
	if(rc_adc_batt() < 6.0) {
		fprintf(stderr,"ERROR: battery disconnected or insufficiently charged to drive servos\n");
		return -1;
	}
	rc_adc_cleanup();

	// Initialize PRU
	if(rc_servo_init()) return -1;

	// Turn on power
	printf("Turning On 6V Servo Power Rail\n");
	rc_servo_power_rail_en(1);

	// Send pulse(s)
	printf("Moving servo to %f...\n", SERVO_POS * direction);

	for (int i=0; i<NUM_PULSES; i++) {
		if(rc_servo_send_pulse_normalized(SERVO_CH, SERVO_POS * direction) == -1) return -1;
		rc_usleep(1000000 / PULSE_FREQ_HZ);
	}

	printf("Turn off/clean up servo\n");

	// Turn off power rail and cleanup
	rc_servo_power_rail_en(0);
	rc_servo_cleanup();
	printf("Finished servo move\n");
	return 1;
}

void *read_accel(double *buf) {
	/*
	* Reads accelerometer Z data, averages last 'ACCEL_READS' readings to reduce fluctuation
	*/
	double sums[3] = {0};

	printf("\nAccelerometer readings: \n");
	printf("    X     |     Y     |     Z    |\n");

	for (int i=0; i<ACCEL_READS; i++) {
		if(rc_mpu_read_accel(&mpu_data)<0) {
			printf("Read accel data failed\n");
		}
		for (int j=0; j<3; j++) {
			// Print X, Y, Z accel data
			printf("%f | ", mpu_data.accel[j]);
			sums[j] += mpu_data.accel[j]; // Take sums to avg later
		}
		printf("\n");

		usleep(ACCEL_READ_INTERVAL); // Wait to take next reading
	}

	printf("\nAverages:\n");
	printf("    X     |     Y     |     Z    |\n");

	for (int i=0; i<3; i++) {
		buf[i] = sums[i] / ACCEL_READS; // Take the average and assign it to buf
		printf("%f | ", buf[i]);
	}
	printf("\n\n");
}

int main(int argc, char *argv[]) {
	// Init sensor reading
	rc_mpu_config_t mpu_conf = rc_mpu_default_config();

	if(rc_mpu_initialize(&mpu_data, mpu_conf)) {
		fprintf(stderr,"rc_mpu_initialize_failed\n");
		return -1;
	}

	double accel_data[3] = {0};

	// Set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;

	while (running) {

		// Read accelerometer
		printf("Starting reading...\n");
		read_accel(accel_data);

		// Determine which way to move based on accel reading
		if (fabs(G - accel_data[Z]) < 0.13) {
			// Z ~ 9.8
			// TODO: test 0.13 value, make sure it works for servo sides
			printf("IM RIGHT SIDE UP\n");
			break;
		} else if(accel_data[X] > 0) {
			printf("TURN ME COUNTER CLOCKWISE\n");
			run_motor(1.0);

		} else {
			printf("TURN ME CLOCKWISE\n");
			run_motor(-1.0);
		}
		rc_usleep(500000); // Sleep half a second to make sure motor is done moving
	}
}