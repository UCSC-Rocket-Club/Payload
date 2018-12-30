 /**
 * @file ALC.c
 *
 * @brief      Actuated landing control (ALC)--Use a servo to rotate the rover until it's upright.
 *
 * @author     Olivia Wong, parts taken from rc_test_mpu example
 *
 * @date       12/29/2018
 * 1. make loop to PRINT DOWN ACCEL
 * 2. RESET SERVO to 0
 * 3.
 */

#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <rc/mpu.h>
#include <rc/time.h>
#include <unistd.h>

#define I2C_BUS 2 // Bus for Beaglebone blue

static int running = 0;
static unsigned int accel_read_interval = 500000; // Microseconds
static int accel_reads = 5; // # of times to read accelerometer before averaging

rc_mpu_data_t mpu_data; // Struct to hold new MPU data

// Interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy) {
	running = 0;
	return;
}

double read_accel() {
	/*
	* It's noisy and fluctuates a lot, so we average out the last 'accel_reads' readings
	*/

	double readings[accel_reads]; 
	double average = 0;

	printf("Accelerometer Z readings: ");

	for (int i=0; i<accel_reads; i++) {
		if(rc_mpu_read_accel(&mpu_data)<0) {
			printf("read accel data failed\n");
		}
		// We only care about the Z acceleration which is accel[2]
		printf("%f, ", mpu_data.accel[2]);
		average += mpu_data.accel[2];
		usleep(accel_read_interval); // Wait to take next reading
	}

	average /= accel_reads; // Take the average
	printf("\nAverage: %f\n", average);
	return average;

}
int main(int argc, char *argv[]) {
	rc_mpu_config_t mpu_conf = rc_mpu_default_config();

	if(rc_mpu_initialize(&mpu_data, mpu_conf)) {
		fprintf(stderr,"rc_mpu_initialize_failed\n");
		return -1;
	}

	read_accel();
	// Set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;
/*
	while (running) {
		printf("%6.2f %6.2f %6.2f |",
		mpu_data.accel[0],\
		mpu_data.accel[1],\
		mpu_data.accel[2]);

		}*/
}
