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
#include <rc/mpu.h>
#include <rc/time.h>
#include <unistd.h>

#define G 9.8 // Gravity m/s^2
#define ACCEL_READS 5 // # of times to read accelerometer before averaging
#define ACCEL_READ_INTERVAL 500000 // Microseconds
#define X 0
#define Y 1
#define Z 2

static int running = 0;
static rc_mpu_data_t mpu_data; // Struct to hold new MPU data

// Interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy) {
	running = 0;
	return;
}

void *read_accel(double *buf) {
	/*
	* Reads accelerometer Z data, averages last 'ACCEL_READS' readings to reduce fluctuation
	*/
	double sums[3] = {0};

	printf("Accelerometer readings: \n");
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
		// We only care about the Z acceleration which is accel[2]
		
		usleep(ACCEL_READ_INTERVAL); // Wait to take next reading
	}

	printf("\nAverages:\n");
	printf("    X     |     Y     |     Z    |\n");

	for (int i=0; i<3; i++) {
		buf[i] = sums[i] / ACCEL_READS; // Take the average and assign it to buf
		printf("%f | ", buf[i]);
	}
	printf("\n");

}
int main(int argc, char *argv[]) {
	rc_mpu_config_t mpu_conf = rc_mpu_default_config();

	if(rc_mpu_initialize(&mpu_data, mpu_conf)) {
		fprintf(stderr,"rc_mpu_initialize_failed\n");
		return -1;
	}

	double accel_data[3] = {0};

	read_accel(accel_data);

	// Set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;

	while (running && fabs(G - accel_data[Z]) > 0.1) { 
		// Z !~ 9.8
		read_accel(accel_data);
	}
	printf("IM RIGHT SIDE UP");
}