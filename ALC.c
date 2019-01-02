Starting reading...

Accelerometer readings:
    X     |     Y     |     Z    |
-4.831634 | 0.095740 | -9.066608 |

TURN ME CLOCKWISE
Turning On 6V Servo Power Rail
Moving direction -1.000000, 10.000000 pulsesTurn off/clean up servo
Finished servo move
Starting reading...

Accelerometer readings:
    X     |     Y     |     Z    |
-7.895150 | -0.665393 | 4.882386 |

TURN ME CLOCKWISE
Turning On 6V Servo Power Rail
Moving direction -1.000000, 10.000000 pulsesTurn off/clean up servo
Finished servo move
Starting reading...

Accelerometer readings:
    X     |     Y     |     Z    |
7.015874 | -1.836612 | 6.600027 |

TURN ME COUNTER CLOCKWISE/nTurning On 6V Servo Power Rail
Moving direction 1.000000, 10.000000 pulsesTurn off/clean up servo
Finished servo move
Starting reading...

Accelerometer readings:
    X     |     Y     |     Z    |
-6.402851 | 2.726993 | 7.946372 |

TURN ME CLOCKWISE
Turning On 6V Servo Power Rail
Moving direction -1.000000, 10.000000 pulsesTurn off/clean up servo
Finished servo move
Starting reading...

Accelerometer readings:
    X     |     Y     |     Z    |
8.049003 | -0.606353 | 4.338135 |

TURN ME COUNTER CLOCKWISE/nTurning On 6V Servo Power Rail
Moving direction 1.000000, 7.000000 pulsesTurn off/clean up servo
Finished servo move
Starting reading...

Accelerometer readings:
    X     |     Y     |     Z    |
-2.395489 | 0.345462 | 9.418070 |

TURN ME CLOCKWISE
Turning On 6V Servo Power Rail
Moving direction -1.000000, 4.000000 pulsesTurn off/clean up servo
Finished servo move
Starting reading...

Accelerometer readings:
    X     |     Y     |     Z    |
-0.416919 | -0.060635 | 9.743350 |

IM RIGHT SIDE UP
debian@beaglebone:~/Payload$ sudo ./alc
Starting reading...

Accelerometer readings:
    X     |     Y     |     Z    |
-9.525358 | 1.621197 | -0.301480 |

TURN ME CLOCKWISE
Turning On 6V Servo Power Rail
Moving direction -1.000000, 7.000000 pulsesTurn off/clean up servo
Finished servo move
Starting reading...

Accelerometer readings:
    X     |     Y     |     Z    |
-5.517995 | -0.305570 | 8.255786 |

TURN ME CLOCKWISE
Turning On 6V Servo Power Rail
Moving direction -1.000000, 10.000000 pulsesTurn off/clean up servo
Finished servo move
Starting reading...

Accelerometer readings:
    X     |     Y     |     Z    |
7.875220 | -1.391421 | 1.754930 |

TURN ME COUNTER CLOCKWISE/nTurning On 6V Servo Power Rail
Moving direction 1.000000, 7.000000 pulsesTurn off/clean up servo
Finished servo move
Starting reading...

Accelerometer readings:
    X     |     Y     |     Z    |
0.651286 | 0.864851 | 9.354600 |

TURN ME COUNTER CLOCKWISE/nTurning On 6V Servo Power Rail
Moving direction 1.000000, 4.000000 pulsesTurn off/clean up servo
Finished servo move
Starting reading...

Accelerometer readings:
    X     |     Y     |     Z    |
-2.497527 | 0.510613 | 9.590230 |

TURN ME CLOCKWISE
Turning On 6V Servo Power Rail
Moving direction -1.000000, 4.000000 pulsesTurn off/clean up servo
Finished servo move
Starting reading...

Accelerometer readings:
    X     |     Y     |     Z    |
0.330027 | -0.387747 | 9.779845 |

IM RIGHT SIDE UP
debian@beaglebone:~/Payload$ cat ALC.c
/* @title ALC.c
 *
 * @brief      Actuated landing control (ALC)--Use a servo to rotate the rover until it's upright.
 *
 * @author     Olivia Wong, parts taken from rc_test_mpu example
 *
 * @date       12/29/2018
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
#define NUM_PULSES 7
#define PULSE_POS 1.0 // See http://strawsondesign.com/docs/librobotcontrol/group___servo.html
#define PULSE_FREQ_HZ 50 // If this value isn't 50, and PULSE_POS isn't 1.0 it won't be able to move both ways so don't touch it
#define SERVO_CH 8


static int running = 0;
static rc_mpu_data_t mpu_data; // Struct to hold new MPU data

// Interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy) {
	running = 0;
	return;
}
double determine_num_pulses(double accel[3]) {
	/*
	The servo (SM-S4303R) is kind of finicky and doesn't move the exact distance each time
	Also the robotcontrollib servo functions are made for a normal servo and this is a continuous 360 degree servo so it's weird
	Frequency needs to be 50hz for it to be able to move both directions

	But this is what I've found from messing with it: (assuming 50hz and default pulse width)

	Anything less than 4 pulses results in no motion / motion one direction
	4 pulses moves a tiny bit (around 1/40 of a revolution / 10˚)
	5 pulses moves ~45˚
	7 pulses move ~85˚
	10 pulses moves ~120˚

	keep in mind these^ are VERY approximate
	*/
	double num_pulses = 4;

	// Determine # of pulses (how much to turn)
	double z = accel[Z];
	if (z < -4.5) {
		// Turn ~120˚
		num_pulses = 10;
	} else if (z >= -4.5 && z < 4.5) {
		// Turn ~85˚
		num_pulses = 7;
	} else if (z >= 4.5 && z < 8.5) {
		// Turn ~120˚
		num_pulses = 10;
	}

	return (num_pulses);

}
int run_motor(double direction, double num_pulses) {
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

	// Send pulses
	printf("Moving direction %f, %f pulses", direction, num_pulses);
	for (int i=0; i<num_pulses; i++) {
		if(rc_servo_send_pulse_normalized(SERVO_CH, direction * PULSE_POS) == -1) return -1;
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

	for (int i=0; i<ACCEL_READS; i++) {
		if(rc_mpu_read_accel(&mpu_data)<0) {
			printf("Read accel data failed\n");
		}
		for (int j=0; j<3; j++) {
			sums[j] += mpu_data.accel[j]; // Take sums to avg later
		}

		usleep(ACCEL_READ_INTERVAL); // Wait for new data to update
	}
	printf("\nAccelerometer readings: \n");
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

	double direction; // Counter clockwise

	while (running) {
		// Read accelerometer
		printf("Starting reading...\n");
		read_accel(accel_data);


		if (fabs(G - accel_data[Z]) < 0.16) {
			// When accelerometer data for Z ~ 9.8, the BB is upright
			// TODO: test 0.13 value, make sure it works for servo sides
			printf("IM RIGHT SIDE UP\n");
			break;
		} else {
			// Determine which direction to move based on accel reading
			if(accel_data[X] < 0) {
				printf("TURN ME CLOCKWISE\n");
				direction = -1.0;
			} else {
				printf("TURN ME COUNTER CLOCKWISE/n");
				direction = 1.0; // CCW
			}
			run_motor(direction, determine_num_pulses(accel_data));
		}

		rc_usleep(500000); // Sleep half a second to make sure motor is done moving
	}
}