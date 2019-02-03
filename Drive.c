/**
 * @file Drive.c
 * 
 *
 * Demonstrates use of H-bridges to drive motors with the Robotics Cape and
 * BeagleBone Blue. 
 */
#include <stdio.h>
#include <signal.h>
#include <stdlib.h> // for atoi
#include <getopt.h>
#include <rc/motor.h>
#include <rc/time.h>
static int running = 0;

// Ports on Beaglebone for left/right motors
#define L_MOTOR 1 
#define R_MOTOR 2
#define DEFAULT_DUTY 0.8 // Default duty cycle (between -1.0-1.0)


// possible modes, user selected with command line arguments
typedef enum m_mode_t {
        DISABLED,
        NORMAL,
        TURN_LEFT,
        TURN_RIGHT,
        BRAKE,
        EXIT_ROCKET,
        SWEEP
} m_mode_t;
// printed if some invalid argument was given
static void __print_usage(void)
{
        printf("\n");
        printf("-d {duty}   define a duty cycle from -1.0 to 1.0\n");
        printf("-b          enable motor brake function\n");
        printf("-F {freq}   set a custom pwm frequency in HZ, otherwise default 25000 is used\n");
        printf("-f          enable free spin function\n");
        printf("-s {duty}   sweep motors back and forward at duty cycle\n");
        printf("-m {motor}  specify a single motor from 1-4, otherwise all will be driven\n");
        printf("            motors will be driven equally.\n");
        printf("-h          print this help message\n");
        printf("\n");
}
// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        running = 0;
        return;
}
int main(int argc, char *argv[])
{
        double duty = 0.0;
        int freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ;
        m_mode_t m_mode = DISABLED;
        
        // set signal handler so the loop can exit cleanly
        signal(SIGINT, __signal_handler);
        running = 1;
        // initialize hardware first
        if(rc_motor_init_freq(freq_hz)) return -1;

        m_mode = TURN_LEFT;


        // decide what to do
        switch(m_mode){
        case NORMAL:
                printf("Sending duty cycle %0.4f\n", duty);
                rc_motor_set(L_MOTOR, DEFAULT_DUTY);
                rc_motor_set(R_MOTOR, DEFAULT_DUTY);
                break;
        case TURN_LEFT:
                printf("Turning left \n");
                rc_motor_set(L_MOTOR, DEFAULT_DUTY / 2);
                rc_motor_set(R_MOTOR, DEFAULT_DUTY);
                break;
        case TURN_RIGHT:
                printf("Turning right\n");
                rc_motor_set(L_MOTOR, DEFAULT_DUTY);
                rc_motor_set(R_MOTOR, DEFAULT_DUTY / 2);
                break;
        case BRAKE:
                rc_motor_brake(L_MOTOR);
                rc_motor_brake(R_MOTOR);
                break;
        case EXIT_ROCKET:
                printf("Turning right\n");
                rc_motor_set(L_MOTOR, DEFAULT_DUTY);
                rc_motor_set(R_MOTOR, DEFAULT_DUTY / 2);
                break;
        default:
                break;
        }
        // wait untill the user exits
        while(running){
                if(m_mode==SWEEP){
                        duty = -duty; // toggle back and forth to sweep motors side to side
                        printf("sending duty cycle %0.4f\n", duty);
                        fflush(stdout);
                }
                // if not in SWEEP mode, the motors have already been set so do nothing
                rc_usleep(500000);
        }
        // final cleanup
        printf("\ncalling rc_motor_cleanup()\n");
        rc_motor_cleanup();
        return 0;
}