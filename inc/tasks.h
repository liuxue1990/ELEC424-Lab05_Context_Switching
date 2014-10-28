#ifndef TASKS_H_
#define	TASKS_H_

/*
 * Alternates the spinning of each motor.
 *
 * NOTE: The task is not actually running the motor, but simply switching
 * which motor is running. As such, a motor is expected to be rotating
 * even when another task is running.
 */
void task_spin_motors(void);

/*
 * Blinks the onboard green LED.
 */
void task_blink_led(void);

#endif
