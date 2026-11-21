/* Obstacle-stop hook. Replace obstacle.c with a real sensor
 * (HC-SR04 / VL53L0X directly, or distances reported by the Pico). */
#ifndef TARS_OBSTACLE_H
#define TARS_OBSTACLE_H

int   obstacle_init(void);
/* Distance ahead in metres, or < 0 if unknown / no sensor. Called every
 * control tick from the RT thread: must be fast and non-blocking. */
float obstacle_distance_m(void);

#endif
