#include "obstacle.h"

int obstacle_init(void)
{
    /* TODO: open the sensor (I2C for VL53L0X, GPIO for HC-SR04), or have the
     * Pico measure and send "R <cm>" lines back that motor_pico_uart.c parses.
     * HC-SR04 timing is easier on the Pico than on the Pi. */
    return 0;
}

float obstacle_distance_m(void)
{
    return -1.0f;   /* no sensor yet */
}
