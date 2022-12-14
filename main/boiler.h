#ifndef BOILER_HXX_
#define BOILER_HXX_

#include <inttypes.h>

#include "driver/pulse_cnt.h"
#include "hw_ow.h"

typedef struct sensor {
    rom_t rom; /* sensor rom */
    float temp;
} temp_sensor_t;

typedef struct flow {
    uint32_t ppl;           /**< pulse per liter */
    float flow;          /**< calculeted flow, liters/sec*/
    pcnt_unit_handle_t pcn; /**< handler */
} flow_t;

enum sensors_dir_e {
    DIR_IN = 0,
    DIR_OUT = 1,
    DIR_MAX = 2,
    POINT3 = DIR_MAX,
    POINT4 = 3,
    POINT_MAX = 4
};

typedef struct termal {
    temp_sensor_t temp[DIR_MAX];
    flow_t flow;
} termal_t;

typedef struct power {
    float volts, amps;
} power_t;

typedef struct worker {
    bool enabled;
    uint64_t last_updated;
    termal_t termal;
    power_t power;
} worker_t;

/**
 * @brief common context
 */
typedef struct boiler {
    /* components */
    void* server;
    bdc_motor_handle_t cooler_motor;
    float cooler_motor_pwr;

    /* generic counters */
    uint64_t cycles;
    uint64_t boot_ts;
    uint64_t loop_err;

    /* workers */
    worker_t workers[3]; /**< 3 by desing */

    /* flows info */
    termal_t colant;
    termal_t boiler;

    /* aux endpoint */
    temp_sensor_t cooler_temps[POINT_MAX];
    temp_sensor_t aux_temps[POINT_MAX];

    /* connection info*/
    /// @todo

    /* locks */

} boiler_t;

#endif