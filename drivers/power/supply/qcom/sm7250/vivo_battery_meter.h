#ifndef __VIVO_BATTERY_METER_H
#define __VIVO_BATTERY_METER_H

#define PROFILE_SIZE                    5
#define CURVE_STANDARD_SIZE             101

#define TEMPERATURE_T0                  -10
#define TEMPERATURE_T1                  0
#define TEMPERATURE_T2                  10
#define TEMPERATURE_T3                  25
#define TEMPERATURE_T4                  50

typedef enum {
        BATTERY_ID_NOT_LOADED = -1,
        BATTERY_UNKNOWN_SUPPLIER = 0,
        BATTERY_FIRST_SUPPLIER,
        BATTERY_SECOND_SUPPLIER,
        BATTERY_THIRD_SUPPLIER,
        BATTERY_FOURTH_SUPPLIER,
        BATTERY_FIFTH_SUPPLIER,
        BATTERY_DEFAULT_SUPPLIER = 100,
} BATTERY_VENDOR;

struct curve_profile {
        int dod;        /* depth of discharge */
	 int voltage;    /* open circuit voltage */
        int resistance; /* resistance of cell */
};

struct meter_data {
        struct curve_profile *profile[PROFILE_SIZE];
        struct curve_profile suited_profile[CURVE_STANDARD_SIZE];
        int curve_size;
};

struct battery_curve {
        const char *name;
        struct curve_profile battery_profile_t0[101];
        struct curve_profile battery_profile_t1[101];
        struct curve_profile battery_profile_t2[101];
        struct curve_profile battery_profile_t3[101];
        struct curve_profile battery_profile_t4[101];
        int profile_size_t0;
        int profile_size_t1;
        int profile_size_t2;
        int profile_size_t3;
        int profile_size_t4;
};

#endif