//
// Created by swpin on 05/03/2023.
//
//all the IMU and GPS stuff has been removed for now easily added in, it was just done for ease of viewing

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <string.h>

#include <unistd.h>
#include <pthread.h>

#include <time.h>

#include "can.h"

#include "ugr-vcu_api.h"


// typedefs
typedef enum boolean_e {
    FALSE = 0,
    TRUE = 1,
} boolean_e;

typedef struct pack_16_t {
    union {
        volatile uint16_t uword;
        volatile int16_t sword;
        volatile uint8_t bytes[2];
    };
} pack_16_t;

typedef union can_data_t {
    volatile uint8_t ubytes[8];
    volatile int8_t sbytes[8];
    volatile uint16_t uwords[4];
    volatile int16_t swords[4];
    volatile uint32_t ulongs[2];
    volatile int32_t slongs[2];
    volatile float floats[2];
} can_data_t;


// statics
static boolean_e debug_mode = FALSE;
static boolean_e simulate_mode = FALSE;

static const float MOTOR_RATIO = 3.5f;
static const float MOTOR_MAX_RPM = 4000.0f;

static pthread_t can_read_tid;
static pthread_mutex_t can_read_mutex = PTHREAD_MUTEX_INITIALIZER;

static struct timespec last_set, this_set;


// tx frames
////may not be needed
//#define VCU2AI_STATUS_ID		0x520
//#define VCU2AI_DRIVE_F_ID		0x521
//#define VCU2AI_DRIVE_R_ID		0x522
//#define VCU2AI_STEER_ID			0x523
//#define VCU2AI_BRAKE_ID			0x524
//#define VCU2AI_WHEEL_SPEEDS_ID	0x525
//#define VCU2AI_WHEEL_COUNTS_ID	0x526

// i believe it is {canID,#ofBytesUsed}
static struct can_frame VCU2AI_Status		    = {0x520,3};
static struct can_frame VCU2AI_Drive_F;         = {0x521,2};
static struct can_frame VCU2AI_Drive_R;         = {0x522,2};
static struct can_frame VCU2AI_Steer;           = {0x523,4};
static struct can_frame VCU2AI_Brake;           = {0x524,2};
static struct can_frame VCU2AI_Wheel_speeds;    = {0x525,8};
static struct can_frame VCU2AI_Wheel_counts;    = {0x526,8};

// rx frames
#define AI2VCU_STATUS		                0x510
#define AI2VCU_DRIVE_F		                0x511
#define AI2VCU_DRIVE_R		                0x512
#define AI2VCU_STEER		                0x513
#define AI2VCU_BRAKE		                0x514

static struct can_frame AI2VCU_Status;
static struct can_frame AI2VCU_Drive_F;
static struct can_frame AI2VCU_Drive_R;
static struct can_frame AI2VCU_Steer;
static struct can_frame AI2VCU_Brake;

//Removed PCAN_GPS info for now until more is known

// static local data
static can_stats_t can_stats;

static volatile boolean_e AI2VCU_Statusfresh = FALSE;
static volatile boolean_e AI2VCU_Drive_F_fresh = FALSE;
static volatile boolean_e AI2VCU_Drive_R_fresh = FALSE;
static volatile boolean_e AI2VCU_Steer_fresh = FALSE;
static volatile boolean_e AI2VCU_Brake_fresh = FALSE;

// TX Initialization
// ---------------------------------------------------------
// VCU2AI_Status
static volatile ugr_vcu_api_handshake_receive_bit_e	VCU2AI_HANDSHAKE_RECEIVE_BIT = HANDSHAKE_RECEIVE_BIT_OFF;
static volatile ugr_vcu_api_res_go_signal_bit_e		VCU2AI_RES_GO_SIGNAL = RES_GO_SIGNAL_NO_GO;
static volatile ugr_vcu_api_as_state_e				VCU2AI_AS_STATE = AS_OFF;
static volatile ugr_vcu_api_ami_state_e				VCU2AI_AMI_STATE = AMI_NOT_SELECTED;
// remaining fields not relevant to API

// VCU2AI_Drive_F
static volatile uint16_t	VCU2AI_FRONT_AXLE_TORQUE_MAX_raw = 0;
// remaining fields not relevant to API

// VCU2AI_Drive_R
static volatile uint16_t	VCU2AI_REAR_AXLE_TORQUE_MAX_raw = 0;
// remaining fields not relevant to API

// VCU2AI_Steer
static volatile int16_t		VCU2AI_STEER_ANGLE_raw = 0;
static volatile uint16_t	VCU2AI_STEER_ANGLE_MAX_raw = 0;
// remaining fields not relevant to API

// VCU2AI_Brake
static volatile uint8_t	VCU2AI_BRAKE_PRESS_F_raw = 0;
static volatile uint8_t	VCU2AI_BRAKE_PRESS_R_raw = 0;
// remaining fields not relevant to API

// VCU2AI_Wheel_speeds
static volatile uint16_t	VCU2AI_FL_WHEEL_SPEED_rpm = 0;
static volatile uint16_t	VCU2AI_FR_WHEEL_SPEED_rpm = 0;
static volatile uint16_t	VCU2AI_RL_WHEEL_SPEED_rpm = 0;
static volatile uint16_t	VCU2AI_RR_WHEEL_SPEED_rpm = 0;

// VCU2AI_Wheel_counts
static volatile uint16_t	VCU2AI_FL_PULSE_COUNT = 0;
static volatile uint16_t	VCU2AI_FR_PULSE_COUNT = 0;
static volatile uint16_t	VCU2AI_RL_PULSE_COUNT = 0;
static volatile uint16_t	VCU2AI_RR_PULSE_COUNT = 0;
// ---------------------------------------------------------

// RX Initialization
// ---------------------------------------------------------
// ---------------------------------------------------------

// functions
//untouched but should work
int ugr_vcu_api_init(char* CAN_interface, int debug, int simulate) {
    static boolean_e initialised = FALSE;
    int err;

    ugr_vcu_api_clear_can_stats();

    if(initialised) {
        if(debug_mode) { printf("Already initialised...\r\n"); }
        return(EXIT_FAILURE);
    }

    initialised = TRUE;

    if(debug != 0) {
        debug_mode = TRUE;
        printf("Called ugr_vcu_api_init(%s, %d, %d)\r\n",CAN_interface,debug,simulate);
    }

    if(simulate != 0) {
        simulate_mode = TRUE;
        if(debug_mode) { printf("Simulate Mode enabled...\r\n"); }
    }

    if(can_init(CAN_interface) < 0) {
        if(debug_mode) { printf("Can't open [%s]", CAN_interface); }
        return(EXIT_FAILURE);
    }

    // spawn thread
    err = pthread_create(&can_read_tid, NULL, &can_read_thread, NULL);
    if(err != 0) {
        if(debug_mode) { printf("Can't create CAN read thread:[%s]", strerror(err)); }
        return(EXIT_FAILURE);
    }

    clock_gettime(CLOCK_REALTIME,&last_set);

    return(EXIT_SUCCESS);
}

//untouched and will not work
void ugr_vcu_api_vcu2ai_set_data(ugr_vcu_api_ai2vcu *data) {
    // local input data buffers
    ugr_vcu_api_mission_status_e		t_VCU2AI_MISSION_STATUS = 0;
    ugr_vcu_api_direction_request_e	    t_VCU2AI_DIRECTION_REQUEST = 0;
    ugr_vcu_api_estop_request_e		    t_VCU2AI_ESTOP_REQUEST = 0;
    ugr_vcu_api_handshake_send_bit_e	t_VCU2AI_HANDSHAKE_SEND_BIT = 0;
    float							    t_VCU2AI_STEER_ANGLE_REQUEST_deg = 0;
    float							    t_VCU2AI_FRONT_MOTOR_SPEED_REQUEST_rpm = 0;
    float							    t_VCU2AI_REAR_MOTOR_SPEED_REQUEST_rpm = 0;
    float							    t_VCU2AI_FRONT_AXLE_TORQUE_REQUEST_Nm = 0;
    float							    t_VCU2AI_REAR_AXLE_TORQUE_REQUEST_Nm = 0;
    float							    t_VCU2AI_FRONT_BRAKE_PRESS_REQUEST_pct = 0;
    float							    t_VCU2AI_REAR_BRAKE_PRESS_REQUEST_pct = 0;

    float t_FRONT_AXLE_TORQUE_MAX_Nm = (0.1f*VCU2AI_FRONT_AXLE_TORQUE_MAX_raw);
    float t_REAR_AXLE_TORQUE_MAX_Nm = (0.1f*VCU2AI_REAR_AXLE_TORQUE_MAX_raw);
    float t_STEER_ANGLE_MAX_deg = (0.1f*VCU2AI_STEER_ANGLE_MAX_raw);

    uint16_t t_fastest_wheel_rpm = 0;

    if(VCU2AI_FL_WHEEL_SPEED_rpm > t_fastest_wheel_rpm) { t_fastest_wheel_rpm = VCU2AI_FL_WHEEL_SPEED_rpm; }
    if(VCU2AI_FR_WHEEL_SPEED_rpm > t_fastest_wheel_rpm) { t_fastest_wheel_rpm = VCU2AI_FR_WHEEL_SPEED_rpm; }
    if(VCU2AI_RL_WHEEL_SPEED_rpm > t_fastest_wheel_rpm) { t_fastest_wheel_rpm = VCU2AI_RL_WHEEL_SPEED_rpm; }
    if(VCU2AI_RR_WHEEL_SPEED_rpm > t_fastest_wheel_rpm) { t_fastest_wheel_rpm = VCU2AI_RR_WHEEL_SPEED_rpm; }

    t_VCU2AI_MISSION_STATUS = data->VCU2AI_MISSION_STATUS;
    t_VCU2AI_DIRECTION_REQUEST = data->VCU2AI_DIRECTION_REQUEST;
    t_VCU2AI_ESTOP_REQUEST = data->VCU2AI_ESTOP_REQUEST;
    t_VCU2AI_HANDSHAKE_SEND_BIT = data->VCU2AI_HANDSHAKE_SEND_BIT;
    t_VCU2AI_STEER_ANGLE_REQUEST_deg = data->VCU2AI_STEER_ANGLE_REQUEST_deg;
    t_VCU2AI_FRONT_MOTOR_SPEED_REQUEST_rpm = data->VCU2AI_AXLE_SPEED_REQUEST_rpm * MOTOR_RATIO;
    t_VCU2AI_REAR_MOTOR_SPEED_REQUEST_rpm = data->VCU2AI_AXLE_SPEED_REQUEST_rpm * MOTOR_RATIO;
    t_VCU2AI_FRONT_AXLE_TORQUE_REQUEST_Nm = data->VCU2AI_AXLE_TORQUE_REQUEST_Nm;
    t_VCU2AI_REAR_AXLE_TORQUE_REQUEST_Nm = data->VCU2AI_AXLE_TORQUE_REQUEST_Nm;
    t_VCU2AI_FRONT_BRAKE_PRESS_REQUEST_pct = data->VCU2AI_BRAKE_PRESS_REQUEST_pct;
    t_VCU2AI_REAR_BRAKE_PRESS_REQUEST_pct = data->VCU2AI_BRAKE_PRESS_REQUEST_pct;

    // additional torque limit to maintain constant electrical power and minimise risk of over-current trip
    if(t_fastest_wheel_rpm > 700) {
        if(t_FRONT_AXLE_TORQUE_MAX_Nm > 50.0f) { t_FRONT_AXLE_TORQUE_MAX_Nm = 50.0f; }
        if(t_REAR_AXLE_TORQUE_MAX_Nm > 50.0f) { t_REAR_AXLE_TORQUE_MAX_Nm = 50.0f; }
    } else if(t_fastest_wheel_rpm > 600) {
        if(t_FRONT_AXLE_TORQUE_MAX_Nm > 85.0f) { t_FRONT_AXLE_TORQUE_MAX_Nm = 85.0f; }
        if(t_REAR_AXLE_TORQUE_MAX_Nm > 85.0f) { t_REAR_AXLE_TORQUE_MAX_Nm = 85.0f; }
    } else if(t_fastest_wheel_rpm > 500) {
        if(t_FRONT_AXLE_TORQUE_MAX_Nm > 100.0f) { t_FRONT_AXLE_TORQUE_MAX_Nm = 100.0f; }
        if(t_REAR_AXLE_TORQUE_MAX_Nm > 100.0f) { t_REAR_AXLE_TORQUE_MAX_Nm = 100.0f; }
    } else if(t_fastest_wheel_rpm > 400) {
        if(t_FRONT_AXLE_TORQUE_MAX_Nm > 120.0f) { t_FRONT_AXLE_TORQUE_MAX_Nm = 120.0f; }
        if(t_REAR_AXLE_TORQUE_MAX_Nm > 120.0f) { t_REAR_AXLE_TORQUE_MAX_Nm = 120.0f; }
    } else if(t_fastest_wheel_rpm > 300) {
        if(t_FRONT_AXLE_TORQUE_MAX_Nm > 150.0f) { t_FRONT_AXLE_TORQUE_MAX_Nm = 150.0f; }
        if(t_REAR_AXLE_TORQUE_MAX_Nm > 150.0f) { t_REAR_AXLE_TORQUE_MAX_Nm = 150.0f; }
    }

    // validate the 'float' requests
    if(t_VCU2AI_STEER_ANGLE_REQUEST_deg > t_STEER_ANGLE_MAX_deg) { t_VCU2AI_STEER_ANGLE_REQUEST_deg = t_STEER_ANGLE_MAX_deg; }
    if(t_VCU2AI_STEER_ANGLE_REQUEST_deg < (-1.0f*t_STEER_ANGLE_MAX_deg)) { t_VCU2AI_STEER_ANGLE_REQUEST_deg = (-1.0f*t_STEER_ANGLE_MAX_deg); }

    if(t_VCU2AI_FRONT_MOTOR_SPEED_REQUEST_rpm > MOTOR_MAX_RPM) { t_VCU2AI_FRONT_MOTOR_SPEED_REQUEST_rpm = MOTOR_MAX_RPM; }
    if(t_VCU2AI_FRONT_MOTOR_SPEED_REQUEST_rpm < 0.0f) { t_VCU2AI_FRONT_MOTOR_SPEED_REQUEST_rpm = 0.0f; }

    if(t_VCU2AI_REAR_MOTOR_SPEED_REQUEST_rpm > MOTOR_MAX_RPM) { t_VCU2AI_REAR_MOTOR_SPEED_REQUEST_rpm = MOTOR_MAX_RPM; }
    if(t_VCU2AI_REAR_MOTOR_SPEED_REQUEST_rpm < 0.0f) { t_VCU2AI_REAR_MOTOR_SPEED_REQUEST_rpm = 0.0f; }

    if(t_VCU2AI_FRONT_AXLE_TORQUE_REQUEST_Nm > t_FRONT_AXLE_TORQUE_MAX_Nm) { t_VCU2AI_FRONT_AXLE_TORQUE_REQUEST_Nm = t_FRONT_AXLE_TORQUE_MAX_Nm; }
    if(t_VCU2AI_FRONT_AXLE_TORQUE_REQUEST_Nm < 0.0f) { t_VCU2AI_FRONT_AXLE_TORQUE_REQUEST_Nm = 0.0f; }

    if(t_VCU2AI_REAR_AXLE_TORQUE_REQUEST_Nm > t_REAR_AXLE_TORQUE_MAX_Nm) { t_VCU2AI_REAR_AXLE_TORQUE_REQUEST_Nm = t_REAR_AXLE_TORQUE_MAX_Nm; }
    if(t_VCU2AI_REAR_AXLE_TORQUE_REQUEST_Nm < 0.0f) { t_VCU2AI_REAR_AXLE_TORQUE_REQUEST_Nm = 0.0f; }

    if(t_VCU2AI_FRONT_BRAKE_PRESS_REQUEST_pct > 100.0f) { t_VCU2AI_FRONT_BRAKE_PRESS_REQUEST_pct = 100.0f; }
    if(t_VCU2AI_FRONT_BRAKE_PRESS_REQUEST_pct < 0.0f) { t_VCU2AI_FRONT_BRAKE_PRESS_REQUEST_pct = 0.0f; }

    if(t_VCU2AI_REAR_BRAKE_PRESS_REQUEST_pct > 100.0f) { t_VCU2AI_REAR_BRAKE_PRESS_REQUEST_pct = 100.0f; }
    if(t_VCU2AI_REAR_BRAKE_PRESS_REQUEST_pct < 0.0f) { t_VCU2AI_REAR_BRAKE_PRESS_REQUEST_pct = 0.0f; }

    // validate the 'enum' requests
    if(t_VCU2AI_MISSION_STATUS < MISSION_NOT_SELECTED) { t_VCU2AI_MISSION_STATUS = MISSION_NOT_SELECTED; }
    if(t_VCU2AI_MISSION_STATUS > MISSION_FINISHED) { t_VCU2AI_MISSION_STATUS = MISSION_FINISHED; }

    if(t_VCU2AI_DIRECTION_REQUEST < DIRECTION_NEUTRAL) { t_VCU2AI_DIRECTION_REQUEST = DIRECTION_NEUTRAL; }
    if(t_VCU2AI_DIRECTION_REQUEST > DIRECTION_FORWARD) { t_VCU2AI_DIRECTION_REQUEST = DIRECTION_FORWARD; }

    if(t_VCU2AI_ESTOP_REQUEST < ESTOP_NO) { t_VCU2AI_ESTOP_REQUEST = ESTOP_NO; }
    if(t_VCU2AI_ESTOP_REQUEST > ESTOP_YES) { t_VCU2AI_ESTOP_REQUEST = ESTOP_YES; }

    if(t_VCU2AI_HANDSHAKE_SEND_BIT < HANDSHAKE_SEND_BIT_OFF) { t_VCU2AI_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_OFF; }
    if(t_VCU2AI_HANDSHAKE_SEND_BIT > HANDSHAKE_SEND_BIT_ON) { t_VCU2AI_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_ON; }

    // braking has priority over torque - reject implausible inputs
    if((t_VCU2AI_FRONT_BRAKE_PRESS_REQUEST_pct > 0.0f) || (t_VCU2AI_REAR_BRAKE_PRESS_REQUEST_pct > 0.0f)) {
        t_VCU2AI_FRONT_AXLE_TORQUE_REQUEST_Nm = 0.0f;
        t_VCU2AI_REAR_AXLE_TORQUE_REQUEST_Nm = 0.0f;
    }

    // set requests, converting where needed
    VCU2AI_HANDSHAKE_BIT = t_VCU2AI_HANDSHAKE_SEND_BIT;
    VCU2AI_ESTOP_REQUEST = t_VCU2AI_ESTOP_REQUEST;
    VCU2AI_MISSION_STATUS = t_VCU2AI_MISSION_STATUS;
    VCU2AI_DIRECTION_REQUEST = t_VCU2AI_DIRECTION_REQUEST;
    VCU2AI_LAP_COUNTER = 0;
    VCU2AI_CONES_COUNT_ACTUAL = 0;
    VCU2AI_CONES_COUNT_ALL = 0;

    VCU2AI_STEER_REQUEST_raw = (int16_t)(10.0f*t_VCU2AI_STEER_ANGLE_REQUEST_deg);
    VCU2AI_FRONT_MOTOR_SPEED_MAX_rpm = (uint16_t)t_VCU2AI_FRONT_MOTOR_SPEED_REQUEST_rpm;
    VCU2AI_REAR_MOTOR_SPEED_MAX_rpm = (uint16_t)t_VCU2AI_REAR_MOTOR_SPEED_REQUEST_rpm;
    VCU2AI_FRONT_AXLE_TRQ_REQUEST_raw = (uint16_t)(10.0f*t_VCU2AI_FRONT_AXLE_TORQUE_REQUEST_Nm);
    VCU2AI_REAR_AXLE_TRQ_REQUEST_raw = (uint16_t)(10.0f*t_VCU2AI_REAR_AXLE_TORQUE_REQUEST_Nm);
    VCU2AI_HYD_PRESS_F_REQ_raw = (uint8_t)(2.0f*t_VCU2AI_FRONT_BRAKE_PRESS_REQUEST_pct);
    VCU2AI_HYD_PRESS_R_REQ_raw = (uint8_t)(2.0f*t_VCU2AI_REAR_BRAKE_PRESS_REQUEST_pct);

    // load the CAN frames with the validated data
    volatile pack_16_t temp;

    VCU2AI_Status.data[0] = (uint8_t)(VCU2AI_HANDSHAKE_BIT & 0x01);
    VCU2AI_Status.data[1] = (uint8_t)(((VCU2AI_DIRECTION_REQUEST & 0x03) << 6) + ((VCU2AI_MISSION_STATUS & 0x03) << 4) + (VCU2AI_ESTOP_REQUEST & 0x01));
    VCU2AI_Status.data[2] = (VCU2AI_LAP_COUNTER & 0x0F);
    VCU2AI_Status.data[3] = VCU2AI_CONES_COUNT_ACTUAL;
    VCU2AI_Status.data[4] = (uint8_t)(VCU2AI_CONES_COUNT_ALL & 0x00FF);
    VCU2AI_Status.data[5] = (uint8_t)((VCU2AI_CONES_COUNT_ALL & 0xFF00) >> 8);
    VCU2AI_Status.data[6] = 0;
    VCU2AI_Status.data[7] = 0;

    temp.uword = VCU2AI_FRONT_AXLE_TRQ_REQUEST_raw;
    VCU2AI_Drive_F.data[0] = temp.bytes[0];
    VCU2AI_Drive_F.data[1] = temp.bytes[1];
    temp.uword = VCU2AI_FRONT_MOTOR_SPEED_MAX_rpm;
    VCU2AI_Drive_F.data[2] = temp.bytes[0];
    VCU2AI_Drive_F.data[3] = temp.bytes[1];
    VCU2AI_Drive_F.data[4] = 0;
    VCU2AI_Drive_F.data[5] = 0;
    VCU2AI_Drive_F.data[6] = 0;
    VCU2AI_Drive_F.data[7] = 0;

    temp.uword = VCU2AI_REAR_AXLE_TRQ_REQUEST_raw;
    VCU2AI_Drive_R.data[0] = temp.bytes[0];
    VCU2AI_Drive_R.data[1] = temp.bytes[1];
    temp.uword = VCU2AI_REAR_MOTOR_SPEED_MAX_rpm;
    VCU2AI_Drive_R.data[2] = temp.bytes[0];
    VCU2AI_Drive_R.data[3] = temp.bytes[1];
    VCU2AI_Drive_R.data[4] = 0;
    VCU2AI_Drive_R.data[5] = 0;
    VCU2AI_Drive_R.data[6] = 0;
    VCU2AI_Drive_R.data[7] = 0;

    temp.sword = VCU2AI_STEER_REQUEST_raw;
    VCU2AI_Steer.data[0] = temp.bytes[0];
    VCU2AI_Steer.data[1] = temp.bytes[1];
    VCU2AI_Steer.data[2] = 0;
    VCU2AI_Steer.data[3] = 0;
    VCU2AI_Steer.data[4] = 0;
    VCU2AI_Steer.data[5] = 0;
    VCU2AI_Steer.data[6] = 0;
    VCU2AI_Steer.data[7] = 0;

    VCU2AI_Brake.data[0] = VCU2AI_HYD_PRESS_F_REQ_raw;
    VCU2AI_Brake.data[1] = VCU2AI_HYD_PRESS_R_REQ_raw;
    VCU2AI_Brake.data[2] = 0;
    VCU2AI_Brake.data[3] = 0;
    VCU2AI_Brake.data[4] = 0;
    VCU2AI_Brake.data[5] = 0;
    VCU2AI_Brake.data[6] = 0;
    VCU2AI_Brake.data[7] = 0;

    clock_gettime(CLOCK_REALTIME,&this_set);

    long int interval_ns = ((this_set.tv_sec-last_set.tv_sec)* 1000000000) + (this_set.tv_nsec-last_set.tv_nsec);

    if(interval_ns > 8000000) // enforce maximum call rate of approx. 8ms
    {
        // send the CAN frames
        can_send(&VCU2AI_Status);
        can_send(&VCU2AI_Drive_F);
        can_send(&VCU2AI_Drive_R);
        can_send(&VCU2AI_Steer);
        can_send(&VCU2AI_Brake);
        clock_gettime(CLOCK_REALTIME,&last_set);
    }
}


void fs_ai_api_get_can_stats(can_stats_t *data) {
    memcpy(data,&can_stats,sizeof(can_stats_t));
}


//untouched but should work
void ugr_vcu_api_clear_can_stats() {
    memset(&can_stats, 0, sizeof(can_stats_t));
}
