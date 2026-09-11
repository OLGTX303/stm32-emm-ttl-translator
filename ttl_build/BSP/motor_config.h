#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H
#define MOTOR_COUNT 4U
#define MOTOR_DEFAULT_BAUD 115200U
#define HOST_BAUD 115200U
#define TR_QUEUE_SIZE 16U
#define TR_TICK_US 10000UL
#define TR_LATE_US 1500UL
#define TR_BUS_GAP_US 2000UL
#define TR_REPLY_US 8000UL
#define TR_FEEDBACK_US 500000UL
#define TR_ENVIRONMENT_US 2000000UL
#define TR_SETTLE_US 500000UL
#define TR_STARTUP_ORIGIN_MODE 3U
#define TR_HOME_MIN_US 80000UL
#define TR_STATIONARY_US 100000UL
#define TR_STATIONARY_COUNTS 8L
/* 40 legacy counts = 0.879 deg at 16384 counts/rev, closely matching the
 * Emm Rev1.3 default 0.8 deg position-reached window. */
#define TR_REACH_COUNTS 40L
#define TR_FULL_CURRENT_MA 2500U
#define TR_MAX_RPM 3000U
#define TR_CLAMP_MAX_PERCENT 40U
#define MOTOR_STEPS_PER_REV 200U
#define MOTOR_MICROSTEP 16U
#define MOTOR_PULSES_PER_REV (MOTOR_STEPS_PER_REV * MOTOR_MICROSTEP)

/* The PC/cube_motion.py logical IDs are intentionally unchanged:
 *   logical 1 = right finger, logical 2 = right arm,
 *   logical 3 = left finger,  logical 4 = left arm.
 * The installed ZDT addresses are:
 *   physical 1 = right arm,   physical 2 = right finger,
 *   physical 3 = left arm,    physical 4 = left finger.
 * Translation must happen only on the motor-side UART. */
#define TR_PHYSICAL_ID_INITIALIZER {2U, 1U, 4U, 3U}
#define TR_DIRECTION_INITIALIZER {1, 1, 1, 1}
#define TR_OFFSET_INITIALIZER {0, 0, 0, 0}
#define MOTOR_ID_RIGHT_FINGER 1U
#define MOTOR_ID_RIGHT_ARM 2U
#define MOTOR_ID_LEFT_FINGER 3U
#define MOTOR_ID_LEFT_ARM 4U
#define MOTOR_IS_FINGER(id) ((id) == 1U || (id) == 3U)
#endif
