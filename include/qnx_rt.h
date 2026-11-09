/*
 * qnx_rt.h — QNX SDP 8.0 / Neutrino RTOS Deterministic Sensor Fusion & Watchdog Engine
 *
 * Dual-Target Real-Time Header:
 *   - Native QNX SDP 8.0 (`__QNXNTO__`): Uses native <sys/neutrino.h> synchronous IPC
 *     (`ChannelCreate`, `ConnectAttach`, `MsgSend`, `MsgReceive`, `MsgReply`, `MsgSendPulse`).
 *   - Portable POSIX Real-Time Shim (Linux/CI `gcc`): Emulates QNX synchronous rendezvous
 *     message-passing over lock-step priority-inheritance condition channels so developers
 *     and CI pipelines can build, benchmark, and verify deterministic real-time behavior anywhere.
 */

#ifndef QNX_RT_H
#define QNX_RT_H

#define _POSIX_C_SOURCE 200809L

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>

#ifdef __QNXNTO__
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#endif

/* QNX Pulse & Message Subtype Codes */
#define QNX_MSG_TYPE_SENSOR_FRAME     0x1001
#define QNX_MSG_TYPE_WATCHDOG_HB      0x1002
#define QNX_MSG_TYPE_FAULT_INJECT     0x1003
#define QNX_MSG_TYPE_SHUTDOWN         0x10FF

/* Deterministic Control Loop Constants */
#define RT_LOOP_PERIOD_NS             1000000ULL   /* 1.0 ms (1 kHz control loop) */
#define RT_DEADLINE_BUDGET_US         500.0        /* 500 microseconds max execution budget */
#define RT_MAX_MISSED_DEADLINES       0            /* Zero unrecovered deadline misses */

/* Incoming Multi-Modal Edge Sensor Frame (IMU + LiDAR/Odometry) */
typedef struct {
    uint16_t msg_type;
    uint32_t sequence_id;
    uint64_t timestamp_ns;
    double   accel_mps2;        /* Longitudinal acceleration (m/s^2) */
    double   gyro_rad_s;        /* Yaw rate (rad/s) */
    double   lidar_range_m;     /* Forward obstacle range (m) */
    double   wheel_vel_mps;     /* Wheel encoder velocity (m/s) */
    bool     injected_glitch;   /* True if fault injection spike is active */
} qnx_sensor_msg_t;

/* Synchronous Reply from Deterministic Fusion & Watchdog Server */
typedef struct {
    int32_t  status_code;       /* 0 = OK, 1 = GLITCH_REJECTED, 2 = SAFE_STATE_RECOVERED */
    uint32_t sequence_id;
    double   fused_position_m;
    double   fused_velocity_mps;
    double   kalman_gain_pos;
    double   kalman_gain_vel;
    double   innovation_residual;
    double   processing_latency_us;
    bool     watchdog_healthy;
} qnx_fusion_reply_t;

/* 2-State Deterministic Kalman Filter (Position + Velocity) */
typedef struct {
    double x_pos;       /* Estimated position (m) */
    double x_vel;       /* Estimated velocity (m/s) */
    double p00, p01;    /* Error covariance matrix P [2x2] */
    double p10, p11;
    double q_pos;       /* Process noise variance (position) */
    double q_vel;       /* Process noise variance (velocity) */
    double r_lidar;     /* Measurement noise variance (LiDAR range) */
    double r_wheel;     /* Measurement noise variance (Wheel velocity) */
    double gate_sigma;  /* Mahalanobis outlier rejection threshold */
    uint32_t glitches_rejected;
} kalman_fusion_state_t;

/* Deterministic Watchdog & Latency Telemetry Summary */
typedef struct {
    uint32_t total_cycles;
    uint32_t valid_fusions;
    uint32_t glitches_rejected;
    uint32_t watchdog_recoveries;
    uint32_t unrecovered_deadline_misses;
    double   min_latency_us;
    double   avg_latency_us;
    double   p99_latency_us;
    double   max_latency_us;
    double   deadline_budget_us;
    bool     prio_inherit_enabled;
    double   final_position_m;
    double   final_velocity_mps;
} qnx_rt_telemetry_t;

/* Sensor Fusion API (src/sensor_fusion.c) */
void kalman_fusion_init(kalman_fusion_state_t *kf);
int  kalman_fusion_step(
    kalman_fusion_state_t *kf,
    const qnx_sensor_msg_t *msg,
    double dt_sec,
    qnx_fusion_reply_t *out_reply
);

/* QNX IPC & Watchdog Runtime API (src/qnx_rt_watchdog.c) */
int qnx_rt_run_benchmark(
    uint32_t num_cycles,
    bool inject_faults,
    qnx_rt_telemetry_t *out_telemetry
);

#endif /* QNX_RT_H */
