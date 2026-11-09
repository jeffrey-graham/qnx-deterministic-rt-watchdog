# Devpost Submission Guide — QNX Global Hackathon (`https://qnx.devpost.com/`)

## Project Title
**QNX Deterministic Edge Watchdog & 1kHz Sensor Fusion Engine + Interactive Developer Codelab**

## Tagline (Elevator Pitch)
Sub-millisecond (14.1 us p99) QNX SDP 8.0 synchronous IPC sensor fusion engine, Mahalanobis glitch-rejection watchdog, and 5-module interactive developer codelab for safety-critical robotics.

## Target Tracks & Prizes
- **Main Track:** Real-Time Edge, Robotics & Mission-Critical Systems on QNX SDP 8.0 ($50,000 Prize Pool across 15 Winners)
- **Special Category Prize ($3,500):** **Best Codelab / Tutorial** (`CODELAB_TUTORIAL.md` + `codelab/index.html`)

---

## Full Project Description (Copy-Paste into Devpost)

### Inspiration
In autonomous robotics, surgical systems, and automotive ADAS controllers, **average speed is meaningless—only deterministic Worst-Case Execution Time (WCET) keeps people safe**. On monolithic operating systems, developers routinely wire multi-rate IMU, LiDAR, and wheel-odometry threads together using shared-memory ring buffers protected by standard mutexes. Under heavy load, three failure modes inevitably emerge:
1. **Priority Inversion**: Low-priority logging threads hold shared locks and starve the 1kHz safety control loop.
2. **Dynamic Allocation Jitter**: Heap allocations (`malloc`/`free`) inside the sensor loop trigger page faults and unpredictable millisecond stalls.
3. **Sensor Multipath Glitches**: A single `+85m` LiDAR multipath reflection off wet pavement corrupts the Kalman state covariance for hundreds of milliseconds.

Moreover, embedded developers migrating from standard Linux to **QNX Software Development Platform (SDP) 8.0** often lack a clean, hands-on codelab showing how to architect **QNX Neutrino synchronous message passing (`MsgSend` / `MsgReceive` / `MsgReply`)** alongside a hard real-time watchdog. We built both the **Deterministic Edge Watchdog & 1kHz Sensor Fusion Engine** and a **5-Module Interactive QNX SDP 8.0 Developer Codelab** to solve these challenges end-to-end.

### What it does
1. **QNX Neutrino Synchronous IPC Server (`src/qnx_rt_watchdog.c`, `include/qnx_rt.h`)**:
   - Uses native QNX SDP 8.0 synchronous rendezvous message passing (`ChannelCreate`, `ConnectAttach`, `MsgSend`, `MsgReceive`, `MsgReply`) when compiled under `__QNXNTO__`.
   - Includes a portable POSIX synchronous rendezvous shim with priority-inheriting mutexes (`PTHREAD_PRIO_INHERIT`) so developers and CI pipelines can compile, benchmark, and unit-test the exact `SEND-blocked -> RECEIVE-blocked -> REPLY-blocked` state machine on any Linux machine (`gcc -std=c11 -O2 -Wall -Wextra -Werror`) as well as QNX SDP 8.0 targets (`make qnx` with `qcc -Vgcc_ntoaarch64le`).
2. **Zero-Heap $O(1)$ Kalman Filter with 4.5-Sigma Innovation Gate (`src/sensor_fusion.c`)**:
   - Fuses 1kHz longitudinal IMU acceleration, LiDAR obstacle range, and wheel odometry velocity in constant time with **zero dynamic memory allocation**.
   - Computes the normalized Mahalanobis innovation residual before every measurement update, rejecting `+85.0 m` LiDAR multipath spikes in `< 10 microseconds` while safely coasting on inertial IMU predictions.
3. **Hardware/Software Deadline Watchdog Supervisor**:
   - Measures cycle-by-cycle nanosecond execution latency (`CLOCK_MONOTONIC`) against a strict `500.0 us` real-time budget and executes deterministic safe-state recovery when faults are injected.
4. **Flagship 5-Module QNX SDP 8.0 Developer Codelab (`CODELAB_TUTORIAL.md` + `codelab/index.html`)**:
   - Engineered specifically for the **$3,500 Best Codelab / Tutorial** prize, guiding developers step-by-step from QNX IPC fundamentals to priority inheritance, zero-heap Kalman filtering, and interactive 1kHz fault injection in the browser.

### How we built it
- **Standards-Compliant C11 & QNX Neutrino RTOS Primitives**: Implemented in modular C11 (`include/qnx_rt.h`, `src/sensor_fusion.c`, `src/qnx_rt_watchdog.c`) with zero compiler warnings under `-Wall -Wextra -Werror`.
- **Automated Verification Suite (`tests/test_qnx_rt.py` & `tests/run_tests.sh`)**: Compiles the binary, executes 1,000-cycle fault-injected and nominal benchmarks, parses the `--json` telemetry output, and asserts `prio_inherit_enabled == true`, `glitches_rejected == 2`, `unrecovered_deadline_misses == 0`, and `p99_latency_us < 500.0`.
- **Interactive Codelab (`codelab/index.html`)**: Self-contained 5-step interactive tutorial and browser-based 1kHz QNX IPC telemetry simulator.

### Challenges we ran into
- **Dual-Target Determinism (QNX SDP 8.0 `qcc` + Linux CI `gcc`)**: We wanted every developer and hackathon judge to be able to verify sub-millisecond behavior in 2 seconds whether they are running on a QNX SDP 8.0 AArch64 target or a Linux workstation. By building a lock-step `PTHREAD_PRIO_INHERIT` rendezvous shim that mirrors `MsgSend`/`MsgReceive`/`MsgReply`, the exact same application code runs on both targets without changing a single line of `src/sensor_fusion.c`.
- **Coasting Cleanly During Multipath Sensor Spikes**: Simply dropping a bad LiDAR reading without propagating state covariance causes filter divergence. Our `kalman_fusion_step` propagates the prior state and covariance matrix $P' = F P F^T + Q$ using the IMU acceleration input whenever the 4.5-sigma innovation gate trips.

### Accomplishments that we're proud of
- **Sub-15 Microsecond p99 Round-Trip Latency**: Across 1,000 synchronous 1kHz IPC + Kalman fusion cycles, achieved **8.95 us average latency** and **14.08 us p99 latency**—over **35x faster** than our 500.0 us real-time deadline budget.
- **100% Glitch Rejection & Zero Unrecovered Deadline Misses**: Deterministically rejected all injected `+85m` LiDAR multipath glitches while maintaining trajectory accuracy within `0.04 m`.
- **Turnkey Developer Codelab**: Created both a comprehensive Markdown curriculum (`CODELAB_TUTORIAL.md`) and an interactive web codelab (`codelab/index.html`).

### What's next for QNX Deterministic Edge Watchdog
- Extending the 2-state longitudinal filter to a 6-DoF Extended Kalman Filter (EKF) over QNX shared-memory zero-copy buffers (`shm_open` + `MsgSendPulse` notification) for high-density 3D point clouds.
- Publishing the codelab as an open-source QNX SDP 8.0 starter template for university robotics teams.

### Built With
`qnx`, `qnx-sdp-8`, `c11`, `posix-realtime`, `kalman-filter`, `sensor-fusion`, `embedded-systems`, `robotics`, `ipc`, `python`
