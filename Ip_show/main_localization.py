import math
import time
import os
from PID import MinibotDriveOdo, ODO_WHEEL_BASE_M, DRIVE_WHEEL_BASE_M
from Ip_show.ekf_localizer import EKFLocalizer, LidarLocalizationThread, wrap_angle


class LocalizedRobotController:
    def __init__(self):
        self.drive = MinibotDriveOdo()
        self.ekf = EKFLocalizer(
            wheel_base_m=ODO_WHEEL_BASE_M,
            init_pose=(0.0, 0.0, 0.0),  # Yellow start center facing +X
        )

        # Initialize baseline odometry counters without resetting mid-match
        _, _, self.prev_l_m, self.prev_r_m = self.drive.get_telemetry()

        # Start asynchronous LiDAR EKF measurement updates
        self.lidar_thread = LidarLocalizationThread(self.ekf)
        self.lidar_thread.start()

    def step_odometry(self):
        """Reads SPI encoders, computes incremental wheel travel, and runs EKF prediction."""
        l_mps, r_mps, l_m, r_m = self.drive.get_telemetry()
        ds_l = l_m - self.prev_l_m
        ds_r = r_m - self.prev_r_m
        self.prev_l_m = l_m
        self.prev_r_m = r_m

        self.ekf.predict_odometry(ds_l, ds_r)
        return l_mps, r_mps

    def go_to_waypoint(
        self,
        target_x: float,
        target_y: float,
        target_theta_deg: float = None,
        pos_tol_m: float = 0.015,
        ang_tol_deg: float = 2.0,
        max_v_mps: float = 0.28,
        loop_hz: float = 50.0,
    ):
        """
        Drives to a global (target_x, target_y) coordinate [m] using fused EKF pose,
        then optionally aligns to target_theta_deg.
        """
        dt_nom = 1.0 / loop_hz
        last_time = time.time()
        self.drive.reset_pids()

        # Phase 1: Rotate toward waypoint bearing
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            self.step_odometry()
            x, y, theta, _ = self.ekf.get_pose()

            dx = target_x - x
            dy = target_y - y
            if math.hypot(dx, dy) <= pos_tol_m:
                break

            desired_heading = math.atan2(dy, dx)
            heading_err = wrap_angle(desired_heading - theta)
            if abs(heading_err) <= math.radians(5.0):
                break

            omega = max(min(2.5 * heading_err, 2.0), -2.0)
            v_wheel = omega * (0.5 * DRIVE_WHEEL_BASE_M)
            if 0 < v_wheel < 0.07:
                v_wheel = 0.07
            elif -0.07 < v_wheel < 0:
                v_wheel = -0.07

            self.drive.update_speed_control(-v_wheel, +v_wheel, dt)
            time.sleep(dt_nom)

        # Phase 2: Drive to (target_x, target_y) with continuous EKF heading correction
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            self.step_odometry()
            x, y, theta, _ = self.ekf.get_pose()

            dx = target_x - x
            dy = target_y - y
            dist_err = math.hypot(dx, dy)

            if dist_err <= pos_tol_m:
                break

            desired_heading = math.atan2(dy, dx)
            heading_err = wrap_angle(desired_heading - theta)

            # Slow down smoothly if heading error grows
            v_lin = min(1.8 * dist_err, max_v_mps) * max(0.0, math.cos(heading_err))
            if v_lin < 0.08:
                v_lin = 0.08

            omega = 2.8 * heading_err
            v_left = v_lin - omega * (0.5 * DRIVE_WHEEL_BASE_M)
            v_right = v_lin + omega * (0.5 * DRIVE_WHEEL_BASE_M)

            self.drive.update_speed_control(v_left, v_right, dt)
            time.sleep(dt_nom)

        # Phase 3: Final orientation alignment (if requested)
        if target_theta_deg is not None:
            target_rad = math.radians(target_theta_deg)
            while True:
                now = time.time()
                dt = now - last_time
                last_time = now

                self.step_odometry()
                _, _, theta, _ = self.ekf.get_pose()
                ang_err = wrap_angle(target_rad - theta)

                if abs(ang_err) <= math.radians(ang_tol_deg):
                    break

                omega = max(min(2.5 * ang_err, 2.0), -2.0)
                v_wheel = omega * (0.5 * DRIVE_WHEEL_BASE_M)
                if 0 < v_wheel < 0.07:
                    v_wheel = 0.07
                elif -0.07 < v_wheel < 0:
                    v_wheel = -0.07

                self.drive.update_speed_control(-v_wheel, +v_wheel, dt)
                time.sleep(dt_nom)

        self.drive.stop()

    def shutdown(self):
        self.lidar_thread.stop()
        self.drive.emergency_stop()


if __name__ == "__main__":
    robot = LocalizedRobotController()
    try:
        # Allow LiDAR motor to spin up and acquire initial fix
        time.sleep(2.5)

        # Example Eurobot table mission (coordinates in meters relative to Yellow Start)
        waypoints = [
            (0.60, 0.00, 0.0),     # Drive 60 cm forward
            (0.60, 0.50, 90.0),    # Move 50 cm left
            (0.00, 0.50, 180.0),   # Return along X
            (0.00, 0.00, 0.0),     # Return to Yellow Start Center
        ]

        for idx, (wx, wy, wdeg) in enumerate(waypoints, start=1):
            print(f"\n[WAYPOINT {idx}] Target: X={wx:+.2f}m, Y={wy:+.2f}m, Cap={wdeg:+.0f}°")
            robot.go_to_waypoint(wx, wy, wdeg)

            x, y, theta, P = robot.ekf.get_pose()
            std_x, std_y, std_th = math.sqrt(P[0, 0]), math.sqrt(P[1, 1]), math.degrees(math.sqrt(P[2, 2]))
            print(
                f"[REACHED] Fused Pose: X={x:+.3f}m (±{std_x*1000:.1f}mm), "
                f"Y={y:+.3f}m (±{std_y*1000:.1f}mm), "
                f"Cap={math.degrees(theta):+.1f}° (±{std_th:.1f}°) | "
                f"Beacons: {list(robot.ekf.last_matched_beacons.keys())}"
            )
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[STOP] Interrupted by user.")
    finally:
        robot.shutdown()
        os._exit(0)