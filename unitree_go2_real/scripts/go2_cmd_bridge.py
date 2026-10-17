#!/usr/bin/env python3
"""Nav2 velocities -> the Go2's sport API. THIS NODE MOVES THE ROBOT when enabled:=true. Start it on purpose only,
with the remote in hand. It is never started by a launch file.

  ros2 run unitree_go2_real go2_cmd_bridge.py                                   # dry run: logs, sends nothing
  ros2 run unitree_go2_real go2_cmd_bridge.py --ros-args -p enabled:=true -p obstacle_avoidance:=off
  ... -p request_classic:=on      # wait until the operator selects Classic on the remote

In:  /go2/nav2_cmd_vel (Twist, from Nav2's collision_monitor), /sportmodestate, /wirelesscontroller,
     /api/obstacles_avoid/response
Out: /api/sport/request (unitree_api/Request): Move 1008 {"x": vx, "y": vy, "z": wz}, StopMove 1003;
     /api/obstacles_avoid/request: SwitchSet 1001 {"enable": bool}
     (1002 = SwitchGet, seen polled at ~13 Hz in our recordings, answer {"enable": ...}).

Safety:
- enabled false (default): nothing is sent, the commands that would be sent are logged once per second.
- Gait: never changed by this node. request_classic off (default): the robot drives in whatever gait it is in.
  request_classic on: while /sportmodestate error_code is not 2010 (Classic) the operator is asked (every 3 s) to select
  Classic on the remote, and nothing moves until it is. (The ClassicWalk API request, 2049, was tried live: status -1
  when already Classic, and it did not switch the robot out of gait 100.)
- Caps: |vx| <= max_vx, |vy| <= max_vy, |wz| <= max_wz (0.55 m/s, 0.20 m/s, 1.1 rad/s), whatever Nav2 asks.
- Watchdog: no command for cmd_timeout_s (0.5 s), or a zero command -> one StopMove; Move is re-sent at 10 Hz otherwise.
- Remote wins: any stick beyond 0.15 or any button pauses the bridge (StopMove). /wirelesscontroller only publishes
  when something changes (a stick held still sends nothing), so the pause lasts until the remote reports sticks centred
  and no button pressed, plus remote_pause_s (1.5 s).
- Robot must be standing (body_height > 0.2 m).
- obstacle_avoidance: 'leave' (default, untouched) | 'off' | 'on': set via SwitchSet on enable; the state found at start
  is restored on exit. route 'avoid' (Move 1003 through the obstacle-avoidance service, parameters {"x","y","yaw",
  "mode": 0} plus UseRemoteCommandFromApi 1004) follows Unitree's SDK but is NOT yet verified on this robot.
- Exit (Ctrl+C): StopMove, avoidance state restored.
"""

import json
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState, WirelessController

MOVE, STOP_MOVE = 1008, 1003
AVOID_SET, AVOID_GET, AVOID_MOVE, AVOID_USE_API = 1001, 1002, 1003, 1004
CLASSIC = 2010


class Go2CmdBridge(Node):

    def __init__(self):
        super().__init__('go2_cmd_bridge')
        p = self.declare_parameter
        self.enabled = bool(p('enabled', False).value)
        self.route = p('route', 'sport').value
        # 'off' / 'on' on the command line arrive as booleans in ROS 2: accept both spellings
        av = p('obstacle_avoidance', 'leave', ParameterDescriptor(dynamic_typing=True)).value
        self.avoid_mode = {True: 'on', False: 'off', 'true': 'on', 'false': 'off'}.get(av, av)
        rc = p('request_classic', False, ParameterDescriptor(dynamic_typing=True)).value
        self.request_classic = rc in (True, 'on', 'true')
        self.max = (float(p('max_vx', 0.55).value), float(p('max_vy', 0.20).value), float(p('max_wz', 1.10).value))
        self.timeout = float(p('cmd_timeout_s', 0.5).value)
        self.stick_pause = float(p('remote_pause_s', 1.5).value)
        if self.route not in ('sport', 'avoid') or self.avoid_mode not in ('leave', 'on', 'off'):
            raise ValueError('route: sport|avoid, obstacle_avoidance: leave|on|off')
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sport_pub = self.create_publisher(Request, '/api/sport/request', rel)
        self.avoid_pub = self.create_publisher(Request, '/api/obstacles_avoid/request', rel)
        self.create_subscription(Twist, '/go2/nav2_cmd_vel', self.on_cmd, rel)
        self.create_subscription(SportModeState, '/sportmodestate', self.on_state, qos_profile_sensor_data)
        self.create_subscription(WirelessController, '/wirelesscontroller', self.on_remote, qos_profile_sensor_data)
        self.create_subscription(Response, '/api/obstacles_avoid/response', self.on_avoid, qos_profile_sensor_data)
        self.create_subscription(Response, '/api/sport/response', self.on_sport_response, qos_profile_sensor_data)
        self.cmd, self.cmd_t = (0.0, 0.0, 0.0), 0.0
        self.state = None
        self.remote_active = False     # last /wirelesscontroller message had a stick off centre or a button down
        self.remote_idle_t = -1e9      # when the remote last went back to idle
        self.avoid_found = None        # avoidance state seen at start (restored on exit)
        self.avoid_now = None
        self.gait_logged = False       # gait situation reported once at setup
        self.classic_reminder_t = 0.0  # last 'switch to Classic' reminder
        self.moving = False            # a Move was sent since the last StopMove
        self.sent_ids = {}
        self.last_log = 0.0
        self.setup_done = False
        self.closing = False
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().warn('ENABLED: this node will move the robot' if self.enabled else
                               'dry run (enabled false): nothing is sent to the robot')

    # ------------------------------------------------------------------ inputs
    def on_cmd(self, m):
        self.cmd, self.cmd_t = (m.linear.x, m.linear.y, m.angular.z), time.monotonic()

    def on_state(self, m):
        self.state = m

    def on_remote(self, m):
        active = max(abs(m.lx), abs(m.ly), abs(m.rx), abs(m.ry)) > 0.15 or m.keys != 0
        if active and not self.remote_active:
            self.get_logger().info('remote in use: bridge paused')
        elif not active and self.remote_active:
            self.remote_idle_t = time.monotonic()
        self.remote_active = active

    def remote_in_use(self, now):
        return self.remote_active or now - self.remote_idle_t < self.stick_pause

    def on_avoid(self, m):
        if m.header.identity.api_id == AVOID_GET:
            try:
                self.avoid_now = bool(json.loads(m.data).get('enable'))
            except (ValueError, AttributeError):
                return
            if self.avoid_found is None:
                self.avoid_found = self.avoid_now
                self.get_logger().info(f'obstacle avoidance found {"on" if self.avoid_now else "off"}')
        self.check_response(m)

    def on_sport_response(self, m):
        self.check_response(m)

    def check_response(self, m):
        api = self.sent_ids.pop(m.header.identity.id, None)
        if api is not None and m.header.status.code != 0:
            self.get_logger().error(f'robot rejected api {api}: status {m.header.status.code} {m.data[:100]}')

    # ------------------------------------------------------------------ outputs
    def send(self, pub, api, param=None):
        r = Request()
        r.header.identity.id = time.time_ns()
        r.header.identity.api_id = api
        r.parameter = json.dumps(param) if param is not None else ''
        if self.enabled:
            pub.publish(r)
            self.sent_ids[r.header.identity.id] = api
            if len(self.sent_ids) > 200:
                self.sent_ids.pop(next(iter(self.sent_ids)))
        return r

    def stop(self, why):
        if self.moving:
            self.send(self.sport_pub, STOP_MOVE)
            if self.route == 'avoid':
                self.send(self.avoid_pub, AVOID_MOVE, {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'mode': 0})
            self.moving = False
            self.get_logger().info(f'StopMove ({why})')

    def setup(self):
        """Once, when enabled and the robot's state is known: report the gait, set the avoidance switch, API route."""
        if self.state is None:
            return False
        # Gait: never changed by this node. request_classic on -> the operator is asked to select Classic on the remote
        # (the ClassicWalk API request did not switch this robot live 2026-10-17: it stayed in gait 100), Move waits.
        if not self.gait_logged:
            self.gait_logged = True
            if not self.request_classic:
                self.get_logger().info(f'request_classic off: driving in the current gait (error_code {self.state.error_code})')
            elif self.state.error_code != CLASSIC:
                self.get_logger().warn(f'robot is not in Classic (error_code {self.state.error_code}): '
                                       'switch it to Classic with the remote; waiting')
            else:
                self.get_logger().info('robot is in Classic')
        if self.avoid_mode != 'leave':
            if self.avoid_found is None:
                return False                       # wait to learn the current state, to restore it later
            want = self.avoid_mode == 'on'
            if self.avoid_now != want:
                self.send(self.avoid_pub, AVOID_SET, {'enable': want})
                self.get_logger().warn(f'obstacle avoidance -> {"on" if want else "off"}')
        if self.route == 'avoid':
            self.send(self.avoid_pub, AVOID_USE_API, {'is_remote_commands_from_api': True})
        return True

    def tick(self):
        if self.closing:
            return
        now = time.monotonic()
        if self.enabled and not self.setup_done:
            self.setup_done = self.setup()
            if not self.setup_done:
                return
        vx, vy, wz = self.cmd
        vx = max(-self.max[0], min(self.max[0], vx))
        vy = max(-self.max[1], min(self.max[1], vy))
        wz = max(-self.max[2], min(self.max[2], wz))
        reason = None
        if now - self.cmd_t > self.timeout:
            reason = 'no command'
        elif abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3:
            reason = 'zero command'
        elif self.remote_in_use(now):
            reason = 'remote in use'
        elif self.state is None:
            reason = 'no /sportmodestate'
        elif self.request_classic and self.state.error_code != CLASSIC:
            if now - self.classic_reminder_t > 3.0:            # remind the operator every 3 s while Nav2 wants to move
                self.get_logger().warn(f'robot is not in Classic (error_code {self.state.error_code}): '
                                       'switch it to Classic with the remote; waiting')
                self.classic_reminder_t = now
            self.stop('waiting for Classic')
            return
        elif self.state.body_height < 0.2:
            reason = f'not standing (body height {self.state.body_height:.2f} m)'
        if reason:
            self.stop(reason)
            if now - self.last_log > 5.0 and reason not in ('no command', 'zero command'):
                self.get_logger().info(f'holding: {reason}')
                self.last_log = now
            return
        if self.route == 'avoid':
            self.send(self.avoid_pub, AVOID_MOVE, {'x': vx, 'y': vy, 'yaw': wz, 'mode': 0})
        else:
            self.send(self.sport_pub, MOVE, {'x': vx, 'y': vy, 'z': wz})
        self.moving = True
        if now - self.last_log > 1.0:
            self.get_logger().info(f'{"Move" if self.enabled else "[dry run] would send Move"} '
                                   f'x {vx:+.2f} y {vy:+.2f} yaw {wz:+.2f}')
            self.last_log = now

    def shutdown(self):
        # stop the 10 Hz loop FIRST: the shutdown flush spins the node, and the loop sent a Move after the final
        # StopMove in the offline test (nav_test_tools/bridge_cmd_test.py)
        self.closing = True
        self.timer.cancel()
        self.moving = True
        self.stop('exit')
        if self.enabled and self.route == 'avoid':
            self.send(self.avoid_pub, AVOID_USE_API, {'is_remote_commands_from_api': False})
        if self.enabled and self.avoid_mode != 'leave' and self.avoid_found is not None and self.avoid_now != self.avoid_found:
            self.send(self.avoid_pub, AVOID_SET, {'enable': self.avoid_found})
            self.get_logger().warn(f'obstacle avoidance restored to {"on" if self.avoid_found else "off"}')


def main():
    # Ctrl+C is handled here, not by rclpy: rclpy's handler would close the connection before the final StopMove and
    # the avoidance restore could be sent (found in the offline test, nav_test_tools/bridge_cmd_test.py).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)       # explicit: a background start may inherit SIGINT as ignored
    signal.signal(signal.SIGTERM, interrupt)      # kill / launch shutdown also stop the robot first
    n = Go2CmdBridge()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            n.shutdown()
            end = time.monotonic() + 0.5          # let the final StopMove / restore leave
            while time.monotonic() < end:
                rclpy.spin_once(n, timeout_sec=0.05)
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
