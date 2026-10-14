#!/usr/bin/env python3
"""Fail-closed recovery for LEFT incremental-encoder steering after ESC reboot."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String


def state_qos():
    q = QoSProfile(depth=20)
    q.reliability = ReliabilityPolicy.RELIABLE
    q.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return q


class SteeringRecoveryGuard(Node):
    def __init__(self):
        super().__init__('steering_recovery_guard')
        self.cmd_pub = self.create_publisher(String, '/esc/vesc/tool_command', 10)
        self.status_pub = self.create_publisher(String, '/esc/steering_recovery_status', state_qos())
        self.create_subscription(String, '/esc/foc/telemetry', self.on_foc, 20)
        self.create_subscription(String, '/esc/mux/active_source', self.on_source, state_qos())
        self.create_subscription(Float64, '/esc/speed', self.on_speed, 10)
        self.create_subscription(Bool, '/esc/vesc/maintenance_active', self.on_maintenance, state_qos())
        self.create_subscription(String, '/esc/vesc/steering_state', self.on_steering_state, state_qos())
        self.telemetry = None
        self.telemetry_at = 0.0
        self.source = 'UNKNOWN'
        self.speed_mps = 0.0
        self.maintenance = False
        self.state = 'IDLE'
        self.state_since = time.monotonic()
        self.unsynced_since = None
        self.last_keepalive = 0.0
        self.attempts = 0
        self.home_reply = None
        self.verify_reply = None
        self.cooldown_until = 0.0
        self.timer = self.create_timer(0.1, self.tick)
        self.publish_status('IDLE', 'guard started')

    def publish_cmd(self, text):
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)

    def publish_status(self, state, detail):
        msg = String()
        msg.data = json.dumps({'state': state, 'detail': detail, 'attempts': self.attempts})
        self.status_pub.publish(msg)

    def transition(self, state, detail=''):
        self.state = state
        self.state_since = time.monotonic()
        self.publish_status(state, detail)
    def on_source(self, msg):
        self.source = msg.data.strip().upper()

    def on_speed(self, msg):
        if math.isfinite(msg.data):
            self.speed_mps = msg.data

    def on_maintenance(self, msg):
        self.maintenance = bool(msg.data)

    def on_foc(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.telemetry = data
        self.telemetry_at = time.monotonic()

    def on_steering_state(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        op = data.get('op')
        if op == 13:
            self.home_reply = data
        elif op == 10:
            self.verify_reply = data

    def safe_unsynced(self):
        if not self.telemetry or time.monotonic() - self.telemetry_at > 0.7:
            return False, 'telemetry stale'
        left = self.telemetry.get('left') or {}
        cal = self.telemetry.get('steering_cal') or {}
        if not bool(cal.get('calibrated')):
            return False, 'steering calibration missing'
        if bool(cal.get('homed')) and bool(cal.get('encoder_synced')):
            return False, 'steering already ready'
        if int(left.get('fault', 255)) != 0:
            return False, f"left fault={left.get('fault')}"
        if float(left.get('vbus_v', 0.0)) < 20.0:
            return False, 'bus voltage too low'
        if abs(self.speed_mps) > 0.03:
            return False, 'vehicle moving'
        if self.source not in ('IDLE', 'STOP', 'TELEOP_STALE_STOP'):
            return False, f'active source={self.source}'
        pos = float(left.get('position_deg', 999.0))
        if not math.isfinite(pos) or abs(pos) > 3.0:
            return False, f'not near center ({pos:.2f} deg)'
        return True, 'safe unsynced steering'

    def fail_to_runtime(self, detail):
        if self.maintenance:
            self.publish_cmd('SAFE_STOP:BOTH')
            self.publish_cmd('MODE:RUNTIME')
        self.cooldown_until = time.monotonic() + 3.0
        self.transition('COOLDOWN', detail)

    def tick(self):
        now = time.monotonic()
        ok, reason = self.safe_unsynced()
        if self.state == 'IDLE':
            if not ok:
                self.unsynced_since = None
                return
            if self.unsynced_since is None:
                self.unsynced_since = now
                return
            if now - self.unsynced_since < 1.0:
                return
            self.attempts = 0
            self.home_reply = None
            self.verify_reply = None
            self.publish_cmd('MODE:MAINTENANCE')
            self.transition('ENTERING', reason)
            return

        if self.state == 'ENTERING':
            if self.maintenance:
                self.attempts += 1
                self.publish_cmd('KEEPALIVE:WEB')
                self.publish_cmd('STEERING:HOME')
                self.last_keepalive = now
                self.transition('HOMING', f'HOME attempt {self.attempts}')
            elif now - self.state_since > 3.0:
                self.fail_to_runtime('maintenance entry timeout')
            return

        if self.state == 'HOMING':
            if now - self.last_keepalive > 0.8:
                self.publish_cmd('KEEPALIVE:WEB')
                self.last_keepalive = now
            if self.home_reply is not None:
                h = self.home_reply
                good = int(h.get('status', 99)) == 0 and bool(h.get('homed')) and bool(h.get('encoder_synced'))
                if good:
                    self.verify_reply = None
                    self.publish_cmd('STEERING:CAL')
                    self.transition('VERIFYING', 'HOME passed')
                elif self.attempts < 2 and ok:
                    self.home_reply = None
                    self.publish_cmd('STEERING:HOME')
                    self.attempts += 1
                    self.last_keepalive = now
                    self.publish_status('HOMING', f'retry HOME {self.attempts}')
                else:
                    self.fail_to_runtime(f'HOME failed: {h}')
            elif now - self.state_since > 20.0:
                self.fail_to_runtime('HOME reply timeout')
            return

        if self.state == 'VERIFYING':
            if now - self.last_keepalive > 0.8:
                self.publish_cmd('KEEPALIVE:WEB')
                self.last_keepalive = now
            if self.verify_reply is not None:
                v = self.verify_reply
                good = int(v.get('status', 99)) == 0 and bool(v.get('homed')) and bool(v.get('encoder_synced'))
                if good:
                    self.publish_cmd('MODE:RUNTIME')
                    self.transition('EXITING', 'verified homed+synced')
                else:
                    self.fail_to_runtime(f'verification failed: {v}')
            elif now - self.state_since > 3.0:
                self.publish_cmd('STEERING:CAL')
                self.state_since = now
            return

        if self.state == 'EXITING':
            if not self.maintenance:
                self.unsynced_since = None
                self.cooldown_until = now + 2.0
                self.transition('COOLDOWN', 'runtime restored')
            elif now - self.state_since > 3.0:
                self.publish_cmd('MODE:RUNTIME')
            return

        if self.state == 'COOLDOWN':
            if now >= self.cooldown_until:
                self.transition('IDLE', 'armed for next reconnect')


def main():
    rclpy.init()
    node = SteeringRecoveryGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
