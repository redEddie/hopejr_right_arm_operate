"""
Motor State Publisher Node
──────────────────────────
Feetech 서보에서 position, velocity, load를 sync_read하여
/motor_states (sensor_msgs/JointState)로 발행한다.

- position: 정규화된 모터 위치 (-100~100 또는 0~100)
- velocity: 모터 속도 (raw)
- effort:   모터 부하 (raw) → 텐돈 장력의 간접 측정치
"""
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus


ARM_MOTORS = {
    "shoulder_pitch":       Motor(1, "sm8512bl", MotorNormMode.RANGE_M100_100),
    "shoulder_yaw":         Motor(2, "sts3250", MotorNormMode.RANGE_M100_100),
    "shoulder_roll":        Motor(3, "sts3250", MotorNormMode.RANGE_M100_100),
    "elbow_flex":           Motor(4, "sts3250", MotorNormMode.RANGE_M100_100),
    "wrist_roll":           Motor(5, "sts3250", MotorNormMode.RANGE_M100_100),
    "wrist_yaw":            Motor(6, "sts3250", MotorNormMode.RANGE_M100_100),
    "wrist_pitch":          Motor(7, "sts3250", MotorNormMode.RANGE_M100_100),
}

HAND_MOTORS = {
    "thumb_cmc":            Motor(1, "scs0009", MotorNormMode.RANGE_0_100),
    "thumb_mcp":            Motor(2, "scs0009", MotorNormMode.RANGE_0_100),
    "thumb_pip":            Motor(3, "scs0009", MotorNormMode.RANGE_0_100),
    "thumb_dip":            Motor(4, "scs0009", MotorNormMode.RANGE_0_100),
    "index_radial_flexor":  Motor(5, "scs0009", MotorNormMode.RANGE_0_100),
    "index_ulnar_flexor":   Motor(6, "scs0009", MotorNormMode.RANGE_0_100),
    "index_pip_dip":        Motor(7, "scs0009", MotorNormMode.RANGE_0_100),
    "middle_radial_flexor": Motor(8, "scs0009", MotorNormMode.RANGE_0_100),
    "middle_ulnar_flexor":  Motor(9, "scs0009", MotorNormMode.RANGE_0_100),
    "middle_pip_dip":       Motor(10, "scs0009", MotorNormMode.RANGE_0_100),
    "ring_radial_flexor":   Motor(11, "scs0009", MotorNormMode.RANGE_0_100),
    "ring_ulnar_flexor":    Motor(12, "scs0009", MotorNormMode.RANGE_0_100),
    "ring_pip_dip":         Motor(13, "scs0009", MotorNormMode.RANGE_0_100),
    "pinky_radial_flexor":  Motor(14, "scs0009", MotorNormMode.RANGE_0_100),
    "pinky_ulnar_flexor":   Motor(15, "scs0009", MotorNormMode.RANGE_0_100),
    "pinky_pip_dip":        Motor(16, "scs0009", MotorNormMode.RANGE_0_100),
}


class SafeReadBus:
    """읽기 전용 SafeBus — 모터 상태를 sync_read한다."""

    def __init__(self, port, motors, proto, name):
        self.bus = FeetechMotorsBus(port=port, motors=motors, protocol_version=proto)
        self.name = name
        self.motor_names = list(motors.keys())
        try:
            self.bus.connect()
        except RuntimeError as e:
            if "firmware versions" in str(e):
                print(f"{name}: Firmware mismatch OK - opening manually")
                self.bus.port_handler.openPort()
            else:
                raise

        self.bus.calibration = self.bus.read_calibration()
        print(f"✓ {name} State Reader Ready")

    def read_position(self):
        try:
            return self.bus.sync_read("Present_Position")
        except Exception as e:
            print(f"{self.name} read position error: {e}")
            return {}

    def read_velocity(self):
        try:
            return self.bus.sync_read("Present_Speed")
        except Exception as e:
            print(f"{self.name} read velocity error: {e}")
            return {}

    def read_load(self):
        try:
            return self.bus.sync_read("Present_Load")
        except Exception as e:
            print(f"{self.name} read load error: {e}")
            return {}


class MotorStatePublisher(Node):
    def __init__(self, mode, serial_port, rate=50.0):
        super().__init__(f'hopejr_{mode}_motor_state_pub')
        self.mode = mode

        if mode == "arm":
            self.bus = SafeReadBus(serial_port, ARM_MOTORS, 0, "ARM_READ")
            self.motor_names = list(ARM_MOTORS.keys())
            topic = '/arm_motor_states'
        else:
            self.bus = SafeReadBus(serial_port, HAND_MOTORS, 1, "HAND_READ")
            self.motor_names = list(HAND_MOTORS.keys())
            topic = '/hand_motor_states'

        self.pub = self.create_publisher(JointState, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_state)
        self.get_logger().info(
            f'Motor state publisher [{mode}] on {serial_port} at {rate}Hz -> {topic}'
        )

    def publish_state(self):
        positions = self.bus.read_position()
        velocities = self.bus.read_velocity()
        loads = self.bus.read_load()

        if not positions:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.motor_names
        msg.position = [float(positions.get(n, 0.0)) for n in self.motor_names]
        msg.velocity = [float(velocities.get(n, 0.0)) for n in self.motor_names]
        msg.effort = [float(loads.get(n, 0.0)) for n in self.motor_names]
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--type", choices=["arm", "hand"], required=True)
    parser.add_argument("--serial", required=True, help="Serial port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--rate", type=float, default=50.0, help="Publish rate in Hz")
    args = parser.parse_args()

    rclpy.init()
    node = MotorStatePublisher(args.type, args.serial, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\nShutting down {args.type} state publisher...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
