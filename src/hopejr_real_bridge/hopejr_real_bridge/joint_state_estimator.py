"""
Joint State Estimator Node
───────────────────────────
/arm_motor_states + /hand_motor_states를 구독하여
27 DOF /estimated_joint_states (sensor_msgs/JointState)로 발행한다.

Joint Space 정의 (27 DOF):
  팔  7: shoulder_pitch/yaw/roll, elbow_flex, wrist_roll/yaw/pitch
  엄지 4: thumb_cmc(직결), thumb_mcp/pip/dip(텐돈+스프링)
  검지 4: index_mcp_abduction, index_mcp_flexion, index_pip, index_dip
  중지 4: middle_mcp_abduction, middle_mcp_flexion, middle_pip, middle_dip
  약지 4: ring_mcp_abduction, ring_mcp_flexion, ring_pip, ring_dip
  새끼 4: pinky_mcp_abduction, pinky_mcp_flexion, pinky_pip, pinky_dip

Motor Space (23 모터):
  팔 7: 직결
  엄지 4: thumb_cmc(직결), thumb_mcp/pip/dip(각 1텐돈)
  검지~새끼 각 3: radial_flexor, ulnar_flexor, pip_dip

Underactuated DOF 4개:
  검지~새끼의 dip — pip와 텐돈 공유, 스프링 비율로 분리 추정
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .config import (
    ARM_MOTORS_LIMITS,
    HAND_MOTORS_LIMITS,
    SPRING_CONSTANTS,
    MOMENT_ARMS,
    LOAD_TO_TENSION_SCALE,
    TENDON_MM_PER_MOTOR_UNIT,
)
from .calculate_jm import (
    thumb_motor_to_joint,
    pip_dip_motor_to_joint,
    radial_ulnar_motor_to_joint,
)


# 전체 관절 이름 (27 DOF)
ARM_JOINT_NAMES = [
    'shoulder_pitch', 'shoulder_yaw', 'shoulder_roll',
    'elbow_flex', 'wrist_roll', 'wrist_yaw', 'wrist_pitch',
]

THUMB_JOINT_NAMES = [
    'thumb_cmc', 'thumb_mcp', 'thumb_pip', 'thumb_dip',
]

FINGER_NAMES = ['index', 'middle', 'ring', 'pinky']

def finger_joint_names(finger):
    return [
        f'{finger}_mcp_abduction',
        f'{finger}_mcp_flexion',
        f'{finger}_pip',
        f'{finger}_dip',
    ]

ALL_JOINT_NAMES = (
    ARM_JOINT_NAMES
    + THUMB_JOINT_NAMES
    + [name for f in FINGER_NAMES for name in finger_joint_names(f)]
)


class JointStateEstimator(Node):
    def __init__(self):
        super().__init__('hopejr_joint_state_estimator')

        # 최신 motor states 저장
        self.arm_motors = {}   # name -> (position, velocity, effort)
        self.hand_motors = {}

        self.arm_sub = self.create_subscription(
            JointState, '/arm_motor_states', self.arm_motor_cb, 10)
        self.hand_sub = self.create_subscription(
            JointState, '/hand_motor_states', self.hand_motor_cb, 10)

        self.joint_pub = self.create_publisher(
            JointState, '/estimated_joint_states', 10)

        # 50Hz로 발행
        self.timer = self.create_timer(0.02, self.publish_estimated_joints)

        self.get_logger().info(
            f'Joint State Estimator started — {len(ALL_JOINT_NAMES)} DOF'
        )

    def arm_motor_cb(self, msg):
        for i, name in enumerate(msg.name):
            self.arm_motors[name] = (
                msg.position[i] if i < len(msg.position) else 0.0,
                msg.velocity[i] if i < len(msg.velocity) else 0.0,
                msg.effort[i] if i < len(msg.effort) else 0.0,
            )

    def hand_motor_cb(self, msg):
        for i, name in enumerate(msg.name):
            self.hand_motors[name] = (
                msg.position[i] if i < len(msg.position) else 0.0,
                msg.velocity[i] if i < len(msg.velocity) else 0.0,
                msg.effort[i] if i < len(msg.effort) else 0.0,
            )

    # ── 팔: 정규화된 모터 위치 → radian ──
    def denormalize_arm(self, motor_name, normalized):
        """normalized (-100~100) → 실제 각도(degree) → radian"""
        limits = ARM_MOTORS_LIMITS.get(motor_name)
        if limits is None:
            return 0.0
        min_deg, max_deg = limits
        deg = (normalized + 100.0) / 200.0 * (max_deg - min_deg) + min_deg
        return math.radians(deg)

    # ── 엄지: 모터 위치 → 관절 각도 ──
    def estimate_thumb(self):
        """thumb_cmc: 직결, thumb_mcp/pip/dip: 텐돈 순변환"""
        joints = {}

        # thumb_cmc — 직결 모터. 정규화 역변환
        cmc_motor = self.hand_motors.get('thumb_cmc', (0.0, 0.0, 0.0))
        cmc_pos = cmc_motor[0]
        limits = HAND_MOTORS_LIMITS.get('thumb_cmc')
        if limits:
            min_v, max_v = limits
            joints['thumb_cmc'] = cmc_pos / 100.0 * (max_v - min_v) + min_v
        else:
            joints['thumb_cmc'] = 0.0

        # thumb_mcp, pip, dip — 각각 독립 텐돈 + 스프링
        for jname in ['thumb_mcp', 'thumb_pip', 'thumb_dip']:
            motor = self.hand_motors.get(jname, (0.0, 0.0, 0.0))
            motor_pos = motor[0]
            # 정규화 역변환: 0~100 → min~max (radian 스케일)
            limits = HAND_MOTORS_LIMITS.get(jname)
            if limits:
                min_v, max_v = limits
                denorm = motor_pos / 100.0 * (max_v - min_v) + min_v
            else:
                denorm = 0.0
            # 기구학 순변환 (모터 → 관절)
            try:
                joints[jname] = thumb_motor_to_joint(denorm)
            except (ValueError, ZeroDivisionError):
                joints[jname] = 0.0

        return joints

    # ── 검지~새끼: 3 모터 → 4 관절 (pip-dip 분리 추정) ──
    def estimate_finger(self, finger):
        """
        radial + ulnar → mcp_abduction, mcp_flexion
        pip_dip 모터  → pip, dip (부하 기반 분리)
        """
        joints = {}

        # (A) Abduction / Flexion: 기존 radial_ulnar 역변환 사용
        rad_motor = self.hand_motors.get(f'{finger}_radial_flexor', (0.0, 0.0, 0.0))
        uln_motor = self.hand_motors.get(f'{finger}_ulnar_flexor', (0.0, 0.0, 0.0))

        # 정규화 역변환
        rad_pos = rad_motor[0]
        uln_pos = uln_motor[0]
        rad_limits = HAND_MOTORS_LIMITS.get(f'{finger}_radial_flexor')
        uln_limits = HAND_MOTORS_LIMITS.get(f'{finger}_ulnar_flexor')
        if rad_limits:
            rad_denorm = rad_pos / 100.0 * (rad_limits[1] - rad_limits[0]) + rad_limits[0]
        else:
            rad_denorm = 0.0
        if uln_limits:
            uln_denorm = uln_pos / 100.0 * (uln_limits[1] - uln_limits[0]) + uln_limits[0]
        else:
            uln_denorm = 0.0

        try:
            alpha, beta = radial_ulnar_motor_to_joint(rad_denorm, uln_denorm)
            joints[f'{finger}_mcp_abduction'] = float(alpha)
            joints[f'{finger}_mcp_flexion'] = float(beta)
        except Exception:
            joints[f'{finger}_mcp_abduction'] = 0.0
            joints[f'{finger}_mcp_flexion'] = 0.0

        # (B) PIP-DIP 분리 추정
        pipdip_motor = self.hand_motors.get(f'{finger}_pip_dip', (0.0, 0.0, 0.0))
        pipdip_pos = pipdip_motor[0]
        pipdip_load = pipdip_motor[2]  # effort = 부하

        pipdip_limits = HAND_MOTORS_LIMITS.get(f'{finger}_pip_dip')
        if pipdip_limits:
            min_v, max_v = pipdip_limits
            pipdip_denorm = pipdip_pos / 100.0 * (max_v - min_v) + min_v
        else:
            pipdip_denorm = 0.0

        # 전체 굽힘 각도 (텐돈 길이 구속)
        try:
            total_flexion = pip_dip_motor_to_joint(pipdip_denorm)
        except (ValueError, ZeroDivisionError):
            total_flexion = 0.0

        # 스프링 비율로 pip/dip 분리
        #   정적 평형: T * r_pip = k_pip * θ_pip
        #              T * r_dip = k_dip * θ_dip
        #   → θ_pip / θ_dip = (k_dip * r_pip) / (k_pip * r_dip)
        #   → θ_pip + θ_dip = total_flexion
        k_pip = SPRING_CONSTANTS.get(f'{finger}_pip', 5.0)
        k_dip = SPRING_CONSTANTS.get(f'{finger}_dip', 5.0)
        r_pip = MOMENT_ARMS.get(f'{finger}_pip', 2.25)
        r_dip = MOMENT_ARMS.get(f'{finger}_dip', 2.25)

        # ratio = θ_pip / θ_dip
        denom = k_pip * r_dip
        if abs(denom) < 1e-9:
            ratio = 1.0
        else:
            ratio = (k_dip * r_pip) / denom

        # θ_pip = ratio * θ_dip
        # ratio * θ_dip + θ_dip = total_flexion
        # θ_dip = total_flexion / (ratio + 1)
        if abs(ratio + 1.0) < 1e-9:
            theta_dip = total_flexion / 2.0
        else:
            theta_dip = total_flexion / (ratio + 1.0)
        theta_pip = total_flexion - theta_dip

        # 부하 기반 보정: 외부 하중이 있으면 비율이 바뀜
        # T = load * scale
        scale = LOAD_TO_TENSION_SCALE.get(f'{finger}_pip_dip', 0.01)
        tension = abs(pipdip_load) * scale

        # 외력이 dip에 가해지면 dip 스프링이 더 눌림 → dip 각도 증가
        # 보정량: Δθ_dip ≈ T * r_dip / k_dip - θ_dip (from static eq.)
        # 단, 캘리브레이션 전에는 소량 보정만 적용
        if tension > 0 and k_dip > 0:
            theta_dip_from_tension = (tension * r_dip) / k_dip
            theta_pip_from_tension = (tension * r_pip) / k_pip

            # 텐돈 구속 유지: 합은 total_flexion
            total_from_tension = theta_pip_from_tension + theta_dip_from_tension
            if abs(total_from_tension) > 1e-9:
                # 장력 기반 비율로 재분배
                theta_pip = total_flexion * (theta_pip_from_tension / total_from_tension)
                theta_dip = total_flexion * (theta_dip_from_tension / total_from_tension)

        joints[f'{finger}_pip'] = max(0.0, float(theta_pip))
        joints[f'{finger}_dip'] = max(0.0, float(theta_dip))

        return joints

    def publish_estimated_joints(self):
        if not self.arm_motors and not self.hand_motors:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINT_NAMES)

        positions = {}
        velocities = {}
        efforts = {}

        # ── 팔 7 DOF ──
        for name in ARM_JOINT_NAMES:
            motor = self.arm_motors.get(name, (0.0, 0.0, 0.0))
            positions[name] = self.denormalize_arm(name, motor[0])
            velocities[name] = motor[1]
            efforts[name] = motor[2]

        # ── 엄지 4 DOF ──
        thumb_joints = self.estimate_thumb()
        for name in THUMB_JOINT_NAMES:
            positions[name] = thumb_joints.get(name, 0.0)
            motor_key = name  # 엄지는 motor name = joint name
            motor = self.hand_motors.get(motor_key, (0.0, 0.0, 0.0))
            velocities[name] = motor[1]
            efforts[name] = motor[2]

        # ── 검지~새끼 각 4 DOF ──
        for finger in FINGER_NAMES:
            finger_joints = self.estimate_finger(finger)
            for jname in finger_joint_names(finger):
                positions[jname] = finger_joints.get(jname, 0.0)

            # velocity/effort: abduction/flexion → radial motor 대표
            rad = self.hand_motors.get(f'{finger}_radial_flexor', (0.0, 0.0, 0.0))
            uln = self.hand_motors.get(f'{finger}_ulnar_flexor', (0.0, 0.0, 0.0))
            pipdip = self.hand_motors.get(f'{finger}_pip_dip', (0.0, 0.0, 0.0))

            velocities[f'{finger}_mcp_abduction'] = rad[1] - uln[1]
            velocities[f'{finger}_mcp_flexion'] = (rad[1] + uln[1]) / 2.0
            velocities[f'{finger}_pip'] = pipdip[1]
            velocities[f'{finger}_dip'] = pipdip[1]  # 같은 텐돈

            efforts[f'{finger}_mcp_abduction'] = abs(rad[2]) + abs(uln[2])
            efforts[f'{finger}_mcp_flexion'] = (rad[2] + uln[2]) / 2.0
            efforts[f'{finger}_pip'] = pipdip[2]
            efforts[f'{finger}_dip'] = pipdip[2]

        msg.position = [positions.get(n, 0.0) for n in ALL_JOINT_NAMES]
        msg.velocity = [velocities.get(n, 0.0) for n in ALL_JOINT_NAMES]
        msg.effort = [efforts.get(n, 0.0) for n in ALL_JOINT_NAMES]
        self.joint_pub.publish(msg)


def main():
    rclpy.init()
    node = JointStateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down joint state estimator...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
