"""
Calibration Tool
────────────────
스프링 강성(k), 부하-장력 스케일(T), 모멘트 암(r)을 실측 데이터로 튜닝한다.

사용법:
  ros2 run hopejr_real_bridge calibration --mode spring_k
  ros2 run hopejr_real_bridge calibration --mode tension_scale
  ros2 run hopejr_real_bridge calibration --mode moment_arm
  ros2 run hopejr_real_bridge calibration --mode full

캘리브레이션 원리:
──────────────────
1. spring_k (스프링 강성 측정)
   - 토크를 끄고 손가락을 수동으로 굽힌 뒤 놓아서 스프링으로 복귀시킨다
   - 여러 각도에서 모터 부하(정적)를 측정하여 k = T*r / θ 로 계산

2. tension_scale (부하→장력 변환 계수)
   - 알려진 무게(force gauge)를 텐돈에 걸고 모터 부하 값을 읽는다
   - scale = F_known / load_raw

3. moment_arm (모멘트 암 — 각도 의존 비선형 함수)
   - 여러 관절 각도에서 (모터위치, 부하, 실제각도)를 수집
   - r(θ) = T / (k * θ) 로 각도별 r을 계산
   - 다항식 또는 스플라인으로 r(θ) 함수를 피팅

결과는 JSON 파일로 저장되어 config.py 파라미터를 대체한다.
"""
import argparse
import json
import time
import os
import math
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


CALIBRATION_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'calibration_data'
)

FINGER_NAMES = ['index', 'middle', 'ring', 'pinky']
THUMB_TENDON_JOINTS = ['thumb_mcp', 'thumb_pip', 'thumb_dip']


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('hopejr_calibration')
        self.hand_motors = {}
        self.hand_sub = self.create_subscription(
            JointState, '/hand_motor_states', self.hand_cb, 10)
        os.makedirs(CALIBRATION_DIR, exist_ok=True)

    def hand_cb(self, msg):
        for i, name in enumerate(msg.name):
            self.hand_motors[name] = {
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'load': msg.effort[i] if i < len(msg.effort) else 0.0,
            }

    def wait_for_data(self, timeout=5.0):
        """motor_state_publisher가 데이터를 보낼 때까지 대기"""
        start = time.time()
        while not self.hand_motors and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.hand_motors:
            self.get_logger().error(
                'No motor state data received. '
                'Is motor_state_pub running? '
                '(ros2 run hopejr_real_bridge motor_state_pub --type hand --serial /dev/ttyUSBx)'
            )
            return False
        return True

    def collect_sample(self):
        """현재 시점의 모터 상태 스냅샷을 반환"""
        rclpy.spin_once(self, timeout_sec=0.1)
        return dict(self.hand_motors)

    # ================================================================
    # 1. 스프링 강성 캘리브레이션
    # ================================================================
    def calibrate_spring_k(self):
        """
        절차:
        1) 모터 토크를 끈다 (teleop_server 종료 상태)
        2) 사용자가 손가락을 여러 각도로 굽힘
        3) 각 각도에서 정적 상태의 (position, load)를 기록
        4) k = (load * scale * r) / θ 에서 k를 최소자승 피팅

        주의: tension_scale과 moment_arm이 대략적으로라도 알려져 있어야 한다.
              아직 모르면 기본값(config.py)을 사용하고, 나중에 반복 캘리브레이션한다.
        """
        print("\n" + "=" * 60)
        print("  스프링 강성 (k) 캘리브레이션")
        print("=" * 60)
        print("준비사항:")
        print("  - motor_state_pub 노드가 실행 중이어야 합니다")
        print("  - teleop_server(모터 토크)는 꺼져 있어야 합니다")
        print("  - 각 손가락을 수동으로 여러 각도로 굽혀주세요")
        print()

        if not self.wait_for_data():
            return

        results = {}
        targets = THUMB_TENDON_JOINTS + [
            f'{f}_pip_dip' for f in FINGER_NAMES
        ]

        for motor_name in targets:
            if motor_name not in self.hand_motors:
                print(f"  [{motor_name}] 모터를 찾을 수 없습니다. 건너뜁니다.")
                continue

            print(f"\n--- {motor_name} 캘리브레이션 ---")
            print("손가락을 자유 상태(펴진 상태)로 둔 뒤 Enter를 누르세요.")
            input(">>> ")
            free_sample = self.collect_sample()
            free_pos = free_sample.get(motor_name, {}).get('position', 0.0)

            samples = []
            print(f"이제 손가락을 다양한 각도로 굽히며 각 위치에서 Enter를 누르세요.")
            print("'q'를 입력하면 이 모터의 캘리브레이션을 종료합니다.")

            while True:
                user_input = input(f"  [{motor_name}] 샘플 #{len(samples)+1} (Enter/q): ")
                if user_input.strip().lower() == 'q':
                    break
                sample = self.collect_sample()
                motor_data = sample.get(motor_name, {})
                pos = motor_data.get('position', 0.0)
                load = motor_data.get('load', 0.0)
                displacement = abs(pos - free_pos)
                samples.append({
                    'position': pos,
                    'load': load,
                    'displacement': displacement,
                })
                print(f"    pos={pos:.3f}, load={load:.3f}, Δ={displacement:.3f}")

            if len(samples) >= 2:
                # 최소자승: k ≈ mean(load / displacement) * scale * r
                # 단순화: k_relative = Σ(load * displacement) / Σ(displacement²)
                disps = np.array([s['displacement'] for s in samples])
                loads = np.array([abs(s['load']) for s in samples])
                # load ∝ k * displacement → k_ratio = load / displacement
                valid = disps > 0.01
                if valid.any():
                    k_est = float(np.mean(loads[valid] / disps[valid]))
                    results[motor_name] = k_est
                    print(f"  → 추정 k (상대값) = {k_est:.4f}")
                else:
                    print(f"  → 유효한 샘플 부족")
            else:
                print(f"  → 샘플 부족 (최소 2개 필요)")

        # 저장
        output = {
            'type': 'spring_k',
            'timestamp': datetime.now().isoformat(),
            'values': results,
        }
        path = os.path.join(CALIBRATION_DIR, 'spring_k.json')
        with open(path, 'w') as f:
            json.dump(output, f, indent=2)
        print(f"\n결과 저장: {path}")
        self._print_config_update('SPRING_CONSTANTS', results)

    # ================================================================
    # 2. 부하-장력 스케일 캘리브레이션
    # ================================================================
    def calibrate_tension_scale(self):
        """
        절차:
        1) 알려진 힘(무게추 또는 force gauge)을 텐돈에 건다
        2) 모터 부하 값을 읽는다
        3) scale = F_known(N) / load_raw
        """
        print("\n" + "=" * 60)
        print("  부하→장력 변환 스케일 (T) 캘리브레이션")
        print("=" * 60)
        print("준비사항:")
        print("  - force gauge 또는 알려진 무게추가 필요합니다")
        print("  - motor_state_pub 노드가 실행 중이어야 합니다")
        print()

        if not self.wait_for_data():
            return

        results = {}
        targets = THUMB_TENDON_JOINTS + [
            f'{f}_pip_dip' for f in FINGER_NAMES
        ]

        for motor_name in targets:
            if motor_name not in self.hand_motors:
                continue

            print(f"\n--- {motor_name} ---")
            print("텐돈에 아무 힘도 가하지 않은 상태에서 Enter")
            input(">>> ")
            zero_load = self.collect_sample().get(motor_name, {}).get('load', 0.0)

            print("알려진 힘(N)을 텐돈에 가한 뒤 그 힘의 크기를 입력하세요.")
            print("여러 번 측정 가능. 'q'로 종료.")
            scales = []

            while True:
                force_str = input(f"  가한 힘 (N, 또는 q): ")
                if force_str.strip().lower() == 'q':
                    break
                try:
                    force_n = float(force_str)
                except ValueError:
                    print("  숫자를 입력하세요.")
                    continue

                sample = self.collect_sample()
                current_load = sample.get(motor_name, {}).get('load', 0.0)
                delta_load = abs(current_load - zero_load)
                if delta_load > 0.001:
                    s = force_n / delta_load
                    scales.append(s)
                    print(f"    load_raw={current_load:.3f}, Δload={delta_load:.3f}, scale={s:.6f}")
                else:
                    print(f"    부하 변화 없음 — 힘이 너무 약하거나 감지 안 됨")

            if scales:
                avg_scale = float(np.mean(scales))
                results[motor_name] = avg_scale
                print(f"  → 평균 scale = {avg_scale:.6f} N/unit")

        output = {
            'type': 'tension_scale',
            'timestamp': datetime.now().isoformat(),
            'values': results,
        }
        path = os.path.join(CALIBRATION_DIR, 'tension_scale.json')
        with open(path, 'w') as f:
            json.dump(output, f, indent=2)
        print(f"\n결과 저장: {path}")
        self._print_config_update('LOAD_TO_TENSION_SCALE', results)

    # ================================================================
    # 3. 모멘트 암 캘리브레이션 (각도 의존 비선형 함수)
    # ================================================================
    def calibrate_moment_arm(self):
        """
        절차:
        1) 모터를 여러 위치로 이동시키면서 (position, load)를 수집
        2) 동시에 각 위치에서의 실제 관절 각도를 외부 측정 (각도기/비전)
        3) r(θ) = (load * tension_scale) / (k * θ_joint) 로 계산
        4) r(θ)를 다항식으로 피팅하여 비선형 함수 생성

        외부 각도 측정이 불가하면 모터 위치 기반 근사를 사용한다.
        """
        print("\n" + "=" * 60)
        print("  모멘트 암 r(θ) 캘리브레이션")
        print("=" * 60)
        print("준비사항:")
        print("  - spring_k, tension_scale 캘리브레이션이 완료되어야 합니다")
        print("  - 각도기 또는 비전 기반 각도 측정 도구 권장")
        print("  - motor_state_pub 노드가 실행 중이어야 합니다")
        print()

        # 이전 캘리브레이션 결과 로드
        k_data = self._load_calibration('spring_k.json')
        t_data = self._load_calibration('tension_scale.json')

        if not self.wait_for_data():
            return

        results = {}
        targets = THUMB_TENDON_JOINTS + [
            f'{f}_pip_dip' for f in FINGER_NAMES
        ]

        for motor_name in targets:
            if motor_name not in self.hand_motors:
                continue

            k = k_data.get(motor_name, 5.0)
            t_scale = t_data.get(motor_name, 0.01)

            print(f"\n--- {motor_name} (k={k:.3f}, t_scale={t_scale:.6f}) ---")
            print("모터를 다양한 위치로 이동시키며 각 위치에서:")
            print("  1) 실제 관절 각도(degree)를 입력하고 Enter")
            print("  2) 'q'로 종료")

            data_points = []

            while True:
                angle_str = input(f"  실제 관절 각도 (deg, 또는 q): ")
                if angle_str.strip().lower() == 'q':
                    break
                try:
                    angle_deg = float(angle_str)
                except ValueError:
                    print("  숫자를 입력하세요.")
                    continue

                sample = self.collect_sample()
                motor_data = sample.get(motor_name, {})
                load = motor_data.get('load', 0.0)
                pos = motor_data.get('position', 0.0)
                theta = math.radians(angle_deg)

                # r = T / (k * θ) where T = load * t_scale
                tension = abs(load) * t_scale
                if abs(theta) > 0.01 and k > 0:
                    r = tension / (k * abs(theta))
                else:
                    r = 0.0

                data_points.append({
                    'angle_deg': angle_deg,
                    'angle_rad': theta,
                    'motor_pos': pos,
                    'load': load,
                    'tension': tension,
                    'r_estimated': r,
                })
                print(f"    θ={angle_deg:.1f}°, load={load:.3f}, T={tension:.4f}N, r={r:.3f}mm")

            if len(data_points) >= 3:
                # 다항식 피팅: r(θ) = a0 + a1*θ + a2*θ²
                thetas = np.array([d['angle_rad'] for d in data_points])
                rs = np.array([d['r_estimated'] for d in data_points])

                # 유효한 점만 사용
                valid = rs > 0.01
                if valid.sum() >= 3:
                    coeffs = np.polyfit(thetas[valid], rs[valid], min(2, valid.sum() - 1))
                    results[motor_name] = {
                        'poly_coefficients': coeffs.tolist(),
                        'data_points': data_points,
                        'description': 'r(theta) = coeffs[0]*theta^n + ... + coeffs[-1]',
                    }
                    print(f"  → 피팅 계수: {coeffs}")
                    print(f"  → r(0) = {np.polyval(coeffs, 0):.3f}mm")
                else:
                    print(f"  → 유효 데이터 부족 (r > 0인 점 3개 이상 필요)")
                    results[motor_name] = {
                        'poly_coefficients': [],
                        'data_points': data_points,
                    }
            else:
                print(f"  → 데이터 부족 (최소 3개 필요)")

        output = {
            'type': 'moment_arm',
            'timestamp': datetime.now().isoformat(),
            'values': {k: v for k, v in results.items()},
        }
        path = os.path.join(CALIBRATION_DIR, 'moment_arm.json')
        with open(path, 'w') as f:
            json.dump(output, f, indent=2, default=str)
        print(f"\n결과 저장: {path}")
        print("\n사용법: joint_state_estimator가 이 파일을 자동 로드하여")
        print("r(θ) 비선형 함수를 적용합니다.")

    # ================================================================
    # 유틸리티
    # ================================================================
    def _load_calibration(self, filename):
        path = os.path.join(CALIBRATION_DIR, filename)
        if os.path.exists(path):
            with open(path) as f:
                data = json.load(f)
            print(f"  캘리브레이션 로드: {path}")
            return data.get('values', {})
        else:
            print(f"  캘리브레이션 파일 없음: {path} — 기본값 사용")
            return {}

    def _print_config_update(self, var_name, values):
        if not values:
            return
        print(f"\n config.py {var_name} 업데이트 값:")
        print(f"  {var_name} = {{")
        for k, v in values.items():
            print(f"      '{k}': {v:.6f},")
        print(f"  }}")


def main():
    parser = argparse.ArgumentParser(description='HopeJr 텐돈-스프링 캘리브레이션')
    parser.add_argument(
        '--mode',
        choices=['spring_k', 'tension_scale', 'moment_arm', 'full'],
        required=True,
        help='캘리브레이션 모드',
    )
    args = parser.parse_args()

    rclpy.init()
    node = CalibrationNode()

    try:
        if args.mode == 'spring_k':
            node.calibrate_spring_k()
        elif args.mode == 'tension_scale':
            node.calibrate_tension_scale()
        elif args.mode == 'moment_arm':
            node.calibrate_moment_arm()
        elif args.mode == 'full':
            print("=== Full Calibration: spring_k → tension_scale → moment_arm ===\n")
            node.calibrate_spring_k()
            node.calibrate_tension_scale()
            node.calibrate_moment_arm()
    except KeyboardInterrupt:
        print("\n캘리브레이션 중단")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
