ARM_MOTORS_LIMITS = {
    'shoulder_pitch': (180, -180),
    'shoulder_yaw': (96.31420536149139, -97.97578296737078),
    'shoulder_roll': (67.15065358933248, -67.15065358933248),
    'elbow_flex': (-55.00394833255903, 80.78704911344609),
    'wrist_roll': (-45, 210),
    'wrist_yaw': (-10, 10),
    'wrist_pitch': (-75, 80),
}

HAND_MOTORS_LIMITS = {
    'thumb_cmc': (-3, -0.043),
    'thumb_mcp': (0, 0.88),
    'thumb_pip': (0, 0.88),
    'thumb_dip': (0.88, 0),
    'index_radial_flexor': (0, 1.6562007649872527),
    'index_ulnar_flexor': (-1.8, 0),
    'index_pip_dip': (-0.02, 2.22),
    'middle_radial_flexor': (0, 1.6562007649872527),
    'middle_ulnar_flexor': (-1.8, 0),
    'middle_pip_dip': (-0.02, 2.22),
    'ring_radial_flexor': (0, 1.6562007649872527),
    'ring_ulnar_flexor': (-1.8, 0),
    'ring_pip_dip': (-0.02, 2.22),
    'pinky_radial_flexor': (0, 1.6562007649872527),
    'pinky_ulnar_flexor': (-1.8, 0),
    'pinky_pip_dip': (2.22, -0.02),
}

# ============================================================
# 텐돈-스프링 메커니즘 파라미터
# ============================================================

# 스프링 강성 계수 k (N·mm/rad)
# 캘리브레이션 전 초기값 — calibrate_spring_params()로 업데이트
SPRING_CONSTANTS = {
    # 엄지: mcp, pip, dip 각각 독립 스프링
    'thumb_mcp': 5.0,
    'thumb_pip': 5.0,
    'thumb_dip': 5.0,
    # 검지~새끼: pip, dip 각각 독립 스프링 (텐돈은 공유)
    'index_pip': 5.0,
    'index_dip': 5.0,
    'middle_pip': 5.0,
    'middle_dip': 5.0,
    'ring_pip': 5.0,
    'ring_dip': 5.0,
    'pinky_pip': 5.0,
    'pinky_dip': 5.0,
}

# 텐돈 모멘트 암 r (mm)
# 실제로는 관절 각도에 따라 변하는 비선형 함수이지만,
# 캘리브레이션 전에는 상수로 근사한다.
# calibrate_moment_arms()로 데이터 기반 함수로 교체 가능.
MOMENT_ARMS = {
    'thumb_mcp': 4.5,
    'thumb_pip': 4.5,
    'thumb_dip': 4.5,
    'index_pip': 2.25,
    'index_dip': 2.25,
    'middle_pip': 2.25,
    'middle_dip': 2.25,
    'ring_pip': 2.25,
    'ring_dip': 2.25,
    'pinky_pip': 2.25,
    'pinky_dip': 2.25,
}

# 모터 부하(load) → 텐돈 장력(N) 변환 계수
# Feetech 서보의 load 값은 0~1000 (무차원) → 실제 장력으로 스케일링 필요
# 캘리브레이션으로 결정: T(N) = LOAD_TO_TENSION_SCALE * load_raw
LOAD_TO_TENSION_SCALE = {
    'thumb_mcp': 0.01,
    'thumb_pip': 0.01,
    'thumb_dip': 0.01,
    'index_pip_dip': 0.01,
    'middle_pip_dip': 0.01,
    'ring_pip_dip': 0.01,
    'pinky_pip_dip': 0.01,
}

# 텐돈 길이 구속: 모터 위치(normalized) → 텐돈 당김 길이(mm)
# θ_pip + θ_dip 를 구속하는 데 사용
TENDON_MM_PER_MOTOR_UNIT = {
    'thumb_mcp': 4.5,
    'thumb_pip': 4.5,
    'thumb_dip': 4.5,
    'index_pip_dip': 2.25,
    'middle_pip_dip': 2.25,
    'ring_pip_dip': 2.25,
    'pinky_pip_dip': 2.25,
}