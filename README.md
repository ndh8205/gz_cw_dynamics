# gz_cw_dynamics

Gazebo Harmonic (gz-sim 8) + ROS 2 Jazzy 기반 **위성 상대운동 시뮬레이션 스택**.
Clohessy–Wiltshire 동역학, 추력기, 궤도 인식 IMU, 별센서, GPS, SGP4 기반 TLE
chief 전파기까지 한 번에 제공합니다. ControLA 연구실 세미나의 SSA 미션 실습
인프라로 사용됩니다.

- **선행 패키지**: [Controla_ROS2_lec (orbit_sim)](https://github.com/ndh8205/Controla_ROS2_lec) — 위성 메시 모델 제공자
- **환경**: WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (DART)

---

## 핵심 개념

**Gazebo 월드 좌표계 = chief 의 LVLH (Hill) 프레임**.
- $\hat x$: radial outward
- $\hat y$: along-track
- $\hat z$: orbit normal

Deputy 의 상대운동은 CW pseudo-force 를 deputy link 에 매 step 인가하는
방식으로 DART 가 자동 적분. Chief 는 LVLH 원점 고정, ECI 상태는 별도
ROS 2 노드가 SGP4 로 전파해 토픽으로 발행.

---

## 제공 플러그인 (6종) + 노드 (1종)

| 이름 | 역할 |
|---|---|
| `gz_cw_dynamics-system` | CW pseudo-force (deputy 에 부착, J2 토글 지원) |
| `gz_thruster_ros2-system` | ROS 2 `std_msgs/Float32` throttle → body-frame 추력 |
| `gz_reaction_wheel_ros2-system` | ROS 2 `std_msgs/Float32` torque → 휠 joint → body 반력 토크 |
| `gz_orbit_imu-system` | LVLH 인식 IMU (gyro 에 $\omega_{LVLH/I}=(0,0,n)$ 포함, CW pseudo-accel 차감, Gaussian 노이즈 + RW bias) |
| `gz_star_tracker-system` | $q_{body/ECI}$ 쿼터니언 + 소각 Gaussian 노이즈 |
| `gz_gps-system` | Deputy ECI 위치/속도 + Gaussian 노이즈 |
| `gz_chief_propagator-system` | (레거시, 비활성) |
| `chief_propagator_node.py` | **현재 사용** — Keplerian truth + SGP4-based TLE 추정 + sun vector 발행 |

## 제공 월드 (3종)

| 월드 | 용도 |
|---|---|
| `worlds/mission.sdf` | **세미나 미션**: chief + deputy_formation (팀1) + deputy_docking (팀2), 각 5 km 떨어진 시작 |
| `worlds/gco_test.sdf` | 단일 deputy 50 m GCO 동역학 검증 + 시각화 |
| `worlds/gco_verify.sdf` | 헤드리스 수치 검증 (카메라 없음, RTF=100) |
| `worlds/thruster_test.sdf` | 6-추력기 단위 테스트 |

## 검증 스크립트 (5종)

```bash
bash ~/space_ros_ws/install/gz_cw_dynamics/lib/gz_cw_dynamics/run_gco_verify.sh        # CW 3-orbit 수치 검증
bash ~/space_ros_ws/install/gz_cw_dynamics/lib/gz_cw_dynamics/run_thruster_test.sh     # 6 추력기 방향/크기
bash ~/space_ros_ws/install/gz_cw_dynamics/lib/gz_cw_dynamics/run_imu_test.sh          # IMU truth + LVLH 회전
bash ~/space_ros_ws/install/gz_cw_dynamics/lib/gz_cw_dynamics/run_chief_test.sh        # Chief ECI propagation
bash ~/space_ros_ws/install/gz_cw_dynamics/lib/gz_cw_dynamics/run_all_sensors_test.sh  # 종합 7 항목 (ST rotated 포함)
```

## 학생 스타터 코드 (3종)

```bash
ros2 run gz_cw_dynamics thruster_commander.py --deputy deputy_docking --axis fy_plus --throttle 0.5 --duration 2
ros2 run gz_cw_dynamics rw_commander.py       --deputy deputy_docking --axis z --torque 0.001 --duration 3
ros2 run gz_cw_dynamics sensor_monitor.py     --deputy deputy_formation
ros2 run gz_cw_dynamics camera_saver.py       --deputy deputy_docking --out /tmp/frames
```

---

## 설치 & 빌드

```bash
mkdir -p ~/space_ros_ws/src && cd ~/space_ros_ws/src
git clone https://github.com/ndh8205/Controla_ROS2_lec.git orbit_sim
git clone <이_저장소_URL> gz_cw_dynamics

sudo apt install -y python3-sgp4 python3-gz-transport13 python3-gz-msgs10 \
    ros-jazzy-ros-gz ros-jazzy-ros-gz-image ros-jazzy-ros-gz-bridge

cd ~/space_ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select orbit_sim gz_cw_dynamics
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/space_ros_ws/install/orbit_sim/share/orbit_sim/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/space_ros_ws/install/gz_cw_dynamics/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

---

## 미션 실행 (세미나 Part 2)

```bash
ros2 launch gz_cw_dynamics mission.launch.py
```

GUI 가 뜨고 3초 후 `chief_propagator_node` 가 자동 시작.

### 토픽 구성

**Chief 공유 (모든 팀)**
- `/chief/eci_state` (nav_msgs/Odometry) — TLE 기반 추정 (노이즈 포함, 학생이 사용)
- `/chief/eci_truth` (nav_msgs/Odometry) — 진실 (센서 플러그인 내부 전용)
- `/chief/sun_vector_lvlh` (geometry_msgs/Vector3Stamped)

**팀 1: Formation**
- `/deputy_formation/cw_pseudo_accel` (gz Vector3d)
- `/deputy_formation/thruster/{fx,fy,fz}_{plus,minus}/cmd` (std_msgs/Float32)
- `/deputy_formation/imu/data` (sensor_msgs/Imu)
- `/deputy_formation/star_tracker/attitude` (geometry_msgs/QuaternionStamped, frame_id=eci)
- `/deputy_formation/gps/odometry` (nav_msgs/Odometry, frame_id=eci)
- `/nasa_satellite/camera` (gz Image) — 내장 mesh 모델 카메라

**팀 2: Docking**
- 동일 구조, `/deputy_docking/...` 와 `/nasa_satellite2/camera`

---

## 프레임 / 단위 컨벤션

| 항목 | 값 |
|---|---|
| Chief 궤도 | 545 km SSO, $a = 6923.137$ km, $n = 1.0959\text{e-}3$ rad/s, $T = 95.55$ min |
| Chief 모델 | intel_sat_dummy (질량 1 kg) |
| Deputy 질량 | nasa_satellite 2.6 kg, nasa_satellite2 1 kg |
| 모든 길이 | m (SI) |
| 모든 속도 | m/s |
| 모든 각속도 | rad/s |
| Gyro noise | $\sigma_{\omega}$ = 1.745e-5 rad/s |
| Accel noise | $\sigma_a$ = 1e-3 m/s² |
| Bias RW (gyro/accel) | MATLAB Space_SLAM 모델과 동일 |
| Star tracker noise | 0.05° (1-sigma, 소각 Gaussian) |
| GPS position noise | 5 m (1-sigma) |
| GPS velocity noise | 0.05 m/s (1-sigma) |
| TLE position noise | 100 m (1-sigma, 3-sigma = 300 m) |
| TLE velocity noise | 0.1 m/s (1-sigma) |

---

## 이론 참고

**Clohessy-Wiltshire 방정식** (Hill 프레임, 원형 reference orbit):

$$\ddot x - 2n\dot y - 3n^2 x = a_x$$
$$\ddot y + 2n\dot x = a_y$$
$$\ddot z + n^2 z = a_z$$

**LVLH 회전 속도** (관성계 대비): $\vec\omega_{LVLH/I} = (0, 0, n)$

→ 본체가 LVLH 와 정렬된 Deputy 의 자이로는 $(0, 0, n)$ 출력해야 함 → `OrbitImu` 플러그인이 이를 자동 처리.

**Schweighart-Sedwick J2 보정** (옵션, `<enable_j2>true</enable_j2>`):

$$s = \tfrac{3 J_2}{8}(R_e/a)^2 (1 + 3\cos 2i), \quad c = \sqrt{1+s}$$

→ 수정된 stiffness $(5c^2-2)n^2$, 수정된 Coriolis $2(nc)$.

---

## 세미나 가이드

- [docs/seminar_setup.md](docs/seminar_setup.md) — 학생 설치 및 실행 가이드
- [docs/team1_formation_brief.md](docs/team1_formation_brief.md) — 팀 1 (50 m GCO + 사진)
- [docs/team2_docking_brief.md](docs/team2_docking_brief.md) — 팀 2 (5 km → 도킹)

---

## 라이선스

Apache-2.0
