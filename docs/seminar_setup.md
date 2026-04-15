# 세미나 설정 가이드 (학생용)

## 1. 선행 설치 (개인 노트북)

```bash
# Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic 이 이미 설치되어 있다고 가정.
sudo apt update
sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-bridge \
    python3-sgp4 \
    python3-gz-transport13 \
    python3-gz-msgs10
```

## 2. 워크스페이스 구축

```bash
mkdir -p ~/space_ros_ws/src && cd ~/space_ros_ws/src

# 두 저장소 clone
git clone https://github.com/ndh8205/Controla_ROS2_lec.git orbit_sim
git clone <gz_cw_dynamics GitHub URL> gz_cw_dynamics

cd ~/space_ros_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 3. 환경 변수 (~/.bashrc 에 추가)

```bash
source /opt/ros/jazzy/setup.bash
source ~/space_ros_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/space_ros_ws/install/orbit_sim/share/orbit_sim/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/space_ros_ws/install/gz_cw_dynamics/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

## 4. 네트워크 확인 (학생 ↔ 메인 데스크탑)

강사가 메인 데스크탑에서 gz sim 을 실행하면, 학생 노트북에서 토픽이 자동으로 보여야 합니다.

**모든 노트북에서 동일 설정 필요**:
```bash
export ROS_DOMAIN_ID=7          # 강사가 공지하는 ID (기본 7 가정)
```

**확인**:
```bash
ros2 topic list
```

다음 토픽들이 보이면 성공:
- `/chief/eci_state`, `/chief/eci_truth`, `/chief/sun_vector_lvlh`
- `/deputy_formation/{gps,imu,star_tracker,cw_pseudo_accel}/...`
- `/deputy_docking/{gps,imu,star_tracker,cw_pseudo_accel}/...`
- `/deputy_formation/thruster/*/cmd`
- `/deputy_docking/thruster/*/cmd`

## 5. 메인 데스크탑 실행 (강사 전용)

```bash
bash ~/kill_sim.sh     # 이전 세션 정리
ros2 launch gz_cw_dynamics mission.launch.py
```

GUI 가 뜨면서:
- 원점에 chief
- 5 km 떨어진 두 deputy (보려면 줌 아웃)
- 3초 후 chief_propagator_node 가 자동 시작

## 6. 학생 노트북 사용 명령

### 센서 모니터링
```bash
ros2 run gz_cw_dynamics sensor_monitor.py --deputy deputy_formation
ros2 run gz_cw_dynamics sensor_monitor.py --deputy deputy_docking
```

### 추력기 명령 (단발성)
```bash
# +y 방향 0.5 N 2초 발사 (팀1: chief 쪽으로 -y 방향)
ros2 run gz_cw_dynamics thruster_commander.py \
    --deputy deputy_formation --axis fy_minus --throttle 0.5 --duration 2.0
```

### 카메라 프레임 저장
```bash
ros2 run gz_cw_dynamics camera_saver.py --deputy deputy_formation --out /tmp/team1
```

### 개별 토픽 모니터
```bash
ros2 topic echo /deputy_formation/imu/data
ros2 topic echo /deputy_formation/star_tracker/attitude
ros2 topic echo /deputy_formation/gps/odometry --once
ros2 topic echo /chief/eci_state --once    # TLE noisy estimate (학생에게 주어진 chief 정보)
```

### RQT 그래프 / 뷰어
```bash
rqt_graph                                  # 노드 연결 그래프
ros2 run rqt_image_view rqt_image_view     # 카메라 GUI 뷰어 (rqt)
```

## 7. 시뮬레이션 속도 제어

```bash
# RTF 30 배로 빠르게 (far approach 구간 추천)
gz service -s /world/mission/set_physics \
    --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 1000 \
    --req 'max_step_size: 0.01, real_time_factor: 30.0'

# 실시간 (근접 구간 추천)
gz service -s /world/mission/set_physics \
    --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 1000 \
    --req 'max_step_size: 0.01, real_time_factor: 1.0'
```

## 8. 빠른 자체 검증 (학생 시작 전 선택)

```bash
# CW 동역학 50m GCO 검증 (3 orbit ~ 3분)
bash ~/space_ros_ws/install/gz_cw_dynamics/lib/gz_cw_dynamics/run_gco_verify.sh

# 6 추력기 방향/크기 검증 (~30초)
bash ~/space_ros_ws/install/gz_cw_dynamics/lib/gz_cw_dynamics/run_thruster_test.sh

# 전체 센서 7개 항목 검증 (~10초)
bash ~/space_ros_ws/install/gz_cw_dynamics/lib/gz_cw_dynamics/run_all_sensors_test.sh
```

모두 PASS 뜨면 학생 시스템 OK.

## 9. 트러블슈팅

| 증상 | 해결 |
|------|------|
| 학생 노트북에서 토픽 안 보임 | `ROS_DOMAIN_ID` 일치 확인, 같은 LAN 상 확인 |
| GUI 검은 화면 + 프리즈 | WSLg 이슈. Windows 에서 `wsl --shutdown` 후 재접속 |
| `colcon build` 실패 | `sudo apt install python3-sgp4`, `source /opt/ros/jazzy/setup.bash` |
| 모델 로딩 실패 | `GZ_SIM_RESOURCE_PATH` 에 orbit_sim/models 포함 확인 |
| 플러그인 로딩 실패 | `GZ_SIM_SYSTEM_PLUGIN_PATH` 에 gz_cw_dynamics/lib 포함 확인 |
| 시뮬레이션 프로세스 잔존 | `bash ~/kill_sim.sh` (orbit_sim 이 제공) |
| 카메라 프레임 안 저장됨 | `python3-gz-transport13` + `python3-pil` 설치 확인 |
