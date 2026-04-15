# [Draft for Controla_ROS2_lec/README.md]

> 이 파일은 Controla_ROS2_lec 저장소의 README.md 를 세미나에 맞게 업데이트할 수 있는 **초안**입니다.
> 실제 github 에 반영할 때는 이 파일 내용을 복사해서 Controla_ROS2_lec/README.md 로 커밋하세요.

---

# orbit_sim — ControLA ROS 2 우주 시뮬레이션 실습 패키지

ROS 2 Jazzy + Gazebo Harmonic 기반 세미나 실습 패키지.
4시간 세미나에서 ROS 2 / Gazebo 기초를 익히고, 실제 **SSA 미션(5 km 접근)** 시뮬레이션까지 진행합니다.

**환경:** WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (DART), NVIDIA GPU (D3D12)

---

## 세미나 구성 (총 4시간)

| 시간 | 파트 | 내용 |
|---|---|---|
| 0:00 – 1:00 | Part 1 — ROS 2 기초 | Node, Topic, pub/sub, CLI, launch 파일 |
| 1:00 – 2:00 | Part 1 — Gazebo 기초 | SDF, world/model, ros_gz_bridge, `orbit_sim` 위성 모델 |
| 2:00 – 4:00 | **Part 2 — 팀 미션** | [gz_cw_dynamics](https://github.com/<TBD>/gz_cw_dynamics) 스택 기반 SSA 미션 |

## 팀 미션 개요 (Part 2, 2시간)

- **인프라**: 메인 데스크탑에서 Gazebo 실행, 모든 학생 노트북이 ROS 2 DDS 로 연결됨
- **환경**: 545 km SSO chief + 두 deputy, 각 5 km 거리에서 시작
- **역할 분담** (팀 내 자세 / 궤도 / 영상 담당)

| 팀 | 인원 | 미션 | Deputy 명 |
|---|---|---|---|
| 1 | 4 | 50 m GCO 포메이션 + chief 사진 확보 | `deputy_formation` |
| 2 | 3 | 5 km → 도킹 (VBN 활용) | `deputy_docking` |

팀별 상세 브리프:
- Team 1 brief → `gz_cw_dynamics/docs/team1_formation_brief.md`
- Team 2 brief → `gz_cw_dynamics/docs/team2_docking_brief.md`

---

## 설치 (학생 노트북)

### 1. 선행 패키지
```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-bridge \
    python3-sgp4 \
    python3-gz-transport13 \
    python3-gz-msgs10
```

### 2. 워크스페이스
```bash
mkdir -p ~/space_ros_ws/src && cd ~/space_ros_ws/src

git clone https://github.com/ndh8205/Controla_ROS2_lec.git orbit_sim
git clone <gz_cw_dynamics_github_url>           # 강사가 공지

cd ~/space_ros_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select orbit_sim gz_cw_dynamics
source install/setup.bash
```

### 3. 환경변수 (~/.bashrc)
```bash
source /opt/ros/jazzy/setup.bash
source ~/space_ros_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/space_ros_ws/install/orbit_sim/share/orbit_sim/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/space_ros_ws/install/gz_cw_dynamics/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export ROS_DOMAIN_ID=7        # 강사가 공지하는 값
```

---

## Part 1 실습 (ROS 2 + Gazebo 기초)

### 간단한 위성 시뮬레이션 구동
```bash
# 궤도 환경 (기존 gco_test.world)
ros2 launch orbit_sim gco_test.launch.py
```

### 토픽 확인
```bash
ros2 topic list
ros2 topic echo /nasa_satellite5/imu
rqt_graph
```

### 카메라 뷰어
```bash
ros2 run rqt_image_view rqt_image_view
```

---

## Part 2 실습 (SSA 미션)

**메인 데스크탑** 에서:
```bash
bash ~/kill_sim.sh
ros2 launch gz_cw_dynamics mission.launch.py
```

**학생 노트북** 에서 역할에 맞게:
```bash
# 센서 모니터링 (자세/궤도 담당 공용)
ros2 run gz_cw_dynamics sensor_monitor.py --deputy deputy_formation
ros2 run gz_cw_dynamics sensor_monitor.py --deputy deputy_docking

# 추력기 명령
ros2 run gz_cw_dynamics thruster_commander.py \
    --deputy deputy_docking --axis fy_plus --throttle 0.5 --duration 2.0

# 카메라 프레임 저장
ros2 run gz_cw_dynamics camera_saver.py --deputy deputy_formation --out /tmp/team1
```

상세 토픽 목록 / 센서 노이즈 파라미터 / 궤도 제원 →
`gz_cw_dynamics/README.md` 및 `gz_cw_dynamics/docs/seminar_setup.md` 참조.

---

## 제공 모델 (orbit_sim/models/)

### 위성 / 우주
- `intel_sat_dummy` — 미션 월드의 chief
- `nasa_satellite`, `nasa_satellite2~6` — deputy 로 사용
- `iss`, `capsule`, `earth`, `saturn`

### 지원 자원
- `canadarm`, `kari_arm`, `kari_dual_arm` — 도킹 암 (향후 확장)
- `aruco_marker`, `solar_panel`, `truss`

전체 목록 → `models/` 디렉토리 확인

---

## 트러블슈팅

| 증상 | 해결 |
|---|---|
| 학생 노트북에서 토픽 안 보임 | `ROS_DOMAIN_ID` 일치, 동일 LAN 확인 |
| GUI 검은 화면/프리즈 | WSL: `wsl --shutdown` 후 재접속 |
| `colcon build` 실패 | `python3-sgp4` 설치 확인, `source /opt/ros/jazzy/setup.bash` |
| 모델 로딩 실패 | `GZ_SIM_RESOURCE_PATH` 에 models 경로 포함 |
| 플러그인 로딩 실패 | `GZ_SIM_SYSTEM_PLUGIN_PATH` 에 gz_cw_dynamics/lib 포함 |
| 프로세스 잔존 | `bash ~/kill_sim.sh` |

---

## 관련 저장소

| 저장소 | 역할 |
|---|---|
| [Controla_ROS2_lec](https://github.com/ndh8205/Controla_ROS2_lec) (이 저장소) | ROS 2 + Gazebo 입문 + 위성 모델 자원 |
| [gz_cw_dynamics](https://github.com/<TBD>/gz_cw_dynamics) | SSA 미션용 CW 동역학 + 센서 플러그인 |

*ControLA 세미나 실습용 — ROS 2 + Gazebo 기초부터 SSA 미션까지*
