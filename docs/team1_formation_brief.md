# Team 1 — GCO Formation 사진 확보 미션

## 목표
Chief 위성(intel_sat_dummy) 주변 **50 m GCO 궤도**로 진입하여, 서로 다른 각도에서 chief 사진을 **최소 6장** 확보한다.

## 시작 조건

| 항목 | 값 |
|---|---|
| Deputy | `deputy_formation` (nasa_satellite) |
| 초기 LVLH 위치 | $(0, +5000, 0)$ m (V-bar + 5 km) |
| 초기 속도 | $(0, 0, 0)$ |
| 연료/추력기 | 6 방향 (각 1 N), 스로틀 [0,1] |
| Chief 정보 | `/chief/eci_state` (TLE, 노이즈 포함) |

## 미션 단계 (권장)

1. **Chief 탐색 (5 km → 1 km)**
   - TLE 기반 chief ECI 위치 추정 (`/chief/eci_state`)
   - Deputy GPS (`/deputy_formation/gps/odometry`) 와 비교해 상대 벡터 계산
   - 카메라 시야에 chief 들어오도록 자세 정렬 (자세 담당)
   - V-bar 따라 접근 (궤도 담당)

2. **GCO 진입 (1 km → 50 m)**
   - CW 2-impulse 또는 glide-slope 로 접근
   - 최종 상태: 50 m GCO 초기조건
     - 위치 $(0, 50, 0)$ m
     - 속도 $(0.0274, 0, 0.0475)$ m/s

3. **GCO 궤도 유지 + 촬영**
   - Deputy 가 chief 주위 3D 원궤도 (50 m 반경) 로 자연히 돈다
   - 한 주기 = 95.5 분 (실시간) / RTF 로 단축 가능
   - 서로 다른 phase 에서 카메라로 chief 캡처

## 팀 내 역할 분담 제안

| 역할 | 담당 센서 / 토픽 |
|---|---|
| 궤도 담당 | `/deputy_formation/gps/odometry`, `/chief/eci_state`, thruster 명령 |
| 자세 담당 | `/deputy_formation/imu/data`, `/deputy_formation/star_tracker/attitude`, `/chief/sun_vector_lvlh` |
| 영상 담당 | `/nasa_satellite/camera` (gz 토픽) |
| 미션 매니저 | 전체 상태 모니터링, 브리핑 |

## 사용 가능 스타터 코드

```bash
# 추력기 점화 (0.5 N, 2초)
ros2 run gz_cw_dynamics thruster_commander.py \
    --deputy deputy_formation --axis fy_minus --throttle 0.5 --duration 2.0

# 센서 상태 모니터링
ros2 run gz_cw_dynamics sensor_monitor.py --deputy deputy_formation

# 카메라 프레임 저장
ros2 run gz_cw_dynamics camera_saver.py --deputy deputy_formation --out /tmp/team1

# Reaction wheel 토크 (자세 제어, 예: +z 축으로 1 mN·m 3초)
ros2 run gz_cw_dynamics rw_commander.py \
    --deputy deputy_formation --axis z --torque 0.001 --duration 3.0
```

## 성공 판정

- 카메라로 chief 가 식별 가능한 사진 **6장 이상** (서로 다른 phase)
- 최종 $|r|$ = 50 m ± 10 m 궤도에서 최소 1 orbit (실시간 95분, RTF 100 기준 57초) 안정 유지

## 힌트

- 처음 5 km 접근은 대기 길다. RTF 를 올려서 빨리 감기 → 근접 후 RTF=1 로 낮춰 정밀 제어.
- V-bar (along-track) 접근은 CW 에서 "자연 감속" 특성. 적절한 $-y$ 방향 추력으로 닫아감.
- GCO 초기조건은 정확히 넣어야 닫힌 원궤도. 오차 있으면 drift → 몇 orbit 후 궤도 무너짐.
- Chief 의 TLE 위치는 km 수준 오차. 근접 시 GPS + 카메라 로 보정.

## 자주 쓰는 확인 명령

```bash
# 현재 Deputy 위치 (LVLH)
gz topic -e -t /world/mission/pose/info | grep -B1 -A4 deputy_formation

# 실시간 조정
gz service -s /world/mission/set_physics \
    --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 1000 \
    --req 'max_step_size: 0.01, real_time_factor: 30.0'
```
