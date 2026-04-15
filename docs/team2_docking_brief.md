# Team 2 — Chief 도킹 미션

## 목표
Chief 위성(intel_sat_dummy)까지 **5 km → 1 m 이내** 도킹한다. 마지막 구간은 카메라 기반 VBN 으로 상대 위치를 보정한다.

## 시작 조건

| 항목 | 값 |
|---|---|
| Deputy | `deputy_docking` (nasa_satellite2) |
| 초기 LVLH 위치 | $(0, -5000, 0)$ m (V-bar - 5 km) |
| 초기 속도 | $(0, 0, 0)$ |
| 연료/추력기 | 6 방향 (각 1 N) |
| Chief 정보 | `/chief/eci_state` (TLE, 노이즈 포함) |

## 미션 단계

1. **원거리 접근 (5 km → 1 km)**
   - TLE 기반 chief ECI → 상대 LVLH 변환
   - Deputy GPS + TLE → 상대 벡터 계산 (km 수준 오차)
   - +y 방향 추력으로 V-bar 따라 chief 쪽 접근
   - 카메라로 chief 시야 확인 (조명/시야각 고려)

2. **근접 접근 (1 km → 100 m)**
   - CW 제어 (2-impulse 또는 glide-slope)
   - 접근 속도 제한: 0.5 m/s 이하 (안전)
   - 카메라에서 chief 명확히 보임 → VBN 시작

3. **VBN 종단 접근 (100 m → 10 m)**
   - 카메라 중심에 chief 맞추기 (자세 제어)
   - 속도 0.05~0.1 m/s 로 감속
   - Keep-out sphere (10 m) 진입 시 GN&C 엄격

4. **최종 도킹 (10 m → <1 m)**
   - 속도 < 0.02 m/s (접촉 속도)
   - 측면 드리프트 최소화
   - Chief 중심 정렬 유지

## 팀 내 역할 분담 제안 (3명)

| 역할 | 담당 센서 / 토픽 |
|---|---|
| 궤도/제어 | `/deputy_docking/gps/odometry`, `/chief/eci_state`, thruster 명령 |
| 자세 | `/deputy_docking/imu/data`, `/deputy_docking/star_tracker/attitude` |
| VBN | `/nasa_satellite2/camera` (gz 토픽), 이미지 분석 |

## 사용 가능 스타터 코드

```bash
# 추력기 (예: -y 방향 가속으로 chief 쪽 접근)
ros2 run gz_cw_dynamics thruster_commander.py \
    --deputy deputy_docking --axis fy_plus --throttle 0.5 --duration 5.0

# 센서 상태
ros2 run gz_cw_dynamics sensor_monitor.py --deputy deputy_docking

# 카메라
ros2 run gz_cw_dynamics camera_saver.py --deputy deputy_docking --out /tmp/team2

# Reaction wheel (자세 정렬)
ros2 run gz_cw_dynamics rw_commander.py \
    --deputy deputy_docking --axis z --torque 0.001 --duration 3.0
```

## 성공 판정

- **Good** (완료): chief 중심에서 $|r|$ < 1 m, 상대속도 < 0.1 m/s
- **Great**: $|r|$ < 0.5 m, 상대속도 < 0.05 m/s, 충돌 없음
- **Perfect**: chief 도킹 포트 방향 정렬까지

## 힌트

- TLE 오차로 1 km 이내부터는 **chief 위치 추정이 부정확**. 카메라 VBN 필수.
- 접근 속도 중요: 2 m/s 로 5 km 부터 오면 2500초 = 41분. 초기 0.5~1 m/s.
- V-bar 접근은 chief 의 orbit 이 "감속" 효과. 완전히 멈추려면 추가 ΔV 필요.
- 카메라 원거리에서는 chief 가 점처럼 작음. 자세 정렬로 FOV 안에 넣기.

## 자주 쓰는 확인 명령

```bash
# Chief 과 deputy 의 상대 거리 (대략)
python3 -c "
import subprocess, re
def pos(t):
    o = subprocess.check_output(['ros2','topic','echo',t,'--once'], timeout=3).decode()
    m = re.search(r'position:\s*x:\s*([\d\.\-eE]+)\s*y:\s*([\d\.\-eE]+)\s*z:\s*([\d\.\-eE]+)', o)
    return [float(x) for x in m.groups()]
tle = pos('/chief/eci_state')
gps = pos('/deputy_docking/gps/odometry')
d = [tle[i]-gps[i] for i in range(3)]
print(f'relative (GPS-TLE frame): {d}, |d|={(sum(x*x for x in d))**0.5:.1f} m')
"
```
