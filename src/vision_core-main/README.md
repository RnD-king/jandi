# 공통 비전 코어 사용 설명서

`vision_core`는 실제 카메라용 ROS2 `vision` 패키지와 G1 시뮬레이터가
같은 비전 계산과 속도 명령 계산을 사용하도록 만든 독립 C++ 라이브러리다.

별도 노드로 실행하는 프로그램이 아니다. `vision`과 G1의 프로세스 안에서
공유 라이브러리 함수가 바로 호출되므로, 코어를 분리한 것 때문에 ROS2 토픽이
추가되거나 직렬화 비용이 생기지 않는다.

## 역할 구분

### 실제 카메라용 `vision` 패키지

`/home/noh/my_cv/src/vision`에 있으며 장치와 ROS2에 관련된 일을 담당한다.

- 카메라 영상과 IMU 토픽 수신
- CUDA 전처리와 TensorRT YOLO 추론
- YOLO 검출 결과를 공통 코어 자료형으로 변환
- 화면 표시와 진단 로그
- 코어가 선택한 통합 명령을 속도·액션·카메라 ROS2 명령으로 분배
- `/jandi_vision/cmd_vel`, `/jandi_vision/action_cmd`,
  `/jandi_vision/camera_cmd` 토픽 발행
- 액션·카메라 명령의 `ACK`/`DONE` 상태 토픽 수신

### 공통 `vision_core`

`/home/noh/vision_core`에 있으며 실제 판단과 제어 계산을 담당한다.

- IMU roll/pitch를 이용한 픽셀 좌표 보정
- 점선 중심점에서 8차원 특징 계산
- 점선 추종 속도 계산
- 점선을 놓쳤을 때 최근 방향과 이동 경로 기억을 이용한 복구
- 공·골대·백보드·허들 중 사용할 검출 대상 선택
- 공·허들·골대 검출 안정화와 접근 상태 전환
- 공·허들·골대 접근 속도 및 임시 동작 요청 계산
- Line 상태에서 골대 > 허들 > 공 순서로 새 미션을 선택하고, 진입한 미션은
  해당 controller의 성공·유실 종료 전까지 중앙 코디네이터에서 잠금

### 시뮬레이터

Python 시뮬레이터는 `ctypes` 연결층을 통해 같은
`libshared_vision_core.so`를 호출한다. 따라서 특징 수식이나 규칙기반 속도
수식을 코어에서 변경하면 실제 비전과 시뮬레이터에 같은 계산을 적용할 수 있다.

Jandi MJLab 어댑터는 실제 경기장 좌표를 line/object bbox와 RGB-D 표본으로
변환해 `vision_mission_controller_step_perception_v1()`에 전달한다. 따라서
거리 검증, IMU 좌표 보정, depth pose, association tracker, 연속 검출/lost 처리와
미션 전이는 ROS와 같은 `MissionController::StepPerception()`에서 수행된다.

## 공통 알고리즘 설정

`config/vision_algorithm.yaml`이 실제 ROS 비전과 시뮬레이터가 공유하는
알고리즘 기준값의 단일 원본이다. 설치하면
`share/shared_vision_core/config/vision_algorithm.yaml`로 복사되며,
`MissionController`와 ROS `line_perception_node`가 자동으로 읽는다.
알고리즘 Config 구조체는 값 전달용이며 숫자 기본값을 중복 보관하지
않는다. 공통 YAML이 없거나 필수 키/타입이 잘못되면 0 기본값으로
계속하지 않고 시작 시 실패한다.

기존 연속속도 라인 특징 필드 구조를 유지하면서 P2P 판단용 최소 표현인
`LineGuide`를 함께 계산한다. `LineGuide`는 가까운 점군의 `offset`과
`heading`, 가까운/먼 점군 방향 차이인 signed `curvature`, 두 local fit의
품질을 합친 `confidence`만 제공한다. 변경 전 extractor는 비교용으로
`legacy/line_feature_extractor_velocity_legacy.cpp`에 보존되어 있으며 빌드에는
포함되지 않는다.

라인 pixel 특징(`u_err`, `slope`, `LineGuide`)의 signed convention은 모두
화면 오른쪽이 양수다. 로봇 명령은 ROS convention에 따라 `vy`, `wz`의 좌측이
양수이므로 line controller가 조향을 만들 때 한 번 부호를 반전한다.
`StepPerception()`은 YAML의 `image_center_u`보다 현재 frame의 camera
`intrinsics.cx`를 우선하며, 직접 `Step()`을 호출할 때만 YAML 값을 fallback으로
사용한다.

velocity backend는 기존 연속속도 계산을 그대로 사용한다. P2P backend는
`offset_gain`, `heading_gain`, `curvature_gain` 세 값만으로 LineGuide를
PRE-P2P 속도 의도로 바꾼다. 5걸음·곡선·회전 후 직진 계열 action은
ACK부터 DONE까지 관측한
특징 중 실제 실행시간 후반 50%를 끝에 가까울수록 크게 선형 가중 평균하여
다음 action을 고른다. 1걸음과 제자리회전처럼 짧은 action은 DONE 시점의
최신 프레임을 사용한다.

실행 시 우선순위는 `공통 YAML < ROS params-file < ROS CLI -p`다. 따라서
공통 값을 바꿀 때는 YAML을 수정하고 core를 다시 install하면 되며,
일회성 실험은 기존처럼 `ros2 run ... --ros-args -p 이름:=값`으로
덮어쓴다. 다른 공통 YAML을 시험하려면
두 환경 모두 다음 변수로 같은 파일을 지정한다.

```bash
export VISION_CORE_ALGORITHM_CONFIG=/절대/경로/vision_algorithm.yaml
```

ROS 토픽, 화면 출력, 시뮬레이션 피드백 같은 어댑터 전용 값은
`vision/config/vision_params.yaml`에 따로 둔다. YOLO 모델 경로는 기존
`vision/config/yolo26_runtime.yaml`의 PC/Jetson 선택 규칙을 그대로 사용한다.

`object_association` 설정은 ball, backboard, hurdle의 클래스별 tracker에
사용된다. confidence는 후보 threshold에만 사용하고, 연결 점수는
이전 중심의 영상 대각선 정규화 거리와 width/height의 대칭 log-ratio
변화만 사용한다. 검출이 비면 identity는 `missing_frame_limit` 동안 보존하지만,
이전 bbox를 현재 검출로 controller에 전달하지는 않는다. goal 클래스는
이 tracker 대상이 아니다.

## 실제 비전 노드의 처리 순서

```text
[vision] ROS2 realsense 영상, IMU 토픽 수신
    ↓
[vision] CUDA 영상 전처리
    ↓
[vision] TensorRT YOLO 추론 → 객체별 bbox 생성
    ↓
[vision] 라인 bbox → 라인 중심점 목록 생성
[vision] 미션 물체 bbox → 공·골대·백보드·허들 좌표 목록 생성
    ↓
[vision_core] 객체별 좌표 목록 중 필요한 좌표 선택
    ↓
[vision_core] IMU 기반 카메라 좌표 보정
    ↓
[vision_core] MissionController::Step 한 번 호출
    ├─ LINE: 라인 특징/명령 + 허들·공·골대 진입 판정
    ├─ BALL: BallController만 실행
    ├─ HURDLE: HurdleController만 실행
    └─ GOAL: GoalController만 실행
    ↓
[vision_core] 미션 잠금과 속도/액션/카메라를 하나의 ControlCommand로 조정
    ↓
[vision] 속도는 매 프레임, 액션/카메라는 ACK 전까지 해당 토픽으로 발행
```

## 통합 명령과 ROS2 전달 규칙

코어 내부의 `ControlCommand`는 `command_type`, `mission`, `mission_phase`,
`control_phase`, `vx/vy/wz`, `action`, `action_id`, `camera_request`를 한 번에
보관한다. ROS2 `vision` 패키지는 이를 실행 특성에 따라 다음처럼 분배한다.

```text
/jandi_vision/cmd_vel        geometry_msgs/Twist       매 추론 프레임 발행
/jandi_vision/action_cmd     vision/ActionCommand      ACK 전까지 동일 ID 재발행
/jandi_vision/action_status  vision/CommandStatus      ACK/DONE 수신
/jandi_vision/camera_cmd     vision/CameraCommand      ACK 전까지 동일 ID 재발행
/jandi_vision/camera_status  vision/CommandStatus      ACK/DONE 수신
```

액션 실행 중에는 속도 토픽에 `0,0,0`을 계속 발행한다. 접근 보행에서 액션으로
바뀔 때는 즉시 자세 제어권을 넘기지 않고 `RL_STOPPING` 동안
`command_type=VELOCITY`, `vx=vy=wz=0`을 유지한다. 기본 정지 시간은 1.5초이며
ROS 파라미터 `rl_stop_duration_sec`로 바꾼다. 이후 ACTION을 같은 `action_id`로
ACK까지 반복하고, ACK 뒤에는 DONE까지 HOLD한다. DONE을 받은 다음 프레임부터
해당 미션 controller가 다음 판단을 이어간다.

기존 직접 controller 호출자는 호환을 위해 설정 시간 기반 placeholder를 계속
사용할 수 있다. `MissionController` 호출자는 `command_transport_enabled=true`로
실행기 사용을 알리고 `CommandDeliveryFeedback`의 동일 `action_id` ACK/DONE만
반환한다. controller용 완료 신호는 MissionController가 내부에서 라우팅한다.

`MissionController`가 전체 미션의 단일 진입점이다. ROS 노드는 각 객체
controller를 직접 호출하거나 `StartAfterPickup()`, `Reset()`, 우선순위 판단을
수행하지 않는다. 보정된 line 중심점, 객체 target, 카메라 상태와 ACK/DONE만
전달하고 반환된 `ControlCommand`를 토픽으로 변환한다.

LINE에서는 라인 명령을 계속 계산하면서 진입 안정화만 수행한다. 물체 미션이
선택되면 해당 controller만 실행하며 다른 controller의 검출 이력과 후보 명령은
계산하지 않는다. Ball의 `POST_PICKUP_LINE_RECOVERY`와 Goal의
`HEADING_RECOVERY`처럼 라인이 필요한 단계에서만 line 특징과 명령을 다시 계산한다.

## `line_detection_adapter`가 라인에만 있는 이유

YOLO는 라인도 사각형 bbox로 검출한다. 점선 특징 계산에는 사각형 자체보다
여러 점의 중심 좌표가 필요하기 때문에 `line_detection_adapter`가 다음 변환을
담당한다.

```text
라인 bbox 여러 개 → std::vector<cv::Point2f> 중심점 목록
```

Python 딕셔너리가 아니라 C++ `std::vector` 목록이다.

공·골대·백보드·허들은 별도의 어댑터 파일이 없다.
`line_perception_node.cpp`의 `ToCoreDetections()`가 YOLO의 모든 bbox를
`std::vector<vision_core::Detection>`으로 한꺼번에 변환하고,
`vision_core::ExtractObjectTargets()`가 class별 대상을 선택한다.

## 파일별 역할

```text
vision_core/
├── include/vision_core/
│   ├── types.hpp
│   ├── coordinate_rectifier.hpp
│   ├── line_feature_extractor.hpp
│   ├── line_velocity_controller.hpp
│   ├── object_target_extractor.hpp
│   ├── ball_controller.hpp
│   ├── hurdle_controller.hpp
│   ├── goal_controller.hpp
│   ├── motion_command_selector.hpp
│   ├── p2p_motion_quantizer.hpp
│   ├── mission_controller.hpp
│   └── c_api.h
├── src/
│   ├── coordinate_rectifier.cpp
│   ├── line_feature_extractor.cpp
│   ├── line_velocity_controller.cpp
│   ├── object_target_extractor.cpp
│   ├── ball_controller.cpp
│   ├── hurdle_controller.cpp
│   ├── goal_controller.cpp
│   ├── motion_command_selector.cpp
│   ├── p2p_motion_quantizer.cpp
│   ├── mission_controller.cpp
│   └── c_api.cpp
└── CMakeLists.txt
```

- `types`: 점, bbox, 검출 결과, 특징, 속도 명령 등 공통 자료형
- `coordinate_rectifier`: IMU 기반 좌표 보정식
- `line_feature_extractor`: 점선 중심점에서 8차원 특징 계산
- `line_velocity_controller`: 점선 추종과 점선 누락 복구 속도 계산
- `object_target_extractor`: class별 신뢰도 조건을 적용하고 사용할 객체 선택
- `ball_controller`: 공 검출 안정화, 원거리 접근, 카메라 하향, 임시 정지 시퀀스
- `hurdle_controller`: 허들 접근, 하향 감속, 잔발/넘기 placeholder 시퀀스
- `goal_controller`: 집기 후 대기, 골대 탐색/접근, 미세조정/슛 placeholder 시퀀스
- `p2p_motion_quantizer`: 연속 `vx/vy/wz`를 고정 보행 primitive로 변환
- `mission_controller`: 활성 미션 진입·잠금·이탈 및 필요한 controller만 실행
- `control_command`: 활성 미션 결과의 ACTION ID와 ACK/DONE 생명주기 관리
- `motion_command_selector`: 기존 C++/C API 호출자용 stateless 호환 선택기
- `c_api`: G1 Python에서 C++ 코어를 호출하기 위한 연결 인터페이스

## 선택형 보행 backend

인자 없는 `MissionController`와 `ControlCommandCoordinator`는 설치된
`vision_algorithm.yaml`을 읽으며, 현재 공통 기본값은 `p2p`다. 명시적인
`ControlCommandConfig`를 넘기는 호출자는 그 설정을 그대로 사용한다.

`kP2pAction`을 선택하면 각 controller와 MissionController의 속도 계산은 그대로
두고, 최종 선택된 `MotionCommand`만 `P2pMotionQuantizer`를 통과한다.

```text
controller의 mission action_request 존재
    -> ActionCategory::kMission (항상 우선)
그 외의 최종 vx/vy/wz
    -> kVelocity backend: VELOCITY
    -> kP2pAction backend: ActionCategory::kLocomotion
```

기본 P2P primitive는 1/5걸음 직진, 좌·우 곡선 전진, 후진/횡이동,
좌·우 제자리 회전, 제자리회전 후 직진이다. quantizer는 최종 `mission + phase`로
Normal/Fine/Recovery 프로필을 먼저 고르고 각 프로필의 임계값으로 속도를
양자화한다. LINE과 일반 접근은 Normal, 근접 접근·미세조정은 Fine,
탐색·유실복구·라인 재획득은 Recovery다. Fine/Recovery 기본값은 매 1걸음 뒤
다시 관측하도록 긴 5걸음 선택 임계값을 높여 두었다. 실제 P2P 모션 이동량에
맞춰 `P2pMotionConfig` 또는 ROS의 `p2p_fine_*`, `p2p_recovery_*` 파라미터를
조정한다. 유한하지 않은 속도나 모든 축이 deadband 안인 명령은 새 액션을
만들지 않고 HOLD한다.

Mission 액션과 Locomotion 액션은 한 실행기에서 직렬 실행되지만 생명주기는
분리된다. Mission 액션은 DONE 뒤 controller가 요청을 해제할 때까지 같은 액션을
억제한다. Locomotion 액션은 한 블록의 DONE 뒤 최신 영상으로 다시 계산하며,
같은 primitive가 필요하면 새 `action_id`로 즉시 반복할 수 있다. 이미 ACK된
동작은 중간에 교체하지 않으며 pending 동작의 DONE 뒤 Mission 요청을 우선한다.

`ControlCommand`는 최종 출력인 `action`과 양자화 전 연속속도인
`pre_p2p_motion`을 동시에 보존한다. ROS 실행기는 기존과 같이 `action`만
발행한다. MuJoCo 실행기 어댑터는 `action_execution_kind`가
`kVelocityCompatible`이면 같은 `action_id`를 활성 상태로 유지하면서
`pre_p2p_motion`을 RL 보행기에 전달할 수 있다. `kDiscrete`는 집기·허들·슛처럼
시뮬레이터 구현 또는 정지 후 DONE 모사가 필요한 동작이고,
`kStationary`는 정지 관측 동작이다. 어느 방식을 선택해도 어댑터는 core 상태를
직접 바꾸지 않고 해당 `action_id`의 ACK/DONE만 다음 프레임에 반환한다.

`ControlCommand.action_category`는 송신부 내부의 피드백 라우팅 정보다. ROS
메시지에 category를 추가하지 않고, 외부 실행 코드는 서로 겹치지 않는 `action`
코드와 `action_id`로 동작을 식별한다. P2P action 코드 15~25는 ROS
`ActionCommand.msg`에도 같은 값으로 정의돼 있다. ROS에서
ROS `line_perception_node`의 기본 backend는 `p2p`이며 `/cmd_vel`을 발행하지 않고
보행과 미션을 모두 `/jandi_vision/action_cmd`로 보낸다. 기존 ROS 메시지의
필드와 action 번호는 변경하지 않았다. C 호출자는 기존 ABI의 v1/v2를 계속 쓸 수
있고, MuJoCo 어댑터는 `vision_control_command_compute_v3()`에서 PRE-P2P 속도와
실행 정책을 함께 받을 수 있다.

시뮬레이터가 detection 후보부터 공통 perception/mission 경로를 사용해야 할 때는
additive C API인 `vision_mission_controller_step_perception_v1()`을 사용한다.
기존 개별 controller C API는 ABI 호환을 위해 유지되지만, 외부에서 별도 미션
FSM을 구성하는 용도로 사용하지 않는다.

```bash
ros2 run vision line_perception_node --ros-args \
  -p locomotion_backend:=p2p
```

## 현재 공 접근 임시 시퀀스

실제 pickup/미세 한걸음 모션이 준비되기 전까지 공 제어기는 다음 상태만
실행한다.

```text
LINE_FOLLOW
  -> BALL_APPROACH
  -> CAMERA_TILT_DOWN_AND_APPROACH
  -> BALL_FINE_ADJUST (현재 1.5초 저속 직진 placeholder)
  -> BALL_PICKUP (현재 3초 정지 placeholder)
  -> BALL_PICKUP_VERIFY
  -> BALL_PICKUP_VERIFY_OBSERVATION
       공 미검출: has_ball=true
       공 안정 검출: BALL_PICKUP 재시도(최대 3회)
  -> BALL_STAND_UP
  -> CAMERA_RETURN_TO_LINE
  -> BALL_POST_PICKUP_BACK_AWAY
  -> BALL_POST_PICKUP_LINE_RECOVERY (line wz로 제자리 회전)
  -> LINE_FOLLOW
```

- line 후보명령은 ball 활성 여부와 관계없이 별도로 계산·보관한다.
- line 후보와 함께 `line_reference_valid`를 전달한다. 이 값은 정상 `TRACK`일
  때만 true이고, 양수 속도를 내는 `RECOV`/coast/search에서도 false다. 코어는
  가장 최근의 유효한 양수 line `vx`를 `BALL_APPROACH` 전이에 한 번
  고정한다. 원거리 접근 `vx`는 `고정한 line_vx * far_speed_scale`이며 이후
  백그라운드 line controller의 복구속도와 무관하다.
- 공의 raw 화면 중심이 영상 높이의 65% 아래인 판정이 최근 10프레임 중
  6프레임 이상일 때 카메라 하향 상태로 들어간다. 조건을 벗어난 프레임이나
  공을 놓친 프레임은 실패 프레임으로 기록하되 누적 판정을 즉시 초기화하지 않는다.
- 공 자체의 최초 안정 검출도 최근 10프레임 중 7프레임을 사용한다.
- 카메라 전환 중에는 직전 접근속도를 축소한 값으로 직진하고 `wz=0`을 쓴다.
- feedback API에서는 카메라가 실제 DOWN+settled를 보고한 뒤에만 탑뷰
  `BALL_FINE_ADJUST`를 시작한다. 현재는 실제 미세보행 대신 1.5초 저속 직진을 쓰며,
  기존 API는 호환을 위해 설정된 시간으로 카메라 도달을 추정한다.
- `has_ball=true`인 동안에는 새 Ball 미션을 시작하지 않는다. 공 보유 상태가
  해제된 경우에도 Ball 미션 종료 시점부터 기본 10초 동안 공 tracker를 무시한다.
- G1의 90° 가상 카메라는 몸통 roll/pitch를 점진적으로 상쇄하여 endpoint의
  광축을 월드 바닥 수직(-Z)에 맞춘다.
- `VERIFY_PICKUP` 완료 뒤 공이 기본 5프레임 연속 미검출되면 집기 성공으로
  판정한다. 공이 최근 10프레임 중 7회 안정 검출되면 실패로 보고 최대 3회까지
  `PICKUP_BALL`을 다시 요청한다.
- 일어나기와 카메라 정면 복귀 뒤 기본 `vx=-0.10`으로 1초 후진한다. 이후
  line controller의 `wz`만 사용해 제자리 회전하고 `line_reference_valid=true`에서
  Ball mode를 종료한다.

G1 Python은 기존 `VisionBallResult`와 기존 함수 ABI를 바꾸지 않고 새
`vision_ball_controller_compute_v4()`로 line 속도, 정상 TRACK 기준값 여부,
실제 카메라 상태와 ACTION 피드백을 전달한다. 공 보유 여부와 시도 횟수는 별도
getter로 읽어 기존 결과 구조체 ABI를 유지한다.
통합 선택/명령 C API는 `vision_select_mission_command_v2()`와
`vision_control_command_compute_v2()`에 `ball_has_ball`을 전달해야 공 보유 중
허들 억제까지 적용된다.
매 프레임 Ball 계산 뒤 `goal_controller.UpdateBallState(ball_result)`를 호출하고,
Goal 계산 뒤 `ball_controller.SetHasBall(goal_controller.HasBall())`로 되돌려 쓰면
슛 완료 시 두 controller의 공 보유 상태가 함께 false가 된다. C API도 각각
`vision_goal_controller_update_ball_state()`와
`vision_ball_controller_set_has_ball()`을 같은 순서로 호출한다.

C++의 `camera_request`는 `NONE`, `DOWN`, `FORWARD`, `GOAL` 네 상태다. 평소와
카메라 자세 유지 중에는 `NONE`이고, 자세를 바꿀 때만 목표 자세 요청을
실제 도달 피드백이 올 때까지 매 프레임 반환한다. ROS2 연결 코드는 `NONE`일 때
토픽을 발행하지 않는다. 기존 G1 C API의 `request_camera_down`은 ABI 호환을 위해
종전의 자세 level 신호(`1=DOWN`, `0=FORWARD`)로 변환해서 유지한다.

## 현재 허들 임시 시퀀스

```text
LINE_FOLLOW
  -> HURDLE_APPROACH
  -> HURDLE_RECOV_FORWARD (접근 중 미검출 시, 저속 전진하며 재검출 대기)
  -> HURDLE_FAILED (복구 진입 뒤 5초 동안 재검출하지 못하면 정지)
  -> HURDLE_CAMERA_TILT_DOWN_AND_SLOW
  -> HURDLE_RECOV_DOWN (하향 시야에서 미검출 시, 저속 전진하며 재검출 대기)
  -> HURDLE_CONTACT_WALK (현재 2초 저속 직진 placeholder)
  -> HURDLE_CROSS (현재 3초 정지 placeholder)
  -> HURDLE_CAMERA_RETURN_TO_LINE
  -> LINE_FOLLOW
```

- 최초 진입은 허들 중심이 원본 영상 높이의 60% 이상 내려온 프레임이 최근
  30프레임 중 20프레임 이상이고 현재도 보일 때만 허용한다.
- 진입 뒤 허들 중심이 영상 높이의 75% 아래인 판정이 최근 10프레임 중
  7프레임이면 카메라 하향/감속 단계로 들어간다.
- 카메라 전환 중 관측은 안정화/손실 이력에서 제외한다. 접근 중 5프레임 연속
  미검출되면 복구로 들어가고, 3프레임 연속 재검출하면 접근으로 복귀한다.
  복구 중에는 `vx=0.10`으로 저속 전진하며 마지막 검출 방향으로 보정하고,
  5초 동안 재검출하지 못하면 `HURDLE_FAILED`에서 정지한다. 실패 상태도
  Hurdle 미션을 계속 잠그며 명시적인 `Reset()` 전까지 Line/Ball로 이탈하지 않는다.
- `action_request=CONTACT_WALK/CROSS`는 이후 G1 모션 패키지가 실행할 동작 종류다.
- 실제 완료 피드백이 연결되기 전에는 각각 설정된 placeholder 시간으로 넘어간다.

## 현재 골대 임시 시퀀스

```text
StartAfterPickup
  -> LINE_FOLLOW (공 보유 상태로 골대 안정 검출 대기)
  -> GOAL_POST_PICKUP_WAIT
  -> CAMERA_TILT_TO_GOAL_VIEW
  -> GOAL_SEARCH
  -> GOAL_APPROACH
  -> GOAL_FINE_ADJUST (현재 2초 정지 placeholder)
  -> GOAL_SHOOT (현재 3초 정지 placeholder)
  -> CAMERA_RETURN_TO_LINE_VIEW
  -> GOAL_HEADING_RECOVERY
  -> LINE_FOLLOW
```

- `BallResult.has_ball=true`이고 Ball mode가 `LINE_FOLLOW`까지 끝난 뒤의 골대
  검출만 진입 판정에 사용한다. C++은 `UpdateBallState()`, C API는
  `vision_goal_controller_update_ball_state()`로 두 controller 상태를 연결한다.
- `StartAfterPickup()`은 호환용으로 공 보유/골대 진입 대기 상태만 켜며, 골대가
  안정 검출되기 전에는 Line 명령을 덮어쓰지 않는다.
- 공을 3회 모두 집지 못하면 `pickup_failed=true`, `has_ball=false`이므로 골대는
  무시한다. 이후 새 공이 안정 검출되어 Ball 미션이 다시 시작되면 이전 실패
  플래그와 시도 횟수를 초기화한다.
- 골대 최초 검출과 미세조정 진입은 각각 최근 10프레임 중 7프레임을 사용한다.
- `GOAL_VIEW`는 골대를 보는 수평 시야, `FORWARD`는 기존 라인용 평소 시야다.
- 카메라가 평소각에 도달하면 골대 제어는 속도를 덮어쓰지 않고 line controller의
  RECOV 명령을 사용한다. `line_reference_valid=true`가 되면 정상 트래킹으로 끝난다.
- 미세조정과 슛은 현재 `v=0`이지만 `action_request`와 상태는 실제 모션 연결에
  사용할 수 있도록 분리돼 있다.

라인도 동일하게 최근 10프레임 중 중앙에 안정적으로 잡힌 프레임이 7개 이상일
때만 복구를 종료한다. 이후 최대 3프레임 누락은 정상 추종을 유지하고, 네 번째
실패 프레임부터 다시 복구 상태로 들어간다.

여기서 보존한다고 말하는 ABI는 C API의 기존 함수와 `VisionBallResult` 배치다.
C++ 공개 class/struct가 바뀌었으므로 C++ 소비자는 코어 설치 후 반드시 다시
빌드해야 한다. 이미 line 후보를 계산한 C 호출자는 상태를 두 번 갱신하지 않도록
선택만 수행하는 `vision_select_motion_command_v2()`를 사용한다.

ROS2 `line_perception_node`는 액션과 카메라 명령 및 ACK/DONE 토픽을 제공한다.
실제 모션·카메라 실행기는 명령 ID를 그대로 되돌려야 하며, 같은 ID의 명령을
중복 실행하면 안 된다. 실행기 없이 알고리즘 화면만 확인할 때는
`enable_command_transport=false`, `simulate_camera_feedback=true`를 사용한다.

## 코어 빌드와 설치

코어를 수정한 뒤에는 단순히 `cmake --build`만 하지 말고, 아래 순서를
사용하는 것이 가장 안전하다.

```bash
cd /home/noh/vision_core

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/noh/vision_core/install

cmake --build build -j
ctest --test-dir build --output-on-failure
cmake --install build
```

각 명령의 의미는 다음과 같다.

1. `cmake -S . -B build`: 소스 파일과 `CMakeLists.txt` 변경을 빌드 설정에 반영
2. `cmake --build build -j`: 공유 라이브러리와 회귀 테스트 빌드
3. `ctest --test-dir build --output-on-failure`: 공·허들·골대 상태 전이, 시간 경계,
   C API와 명령 선택 회귀 검사
4. `cmake --install build`: 새 라이브러리와 헤더를 실제 사용 위치인
   `/home/noh/vision_core/install`에 설치

마지막 `cmake --install`을 생략하면 `vision`과 G1이 계속 이전에 설치된
라이브러리를 사용할 수 있으므로 반드시 실행한다.

## 어떤 것을 다시 빌드해야 하는가

먼저 아래 표로 판단한다.

| 변경 내용 | 코어 빌드·설치 | `vision` 빌드 | G1 쪽 처리 |
|---|---:|---:|---|
| 특징값의 계산식만 변경, 특징 개수·이름·순서는 동일 | 필요 | 불필요 | 실행 중이면 재시작 |
| 점선·공 속도 수식이나 복구·명령 선택 조건만 변경 | 필요 | 불필요 | 실행 중이면 재시작 |
| 특징 개수·이름·순서 또는 C++ 자료형 변경 | 필요 | 필요 | `core_bridge.py`와 G1 특징 구성 코드 수정 후 재시작 |
| 코어 함수 추가·삭제·인자 변경 또는 C API 변경 | 필요 | 필요 | 사용하는 C API라면 `core_bridge.py`도 수정 |
| 코어에 `.cpp` 파일 추가 또는 `CMakeLists.txt` 변경 | 필요 | 새 기능을 `vision`이 사용하면 필요 | 새 기능을 G1이 사용하면 연결 코드 수정 |
| 카메라·IMU 토픽, YOLO, 화면 표시, ROS 발행만 변경 | 불필요 | 필요 | 불필요 |
| G1 경기장·센서 모사·Python 연결 코드만 변경 | 불필요 | 불필요 | G1 프로세스 재시작 |

여기서 **코어 빌드·설치**는 항상 다음 세 명령 전체를 의미한다.

```bash
cd /home/noh/vision_core

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/noh/vision_core/install

cmake --build build -j
cmake --install build
```

### 코어의 계산식 `.cpp`만 수정한 경우

예를 들어 좌표 보정식, 특징 수식, 점선 속도 수식, 공 접근 수식 또는 명령
선택 조건만 수정했다면 코어만 빌드하고 설치하면 된다.

```bash
cd /home/noh/vision_core

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/noh/vision_core/install

cmake --build build -j
cmake --install build
```

함수 이름과 공개 자료형이 그대로라면 `vision` 패키지는 다시 빌드하지 않아도
새로 설치된 공유 라이브러리를 실행 시 불러온다. 이미 실행 중인 ROS2 노드나
G1 시뮬레이터는 종료한 뒤 다시 실행해야 새 라이브러리가 적용된다.

### 특징 벡터의 수식만 변경한 경우

현재 특징의 개수·이름·순서를 유지하면서 각 특징값의 계산식만 바꾼다면
`line_feature_extractor.cpp`를 수정하고 코어만 빌드·설치한다. `vision`은
다시 빌드하지 않아도 되며, `vision` 노드와 G1은 종료 후 다시 실행한다.

다만 BC와 PPO 모델은 이전 특징값 분포로 학습되어 있다. 특징 차원은 같아도
값의 의미나 범위가 크게 달라지면 기존 체크포인트와 정규화 통계가 더 이상 잘
맞지 않을 수 있으므로 성능을 다시 확인하고 필요하면 데이터 생성과 학습을 다시 한다.

### 특징 벡터의 개수·이름·순서를 변경한 경우

이 경우는 단순한 수식 변경이 아니라 인터페이스 변경이다. 현재 G1은 기본 특징
8개와 `history_state`를 이용해 관측 차원을 만들고, BC 체크포인트의
`feature_mean`, `feature_std`, `feature_names`도 그 구조를 기준으로 사용한다.

따라서 다음 항목을 함께 맞춰야 한다.

1. `vision_core/include/vision_core/types.hpp`의 `Features`
2. `vision_core/include/vision_core/c_api.h`의 `VisionLineFeatures`
3. `vision_core/src/c_api.cpp`의 C++·C 자료형 변환
4. G1 `core_bridge.py`의 `ctypes` `_Features`
5. G1 `perception/features.py`의 `BASE_FEATURE_NAMES`와 특징 묶음
6. G1 `vision_rl/config.py`의 관측 차원과 관련 설정
7. 필요하면 `vision` 노드에서 해당 특징을 사용하는 로그·제어 코드

변경 후 순서는 다음과 같다.

```text
코어 소스·헤더·C API 수정
    ↓
vision_core 빌드·설치
    ↓
vision 패키지 colcon 빌드
    ↓
G1 Python 연결 및 관측 차원 수정
    ↓
기존 데이터셋·BC·PPO 모델 호환성 확인
```

특징 개수나 순서가 달라지면 기존 데이터셋, BC 체크포인트, PPO 체크포인트는
입력 차원이나 의미가 달라져 그대로 사용할 수 없다. 새 특징 기준으로 데이터셋을
다시 만들고 BC·PPO 모델을 다시 학습하는 것이 원칙이다.

### 코어 헤더·함수·C API 또는 `CMakeLists.txt`를 수정한 경우

코어를 먼저 빌드·설치하고, 그다음 `vision` 패키지도 다시 빌드한다.

```bash
cd /home/noh/my_cv
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select vision \
  --allow-overriding vision

source install/setup.bash
```

G1은 ROS2 패키지가 아니므로 `colcon build`하지 않는다. C API의 자료형이나
함수 인자가 바뀌었다면 `/home/noh/G1/g1_rl_workspace/legged_gym/vision/g1/core_bridge.py`
의 `ctypes` 선언을 맞춘 뒤 G1 실행을 다시 시작한다.

### `vision` 노드나 YOLO 연결 코드만 수정한 경우

코어 계산을 바꾸지 않았다면 `vision` 패키지만 다시 빌드하면 된다.

```bash
cd /home/noh/my_cv
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select vision \
  --allow-overriding vision

source install/setup.bash
```

## 수정 위치 기준

- 좌표 보정식: `src/coordinate_rectifier.cpp`
- 점선 특징 종류와 수식: `src/line_feature_extractor.cpp`
- 점선 추종·복구 수식: `src/line_velocity_controller.cpp`
- 객체 선택 조건: `src/object_target_extractor.cpp`
- 공 접근 상태와 속도 수식: `src/ball_controller.cpp`
- 허들 접근 상태와 임시 동작: `src/hurdle_controller.cpp`
- 골대 접근 상태와 임시 동작: `src/goal_controller.cpp`
- 골대·공·허들·점선 최종 미션 잠금: `src/control_command.cpp`
- G1에 새 기능 공개: `include/vision_core/c_api.h`, `src/c_api.cpp`
- 카메라·IMU 토픽과 YOLO 추론: `/home/noh/my_cv/src/vision`

특징 종류나 속도 수식을 바꿀 때는 가능하면 `vision`과 G1에 각각 같은 계산을
복사하지 말고 코어에서 한 번만 변경한다.

## 마무리 전 주의사항

- 특징·좌표 보정·점선 제어·공 제어·최종 명령 잠금의 기준 코드는
  `vision_core`다. 같은 계산을 `vision`과 G1에 각각 복사하지 않는다.
- 코어 헤더, C API 또는 `CMakeLists.txt`까지 변경했다면
  `vision_core` 빌드·설치 후 `vision`도 `colcon build`한다.
- G1 Python 파일만 변경했다면 별도 빌드는 필요 없고 실행 중인 G1을 종료한 뒤
  다시 실행하면 된다.
- 특징 개수나 순서를 변경하면 `core_bridge.py`, G1 관측 차원, 데이터셋,
  정규화 통계, BC·PPO 체크포인트의 호환성을 모두 다시 확인한다.
- 현재 G1 PID 경기장에는 점선과 현재 순번 농구공 하나가 시뮬레이션 입력으로
  연결되어 있다. 골대·백보드·허들용 C API는 준비되어 있지만 G1 입력 생성은
  아직 연결하지 않았다.
- 공은 접근 상태와 속도 계산이 구현되어 있다. 골대·백보드·허들은 대상 선택과
  좌표 보정까지만 있고 행동·속도 수식은 아직 없다.
- 현재 코스 완주 판정은 없다. 마지막 점선을 지나면 점선 누락으로 판단해 복구
  상태에 들어가며, 선호 방향 두 구간과 반대 방향 한 구간의 탐색을 반복한다.
- 실제 ROS2 비전 노드는 아직 로봇 위치와 경기장 기준 경로를 코어에 전달하지
  않는다. 따라서 G1의 위치·경로 기반 복구와 달리 누적된 점선 좌우 방향 기억을
  이용한 복구만 사용할 수 있다.
- 코어 변경 후 `cmake --install build`를 생략하면 `vision`과 G1이 이전에
  설치된 공유 라이브러리를 계속 사용할 수 있다.
- 변경을 마무리할 때는 G1 PID 실행과 실제 `line_perception_node` 실행을 각각
  확인한다.
