#pragma once
#include "Core/GameObject.h"
#include "CarSpec.h"
#include "VehicleController.h"
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <string>
#include <vector>
#include <Nav/RoadDataManager.h>
#include "Nav/ReedsShepp.h"
#include "Nav/VehicleCollision.h"

class SimulationState;

class Car : public GameObject
{
public:
    void Init(const CarSpec &spec, SimulationState *simState, JPH::Vec3 position = JPH::Vec3::sZero());

    void UpdatePhysics(float dt) override; // 고정 물리 dt
    void Update(float dt) override;        // 매프레임
    void UpdateUI(float dt) override;      // 매프레임
    void Draw(ID3D11DeviceContext *context, IEffect &effect) override;
    void Destroy() override;

    // Getter / Setter (Accessors)
    Vec3 GetPosition() const override;                                      // True origin of the object (front axle)
    Vec3 GetRigidbodyPosition() const { return m_rigidbody.GetPosition(); } // rear axel
    Vec3 GetForwardAxis() const;
    float GetDeltaTime() const { return m_deltaTime; }
    float GetSpeed() const { return m_speed; }
    float GetSteerAngle() const { return m_steerAngle; }
    float GetWheelbase() const { return m_wheelbase; }

    void SetPosition(Vec3 position) override;
    void SetRotation(Vec3 direction);
    // 포커스된(선택된) 차량만 콜라이더/트레일 등 디버그 표시를 그린다.
    void SetFocused(bool focused)
    {
        m_isFocused = focused;
        m_drawCollider = focused;
    }
    void SetDestination(const shared_ptr<RoadNode> &parkNode);
    void SetRoaming(bool roaming) { m_roaming = roaming; } // 배회 모드 on/off
    float GetAcceleration() const { return m_acceleration; }
    float GetLength() const { return m_halfExtents.GetZ() * 2.0f; }
    float GetHalfWidth() const { return m_halfExtents.GetX(); }

    // 조작 및 제어 인터페이스 (Control Interface)
    void Accelerate(float desiredVelocity);
    void EmergBrake();
    void Steer(float desiredRadian, float steerRamp = 0.4f);
    void ChangeGear(); // 속도가 낮을 때 전진/후진 기어 토글
    bool IsReverse() const { return m_isReverse; }

    void DriveControl(); // VehicleController에서 호출
    float PurePursuit(Vec3 target);

private:
    // 내부 물리 및 제어 로직 (Internal Physics & Control)
    void UpdateCar();
    void UpdateWithControl();
    void ApplyMotion();
    float GetSignedSpeed() const { return m_speed * (m_isReverse ? -1.0f : 1.0f); }
    JPH::Vec3 ComputeDesiredVelocity() const;

    // 디버그 및 트레일(자국) 렌더링 (Debug & Rendering Helpers)
    void DebugInit();
    void UpdateDebugWindow();
    void UpdateTrail();
    void RebuildTrailRender(RenderObject &render, const std::deque<DirectX::XMFLOAT3> &trail,
                            const std::string &name, const DirectX::XMFLOAT4 &color);
    void RebuildSplineRender();
    void RebuildRSDebugRender(const ReedsShepp::Path &path, const Vec3 &startPos, float startAngleRad,
                              float turningRadius, const Vec3 &targetPos, float targetAngleRad);

#pragma region FSM
    enum class Mode
    {
        Stop,
        Park,
        Drive
    };

    const char *Car::StateToString(Mode mode) const
    {
        switch (mode)
        {
        case Mode::Stop:
            return "Stop";
        case Mode::Park:
            return "Park";
        case Mode::Drive:
            return "Drive";
        }
        return "?";
    }

    enum class SubMode
    {
        None,

        // Drive
        D_Normal,     // 일반 주행
        D_WaitSignal, // 신호대기

        // Park
        P_EXIT,        // 출차
        P_ENTER_LEG1,  // 입차: 스팟 앞 스플라인 점(P)까지
        P_ENTER_LEG2,  // 입차: P -> 스팟
        P_ENTER_ALIGN, // 입차: 최종 정밀 정렬
    };
    const char *Car::SubStateToString(SubMode subMode) const
    {
        switch (subMode)
        {
        case SubMode::None:
            return "None";
        case SubMode::D_Normal:
            return "Normal";
        case SubMode::D_WaitSignal:
            return "WaitSignal";
        case SubMode::P_EXIT:
            return "ParkExit";
        case SubMode::P_ENTER_LEG1:
            return "ParkEnterLeg1";
        case SubMode::P_ENTER_LEG2:
            return "ParkEnterLeg2";
        case SubMode::P_ENTER_ALIGN:
            return "ParkEnterAlign";
        }
        return "?";
    }

    void UpdateMode();
    Mode DecideNextMode(const char **reason) const;
    void OnModeEnter(Mode prev);
    void OnModeExit(Mode next);    // next: 이번에 새로 전환될 상태(m_mode는 아직 지금 나가는 상태 그대로)
    void SetSubMode(SubMode next); // m_subMode를 직접 대입하지 않고 항상 이 함수를 거친다 (전환 로그).

    VehicleCollision::VehicleShape BuildVehicleShape() const;
    void UpdateFindPath();
    void SetCurrentLane(const shared_ptr<Lane> &lane);            // m_currentLane을 직접 대입하지 않고 항상 이 함수를 거친다.
    bool ShouldStopForSignal(const shared_ptr<Lane> &lane) const; // CheckPath(Drive)와 ScanRoadSpeedConstraints/ViolatesSignal(BehaviorPlan)이 공유.
    bool TryFindPathAndSetLane();

    // 배회(roaming) 모드: 목적지 없이 랜덤 후속 레인으로 계속 주행. m_path를 랜덤 successor로 채워 유지한다.
    void EnsureRoamingPath();                                                   // currentLane/초기 path 세팅 + 유지
    void MaintainRoamingPath();                                                 // 지나온 앞부분 트림 + 앞쪽 버퍼 채우기
    shared_ptr<Lane> PickRandomSuccessor(const shared_ptr<Lane> &lane) const;   // successor 중 랜덤(없으면 nullptr)
    vector<LaneStep> BuildRoamingPath(const shared_ptr<Lane> &startLane) const; // startLane + 랜덤 후속 몇 개

    void UpdatePark();
    void BeginParkPlan();
    void BeginParkSpotLeg();
    bool PlanParkLegTo(const Vec3 &targetPos, float targetAngleRad, bool exact = false);
    shared_ptr<RoadNode> GetParkTargetNode() const { return m_parkSpot ? m_parkSpot : m_pendingParkNode; }

    bool PlanEnterForCurrentSpot();
    const Spline *FindBestParkingSpline() const;
    bool ComputeParkPrePose(Vec3 &outPos, float &outAngleRad) const;
    bool ReserveNextParkSpot();
    bool BeginParkEnterOrRetry();

    void UpdateStop();

    void UpdateDrive();
    bool CheckPath();

    void GetLookaheadPose(const shared_ptr<Lane> &startLane, size_t startPathIndex,
                          const Vec3 &fromPosition, float distance, Vec3 &outPosition, Vec3 &outDirection) const;
    float ComputeLookaheadDistance() const;
#pragma endregion

#pragma region BehaviorPlan
    struct RoadSpeedSample
    {
        Vec3 position;
        float distance;
        float speed;
    };

    enum class LaneChoice
    {
        Keep,
        ChangeLeft,
        ChangeRight,
        Abort // 차선변경 매뉴버 중, 원래 레인으로 복귀 (m_laneChangeActive일 때만 생성)
    };

    enum class SpeedAction
    {
        Accelerate,     // +m_maxAccel로 가정하고 시뮬레이션 (desiredSpeed를 넘지 않게 클램프)
        AccelerateHalf, // +m_maxAccel*0.5
        Maintain,       // 가속도 0으로 가정 (지금 속도 유지)
        DecelerateHalf, // -m_maxBrake*0.5
        Decelerate      // -m_maxBrake로 가정
    };

    // 디버그 로그/UI 공용 (UpdateBehaviorPlan의 진단 로그, UpdateDebugWindow의 상태 표시).
    static const char *LaneChoiceToString(LaneChoice c)
    {
        switch (c)
        {
        case LaneChoice::Keep:
            return "Keep";
        case LaneChoice::ChangeLeft:
            return "Left";
        case LaneChoice::ChangeRight:
            return "Right";
        default:
            return "Abort";
        }
    }
    static const char *SpeedActionToString(SpeedAction a)
    {
        switch (a)
        {
        case SpeedAction::Accelerate:
            return "Accel";
        case SpeedAction::AccelerateHalf:
            return "AccelH";
        case SpeedAction::Maintain:
            return "Maint";
        case SpeedAction::DecelerateHalf:
            return "DecelH";
        default:
            return "Decel";
        }
    }

    struct BehaviorCandidate
    {
        LaneChoice laneChoice = LaneChoice::Keep;
        SpeedAction speedAction = SpeedAction::Maintain;
        shared_ptr<Lane> targetLane;                              // Keep이면 m_currentLane, 차선변경이면 그 인접(또는 복귀) 레인
        vector<LaneStep> newPath;                                 // 차선변경/복귀 후보일 때만 채움: targetLane에서 목적지까지 다시 탐색한 경로
        float targetSpeed = 0.0f;                                 // BEHAVIOR_PLAN_INTERVAL(0.2초) 뒤 예상 속도 -- DriveControl이 실제로 명령할 값
        float horizonEndSpeed = 0.0f;                             // BEHAVIOR_SAFETY_HORIZON(3초) 뒤 예상 속도 -- 목표속도 대비 비용평가용
        float minApproachGap = std::numeric_limits<float>::max(); // 앞차(전방 동방향 리더)와의 최소 범퍼 gap(m). 리더 없으면 max.
        float minTimeHeadway = std::numeric_limits<float>::max(); // 앞차와의 최소 시간헤드웨이(s) = 범퍼gap/자기속도. 짧을수록 바짝 붙음.
        bool collisionFree = true;                                // 시뮬레이션한 3초 동안 한 번도 겹치지 않았는지
        bool signalViolation = false;                             // 신호를 지켜야 하는데(ShouldStopForSignal) 못 멈추고 정지선을 넘는지
        float maxSpeedOvershoot = 0.0f;                           // 궤적 지점별 국소 안전속도 상한을 가장 크게 초과한 값(m/s)
        float maxLateralAccel = 0.0f;                             // 궤적 중 최대 |횡가속|(m/s^2) -- 커브/차선변경 승차감 비용(w3)용
    };
    // 후보 안전판정용으로 시뮬레이션한 미래 한 시점의 (위치, 진행방향, 속도, 지금까지 이동한 거리).
    struct TrajectorySample
    {
        Vec3 position;
        Vec3 direction;
        float speed = 0.0f;
        float distanceTraveled = 0.0f; // t=0(지금)부터 이 샘플까지 이동한 거리(m) -- 도로제약 샘플과 대조할 때 씀
    };
    struct TrajectorySafety
    {
        bool collisionFree = true;
        float minGap = std::numeric_limits<float>::max();         // 리더와의 최소 범퍼 gap(m)
        float minTimeHeadway = std::numeric_limits<float>::max(); // 리더와의 최소 시간헤드웨이(s)
    };

    struct NearbyCar
    {
        Car *car = nullptr;
        bool yieldsToMe = false;
    };

    void UpdateBehaviorPlan();
    BehaviorCandidate BuildCandidate(LaneChoice laneChoice, SpeedAction speedAction, const shared_ptr<Lane> &lane,
                                     const std::vector<RoadSpeedSample> &roadSamples,
                                     const std::vector<NearbyCar> &nearbyCars) const;
    bool IsCandidateSafe(const BehaviorCandidate &candidate) const;
    float EvaluateCandidateCost(const BehaviorCandidate &candidate, float desiredSpeed) const;
    float ComputeSpeedCapFromSamples(const std::vector<RoadSpeedSample> &samples, float distanceOffset) const; // 안전속도 상한 구하기
    std::vector<NearbyCar> CollectNearbyCars() const;
    bool IsTurningAhead() const;                  // 교차로 우선순위: 직진 > 회전 판정용
    bool HasPriorityOver(const Car *other) const; // 우선순위 직진 > 회전, 동급이면 이름 비교
    void AppendCarConstraintSamples(std::vector<RoadSpeedSample> &samples,
                                    const std::vector<NearbyCar> &nearbyCars, float lookDistance) const; // nearbyCars 중 내 예정 경로 코리도 안의 차를 도로제약 샘플(가상 리더)로 변환해 추가
    std::vector<TrajectorySample> SimulateEgoTrajectory(const shared_ptr<Lane> &lane, float simAccel,
                                                        const std::vector<RoadSpeedSample> &roadSamples) const; // lane의 스플라인을 따라 Pure Pursuit + 자전거 모델로 시뮬래아션한 궤적
    // 상대 차 미래 pose 예측용
    struct OtherPrediction
    {
        struct Segment
        {
            const Spline *spline;
            float startT;
            float arcLength; // startT부터 이 세그먼트 끝까지의 호길이
        };
        std::vector<Segment> segments; // 비어 있으면 직진 폴백
        Vec3 basePos = Vec3::sZero();  // 예측 시작점(뒷축)
        Vec3 baseFwd = Vec3::sZero();  // 시작 진행방향
        float lateralOffset = 0.0f;    // 레인 중심선 기준 횡 오프셋(부호 포함) -- 예측 내내 유지
    };
    OtherPrediction BuildOtherPrediction(const Car *other) const;
    static void PredictOtherPose(const OtherPrediction &pred, float distance, Vec3 &outPos, Vec3 &outFwd);
    // others는 각자 자기 레인 스플라인을 따라 등속 전진한다고 예측해 매 스텝 ego 궤적과 겹치는지
    // 본다. 실제로 겹친 적이 있으면 collisionFree=false, 그와 별개로 전 구간 최소 중심간 거리도 반환.
    TrajectorySafety EvaluateTrajectorySafety(const std::vector<TrajectorySample> &trajectory,
                                              const std::vector<NearbyCar> &others) const;
    // lane에 신호가 있고 지금 서야 하는 상황(ShouldStopForSignal)인데, trajectory가 멈추지 못하고
    // 정지선(신호 노드 위치)을 넘어버리는지 확인한다.
    bool ViolatesSignal(const shared_ptr<Lane> &lane, const std::vector<TrajectorySample> &trajectory) const;
    std::vector<RoadSpeedSample> ScanRoadSpeedConstraints(float lookDistance) const;
#pragma endregion

public:
    static constexpr float ARRIVE_DISTANCE = 5.0f;
    static constexpr float PARK_ARRIVE_DISTANCE = 10.0f;
    static constexpr float LANE_TRANSITION_THRESHOLD = 2.0f; // 다음 차선으로 완전히 넘어가는(전환되는) 임계값
    static constexpr float LANE_CURVE_LOOKAHEAD_THRESHOLD = 8.0f;

private:
    // 설정 및 스펙 상수/변수 (Constants & Specifications)
    const float m_maxSpeed = 200.0f / 3.6f;           // 200 km/h
    const float m_maxAccel = (100.0f / 3.6f) / 14.0f; // 0-100 km/h in 14s
    const float m_maxBrake = (100.0f / 3.6f) / 15.0f;
    const float m_maxEmergBrake = (100.0f / 3.6f) / 3.0f; // 100-0 km/h in 3s

    float m_wheelbase = 0.0f;
    float m_mass = 1.0f;
    Vec3 m_halfExtents = Vec3::sZero();              // 충돌판정용 차체 반크기(x=반폭, z=반길이). CarSpec::halfExtents.
    float m_maxSteerAngle = ToRadians(45.0f);        // 최대 조향각 (45도)
    static constexpr float CURVE_SPEED_COEFF = 1.5f; // 최대 코너링 속도 = CURVE_SPEED_COEFF * sqrt(R)
    static constexpr float STEER_RAMP_RATE = 0.4f;

    // 차량 주행 상태 변수 (Vehicle States)
    float m_speed = 0.0f;
    float m_acceleration = 0.0f;
    float m_steerAngle = 0.0f;
    bool m_isReverse = false;
    float m_deltaTime = 0.0f;

    bool m_wantSegmentTick = false;
    bool m_isFocused = false; // 포커스 여부 (입력 처리용)

    // 컴포넌트 및 AI 상태 (Components & Systems)
    SimulationState *m_SimState = nullptr;
    Mode m_mode = Mode::Stop;
    SubMode m_subMode = SubMode::D_Normal; // DriveMode::Drive 안에서만 의미 있음
    VehicleController m_vehicleController; // DriveMode가 세운 계획(세그먼트)을 실제로 실행
    shared_ptr<Lane> m_destLane;
    shared_ptr<Lane> m_currentLane;

    shared_ptr<RoadNode> m_parkSpot;        // 예약된 목표 주차칸(있는 동안은 "이 자리에 주차 중/주차 예정")
    shared_ptr<RoadNode> m_pendingParkNode; // 예약 전, 도착하면 그때 주차칸을 예약할 목표 Park 노드
    bool m_parkSequenceActive = false;
    int m_parkNodeId = -1;                 // 이번 입차의 대상 Park 노드 id (다른 빈 자리 재예약에 씀)
    int m_parkLegTries = 0;                // 현재 스팟의 진입 leg 체인 시도 횟수 (무한 체인 방지)
    unordered_set<int> m_triedParkSpotIds; // 이번 입차에서 경로탐색이 실패해 이미 시도해본 ParkSpot id들
    bool m_parkPlanPending = false;

    bool m_roaming = false;                        // 배회 모드: 목적지 없이 스플라인 따라 랜덤 후속 레인으로 계속 주행
    static constexpr size_t ROAMING_MIN_AHEAD = 3; // 배회 시 현재 레인 앞으로 항상 유지할 최소 레인 버퍼 수

    vector<LaneStep> m_path;
    size_t m_pathIndex = 0;
    float m_currentTime = 0.0f;
    // 노란불 때 "정지거리 안쪽이라 통과" 확정한 신호 id(초록될 때까지 유지). 없으면 -1.
    mutable int m_committedYellowNodeId = -1;

    // 행동 계획(Behavior Plan) 상태
    static constexpr float BEHAVIOR_PLAN_INTERVAL = 0.2f;   // 행동 후보 재판단 주기
    static constexpr float BEHAVIOR_LOOKAHEAD_TIME = 5.0f;  // 목표속도 산정 시 도로제약을 내다보는 시간(초)
    static constexpr float BEHAVIOR_SAFETY_HORIZON = 3.0f;  // 궤적 시뮬레이션으로 안전판정할 미래 시야(초, 사람의 3초 룰)
    static constexpr float BEHAVIOR_SIM_STEP = 0.1f;        // 궤적 시뮬레이션 적분 간격(초)
    static constexpr float MIN_SAFE_GAP = 1.0f;             // 시뮬레이션 중 앞차와 최소로 유지해야 하는 범퍼 gap(m).
    static constexpr float DESIRED_HEADWAY = 1.5f;          // 이보다 시간헤드웨이가 짧으면 부족분에 following 비용(w6).
    static constexpr float LANE_CHANGE_DONE_LATERAL = 0.5f; // 목표 레인 중심에서 이 거리 안이면 차선변경 완료로 본다(m).

    BehaviorWeights m_behaviorWeights;
    float m_lastBehaviorPlanTime = -1000.0f; // 처음 Drive 진입 시 바로 첫 판단이 돌도록 충분히 과거로 초기화
    BehaviorCandidate m_currentBehaviorPlan;
    float m_lastDesiredSpeed = 0.0f; // UpdateBehaviorPlan이 마지막으로 계산한, 지금 위치 기준 속도 캡(디버그 UI 표시용)
    bool m_emergencyBrake = false;

    // 차선변경 매뉴버 진행 상태: 커밋 순간부터 목표 레인 중심에 정착할 때까지 active. active 동안엔
    // 원 레인 복귀(Abort) 후보를 함께 만들어, 목표 레인이 도중에 unsafe로 바뀌면 되돌아갈 수 있게 한다.
    bool m_laneChangeActive = false;
    shared_ptr<Lane> m_laneChangeFromLane; // Abort 시 복귀할 원래 레인

    // 스폰 및 리셋 데이터 (Spawn / Reset Data)
    DirectX::XMFLOAT3 m_spawnPosition = {0.0f, 0.0f, 0.0f};
    DirectX::XMFLOAT4 m_spawnRotation = {0.0f, 0.0f, 0.0f, 1.0f};

    // 트레일 및 디버그 렌더링 리소스 (Render Resources & Trail Data)
    static constexpr float TRAIL_SAMPLE_DISTANCE = 0.5f;
    static constexpr size_t TRAIL_MAX_POINTS = 100;

    std::deque<DirectX::XMFLOAT3> m_rearTrail;
    std::deque<DirectX::XMFLOAT3> m_frontTrail;
    RenderObject m_rearTrailRender;
    RenderObject m_frontTrailRender;
    RenderObject m_debugBox;
    bool m_drawCollider = false;
    RenderObject m_steerLine;
    RenderObject m_targetMarker;
    RenderObject m_splineRender;
    RenderObject m_parkPathRender;   // Park 계획(RS 경로) 폴리라인
    RenderObject m_parkTargetMarker; // Park 목표 위치
    RenderObject m_parkTargetLine;   // Park 목표 방향
};

static float CalcMaxSteerAngle(float speed)
{
    constexpr float LOW_SPEED_CUTOFF = 18.26f / 3.6f;   // use MAX_STEER_ANGLE (cause 20.2f / LOW_SPEED_CUTOFF^2 > 45 degree)
    constexpr float MAX_STEER_ANGLE = ToRadians(45.0f); // 45 degree
    return (speed <= LOW_SPEED_CUTOFF) ? MAX_STEER_ANGLE : 20.2f / (speed * speed);
}
static float CalcMaxSpeed(float targetAngle)
{
    constexpr float MAX_STEER_ANGLE = ToRadians(45.0f); // 45 degree
    targetAngle = std::clamp(std::abs(targetAngle), 0.15f, MAX_STEER_ANGLE);
    return std::sqrt(20.2f / targetAngle);
}