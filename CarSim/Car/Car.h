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

class Car : public GameObject
{
public:
    void Init(const CarSpec &spec, RoadDataManager *roadDataManager, JPH::Vec3 position = JPH::Vec3::sZero());

    void Update(float dt) override;   // 고정 물리 dt
    void UpdateAI(float dt) override; // 매프레임
    void UpdateUI(float dt) override; // 매프레임
    void Draw(ID3D11DeviceContext *context, IEffect &effect) override;
    void Destroy() override;

    // Getter / Setter (Accessors)
    Vec3 GetPosition() const override;
    Vec3 GetForwardAxis() const;
    float GetDeltaTime() const { return m_deltaTime; }
    float GetSpeed() const { return m_speed; }
    float GetSteerAngle() const { return m_steerAngle; }
    Vec3 GetRigidbodyPosition() const { return m_rigidbody.GetPosition(); }
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
    float GetAcceleration() const { return m_acceleration; }
    float GetLength() const { return m_halfExtents.GetZ() * 2.0f; }

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

    bool IsOffCourse();

    shared_ptr<RoadNode> GetParkTargetNode() const { return m_parkSpot ? m_parkSpot : m_pendingParkNode; }

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

    void UpdateStop();
    void UpdatePark();
    void UpdateDrive();
    bool CheckPath();

    void GetLookaheadPose(const shared_ptr<Lane> &startLane, size_t startPathIndex,
                          const Vec3 &fromPosition, float distance, Vec3 &outPosition, Vec3 &outDirection) const;
    float ComputeLookaheadDistance() const;

    // 도로 위 한 지점의 속도 제약 샘플(제한속도/커브/신호 등). ScanRoadSpeedConstraints가 만들고,
    // 행동 계획(BuildCandidate/ComputeSpeedCapFromSamples)도 그대로 재사용한다.
    struct RoadSpeedSample
    {
        Vec3 position;
        float distance;
        float speed;
    };

    // 행동 계획(behavior plan): BEHAVIOR_PLAN_INTERVAL(0.2초)마다 후보를 평가해 하나를 고르고, 그 사이엔
    // DriveControl이 고른 후보(m_currentBehaviorPlan)를 그대로 따라간다.
    enum class LaneChoice
    {
        Keep,
        ChangeLeft,
        ChangeRight
    };
    // 후보의 종방향 속도 성향. 실제 목표속도는 이 성향 + 도로제약(desiredSpeed)로 시뮬레이션해서 구한다.
    enum class SpeedAction
    {
        Accelerate, // +m_maxAccel로 가정하고 시뮬레이션 (desiredSpeed를 넘지 않게 클램프)
        Maintain,   // 가속도 0으로 가정 (지금 속도 유지)
        Decelerate  // -m_maxBrake로 가정
    };
    struct BehaviorCandidate
    {
        LaneChoice laneChoice = LaneChoice::Keep;
        SpeedAction speedAction = SpeedAction::Maintain;
        shared_ptr<Lane> targetLane;  // Keep이면 m_currentLane, 차선변경이면 그 인접 레인
        vector<LaneStep> newPath;     // 차선변경 후보일 때만 채움: targetLane에서 목적지까지 다시 탐색한 경로
        float targetSpeed = 0.0f;     // BEHAVIOR_PLAN_INTERVAL(0.2초) 뒤 예상 속도 -- DriveControl이 실제로 명령할 값
        float horizonEndSpeed = 0.0f; // BEHAVIOR_SAFETY_HORIZON(3초) 뒤 예상 속도 -- 목표속도 대비 비용평가용
        float minApproachGap = std::numeric_limits<float>::max(); // 시뮬레이션 동안 주변 차와의 최근접 거리(중심간)
        bool collisionFree = true;    // 시뮬레이션한 3초 동안 한 번도 겹치지 않았는지
        bool signalViolation = false; // 신호를 지켜야 하는데(ShouldStopForSignal) 못 멈추고 정지선을 넘는지
        // 궤적의 각 지점에서, "그 지점부터 남은 도로제약까지의 거리" 기준으로 다시 계산한 국소 안전속도
        // 상한을 얼마나 초과했는지의 최댓값(m/s). 지금 시점의 desiredSpeed 하나만 보면, desiredSpeed가
        // 계속 낮아지는 접근 구간에서 "3초 뒤엔 이미 desiredSpeed가 더 낮아져 있을 것"이라는 걸 놓쳐
        // 감속이 늦어진다 -- 그래서 매 지점마다 그 지점 기준 상한과 비교해 가장 심하게 넘어선 값을 잡는다.
        float maxSpeedOvershoot = 0.0f;
    };
    // 주변 차량 인지 결과: 같은 레인 위에서 gap(범퍼~범퍼 거리)이 가장 작은 차 하나.
    struct NearbyCarGap
    {
        Car *car = nullptr;
        float gap = 0.0f;
        float speed = 0.0f;
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
        float minGap = std::numeric_limits<float>::max();
    };

    void UpdateBehaviorPlan();
    BehaviorCandidate BuildCandidate(LaneChoice laneChoice, SpeedAction speedAction, const shared_ptr<Lane> &lane,
                                     const std::vector<RoadSpeedSample> &roadSamples) const;
    bool IsCandidateSafe(const BehaviorCandidate &candidate) const;
    float EvaluateCandidateCost(const BehaviorCandidate &candidate, float desiredSpeed) const;
    // samples 중 distanceOffset보다 먼 것들만 골라, "그 지점부터 남은 거리" 기준으로 안전속도 상한을
    // 구한다. distanceOffset=0이면 지금 이 순간의 목표속도(desiredSpeed)와 같다.
    float ComputeSpeedCapFromSamples(const std::vector<RoadSpeedSample> &samples, float distanceOffset) const;
    NearbyCarGap FindLeaderOnLane(const shared_ptr<Lane> &lane) const;
    NearbyCarGap FindFollowerOnLane(const shared_ptr<Lane> &lane) const;
    // lane의 스플라인을 따라 Pure Pursuit + 자전거 모델로 BEHAVIOR_SAFETY_HORIZON 동안 BEHAVIOR_SIM_STEP
    // 간격으로 전진시켜본 궤적. simAccel(가정 가속도)로 speedAction의 종방향 프로파일을 반영한다. 속도는
    // 물리적 한계([0, m_maxSpeed])로만 클램프한다 -- desiredSpeed에 맞는지는 여기서 미리 잘라내지 않고
    // EvaluateCandidateCost가 판단한다 (그래야 "멈춰야 하는데 못 멈추는" 후보가 실제로 나쁘게 평가됨).
    std::vector<TrajectorySample> SimulateEgoTrajectory(const shared_ptr<Lane> &lane, float simAccel) const;
    // others(주로 리더/팔로워)는 지금 속도로 등속 직진한다고 가정하고 외삽해, 매 스텝 ego 궤적과 겹치는지
    // 본다. 실제로 겹친 적이 있으면 collisionFree=false, 그와 별개로 전 구간 최소 중심간 거리도 반환.
    TrajectorySafety EvaluateTrajectorySafety(const std::vector<TrajectorySample> &trajectory,
                                              const std::vector<Car *> &others) const;
    // lane에 신호가 있고 지금 서야 하는 상황(ShouldStopForSignal)인데, trajectory가 멈추지 못하고
    // 정지선(신호 노드 위치)을 넘어버리는지 확인한다.
    bool ViolatesSignal(const shared_ptr<Lane> &lane, const std::vector<TrajectorySample> &trajectory) const;

    void BeginParkPlan();
    void BeginParkSpotLeg();
    bool PlanParkLegTo(const Vec3 &targetPos, float targetAngleRad, bool exact = false);

    bool PlanEnterForCurrentSpot();
    bool ReserveNextParkSpot();
    bool BeginParkEnterOrRetry();

    VehicleCollision::VehicleShape BuildVehicleShape() const;

    void UpdateFindPath();
    bool TryFindPathAndSetLane();

    void SetCurrentLane(const shared_ptr<Lane> &lane); // m_currentLane을 직접 대입하지 않고 항상 이 함수를 거친다.

    std::vector<RoadSpeedSample> ScanRoadSpeedConstraints(float lookDistance) const;
    bool ShouldStopForSignal(const shared_ptr<Lane> &lane) const;

public:
    // 다음 차선으로 완전히 넘어가는(전환되는) 임계값
    static constexpr float LANE_TRANSITION_THRESHOLD = 2.0f;
    // 목적지 레인 끝점과 이 거리 안이면 "도착"으로 본다 (DecideNextMode/UpdateStop 공용).
    static constexpr float ARRIVE_DISTANCE = 5.0f;
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

    // 컴포넌트 및 AI 상태 (Components & Systems)
    RoadDataManager *m_RoadDataManager = nullptr;
    bool m_isFocused = false; // 포커스 여부 (입력 처리용)
    Mode m_mode = Mode::Stop;
    SubMode m_subMode = SubMode::D_Normal; // DriveMode::Drive 안에서만 의미 있음
    VehicleController m_vehicleController; // DriveMode가 세운 계획(세그먼트)을 실제로 실행
    shared_ptr<Lane> m_destLane;
    shared_ptr<Lane> m_currentLane;

    shared_ptr<RoadNode> m_parkSpot;        // 예약된 목표 주차칸(있는 동안은 "이 자리에 주차 중/주차 예정")
    shared_ptr<RoadNode> m_pendingParkNode; // 예약 전, 도착하면 그때 주차칸을 예약할 목표 Park 노드
    bool m_parkSequenceActive = false;
    int m_parkNodeId = -1;                 // 이번 입차의 대상 Park 노드 id (다른 빈 자리 재예약에 씀)
    unordered_set<int> m_triedParkSpotIds; // 이번 입차에서 경로탐색이 실패해 이미 시도해본 ParkSpot id들
    bool m_parkPlanPending = false;

    vector<LaneStep> m_path;
    size_t m_pathIndex = 0;
    float m_currentTime = 0.0f;
    // 노란불 때 "정지거리 안쪽이라 통과" 확정한 신호 id(초록될 때까지 유지). 없으면 -1.
    mutable int m_committedYellowNodeId = -1;

    // 행동 계획(Behavior Plan) 상태
    static constexpr float BEHAVIOR_PLAN_INTERVAL = 0.2f;  // 행동 후보 재판단 주기
    static constexpr float BEHAVIOR_LOOKAHEAD_TIME = 5.0f; // 목표속도 산정 시 도로제약을 내다보는 시간(초)
    static constexpr float BEHAVIOR_SAFETY_HORIZON = 3.0f; // 궤적 시뮬레이션으로 안전판정할 미래 시야(초, 사람의 3초 룰)
    static constexpr float BEHAVIOR_SIM_STEP = 0.1f;       // 궤적 시뮬레이션 적분 간격(초)
    static constexpr float MIN_SAFE_GAP = 2.0f;            // 시뮬레이션 중 주변 차와 최소로 유지해야 하는 거리(m). 차선유지/변경 공통.

    // 디버그 창(UpdateDebugWindow)에서 조절 가능한 차량별 성향(가중치). CarSpec::behaviorWeights에서
    // 기본값이 오고, 창에서 바꾸면 그 차에만 적용된다.
    BehaviorWeights m_behaviorWeights;
    float m_lastBehaviorPlanTime = -1000.0f; // 처음 Drive 진입 시 바로 첫 판단이 돌도록 충분히 과거로 초기화
    BehaviorCandidate m_currentBehaviorPlan;

    // 차량 주행 상태 변수 (Vehicle States)
    float m_speed = 0.0f;
    float m_acceleration = 0.0f;
    float m_steerAngle = 0.0f;
    bool m_isReverse = false;
    float m_deltaTime = 0.0f;
    bool m_wantSegmentTick = false;

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
    RenderObject m_originMarker;
    bool m_drawCollider = false;
    RenderObject m_steerLine;
    RenderObject m_targetMarker;
    RenderObject m_splineRender;
    RenderObject m_parkPathRender;                // Park 계획(RS 경로) 폴리라인
    RenderObject m_parkTargetMarker;              // Park 목표 위치
    RenderObject m_parkTargetLine;                // Park 목표 방향
};

static float CalcMaxSteerAngle(float speed)
{
    constexpr float LOW_SPEED_CUTOFF = 5.072f;          // use MAX_STEER_ANGLE (cause 20.2f / LOW_SPEED_CUTOFF^2 > 45 degree)
    constexpr float MAX_STEER_ANGLE = ToRadians(45.0f); // 45 degree
    return (speed <= LOW_SPEED_CUTOFF) ? MAX_STEER_ANGLE : 20.2f / (speed * speed);
}
static float CalcMaxSpeed(float targetAngle)
{
    constexpr float LOW_SPEED_CUTOFF = 5.072f;          // use MAX_STEER_ANGLE (cause 20.2f / LOW_SPEED_CUTOFF^2 > 45 degree)
    constexpr float MAX_STEER_ANGLE = ToRadians(45.0f); // 45 degree
    targetAngle = std::clamp(std::abs(targetAngle), 0.15f, MAX_STEER_ANGLE);
    return std::sqrt(20.2f / targetAngle);
}