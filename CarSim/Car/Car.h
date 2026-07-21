#pragma once
#include "Core/GameObject.h"
#include "CarSpec.h"
#include "VehicleController.h"
#include <cmath>
#include <deque>
#include <string>
#include <vector>
#include <Nav/RoadDataManager.h>
#include "Nav/ReedsShepp.h"
#include "Nav/HybridAStar.h"
#include "Nav/CarFollowing.h"
#include "Nav/Mobil.h"

class Car : public GameObject
{
public:
    void Init(const CarSpec &spec, RoadDataManager *roadDataManager, JPH::Vec3 position = JPH::Vec3::sZero());

    void Update(float dt) override;   // 고정 물리 dt
    void UpdateAI(float dt) override; // 매프레임
    void UpdateUI(float dt) override; // 매프레임
    void Draw(ID3D11DeviceContext *context, IEffect &effect) override;

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
    void SetFocused(bool focused) { m_isFocused = focused; }
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
    void RebuildParkDebugRender(const ReedsShepp::Path &path, const Vec3 &startPos, float startAngleRad,
                                float turningRadius, const Vec3 &targetPos, float targetAngleRad);

    bool IsOffCourse();

    shared_ptr<RoadNode> GetParkTargetNode() const { return m_parkSpot ? m_parkSpot : m_pendingParkNode; }

    enum class State
    {
        Stop,
        Park,
        Drive
    };

    const char *Car::StateToString(State mode) const
    {
        switch (mode)
        {
        case State::Stop:
            return "Stop";
        case State::Park:
            return "Park";
        case State::Drive:
            return "Drive";
        }
        return "?";
    }

    enum class SubState
    {
        None,

        // Drive
        D_Normal,     // 일반 주행
        D_Avoid,      // 회피 하이브리드 A*
        D_Stop,       // 정차
        D_WaitSignal, // 신호대기

        // Park
        P_EXIT,        // 출차
        P_ENTER_LEG1,  // 입차: 스팟 앞 스플라인 점(P)까지
        P_ENTER_LEG2,  // 입차: P -> 스팟
        P_ENTER_ALIGN, // 입차: 최종 정밀 정렬
    };
    const char *Car::SubStateToString(SubState subMode) const
    {
        switch (subMode)
        {
        case SubState::D_Normal:
            return "Normal";
        case SubState::D_Avoid:
            return "Avoid";
        case SubState::D_Stop:
            return "Stop";
        case SubState::D_WaitSignal:
            return "WaitSignal";
        case SubState::P_EXIT:
            return "ParkExit";
        case SubState::P_ENTER_LEG1:
            return "ParkEnterLeg1";
        case SubState::P_ENTER_LEG2:
            return "ParkEnterLeg2";
        case SubState::P_ENTER_ALIGN:
            return "ParkEnterAlign";
        }
        return "?";
    }
    void UpdateMode();
    State DecideNextMode(const char **reason) const;
    void OnModeEnter(State prev);
    void OnModeExit(State next); // next: 이번에 새로 전환될 상태(m_mode는 아직 지금 나가는 상태 그대로)

    void UpdateStop();
    void UpdatePark();
    void UpdateDrive();
    bool CheckPath();
    bool TryLaneChange();
    bool TryAvoidObstacle();

    void BeginParkPlan();
    void BeginParkSpotLeg();
    bool PlanParkLegTo(const Vec3 &targetPos, float targetAngleRad, bool exact = false);
    bool PlanEnterForCurrentSpot();
    bool ReserveNextParkSpot();
    bool BeginParkEnterOrRetry();

    void BeginAvoidPlan();
    HybridAStar::VehicleShape BuildVehicleShape() const;

    void UpdateFindPath();
    bool TryFindPathAndSetLane();

    void SetCurrentLane(const shared_ptr<Lane> &lane); // m_currentLane을 직접 대입하지 않고 항상 이 함수를 거친다.

    struct RoadSpeedSample
    {
        Vec3 position;
        float distance;
        float speed;
    };
    std::vector<RoadSpeedSample> ScanRoadSpeedConstraints(float lookDistance) const;
    void RescanRoadSpeedConstraints();
    void DriveSpeedIDM(float steerSpeedCap);
    CarFollowing::Params BuildIdmParams(float v0) const;

    // 어떤 레인 위에서 찾은 다른 차 하나 + 그 레인 기준 위치(arclength).
    struct LaneNeighbor
    {
        Car *car = nullptr;
        float position = 0.0f;
    };
    void FindLaneNeighbors(const shared_ptr<Lane> &lane, float myPosition,
                           LaneNeighbor &outLeader, LaneNeighbor &outFollower) const;
    Mobil::VehicleState ToVehicleState(const LaneNeighbor &n) const;

public:
    // 차선 진입 허용 오차/임계값 (예: 현재 타깃 차선에 안착했는지 확인하는 기준)
    static constexpr float LANE_ENTRY_THRESHOLD = 5.0f;
    // 다음 차선으로 완전히 넘어가는(전환되는) 임계값
    static constexpr float LANE_TRANSITION_THRESHOLD = 2.0f;

private:
    // 설정 및 스펙 상수/변수 (Constants & Specifications)
    const float m_maxSpeed = 200.0f / 3.6f;           // 200 km/h
    const float m_maxAccel = (100.0f / 3.6f) / 14.0f; // 0-100 km/h in 14s
    const float m_maxBrake = (100.0f / 3.6f) / 15.0f;
    const float m_maxEmergBrake = (100.0f / 3.6f) / 3.0f; // 100-0 km/h in 3s
    float m_accelRampTime = 0.0f;                         // 현재 가속/제동 램프 시작 후 경과 시간 (S자 보간용)
    static constexpr float ACCEL_RAMP_DURATION = 1.6f;
    static constexpr float BRAKE_RAMP_DURATION = 2.5f;

    float m_wheelbase = 0.0f;
    float m_mass = 1.0f;
    Vec3 m_halfExtents = Vec3::sZero();              // 충돌판정용 차체 반크기(x=반폭, z=반길이). CarSpec::halfExtents.
    float m_maxSteerAngle = ToRadians(45.0f);        // 최대 조향각 (45도)
    static constexpr float CURVE_SPEED_COEFF = 0.8f; // 최대 코너링 속도 = CURVE_SPEED_COEFF * sqrt(R)
    static constexpr float STEER_RAMP_RATE = 0.4f;

    // 컴포넌트 및 AI 상태 (Components & Systems)
    RoadDataManager *m_RoadDataManager = nullptr;
    bool m_isFocused = false; // 포커스 여부 (입력 처리용)
    State m_mode = State::Stop;
    SubState m_subMode = SubState::D_Normal; // DriveMode::Drive 안에서만 의미 있음
    VehicleController m_vehicleController;   // DriveMode가 세운 계획(세그먼트)을 실제로 실행
    shared_ptr<Lane> m_destLane;
    shared_ptr<Lane> m_currentLane;

    shared_ptr<RoadNode> m_parkSpot;        // 예약된 목표 주차칸(있는 동안은 "이 자리에 주차 중/주차 예정")
    shared_ptr<RoadNode> m_pendingParkNode; // 예약 전, 도착하면 그때 주차칸을 예약할 목표 Park 노드
    bool m_parkSequenceActive = false;
    int m_parkNodeId = -1;                 // 이번 입차의 대상 Park 노드 id (다른 빈 자리 재예약에 씀)
    unordered_set<int> m_triedParkSpotIds; // 이번 입차에서 경로탐색이 실패해 이미 시도해본 ParkSpot id들
    bool m_parkPlanPending = false;

    float m_avoidReplanCooldown = 0.0f;
    vector<LaneStep> m_path;
    size_t m_pathIndex = 0;
    static constexpr float LOOK_PROFILE_TIME = 5.0f;
    static constexpr size_t SPEED_PROFILE_COUNT = 10; // 재스캔 주기(LOOK_PROFILE_TIME / 이 값) 계산에만 씀
    std::vector<RoadSpeedSample> m_roadConstraints;
    float m_lastConstraintScanTime = 0.0f;
    Spline m_currentSpline;
    float m_currentTime = 0.0f;

    // DriveSpeedIDM이 도로 제약(커브/레인 제한속도/정지점)을 가상 리더로 다룰 때 쓰는 IIDM+CAH 파라미터.
    static constexpr float IDM_TIME_HEADWAY = 1.2f;        // T
    static constexpr float IDM_STANDSTILL_DISTANCE = 2.0f; // s0
    static constexpr float IDM_ACCEL_EXPONENT = 4.0f;      // delta
    static constexpr float IDM_COOLNESS = 1.0f;            // c

    // MOBIL 파라미터
    static constexpr float MOBIL_SAFE_DECEL = 3.0f;    // b_safe
    static constexpr float MOBIL_POLITENESS = 0.3f;    // p
    static constexpr float MOBIL_THRESHOLD = 0.2f;     // a_thr
    static constexpr float MOBIL_EVAL_INTERVAL = 2.0f; // MOBIL 재평가 주기
    float m_lastLaneChangeTime = 0.0f;

    // 차량 주행 상태 변수 (Vehicle States)
    float m_speed = 0.0f;
    float m_acceleration = 0.0f;
    float m_steerAngle = 0.0f;
    bool m_isReverse = false;
    float m_deltaTime = 0.0f;
    bool m_wantSegmentTick = false;

    enum class AccelMode
    {
        None,
        Accelerating,
        Braking
    };
    AccelMode m_accelMode = AccelMode::None;

    // 스폰 및 리셋 데이터 (Spawn / Reset Data)
    DirectX::XMFLOAT3 m_spawnPosition = {0.0f, 0.0f, 0.0f};
    DirectX::XMFLOAT4 m_spawnRotation = {0.0f, 0.0f, 0.0f, 1.0f};

    // 트레일 및 디버그 렌더링 리소스 (Render Resources & Trail Data)
    static constexpr float TRAIL_SAMPLE_DISTANCE = 0.5f;
    static constexpr size_t TRAIL_MAX_POINTS = 2000;

    std::deque<DirectX::XMFLOAT3> m_rearTrail;
    std::deque<DirectX::XMFLOAT3> m_frontTrail;
    RenderObject m_rearTrailRender;
    RenderObject m_frontTrailRender;
    RenderObject m_steerLine;
    RenderObject m_targetMarker;
    RenderObject m_splineRender;
    RenderObject m_parkPathRender;   // Park 계획(RS 경로) 폴리라인
    RenderObject m_parkTargetMarker; // Park 목표 위치
    RenderObject m_parkTargetLine;   // Park 목표 방향
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