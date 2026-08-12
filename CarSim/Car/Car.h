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
#include "Nav/IDM.h"
#include "Nav/Mobil.h"

class SimulationState;
class Spline;
class VehicleSegment;

class Car : public GameObject
{
public:
    void Init(const CarSpec &spec, const CarPersonality &personality, SimulationState *simState, JPH::Vec3 position = JPH::Vec3::sZero());

    void UpdatePhysics(float dt) override;
    void Update(float dt) override;
    void UpdateUI(float dt) override;
    void Draw(ID3D11DeviceContext *context, IEffect &effect) override;
    void Destroy() override;

    Vec3 GetPosition() const override; // 앞차축 기준
    Vec3 GetRigidbodyPosition() const { return m_rigidbody.GetPosition(); }
    Vec3 GetForwardAxis() const;
    float GetDeltaTime() const { return m_deltaTime; }
    float GetSpeed() const { return m_speed; }
    float GetSteerAngle() const { return m_steerAngle; }
    float GetWheelbase() const { return m_wheelbase; }

    void SetPosition(Vec3 position) override;
    void SetRotation(Vec3 direction);
    void SetFocused(bool focused)
    {
        m_isFocused = focused;
        m_drawCollider = focused;
    }
    void SetHighlighted(bool on) { m_highlighted = on; }
    void SetMobilHighlighted(bool on) { m_mobilHighlighted = on; }
    Car *GetIdmLeader() const; // 죽은 차면 nullptr
    void GetLaneChangeNeighbors(Car *&outLeader, Car *&outFollower) const;
    void SetDestination(const shared_ptr<RoadNode> &parkNode);
    void SetRoaming(bool roaming) { m_roaming = roaming; }
    void SetId(int id) { m_id = id; }
    int GetId() const { return m_id; }
    void SetControl(bool isControl) { m_isControl = isControl; }
    bool IsUserControl() const { return m_isControl; }
    float GetAcceleration() const { return m_acceleration; }
    float GetLength() const { return m_halfExtents.GetZ() * 2.0f; }
    float GetHalfWidth() const { return m_halfExtents.GetX(); }
    // 앞축→범퍼 거리
    float FrontOverhang() const { return m_colliderOffset.z + m_halfExtents.GetZ() - m_wheelbase; }
    float RearOverhang() const { return m_wheelbase - m_colliderOffset.z + m_halfExtents.GetZ(); }

    void AccelerateVel(float desiredVelocity);
    void Accelerate(float desiredAccel);
    void EmergBrake();
    void Steer(float desiredRadian, float steerRamp = 1.0f);
    void ChangeGear();
    bool IsReverse() const { return m_isReverse; }

    void DriveControl();
    float PurePursuit(Vec3 target);
    float Stanley(const Spline &spline);

private:
    void UpdateCar();
    void UpdateWithControl();
    void ApplyMotion();
    float GetSignedSpeed() const { return m_speed * (m_isReverse ? -1.0f : 1.0f); }
    JPH::Vec3 ComputeDesiredVelocity() const;

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
        D_Avoid,      // 오프셋 회피주행
        D_LaneChange, // MOBIL 차선변경

        // Park
        P_EXIT,        // 출차
        P_ENTER_LEG1,  // 입차1: P점까지
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
        case SubMode::D_Avoid:
            return "Avoid";
        case SubMode::D_LaneChange:
            return "LaneChange";
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
    void OnModeExit(Mode next);
    void SetSubMode(SubMode next);
    void BeginSegment(std::unique_ptr<VehicleSegment> segment);

    VehicleCollision::VehicleShape BuildVehicleShape() const;
    void UpdateFindPath();
    void SetCurrentRoad(const shared_ptr<Road> &road, float offset, LaneDirection direction);
    void SetCurrentOffset(float offset); // 오프셋+밴드 동기화

    float TravelSign() const { return GetTravelSign(m_travelDir); }
    RoadRef CurrentRoadRef() const { return RoadRef{m_currentRoad, m_travelDir}; }
    bool ShouldStopForSignal(const shared_ptr<Road> &road, LaneDirection direction, int nextRoadId = -1) const;
    bool TryFindPathAndSetRoad();

    void EnsureRoamingPath();
    void MaintainRoamingPath();
    RoadRef PickRandomSuccessor(const RoadRef &road) const;
    vector<RoadRef> BuildRoamingPath(const RoadRef &startRoad) const;

    void UpdatePark();
    bool IsArrivedAtDest() const;
    void TryBeginParkCruise();
    bool BeginParkCruise();
    bool BeginParkExitCruise(int lotNodeId);
    void BeginParkPlan();
    void BeginParkSpotLeg();
    bool PlanParkLegTo(const Vec3 &targetPos, float targetAngleRad, bool exact = false);
    shared_ptr<RoadNode> GetParkTargetNode() const { return m_parkSpot ? m_parkSpot : m_pendingParkNode; }

    bool PlanEnterForCurrentSpot();
    const Spline *FindBestParkingSpline(LaneDirection *outDirection = nullptr) const;
    bool ComputeParkPrePose(Vec3 &outPos, float &outAngleRad) const;
    bool ReserveNextParkSpot();
    bool BeginParkEnterOrRetry();

    void UpdateStop();

    void UpdateDrive();
    bool CheckPath();
    bool CanEnterFromCurrentBand(const RoadRef &next) const;
    bool RepathFromCurrentBand();

    void UpdateHorn(float dt);
    bool IsHornSituation() const;
    bool KnowsRedSignalAhead() const;
    bool IsEmergencyRayBlocked() const;

#pragma endregion

#pragma region BehaviorPlan
    // 분기용 태그만
    enum class RoadSampleKind
    {
        Other,
        Signal,
        JunctionWait,
    };
    struct RoadSpeedSample
    {
        Vec3 position;
        float distance;
        float speed;
        Car *leader = nullptr;
        const char *debugStr = ""; // 표시전용, 분기는 kind
        RoadSampleKind kind = RoadSampleKind::Other;
        Vec3 leaderScanPosition = Vec3::sZero();
        VehicleCollision::Obstacle obstacle; // 히트때만 유효
        bool hasObstacle = false;
    };

    struct SpeedLimitDebug
    {
        std::string label = "free";
        float targetSpeed = 0.0f;
        float gap = 0.0f;
    };

    // MOBIL 판정용
    struct LaneNeighbors
    {
        bool hasLeader = false;
        bool hasFollower = false;
        Mobil::VehicleState leader;
        Mobil::VehicleState follower;
        Car *leaderCar = nullptr;
        Car *followerCar = nullptr;
    };

    void UpdateDrivePlan();
    IDM::Params BuildIdmParams(const shared_ptr<Road> &road) const; // v0=도로 제한속도
    float ComputeIdmAcceleration(const std::vector<RoadSpeedSample> &samples, const IDM::Params &params,
                                 float distanceOffset, float elapsedTime,
                                 const VehicleCollision::Obstacle **outLeader = nullptr,
                                 SpeedLimitDebug *outDebug = nullptr) const;
    bool IsSafeLaneEntry(const RoadRef &road, float targetOffset, bool checkLeader = true) const;
    bool ShouldHoldForMerge(const RoadRef &nextRoad) const;
    bool ShouldHoldForJunction(const RoadRef &nextRoad) const;
    bool TryReserveJunction(const RoadRef &nextRoad);
    void ReleaseJunctionReservation();
    LaneNeighbors GatherLaneNeighbors(const shared_ptr<Road> &road, const Spline &refLine, float bandCenter,
                                      float bandHalfWidth, float egoS, float dirSign) const;
    float CurrentLaneCenter() const;

    struct RouteLaneGoal
    {
        bool active = false;       // 차로 제약 여부
        float targetOffset = 0.0f; // 가장 가까운 밴드중심
        float urgency = 0.0f;      // 교차로 임박도 0~1
        float bias = 0.0f;         // urgency를 m/s^2로
    };
    RouteLaneGoal ComputeRouteLaneGoal() const;
    float ComputeLateralTarget(const IDM::Params &idm,
                               float *outLaneCenter = nullptr, const char **outReason = nullptr) const;
    std::vector<RoadSpeedSample> ScanRoadSpeedConstraints(float lookDistance) const;
    void AppendSensorConstraintSample(std::vector<RoadSpeedSample> &samples) const;
    void ComputeDrivableRange(const RoadRef &road, float &outMin, float &outMax) const;
#pragma endregion

#pragma region Avoid
    // 두 서브모드 공통 상태
    struct ManeuverState
    {
        float laneOffset = 0.0f;       // 시작시점 차로중심
        float avoidOffset = 0.0f;      // D_Avoid 목표 d
        float laneChangeTarget = 0.0f; // 차선변경 목표 d
        float clearTimer = 0.0f;       // 회피종료 디바운스
        float lastPlanTime = -1000.0f; // 재계획 디더링 방지
    };

    // 위협 종류 한번만 분기
    enum class ThreatKind
    {
        None,    // 위협 없음 -- IDM 추종에 맡긴다
        Vehicle, // 차 -- IDM(종방향)이 이미 처리한다
        Static,  // 멈춰 있는 장애물 -- 오프셋 회피
    };

    void UpdateSensors();                   // 장애물 목록 수집
    bool HandleContactPending();            // 충돌 뒷수습, 최우선
    void DecideAvoidance();                 // 위협분류→서브모드
    void UpdateAvoid();                     // D_Avoid: 오프셋 회피 재계획/복귀/종료
    void UpdateLaneChange();                // 차선변경 진행/취소/완료
    void HandleAvoidStuck();                // 회피 불가: 그 자리에 정지 유지
    ThreatKind ClassifyFrontThreat() const; // 전방 최근접 히트를 차/정적으로 분류
    bool IsOnLane(float offset) const;      // 지금 밴드 폭 안에 있나
    // 현재 목표 횡오프셋
    float AvoidTargetOffset() const;
    Vec3 GetBodyCenter() const;                             // OBB 판정 기준점
    VehicleCollision::Obstacle MakeVehicleObstacle() const; // 이 차의 차체 OBB를 장애물 하나로
    // 무충돌 후보 오프셋 탐색
    bool FindAvoidOffset(float laneCenter, float &outOffset) const;
    // 첫 충돌까지 거리(스윕)
    float SweepBodyPath(float targetOffset, const std::vector<VehicleCollision::Obstacle> &obstacles,
                        float speed, float maxDistance) const;
    // 예측시간 내 무충돌?
    bool SimulateAvoidPath(float targetOffset, const std::vector<VehicleCollision::Obstacle> &obstacles) const;
    void RebuildSensorRender();
    void RebuildEmergRayRender();
#pragma endregion

public:
    static constexpr float ARRIVE_DISTANCE = 5.0f;
    static constexpr float PARK_ARRIVE_DISTANCE = 10.0f;
    static constexpr float STOPPED_SPEED = 0.01f;

private:
    const float m_maxSpeed = 200.0f / 3.6f;           // 200 km/h
    const float m_maxAccel = (100.0f / 3.6f) / 14.0f; // 0-100 km/h in 14s
    const float m_maxBrake = (100.0f / 3.6f) / 3.0f;  // 100-0 km/h in 3s
    float m_speedGain = 2.0f;                         // 속도오차 -> 목표가속 비례게인
    float m_jerkUp = 4.0f;                            // 가속 저크 상한
    float m_jerkDown = 15.0f;                         // 제동 저크 상한
    CarPersonality m_personality;                     // IDM 파라미터에 반영

    float m_wheelbase = 0.0f;
    float m_mass = 1.0f;
    Vec3 m_halfExtents = Vec3::sZero();       // x=반폭 z=반길이
    float m_maxSteerAngle = ToRadians(45.0f); // 최대 조향각 (45도)
    float m_stanleyGain = 1.0f;               // Stanley 횡오차 게인 k
    float m_stanleySoft = 1.0f;               // 저속 발산 방지

    float m_speed = 0.0f;
    float m_acceleration = 0.0f;
    float m_steerAngle = 0.0f;
    bool m_isReverse = false;
    float m_deltaTime = 0.0f;

    bool m_wantSegmentTick = false;
    bool m_isFocused = false;
    bool m_highlighted = false;      // IDM 리더 표시
    bool m_mobilHighlighted = false; // MOBIL 앞/뒤차 표시
    bool m_isControl = false;        // true면 수동조작
    int m_id = -1;

    SimulationState *m_SimState = nullptr;
    Mode m_mode = Mode::Stop;
    SubMode m_subMode = SubMode::D_Normal;
    VehicleController m_vehicleController; // 계획 세그먼트 실행
    shared_ptr<Road> m_destRoad;
    LaneDirection m_destDir = LaneDirection::Forward; // 도착판정 방향
    shared_ptr<Road> m_currentRoad;
    LaneDirection m_travelDir = LaneDirection::Forward; // d는 항상 참조선 기준
    float m_currentOffset = 0.0f;                       // 실측아닌 계획값
    const LaneBand *m_currentBand = nullptr;            // SetCurrentOffset이 갱신
    Spline m_currentSpline;                             // 오프셋 d 스플라인 캐시

    shared_ptr<RoadNode> m_parkSpot;        // 예약된 목표 주차칸
    shared_ptr<RoadNode> m_pendingParkNode; // 예약전 목표 Park노드
    bool m_parkSequenceActive = false;
    int m_parkNodeId = -1;                 // 재예약용 Park id
    int m_parkLegTries = 0;                // 무한체인 방지 카운트
    unordered_set<int> m_triedParkSpotIds; // 실패한 스팟 id들
    bool m_parkPlanPending = false;
    bool m_lotCruise = false;          // 스팟 앞까지 주행중
    Vec3 m_lotStopPos = Vec3::sZero(); // 통로주행 목표 P점
    // Park 직행 방지용
    bool m_lotEntryHold = false;
    // leg1 목표 pose
    static constexpr float PARK_PRE_LEAD_DISTANCE = 3.0f;

    bool m_roaming = true;                         // 목적지 없이 배회
    static constexpr size_t ROAMING_MIN_AHEAD = 3; // 배회 최소 버퍼 수

    vector<RoadRef> m_path;
    size_t m_pathIndex = 0;
    int m_reservedJunctionId = -1; // 교차로 통과중 점유
    float m_currentTime = 0.0f;
    // 통과확정 노랑신호 id
    mutable int m_committedYellowNodeId = -1;

    static constexpr float HORN_STOP_SPEED = 0.3f; // 정지 기준 속도(m/s)
    float m_hornStoppedDuration = 0.0f;            // 막혀서 정지해 있던 누적 시간
    float m_hornFlashTimer = 0.0f;                 // 남은 빨간표시 시간
    Model *m_carModel = nullptr;                   // 경적시 색 변경용

    static constexpr float BEHAVIOR_PLAN_INTERVAL = 0.2f; // 행동계획 주기
    static constexpr float SAFE_GAP = 2.0f;               // 표준 범퍼 gap, IDM s0
    static constexpr float MIN_SAFE_GAP = 0.3f;           // 진짜 충돌 방지 갭

    // IDM/MOBIL 파라미터
    static constexpr float MOBIL_B_SAFE = 3.0f; // 최대 안전감속도
    static constexpr float MOBIL_A_THR = 1.0f;  // 차선변경 최소 진입 장벽(m/s^2)
    // 겹치면 후보 제외
    static constexpr float MOBIL_LEADER_ALIGN_GAP = 5.0f;
    static constexpr float MOBIL_LANE_CHANGE_COOLDOWN = 10.0f; // 변경후 재평가 금지

    float m_lastBehaviorPlanTime = -1000.0f;                     // 첫 판단 즉시 실행되게
    float m_lastLaneChangeTime = -1000.0f;                       // 쿨다운 게이팅용
    float m_planAccelDebug = 0.0f;                               // 매프레임 IDM 가속도
    SpeedLimitDebug m_limitDebug;                                // 가속도 결정 근거
    const VehicleCollision::Obstacle *m_currentLeader = nullptr; // IDM이 고른 앞 장애물
    std::vector<VehicleCollision::Obstacle> m_obstacles;
    std::vector<Car *> m_nearbyCars;                // 재수집 비용 회피 캐시
    std::vector<RoadSpeedSample> m_lastRoadSamples; // 매프레임 IDM 재계산용
    IDM::Params m_lastIdmParams;                    // 스캔시점 IDM 캐시
    Vec3 m_planScanPosition = Vec3::sZero();        // gap 보정 기준위치
    mutable std::vector<Vec3> m_sweepDebugCorners;
    mutable std::vector<Vec3> m_emergRayDebugLines;
    mutable bool m_emergRayDebugBlocked = false;

    // 매프레임 스윕 갱신
    static constexpr float AVOID_LOW_SPEED = 18.26f / 3.6f;
    static constexpr float AVOID_CLEAR_DELAY = 0.5f;      // 복귀 디바운스(s)
    static constexpr float AVOID_RETURN_TOLERANCE = 0.3f; // 회피종료 판정거리
    static constexpr float AVOID_MIN_SHIFT = 0.5f;        // 최소 유효 횡이동
    static constexpr float AVOID_SIM_TIME = 3.0f;         // 스윕 예측시간(s)
    static constexpr float AVOID_SIM_MIN_SPEED = 3.0f;    // 스윕 최소속도(m/s)

    // 물리/판단 틱 다름
    bool m_contactPending = false;

    float m_staticBlockTimer = 0.0f; // 정적장애물 막힘 시간
    bool m_stuck = false;            // 못피해 정지
    ManeuverState m_maneuver;
    float m_speedCap = -1.0f;

    DirectX::XMFLOAT3 m_spawnPosition = {0.0f, 0.0f, 0.0f};
    DirectX::XMFLOAT4 m_spawnRotation = {0.0f, 0.0f, 0.0f, 1.0f};

    std::deque<DirectX::XMFLOAT3> m_rearTrail;
    std::deque<DirectX::XMFLOAT3> m_frontTrail;
    RenderObject m_rearTrailRender;
    RenderObject m_frontTrailRender;
    RenderObject m_debugBox;
    bool m_drawCollider = false;
    RenderObject m_steerLine;
    RenderObject m_targetMarker;
    RenderObject m_splineRender;
    RenderObject m_sensorRender;     // 스윕박스 윤곽선(디버그)
    RenderObject m_emergRayRender;   // 긴급제동 레이(디버그)
    RenderObject m_parkPathRender;   // Park 계획(RS 경로) 폴리라인
    RenderObject m_parkTargetMarker; // Park 목표 위치
    RenderObject m_parkTargetLine;   // Park 목표 방향
};

static float CalcMaxSteerAngle(float speed)
{
    constexpr float LOW_SPEED_CUTOFF = 18.26f / 3.6f;
    constexpr float MAX_STEER_ANGLE = ToRadians(45.0f);
    return (speed <= LOW_SPEED_CUTOFF) ? MAX_STEER_ANGLE : 20.2f / (speed * speed);
}
static float CalcMaxSpeed(float targetAngle)
{
    constexpr float MAX_STEER_ANGLE = ToRadians(45.0f);
    targetAngle = std::clamp(std::abs(targetAngle), 0.0005f, MAX_STEER_ANGLE);
    return std::sqrt(20.2f / targetAngle);
}
