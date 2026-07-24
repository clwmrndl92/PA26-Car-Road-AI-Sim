#pragma once
#include "Core/GameObject.h"
#include "CarSpec.h"
#include "VehicleController.h"
#include <cmath>
#include <deque>
#include <functional>
#include <string>
#include <vector>
#include <Nav/RoadDataManager.h>
#include "Nav/ReedsShepp.h"
#include "Nav/VehicleCollision.h"
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
    void RebuildBBDebugRender(const std::vector<Vec3> &positions, const std::vector<Vec3> &directions,
                              const std::vector<VehicleCollision::Obstacle> &obstacles,
                              const VehicleCollision::VehicleShape &shape);
    // IsParkObstacleAhead의 부채꼴 레이 하나하나를 선으로 그린다(맞은 레이=빨강, 안 맞은 레이=초록).
    void RebuildParkRayDebugRender(const std::vector<Vec3> &rayStarts, const std::vector<Vec3> &rayEnds,
                                   const std::vector<bool> &hits);

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
    bool TryLaneChange(bool ignoreCooldown = false);

    // SimulateBBoxTrajectory 전방 sweep으로 정적+동적(주변 차량) 장애물을 검사해 최근접 충돌까지의 거리를
    // 반환한다(없으면 -1). DriveSpeedIDM이 이 값을 정지한 가상 리더로 취급. RebuildBBDebugRender로 디버그
    // 렌더링도 갱신하므로 const가 아니다.
    float ScanBBoxObstacleGap();
    void GetLookaheadPose(const Spline *startSpline, const shared_ptr<Lane> &startLane, size_t startPathIndex,
                          const Vec3 &fromPosition, float distance, Vec3 &outPosition, Vec3 &outDirection) const;
    float ComputeLookaheadDistance() const;
    void SimulateBBoxTrajectory(float lateralOffset, std::vector<Vec3> &outPositions,
                                std::vector<Vec3> &outDirections) const;

    void BeginParkPlan();
    void BeginParkSpotLeg();
    bool PlanParkLegTo(const Vec3 &targetPos, float targetAngleRad, bool exact = false);
    // RS 매뉴버(주차) 실행 중 매 틱 확인하는 짧은 부채꼴 레이캐스트 감지 -- ScanBBoxObstacleGap과 달리
    // 레인 스플라인을 따라가지 않고 지금 진행 방향(전진/후진)을 중심으로 좌우로 펼친 레이 여러 개를
    // 짧게 쏜다. 정적 장애물은 RS 플래닝이 이미 피해서 무시하고, 동적(주변 차량)만 감지 대상이다.
    // RebuildParkRayDebugRender로 디버그 렌더링도 갱신하므로 const가 아니다.
    bool IsParkObstacleAhead();
    bool PlanEnterForCurrentSpot();
    bool ReserveNextParkSpot();
    bool BeginParkEnterOrRetry();
    // startLane 안에서 lookaheadDistance만큼 못 걸으면 successor 레인으로 넘어가 계속 걷는다 -- 짧은 레인
    // 끝 근처(예: 출차 지점)에서 시작하면 결과가 레인 끝점에 눌러붙는 걸 막기 위함.
    void GetLaneLookaheadPoint(const shared_ptr<Lane> &startLane, const Vec3 &position, float lookaheadDistance,
                               Vec3 &outPosition, float &outAngleRad) const;

    VehicleCollision::VehicleShape BuildVehicleShape() const;

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
    // lane의 신호 때문에 지금 서야 하는지 (CheckPath의 레인 전환 가드와 공유하는 판단).
    bool ShouldStopForSignal(const shared_ptr<Lane> &lane) const;

    // 어떤 레인 위에서 찾은 다른 차 하나 + root 레인 기준 통일 거리(root 위치로부터의 부호 있는 거리, m).
    struct LaneNeighbor
    {
        Car *car = nullptr;
        float position = 0.0f;
    };
    // rootLane과 successor/predecessor로 연결된 차선(속도 기반 탐색 예산 거리, 합류 지점의
    // 곁가지 predecessor 포함)을 훑으며, 그 위의 모든 차에 대해 visitor(car, u)를 호출한다. u는 root
    // 기준 통일 거리(부호 있음, +면 앞) -- rootLane 자신 위의 차는 u = 그 차의 레인 arclength - rootPosition.
    void WalkConnectedLanes(const shared_ptr<Lane> &rootLane, float rootPosition,
                            const std::function<void(Car *, float)> &visitor) const;
    // rootLane 하나만 보는 게 아니라, WalkConnectedLanes로 연결된 차선까지 넓혀서 실제 앞차/뒷차를 찾는다.
    // outLeader/outFollower.position은 rootPosition 기준 상대 거리(u)로 채워진다(레인 로컬 arclength 아님).
    // 합류 지점에서 다른 레인 위의 차가 root 레인의 진행경로를 막는 경우까지 잡기 위함.
    void FindGraphNeighbors(const shared_ptr<Lane> &rootLane, float rootPosition,
                            LaneNeighbor &outLeader, LaneNeighbor &outFollower) const;
    Mobil::VehicleState ToVehicleState(const LaneNeighbor &n) const;
    // WalkConnectedLanes가 훑는 것과 같은 연결된 차선 집합에서 차량 목록만 모아 반환한다
    // (IsParkObstacleAhead가 동적 장애물 후보 풀로 재사용).
    std::vector<Car *> CollectNearbyCars(const shared_ptr<Lane> &rootLane, float rootPosition) const;
    // 레인 연결 여부와 무관하게, 전체 레인의 등록 차량 중 반경 안의 차만 모은다(자신 제외).
    // ScanBBoxObstacleGap의 bbox 동적 장애물 후보 풀 -- 차선변경 중 옆 레인 차처럼 레인 그래프로는
    // 안 잡히는 상대까지 잡기 위해 CollectNearbyCars 대신 이걸 쓴다.
    std::vector<Car *> CollectCarsWithinRadius(float radius) const;
    // lane 위에서 rootPosition(그 레인 자신의 누적 거리 기준) 앞쪽으로 MOBIL 탐색 예산 안에 있는 가장
    // 가까운 정적 장애물을 찾아 Mobil::VehicleState(정지 차량 취급)로 채운다. 레인 폭 절반 밖(=다른
    // 레인 소속)의 장애물은 걸러낸다. 없으면 false.
    bool FindNearestObstacleOnLane(const shared_ptr<Lane> &lane, float rootPosition, Mobil::VehicleState &out) const;
    // carLeader(실제 앞차)와 그 레인 위 정적 장애물 중 더 가까운 쪽을 MOBIL 리더로 반환한다. 둘 다
    // 없으면 laneLimitSpeed로 달리는 가상 리더(뚫린 도로).
    Mobil::VehicleState BuildLeaderState(const shared_ptr<Lane> &lane, float rootPosition, const LaneNeighbor &carLeader,
                                         float laneLimitSpeed) const;

public:
    // 차선 진입 허용 오차/임계값 (예: 현재 타깃 차선에 안착했는지 확인하는 기준)
    static constexpr float LANE_ENTRY_THRESHOLD = 3.0f;
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

    float m_avoidLateralOffset = 0.0f;                                      // 회피용 좌우 오프셋(+우/-좌, m). DriveControl이 조준점에 더한다. (예정된 Reynolds 회피가 채울 자리 -- 현재는 항상 0)
    float m_obstacleAheadGap = -1.0f;                                       // ScanBBoxObstacleGap이 찾은, 경로 폭 안 최근접 장애물까지의 범퍼 대 범퍼 거리(m). 없으면 -1. DriveSpeedIDM이 가상 리더 gap으로 사용.
    float m_obstacleAheadSpeed = 0.0f;                                      // 위 장애물의 속도를 내 진행방향에 투영한 성분(m/s). 정적 장애물이면 0. m_obstacleAheadGap이 -1이면 의미 없음.
    std::string m_lastBrakeCause;                                           // DriveSpeedIDM 제동 로그 중복 방지용(비어있으면 비제동 상태).
    static constexpr float AVOID_DETECT_DISTANCE = 20.0f;                   // 박스캐스트 바운딩박스 스윕 길이 (전방 감지 거리)
    static constexpr float AVOID_SAMPLE_STEP = 2.0f;                        // 바운딩박스 스윕을 따라 박스를 검사하는 간격
    // CollectCarsWithinRadius 검색 반경 -- bbox 스윕이 실제로 닿는 범위(AVOID_DETECT_DISTANCE)보다
    // 여유를 둔다(후보만 좁히는 용도라 정밀할 필요 없음, 실제 충돌 여부는 OBB 검사가 가른다).
    static constexpr float NEARBY_CAR_SEARCH_RADIUS = AVOID_DETECT_DISTANCE + 10.0f;
    static constexpr float PARK_OBSTACLE_DETECT_DISTANCE = 5.0f;            // IsParkObstacleAhead 레이 길이 -- RS 저속 매뉴버라 짧게만 봐도 충분
    static constexpr float PARK_OBSTACLE_FAN_HALF_ANGLE = ToRadians(35.0f); // IsParkObstacleAhead 부채꼴의 좌우 반각(중앙 기준)
    static constexpr int PARK_OBSTACLE_FAN_RAY_COUNT = 5;                   // IsParkObstacleAhead 부채꼴 레이 개수(홀수면 정중앙 레이 포함)
    static constexpr float AVOID_OBSTACLE_STANDSTILL_DISTANCE = 1.0f;       // 장애물 가상 리더용 s0 -- 일반 차량 추종(IDM_STANDSTILL_DISTANCE=2m)보다 더 붙어도 되게 별도로 둔다.

    // bbox 장애물로 인한 데드락 강제 탈출용: 정지 상태(m_speed < STALL_SPEED_THRESHOLD)로
    // OBSTACLE_STALL_TIMEOUT초 이상 묶여 있으면, FORCED_ESCAPE_DURATION초 동안 bbox 장애물 감지를
    // 무시하고 진행해 서로 양보만 하는 맞물림 상태를 풀어준다.
    static constexpr float STALL_SPEED_THRESHOLD = 0.3f;
    static constexpr float OBSTACLE_STALL_TIMEOUT = 5.0f;
    static constexpr float FORCED_ESCAPE_DURATION = 3.0f;
    float m_obstacleStallTime = 0.0f; // bbox 장애물에 막혀 정지 상태로 있은 누적 시간
    float m_forcedEscapeTimer = 0.0f; // > 0이면 강제 탈출 중(bbox 장애물 무시), 매 틱 감소

    vector<LaneStep> m_path;
    size_t m_pathIndex = 0;
    static constexpr float LOOK_PROFILE_TIME = 5.0f;
    static constexpr size_t SPEED_PROFILE_COUNT = 10; // 재스캔 주기(LOOK_PROFILE_TIME / 이 값) 계산에만 씀
    std::vector<RoadSpeedSample> m_roadConstraints;
    float m_lastConstraintScanTime = 0.0f;
    Spline m_currentSpline;
    float m_currentTime = 0.0f;
    // 노란불 때 "정지거리 안쪽이라 통과" 확정한 신호 id(초록될 때까지 유지). 없으면 -1.
    mutable int m_committedYellowNodeId = -1;

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
    bool m_hadObstacleAhead = false; // 직전 틱에 현재 레인에 장애물 리더가 있었는지 -- 새로 나타나면 TryLaneChange가 쿨다운을 건너뛰고 즉시 재평가.

    // FindGraphNeighbors가 successor/predecessor를 타고 넘어가는 탐색 예산(레인 그래프 누적 거리, m).
    // 속도가 빠를수록 더 멀리 봐야 하므로 speed*time 기반으로 계산하고(WalkConnectedLanes), 저속/정지
    // 시에도 최소한은 보게 바닥값을 둔다.
    static constexpr float MOBIL_SEARCH_FORWARD_TIME = 2.0f;  // 정방향(successor) 탐색 예산 = 속도 * 이 시간(s)
    static constexpr float MOBIL_SEARCH_FORWARD_MIN = 30.0f;  // 정방향 탐색 예산의 최소값(m)
    static constexpr float MOBIL_SEARCH_BACKWARD_TIME = 0.6f; // root 레인 자신의 역방향(predecessor) 예산 = 속도 * 이 시간(s)
    static constexpr float MOBIL_SEARCH_BACKWARD_MIN = 10.0f; // 역방향 탐색 예산의 최소값(m)

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
    std::vector<RenderObject> m_bboxDebugRenders; // ScanBBoxObstacleGap/IsParkObstacleAhead 바운딩박스 샘플 박스(충돌=빨강/통과=초록) -- 둘이 공유하므로 나중에 호출된 쪽 것으로 매번 덮어써진다
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