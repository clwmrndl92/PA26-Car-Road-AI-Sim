#pragma once
#include "Core/GameObject.h"
#include "CarSpec.h"
#include "RaceLine.h"
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

class Car;
class SimulationState;
class Spline;
class VehicleSegment;

// 레이싱 라인 위로 투영해 찾은 "같은 라인 앞차". 종방향 갭 유지의 기준점이자,
// 추월 판단(오버랩 확보 여부, 기회 탐색)이 얹히는 자리다.
struct LeaderInfo
{
    Car *car = nullptr;
    float gap = 0.0f;             // 범퍼 사이 종방향 거리(m)
    float speed = 0.0f;           // 앞차 속도(m/s)
    float closingSpeed = 0.0f;    // 내 속도 - 앞차 속도(m/s). 양수면 접근 중
    float lateralOffset = 0.0f;   // 레이싱 라인 기준 앞차 횡오프셋(m)
    float myLateralOffset = 0.0f; // 같은 기준의 내 횡오프셋(m)
    bool IsValid() const { return car != nullptr; }
};

// 프레네 투영 한 패스로 함께 뽑는 앞차 정보. leader와 슬립스트림은 조건이 다르다 --
// leader는 횡 오버랩이 필수지만, 슬립스트림은 옆으로 빠질수록 서서히 약해질 뿐이라
// 추월하려고 라인을 벗어나는 순간 와류가 뚝 끊기면 안 된다.
struct RaceNeighbors
{
    LeaderInfo leader;
    bool selfProjected = false; // leader 유무와 무관하게 내 (s, d) 투영이 유효한가
    float slipstream = 0.0f;    // 0=클린에어 ~ 1=앞차 바로 뒤
    float slipstreamGap = 0.0f; // 와류를 준 차와의 갭(m). 디버그용

    // 추월 대상은 앞/뒤 관계없이 따로 추적한다. 옆으로 나가면 leader 조건(횡 오버랩)에서
    // 빠지는데, 그때도 "앞질렀는가"를 판정하려면 상대 위치를 계속 알아야 한다.
    bool targetSeen = false;
    float targetDeltaS = 0.0f;  // 양수면 아직 앞에 있다
    float targetLateral = 0.0f; // 내 기준 상대 횡오프셋
    float targetSpeed = 0.0f;   // 물러날 때 이 속도 밑으로 내려가야 겹침이 풀린다
    // 추월 대상과 나란히 서려면 실제로 필요한 횡간격(요각 반영). Setup 통과 판정과
    // Attack 중 재검사가 같은 기준을 쓰도록 여기서 한 번만 구해 둔다.
    float targetRequiredLateral = 0.0f;

    // 나를 추월 중인 뒤차. 뒤차가 절반 길이 이상 들어오면 공간을 받을 권리를 얻고,
    // 앞차는 이 정보를 잠가 추월이 끝날 때까지 해당 쪽 라인을 침범하지 않는다.
    Car *yieldChallenger = nullptr;
    float yieldChallengerSide = 0.0f;
    float yieldChallengerOverlap = 0.0f;
    float yieldChallengerLateral = 0.0f;

    // 이미 권리를 준 차는 추월 FSM이 Return으로 바뀐 뒤에도 종방향 겹침이 풀릴 때까지 본다.
    bool yieldTargetSeen = false;
    float yieldTargetDeltaS = 0.0f;
    float yieldTargetLateral = 0.0f;

    // 종방향으로 곧 겹칠 수 있는 주변 차량들이 허용하는 내 중심의 횡오프셋 범위.
    // 전술이 어떤 값을 만들든 마지막에 이 구간으로 잘라 차량 쪽으로 이동하지 않게 한다.
    float safeOffsetMin = -std::numeric_limits<float>::max();
    float safeOffsetMax = std::numeric_limits<float>::max();
};

class Car : public GameObject
{
public:
    // 코너에서 에이펙스를 어디에 찍을지
    enum class ApexStrategy
    {
        Early, // 일찍 -- 진입 빠르고 탈출 좁음
        Apex,  // 곡률 정점 그대로
        Late,  // 늦게 -- 탈출 넓고 가속 유리
        Count,
    };

    void Init(const CarSpec &spec, const CarPersonality &personality, SimulationState *simState, JPH::Vec3 position = JPH::Vec3::sZero(),
              float aiStartDelay = 0.0f);

#pragma region Race
    // 트랙 전체 레이싱 라인을 전략별로 한 번 풀어 SimulationState에 넣는다.
    // 도로를 다 읽은 뒤, 레이스를 시작하기 전에 한 번만 부를 것(전략당 수백 ms).
    static void BuildSharedRaceLines(SimulationState &simState);
    // 차 인스턴스 없이도 레이싱 라인을 그릴 수 있도록 렌더 오브젝트만 따로 뽑아내는 헬퍼.
    static void BuildRaceLineRenderObject(const RaceLine &line, const std::string &modelKey,
                                          RenderObject &out,
                                          const DirectX::XMFLOAT4 &color = DirectX::XMFLOAT4(0.0f, 0.3f, 1.0f, 1.0f));
    // 공유 라인을 만들 때 쓴 횡방향 여유(차선 반폭 - 최대 차폭 - 여유).
    // 추월 오프셋 클램프도 같은 값을 써야 라인이 도로 밖으로 안 나간다.
    static float SharedLineRoom();

    // 레이스 모드 on/off. 켜는 쪽은 공유 라인이 이미 만들어져 있어야 한다.
    void SetRaceMode(bool on);
    bool IsRaceMode() const { return m_raceMode; }
    void SetApexStrategy(ApexStrategy strategy) { m_apexStrategy = strategy; }
    ApexStrategy GetApexStrategy() const { return m_apexStrategy; }

    // 순위 기준값. 랩 * 랩길이 + 현재 s라 랩을 먹은 차가 자연히 앞선다.
    float GetRaceProgress() const { return m_lap * m_lapLength + m_raceS; }
    int GetLap() const { return m_lap; }
    float GetLastLapTime() const { return m_lastLapTime; }
    float GetBestLapTime() const { return m_bestLapTime; }
    int GetRacePosition() const { return m_racePosition; }
    int GetOvertakes() const { return m_overtakes; }
    int GetOvertakenBy() const { return m_overtakenBy; }
    // 순위가 잠깐 뒤집혔다 돌아오는 걸 추월로 세지 않도록 일정 시간 유지될 때만 확정한다.
    void UpdateRacePosition(int position, float now);
#pragma endregion

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
    void SetSpeed(float speed) { m_speed = std::clamp(speed, 0.0f, m_maxSpeed); } // 가속/저크 안 거치고 즉시 세팅
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
    void SetSirenOn(bool on);
    bool IsSirenOn() const { return m_sirenOn; }
    void SetChaseOn(bool on);
    bool IsChaseOn() const { return m_chaseOn; }
    void SetFleeOn(bool on);
    bool IsFleeOn() const { return m_fleeOn; }
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
    void FullAccelerateVel(float desiredVelocity); // 저크램프 생략
    void FullAccelerate(float desiredAccel);
    void EmergBrake();
    void Steer(float desiredRadian, float steerRamp = 1.0f);
    void ChangeGear();
    bool IsReverse() const { return m_isReverse; }
    bool IsReverseEscaping() const { return m_reverseEscapeTimer > 0.0f; }

    void DriveControl();
    float PurePursuit(Vec3 target);
    float Stanley(const Spline &spline);

private:
    bool IsAiDelayed() const { return !m_isControl && m_aiDelayTimer < m_aiStartDelay; }

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
        Drive,
        Race // 레이싱 라인 주행 + 추월. 신호/차선 규칙 대신 트랙 규칙을 따른다
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
        case Mode::Race:
            return "Race";
        }
        return "?";
    }

    // 추월은 몇 초에 걸친 의도다. 매 프레임 반응형 조향으로는 유지되지 않아서 상태로 들고 간다 --
    // 이게 없으면 옆으로 나갔다가 곧바로 라인에 끌려 돌아오길 반복할 뿐 추월이 성립하지 않는다.
    enum class OvertakeState
    {
        None,   // 대상 없음 -- 레이싱 라인
        Follow, // 뒤에 붙어 기회 대기(토우 안)
        Setup,  // 옆으로 나가는 중. 다 나가기 전에는 붙지 않는다
        Attack, // 실제로 벌어진 걸 확인하고 나란히 서서 지나간다
        Return, // 성공/포기 -- 라인으로 복귀
    };
    const char *OvertakeStateToString(OvertakeState state) const
    {
        switch (state)
        {
        case OvertakeState::None:
            return "None";
        case OvertakeState::Follow:
            return "Follow";
        case OvertakeState::Setup:
            return "Setup";
        case OvertakeState::Attack:
            return "Attack";
        case OvertakeState::Return:
            return "Return";
        }
        return "?";
    }

    enum class SubMode
    {
        None,

        // Drive
        D_Normal,     // 일반 주행
        D_Pursuit,    // 추격/도망 반응형 주행
        D_Avoid,      // 오프셋 회피주행
        D_LaneChange, // MOBIL 차선변경
        D_SirenWait,  // 사이렌차 양보정차
        D_ChaseBlock, // 도망차 앞 차단정차

        // Park
        P_Exit,        // 출차
        P_Enter_Prep,  // 입차1: P점까지
        P_Enter,       // 입차: P -> 스팟
        P_Enter_Align, // 입차: 최종 정밀 정렬

        // Race
        R_Racing, // 레이싱 라인 추종(추월 상태는 OvertakeState가 따로 든다)
    };
    const char *Car::SubStateToString(SubMode subMode) const
    {
        switch (subMode)
        {
        case SubMode::None:
            return "None";
        case SubMode::D_Normal:
            return "Normal";
        case SubMode::D_Pursuit:
            return "Pursuit";
        case SubMode::D_Avoid:
            return "Avoid";
        case SubMode::D_LaneChange:
            return "LaneChange";
        case SubMode::D_SirenWait:
            return "SirenWait";
        case SubMode::D_ChaseBlock:
            return "ChaseBlock";
        case SubMode::P_Exit:
            return "ParkExit";
        case SubMode::P_Enter_Prep:
            return "ParkEnterLeg1";
        case SubMode::P_Enter:
            return "ParkEnterLeg2";
        case SubMode::P_Enter_Align:
            return "ParkEnterAlign";
        case SubMode::R_Racing:
            return "Racing";
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
    bool IsChasing() const { return m_chaseOn && m_sirenOn; } // 대상 잡고 사이렌 켠 상태
    // 추격/도망은 신호·교차로 대기 무시. 레이스도 마찬가지로 트랙 규칙만 따른다.
    // 모드가 아니라 플래그를 보는 이유: 레이스를 켠 첫 틱에는 아직 Stop이라, 모드로
    // 판정하면 도로 진입(EnsureRoamingPath)이 MOBIL 안전검사에 걸려 출발을 못 한다.
    bool IgnoresTrafficRules() const { return IsChasing() || m_fleeOn || m_raceMode; }
    bool UsesReactiveSteer() const { return m_subMode == SubMode::D_Pursuit; }
    // 기동이 끝났을 때 돌아갈 주행 상태
    SubMode BaseDriveSubMode() const
    {
        return IgnoresTrafficRules() ? SubMode::D_Pursuit : SubMode::D_Normal;
    }
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
    bool IsChaseCarObstacle(const VehicleCollision::Obstacle &obstacle) const; // 추격차 차체인가

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
    float RoadTargetSpeed(const shared_ptr<Road> &road) const;      // 무시상태면 차량 최고속
    bool IgnoresSpeedLimit(const shared_ptr<Road> &road) const;     // 직선길에서만 무시
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
    // 추격/도망은 역주행 차로도 포함
    std::vector<const LaneBand *> GatherCandidateBands(const RoadRef &road) const;

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
        // 회피 유발 장애물, 지나칠 때까지 유지
        VehicleCollision::Obstacle target;
        bool hasTarget = false;
    };

    // 위협 종류 한번만 분기
    enum class ThreatKind
    {
        None,    // 위협 없음 -- IDM 추종에 맡긴다
        Vehicle, // 차 -- IDM(종방향)이 이미 처리한다
        Static,  // 멈춰 있는 장애물 -- 오프셋 회피
    };

    void ApplySpecialPersonality();                                   // 사이렌/추격/도망 성격 전환
    void UpdateSensors();                                             // 장애물 목록 수집
    bool HandleContactPending();                                      // 충돌 뒷수습, 최우선
    Car *FindContactCar(JPH::BodyID otherId) const;                   // 부딪힌 BodyID -> Car
    bool IsHeadOnContact(const Car *other) const;                     // 정면(헤딩 평행)으로 박았나
    float ComputeReverseEscapeSteer(const Vec3 &blockerCenter) const; // 후진중 코를 돌릴 조향
    void TryStartStaticReverseEscape();                               // 정적장애물에 막히면 후진
    bool UpdateSirenWait();                                           // 사이렌차 감지→강제전환/해제, 처리시 true
    bool HasSirenCarBehind() const;                                   // 반경 내 뒤쪽 사이렌차 존재?
    float ComputeSirenStopOffset() const;                             // 가장 오른쪽 밴드 오른쪽 경계 d
    bool UpdateChase();                                               // 추격: 경로갱신+차단정차, 처리시 true
    Car *FindFleeTarget() const;                                      // 반경 내 최근접 도망차
    bool BuildChasePath(Car *target);                                 // 도망차 도로까지 경로 재계산
    bool IsAheadOfFleeTarget(const Car *target) const;                // 같은 도로에서 앞질렀나
    float ComputeChaseBlockOffset(const Car *target) const;           // 도망차 차로중심 d
    void ClearChase();                                                // 추격상태 해제
    // 부채꼴 슬롯 interest/danger로 매프레임 조향각 생성(추격/도망 전용)
    float ComputeContextSteer(const Vec3 &pursuitTarget, float pursuitSteer) const;
    void DecideAvoidance();                                                          // 위협분류→서브모드
    void UpdateAvoid();                                                              // D_Avoid: 오프셋 회피 재계획/복귀/종료
    void UpdateLaneChange();                                                         // 차선변경 진행/취소/완료
    void HandleAvoidStuck();                                                         // 회피 불가: 그 자리에 정지 유지
    bool HasPassedAvoidTarget() const;                                               // 유발 장애물을 뒤로 보냈나
    float DistanceToClearObstacle(const VehicleCollision::Obstacle &obstacle) const; // 뒷범퍼가 완전히 지나칠 거리
    ThreatKind ClassifyFrontThreat() const;                                          // 전방 최근접 히트를 차/정적으로 분류
    bool IsOnLane(float offset) const;                                               // 지금 밴드 폭 안에 있나
    // 현재 목표 횡오프셋
    float AvoidTargetOffset() const;
    Vec3 GetBodyCenter() const;                             // OBB 판정 기준점
    VehicleCollision::Obstacle MakeVehicleObstacle() const; // 이 차의 차체 OBB를 장애물 하나로
    bool FindAvoidOffset(float laneCenter, const VehicleCollision::Obstacle &target, float &outOffset) const;
    float SweepBodyPath(float targetOffset, const std::vector<VehicleCollision::Obstacle> &obstacles,
                        float speed, float maxDistance) const;
    bool SimulateAvoidPath(float targetOffset, const VehicleCollision::Obstacle &target,
                           const std::vector<VehicleCollision::Obstacle> &obstacles) const;
    void RebuildSensorRender();
    void RebuildEmergRayRender();
    void RebuildCtxSteerRender();
#pragma endregion

#pragma region RaceInternal
    void UpdateRace();      // Mode::Race 틱. 경로 유지 + 센서/계획 주기
    void RaceControl();     // DriveControl의 레이스 분기. 조향/가감속을 직접 낸다
    void UpdateRacePlan();  // 계획 오프셋을 실제 위치에 맞춰 클램프
    void OnRaceModeEnter(); // 성능 envelope을 레이싱 값으로 교체
    void OnRaceModeExit();  // 일반 주행 값 복원
    void OnLapCompleted();
    void RebuildRacingLineRender();
    bool IsOutsideDrivingBands() const;

    // 레이싱 라인에 투영해 "내 앞, 같은 라인" 차를 고른다
    RaceNeighbors FindRaceNeighbors(const RaceLine &line, size_t myIndex, float myS) const;
    // 추월 상태를 굴리고 목표 횡오프셋(m_raceLateralOffset)을 갱신한다
    void UpdateOvertake(const RaceLine &line, size_t myIndex, const RaceNeighbors &neighbors,
                        float lineRoom, float laneOffsetAtCar, float paceSpeed, float curvature);
    // 어느 쪽으로 나갈지. 0이면 양쪽 다 공간이 없다는 뜻
    float PickOvertakeSide(const RaceLine &line, size_t myIndex, float lineRoom,
                           float laneOffsetAtCar) const;
    // 지금 방향으로 낼 수 있는 횡오프셋(도로 폭으로 클램프)
    float OvertakeOffset(size_t myIndex, float lineRoom, float laneOffsetAtCar) const;
    // 얼리 에이펙스 라인이 지금 라인보다 얼마나 안쪽으로 들어가 있는가(부호=방향).
    // 직선에서는 0, 코너 진입에서 커진다.
    float EarlyApexDelta(size_t index, float laneOffsetAtCar) const;
    // IDM 상호작용 항 -- 갭이 목표보다 좁으면 음수(감속 요구)를 돌려준다
    float LeaderFollowAccel(const LeaderInfo &leader, float brakeAvailNow, float headwayScale) const;
    // 접근속도 0일 때 유지하려는 갭. 추월 트리거 기준으로도 쓴다
    float DesiredFollowGap(float headwayScale) const;
    // 상대 대비 진짜 페이스 우위(m/s). 같은 라인 같은 곡률 위에 있으므로 코너에서는
    // 그립 차이, 직선에서는 최고속 차이가 그대로 속도 차이가 된다.
    float PaceAdvantageOver(const Car &other, float curvature) const;
    // 뒤차가 절반 길이 이상 오버랩하면 그쪽에 한 대가 지나갈 공간을 보장한다.
    void ApplyYieldRule(const RaceNeighbors &neighbors, float &desiredOffset);
    // 레이싱라인 조향을 유지하되, 짧은 시간 안에 실제 차체 충돌이 예측될 때만 최소 보정
    float ComputeRaceContextSteer(float pursuitSteer) const;
#pragma endregion

public:
    static constexpr float ARRIVE_DISTANCE = 5.0f;
    static constexpr float PARK_ARRIVE_DISTANCE = 10.0f;
    static constexpr float STOPPED_SPEED = 0.01f;

private:
    // 일반 주행 기준값. 레이스 모드에서는 OnRaceModeEnter가 레이싱 값으로 갈아끼우고
    // OnRaceModeExit가 이 값들을 되돌린다(그래서 const가 아니다).
    static constexpr float ROAD_MAX_SPEED = 200.0f / 3.6f;          // 200 km/h
    static constexpr float ROAD_MAX_BRAKE = (100.0f / 3.6f) / 3.0f; // 100-0 km/h in 3s
    static constexpr float ROAD_SPEED_GAIN = 2.0f;

    float m_maxSpeed = ROAD_MAX_SPEED;
    float m_maxAccel = (100.0f / 3.6f) / 14.0f; // personality.maxAccel로 갱신됨
    float m_maxBrake = ROAD_MAX_BRAKE;
    float m_speedGain = ROAD_SPEED_GAIN; // 속도오차 -> 목표가속 비례게인
    float m_jerkUp = 4.0f;               // 가속 저크 상한
    float m_jerkDown = 15.0f;            // 제동 저크 상한
    CarPersonality m_personality;                    // IDM 파라미터에 반영
    CarPersonality m_prevPersonality;                // 특수상태 끝나면 복원할 원래 성격
    bool m_specialPersonalityOn = false;             // 특수 성격 적용중

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
    bool m_sirenOn = false;          // 사이렌 on/off(디버그 토글)
    bool m_sirenPulledOver = false;  // 도로끝 도착후에만 정차
    bool m_chaseOn = false;          // 추격 on/off(디버그 토글)
    bool m_fleeOn = false;           // 도망 on/off(디버그 토글)
    bool m_chaseSiren = false;       // 추격이 켠 사이렌인가
    Car *m_chaseTarget = nullptr;    // 추격중인 도망차, 역참조전 생존확인
    int m_chaseTargetRoadId = -1;    // 바뀌면 즉시 재계획
    float m_lastChasePlanTime = -1000.0f;
    bool m_isControl = false; // true면 수동조작
    int m_id = -1;
    float m_aiStartDelay = 0.0f; // Init에서 지정, 이 시간까지 AI 정지
    float m_aiDelayTimer = 0.0f;

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
    mutable LaneBand m_virtualBands[2];                 // 현재도로 양끝 밖 가상차로(추격/도망)
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
    // 스팟 반대쪽 랜덤 조향
    static constexpr float PARK_PRE_YAW_MAX = ToRadians(90.0f);
    static constexpr float PARK_PRE_SIDE_MAX = 1.5f; // 조향각 비례 측면 오프셋
    float m_parkPreYaw = 0.0f;                       // 스팟당 1회 추첨

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
    mutable float m_ctxSteerDebugRad = 0.0f;          // 반응형 조향이 고른 슬롯각
    mutable float m_ctxSteerDebugDanger = 0.0f;       // 그 슬롯의 danger
    mutable std::vector<Vec3> m_ctxSteerOpenLines;    // 안전 슬롯 레이(초록)
    mutable std::vector<Vec3> m_ctxSteerBlockedLines; // 위험 슬롯 레이(빨강)

    // 매프레임 스윕 갱신
    static constexpr float LOW_SPEED = 18.26f / 3.6f;
    static constexpr float AVOID_CLEAR_DELAY = 0.5f;      // 복귀 디바운스(s)
    static constexpr float AVOID_RETURN_TOLERANCE = 0.3f; // 회피종료 판정거리
    static constexpr float AVOID_MIN_SHIFT = 0.5f;        // 최소 유효 횡이동
    static constexpr float AVOID_SIM_MIN_SPEED = 3.0f;    // 스윕 최소속도(m/s)
    // 정면충돌 후진탈출(추격/도망 전용)
    static constexpr float REVERSE_ESCAPE_TIME = 5.0f;     // 후진 상한(s), 안전장치
    static constexpr float REVERSE_ESCAPE_TURN_COS = 0.7f; // 코 60도 돌면 탈출
    static constexpr float REVERSE_ESCAPE_SPEED = 2.5f;    // 후진 속도(m/s)
    static constexpr float FLEE_LAUNCH_SPEED = 5.0f;       // 이 아래면 출발가속 취급(m/s)
    static constexpr float REVERSE_ESCAPE_STEER = ToRadians(30.0f);
    static constexpr float REVERSE_STEER_RAMP = 2.0f;
    static constexpr float HEADON_PARALLEL_COS = 0.82f; // 헤딩 평행 판정(~35도)
    static constexpr float HEADON_FRONT_COS = 0.5f;     // 앞쪽 60도 안에서 부딪혔나
    // 정적장애물은 접촉이벤트가 없다(Static 바디). IDM이 닿기 전에 세우므로 "막혀 선 시간"으로 판정
    static constexpr float STATIC_BLOCK_TRIGGER = 0.3f; // 이 시간 막혀 있으면 후진(s)
    static constexpr float STATIC_BLOCK_RANGE = 3.0f;   // 앞범퍼 기준 이 안이면 막힘
    static constexpr float STATIC_BLOCK_SPEED = 0.5f;   // 사실상 정지(m/s)

    // 물리/판단 틱 다름
    bool m_contactPending = false;
    float m_reverseEscapeTimer = 0.0f;            // >0이면 정면충돌 후진중
    float m_reverseEscapeSteer = 0.0f;            // 후진하며 코를 돌릴 방향
    Vec3 m_reverseEscapeStartFwd = Vec3::sZero(); // 회전량 측정 기준

    float m_staticBlockTimer = 0.0f;           // 정적장애물 막힘 시간
    Vec3 m_staticBlockLastPos = Vec3::sZero(); // 실제 이동량 측정 기준
    bool m_stuck = false;                      // 못피해 정지
    ManeuverState m_maneuver;
    float m_speedCap = -1.0f;

#pragma region RaceTuning
    // Track-focused GT racing specification. 개체별 편차는 CarPersonality의 레이스 항목이
    // 여기에 곱해진다(OnRaceModeEnter에서 1회 적용).
    static constexpr float RACE_BASE_MAX_SPEED = 320.0f / 3.6f;          // 최고속도 320 km/h
    static constexpr float RACE_BASE_MAX_ACCEL = (100.0f / 3.6f) / 3.0f; // 0-100 km/h 약 3.0초
    static constexpr float RACE_BASE_MAX_BRAKE = (100.0f / 3.6f) / 1.8f; // 100-0 km/h 약 1.8초 (약 1.57 g)
    static constexpr float RACE_BASE_GRIP_ACCEL = 15.0f;                 // 타이어 마찰원 반경(약 1.53 g)
    static constexpr float RACE_SPEED_GAIN = 3.5f; // 목표속도 변화에 빠르게 풀스로틀/브레이크
    static constexpr float RACE_MAX_STEER_ANGLE = ToRadians(35.0f);
    // 그립 한계가 조향 포화와 코너 목표속도 두 군데에 따로 박혀 있으면 어긋난다. 하나로 묶되
    // 원래 튜닝을 유지하려고 그 비율을 "계획은 타이어 한계를 다 쓰지 않는다"는 마진으로 명시한다.
    static constexpr float PLAN_GRIP_RATIO = 8.0f / 15.0f;

    // 슬립스트림: 앞차 와류 안에서는 항력이 줄어 최고속과 가속이 함께 올라간다.
    // 항력 모델을 따로 두지 않고 상한에 배수를 먹이는 근사다.
    // 추종 차간(FOLLOW_TIME_HEADWAY)보다 넉넉히 넓어야 한다. 좁으면 종방향 제어가
    // 와류 밖에 차를 세워둬서 토우가 영영 걸리지 않는다.
    static constexpr float SLIPSTREAM_RANGE = 90.0f;          // 이보다 뒤면 와류가 닿지 않는다
    static constexpr float SLIPSTREAM_LATERAL_SPAN = 2.5f;    // 차폭 합의 몇 배까지 옆으로 남는가
    static constexpr float SLIPSTREAM_MIN_SPEED = 30.0f;      // m/s. 항력은 v^2이라 저속에선 무의미
    static constexpr float SLIPSTREAM_TOP_SPEED_GAIN = 0.06f; // 바로 뒤에 붙었을 때 최고속 +6%
    static constexpr float SLIPSTREAM_ACCEL_GAIN = 0.25f;     // 같은 조건에서 가속 여유 +25%
    static constexpr float SLIPSTREAM_RESPONSE = 3.0f;        // 와류가 붙고 떨어지는 응답(1/s)

    // 추종 갭. 추월 트리거도 이 값을 기준으로 잡아야 personality/속도에 같이 따라간다.
    static constexpr float FOLLOW_STANDSTILL_GAP = 1.0f; // 정지 시 유지할 최소 갭(m). 레이싱이라 짧다
    // 일반도로 값(0.5s)을 쓰면 320km/h에서 45m가 나온다 -- SLIPSTREAM_RANGE 밖이라
    // 토우가 절대 안 걸린다. 실제 레이싱의 추종 간격(0.2~0.3s)에 맞춰 좁혔다.
    static constexpr float FOLLOW_TIME_HEADWAY = 0.2f; // 목표 차간시간(s)

    // 추월 판단
    // 얼리 에이펙스 라인이 이만큼은 안쪽으로 들어가야 "다이브할 코너"로 본다.
    static constexpr float OVERTAKE_EARLY_MIN_DELTA = 0.8f;
    static constexpr float OVERTAKE_PACE_MARGIN = 1.5f; // 앞차보다 이만큼 빨라야 시도한다(m/s)
    // 트리거를 고정 거리로 두면 안 된다. 차간이 넓은(headwayFactor가 큰) 드라이버는
    // 평형 갭이 그 거리보다 멀어서 영영 시도조차 못 하게 된다.
    static constexpr float OVERTAKE_TRIGGER_SCALE = 2.0f; // 평형 추종 갭의 이 배 안이면 시도
    // 두 차가 나란히 서고도 여유가 남을 만큼은 있어야 한다(차폭 합 2.7m + 여유).
    static constexpr float OVERTAKE_MIN_ROOM = 3.6f;
    // 지금 열려 있어도 코너에서 레이싱 라인이 그쪽으로 흐르면 공간이 사라진다.
    // 시작 전에 이 거리만큼 앞을 보고 "끝까지 열려 있는 쪽"을 고른다.
    static constexpr float OVERTAKE_SIDE_PREVIEW = 60.0f;
    // 코너 한복판에서는 시작하지 않는다. 추월은 직선이나 브레이킹 존 진입에서 걸고
    // 코너로 가지고 들어가는 것이지, 굽은 한가운데서 나란히 서는 게 아니다.
    // (시작 조건에만 건다 -- 일단 걸린 추월은 코너 안까지 이어져야 한다)
    static constexpr float OVERTAKE_MAX_CURVATURE = 1.0f / 250.0f;
    static constexpr float OVERTAKE_STALL_TIME = 3.5f;   // 진전 없이 이만큼 지나면 접는다
    static constexpr float OVERTAKE_PROGRESS_EPS = 0.5f; // 이만큼은 줄어야 '진전'으로 본다
    // 차체 요각은 LateralHalfExtent가 이미 반영하므로, 여기 값은 순수한 여유분이다.
    static constexpr float SIDE_SAFE_CLEARANCE = 0.8f; // 나란히 있을 때 유지할 차체 간 여유
    // 완전히 겹친 뒤에 밀기 시작하면 늦다. 다가붙는 동안부터 벌린다.
    static constexpr float ALONGSIDE_APPROACH = 4.0f;
    // 명령한 오프셋과 실제 위치는 다르다. 그래서 "실제로 이만큼 벌어졌는가"를 확인한
    // 뒤에야 붙기 시작한다. leader 판정 임계(LATERAL_MARGIN)보다 커야 한다 -- 작으면
    // Setup을 통과하고도 여전히 leader로 잡혀 앞차 뒤에 눌린 채 옆에만 서 있게 된다.
    static constexpr float OVERTAKE_SETUP_CLEARANCE = 1.4f;
    static constexpr float OVERTAKE_SETUP_TIMEOUT = 4.0f; // 그 안에 못 벌리면 접는다
    static constexpr float OVERTAKE_TIMEOUT = 8.0f;       // 이 시간 안에 못 지나가면 포기
    static constexpr float OVERTAKE_COOLDOWN = 2.0f;      // 포기 후 재시도 금지 시간
    // 내 뒤범퍼가 상대 앞범퍼보다 이만큼 앞서야 통과다. 중심간 거리로 재면 차 길이만큼
    // 아직 겹쳐 있는 상태에서 라인으로 복귀해 옆구리를 긁는다.
    static constexpr float OVERTAKE_CLEAR_MARGIN = 3.0f;
    static constexpr float OVERTAKE_HEADWAY_SCALE = 0.7f; // 나간 동안엔 차간을 이만큼 좁힌다
    static constexpr float YIELD_ACQUIRE_OVERLAP = 0.5f;  // 뒤차 길이의 절반이 들어오면 권리 획득
    static constexpr float YIELD_RELEASE_OVERLAP = 0.25f; // 이 아래로 후퇴해야 권리 해제(히스테리시스)
    // 접촉 회피가 이 응답을 타므로 너무 느리면 밀어내기가 제때 안 먹는다.
    static constexpr float LATERAL_RESPONSE = 2.5f; // 목표 오프셋으로 옮겨가는 응답(1/s)
#pragma endregion

#pragma region RaceState
    bool m_raceMode = false;
    // 일반 주행 값 복원용. OnRaceModeEnter가 덮어쓰기 전 스냅샷.
    float m_savedMaxAccel = 0.0f;
    float m_savedJerkUp = 0.0f;
    float m_savedJerkDown = 0.0f;
    float m_gripAccel = RACE_BASE_GRIP_ACCEL; // 조향 포화와 마찰원이 공유하는 그립
    // 슬립스트림이 얹힌 현재 속도 상한. m_maxSpeed로 직접 클램프하면 와류 이득이 잘려나간다.
    // (일반 주행의 m_speedCap과 의미가 달라 따로 둔다 -- 저쪽은 -1이 "제한 없음"이다)
    float m_raceSpeedCap = RACE_BASE_MAX_SPEED;

    LeaderInfo m_leader; // 매 RaceControl마다 갱신
    int m_lap = 0;
    float m_lapLength = 0.0f;
    float m_lapStartTime = -1.0f; // 첫 결승선 통과 전에는 랩타임을 재지 않는다
    float m_lastLapTime = 0.0f;
    float m_bestLapTime = 0.0f;
    float m_raceS = -1.0f; // 공유 라인 위 호길이. 음수면 아직 한 번도 못 잡았다
    float m_raceD = 0.0f;  // 그 지점 기준 내 횡오프셋
    float m_raceCurvature = 0.0f;
    int m_racePosition = 0; // 1부터. 0이면 아직 미확정
    int m_pendingPosition = 0;
    float m_pendingPositionSince = 0.0f;
    int m_overtakes = 0;
    int m_overtakenBy = 0;

    float m_slipstream = 0.0f;         // 감쇠를 거친 현재 와류 세기
    float m_slipstreamGapDebug = 0.0f; // 와류를 준 차와의 갭(디버그)

    OvertakeState m_overtakeState = OvertakeState::None;
    Car *m_overtakeTarget = nullptr;   // Attack 중 추적 대상. leader에서 빠져도 계속 본다
    float m_overtakeSide = 0.0f;       // -1=왼쪽, +1=오른쪽. 진입 시 한 번 정하고 유지한다
    float m_overtakeTimer = 0.0f;      // 현재 상태 유지 시간
    float m_overtakeCooldown = 0.0f;   // 포기 후 재시도 금지 잔여 시간
    float m_overtakeBestDeltaS = 0.0f; // Attack 중 관측한 최소 상대 s(진전 판단용)
    float m_overtakeStall = 0.0f;      // 진전 없이 흐른 시간
    const char *m_overtakeBlock = "";  // 시작을 막고 있는 조건(디버그)
    int m_overtakeAttempts = 0;        // Follow -> Setup 진입 횟수
    int m_overtakeAborts = 0;          // 완주 못 하고 Return으로 접은 횟수
    int m_overtakeSuccess = 0;         // 실제로 앞질러서 끝난 횟수
    bool m_overtakeHoldOffset = false; // 물러나는 중 겹침이 안 풀려 오프셋을 붙잡고 있다
    Car *m_yieldTarget = nullptr;      // 내 뒤에서 권리를 얻어 공간을 보장 중인 추월차
    float m_yieldSide = 0.0f;          // 추월차가 들어온 쪽(-1=왼쪽, +1=오른쪽)
    float m_raceLateralOffset = 0.0f;  // 레이싱 라인 기준 목표 횡오프셋(감쇠 적용된 현재값)
    // 시작 위치를 0으로 당기지 않도록 실제 d로 한 번 초기화한다
    bool m_raceLateralOffsetInitialized = false;

    RenderObject m_racingLineRender;      // 레이싱 라인(디버그)
    const RaceLine *m_raceLine = nullptr; // SimulationState가 소유한 공유 라인
    // 추월할 때 참고하는 얼리 에이펙스 라인. 갈아타는 게 아니라 "얼마나 안쪽으로
    // 파고들 수 있는가"의 기준으로만 쓴다 -- 라인만 바꿔서는 간격이 안 나온다.
    const RaceLine *m_attackLine = nullptr;
    size_t m_raceLineCursor = 0; // 최근접점 탐색 시작 위치(전체 훑기 방지)
    ApexStrategy m_apexStrategy = ApexStrategy::Late;
    ApexStrategy m_boundStrategy = ApexStrategy::Count; // m_raceLine이 가리키는 전략
#pragma endregion

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
    RenderObject m_sensorRender;          // 스윕박스 윤곽선(디버그)
    RenderObject m_emergRayRender;        // 긴급제동 레이(디버그)
    RenderObject m_ctxSteerOpenRender;    // 반응형 조향 안전 레이(디버그)
    RenderObject m_ctxSteerBlockedRender; // 반응형 조향 위험 레이(디버그)
    RenderObject m_parkPathRender;        // Park 계획(RS 경로) 폴리라인
    RenderObject m_parkTargetMarker;      // Park 목표 위치
    RenderObject m_parkTargetLine;        // Park 목표 방향
};

static float CalcMaxSteerAngle(float speed)
{
    constexpr float LOW_SPEED_CUTOFF = 18.26f / 3.6f;
    constexpr float MAX_STEER_ANGLE = ToRadians(45.0f);
    return (speed <= LOW_SPEED_CUTOFF) ? MAX_STEER_ANGLE : 20.2f / (speed * speed);
}
// 레이스 모드 전용. gripAccel은 Car::m_gripAccel(개체별 편차 반영)을 받는다. 그립 좋은
// 차가 고속에서 더 큰 조향각까지 쓸 수 있어야 코너 통과속도 차이가 실제 궤적으로 나타난다.
static float CalcRaceMaxSteerAngle(float speed, float wheelbase, float gripAccel)
{
    constexpr float MAX_STEER_ANGLE = ToRadians(35.0f);
    constexpr float MIN_WHEELBASE = 0.1f;

    wheelbase = std::max(wheelbase, MIN_WHEELBASE);
    gripAccel = std::max(gripAccel, 0.1f);
    float transitionSpeed = std::sqrt(gripAccel * wheelbase / std::tan(MAX_STEER_ANGLE));
    if (speed <= transitionSpeed)
        return MAX_STEER_ANGLE;

    // bicycle model: lateral accel = v^2 * tan(steer) / wheelbase
    return std::atan(gripAccel * wheelbase / (speed * speed));
}
static float CalcMaxSpeed(float targetAngle)
{
    constexpr float MAX_STEER_ANGLE = ToRadians(45.0f);
    targetAngle = std::clamp(std::abs(targetAngle), 0.0005f, MAX_STEER_ANGLE);
    return std::sqrt(20.2f / targetAngle);
}
