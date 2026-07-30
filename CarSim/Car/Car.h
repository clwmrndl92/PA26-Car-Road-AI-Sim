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
    void Accelerate(float desiredVelocity, float aFF = 0.0f); // aFF: 계획감속 피드포워드(리더면 앞차 가속도=CAH, 정적이면 −planBrake)
    void CommandAcceleration(float aTarget);                  // 목표 가속도를 저크제한으로 수렴(IDM 출력용)
    void EmergBrake();
    void Steer(float desiredRadian, float steerRamp = 1.0f);
    void ChangeGear(); // 속도가 낮을 때 전진/후진 기어 토글
    bool IsReverse() const { return m_isReverse; }

    void DriveControl(); // VehicleController에서 호출
    float PurePursuit(Vec3 target);
    float Stanley(const Spline &spline); // 앞축 기준 경로추종 조향각 (부호 규약은 PurePursuit와 동일: + = 우조향)

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
    void SetCurrentRoad(const shared_ptr<Road> &road, float offset); // m_currentRoad/Offset/Spline을 항상 이 함수로 세팅.
    bool ShouldStopForSignal(const shared_ptr<Road> &road) const;    // CheckPath(Drive)와 ScanRoadSpeedConstraints(BehaviorPlan)이 공유.
    bool TryFindPathAndSetRoad();

    // 배회(roaming) 모드: 목적지 없이 랜덤 후속 road로 계속 주행. m_path를 랜덤 successor로 채워 유지한다.
    void EnsureRoamingPath();                                                           // currentRoad/초기 path 세팅 + 유지
    void MaintainRoamingPath();                                                         // 지나온 앞부분 트림 + 앞쪽 버퍼 채우기
    shared_ptr<Road> PickRandomSuccessor(const shared_ptr<Road> &road) const;           // successor 중 랜덤(없으면 nullptr)
    vector<shared_ptr<Road>> BuildRoamingPath(const shared_ptr<Road> &startRoad) const; // startRoad + 랜덤 후속 몇 개

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

    // 경적: 장애물/차에 막혀 정지해 있으면 5초마다 한 번 울린다(2초간 차체를 빨갛게).
    void UpdateHorn(float dt);
    bool IsHornSituation() const;     // Drive 중 (빨간불 대기가 아닌) 정지 상태인가
    bool KnowsRedSignalAhead() const; // 경로 앞쪽 가장 가까운 신호가 빨강인 걸 아는 상태인가(그럼 경적 안 울림)

    float ComputeLookaheadDistance() const;
#pragma endregion

#pragma region BehaviorPlan
    struct RoadSpeedSample
    {
        Vec3 position;
        float distance;
        float speed;
        Car *leader = nullptr;            // 이 샘플이 앞차(리더)면 그 차, 정적 제약(신호/커브/정지선 등)이면 nullptr
        float leaderLateralOffset = 0.0f; // road 참조선 기준 리더의 d(횡오프셋). leader!=nullptr일 때만 유효 -- 후보가 이 리더를 옆으로 피할 수 있는지 판단용
    };

    struct NearbyCar
    {
        Car *car = nullptr;
        bool yieldsToMe = false;
    };

    // 한 밴드(centerOffset)에서 ego 앞/뒤로 가장 가까운 차를 MOBIL 판정용 상태로 뽑은 것.
    struct LaneNeighbors
    {
        bool hasLeader = false;
        bool hasFollower = false;
        Mobil::VehicleState leader;
        Mobil::VehicleState follower;
    };

    void UpdateBehaviorPlan();
    IDM::Params BuildIdmParams(const shared_ptr<Road> &road) const; // road 제한속도를 v0로 반영한 IDM 파라미터
    // samples를 IDM 가상 리더로 보고 각각의 IDM 가속도 중 최솟값을 반환(매프레임 호출 가능). 실제 앞차(leader!=nullptr)는
    // 그 차의 현재 속도/가속도/거리를 매번 새로 읽고, 정적 제약(신호/커브 등)은 distanceOffset(스캔 이후 이동거리)만큼
    // gap을 보정한다 -- samples 자체(리더가 '누구인지')는 0.2초 스캔 결과를 재사용해도 된다.
    float ComputeIdmAcceleration(const std::vector<RoadSpeedSample> &samples, const IDM::Params &params,
                                 float distanceOffset) const;
    // road의 targetOffset 근처 밴드로 진입해도 뒤차에게 안전한지 MOBIL 안전기준(강제 차선변경)으로 판정.
    // 도로 밖에서 처음 진입(UpdateFindPath/EnsureRoamingPath)과 다음 road로 합류(ShouldHoldForMerge)가 공유.
    bool IsSafeLaneEntry(const shared_ptr<Road> &road, float targetOffset, const std::vector<NearbyCar> &nearby) const;
    // nextRoad 진입(합류)이 지금 뒤차에게 안전한지 판정(IsSafeLaneEntry + m_lastNearbyCars).
    // CheckPath(Drive)와 ScanRoadSpeedConstraints(BehaviorPlan)이 공유.
    bool ShouldHoldForMerge(const shared_ptr<Road> &nextRoad) const;
    std::vector<NearbyCar> CollectNearbyCars() const;
    bool IsTurningAhead() const;                  // 교차로 우선순위: 직진 > 회전 판정용
    bool HasPriorityOver(const Car *other) const; // 우선순위 직진 > 회전, 동급이면 이름 비교
    void AppendCarConstraintSamples(std::vector<RoadSpeedSample> &samples,
                                    const std::vector<NearbyCar> &nearbyCars, float lookDistance) const; // nearbyCars 중 내 예정 경로 코리도 안의 차를 도로제약 샘플(가상 리더)로 변환해 추가
    // nearby 중 refLine 기준 bandCenter 밴드에 속한 차들에서 egoS 앞/뒤 가장 가까운 leader/follower를 MOBIL 상태로.
    LaneNeighbors GatherLaneNeighbors(const std::vector<NearbyCar> &nearby, const Spline &refLine,
                                      float bandCenter, float bandHalfWidth, float egoS) const;
    // 현재 밴드 유지 vs MOBIL 판정 인접 밴드로 변경 -> 목표 횡오프셋 d.
    float ComputeLateralTarget(const std::vector<NearbyCar> &nearbyCars, const IDM::Params &idm) const;
    std::vector<RoadSpeedSample> ScanRoadSpeedConstraints(float lookDistance) const;
    // road 횡단면(GetLateralProfile)에서 차체가 도로 밖으로 안 나가는 drivable d 범위. 밴드 없으면 참조선 중심 좁은 범위.
    void ComputeDrivableRange(const shared_ptr<Road> &road, float &outMin, float &outMax) const;
#pragma endregion

public:
    static constexpr float ARRIVE_DISTANCE = 5.0f;
    static constexpr float PARK_ARRIVE_DISTANCE = 10.0f;
    static constexpr float LANE_TRANSITION_THRESHOLD = 2.0f; // 다음 차선으로 완전히 넘어가는(전환되는) 임계값

private:
    // 설정 및 스펙 상수/변수 (Constants & Specifications)
    const float m_maxSpeed = 200.0f / 3.6f;               // 200 km/h
    const float m_maxAccel = (100.0f / 3.6f) / 14.0f;     // 0-100 km/h in 14s
    const float m_maxBrake = (100.0f / 3.6f) / 6.0f;      // 0-100 km/h in 6s (~4.6 m/s², 현실적 상용제동)
    const float m_maxEmergBrake = (100.0f / 3.6f) / 3.0f; // 100-0 km/h in 3s
    float m_speedGain = 2.0f;                             // 속도오차 -> 목표가속 비례게인
    float m_jerkUp = 4.0f;                                // 가속 방향 저크 상한 (m/s^3). Init에서 m_personality로 덮어씀
    float m_jerkDown = 15.0f;                             // 제동 방향 저크 상한. Init에서 m_personality로 덮어씀
    CarPersonality m_personality;                         // 운전자 성격(CarSpec::personality) -- IDM/MOBIL 파라미터에 반영, 디버그 UI로 조절

    float m_wheelbase = 0.0f;
    float m_mass = 1.0f;
    Vec3 m_halfExtents = Vec3::sZero();               // 충돌판정용 차체 반크기(x=반폭, z=반길이). CarSpec::halfExtents.
    float m_maxSteerAngle = ToRadians(45.0f);         // 최대 조향각 (45도)
    float m_stanleyGain = 1.0f;                       // Stanley 횡오차 게인 k
    float m_stanleySoft = 1.0f;                       // Stanley 저속 소프트닝 상수 (분모 v+soft, 발산 방지)
    static constexpr float CURVE_SPEED_COEFF = 1.22f; // 최대 코너링 속도 = CURVE_SPEED_COEFF * sqrt(R)
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
    shared_ptr<Road> m_destRoad;
    shared_ptr<Road> m_currentRoad;
    float m_currentOffset = 0.0f; // 계획된(committed) 횡오프셋 d -- 실측이 아니라 리플랜 기준 상태
    Spline m_currentSpline;       // 현재 주행 스플라인(참조선 offset d) 캐시

    shared_ptr<RoadNode> m_parkSpot;        // 예약된 목표 주차칸(있는 동안은 "이 자리에 주차 중/주차 예정")
    shared_ptr<RoadNode> m_pendingParkNode; // 예약 전, 도착하면 그때 주차칸을 예약할 목표 Park 노드
    bool m_parkSequenceActive = false;
    int m_parkNodeId = -1;                 // 이번 입차의 대상 Park 노드 id (다른 빈 자리 재예약에 씀)
    int m_parkLegTries = 0;                // 현재 스팟의 진입 leg 체인 시도 횟수 (무한 체인 방지)
    unordered_set<int> m_triedParkSpotIds; // 이번 입차에서 경로탐색이 실패해 이미 시도해본 ParkSpot id들
    bool m_parkPlanPending = false;

    bool m_roaming = true;                         // 배회 모드: 목적지 없이 랜덤 후속 road로 계속 주행
    static constexpr size_t ROAMING_MIN_AHEAD = 3; // 배회 시 현재 road 앞으로 항상 유지할 최소 road 버퍼 수

    vector<shared_ptr<Road>> m_path;
    size_t m_pathIndex = 0;
    float m_currentTime = 0.0f;
    // 노란불 때 "정지거리 안쪽이라 통과" 확정한 신호 id(초록될 때까지 유지). 없으면 -1.
    mutable int m_committedYellowNodeId = -1;

    // 경적 상태
    static constexpr float HORN_STOP_SPEED = 0.3f;             // 이 속도 이하면 "정지"로 본다(m/s)
    static constexpr float HORN_INTERVAL = 5.0f;               // 막혀 있는 동안 경적 주기(초)
    static constexpr float HORN_FLASH_DURATION = 2.0f;         // 경적 표시로 빨갛게 두는 시간(초)
    static constexpr float HORN_SIGNAL_AWARE_DISTANCE = 40.0f; // 이 거리 안의 빨간 신호는 "알고 서 있다"고 보고 경적을 참는다(대기줄 커버, m)
    float m_hornStoppedDuration = 0.0f;                        // 막혀서 정지해 있던 누적 시간
    float m_hornFlashTimer = 0.0f;                             // 남은 빨간 표시 시간(>0이면 빨갛게 그린다)
    Model *m_carModel = nullptr;                               // 차체 모델(경적 시 재질 색을 잠깐 빨갛게 바꾼다)

    // 행동 계획(Behavior Plan) 상태
    static constexpr float BEHAVIOR_PLAN_INTERVAL = 0.2f;  // 행동 후보 재판단 주기
    static constexpr float BEHAVIOR_SAFETY_HORIZON = 3.0f; // 주변 차 수집 반경 산정용 미래 시야(초, 사람의 3초 룰)
    static constexpr float MIN_SAFE_GAP = 2.0f;            // 앞차/정지선과 유지할 표준 범퍼 gap(m). IDM s0로도 쓴다.
    static constexpr float DESIRED_HEADWAY = 2.0f;         // 디버그 표시용 목표 시간헤드웨이(s).

    // IDM(종방향) / MOBIL(차선변경) 파라미터. 이타성(politeness)은 m_personality에서 옴(D. 양보).
    static constexpr float IDM_TIME_HEADWAY = 1.5f; // IDM T 기본값: 앞차와 원하는 시간 간격(s). m_personality.headwayFactor를 곱해 씀.
    static constexpr float MOBIL_B_SAFE = 3.0f;     // 뒤차에 강제 가능한 최대 안전 감속도(m/s^2)
    static constexpr float MOBIL_A_THR = 0.2f;      // 차선변경 최소 진입 장벽(m/s^2)
    // 횡오프셋을 목표(밴드 중심)로 당기는 Lerp 비율. 리플랜(0.2초)마다 이만큼 목표 쪽으로 이동. m_personality.laneChangeLerpAlpha에서 옴.

    float m_lastBehaviorPlanTime = -1000.0f;        // 처음 Drive 진입 시 바로 첫 판단이 돌도록 충분히 과거로 초기화
    float m_planAccel = 0.0f;                       // DriveControl이 매프레임 계산한 IDM 목표가속도(디버그 UI 표시용 캐시)
    std::vector<NearbyCar> m_lastNearbyCars;        // UpdateBehaviorPlan이 마지막으로 수집한 주변 차 목록 -- ShouldHoldForMerge가 매 프레임 재사용(재수집 비용 회피)
    std::vector<RoadSpeedSample> m_lastRoadSamples; // UpdateBehaviorPlan이 마지막으로 스캔한 리더/제약 목록 -- DriveControl이 매프레임 IDM 가속도 재계산에 재사용
    IDM::Params m_lastIdmParams;                    // 위 스캔 시점의 IDM 파라미터(v0 등) 캐시
    Vec3 m_planScanPosition = Vec3::sZero();        // 위 스캔 시점의 ego 위치 -- 정적 제약 gap을 매프레임 보정(distanceOffset)하는 기준

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
    targetAngle = std::clamp(std::abs(targetAngle), 0.0005f, MAX_STEER_ANGLE);
    return std::sqrt(20.2f / targetAngle);
}