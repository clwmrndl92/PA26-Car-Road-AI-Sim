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
    void Init(const CarSpec &spec, const CarPersonality &personality, SimulationState *simState, JPH::Vec3 position = JPH::Vec3::sZero());

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
    void AccelerateVel(float desiredVelocity);
    void Accelerate(float desiredAccel);
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
        D_Normal,       // 일반 주행
        D_WaitSignal,   // 신호대기
        D_WaitObstacle, // 움직이는 장애물이 진로를 비울 때까지 정지 대기
        D_Avoid,        // 장애물 회피(차로 중심을 벗어난 오프셋 주행)
        // 옆 차로 중심으로 이동 중(차로 '중심'으로 가므로 D_Avoid와 다르다). 회피용 차선변경(DecideAvoidance
        // -> UpdateLaneChange, 커밋된 매뉴버)과 MOBIL의 일반 차선변경(UpdateDrivePlan, 매 틱 재평가) 둘 다
        // 이 서브모드로 보인다 -- 판단/취소 로직은 서로 다르고, 여기서는 라벨만 통일한다.
        D_LaneChange,

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
        case SubMode::D_WaitObstacle:
            return "WaitObstacle";
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
    void OnModeExit(Mode next);    // next: 이번에 새로 전환될 상태(m_mode는 아직 지금 나가는 상태 그대로)
    void SetSubMode(SubMode next); // m_subMode를 직접 대입하지 않고 항상 이 함수를 거친다 (전환 로그).

    VehicleCollision::VehicleShape BuildVehicleShape() const;
    void UpdateFindPath();
    // m_currentRoad/Offset/Dir/Spline을 항상 이 함수로 세팅.
    void SetCurrentRoad(const shared_ptr<Road> &road, float offset, LaneDirection direction);
    // 참조선 s를 진행방향으로 증가하는 부호로 바꾸는 계수(+1 정방향 / -1 역방향).
    float TravelSign() const { return GetTravelSign(m_travelDir); }
    RoadRef CurrentRoadRef() const { return RoadRef{m_currentRoad, m_travelDir}; }
    // CheckPath(Drive)와 ScanRoadSpeedConstraints(BehaviorPlan)이 공유.
    bool ShouldStopForSignal(const shared_ptr<Road> &road, LaneDirection direction) const;
    bool TryFindPathAndSetRoad();

    // 배회(roaming) 모드: 목적지 없이 랜덤 후속 road로 계속 주행. m_path를 랜덤 successor로 채워 유지한다.
    void EnsureRoamingPath();                                         // currentRoad/초기 path 세팅 + 유지
    void MaintainRoamingPath();                                       // 지나온 앞부분 트림 + 앞쪽 버퍼 채우기
    RoadRef PickRandomSuccessor(const RoadRef &road) const;           // successor 중 랜덤(없으면 road==nullptr)
    vector<RoadRef> BuildRoamingPath(const RoadRef &startRoad) const; // startRoad + 랜덤 후속 몇 개

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

    void UpdateDrivePlan();
    IDM::Params BuildIdmParams(const shared_ptr<Road> &road) const; // road 제한속도를 v0로 반영한 IDM 파라미터
    // samples를 IDM 가상 리더로 보고 각각의 IDM 가속도 중 최솟값을 반환(매프레임 호출 가능). 실제 앞차(leader!=nullptr)는
    // 그 차의 현재 속도/가속도/거리를 매번 새로 읽고, 정적 제약(신호/커브 등)은 distanceOffset(스캔 이후 이동거리)만큼
    // gap을 보정한다 -- samples 자체(리더가 '누구인지')는 0.2초 스캔 결과를 재사용해도 된다.
    float ComputeIdmAcceleration(const std::vector<RoadSpeedSample> &samples, const IDM::Params &params,
                                 float distanceOffset) const;
    // road의 targetOffset 근처 밴드로 진입해도 뒤차에게 안전한지 MOBIL 안전기준(강제 차선변경)으로 판정.
    // 도로 밖에서 처음 진입(UpdateFindPath/EnsureRoamingPath)과 다음 road로 합류(ShouldHoldForMerge)가 공유.
    bool IsSafeLaneEntry(const RoadRef &road, float targetOffset, const std::vector<NearbyCar> &nearby) const;
    // nextRoad 진입(합류)이 지금 뒤차에게 안전한지 판정(IsSafeLaneEntry + m_lastNearbyCars).
    // CheckPath(Drive)와 ScanRoadSpeedConstraints(BehaviorPlan)이 공유.
    bool ShouldHoldForMerge(const RoadRef &nextRoad) const;
    std::vector<NearbyCar> CollectNearbyCars() const;
    bool IsTurningAhead() const;                  // 교차로 우선순위: 직진 > 회전 판정용
    bool HasPriorityOver(const Car *other) const; // 우선순위 직진 > 회전, 동급이면 이름 비교
    void AppendCarConstraintSamples(std::vector<RoadSpeedSample> &samples,
                                    const std::vector<NearbyCar> &nearbyCars, float lookDistance) const; // nearbyCars 중 내 예정 경로 코리도 안의 차를 도로제약 샘플(가상 리더)로 변환해 추가
    // nearby 중 refLine 기준 bandCenter 밴드에 속한 차들에서 egoS 앞/뒤 가장 가까운 leader/follower를 MOBIL 상태로.
    // s는 전부 TravelS(진행방향으로 증가)로 재므로 역주행 차로에서도 앞/뒤가 뒤집히지 않는다.
    LaneNeighbors GatherLaneNeighbors(const std::vector<NearbyCar> &nearby, const Spline &refLine,
                                      float bandCenter, float bandHalfWidth, float egoS, float dirSign) const;
    // 현재 오프셋에서 가장 가까운 driving 밴드 중심(MOBIL 판정 없이 밴드 조회만).
    float CurrentLaneCenter() const;
    // 현재 밴드 유지 vs MOBIL 판정 인접 밴드로 변경 -> 목표 횡오프셋 d.
    // outLaneCenter가 nullptr이 아니면 "지금 속한 차로의 중심 d"를 따로 써준다 -- 반환값과 다르면 MOBIL이
    // 차선변경을 하기로 했다는 뜻이라, 회피 판단(DecideAvoidance)이 이걸로 "정상 차선변경 가능"을 본다.
    float ComputeLateralTarget(const std::vector<NearbyCar> &nearbyCars, const IDM::Params &idm,
                               float *outLaneCenter = nullptr) const;
    std::vector<RoadSpeedSample> ScanRoadSpeedConstraints(float lookDistance) const;
    // 레이에 잡힌 전방 장애물을 IDM 가상 리더(정적 제약)로 samples에 추가한다.
    void AppendSensorConstraintSample(std::vector<RoadSpeedSample> &samples) const;
    // road 횡단면에서 차체가 도로 밖으로 안 나가는 drivable d 범위. 마주 오는 차로로 새지 않게 진행방향
    // 밴드만 본다. 밴드 없으면 참조선 중심 좁은 범위.
    void ComputeDrivableRange(const RoadRef &road, float &outMin, float &outMax) const;
#pragma endregion

#pragma region Avoid
    // 차체 사각에서 쏜 감지 레이 하나. hitDistance<0이면 maxDistance 안에서 아무것도 안 맞았다는 뜻.
    struct SensorRay
    {
        Vec3 origin;
        Vec3 end; // 맞았으면 히트 지점, 아니면 레이 끝 (디버그 렌더용)
        float hitDistance = -1.0f;
    };

    // 매프레임 갱신되는 레이 스캔 결과. 그룹별로 "막혔는가"만 뽑아 회피 판단에 쓴다.
    struct SensorScan
    {
        bool frontBlocked = false; // 전방 코리도 안에 (거의) 멈춰 있는 장애물이 잡힘 -- 회피 트리거 후보

        bool sideNear = false;
        bool leftBlocked = false;    // 왼쪽 바로 옆에 뭔가 있음 -- 그쪽으로는 못 피한다
        bool rightBlocked = false;   // 오른쪽 바로 옆에 뭔가 있음
        float frontDistance = -1.0f; // 내 진로 코리도 안에 들어온 전방 히트 중 최단 거리(범퍼 기준). 없으면 -1
        Vec3 frontHitPosition;       // 위 히트 지점 (IDM 가상 리더 샘플 위치)
        // 위 히트 대상이 '내 진행방향으로' 멀어지는 속도(IDM 리더 속도). obstacle.speed는 headingRad 기준
        // 부호 없는 스칼라라, 그대로 쓰면 마주 오는 대상도 같이 도망가는 리더로 잡힌다 -- 투영해서 쓴다.
        float frontHitSpeed = 0.0f;
        bool hasFrontHitObstacle = false;            // 아래 원본 정보가 유효한가
        VehicleCollision::Obstacle frontHitObstacle; // 위 히트의 원본(위협 분류/TTC 계산용)
        // 아직 진로 밖이지만 등속으로 굴려보면 내가 지나갈 때 겹치는 동적 장애물이 있다(비스듬히 접근).
        // 이 경우 '비켜줄까'를 따질 게 아니라 이미 부딪히는 것으로 판정났으므로 무조건 정지 대기다.
        bool movingConflict = false;
        float rearDistance = -1.0f; // 후방 레이 최단 히트 거리. 없으면 -1
        // 차체(OBB) 스윕이 예고한 접촉까지의 주행거리. 없으면 -1. 레이(점)로는 못 잡는 '회전 중
        // 바깥쪽 앞 꼭지점이 쓸고 가는 면적'을 담당한다.
        float bodyContactDistance = -1.0f;
        std::vector<SensorRay> rays; // 디버그 렌더용 전체 목록
    };

    // 회피 진행 상태. "지금 회피 중인가"는 m_subMode(D_Avoid / D_LaneChange)가 유일한 기준이고,
    // 여기에는 그 서브모드 안에서만 의미 있는 값들만 둔다 -- 플래그를 따로 두면 둘이 어긋난다.
    struct AvoidState
    {
        bool stuck = false;            // 좌우 어느 쪽으로도 못 피함 -- 정지(경적은 UpdateHorn이 알아서)
        bool backingUp = false;        // ReverseSegment 실행 중
        float laneOffset = 0.0f;       // 회피 시작 시점의 원래 차로 중심 d -- 재계획 기준점(방향/크기 계산용)
        float avoidOffset = 0.0f;      // 회피 목표 d
        float blockedTimer = 0.0f;     // 전방이 막힌 채로 지난 시간 (진입 트리거용)
        float clearTimer = 0.0f;       // 레이가 깨끗한 채로 지난 시간 (회피 종료 디바운스용)
        float lastPlanTime = -1000.0f; // 마지막으로 오프셋을 (재)탐색한 시각 -- 재계획 디더링 방지
        float laneChangeTarget = 0.0f; // D_LaneChange일 때 목표 차로 중심 d
    };

    // 움직이는 장애물이 진로를 비울 때까지 그 앞에서 정지 대기하는 상태(D_WaitObstacle).
    struct WaitState
    {
        float elapsed = 0.0f; // 대기 누적 시간 -- AVOID_WAIT_TIMEOUT을 넘으면 '안 비켜준다'로 보고 정적 취급
        // 타임아웃으로 정적 취급 중. 이 대상이 시야에서 사라질 때까지 유지해야 한다 -- 안 그러면 대기를
        // 푼 다음 프레임에 같은 대상이 다시 Dynamic으로 분류돼 대기/타임아웃을 무한 반복한다.
        bool timedOut = false;
    };

    // 전방 최근접 위협을 무엇으로 볼 것인가. 대상마다 대응이 완전히 갈리므로 여기서 한 번만 분기한다.
    enum class ThreatKind
    {
        None,    // 위협 없음 또는 판단 보류(속도 임계 사이) -- IDM 추종에 맡긴다
        Vehicle, // 차 -- IDM(종방향)/MOBIL(횡방향)이 이미 처리한다
        Dynamic, // 움직이는 장애물 -- 정지 원칙(TTC가 넉넉할 때만 통과)
        Static,  // 멈춰 있는 장애물 -- 차선변경 또는 오프셋 회피
    };

    // Drive 서브모드별 틱. UpdateDrive가 m_subMode로 갈라 부른다 -- 어느 함수가 도는지가 곧 현재 상태다.
    void UpdateSensors();                   // (공통) 장애물 목록 수집 + 레이 스캔 + 차체접촉/교차충돌 예측
    bool HandleContactPending();            // (공통) 실제 물리 충돌 뒷수습. 서브모드와 무관하게 먼저 끊는다
    bool UpdateBackupState();               // (공통) 후진 매뉴버 진행/종료. 도는 동안 서브모드 틱을 건너뛴다
    void DecideAvoidance();                 // D_Normal: 위협을 분류해 다음 서브모드를 고른다
    void UpdateWaitObstacle();              // D_WaitObstacle: 정지 대기 유지/해제
    void UpdateAvoid();                     // D_Avoid: 오프셋 회피 재계획/복귀/종료
    void UpdateLaneChange();                // D_LaneChange: 차선변경 진행/취소/완료
    void HandleAvoidStuck();                // 회피 불가: 그 자리 정지 + 뒤가 비었으면 후진(최초 탐색/재탐색 실패가 공유)
    ThreatKind ClassifyFrontThreat() const; // 전방 최근접 히트를 차/동적/정적으로 분류
    bool IsDynamicThreatClear() const;      // 동적 장애물이 내가 닿기 전에 진로를 비우는가(TTC)
    // 회피(오프셋/차선변경)가 지금 실제로 향하고 있는 횡오프셋. 진행 중이 아니면 현재 오프셋.
    // 궤적을 굴려보는 쪽(PredictBodyContact)과 Lerp하는 쪽이 같은 목표를 봐야 한다 -- 어긋나면
    // 빠져나가는 중인 궤적을 충돌로 잘못 보고 스스로 제동한다.
    float AvoidTargetOffset() const;
    // 그 밴드(차로) 앞쪽에 정지 장애물이 걸쳐 있지 않은가. '거기까지 가는 궤적이 뚫렸나'가 아니라
    // '그 차로 자체가 뚫렸나'를 본다 -- 3차로에서 1차로로 갈 때 중간 2차로가 막혔다고 1차로까지
    // 못 쓰는 차로로 판정해버리면 갈 곳이 없어져 후진만 반복한다.
    bool IsBandClearAhead(float bandCenter, float bandHalfWidth) const;
    // 그 차로에 들어갔을 때 앞차/뒤차를 모두 감당할 수 있는가. IsSafeLaneEntry(뒤차 기준)에 앞차 gap 검사를
    // 더한 것. 차선변경 결정 자체는 MOBIL(ComputeLateralTarget)이 다 하므로, 여기서는 진행 중인
    // D_LaneChange가 도중에 막혔는지(UpdateLaneChange) 확인하는 용도로만 쓰인다.
    bool IsLaneEntryClear(float targetOffset) const;
    Vec3 GetBodyCenter() const;                                           // 콜라이더(차체 사각)의 중심 -- 레이 원점/OBB 판정 기준
    std::vector<VehicleCollision::Obstacle> BuildSensorObstacles() const; // 정적 장애물 + 주변 차를 OBB 목록으로
    // 지도 정적/동적 장애물 중 센서 사거리(AVOID_FRONT_RAY_MAX) 안에 든 것만. BuildSensorObstacles가
    // 레이캐스트 대상(장애물+차) 목록을 만들 때 쓰는 전반부.
    std::vector<VehicleCollision::Obstacle> CollectMapObstaclesInSensorRange() const;
    SensorScan ScanSensors(const std::vector<VehicleCollision::Obstacle> &obstacles) const;
    // 좌/우 후보 오프셋(차로 반폭 -> 한 폭 -> 두 폭)을 가까운 쪽부터 궤적 시뮬레이션해 첫 무충돌 오프셋을 고른다.
    bool FindAvoidOffset(const SensorScan &scan, const std::vector<VehicleCollision::Obstacle> &obstacles,
                         float laneCenter, float &outOffset) const;
    // targetOffset으로 Lerp해 들어가는 궤적을 자전거 모델로 굴려, 차체(OBB)가 처음 겹치는 지점까지의
    // 주행거리를 반환(maxDistance까지 안 겹치면 -1). 아래 둘이 공유하는 코어.
    float SweepBodyPath(float targetOffset, const std::vector<VehicleCollision::Obstacle> &obstacles,
                        float speed, float maxDistance) const;
    // 회피 후보 검증: targetOffset으로 들어가는 궤적이 예측시간 내내 무충돌이면 true.
    bool SimulateAvoidPath(float targetOffset, const std::vector<VehicleCollision::Obstacle> &obstacles) const;
    // 지금 주행선을 따라갔을 때 실제 차체가 정지 장애물에 처음 닿는 거리(없으면 -1). 제동/회피 트리거용.
    float PredictBodyContact(const std::vector<VehicleCollision::Obstacle> &obstacles) const;
    // 움직이는 장애물과 서로를 등속으로 굴려 만나는 지점까지 내가 갈 거리(없으면 -1). 레이 코리도 필터는
    // '지금' 진로 위인 것만 통과시켜서 비스듬히 들어오는 대상을 코앞까지 못 보는데, 그걸 메운다.
    float PredictMovingConflict(const std::vector<VehicleCollision::Obstacle> &obstacles,
                                const VehicleCollision::Obstacle **outObstacle) const;
    void RebuildSensorRender();
#pragma endregion

public:
    static constexpr float ARRIVE_DISTANCE = 5.0f;
    static constexpr float PARK_ARRIVE_DISTANCE = 10.0f;
    static constexpr float LANE_TRANSITION_THRESHOLD = 2.0f; // 다음 차선으로 완전히 넘어가는(전환되는) 임계값

private:
    // 설정 및 스펙 상수/변수 (Constants & Specifications)
    const float m_maxSpeed = 200.0f / 3.6f;           // 200 km/h
    const float m_maxAccel = (100.0f / 3.6f) / 14.0f; // 0-100 km/h in 14s
    const float m_maxBrake = (100.0f / 3.6f) / 3.0f;  // 100-0 km/h in 3s
    float m_speedGain = 2.0f;                         // 속도오차 -> 목표가속 비례게인
    float m_jerkUp = 4.0f;                            // 가속 방향 저크 상한 (m/s^3). Init에서 m_personality로 덮어씀
    float m_jerkDown = 15.0f;                         // 제동 방향 저크 상한. Init에서 m_personality로 덮어씀
    CarPersonality m_personality;                     // 운전자 성격(CarSpec::personality) -- IDM/MOBIL 파라미터에 반영, 디버그 UI로 조절

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
    LaneDirection m_destDir = LaneDirection::Forward; // destRoad를 어느 방향으로 달려 도착하는가(도착 판정 끝점)
    shared_ptr<Road> m_currentRoad;
    LaneDirection m_travelDir = LaneDirection::Forward; // 현재 road의 진행방향. d는 이것과 무관하게 항상 참조선 프레임.
    float m_currentOffset = 0.0f;                       // 계획된(committed) 횡오프셋 d -- 실측이 아니라 리플랜 기준 상태
    Spline m_currentSpline;                             // 현재 주행 스플라인(참조선 offset d, 역방향이면 뒤집힌 것) 캐시

    shared_ptr<RoadNode> m_parkSpot;        // 예약된 목표 주차칸(있는 동안은 "이 자리에 주차 중/주차 예정")
    shared_ptr<RoadNode> m_pendingParkNode; // 예약 전, 도착하면 그때 주차칸을 예약할 목표 Park 노드
    bool m_parkSequenceActive = false;
    int m_parkNodeId = -1;                 // 이번 입차의 대상 Park 노드 id (다른 빈 자리 재예약에 씀)
    int m_parkLegTries = 0;                // 현재 스팟의 진입 leg 체인 시도 횟수 (무한 체인 방지)
    unordered_set<int> m_triedParkSpotIds; // 이번 입차에서 경로탐색이 실패해 이미 시도해본 ParkSpot id들
    bool m_parkPlanPending = false;

    bool m_roaming = true;                         // 배회 모드: 목적지 없이 랜덤 후속 road로 계속 주행
    static constexpr size_t ROAMING_MIN_AHEAD = 3; // 배회 시 현재 road 앞으로 항상 유지할 최소 road 버퍼 수

    vector<RoadRef> m_path;
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
    float m_planAccelDebug = 0.0f;                  // DriveControl이 매프레임 계산한 IDM 목표가속도(디버그 UI 표시용 캐시)
    std::vector<NearbyCar> m_lastNearbyCars;        // UpdateBehaviorPlan이 마지막으로 수집한 주변 차 목록 -- ShouldHoldForMerge가 매 프레임 재사용(재수집 비용 회피)
    std::vector<RoadSpeedSample> m_lastRoadSamples; // UpdateBehaviorPlan이 마지막으로 스캔한 리더/제약 목록 -- DriveControl이 매프레임 IDM 가속도 재계산에 재사용
    IDM::Params m_lastIdmParams;                    // 위 스캔 시점의 IDM 파라미터(v0 등) 캐시
    Vec3 m_planScanPosition = Vec3::sZero();        // 위 스캔 시점의 ego 위치 -- 정적 제약 gap을 매프레임 보정(distanceOffset)하는 기준

    // 장애물 회피(Avoid). 레이 스캔은 IDM/MOBIL의 0.2초 주기와 달리 매프레임 돈다 -- 갑자기 끼어든 차나
    // 차로 코리도 판정으로는 안 잡히는 정적 장애물에 프레임 단위로 반응해야 하기 때문.
    static constexpr float AVOID_BLOCK_SPEED = 1.0f; // 이 속도 이하로 움직이는 대상은 '길을 막고 있다'고 본다(m/s)
    // 이 속도 이상이면 '움직이는 장애물'(정지 원칙 대상). AVOID_BLOCK_SPEED와 이 값 사이는 판단 보류라
    // 정지/회피 어느 쪽으로도 안 튄다 -- 임계값 하나면 그 근처에서 분류가 매 프레임 뒤집힌다.
    static constexpr float AVOID_DYNAMIC_SPEED = 1.5f;
    // 최대 조향이 가능한 저속(CalcMaxSteerAngle의 LOW_SPEED_CUTOFF와 같은 값). 위협 판단/회피 중 속도 상한.
    static constexpr float AVOID_LOW_SPEED = 18.26f / 3.6f;
    static constexpr float AVOID_TTC_MARGIN = 1.5f;       // 동적 장애물이 진로를 비우는 시각보다 이만큼 더 늦게 닿아야 그냥 통과(s)
    static constexpr float AVOID_WAIT_TIMEOUT = 5.0f;     // 정지 대기가 이만큼 이어지면 '안 비켜준다'로 보고 정적 취급(회피 시도)
    static constexpr float AVOID_TRIGGER_DELAY = 0.6f;    // 전방이 이만큼 계속 막혀 있어야 회피 시작(s) -- 순간 오검출로 안 흔들리게
    static constexpr float AVOID_CLEAR_DELAY = 0.5f;      // 레이가 이만큼 계속 깨끗해야 원래 차로로 복귀(s)
    static constexpr float AVOID_REPLAN_INTERVAL = 0.5f;  // 회피 중 오프셋 재탐색 최소 간격(s) -- 좌/우 진동 방지
    static constexpr float AVOID_RETURN_TOLERANCE = 0.3f; // 복귀 목표 오프셋에 이만큼 가까워지면 회피 종료(m)
    static constexpr float AVOID_MIN_SHIFT = 0.5f;        // 이보다 작은 횡이동은 회피 효과가 없다고 보고 후보에서 뺀다(m)
    // 전방 히트를 '내 진로 위'로 볼 기준: 주행 스플라인까지의 거리가 차체 반폭 + 이 여유 이내(m).
    // 회피를 걸지 말지(frontBlocked) 판단용이라 넉넉하게 -- 결과가 '옆으로 조금 비켜라'라서 싸다.
    static constexpr float AVOID_CORRIDOR_MARGIN = 0.5f;
    // 종방향 제동(IDM 가상 리더/비상제동)을 걸 기준 여유(m). 위 트리거 코리도보다 훨씬 타이트해야 한다 --
    // 넓게 잡으면 회피로 옆을 스치듯 통과하는 중인 장애물에도 제동이 걸리고, IDM은 s0(MIN_SAFE_GAP)만큼
    // 앞에서 서려 하므로 그 지점에 갇혀 영영 못 지나간다. 반대로 너무 좁히면 실제로 스칠 것에 안 선다.
    static constexpr float AVOID_PASS_CLEARANCE = 0.2f;
    static constexpr float AVOID_SIM_TIME = 3.0f;      // 회피 후보 검증의 예측 시간(s)
    static constexpr float AVOID_SIM_MIN_SPEED = 3.0f; // 정지 중에도 앞으로 굴려보기 위한 최소 가정 속도(m/s)
    // 차체 스윕 적분 스텝 수(거리 기준으로 등분). 스텝 간격이 차 길이보다 작게 유지되므로 얇은 장애물을
    // 건너뛰지 않고, 저속에서도 스텝 수가 폭발하지 않는다.
    static constexpr int AVOID_SWEEP_STEPS = 16;
    static constexpr float AVOID_FRONT_RAY_MIN = 8.0f;  // 전방 레이 최소 길이(정지 중에도 바로 앞은 보이게, m)
    static constexpr float AVOID_FRONT_RAY_MAX = 30.0f; // 전방 레이 최대 길이(m)
    // 정면 중앙 레이만 따로 더 멀리 본다. 여기 잡힌 장애물은 IDM 가상 리더 + MOBIL의 현재 차로 리더로
    // 들어가서, 코앞까지 가서 회피로 비트는 대신 한참 전에 정상 차선변경으로 빠져나가게 한다.
    // (회피 트리거 자체는 여전히 AVOID_FRONT_RAY_MAX 이내 히트만 쓴다 -- 60m 밖 장애물에 미리 몸을
    //  틀어버리면 안 되므로.)
    static constexpr float AVOID_FRONT_RAY_FAR_TIME = 5.0f; // 이 시간만큼 앞을 미리 본다(s)
    static constexpr float AVOID_FRONT_RAY_FAR_MAX = 60.0f; // 그 상한(m)
    // 대각/측면 레이 길이(m). "바로 옆 틈이 비었나"만 보는 값싼 사전 거부용이라 옆차로까지 닿게 잡지
    // 않는다 -- 옆차로 차와 부딪히는지는 SimulateAvoidPath의 OBB 판정이 어차피 본다. 길게 잡으면
    // 옆차로에 차가 있는 내내 sideNear가 서서 회피 오프셋에서 못 빠져나온다.
    static constexpr float AVOID_SIDE_RAY_LENGTH = 2.5f;
    // 이 이상 꺾여 있어야 "그쪽으로 돌고 있다"고 보고 같은 쪽 대각선 히트를 막힘으로 승격시킨다.
    // 직진 중 미세한 조향 떨림에 대각선이 계속 막힘으로 잡히는 걸 막는 데드존.
    static constexpr float AVOID_STEER_DEADZONE = ToRadians(3.0f);
    static constexpr float AVOID_REAR_RAY_LENGTH = 6.0f; // 후방 레이 길이(m)
    // 후진해도 되는지 볼 때 물러날 거리 뒤로 더 남아 있어야 하는 여유(m). 저속 직선 후진이라 주행 중
    // 표준 간격(MIN_SAFE_GAP)만큼 잡을 필요는 없다 -- 너무 크면 물러날 자리가 있어도 스턱에서 못 벗어난다.
    static constexpr float AVOID_BACKUP_CLEARANCE = 1.0f;

    // ApplyMotion(물리 틱)이 실제 충돌을 감지하면 세우고, HandleContactPending(프레임 틱)이 소비하며 지운다.
    // 물리/판단이 서로 다른 틱이라 즉시 처리 대신 이렇게 걸어둔다.
    bool m_contactPending = false;

    SensorScan m_sensor; // 이번 프레임 레이 스캔 결과 (UpdateSensors가 갱신)
    AvoidState m_avoid;
    WaitState m_wait;
    // 이번 프레임 레이/OBB 판정 대상 목록. UpdateSensors가 만들고 UpdateBehaviorPlan(차선변경 궤적 검증)이 재사용.
    std::vector<VehicleCollision::Obstacle> m_sensorObstacles;
    // 위에서 움직이는 대상을 뺀 것. 궤적 스윕(SweepBodyPath)은 상대를 정지한 것으로 보므로, 같이 굴러가는
    // 차까지 넣으면 어떤 차선변경도 항상 충돌로 나온다 -- 움직이는 차는 MOBIL/IDM이 맡는다.
    std::vector<VehicleCollision::Obstacle> m_stationaryObstacles;
    // 이번 프레임 종방향 속도 상한(m/s). <0이면 제한 없음. 서브모드 틱이 정하고 DriveControl이 IDM 가상
    // 리더로 걸어 부드럽게 수렴시킨다 -- IDM 자체를 끄면 신호/앞차/제한속도 제약이 같이 사라진다.
    float m_speedCap = -1.0f;

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
    RenderObject m_sensorRender;     // 회피 감지 레이 다발
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