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
#include "Nav/VehicleCollision.h"

class Car;
class SimulationState;
class Spline;
class VehicleSegment;

// 레이싱 라인 위로 투영해 찾은 "같은 라인 앞차". 종방향 갭 유지의 기준점이자,
// 이후 추월 판단(오버랩 확보 여부, 기회 탐색)이 얹힐 자리다.
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

    // 트랙 전체 레이싱 라인을 전략별로 한 번 풀어 SimulationState에 넣는다.
    // 도로를 다 읽은 뒤, 차를 스폰하기 전에 한 번만 부를 것.
    static void BuildSharedRaceLines(SimulationState &simState);
    // 차 인스턴스 없이도 레이싱 라인을 그릴 수 있도록 렌더 오브젝트만 따로 뽑아내는 헬퍼.
    // Car::RebuildRacingLineRender와 CarSim의 상시 표시가 이 하나를 같이 쓴다.
    static void BuildRaceLineRenderObject(const RaceLine &line, const std::string &modelKey,
                                          RenderObject &out,
                                          const DirectX::XMFLOAT4 &color = DirectX::XMFLOAT4(0.0f, 0.3f, 1.0f, 1.0f));
    // 공유 라인을 만들 때 쓴 횡방향 여유(차선 반폭 - 최대 차폭 - 여유).
    // 추월 오프셋 클램프도 같은 값을 써야 라인이 도로 밖으로 안 나간다.
    static float SharedLineRoom();

#pragma region RaceTelemetry
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

    // 충돌 로그용 스냅샷
    float GetRaceS() const { return m_raceS; }
    float GetRaceD() const { return m_raceD; }
    float GetRaceCurvature() const { return m_raceCurvature; }
    float GetLapLength() const { return m_lapLength; }
    const char *GetOvertakeStateName() const { return OvertakeStateToString(m_overtakeState); }
    float GetOvertakeSide() const { return m_overtakeSide; }
    float GetRaceLateralOffset() const { return m_raceLateralOffset; }
    const Car *GetOvertakeTarget() const { return m_overtakeTarget; }
    // 후방충돌 진단용 -- IDM이 이 차를 아예 못 봤는지, 봤는데 늦었는지 가르는 값
    const Car *GetLeaderCar() const { return m_leader.car; }
    float GetLeaderGap() const { return m_leader.gap; }
    float GetLeaderClosing() const { return m_leader.closingSpeed; }
    const Car *GetYieldTarget() const { return m_yieldTarget; }
    // 추월이 왜 안 일어나는지 집계용 -- 어떤 게이트가 막고 있는지, 시도/포기가 몇 번인지
    const char *GetOvertakeBlock() const { return m_overtakeBlock; }
    int GetOvertakeAttempts() const { return m_overtakeAttempts; }
    int GetOvertakeAborts() const { return m_overtakeAborts; }
    int GetAbortSetup() const { return m_abortSetup; }
    int GetAbortClearance() const { return m_abortClearance; }
    int GetAbortStall() const { return m_abortStall; }
    int GetOvertakeSuccess() const { return m_overtakeSuccess; }
    JPH::BodyID GetBodyID() const { return m_rigidbody.GetBodyID(); }
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
    void Steer(float desiredRadian, float steerRamp = 1.0f);
    void ChangeGear();
    bool IsReverse() const { return m_isReverse; }

    void DriveControl();

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
    void RebuildRacingLineRender();

#pragma region FSM
    enum class Mode
    {
        Stop,
        Drive
    };

    const char *Car::StateToString(Mode mode) const
    {
        switch (mode)
        {
        case Mode::Stop:
            return "Stop";
        case Mode::Drive:
            return "Drive";
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
        D_Pursuit, // 반응형 조향 주행(항상 사용)
    };
    const char *Car::SubStateToString(SubMode subMode) const
    {
        switch (subMode)
        {
        case SubMode::None:
            return "None";
        case SubMode::D_Pursuit:
            return "Pursuit";
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
    void SetCurrentRoad(const shared_ptr<Road> &road, float offset);
    void SetCurrentOffset(float offset); // 오프셋+밴드 동기화

    RoadRef CurrentRoadRef() const { return RoadRef{m_currentRoad}; }
    bool UsesReactiveSteer() const { return m_subMode == SubMode::D_Pursuit; }

    void EnsureRoamingPath();
    void MaintainRoamingPath();
    RoadRef PickRandomSuccessor(const RoadRef &road) const;
    vector<RoadRef> BuildRoamingPath(const RoadRef &startRoad) const;

    void UpdateStop();

    void UpdateDrive();
    bool CheckPath();
    bool IsOutsideDrivingBands() const;
    bool CanEnterFromCurrentBand(const RoadRef &next) const;
    bool RepathFromCurrentBand();

    void OnLapCompleted();
    // 직전 물리 스텝에서 생긴 접촉을 읽어 기록한다(Physics.Update가 매 스텝 Clear하므로 먼저 읽어야 한다)
    void DetectContact();

    void UpdateHorn(float dt);
    bool IsHornSituation() const;
    bool KnowsRedSignalAhead() const;

#pragma endregion

#pragma region BehaviorPlan
    void UpdateDrivePlan();
    float RoadTargetSpeed(const shared_ptr<Road> &road, float maxSpeed) const; // 도로 제한속도(최고속 캡)
    void ComputeDrivableRange(const RoadRef &road, float &outMin, float &outMax) const;
#pragma endregion

#pragma region Race
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
#pragma endregion

#pragma region Avoid
    void UpdateSensors(); // 장애물 목록 수집
    // 레이싱라인 조향을 유지하되, 짧은 시간 안에 실제 차체 충돌이 예측될 때만 최소 보정
    float ComputeContextSteer(const Vec3 &pursuitTarget, float pursuitSteer) const;
    Vec3 GetBodyCenter() const;                             // OBB 판정 기준점
    VehicleCollision::Obstacle MakeVehicleObstacle() const; // 이 차의 차체 OBB를 장애물 하나로
    void RebuildCtxSteerRender();
#pragma endregion

private:
    // Track-focused GT racing specification.
    // 기준 스펙. 개체별 편차는 CarPersonality가 여기에 곱해진다(Init에서 1회 적용).
    static constexpr float BASE_MAX_SPEED = 320.0f / 3.6f;          // 최고속도 320 km/h
    static constexpr float BASE_MAX_ACCEL = (100.0f / 3.6f) / 3.0f; // 0-100 km/h 약 3.0초
    static constexpr float BASE_MAX_BRAKE = (100.0f / 3.6f) / 1.8f; // 100-0 km/h 약 1.8초 (약 1.57 g)
    static constexpr float BASE_GRIP_ACCEL = 15.0f;                 // 타이어 마찰원 반경(약 1.53 g)
    // 그립 한계가 조향 포화(15.0)와 코너 목표속도(8.0) 두 군데에 따로 박혀 있었다. 하나로 묶되
    // 원래 튜닝을 유지하려고 그 비율을 "계획은 타이어 한계를 다 쓰지 않는다"는 마진으로 명시한다.
    static constexpr float PLAN_GRIP_RATIO = 8.0f / 15.0f;

    // 슬립스트림: 앞차 와류 안에서는 항력이 줄어 최고속과 가속이 함께 올라간다.
    // 항력 모델을 따로 두지 않고 상한에 배수를 먹이는 근사다.
    // 추종 차간(LeaderFollowAccel의 TIME_HEADWAY)보다 넉넉히 넓어야 한다. 좁으면 IDM이
    // 와류 밖에 차를 세워둬서 토우가 영영 걸리지 않는다. 320km/h * 0.3s = 약 27m가 기준.
    // 차간을 0.45초로 늘리면서 평형 갭이 320km/h에서 약 42m가 됐다. 범위가 60m면
    // 토우가 0.3밖에 안 걸려 따라잡지를 못한다(로그: 추월 차단 사유의 69%가 "갭이 멀다").
    static constexpr float SLIPSTREAM_RANGE = 90.0f;          // 이보다 뒤면 와류가 닿지 않는다
    static constexpr float SLIPSTREAM_LATERAL_SPAN = 2.5f;    // 차폭 합의 몇 배까지 옆으로 남는가
    static constexpr float SLIPSTREAM_MIN_SPEED = 30.0f;      // m/s. 항력은 v^2이라 저속에선 무의미
    static constexpr float SLIPSTREAM_TOP_SPEED_GAIN = 0.06f; // 바로 뒤에 붙었을 때 최고속 +6%
    static constexpr float SLIPSTREAM_ACCEL_GAIN = 0.25f;     // 같은 조건에서 가속 여유 +25%
    static constexpr float SLIPSTREAM_RESPONSE = 3.0f;        // 와류가 붙고 떨어지는 응답(1/s)

    // 추종 갭. 추월 트리거도 이 값을 기준으로 잡아야 personality/속도에 같이 따라간다.
    static constexpr float FOLLOW_STANDSTILL_GAP = 1.0f; // 정지 시 유지할 최소 갭(m). 레이싱이라 짧다
    // 0.5s는 일반도로 값이라 320km/h에서 45m가 나온다 -- SLIPSTREAM_RANGE 밖이라
    // 토우가 절대 안 걸린다. 실제 레이싱의 추종 간격(0.2~0.3s)에 맞춰 좁혔다.
    static constexpr float FOLLOW_TIME_HEADWAY = 0.2f; // 목표 차간시간(s)

    // 추월 판단
    // 얼리 에이펙스 라인이 이만큼은 안쪽으로 들어가야 "다이브할 코너"로 본다.
    // 측정값: 직선에서 두 라인 차이는 0.03m, 코너 평균 1.33m, 최대 3.61m.
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
    // 명령한 오프셋과 실제 위치는 다르다. 로그를 보면 4.3m를 명령했는데 실제로는 2.0m만
    // 벌어진 채로 나란히 서서 부딪혔다(달성률 28~56%). 그래서 "실제로 이만큼 벌어졌는가"를
    // 확인한 뒤에야 붙기 시작한다. 오버랩 판정 폭(반폭 합 + 0.3)보다 0.4m 더 확보한다.
    // leader 판정 임계(LATERAL_MARGIN)보다 커야 한다. 작으면 Setup을 통과하고도
    // 여전히 leader로 잡혀 앞차 뒤에 눌린 채 옆에만 서 있게 된다.
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

    float m_maxSpeed = BASE_MAX_SPEED;
    float m_maxAccel = BASE_MAX_ACCEL;
    float m_maxBrake = BASE_MAX_BRAKE;
    float m_gripAccel = BASE_GRIP_ACCEL; // 조향 포화와 마찰원이 공유하는 그립
    // 슬립스트림이 얹힌 현재 속도 상한. m_maxSpeed로 직접 클램프하면 와류 이득이 잘려나간다.
    float m_speedCap = BASE_MAX_SPEED;
    float m_speedGain = 3.5f; // 목표속도 변화에 빠르게 풀스로틀/브레이크
    float m_jerkUp = 30.0f;   // 가속 저크 상한 (m/s^3)
    float m_jerkDown = 60.0f; // 제동 저크 상한 (m/s^3)
    CarPersonality m_personality;

    float m_wheelbase = 0.0f;
    float m_mass = 1.0f;
    Vec3 m_halfExtents = Vec3::sZero();       // x=반폭 z=반길이
    float m_maxSteerAngle = ToRadians(35.0f); // 저속 최대 조향각

    float m_speed = 0.0f;
    float m_acceleration = 0.0f;
    float m_steerAngle = 0.0f;
    bool m_isReverse = false;
    float m_deltaTime = 0.0f;

    bool m_wantSegmentTick = false;
    bool m_isFocused = false;
    bool m_isControl = false; // true면 수동조작
    int m_id = -1;
    float m_aiStartDelay = 0.0f; // Init에서 지정, 이 시간까지 AI 정지
    float m_aiDelayTimer = 0.0f;

    SimulationState *m_SimState = nullptr;
    Mode m_mode = Mode::Stop;
    SubMode m_subMode = SubMode::None;
    VehicleController m_vehicleController; // 계획 세그먼트 실행
    shared_ptr<Road> m_currentRoad;
    float m_currentOffset = 0.0f;            // 실측아닌 계획값
    const LaneBand *m_currentBand = nullptr; // SetCurrentOffset이 갱신
    Spline m_currentSpline;                  // 오프셋 d 스플라인 캐시

    static constexpr size_t ROAMING_MIN_AHEAD = 3; // 앞으로 미리 정해둘 도로 수

    vector<RoadRef> m_path;
    size_t m_pathIndex = 0;
    float m_currentTime = 0.0f;

    static constexpr float HORN_STOP_SPEED = 0.3f; // 정지 기준 속도(m/s)
    float m_hornStoppedDuration = 0.0f;            // 막혀서 정지해 있던 누적 시간
    float m_hornFlashTimer = 0.0f;                 // 남은 빨간표시 시간
    Model *m_carModel = nullptr;                   // 경적시 색 변경용

    static constexpr float BEHAVIOR_PLAN_INTERVAL = 0.2f; // 행동계획 주기

    float m_lastBehaviorPlanTime = -1000.0f; // 첫 판단 즉시 실행되게
    float m_planAccelDebug = 0.0f;           // 매프레임 가속도
    std::vector<VehicleCollision::Obstacle> m_obstacles;
    std::vector<Car *> m_nearbyCars; // 재수집 비용 회피 캐시
    LeaderInfo m_leader;             // 매 DriveControl마다 갱신
    // 레이스 계측
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
    Car *m_lastContactCar = nullptr;
    float m_lastContactTime = -1000.0f;

    float m_slipstream = 0.0f;         // 감쇠를 거친 현재 와류 세기
    float m_slipstreamGapDebug = 0.0f; // 와류를 준 차와의 갭(디버그)

    OvertakeState m_overtakeState = OvertakeState::None;
    Car *m_overtakeTarget = nullptr;                  // Attack 중 추적 대상. leader에서 빠져도 계속 본다
    float m_overtakeSide = 0.0f;                      // -1=왼쪽, +1=오른쪽. 진입 시 한 번 정하고 유지한다
    float m_overtakeTimer = 0.0f;                     // 현재 상태 유지 시간
    float m_overtakeCooldown = 0.0f;                  // 포기 후 재시도 금지 잔여 시간
    float m_overtakeBestDeltaS = 0.0f;                // Attack 중 관측한 최소 상대 s(진전 판단용)
    float m_overtakeStall = 0.0f;                     // 진전 없이 흐른 시간
    const char *m_overtakeBlock = "";                 // 시작을 막고 있는 조건(디버그)
    int m_overtakeAttempts = 0;                       // Follow -> Setup 진입 횟수
    int m_overtakeAborts = 0;                         // 완주 못 하고 Return으로 접은 횟수
    int m_abortSetup = 0;                             // 옆으로 못 벌려서
    int m_abortClearance = 0;                         // 벌린 간격을 다시 잃어서
    int m_abortStall = 0;                             // 진전이 없어서/시간 초과
    int m_overtakeSuccess = 0;                        // 실제로 앞질러서 끝난 횟수
    bool m_overtakeHoldOffset = false;                // 물러나는 중 겹침이 안 풀려 오프셋을 붙잡고 있다
    bool m_overtakeBackOff = false;                   // 레거시 UI 플래그. 종방향 back-off는 사용하지 않는다
    Car *m_yieldTarget = nullptr;                     // 내 뒤에서 권리를 얻어 공간을 보장 중인 추월차
    float m_yieldSide = 0.0f;                         // 추월차가 들어온 쪽(-1=왼쪽, +1=오른쪽)
    float m_raceLateralOffset = 0.0f;                 // 레이싱 라인 기준 목표 횡오프셋(감쇠 적용된 현재값)
    bool m_raceLateralOffsetInitialized = false;      // 시작 위치를 0으로 당기지 않도록 실제 d로 한 번 초기화
    mutable float m_ctxSteerDebugRad = 0.0f;          // 반응형 조향이 고른 슬롯각
    mutable float m_ctxSteerDebugDanger = 0.0f;       // 그 슬롯의 danger
    mutable std::vector<Vec3> m_ctxSteerOpenLines;    // 안전 슬롯 레이(초록)
    mutable std::vector<Vec3> m_ctxSteerBlockedLines; // 위험 슬롯 레이(빨강)

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
    RenderObject m_ctxSteerOpenRender;    // 반응형 조향 안전 레이(디버그)
    RenderObject m_ctxSteerBlockedRender; // 반응형 조향 위험 레이(디버그)
    RenderObject m_racingLineRender;      // 레이싱 라인(디버그)
    const RaceLine *m_raceLine = nullptr; // SimulationState가 소유한 공유 라인
    // 추월할 때 참고하는 얼리 에이펙스 라인. 갈아타는 게 아니라 "얼마나 안쪽으로
    // 파고들 수 있는가"의 기준으로만 쓴다 -- 라인만 바꿔서는 간격이 안 나온다.
    const RaceLine *m_attackLine = nullptr;
    size_t m_raceLineCursor = 0; // 최근접점 탐색 시작 위치(전체 훑기 방지)
    ApexStrategy m_apexStrategy = ApexStrategy::Late;
    ApexStrategy m_boundStrategy = ApexStrategy::Count; // m_raceLine이 가리키는 전략
};

// gripAccel은 Car::m_gripAccel(개체별 편차 반영)을 받는다. 그립 좋은 차가 고속에서
// 더 큰 조향각까지 쓸 수 있어야 코너 통과속도 차이가 실제 궤적으로 나타난다.
static float CalcMaxSteerAngle(float speed, float wheelbase, float gripAccel)
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
