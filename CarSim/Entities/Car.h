#pragma once
#include "Core/GameObject.h"
#include "CarSpec.h"
#include "VehicleController.h"
#include <cmath>
#include <deque>
#include <string>
#include <array>
#include <Nav/RoadDataManager.h>
#include "Nav/ReedsShepp.h"
#include "Nav/HybridAStar.h"

class Car : public GameObject
{
public:
    // 생명주기 및 초기화 (Lifecycle & Initialization)
    void Init(const CarSpec &spec, RoadDataManager *roadDataManager, JPH::Vec3 position = JPH::Vec3::sZero());

    // 부모 클래스 오버라이드 함수 (Overrides)
    void Update(float dt) override;
    void Draw(ID3D11DeviceContext *context, IEffect &effect) override;

    Vec3 GetPosition() const override;
    Vec3 GetForwardAxis() const;
    void SetPosition(Vec3 position) override;
    void SetRotation(const DirectX::XMFLOAT4 &rotation) override;

    // Getter / Setter (Accessors)
    void SetFocused(bool focused) { m_isFocused = focused; }
    void SetDestination(const shared_ptr<RoadNode> &parkNode);

    // 조작 및 제어 인터페이스 (Control Interface)
    void Accelerate(float desiredVelocity);
    void EmergBrake();
    void Steer(float desiredRadian, float steerRamp = 0.4f);
    void ChangeGear(); // 속도가 낮을 때 전진/후진 기어 토글
    float GetSpeed() const { return m_speed; }
    float GetDeltaTime() const { return m_deltaTime; }
    bool IsReverse() const { return m_isReverse; }
    // Steer()가 램프 중인 실제 현재 조향각 (목표각과 다를 수 있음 - ArcMoveSegment가 정렬 여부 판단에 사용).
    float GetSteerAngle() const { return m_steerAngle; }

    // VehicleSegment(SplineFollowSegment)가 호출하는 정속주행 제어 한 틱. VehicleController를
    // 통해서만 호출되도록 의도된 것이라 AI 흐름 밖에서 직접 부르지 않는다.
    void DriveControl();

private:
    // 내부 물리 및 제어 로직 (Internal Physics & Control)
    void UpdateCar();
    void UpdateWithControl();
    void ApplyMotion();
    float GetSignedSpeed() const { return m_speed * (m_isReverse ? -1.0f : 1.0f); }
    JPH::Vec3 ComputeDesiredVelocity() const;

    float PurePursuit(Vec3 target);

    // 디버그 및 트레일(자국) 렌더링 (Debug & Rendering Helpers)
    void UpdateDebugWindow();
    // 1~4초 뒤 목표 속도/위치를 보여주는 ImGui 창. 목표 위치는 노란 구 마커로도 표시한다.
    void UpdateSpeedProfileWindow();
    void UpdateTrail();
    void RebuildTrailRender(RenderObject &render, const std::deque<DirectX::XMFLOAT3> &trail,
                            const std::string &name, const DirectX::XMFLOAT4 &color);
    void RebuildSplineRender();
    // Park 계획(OnModeEnter(Park))이 세워질 때 RS 경로 폴리라인 + 목표 위치/방향을 바닥에 그린다.
    void RebuildParkDebugRender(const ReedsShepp::Path &path, const Vec3 &startPos, float startAngleDeg,
                                float turningRadius, const Vec3 &targetPos, float targetAngleDeg);

    bool IsOffCourse();

    // 현재 향하고 있는 park 목표 노드. 예약이 이미 됐으면 m_parkSpot, 아직이면 m_pendingParkNode.
    // 어느 쪽이든 "park 관련 목적지가 있다"만 확인하면 되는 곳(DecideNextMode의 도착 판정 등)에서 쓴다.
    shared_ptr<RoadNode> GetParkTargetNode() const { return m_parkSpot ? m_parkSpot : m_pendingParkNode; }

    // 모드 FSM: 매 프레임 한 번 돌며, 모드가 바뀔 때만 OnModeExit/OnModeEnter를 호출한다.
    enum class DriveMode
    {
        Stop,
        Park,  // Reeds-Shepp 기반 입차/출차. m_parkSpot이 있을 때만 DecideNextMode가 진입시킴.
        Drive,
        Avoid // Hybrid A* 기반 장애물 회피 우회. TryAvoidObstacle이 막혔다고 판단하고 차가 완전히
              // 멈추면(m_avoidPending) DecideNextMode가 진입시킴.
    };
    const char *Car::DriveModeToString(DriveMode mode) const
    {
        switch (mode)
        {
        case DriveMode::Stop:
            return "Stop";
        case DriveMode::Park:
            return "Park";
        case DriveMode::Drive:
            return "Drive";
        case DriveMode::Avoid:
            return "Avoid";
        }
        return "?";
    }
    void UpdateMode();
    DriveMode DecideNextMode() const;
    void OnModeEnter(DriveMode mode);
    void OnModeExit(DriveMode mode);
    // Park 모드의 RS 입/출차 경로를 계획하고 VehicleController에 실행시킨다. 차가 완전히 멈춘
    // 뒤(m_parkPlanPending 해소 시점)의 위치/방향을 시작점으로 써야 하므로 UpdatePark에서만 호출된다.
    void BeginParkPlan();
    // Avoid 모드의 Hybrid A* 우회 경로를 계획하고 VehicleController에 실행시킨다. 차가 완전히
    // 멈춘 뒤(m_avoidPending 해소 시점)의 위치/방향을 시작점으로 쓴다. OnModeEnter(Avoid)에서만 호출.
    void BeginAvoidPlan();
    // 충돌판정용 차량 형상(휠베이스/최대조향각/피벗-바디중심 오프셋/반길이/반폭). HybridAStar와
    // TryAvoidObstacle 양쪽에서 같은 값을 쓰려고 뽑아둔 헬퍼.
    HybridAStar::VehicleShape BuildVehicleShape() const;

    void UpdateFindPath();
    // m_currentLane이 이미 정해진 상태에서 m_destLane까지 경로를 찾아 m_path/m_currentSpline을
    // 채운다. 도달 불가능하면 destLane/currentLane을 비운다. UpdateFindPath와 출차 완료
    // (UpdatePark) 양쪽에서 공유.
    void EnterCurrentLane();

    // 현재 위치에서 주어진 레인 위로 합류하는 연결 스플라인을 만들어 m_currentSpline에 세팅한다.
    void MergeOntoLane(const shared_ptr<Lane> &lane, const Vec3 &position);

    void UpdateStop();
    void UpdatePark();
    void UpdateAvoid();
    void MoveSpeedProfile();
    void CalculateSpeedProfile();
    void UpdateDrive();
    // 경로상 다음 레인으로 넘어갈지 판단/처리. false면 경로가 끝난 것이므로 이번 프레임은 조향/가속 제어를 건너뛴다.
    bool CheckPath();
    // 코리도어(스플라인 lookahead)가 장애물에 막혔는지 매 프레임 감시. 막혔으면 감속/정지시키고
    // (반환값 true, VehicleController::Tick은 이번 프레임 건너뜀), 완전히 멈추면 m_avoidPending을
    // 켜서 다음 프레임 DecideNextMode가 Avoid 모드(Hybrid A* 우회)로 전환하게 한다.
    bool TryAvoidObstacle();

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
    Vec3 m_halfExtents = Vec3::sZero(); // 충돌판정용 차체 반크기(x=반폭, z=반길이). CarSpec::halfExtents.
    float m_maxSteerAngle = ToRadians(45.0f);        // 최대 조향각 (45도)
    static constexpr float CURVE_SPEED_COEFF = 0.8f; // 최대 코너링 속도 = CURVE_SPEED_COEFF * sqrt(R)
    static constexpr float STEER_RAMP_RATE = 0.4f;

    // 컴포넌트 및 AI 상태 (Components & Systems)
    RoadDataManager *m_RoadDataManager = nullptr;
    bool m_isFocused = false; // 포커스 여부 (입력 처리용)
    DriveMode m_mode = DriveMode::Stop;
    VehicleController m_vehicleController; // DriveMode가 세운 계획(세그먼트)을 실제로 실행
    shared_ptr<Lane> m_destLane;
    shared_ptr<Lane> m_currentLane;
    shared_ptr<RoadNode> m_parkSpot;        // 예약된 목표 주차칸(있는 동안은 "이 자리에 주차 중/주차 예정")
    shared_ptr<RoadNode> m_pendingParkNode; // 예약 전, 도착하면 그때 주차칸을 예약할 목표 Park 노드
    bool m_isExitingPark = false;           // 이번 Park 계획이 출차(주차칸->레인)인지, 입차(레인->주차칸)인지
    // Park 모드에 막 진입해서 정지 대기 중인지. true인 동안은 RS 계획을 세우지 않고 감속만 하다가,
    // 완전히 멈추면(m_speed==0) 그 위치/방향을 시작점으로 BeginParkPlan을 호출한다.
    bool m_parkPlanPending = false;
    // TryAvoidObstacle이 코리도어가 막혔다고 보고 차를 완전히 세운 뒤 켜는 플래그. true면
    // DecideNextMode가 다음 프레임에 Avoid로 전환하고 OnModeEnter(Avoid)가 BeginAvoidPlan을 호출한다.
    bool m_avoidPending = false;
    // Hybrid A* 재계획을 프레임마다 재시도하지 않도록 하는 쿨다운(초). BeginAvoidPlan이 실패했을 때만 의미가 있다.
    float m_avoidReplanCooldown = 0.0f;
    vector<LaneStep> m_path;
    size_t m_pathIndex = 0;
    static constexpr float LOOK_PROFILE_TIME = 5.0f;
    static constexpr size_t SPEED_PROFILE_COUNT = 25;
    std::array<std::pair<Vec3, float>, SPEED_PROFILE_COUNT> m_speedProfile; // 0.2초단위로 5초까지
    size_t m_profileIndex = 0;
    Spline m_currentSpline;
    float m_currentTime = 0.0f;
    float m_lastProfileTime = 0.0f;

    // 차량 주행 상태 변수 (Vehicle States)
    float m_speed = 0.0f;
    float m_acceleration = 0.0f;
    float m_steerAngle = 0.0f;
    bool m_isReverse = false;
    float m_deltaTime = 0.0f;
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
    RenderObject m_parkPathRender;                     // Park 계획(RS 경로) 폴리라인
    RenderObject m_parkTargetMarker;                   // Park 목표 위치
    RenderObject m_parkTargetLine;                     // Park 목표 방향
    std::array<RenderObject, 4> m_speedProfileMarkers; // 1~4초 뒤 속도 프로파일 목표 위치 (노란 구)
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