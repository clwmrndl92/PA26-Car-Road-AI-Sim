#pragma once
#include "Core/GameObject.h"
#include "CarSpec.h"
#include <cmath>
#include <deque>
#include <string>
#include <array>
#include "AI/BehaviourTree.h"
#include <Nav/RoadDataManager.h>

class Car : public GameObject
{
public:
    // 생명주기 및 초기화 (Lifecycle & Initialization)
    void Init(const CarSpec &spec, const RoadDataManager *roadDataManager, JPH::Vec3 position = JPH::Vec3::sZero());

    // 부모 클래스 오버라이드 함수 (Overrides)
    void Update(float dt) override;
    void Draw(ID3D11DeviceContext *context, IEffect &effect) override;

    Vec3 GetPosition() const override;
    Vec3 GetForwardAxis() const;
    void SetPosition(Vec3 position) override;
    void SetRotation(const DirectX::XMFLOAT4 &rotation) override;

    // Getter / Setter (Accessors)
    void SetFocused(bool focused) { m_isFocused = focused; }
    void SetDestination(std::shared_ptr<Lane> lane) { m_destLane = lane; }

        // 조작 및 제어 인터페이스 (Control Interface)
    void Accelerate(float desiredVelocity);
    void EmergBrake();
    void Steer(float desiredRadian);
    void ChangeGear(); // 속도가 낮을 때 전진/후진 기어 토글

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
    void UpdateTrail();
    void RebuildTrailRender(RenderObject &render, const std::deque<DirectX::XMFLOAT3> &trail,
                            const std::string &name, const DirectX::XMFLOAT4 &color);
    void RebuildSplineRender();

    // 인공지능 / 행동 트리 (AI / Behavior Tree)
    std::unique_ptr<BTNode> BuildBehaviourTree();

    bool IsOffCourse();

    std::unique_ptr<BTNode> FindPathNode();

    // 현재 위치에서 주어진 레인 위로 합류하는 연결 스플라인을 만들어 m_currentSpline에 세팅한다.
    void MergeOntoLane(const shared_ptr<Lane> &lane, const Vec3 &position);

    std::unique_ptr<BTNode> StopNode();
    void MoveSpeedProfile();
    void CalculateSpeedProfile();
    std::unique_ptr<BTNode> DriveNode();

private: // 멤버 변수 구역
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
    float m_maxSteerAngle = ToRadians(45.0f);        // 최대 조향각 (45도)
    static constexpr float CURVE_SPEED_COEFF = 0.8f; // 최대 코너링 속도 = CURVE_SPEED_COEFF * sqrt(R)

    // 컴포넌트 및 AI 상태 (Components & Systems)
    const RoadDataManager *m_RoadDataManager = nullptr;
    bool m_isFocused = false; // 포커스 여부 (입력 처리용)
    BehaviourTree m_BehaviourTree;
    shared_ptr<Lane> m_destLane;
    shared_ptr<Lane> m_currentLane;
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