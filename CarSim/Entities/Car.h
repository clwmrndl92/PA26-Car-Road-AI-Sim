#pragma once
#include "Core/GameObject.h"
#include "CarSpec.h"
#include <cmath>
#include <deque>
#include <string>
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
    void SetDestination(std::shared_ptr<RoadNode> roadNode) { m_destNode = roadNode; }

    // 조작 및 제어 인터페이스 (Control Interface)
    void Accelerate(float desiredVelocity);
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

    // 인공지능 / 행동 트리 (AI / Behavior Tree)
    std::unique_ptr<BTNode> BuildBehaviourTree();

    std::unique_ptr<BTNode> FindPathNode();

    std::unique_ptr<BTNode> StopNode();
    std::unique_ptr<BTNode> ChangeLineNode();
    std::unique_ptr<BTNode> ProgressPath();
    std::unique_ptr<BTNode> DriveNode();

private: // 멤버 변수 구역
    // 설정 및 스펙 상수/변수 (Constants & Specifications)
    const float m_maxSpeed = 200.0f / 3.6f;                      // 200 km/h
    const float m_maxAcceleration = (100.0f / 3.6f) / 10.0f;     // 0-100 km/h in 10s
    const float m_maxBrakeDeceleration = (100.0f / 3.6f) / 3.0f; // 100-0 km/h in 3s
    static constexpr float ACCEL_RAMP_RATE = 11.1f;              // reaches m_maxAcceleration in ~0.25s
    static constexpr float BRAKE_RAMP_RATE = 55.6f;              // reaches m_maxBrakeDeceleration in ~0.17s
    float m_wheelbase = 0.0f;                                    // 축거 (Init에서 설정)
    float m_mass = 1.0f;                                         // 질량 (Init에서 설정)
    float m_maxSteerAngle = ToRadians(45.0f);                    // 최대 조향각 (45도)

    // 컴포넌트 및 AI 상태 (Components & Systems)
    const RoadDataManager *m_RoadDataManager = nullptr;
    BehaviourTree m_BehaviourTree;
    bool m_isFocused = false; // 포커스 여부 (입력 처리용)
    shared_ptr<RoadNode> m_destNode;
    shared_ptr<RoadNode> m_currentNode;
    float m_startDistToNode = 0.0f;
    vector<shared_ptr<RoadNode>> m_path;
    size_t m_pathIndex = 0;

    // 차량 주행 상태 변수 (Vehicle States)
    float m_speed = 0.0f;
    float m_acceleration = 0.0f;
    float m_steerAngle = 0.0f;
    bool m_isReverse = false;
    float m_deltaTime = 0.0f;

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