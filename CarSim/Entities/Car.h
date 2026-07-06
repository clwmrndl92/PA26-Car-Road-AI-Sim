#pragma once
#include "Core/GameObject.h"
#include "CarSpec.h"
#include <deque>
#include <string>
#include "AI/BehaviourTree.h"

class Car : public GameObject
{
public:
    // 생명주기 및 초기화 (Lifecycle & Initialization)
    void Init(const CarSpec &spec, JPH::Vec3 position = JPH::Vec3::sZero());

    // 부모 클래스 오버라이드 함수 (Overrides)
    void Update(float dt) override;
    void Draw(ID3D11DeviceContext *context, IEffect &effect) override;

    Vec3 GetPosition() const override;
    void SetPosition(Vec3 position) override;
    void SetRotation(const DirectX::XMFLOAT4 &rotation) override;

    // Getter / Setter (Accessors)
    void SetFocused(bool focused) { m_isFocused = focused; }

    // 조작 및 제어 인터페이스 (Control Interface)
    void Accelerate(int direction); // 1: 가속, -1: 제동, 0: 관성
    void Steer(int direction);      // -1: 좌, 1: 우, 0: 정렬
    void ChangeGear();              // 속도가 낮을 때 전진/후진 기어 토글

private:
    // 내부 물리 및 제어 로직 (Internal Physics & Control)
    void UpdateCar();
    void UpdateWithControl();
    void ApplyMotion();
    float GetSignedSpeed() const { return m_speed * (m_isReverse ? -1.0f : 1.0f); }
    JPH::Vec3 ComputeDesiredVelocity() const;

    // 디버그 및 트레일(자국) 렌더링 (Debug & Rendering Helpers)
    void UpdateDebugWindow();
    void UpdateTrail();
    void RebuildTrailRender(RenderObject &render, const std::deque<DirectX::XMFLOAT3> &trail,
                            const std::string &name, const DirectX::XMFLOAT4 &color);

    // 인공지능 / 행동 트리 (AI / Behavior Tree)
    std::unique_ptr<BTNode> BuildBehaviourTree();

    std::unique_ptr<BTNode> StopNode();
    std::unique_ptr<BTNode> ChangeLineNode();
    std::unique_ptr<BTNode> DriveNode();

private: // 멤버 변수 구역
    // 설정 및 스펙 상수/변수 (Constants & Specifications)
    const float m_maxSpeed = 55.56f;            // 200 km/h
    const float m_maxAcceleration = 2.78f;      // 0-100 km/h in 10s
    const float m_maxBrakeDeceleration = 9.26f; // 100-0 km/h in 3s
    float m_wheelbase = 0.0f;                   // 축거 (Init에서 설정)
    float m_mass = 1.0f;                        // 질량 (Init에서 설정)
    float m_maxSteerAngle = 0.785f;             // 최대 조향각 (45도)

    // 컴포넌트 및 AI 상태 (Components & Systems)
    BehaviourTree m_BehaviourTree;
    bool m_isFocused = false; // 포커스 여부 (입력 처리용)

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
};