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
#include "Nav/VehicleCollision.h"

class SimulationState;
class Spline;
class VehicleSegment;

class Car : public GameObject
{
public:
    void Init(const CarSpec &spec, const CarPersonality &personality, SimulationState *simState, JPH::Vec3 position = JPH::Vec3::sZero(),
              float aiStartDelay = 0.0f);

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
    bool CanEnterFromCurrentBand(const RoadRef &next) const;
    bool RepathFromCurrentBand();

    void UpdateHorn(float dt);
    bool IsHornSituation() const;
    bool KnowsRedSignalAhead() const;
    bool IsEmergencyRayBlocked() const;

#pragma endregion

#pragma region BehaviorPlan
    void UpdateDrivePlan();
    float RoadTargetSpeed(const shared_ptr<Road> &road) const; // 도로 제한속도(최고속 캡)
    void ComputeDrivableRange(const RoadRef &road, float &outMin, float &outMax) const;
#pragma endregion

#pragma region Avoid
    void UpdateSensors();                                             // 장애물 목록 수집
    float ComputeReverseEscapeSteer(const Vec3 &blockerCenter) const; // 후진중 코를 돌릴 조향
    void TryStartStaticReverseEscape();                               // 정적장애물에 막히면 후진
    // 부채꼴 슬롯 interest/danger로 매프레임 조향각 생성
    float ComputeContextSteer(const Vec3 &pursuitTarget, float pursuitSteer) const;
    Vec3 GetBodyCenter() const;                             // OBB 판정 기준점
    VehicleCollision::Obstacle MakeVehicleObstacle() const; // 이 차의 차체 OBB를 장애물 하나로
    void RebuildEmergRayRender();
    void RebuildCtxSteerRender();
#pragma endregion

private:
    const float m_maxSpeed = 200.0f / 3.6f;          // 200 km/h
    float m_maxAccel = (100.0f / 3.6f) / 14.0f;      // personality.maxAccel로 갱신됨
    const float m_maxBrake = (100.0f / 3.6f) / 3.0f; // 100-0 km/h in 3s
    float m_speedGain = 2.0f;                        // 속도오차 -> 목표가속 비례게인
    float m_jerkUp = 4.0f;                           // 가속 저크 상한
    float m_jerkDown = 15.0f;                        // 제동 저크 상한
    CarPersonality m_personality;

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
    static constexpr float SAFE_GAP = 2.0f;               // 긴급제동 레이 여유거리

    float m_lastBehaviorPlanTime = -1000.0f; // 첫 판단 즉시 실행되게
    float m_planAccelDebug = 0.0f;           // 매프레임 가속도
    std::vector<VehicleCollision::Obstacle> m_obstacles;
    std::vector<Car *> m_nearbyCars; // 재수집 비용 회피 캐시
    mutable std::vector<Vec3> m_emergRayDebugLines;
    mutable bool m_emergRayDebugBlocked = false;
    mutable float m_ctxSteerDebugRad = 0.0f;          // 반응형 조향이 고른 슬롯각
    mutable float m_ctxSteerDebugDanger = 0.0f;       // 그 슬롯의 danger
    mutable std::vector<Vec3> m_ctxSteerOpenLines;    // 안전 슬롯 레이(초록)
    mutable std::vector<Vec3> m_ctxSteerBlockedLines; // 위험 슬롯 레이(빨강)

    // 정적장애물 막혀서 후진탈출
    static constexpr float REVERSE_ESCAPE_TIME = 5.0f;     // 후진 상한(s), 안전장치
    static constexpr float REVERSE_ESCAPE_TURN_COS = 0.7f; // 코 60도 돌면 탈출
    static constexpr float REVERSE_ESCAPE_SPEED = 2.5f;    // 후진 속도(m/s)
    static constexpr float REVERSE_ESCAPE_STEER = ToRadians(30.0f);
    static constexpr float REVERSE_STEER_RAMP = 2.0f;
    static constexpr float HEADON_FRONT_COS = 0.5f; // 앞쪽 60도 안에서 부딪혔나
    // 정적장애물은 접촉이벤트가 없다(Static 바디). 긴급제동이 닿기 전에 세우므로 "막혀 선 시간"으로 판정
    static constexpr float STATIC_BLOCK_TRIGGER = 0.3f; // 이 시간 막혀 있으면 후진(s)
    static constexpr float STATIC_BLOCK_RANGE = 3.0f;   // 앞범퍼 기준 이 안이면 막힘
    static constexpr float STATIC_BLOCK_SPEED = 0.5f;   // 사실상 정지(m/s)

    // 물리/판단 틱 다름
    float m_reverseEscapeTimer = 0.0f;            // >0이면 후진탈출중
    float m_reverseEscapeSteer = 0.0f;            // 후진하며 코를 돌릴 방향
    Vec3 m_reverseEscapeStartFwd = Vec3::sZero(); // 회전량 측정 기준

    float m_staticBlockTimer = 0.0f;           // 정적장애물 막힘 시간
    Vec3 m_staticBlockLastPos = Vec3::sZero(); // 실제 이동량 측정 기준

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
    RenderObject m_emergRayRender;        // 긴급제동 레이(디버그)
    RenderObject m_ctxSteerOpenRender;    // 반응형 조향 안전 레이(디버그)
    RenderObject m_ctxSteerBlockedRender; // 반응형 조향 위험 레이(디버그)
};

static float CalcMaxSteerAngle(float speed)
{
    constexpr float LOW_SPEED_CUTOFF = 18.26f / 3.6f;
    constexpr float MAX_STEER_ANGLE = ToRadians(45.0f);
    return (speed <= LOW_SPEED_CUTOFF) ? MAX_STEER_ANGLE : 20.2f / (speed * speed);
}
