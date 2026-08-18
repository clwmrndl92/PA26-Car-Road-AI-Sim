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

// 최소 곡률 QP로 미리 푼 레이싱 라인. 도로 문맥이 바뀔 때만 다시 만든다.
struct RaceLine
{
    std::vector<Vec3> points; // 월드 좌표
    std::vector<float> arcLength;
    std::vector<float> curvature;
    float drawFromS = 0.0f; // 디버그로 그릴 구간(문맥 도로 제외)
    float drawToS = 0.0f;
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
    float Stanley(Vec3 pathPoint, Vec3 pathDirection);

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

    void UpdateHorn(float dt);
    bool IsHornSituation() const;
    bool KnowsRedSignalAhead() const;

#pragma endregion

#pragma region BehaviorPlan
    void UpdateDrivePlan();
    float RoadTargetSpeed(const shared_ptr<Road> &road) const; // 도로 제한속도(최고속 캡)
    void ComputeDrivableRange(const RoadRef &road, float &outMin, float &outMax) const;
#pragma endregion

#pragma region Avoid
    void UpdateSensors(); // 장애물 목록 수집
    // 부채꼴 슬롯 interest/danger로 매프레임 조향각 생성
    float ComputeContextSteer(const Vec3 &pursuitTarget, float pursuitSteer) const;
    Vec3 GetBodyCenter() const;                             // OBB 판정 기준점
    VehicleCollision::Obstacle MakeVehicleObstacle() const; // 이 차의 차체 OBB를 장애물 하나로
    void RebuildCtxSteerRender();
#pragma endregion

private:
    // Track-focused GT racing specification.
    const float m_maxSpeed = 320.0f / 3.6f;          // 최고속도 320 km/h
    float m_maxAccel = (100.0f / 3.6f) / 3.0f;       // 0-100 km/h 약 3.0초
    const float m_maxBrake = (100.0f / 3.6f) / 1.8f; // 100-0 km/h 약 1.8초 (약 1.57 g)
    float m_speedGain = 3.5f;                        // 목표속도 변화에 빠르게 풀스로틀/브레이크
    float m_jerkUp = 30.0f;                          // 가속 저크 상한 (m/s^3)
    float m_jerkDown = 60.0f;                        // 제동 저크 상한 (m/s^3)
    CarPersonality m_personality;

    float m_wheelbase = 0.0f;
    float m_mass = 1.0f;
    Vec3 m_halfExtents = Vec3::sZero();       // x=반폭 z=반길이
    float m_maxSteerAngle = ToRadians(35.0f); // 저속 최대 조향각
    float m_stanleyGain = 2.5f;               // 고속에서도 레이싱 라인을 붙잡는 횡오차 게인
    float m_stanleySoft = 2.0f;               // 저속 조향 진동 억제

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
    std::vector<Car *> m_nearbyCars;                  // 재수집 비용 회피 캐시
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
    RenderObject m_racingLineRender; // 레이싱 라인(디버그)
    RaceLine m_raceLine;
    std::vector<const Road *> m_racingLineRoads; // 재생성 판단용
    ApexStrategy m_apexStrategy = ApexStrategy::Late;
    ApexStrategy m_racingLineStrategy = ApexStrategy::Late; // 라인이 만들어진 전략
};

static float CalcMaxSteerAngle(float speed, float wheelbase)
{
    constexpr float MAX_STEER_ANGLE = ToRadians(35.0f);
    constexpr float MAX_LATERAL_ACCEL = 15.0f; // 약 1.5 g 타이어 그립 한계
    constexpr float MIN_WHEELBASE = 0.1f;

    wheelbase = std::max(wheelbase, MIN_WHEELBASE);
    float transitionSpeed = std::sqrt(MAX_LATERAL_ACCEL * wheelbase / std::tan(MAX_STEER_ANGLE));
    if (speed <= transitionSpeed)
        return MAX_STEER_ANGLE;

    // bicycle model: lateral accel = v^2 * tan(steer) / wheelbase
    return std::atan(MAX_LATERAL_ACCEL * wheelbase / (speed * speed));
}
