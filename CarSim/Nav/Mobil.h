#pragma once
#include "CarFollowing.h"

// MOBIL(Minimizing Overall Braking Induced by Lane changes) 차로 변경 판정.
// Kesting/Treiber의 안전 기준 + 유인 기준 공식을 그대로 옮김 (notes/help.txt 의사코드 기반).
// 실제 게임의 Car/Lane과는 독립적인 순수 함수로 둬서 테스트하기 쉽게 한다 -- 호출부(Car.cpp)가
// Car 상태를 VehicleState로 뽑아 넘긴다.
namespace Mobil
{
    struct Params
    {
        float b_safe;     // Safety Deceleration: 후방 차량에 강제할 수 있는 최대 안전 감속도 (보통 3.0 m/s^2)
        float politeness; // Politeness Factor (p): 이타성 계수 (0.0: 완전 이기주의 ~ 0.5: 현실적 양보)
        float a_thr;      // Threshold: 사소한 이득으로 인한 칼치기 방지용 최소 진입 장벽 (보통 0.1~0.2 m/s^2)
    };

    // 차선을 따라가는 1차원(Frenet s-좌표) 위치 기준 차량 상태. position은 레인 시작점부터의 누적 거리.
    struct VehicleState
    {
        float speed = 0.0f;
        float accel = 0.0f;
        float position = 0.0f;
        float length = 0.0f;
    };

    // follower와 leader 사이의 범퍼 대 범퍼 간격. leader가 없으면(진로가 뚫려있으면) 충분히 큰 값을 반환.
    float GetGap(const VehicleState &follower, const VehicleState *leader);

    // 유인 기준 없이 안전 기준만 판정 -- 라우팅상 강제 차선변경처럼 이득과 무관하게 반드시 옮겨야 하는 경우용.
    bool IsSafeLaneChange(const VehicleState &ego, const VehicleState *newFollower, const Params &p,
                         const CarFollowing::Params &cfParams);

    // ego가 옆 차선으로 변경해도 되는지 판정한다.
    // egoLeader/newLeader는 항상 존재해야 한다 (실제 앞차가 없으면 호출부가 아주 먼 가상 리더를 넘긴다).
    // oldFollower/newFollower는 없으면 nullptr로 넘긴다 (뒤에 차가 없는 경우).
    bool EvaluateLaneChange(const VehicleState &ego, const VehicleState *oldFollower,
                            const VehicleState &egoLeader, const VehicleState &newLeader,
                            const VehicleState *newFollower, const Params &p,
                            const CarFollowing::Params &cfParams);
}
