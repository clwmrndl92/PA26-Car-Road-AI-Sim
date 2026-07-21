#pragma once

// IIDM(Improved Intelligent Driver Model) + CAH(Constant Acceleration Heuristic) 종방향
// 가속도 산출. Treiber/Kesting의 IIDM·CAH 블렌딩 공식을 그대로 옮김 (notes/help.txt 의사코드 기반).
namespace CarFollowing
{
    struct Params
    {
        float v0;       // Desired Speed: 전방이 비었을 때 도달하고자 하는 목표 속도 (m/s)
        float T;        // Desired Time Headway: 앞차와의 원하는 시간 간격 (s, 보통 1.0~1.8s)
        float s0;       // Standstill Distance: 정지 상태 시 확보할 물리적 최소 안전 정지 거리 (m)
        float a;        // Max Acceleration: 차량의 최대 가속 성능 (m/s^2)
        float b;        // Comfortable Deceleration: 운전자가 안락함을 느끼는 제동 감속도 (m/s^2)
        float delta = 4.0f;    // Acceleration Exponent: 속도 도달 시 가속도 감쇄 계수
        float coolness = 1.0f; // Coolness Factor (c): 끼어들기 시 CAH 감속을 얼마나 부드럽게 넘길지 (0~1)
    };

    // egoSpeed/egoAccel: 내 차량의 현재 속도/가속도. leaderSpeed/leaderAccel: 앞 차량의 현재 속도/가속도.
    // gap: 앞 차량과의 범퍼 대 범퍼 실제 차간 거리 (s, m). 겹침/역전 시(gap<=0.001)엔 즉시 비상 제동을 반환.
    float CalculateAcceleration(float egoSpeed, float egoAccel, float leaderSpeed, float leaderAccel,
                                float gap, const Params &p);
}
