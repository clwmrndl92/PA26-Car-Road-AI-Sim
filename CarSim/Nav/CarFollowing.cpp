#include "CarFollowing.h"
#include <algorithm>
#include <cmath>

namespace CarFollowing
{
    float CalculateAcceleration(float egoSpeed, float egoAccel, float leaderSpeed, float leaderAccel,
                                float gap, const Params &p)
    {
        // [예외 처리 1] 차간 거리가 극도로 좁거나 충돌(역전) 시: 강제 비상 제동으로 겹침을 즉각 해소.
        if (gap <= 0.001f)
        {
            return -p.b * 4.0f;
        }

        // =============================================================
        // PHASE 1: IIDM (Improved Intelligent Driver Model)
        // =============================================================

        float delta_v = egoSpeed - leaderSpeed;

        // 원하는 동적 안전거리 s*(v, delta_v) = s0 + max(0, v*T + (v * delta_v) / (2 * sqrt(a * b)))
        float sqrt_ab = 2.0f * std::sqrt(std::max(0.0001f, p.a * p.b));
        float dynamic_gap_term = (egoSpeed * delta_v) / sqrt_ab;
        float s_star = p.s0 + std::max(0.0f, (egoSpeed * p.T) + dynamic_gap_term);

        // 스케일 인덱스 z = s* / s
        float z = s_star / gap;

        // 자유 주행 가속도 a_free = a * (1 - (v / v0)^delta)
        float speed_ratio = std::min(1.0f, egoSpeed / std::max(0.1f, p.v0));
        float a_free = p.a * (1.0f - std::pow(speed_ratio, p.delta));

        float a_iidm = 0.0f;
        if (z >= 1.0f)
        {
            // 접근/정체 영역: IDM 제동 공식
            a_iidm = p.a * (1.0f - (z * z));
        }
        else
        {
            // 자유 흐름 영역
            // [예외 처리 2] 내 속도가 목표 속도 v0에 정확히 일치해 a_free가 0에 수렴할 때 NaN 방지
            if (std::abs(a_free) < 0.001f)
            {
                a_iidm = 0.0f;
            }
            else
            {
                // 지수부: 2a / |a_free| (연속성 보정 수식)
                float exponent = (2.0f * p.a) / std::abs(a_free);
                exponent = std::min(100.0f, exponent); // 수치적 오버플로우 방지용 하드 상한선

                a_iidm = a_free * (1.0f - std::pow(z, exponent));
            }
        }

        // =============================================================
        // PHASE 2: CAH (Constant Acceleration Heuristic)
        // =============================================================

        // 앞차의 가속도를 유효 범위(내 가속도 한계)로 제한
        float effective_leader_accel = std::min(leaderAccel, p.a);

        // Theta(delta_v): 내가 앞차보다 빠를 때만 작동하는 헤비사이드 계단 함수
        float theta = (delta_v >= 0.0f) ? 1.0f : 0.0f;

        // 조건: v_l * (v - v_l) <= -2 * s * a_leader_effective
        float condition_val = leaderSpeed * delta_v;
        float threshold_val = -2.0f * gap * effective_leader_accel;

        float a_cah = 0.0f;
        if (condition_val <= threshold_val)
        {
            // 앞차와의 관계가 극단적으로 좁혀져 충돌 위험이 심각한 경우
            float numerator = egoSpeed * egoSpeed * effective_leader_accel;
            float denominator = (leaderSpeed * leaderSpeed) - (2.0f * gap * effective_leader_accel);

            // [예외 처리 3] 분모가 0이 되는 특이점 제어
            if (std::abs(denominator) < 0.001f)
            {
                a_cah = effective_leader_accel;
            }
            else
            {
                a_cah = numerator / denominator;
            }
        }
        else
        {
            // 일반/완만한 추종 상황
            float braking_risk_term = (delta_v * delta_v * theta) / (2.0f * gap);
            a_cah = effective_leader_accel - braking_risk_term;
        }

        // =============================================================
        // PHASE 3: Blending (두 모델 가속도 혼합)
        // =============================================================
        if (a_iidm >= a_cah)
        {
            // IIDM 값이 CAH보다 보수적(안전)이거나 크면 무조건 IIDM 채택
            return a_iidm;
        }

        // CAH 감속 제한선이 작동 중일 때: tanh로 두 값을 유연하게 혼합해 끼어들기 급제동을 완화
        float tanh_input = (a_iidm - a_cah) / p.b;
        float blended_cah = a_cah + p.b * std::tanh(tanh_input);

        return (1.0f - p.coolness) * a_iidm + p.coolness * blended_cah;
    }
}
