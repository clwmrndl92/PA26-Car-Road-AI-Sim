#include "Mobil.h"

namespace Mobil
{
    float GetGap(const VehicleState &follower, const VehicleState *leader)
    {
        if (leader == nullptr)
        {
            return 99999.0f; // 앞차가 없으면 무한에 가까운 충분한 거리
        }
        return leader->position - follower.position - leader->length;
    }

    namespace
    {
        float Accel(const VehicleState &ego, const VehicleState &leader, const CarFollowing::Params &cfParams)
        {
            return CarFollowing::CalculateAcceleration(ego.speed, ego.accel, leader.speed, leader.accel,
                                                        GetGap(ego, &leader), cfParams);
        }
    }

    bool IsSafeLaneChange(const VehicleState &ego, const VehicleState *newFollower, const Params &p,
                          const CarFollowing::Params &cfParams)
    {
        if (newFollower == nullptr)
            return true;
        float tilde_a_n = Accel(*newFollower, ego, cfParams); // 내가 끼어든 후 새 후방차의 예상 가속도
        return tilde_a_n >= -p.b_safe;
    }

    bool EvaluateLaneChange(const VehicleState &ego, const VehicleState *oldFollower,
                            const VehicleState &egoLeader, const VehicleState &newLeader,
                            const VehicleState *newFollower, const Params &p,
                            const CarFollowing::Params &cfParams)
    {
        // =============================================================
        // STEP 1: 현재 차로에서의 가속도 (Before Lane Change)
        // =============================================================

        float a_c = Accel(ego, egoLeader, cfParams); // 나의 현재 가속도

        float a_o = 0.0f; // 기존 후방차의 현재 가속도 - 내 뒤에 있으므로 리더는 ego
        if (oldFollower != nullptr)
        {
            a_o = Accel(*oldFollower, ego, cfParams);
        }

        float a_n = 0.0f; // 새로운 후방차의 현재 가속도 - 내가 끼어들기 전이므로 리더는 원래의 newLeader
        if (newFollower != nullptr)
        {
            a_n = Accel(*newFollower, newLeader, cfParams);
        }

        // =============================================================
        // STEP 2: 목표 차로로 이동 시 예상 가속도 (After Lane Change)
        // =============================================================

        float tilde_a_c = Accel(ego, newLeader, cfParams); // 나의 예상 가속도 - 목표 차선 앞차를 추종

        float tilde_a_o = a_o; // 뒤차가 없으면 이득 변동 없음
        if (oldFollower != nullptr)
        {
            // 기존 후방차의 예상 가속도 - 내가 사라졌으므로 새로운 리더는 나의 옛 앞차(egoLeader)
            tilde_a_o = Accel(*oldFollower, egoLeader, cfParams);
        }

        float tilde_a_n = 0.0f;
        if (newFollower != nullptr)
        {
            // 새로운 후방차의 예상 가속도 - 내가 앞에 끼어들었으므로 리더는 ego
            tilde_a_n = Accel(*newFollower, ego, cfParams);
        }

        // =============================================================
        // STEP 3: 판정 기준 검사 (Safety & Incentive Check)
        // =============================================================

        // [검사 1] 안전 기준: 내가 끼어들었을 때 뒤차가 밟는 브레이크가 안전 한계보다 가혹하면 절대 불가
        if (newFollower != nullptr && tilde_a_n < -p.b_safe)
        {
            return false;
        }

        // [검사 2] 유인 기준
        float myGain = tilde_a_c - a_c;

        float neighborGain = 0.0f;
        if (newFollower != nullptr)
        {
            neighborGain += (tilde_a_n - a_n); // 목표 차선 뒤차의 감속 손실 (일반적으로 음수)
        }
        if (oldFollower != nullptr)
        {
            neighborGain += (tilde_a_o - a_o); // 기존 차선 뒤차의 가속 이득 (일반적으로 양수)
        }

        return myGain + p.politeness * neighborGain > p.a_thr;
    }
}
