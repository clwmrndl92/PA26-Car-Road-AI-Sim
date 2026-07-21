#include "TestHarness.h"
#include "Nav/Mobil.h"
#include <cmath>

namespace
{
    CarFollowing::Params DefaultCfParams()
    {
        CarFollowing::Params p;
        p.v0 = 30.0f;
        p.T = 1.5f;
        p.s0 = 2.0f;
        p.a = 1.5f;
        p.b = 2.0f;
        p.delta = 4.0f;
        p.coolness = 1.0f;
        return p;
    }

    Mobil::Params DefaultMobilParams()
    {
        Mobil::Params p;
        p.b_safe = 3.0f;
        p.politeness = 0.5f;
        p.a_thr = 0.2f;
        return p;
    }
}

TEST_CASE(Mobil_GetGap_NullLeader_ReturnsLargeValue)
{
    Mobil::VehicleState follower;
    follower.position = 0.0f;
    CHECK(Mobil::GetGap(follower, nullptr) == 99999.0f);
}

TEST_CASE(Mobil_GetGap_ComputesBumperToBumperDistance)
{
    Mobil::VehicleState follower;
    follower.position = 0.0f;
    Mobil::VehicleState leader;
    leader.position = 20.0f;
    leader.length = 4.5f;
    CHECK(std::abs(Mobil::GetGap(follower, &leader) - 15.5f) < 0.001f);
}

// 현재 차선은 느린 앞차에 막혀 크게 감속 중이고, 옆 차선은 완전히 뚫려있음(neighbor 없음)
// -> 이득이 매우 커서 문턱값을 가볍게 넘어야 함.
TEST_CASE(Mobil_ClearBenefitNoNeighbors_ApprovesChange)
{
    CarFollowing::Params cf = DefaultCfParams();
    Mobil::Params p = DefaultMobilParams();

    Mobil::VehicleState ego;
    ego.speed = 20.0f;
    ego.position = 50.0f;
    ego.length = 4.5f;

    Mobil::VehicleState egoLeader; // 현재 차선: 느리고 가까운 앞차
    egoLeader.speed = 5.0f;
    egoLeader.position = 60.5f; // gap = 60.5 - 50 - 4.5 = 6.0f
    egoLeader.length = 4.5f;

    Mobil::VehicleState newLeader; // 목표 차선: 사실상 뚫려있음
    newLeader.speed = 30.0f;
    newLeader.position = 100000.0f;
    newLeader.length = 4.5f;

    bool approved = Mobil::EvaluateLaneChange(ego, nullptr, egoLeader, newLeader, nullptr, p, cf);
    CHECK(approved);
}

// 두 차선의 앞 상황이 완전히 동일하면(같은 리더 상태) 얻는 이득이 정확히 0 -> 문턱값을 못 넘어 거부.
TEST_CASE(Mobil_IdenticalLanes_RejectsChange)
{
    CarFollowing::Params cf = DefaultCfParams();
    Mobil::Params p = DefaultMobilParams();

    Mobil::VehicleState ego;
    ego.speed = 15.0f;
    ego.position = 50.0f;
    ego.length = 4.5f;

    Mobil::VehicleState leader; // egoLeader와 newLeader에 동일한 값을 사용
    leader.speed = 15.0f;
    leader.position = 90.0f;
    leader.length = 4.5f;

    bool approved = Mobil::EvaluateLaneChange(ego, nullptr, leader, leader, nullptr, p, cf);
    CHECK(!approved);
}

// 목표 차선 뒤차 바로 코앞으로 끼어드는 상황: 이득과 무관하게 안전 기준(b_safe) 위반으로 거부돼야 함.
TEST_CASE(Mobil_UnsafeForNewFollower_RejectsChangeRegardlessOfGain)
{
    CarFollowing::Params cf = DefaultCfParams();
    Mobil::Params p = DefaultMobilParams();

    Mobil::VehicleState ego;
    ego.speed = 20.0f;
    ego.position = 100.0f;
    ego.length = 4.5f;

    Mobil::VehicleState egoLeader; // 현재 차선: 느리고 가까운 앞차 (myGain을 크게 키워 검증을 확실히 함)
    egoLeader.speed = 5.0f;
    egoLeader.position = 106.0f;
    egoLeader.length = 4.5f;

    Mobil::VehicleState newLeader; // 목표 차선: 뚫려있음
    newLeader.speed = 30.0f;
    newLeader.position = 100000.0f;
    newLeader.length = 4.5f;

    Mobil::VehicleState newFollower; // ego 바로 뒤, 훨씬 빠른 속도로 접근 중 -> 급브레이크 강요
    newFollower.speed = 30.0f;
    newFollower.position = 95.0f; // gap = 100 - 95 - 4.5 = 0.5f
    newFollower.length = 4.5f;

    bool approved = Mobil::EvaluateLaneChange(ego, nullptr, egoLeader, newLeader, &newFollower, p, cf);
    CHECK(!approved);
}
