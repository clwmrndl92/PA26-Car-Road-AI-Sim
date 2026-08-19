#include "SimulationState.h"
#include "Car/Car.h"
#include "Utill/DebugConsole.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
    // 순위를 매기는 주기. 매 프레임 정렬할 이유가 없고, 너무 잦으면 접전 중에 순위가
    // 떨렸다 돌아오면서 추월 카운트가 부풀려진다.
    constexpr float STANDINGS_INTERVAL = 0.25f;
    // 같은 두 대의 접촉을 이 시간 안에는 다시 기록하지 않는다(한 번 부딪히면 여러 스텝 붙어 있다).
    constexpr float CONTACT_LOG_COOLDOWN = 1.5f;
}

TrafficSignal::Color SimulationState::GetSignalColor(float phaseOffset, float greenDuration, float yellowDuration,
                                                     float redDuration) const
{
    return TrafficSignal::GetColor(greenDuration, yellowDuration, redDuration, phaseOffset, m_simTime);
}

void SimulationState::UnregisterCar(Car *car)
{
    m_cars.erase(std::remove(m_cars.begin(), m_cars.end(), car), m_cars.end());
}

Car *SimulationState::FindCarByBody(JPH::BodyID id) const
{
    for (Car *car : m_cars)
        if (car->GetBodyID() == id)
            return car;
    return nullptr; // 도로나 장애물 등 차가 아닌 바디
}

void SimulationState::UpdateStandings()
{
    if (m_cars.size() < 2 || m_simTime - m_lastStandingsTime < STANDINGS_INTERVAL)
        return;
    m_lastStandingsTime = m_simTime;

    std::vector<Car *> ordered = m_cars;
    std::sort(ordered.begin(), ordered.end(),
              [](const Car *a, const Car *b) { return a->GetRaceProgress() > b->GetRaceProgress(); });

    for (size_t i = 0; i < ordered.size(); ++i)
        ordered[i]->UpdateRacePosition(static_cast<int>(i) + 1, m_simTime);
}

void SimulationState::OpenLogs()
{
    if (m_contactLog.is_open())
        return;

    m_contactLog.open("race_contacts.csv");
    m_contactLog << "time,carA,carB,posA,posB,s,radius,"
                    "stateA,sideA,offsetA,speedA,stateB,sideB,offsetB,speedB,"
                    "deltaS,lateral,bodyGap,aTargetsB,bTargetsA"
                 << std::endl;

    m_lapLog.open("race_laps.csv");
    m_lapLog << "time,car,lap,lapTime,bestLap,position,overtakes,overtakenBy" << std::endl;
}

bool SimulationState::ShouldLogContact(int idA, int idB)
{
    const std::pair<int, int> key(std::min(idA, idB), std::max(idA, idB));
    auto found = m_contactCooldown.find(key);
    if (found != m_contactCooldown.end() && m_simTime - found->second < CONTACT_LOG_COOLDOWN)
        return false;
    m_contactCooldown[key] = m_simTime;
    return true;
}

void SimulationState::LogContact(const Car &a, const Car &b)
{
    OpenLogs();
    ++m_contactCount;

    // 결승선을 사이에 두면 s 차이가 한 바퀴만큼 벌어진다. 짧은 쪽으로 되돌린다.
    float deltaS = b.GetRaceS() - a.GetRaceS();
    const float lapLength = a.GetLapLength();
    if (lapLength > 1.0f)
    {
        if (deltaS > lapLength * 0.5f)
            deltaS -= lapLength;
        else if (deltaS < -lapLength * 0.5f)
            deltaS += lapLength;
    }

    const float lateral = b.GetRaceD() - a.GetRaceD();
    const float bodyGap = std::fabs(lateral) - (a.GetHalfWidth() + b.GetHalfWidth());
    const float curvature = std::fabs(a.GetRaceCurvature());
    const float radius = curvature > 1e-6f ? 1.0f / curvature : 99999.0f;

    m_contactLog << m_simTime << "," << a.GetName() << "," << b.GetName() << ","
                 << a.GetRacePosition() << "," << b.GetRacePosition() << ","
                 << a.GetRaceS() << "," << radius << ","
                 << a.GetOvertakeStateName() << "," << a.GetOvertakeSide() << ","
                 << a.GetRaceLateralOffset() << "," << a.GetSpeed() * 3.6f << ","
                 << b.GetOvertakeStateName() << "," << b.GetOvertakeSide() << ","
                 << b.GetRaceLateralOffset() << "," << b.GetSpeed() * 3.6f << ","
                 << deltaS << "," << lateral << "," << bodyGap << ","
                 << (a.GetOvertakeTarget() == &b ? 1 : 0) << ","
                 << (b.GetOvertakeTarget() == &a ? 1 : 0) << std::endl;

    DebugConsole::Log("CONTACT " + a.GetName() + "(" + a.GetOvertakeStateName() + ") <-> " +
                      b.GetName() + "(" + b.GetOvertakeStateName() + ") @ s=" +
                      ToString(static_cast<int>(a.GetRaceS())) + " R=" + ToString(static_cast<int>(radius)));
}

void SimulationState::LogLap(const Car &car, float lapTime)
{
    OpenLogs();
    m_lapLog << m_simTime << "," << car.GetName() << "," << car.GetLap() << "," << lapTime << ","
             << car.GetBestLapTime() << "," << car.GetRacePosition() << "," << car.GetOvertakes() << ","
             << car.GetOvertakenBy() << std::endl;
}
