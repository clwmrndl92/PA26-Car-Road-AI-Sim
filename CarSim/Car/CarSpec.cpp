#include "CarSpec.h"
#include <algorithm>

const CarSpec &GetCarSpec(CarType type)
{
        static const CarSpec specs[] = {
            CarSpec("Car_0", "Model\\car_1.obj", JPH::Vec3(0.9919f, 0.9674f, 2.1204f), JPH::Vec3(0.0f, 0.0f, 1.5f),
                    JPH::Vec3(0.0f, 0.96f, 1.5f), 3.0f, 1300.0f),
            CarSpec("Car_1", "Model\\car_2.obj", JPH::Vec3(1.3421f, 0.9073f, 2.8342f), JPH::Vec3(0.0f, 0.0f, 1.5f),
                    JPH::Vec3(0.0f, 0.9f, 1.5f), 3.40f, 1600.0f),
            CarSpec("Car_Jeep", "Model\\car_jeep.obj", JPH::Vec3(1.3952f, 0.9922f, 2.5856f), JPH::Vec3(0.0f, 0.0f, 1.65f),
                    JPH::Vec3(0.0f, 0.99f, 1.65f), 3.26f, 1900.0f),
            CarSpec("Car_LittleTruck", "Model\\car_littletruck.obj", JPH::Vec3(1.1088f, 1.1468f, 2.6419f),
                    JPH::Vec3(0.0f, 0.0f, 1.72f), JPH::Vec3(0.0f, 1.15f, 1.72f), 3.42f, 1850.0f),
            CarSpec("Car_Van", "Model\\car_van.obj", JPH::Vec3(1.1859f, 0.9729f, 2.8078f), JPH::Vec3(0.0f, 0.0f, 1.8f),
                    JPH::Vec3(0.0f, 0.97f, 1.8f), 3.58f, 2000.0f),
        };
        return specs[static_cast<size_t>(type)];
}

namespace
{
    // 결정론적 해시. rand()를 쓰면 스폰 순서가 조금만 달라져도 그리드 전체가 바뀌어
    // "어제 그 코너에서 나던 추월"을 다시 볼 수 없다.
    float Jitter01(unsigned int seed, unsigned int axis)
    {
        unsigned int h = seed * 747796405u + axis * 2891336453u + 1u;
        h ^= h >> 15;
        h *= 2246822519u;
        h ^= h >> 13;
        return static_cast<float>(h & 0xFFFFFFu) / static_cast<float>(0xFFFFFF);
    }

    // [-spread, +spread] 균등. seed 0은 지터 없음(프리셋용).
    float Spread(unsigned int seed, unsigned int axis, float spread)
    {
        if (seed == 0u)
            return 0.0f;
        return (Jitter01(seed, axis) * 2.0f - 1.0f) * spread;
    }

    float Lerp(float a, float b, float t) { return a + (b - a) * t; }
}

// 루키 <-> 에이스를 skill로 보간한 뒤 축마다 독립적으로 흔든다.
// 등급 3종으로만 나누면 30대 그리드에 똑같은 차가 열 대씩 생겨 갭이 영원히 안 벌어진다.
CarPersonality MakeRacerPersonality(float skill, unsigned int jitterSeed)
{
    skill = std::clamp(skill, 0.0f, 1.0f);

    CarPersonality personality;
    // 최고속은 직선에서만 벌어지므로 폭을 좁게 둔다. 넓히면 한 랩 만에 줄이 늘어져 배틀이 사라진다.
    personality.topSpeedFactor = Lerp(0.95f, 1.03f, skill) + Spread(jitterSeed, 0u, 0.015f);
    // 그립은 코너 통과속도를 지배한다(v = sqrt(grip/곡률)). 랩타임 차이의 대부분이 여기서 나온다.
    personality.gripFactor = Lerp(0.88f, 1.06f, skill) + Spread(jitterSeed, 1u, 0.03f);
    personality.accelFactor = Lerp(0.92f, 1.05f, skill) + Spread(jitterSeed, 2u, 0.025f);
    personality.brakeFactor = Lerp(0.85f, 1.05f, skill) + Spread(jitterSeed, 3u, 0.03f);
    // 잘하는 드라이버일수록 바짝 붙어 압박한다(작을수록 좁은 차간).
    personality.headwayFactor = Lerp(1.35f, 0.75f, skill) + Spread(jitterSeed, 4u, 0.08f);
    personality.jerkUp = Lerp(22.0f, 36.0f, skill);
    personality.jerkDown = Lerp(45.0f, 70.0f, skill);
    return personality;
}

CarPersonality GetCarPersonality(CarPersonalityType type)
{
    switch (type)
    {
    case CarPersonalityType::Ace:
        return MakeRacerPersonality(1.0f, 0u);
    case CarPersonalityType::Rookie:
        return MakeRacerPersonality(0.0f, 0u);
    default:
        return MakeRacerPersonality(0.5f, 0u);
    }
}
