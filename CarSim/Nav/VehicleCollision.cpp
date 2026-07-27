#include "VehicleCollision.h"
#include <algorithm>
#include <cmath>
#include <utility>

namespace VehicleCollision
{
    namespace
    {
        // ray(origin, dir)를 obstacle 로컬 좌표계(forward=x, right=z)의 슬랩([-halfLength,halfLength]
        // x [-halfWidth,halfWidth])과 교차시킨다. 맞으면 [0, maxDistance] 범위의 진입 거리를 반환.
        std::pair<bool, float> RaySlabIntersect(const Vec3 &origin, const Vec3 &dir, float maxDistance,
                                                const Obstacle &obstacle)
        {
            // 바운딩 서클로 먼저 거른다: 광선이 [0, maxDistance] 구간 어디서도 이 원에 못 들어오면
            // 그 안의 회전된 사각형과도 절대 못 만나므로(false negative 없음), 아래 로컬좌표 변환
            // (cos/sin)과 슬랩 나눗셈을 생략한다. dir은 항상 단위벡터로 넘어온다(호출부 참고).
            float radius = std::sqrt(obstacle.halfLength * obstacle.halfLength + obstacle.halfWidth * obstacle.halfWidth);
            Vec3 toCenter = obstacle.center - origin;
            float tClosest = std::clamp(toCenter.Dot(dir), 0.0f, maxDistance);
            Vec3 offset = obstacle.center - (origin + dir * tClosest);
            float distSq = offset.GetX() * offset.GetX() + offset.GetZ() * offset.GetZ();
            if (distSq > radius * radius)
                return {false, 0.0f};

            Vec3 fwd(std::cos(obstacle.headingRad), 0.0f, std::sin(obstacle.headingRad));
            Vec3 right(-fwd.GetZ(), 0.0f, fwd.GetX());
            Vec3 toOrigin = origin - obstacle.center;

            float lo[2] = {toOrigin.Dot(fwd), toOrigin.Dot(right)};
            float ld[2] = {dir.Dot(fwd), dir.Dot(right)};
            float half[2] = {obstacle.halfLength, obstacle.halfWidth};

            float tmin = 0.0f;
            float tmax = maxDistance;
            for (int axis = 0; axis < 2; ++axis)
            {
                if (std::fabs(ld[axis]) < 1e-9f)
                {
                    if (lo[axis] < -half[axis] || lo[axis] > half[axis])
                        return {false, 0.0f};
                    continue;
                }
                float t1 = (-half[axis] - lo[axis]) / ld[axis];
                float t2 = (half[axis] - lo[axis]) / ld[axis];
                if (t1 > t2)
                    std::swap(t1, t2);
                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);
                if (tmin > tmax)
                    return {false, 0.0f};
            }
            return {true, tmin};
        }

        // 2D SAT: 두 회전된 사각형(중심+heading+반길이/반폭)이 겹치는지.
        bool ObbOverlap(const Vec3 &centerA, float halfLengthA, float halfWidthA, float headingRadA,
                        const Vec3 &centerB, float halfLengthB, float halfWidthB, float headingRadB)
        {
            Vec3 d = centerB - centerA;

            // 바운딩 서클(각 박스의 대각선 절반 = 어느 방향으로 회전해도 벗어날 수 없는 최대 반경)로
            // 먼저 거른다: 두 원이 안 겹치면 그 안의 사각형도 절대 겹칠 수 없으므로(false negative
            // 없음), 대부분의 "안 겹치는" 쌍은 아래 SAT(삼각함수 4번 포함)까지 갈 필요가 없다.
            float radiusA = std::sqrt(halfLengthA * halfLengthA + halfWidthA * halfWidthA);
            float radiusB = std::sqrt(halfLengthB * halfLengthB + halfWidthB * halfWidthB);
            float maxDist = radiusA + radiusB;
            float distSq = d.GetX() * d.GetX() + d.GetZ() * d.GetZ();
            if (distSq > maxDist * maxDist)
                return false;

            Vec3 fwdA(std::cos(headingRadA), 0.0f, std::sin(headingRadA));
            Vec3 rightA(-fwdA.GetZ(), 0.0f, fwdA.GetX());
            Vec3 fwdB(std::cos(headingRadB), 0.0f, std::sin(headingRadB));
            Vec3 rightB(-fwdB.GetZ(), 0.0f, fwdB.GetX());

            const Vec3 axes[4] = {fwdA, rightA, fwdB, rightB};
            for (const Vec3 &axis : axes)
            {
                float projA = halfLengthA * std::fabs(fwdA.Dot(axis)) + halfWidthA * std::fabs(rightA.Dot(axis));
                float projB = halfLengthB * std::fabs(fwdB.Dot(axis)) + halfWidthB * std::fabs(rightB.Dot(axis));
                float dist = std::fabs(d.Dot(axis));
                if (dist > projA + projB)
                    return false; // 분리축 발견 -> 안 겹침
            }
            return true;
        }
    }

    const Obstacle *FindColliding(const Vec3 &position, float headingRad,
                                  const std::vector<Obstacle> &obstacles, const VehicleShape &shape)
    {
        Vec3 forward(std::cos(headingRad), 0.0f, std::sin(headingRad));
        Vec3 bodyCenter = position + forward * shape.pivotToCenter;

        for (const Obstacle &obstacle : obstacles)
        {
            if (ObbOverlap(bodyCenter, shape.halfLength, shape.halfWidth, headingRad,
                           obstacle.center, obstacle.halfLength, obstacle.halfWidth, obstacle.headingRad))
                return &obstacle;
        }
        return nullptr;
    }

    bool IsColliding(const Vec3 &position, float headingRad,
                     const std::vector<Obstacle> &obstacles, const VehicleShape &shape)
    {
        return FindColliding(position, headingRad, obstacles, shape) != nullptr;
    }

    float RaycastObstacles(const Vec3 &origin, float directionRad, float maxDistance,
                           const std::vector<Obstacle> &obstacles)
    {
        Vec3 dir(std::cos(directionRad), 0.0f, std::sin(directionRad));
        float best = -1.0f;

        for (const Obstacle &obstacle : obstacles)
        {
            auto [hit, distance] = RaySlabIntersect(origin, dir, maxDistance, obstacle);
            if (hit && (best < 0.0f || distance < best))
                best = distance;
        }

        return best;
    }
}
