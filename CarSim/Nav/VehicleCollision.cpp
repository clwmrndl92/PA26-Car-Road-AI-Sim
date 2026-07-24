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
            Vec3 fwdA(std::cos(headingRadA), 0.0f, std::sin(headingRadA));
            Vec3 rightA(-fwdA.GetZ(), 0.0f, fwdA.GetX());
            Vec3 fwdB(std::cos(headingRadB), 0.0f, std::sin(headingRadB));
            Vec3 rightB(-fwdB.GetZ(), 0.0f, fwdB.GetX());

            Vec3 d = centerB - centerA;
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
