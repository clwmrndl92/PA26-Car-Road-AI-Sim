#include "VehicleCollision.h"
#include <cmath>

namespace VehicleCollision
{
    namespace
    {
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

    bool IsColliding(const Vec3 &position, float headingRad,
                     const std::vector<Obstacle> &obstacles, const VehicleShape &shape)
    {
        Vec3 forward(std::cos(headingRad), 0.0f, std::sin(headingRad));
        Vec3 bodyCenter = position + forward * shape.pivotToCenter;

        for (const Obstacle &obstacle : obstacles)
        {
            if (ObbOverlap(bodyCenter, shape.halfLength, shape.halfWidth, headingRad,
                           obstacle.center, obstacle.halfLength, obstacle.halfWidth, obstacle.headingRad))
                return true;
        }
        return false;
    }
}
