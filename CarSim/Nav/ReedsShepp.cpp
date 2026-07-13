#include "ReedsShepp.h"
#include <cmath>
#include <algorithm>
#include <limits>

// 참고 구현(reeds-shepp-curves-master, Python)을 그대로 포팅함.
// 논문: Reeds, J.A.; Shepp, L.A. "Optimal paths for a car that goes both forwards
// and backwards." Pacific J. Math. 145 (1990), no. 2, 367-393.
// 아래 12개 pathN 함수는 각각 논문 8.1~8.11절 공식(각 4가지 word 중 1개)에 대응한다.
// 모든 좌표/각도 계산은 회전 반경 1을 기준으로 하며(원본과 동일), 실제 거리로의
// 스케일링은 GetOptimalPath 진입/종료 지점에서만 처리한다.

namespace ReedsShepp
{
    namespace
    {
        constexpr double PI = 3.14159265358979323846;

        double DegToRad(double deg) { return PI * deg / 180.0; }

        // theta를 [-pi, pi) 범위로 정규화.
        double Mod2Pi(double theta)
        {
            theta = std::fmod(theta, 2.0 * PI);
            if (theta < -PI) return theta + 2.0 * PI;
            if (theta >= PI) return theta - 2.0 * PI;
            return theta;
        }

        // (x, y)의 극좌표 (r, theta)를 반환.
        void ToPolar(double x, double y, double &r, double &theta)
        {
            r = std::sqrt(x * x + y * y);
            theta = std::atan2(y, x);
        }

        PathElement MakeElement(double param, Steering steering, Gear gear)
        {
            if (param >= 0)
                return PathElement{static_cast<float>(param), steering, gear};
            gear = (gear == Gear::Forward) ? Gear::Backward : Gear::Forward;
            return PathElement{static_cast<float>(-param), steering, gear};
        }

        // start(x1,y1,theta1)를 원점/각도 0으로 하는 좌표계에서 end의 좌표를 계산.
        void ChangeOfBasis(double x1, double y1, double theta1Deg,
                            double x2, double y2, double theta2Deg,
                            double &outX, double &outY, double &outThetaDeg)
        {
            double theta1 = DegToRad(theta1Deg);
            double dx = x2 - x1;
            double dy = y2 - y1;
            outX = dx * std::cos(theta1) + dy * std::sin(theta1);
            outY = -dx * std::sin(theta1) + dy * std::cos(theta1);
            outThetaDeg = theta2Deg - theta1Deg;
        }

        Path Path1(double x, double y, double phiDeg)
        {
            // Formula 8.1: CSC (same turns)
            double phi = DegToRad(phiDeg);
            Path path;

            double u, t;
            ToPolar(x - std::sin(phi), y - 1 + std::cos(phi), u, t);
            double v = Mod2Pi(phi - t);

            path.push_back(MakeElement(t, Steering::Left, Gear::Forward));
            path.push_back(MakeElement(u, Steering::Straight, Gear::Forward));
            path.push_back(MakeElement(v, Steering::Left, Gear::Forward));

            return path;
        }

        Path Path2(double x, double y, double phiDeg)
        {
            // Formula 8.2: CSC (opposite turns)
            double phi = Mod2Pi(DegToRad(phiDeg));
            Path path;

            double rho, t1;
            ToPolar(x + std::sin(phi), y - 1 - std::cos(phi), rho, t1);

            if (rho * rho >= 4)
            {
                double u = std::sqrt(rho * rho - 4);
                double t = Mod2Pi(t1 + std::atan2(2.0, u));
                double v = Mod2Pi(t - phi);

                path.push_back(MakeElement(t, Steering::Left, Gear::Forward));
                path.push_back(MakeElement(u, Steering::Straight, Gear::Forward));
                path.push_back(MakeElement(v, Steering::Right, Gear::Forward));
            }

            return path;
        }

        Path Path3(double x, double y, double phiDeg)
        {
            // Formula 8.3: C|C|C
            double phi = DegToRad(phiDeg);
            Path path;

            double xi = x - std::sin(phi);
            double eta = y - 1 + std::cos(phi);
            double rho, theta;
            ToPolar(xi, eta, rho, theta);

            if (rho <= 4)
            {
                double A = std::acos(rho / 4);
                double t = Mod2Pi(theta + PI / 2 + A);
                double u = Mod2Pi(PI - 2 * A);
                double v = Mod2Pi(phi - t - u);

                path.push_back(MakeElement(t, Steering::Left, Gear::Forward));
                path.push_back(MakeElement(u, Steering::Right, Gear::Backward));
                path.push_back(MakeElement(v, Steering::Left, Gear::Forward));
            }

            return path;
        }

        Path Path4(double x, double y, double phiDeg)
        {
            // Formula 8.4 (1): C|CC
            double phi = DegToRad(phiDeg);
            Path path;

            double xi = x - std::sin(phi);
            double eta = y - 1 + std::cos(phi);
            double rho, theta;
            ToPolar(xi, eta, rho, theta);

            if (rho <= 4)
            {
                double A = std::acos(rho / 4);
                double t = Mod2Pi(theta + PI / 2 + A);
                double u = Mod2Pi(PI - 2 * A);
                double v = Mod2Pi(t + u - phi);

                path.push_back(MakeElement(t, Steering::Left, Gear::Forward));
                path.push_back(MakeElement(u, Steering::Right, Gear::Backward));
                path.push_back(MakeElement(v, Steering::Left, Gear::Backward));
            }

            return path;
        }

        Path Path5(double x, double y, double phiDeg)
        {
            // Formula 8.4 (2): CC|C
            double phi = DegToRad(phiDeg);
            Path path;

            double xi = x - std::sin(phi);
            double eta = y - 1 + std::cos(phi);
            double rho, theta;
            ToPolar(xi, eta, rho, theta);

            if (rho <= 4)
            {
                double u = std::acos(1 - rho * rho / 8);
                double A = std::asin(2 * std::sin(u) / rho);
                double t = Mod2Pi(theta + PI / 2 - A);
                double v = Mod2Pi(t - u - phi);

                path.push_back(MakeElement(t, Steering::Left, Gear::Forward));
                path.push_back(MakeElement(u, Steering::Right, Gear::Forward));
                path.push_back(MakeElement(v, Steering::Left, Gear::Backward));
            }

            return path;
        }

        Path Path6(double x, double y, double phiDeg)
        {
            // Formula 8.7: CCu|CuC
            double phi = DegToRad(phiDeg);
            Path path;

            double xi = x + std::sin(phi);
            double eta = y - 1 - std::cos(phi);
            double rho, theta;
            ToPolar(xi, eta, rho, theta);

            if (rho <= 4)
            {
                double A, t, u, v;
                if (rho <= 2)
                {
                    A = std::acos((rho + 2) / 4);
                    t = Mod2Pi(theta + PI / 2 + A);
                    u = Mod2Pi(A);
                    v = Mod2Pi(phi - t + 2 * u);
                }
                else
                {
                    A = std::acos((rho - 2) / 4);
                    t = Mod2Pi(theta + PI / 2 - A);
                    u = Mod2Pi(PI - A);
                    v = Mod2Pi(phi - t + 2 * u);
                }

                path.push_back(MakeElement(t, Steering::Left, Gear::Forward));
                path.push_back(MakeElement(u, Steering::Right, Gear::Forward));
                path.push_back(MakeElement(u, Steering::Left, Gear::Backward));
                path.push_back(MakeElement(v, Steering::Right, Gear::Backward));
            }

            return path;
        }

        Path Path7(double x, double y, double phiDeg)
        {
            // Formula 8.8: C|CuCu|C
            double phi = DegToRad(phiDeg);
            Path path;

            double xi = x + std::sin(phi);
            double eta = y - 1 - std::cos(phi);
            double rho, theta;
            ToPolar(xi, eta, rho, theta);
            double u1 = (20 - rho * rho) / 16;

            if (rho <= 6 && 0 <= u1 && u1 <= 1)
            {
                double u = std::acos(u1);
                double A = std::asin(2 * std::sin(u) / rho);
                double t = Mod2Pi(theta + PI / 2 + A);
                double v = Mod2Pi(t - phi);

                path.push_back(MakeElement(t, Steering::Left, Gear::Forward));
                path.push_back(MakeElement(u, Steering::Right, Gear::Backward));
                path.push_back(MakeElement(u, Steering::Left, Gear::Backward));
                path.push_back(MakeElement(v, Steering::Right, Gear::Forward));
            }

            return path;
        }

        Path Path8(double x, double y, double phiDeg)
        {
            // Formula 8.9 (1): C|C[pi/2]SC
            double phi = DegToRad(phiDeg);
            Path path;

            double xi = x - std::sin(phi);
            double eta = y - 1 + std::cos(phi);
            double rho, theta;
            ToPolar(xi, eta, rho, theta);

            if (rho >= 2)
            {
                double u = std::sqrt(rho * rho - 4) - 2;
                double A = std::atan2(2.0, u + 2);
                double t = Mod2Pi(theta + PI / 2 + A);
                double v = Mod2Pi(t - phi + PI / 2);

                path.push_back(MakeElement(t, Steering::Left, Gear::Forward));
                path.push_back(MakeElement(PI / 2, Steering::Right, Gear::Backward));
                path.push_back(MakeElement(u, Steering::Straight, Gear::Backward));
                path.push_back(MakeElement(v, Steering::Left, Gear::Backward));
            }

            return path;
        }

        Path Path9(double x, double y, double phiDeg)
        {
            // Formula 8.9 (2): CSC[pi/2]|C
            double phi = DegToRad(phiDeg);
            Path path;

            double xi = x - std::sin(phi);
            double eta = y - 1 + std::cos(phi);
            double rho, theta;
            ToPolar(xi, eta, rho, theta);

            if (rho >= 2)
            {
                double u = std::sqrt(rho * rho - 4) - 2;
                double A = std::atan2(u + 2, 2.0);
                double t = Mod2Pi(theta + PI / 2 - A);
                double v = Mod2Pi(t - phi - PI / 2);

                path.push_back(MakeElement(t, Steering::Left, Gear::Forward));
                path.push_back(MakeElement(u, Steering::Straight, Gear::Forward));
                path.push_back(MakeElement(PI / 2, Steering::Right, Gear::Forward));
                path.push_back(MakeElement(v, Steering::Left, Gear::Backward));
            }

            return path;
        }

        Path Path10(double x, double y, double phiDeg)
        {
            // Formula 8.10 (1): C|C[pi/2]SC
            double phi = DegToRad(phiDeg);
            Path path;

            double xi = x + std::sin(phi);
            double eta = y - 1 - std::cos(phi);
            double rho, theta;
            ToPolar(xi, eta, rho, theta);

            if (rho >= 2)
            {
                double t = Mod2Pi(theta + PI / 2);
                double u = rho - 2;
                double v = Mod2Pi(phi - t - PI / 2);

                path.push_back(MakeElement(t, Steering::Left, Gear::Forward));
                path.push_back(MakeElement(PI / 2, Steering::Right, Gear::Backward));
                path.push_back(MakeElement(u, Steering::Straight, Gear::Backward));
                path.push_back(MakeElement(v, Steering::Right, Gear::Backward));
            }

            return path;
        }

        Path Path11(double x, double y, double phiDeg)
        {
            // Formula 8.10 (2): CSC[pi/2]|C
            double phi = DegToRad(phiDeg);
            Path path;

            double xi = x + std::sin(phi);
            double eta = y - 1 - std::cos(phi);
            double rho, theta;
            ToPolar(xi, eta, rho, theta);

            if (rho >= 2)
            {
                double t = Mod2Pi(theta);
                double u = rho - 2;
                double v = Mod2Pi(phi - t - PI / 2);

                path.push_back(MakeElement(t, Steering::Left, Gear::Forward));
                path.push_back(MakeElement(u, Steering::Straight, Gear::Forward));
                path.push_back(MakeElement(PI / 2, Steering::Left, Gear::Forward));
                path.push_back(MakeElement(v, Steering::Right, Gear::Backward));
            }

            return path;
        }

        Path Path12(double x, double y, double phiDeg)
        {
            // Formula 8.11: C|C[pi/2]SC[pi/2]|C
            double phi = DegToRad(phiDeg);
            Path path;

            double xi = x + std::sin(phi);
            double eta = y - 1 - std::cos(phi);
            double rho, theta;
            ToPolar(xi, eta, rho, theta);

            if (rho >= 4)
            {
                double u = std::sqrt(rho * rho - 4) - 4;
                double A = std::atan2(2.0, u + 4);
                double t = Mod2Pi(theta + PI / 2 + A);
                double v = Mod2Pi(t - phi);

                path.push_back(MakeElement(t, Steering::Left, Gear::Forward));
                path.push_back(MakeElement(PI / 2, Steering::Right, Gear::Backward));
                path.push_back(MakeElement(u, Steering::Straight, Gear::Backward));
                path.push_back(MakeElement(PI / 2, Steering::Left, Gear::Backward));
                path.push_back(MakeElement(v, Steering::Right, Gear::Forward));
            }

            return path;
        }

        using PathFn = Path (*)(double, double, double);
        constexpr PathFn kPathFns[] = {
            Path1, Path2, Path3, Path4, Path5, Path6,
            Path7, Path8, Path9, Path10, Path11, Path12};

        // 논문 말미에 설명된 timeflip 변환: 모든 세그먼트의 기어를 반전.
        Path Timeflip(const Path &path)
        {
            Path result = path;
            for (auto &e : result)
                e.gear = (e.gear == Gear::Forward) ? Gear::Backward : Gear::Forward;
            return result;
        }

        // 논문 말미에 설명된 reflect 변환: 모든 세그먼트의 조향을 반전.
        Path Reflect(const Path &path)
        {
            Path result = path;
            for (auto &e : result)
            {
                if (e.steering == Steering::Left) e.steering = Steering::Right;
                else if (e.steering == Steering::Right) e.steering = Steering::Left;
            }
            return result;
        }

        // 12개 공식 각각에 대해 4가지 변형(원본/timeflip/reflect/둘 다)을 적용해
        // 최대 48개의 경로 후보를 생성한다 (반경 1 기준 정규화 좌표 x, y, phi(deg)).
        std::vector<Path> GetAllPaths(double x, double y, double phiDeg)
        {
            std::vector<Path> paths;
            paths.reserve(48);

            for (PathFn fn : kPathFns)
            {
                paths.push_back(fn(x, y, phiDeg));
                paths.push_back(Timeflip(fn(-x, y, -phiDeg)));
                paths.push_back(Reflect(fn(x, -y, -phiDeg)));
                paths.push_back(Reflect(Timeflip(fn(-x, -y, phiDeg))));
            }

            for (auto &path : paths)
            {
                path.erase(std::remove_if(path.begin(), path.end(),
                                           [](const PathElement &e) { return e.param == 0.0f; }),
                           path.end());
            }

            paths.erase(std::remove_if(paths.begin(), paths.end(),
                                        [](const Path &p) { return p.empty(); }),
                        paths.end());

            return paths;
        }
    }

    float GetPathLength(const Path &path)
    {
        float total = 0.0f;
        for (const auto &e : path)
            total += e.param;
        return total;
    }

    Path GetOptimalPath(const Vec3 &start, float startAngleDeg,
                        const Vec3 &end, float endAngleDeg,
                        float turningRadius)
    {
        double localX, localY, localThetaDeg;
        ChangeOfBasis(start.GetX(), start.GetZ(), startAngleDeg,
                      end.GetX(), end.GetZ(), endAngleDeg,
                      localX, localY, localThetaDeg);

        // 공식들은 회전 반경 1을 가정하므로 위치만 반경으로 정규화한다 (각도는 스케일 불변).
        localX /= turningRadius;
        localY /= turningRadius;

        std::vector<Path> candidates = GetAllPaths(localX, localY, localThetaDeg);
        if (candidates.empty())
            return {};

        const Path *best = &candidates.front();
        float bestLength = GetPathLength(*best);
        for (const auto &candidate : candidates)
        {
            float length = GetPathLength(candidate);
            if (length < bestLength)
            {
                bestLength = length;
                best = &candidate;
            }
        }

        Path result = *best;
        for (auto &e : result)
            e.param *= turningRadius;
        return result;
    }
}
