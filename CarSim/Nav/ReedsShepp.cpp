#include "ReedsShepp.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <utility>

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

        // theta를 [-pi, pi) 범위로 정규화.
        double Mod2Pi(double theta)
        {
            theta = std::fmod(theta, 2.0 * PI);
            if (theta < -PI)
                return theta + 2.0 * PI;
            if (theta >= PI)
                return theta - 2.0 * PI;
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
        void ChangeOfBasis(double x1, double y1, double theta1,
                           double x2, double y2, double theta2,
                           double &outX, double &outY, double &outTheta)
        {
            double dx = x2 - x1;
            double dy = y2 - y1;
            outX = dx * std::cos(theta1) + dy * std::sin(theta1);
            outY = -dx * std::sin(theta1) + dy * std::cos(theta1);
            outTheta = theta2 - theta1;
        }

        Path Path1(double x, double y, double phi)
        {
            // Formula 8.1: CSC (same turns)
            Path path;

            double u, t;
            ToPolar(x - std::sin(phi), y - 1 + std::cos(phi), u, t);
            double v = Mod2Pi(phi - t);

            path.push_back(MakeElement(t, Steering::Left, Gear::Forward));
            path.push_back(MakeElement(u, Steering::Straight, Gear::Forward));
            path.push_back(MakeElement(v, Steering::Left, Gear::Forward));

            return path;
        }

        Path Path2(double x, double y, double phi)
        {
            // Formula 8.2: CSC (opposite turns)
            phi = Mod2Pi(phi);
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

        Path Path3(double x, double y, double phi)
        {
            // Formula 8.3: C|C|C
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

        Path Path4(double x, double y, double phi)
        {
            // Formula 8.4 (1): C|CC
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

        Path Path5(double x, double y, double phi)
        {
            // Formula 8.4 (2): CC|C
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

        Path Path6(double x, double y, double phi)
        {
            // Formula 8.7: CCu|CuC
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

        Path Path7(double x, double y, double phi)
        {
            // Formula 8.8: C|CuCu|C
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

        Path Path8(double x, double y, double phi)
        {
            // Formula 8.9 (1): C|C[pi/2]SC
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

        Path Path9(double x, double y, double phi)
        {
            // Formula 8.9 (2): CSC[pi/2]|C
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

        Path Path10(double x, double y, double phi)
        {
            // Formula 8.10 (1): C|C[pi/2]SC
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

        Path Path11(double x, double y, double phi)
        {
            // Formula 8.10 (2): CSC[pi/2]|C
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

        Path Path12(double x, double y, double phi)
        {
            // Formula 8.11: C|C[pi/2]SC[pi/2]|C
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

        // 논문 말미에 설명된 timeflip 변환: 모든 세그먼트의 기어를 반전. path를 값으로 받아
        // 제자리에서 수정하므로 임시 경로(prvalue)를 넘기면 추가 복사가 생기지 않는다.
        Path Timeflip(Path path)
        {
            for (auto &e : path)
                e.gear = (e.gear == Gear::Forward) ? Gear::Backward : Gear::Forward;
            return path;
        }

        // 논문 말미에 설명된 reflect 변환: 모든 세그먼트의 조향을 반전.
        Path Reflect(Path path)
        {
            for (auto &e : path)
            {
                if (e.steering == Steering::Left)
                    e.steering = Steering::Right;
                else if (e.steering == Steering::Right)
                    e.steering = Steering::Left;
            }
            return path;
        }

        // 세그먼트 하나를 적분해 월드좌표 폴리라인 점들을 points에 append하고, (x,z,theta)를
        // 그 세그먼트 끝의 pose로 갱신한다. SamplePath/SampleLegs가 공유하는 핵심 적분 로직.
        void AppendElementSamples(const PathElement &element, double &x, double y, double &z, double &theta,
                                  float turningRadius, float sampleSpacing, std::vector<Vec3> &points)
        {
            double g = (element.gear == Gear::Backward) ? -1.0 : 1.0;
            double kappaLabel = 0.0;
            if (element.steering == Steering::Left)
                kappaLabel = 1.0 / turningRadius;
            else if (element.steering == Steering::Right)
                kappaLabel = -1.0 / turningRadius;

            int sampleCount = std::max(1, static_cast<int>(element.param / sampleSpacing));
            double ds = element.param / sampleCount;

            if (std::fabs(kappaLabel) < 1e-9)
            {
                // 직선: 현재 heading 방향으로 기어에 따른 부호만큼 전진.
                for (int i = 1; i <= sampleCount; ++i)
                {
                    double s = ds * i;
                    double px = x + g * s * std::cos(theta);
                    double pz = z + g * s * std::sin(theta);
                    points.push_back(Vec3(static_cast<float>(px), static_cast<float>(y), static_cast<float>(pz)));
                }
                x += g * element.param * std::cos(theta);
                z += g * element.param * std::sin(theta);
            }
            else
            {
                // 곡선: 원 중심은 기어와 무관하게 고정, 헤딩만 기어에 따라 반대 방향으로 돈다.
                double invKappa = 1.0 / kappaLabel;
                double cx = x - invKappa * std::sin(theta);
                double cz = z + invKappa * std::cos(theta);
                double kappaActual = g * kappaLabel;
                for (int i = 1; i <= sampleCount; ++i)
                {
                    double s = ds * i;
                    double th = theta + kappaActual * s;
                    double px = cx + invKappa * std::sin(th);
                    double pz = cz - invKappa * std::cos(th);
                    points.push_back(Vec3(static_cast<float>(px), static_cast<float>(y), static_cast<float>(pz)));
                }
                theta += kappaActual * element.param;
                x = cx + invKappa * std::sin(theta);
                z = cz - invKappa * std::cos(theta);
            }
        }

        // param==0 세그먼트 제거. 길이에는 영향이 없지만(0을 더할 뿐) 반환 경로의 표현을
        // 원본과 동일하게 맞추기 위해 유지한다.
        void StripZeroParams(Path &path)
        {
            path.erase(std::remove_if(path.begin(), path.end(),
                                      [](const PathElement &e)
                                      { return e.param == 0.0f; }),
                       path.end());
        }

        constexpr double REVERSE_WEIGHT = 1.75;      // 후진 페널티
        constexpr double TURN_WEIGHT = 1.3;          // 회전 페널티
        constexpr double GEAR_CHANGE_PENALTY = 3.0;  // 기어변경 페널티
        constexpr double STEER_CHANGE_PENALTY = 1.0; // 회전 방향 변경 페널티

        // 세그먼트 하나의 비용: 실제 거리에 회전/후진 가중치를 곱한다(직진-전진이 가중치 1.0 기준).
        double SegmentCost(const PathElement &e)
        {
            double weight = 1.0;
            if (e.steering != Steering::Straight)
                weight *= TURN_WEIGHT;
            if (e.gear == Gear::Backward)
                weight *= REVERSE_WEIGHT;
            return static_cast<double>(e.param) * weight;
        }

        // 경로 전체 비용 = 세그먼트별 거리 비용 합 + 기어 전환(Cusp) 횟수 패널티 + 인접한 회전
        // 세그먼트끼리 좌/우가 바뀌는 횟수 패널티(사이에 직진이 끼면 카운트하지 않음 — 직진을 섞어
        // 회전 상태를 완화하는 건 사람도 편하게 느끼므로). 값이 작을수록 사람이 운전할 법한 경로.
        double PathCost(const Path &path)
        {
            double cost = 0.0;
            for (const PathElement &e : path)
                cost += SegmentCost(e);

            for (size_t i = 1; i < path.size(); ++i)
            {
                if (path[i].gear != path[i - 1].gear)
                    cost += GEAR_CHANGE_PENALTY;
                if (path[i].steering != Steering::Straight && path[i - 1].steering != Steering::Straight &&
                    path[i].steering != path[i - 1].steering)
                    cost += STEER_CHANGE_PENALTY;
            }

            return cost;
        }

        // 12개 공식 각각에 4가지 변형(원본/timeflip/reflect/둘 다)을 적용한 최대 48개 후보 중
        // PathCost가 가장 낮은(=사람이 운전할 법한) 경로 하나만 생성/보관해서 돌려준다 (반경 1
        // 기준 정규화 좌표 x, y, phi(rad)). 후보 벡터를 물리적으로 만들지 않으므로 매 호출의 힙
        // 할당이 48개 → 최대 2개로 줄어든다. 유효 경로가 없으면 빈 경로를 반환한다.
        Path GetBestPath(double x, double y, double phi)
        {
            Path best;
            double bestCost = std::numeric_limits<double>::max();

            // 생성 순서(공식별 원본→timeflip→reflect→둘 다)와 strict-less 비교를 그대로 유지해
            // 동점 시 먼저 나온 후보를 고른다.
            auto consider = [&](Path candidate)
            {
                StripZeroParams(candidate);
                if (candidate.empty())
                    return;

                double cost = PathCost(candidate);
                if (cost >= bestCost)
                    return;

                bestCost = cost;
                best = std::move(candidate);
            };

            for (PathFn fn : kPathFns)
            {
                consider(fn(x, y, phi));
                consider(Timeflip(fn(-x, y, -phi)));
                consider(Reflect(fn(x, -y, -phi)));
                consider(Reflect(Timeflip(fn(-x, -y, phi))));
            }

            return best;
        }
    }

    float GetPathLength(const Path &path)
    {
        float total = 0.0f;
        for (const auto &e : path)
            total += e.param;
        return total;
    }

    Path GetOptimalPath(const Vec3 &start, float startAngleRad,
                        const Vec3 &end, float endAngleRad,
                        float turningRadius)
    {
        double localX, localY, localTheta;
        ChangeOfBasis(start.GetX(), start.GetZ(), startAngleRad,
                      end.GetX(), end.GetZ(), endAngleRad,
                      localX, localY, localTheta);

        // 공식들은 회전 반경 1을 가정하므로 위치만 반경으로 정규화한다 (각도는 스케일 불변).
        localX /= turningRadius;
        localY /= turningRadius;

        Path best = GetBestPath(localX, localY, localTheta);
        if (best.empty())
            return {};

        for (auto &e : best)
            e.param *= turningRadius;
        return best;
    }

    std::vector<Vec3> SamplePath(const Path &path, const Vec3 &start, float startAngleRad,
                                 float turningRadius, float sampleSpacing)
    {
        std::vector<Vec3> points;
        if (path.empty() || turningRadius <= 0.0f || sampleSpacing <= 0.0f)
            return points;

        double x = start.GetX();
        double z = start.GetZ();
        double y = start.GetY();
        double theta = startAngleRad;

        points.push_back(Vec3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
        for (const PathElement &element : path)
            AppendElementSamples(element, x, y, z, theta, turningRadius, sampleSpacing, points);

        return points;
    }

    namespace
    {
        // 경로 전체의 마지막 leg 끝을 이 길이만큼 최종 헤딩 방향으로 연장한다.
        constexpr float FINAL_ALIGN_EXTENSION_LENGTH = 3.0f;

        size_t ExtendFinalLegForAlignment(std::vector<Vec3> &legPoints, float sampleSpacing)
        {
            size_t endIndex = legPoints.size() - 1;
            if (legPoints.size() < 2)
                return endIndex;

            Vec3 tangent = legPoints[endIndex] - legPoints[endIndex - 1];
            float tangentLength = tangent.Length();
            if (tangentLength < 1e-6f)
                return endIndex;
            tangent = tangent * (1.0f / tangentLength);

            int extraSamples = std::max(1, static_cast<int>(FINAL_ALIGN_EXTENSION_LENGTH / sampleSpacing));
            for (int i = 1; i <= extraSamples; ++i)
                legPoints.push_back(legPoints[endIndex] + tangent * (sampleSpacing * static_cast<float>(i)));

            return endIndex;
        }
    }

    std::vector<Leg> SampleLegs(const Path &path, const Vec3 &start, float startAngleRad,
                                float turningRadius, float sampleSpacing)
    {
        std::vector<Leg> legs;
        if (path.empty() || turningRadius <= 0.0f || sampleSpacing <= 0.0f)
            return legs;

        double x = start.GetX();
        double z = start.GetZ();
        double y = start.GetY();
        double theta = startAngleRad;

        Gear legGear = path.front().gear;
        std::vector<Vec3> legPoints{Vec3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z))};

        for (const PathElement &element : path)
        {
            if (element.gear != legGear)
            {
                size_t cuspEndIndex = legPoints.size() - 1;
                legs.push_back(Leg{std::move(legPoints), legGear, cuspEndIndex});
                legGear = element.gear;
                legPoints = {Vec3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z))};
            }
            AppendElementSamples(element, x, y, z, theta, turningRadius, sampleSpacing, legPoints);
        }
        size_t finalEndIndex = ExtendFinalLegForAlignment(legPoints, sampleSpacing);
        legs.push_back(Leg{std::move(legPoints), legGear, finalEndIndex});

        return legs;
    }
}
