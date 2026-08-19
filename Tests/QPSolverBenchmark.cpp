#include "TestHarness.h"
#include "Car/RacingLineQP.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

namespace
{
    constexpr float PI = 3.14159265358979323846f;
    constexpr float SPACING = 1.0f;
    constexpr float LINE_ROOM = 3.0f;

    std::vector<float> MakeSCurve(size_t pointCount)
    {
        std::vector<float> curvature(pointCount);
        for (size_t i = 0; i < pointCount; ++i)
        {
            float t = static_cast<float>(i) / static_cast<float>(pointCount - 1);
            curvature[i] = 0.012f * std::sin(4.0f * PI * t) +
                           0.004f * std::sin(12.0f * PI * t);
        }
        return curvature;
    }

    double CurvatureSquaredObjective(const std::vector<float> &reference,
                                     const std::vector<float> &offsets)
    {
        double sum = 0.0;
        for (size_t i = 1; i + 1 < reference.size(); ++i)
        {
            double offsetSecondDiff = static_cast<double>(offsets[i - 1]) -
                                      2.0 * static_cast<double>(offsets[i]) +
                                      static_cast<double>(offsets[i + 1]);
            double curvature = static_cast<double>(reference[i]) -
                               offsetSecondDiff / (SPACING * SPACING);
            sum += curvature * curvature;
        }
        return sum;
    }

    double TimeSolver(const std::vector<float> &reference, int repetitions, float &checksum)
    {
        auto begin = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < repetitions; ++i)
        {
            std::vector<float> offsets = RacingLineQP::SolveMinCurvatureCascade(
                reference, SPACING, -LINE_ROOM, LINE_ROOM, 0.0f);
            checksum += offsets[offsets.size() / 2];
        }
        auto end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::micro>(end - begin).count() /
               static_cast<double>(repetitions);
    }
}

TEST_CASE(RacingLineQP_StraightRoadStaysCentered)
{
    std::vector<float> reference(256, 0.0f);
    std::vector<float> offsets = RacingLineQP::SolveMinCurvatureCascade(
        reference, SPACING, -LINE_ROOM, LINE_ROOM, 0.0f);

    CHECK(offsets.size() == reference.size());
    for (float offset : offsets)
        CHECK(std::fabs(offset) < 1e-6f);
}

TEST_CASE(RacingLineQP_RespectsBoundsAndReducesCurvature)
{
    std::vector<float> reference = MakeSCurve(512);
    std::vector<float> centered(reference.size(), 0.0f);
    std::vector<float> offsets = RacingLineQP::SolveMinCurvatureCascade(
        reference, SPACING, -LINE_ROOM, LINE_ROOM, 0.0f);

    CHECK(offsets.size() == reference.size());
    for (float offset : offsets)
    {
        CHECK(std::isfinite(offset));
        CHECK(offset >= -LINE_ROOM);
        CHECK(offset <= LINE_ROOM);
    }

    double centeredObjective = CurvatureSquaredObjective(reference, centered);
    double solvedObjective = CurvatureSquaredObjective(reference, offsets);
    std::printf("\n-- racing-line QP quality --\n");
    std::printf("  curvature-squared objective: centered %.8f -> solved %.8f (%.2f%%)\n",
                centeredObjective, solvedObjective,
                100.0 * solvedObjective / centeredObjective);
    CHECK(solvedObjective < centeredObjective);
}

TEST_CASE(Benchmark_RacingLineQP_Scaling)
{
    std::printf("\n-- racing-line QP solve time --\n");
    std::printf("  points | average solve | time/point\n");

    float checksum = 0.0f;
    for (size_t pointCount : {64u, 256u, 1024u, 4096u})
    {
        std::vector<float> reference = MakeSCurve(pointCount);
        const int repetitions = pointCount <= 256 ? 20 : 8;

        TimeSolver(reference, 2, checksum); // warm-up
        double averageUs = TimeSolver(reference, repetitions, checksum);
        std::printf("  %6zu | %10.2f us | %10.3f us\n",
                    pointCount, averageUs, averageUs / static_cast<double>(pointCount));
    }
    std::printf("  checksum: %.6f\n\n", checksum);
    CHECK(std::isfinite(checksum));
}
