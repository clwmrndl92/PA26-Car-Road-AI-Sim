#include "RacingLineQP.h"

#include <algorithm>

namespace RacingLineQP
{
    namespace
    {
        // Minimum-curvature QP:
        //   min sum(k_i^2) + lengthWeight * sum(refCurvature_i * d_i)
        //   subject to dMin <= d_i <= dMax
        // The optimality stencil is solved with projected Gauss-Seidel (SOR).
        void SolveMinCurvature(const std::vector<float> &refCurvature, float spacing,
                               float dMin, float dMax, float lengthWeight,
                               int iterations, std::vector<float> &offsets)
        {
            const size_t n = refCurvature.size();
            if (n < 5 || offsets.size() != n)
                return;

            std::vector<float> rhs(n, 0.0f);
            for (size_t i = 1; i + 1 < n; ++i)
            {
                float secondDiff = refCurvature[i - 1] - 2.0f * refCurvature[i] + refCurvature[i + 1];
                rhs[i] = spacing * spacing * secondDiff - lengthWeight * refCurvature[i];
            }

            constexpr float OMEGA = 1.7f;
            for (int iter = 0; iter < iterations; ++iter)
            {
                for (size_t j = 2; j + 2 < n; ++j)
                {
                    float target = (rhs[j] - offsets[j - 2] + 4.0f * offsets[j - 1] +
                                    4.0f * offsets[j + 1] - offsets[j + 2]) /
                                   6.0f;
                    offsets[j] = std::clamp(offsets[j] + OMEGA * (target - offsets[j]), dMin, dMax);
                }
            }
        }

        float SampleLinear(const std::vector<float> &values, float x)
        {
            if (values.empty())
                return 0.0f;
            float clamped = std::clamp(x, 0.0f, static_cast<float>(values.size() - 1));
            size_t i = static_cast<size_t>(clamped);
            size_t j = std::min(i + 1, values.size() - 1);
            return values[i] + (values[j] - values[i]) * (clamped - static_cast<float>(i));
        }
    }

    std::vector<float> SolveMinCurvatureCascade(const std::vector<float> &refCurvature,
                                                 float spacing, float dMin, float dMax,
                                                 float lengthWeight)
    {
        const size_t n = refCurvature.size();
        if (n < 8)
            return std::vector<float>(n, 0.0f);

        size_t stride = 1;
        while (stride * 64 < n)
            stride *= 2;

        std::vector<float> coarse;
        for (; stride >= 1; stride /= 2)
        {
            const size_t m = (n + stride - 1) / stride;
            std::vector<float> levelCurvature(m);
            for (size_t i = 0; i < m; ++i)
                levelCurvature[i] = refCurvature[std::min(i * stride, n - 1)];

            std::vector<float> offsets(m, 0.0f);
            for (size_t i = 0; i < m; ++i)
                offsets[i] = SampleLinear(coarse, static_cast<float>(i) * 0.5f);

            int iterations = static_cast<int>(std::clamp(200000.0 / static_cast<double>(m),
                                                         300.0, 20000.0));
            SolveMinCurvature(levelCurvature, spacing * static_cast<float>(stride),
                              dMin, dMax, lengthWeight, iterations, offsets);
            coarse.swap(offsets);
            if (stride == 1)
                break;
        }
        coarse.resize(n, coarse.empty() ? 0.0f : coarse.back());
        return coarse;
    }
}
