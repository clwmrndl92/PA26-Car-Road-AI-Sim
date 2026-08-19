#pragma once

#include <vector>

namespace RacingLineQP
{
    std::vector<float> SolveMinCurvatureCascade(const std::vector<float> &refCurvature,
                                                 float spacing, float dMin, float dMax,
                                                 float lengthWeight);
}
