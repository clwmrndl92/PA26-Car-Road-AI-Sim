#pragma once
#include <string>
#include <vector>
#include "Utill/MathUtil.h"

// Mirrors EditApp's private MarkingLineType/MarkingColor (CarSim/Core/EditApp.h) — kept as a
// separate copy rather than a shared header since the editor and the sim are independent JSON
// consumers by convention (same as RoadDataManager vs EditApp's road/lane/node structs).
enum class MarkingLineType
{
    Solid,
    Dashed
};

enum class MarkingColor
{
    White,
    Yellow,
    Gray
};

// One freehand road-marking line (lane paint, median, shoulder) authored by the EditApp line
// tool and saved independently of data.json (see EditApp::SaveMarkingsToJson).
struct RoadMarking
{
    int id = -1;
    MarkingLineType type = MarkingLineType::Solid;
    float width = 0.15f;
    MarkingColor color = MarkingColor::White;
    float dashLength = 3.0f; // only meaningful when type == Dashed
    float dashGap = 5.0f;    // only meaningful when type == Dashed
    std::vector<Vec3> points;
};

class MarkingDataManager
{
public:
    void Init(const std::string &filePath);

    const std::vector<RoadMarking> &GetMarkings() const { return m_markings; }

private:
    std::vector<RoadMarking> m_markings;
};
