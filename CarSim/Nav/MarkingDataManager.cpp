#include "MarkingDataManager.h"
#include <fstream>
#include <nlohmann/json.hpp>

void MarkingDataManager::Init(const std::string &filePath)
{
    m_markings.clear();

    std::ifstream file(filePath);
    nlohmann::json root = nlohmann::json::parse(file, nullptr, false);
    if (root.is_discarded())
        return;

    for (const nlohmann::json &markingJson : root.value("markings", nlohmann::json::array()))
    {
        RoadMarking marking;
        marking.id = markingJson.value("id", -1);
        marking.type = (markingJson.value("type", std::string("solid")) == "dashed") ? MarkingLineType::Dashed : MarkingLineType::Solid;
        marking.width = markingJson.value("width", 0.15f);

        std::string colorStr = markingJson.value("color", std::string("white"));
        marking.color = (colorStr == "yellow") ? MarkingColor::Yellow : (colorStr == "gray") ? MarkingColor::Gray : MarkingColor::White;

        marking.dashLength = markingJson.value("dash_length", 3.0f);
        marking.dashGap = markingJson.value("dash_gap", 5.0f);

        for (const nlohmann::json &point : markingJson.value("points", nlohmann::json::array()))
        {
            if (point.size() < 3)
                continue;
            marking.points.push_back(Vec3(point[0].get<float>(), point[1].get<float>(), point[2].get<float>()));
        }

        if (marking.points.size() >= 2)
            m_markings.push_back(std::move(marking));
    }
}
