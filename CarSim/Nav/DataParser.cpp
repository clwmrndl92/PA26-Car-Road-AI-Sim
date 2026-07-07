#include "DataParser.h"
#include <fstream>
#include <nlohmann/json.hpp>

using nlohmann::json;

DataParser::DataParser()
{
}

DataParser::~DataParser()
{
}

Spline DataParser::ParseSplineData(const std::string &filePath)
{
    Spline spline;

    std::ifstream file(filePath);
    json root = json::parse(file, nullptr, false);
    if (root.is_discarded())
        return spline;

    const json &geometry = root.value("geometry", json::object());
    spline.SetCycle(geometry.value("cycle", false));

    for (const json &point : geometry.value("control_points", json::array()))
    {
        if (point.size() < 2)
            continue;
        float x = point[0].get<float>();
        float z = point[1].get<float>();
        spline.AddControlPoint(Vec3(x, 0.0f, z));
    }

    return spline;
}
