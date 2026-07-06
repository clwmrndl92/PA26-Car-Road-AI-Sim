#include "DataParser.h"
#include <fstream>
#include <sstream>
#include <regex>

DataParser::DataParser()
{
}

DataParser::~DataParser()
{
}

namespace
{
    // Returns the substring spanning the first '[' found after `key` through its matching ']'.
    std::string ExtractArray(const std::string &json, const std::string &key)
    {
        size_t keyPos = json.find(key);
        if (keyPos == std::string::npos)
            return {};

        size_t start = json.find('[', keyPos);
        if (start == std::string::npos)
            return {};

        int depth = 0;
        for (size_t i = start; i < json.size(); ++i)
        {
            if (json[i] == '[')
                ++depth;
            else if (json[i] == ']')
            {
                if (--depth == 0)
                    return json.substr(start, i - start + 1);
            }
        }
        return {};
    }
}

Spline DataParser::ParseSplineData(const std::string &filePath)
{
    Spline spline;

    std::ifstream file(filePath);
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string json = buffer.str();

    static const std::regex cycleRegex(R"("cycle"\s*:\s*(true|false))");
    std::smatch cycleMatch;
    if (std::regex_search(json, cycleMatch, cycleRegex))
        spline.SetCycle(cycleMatch[1] == "true");

    // control_points entries are [x, y] pairs on the ground plane; y maps to Vec3's Z
    // axis, matching the XZ-plane convention used elsewhere (see EditApp::UpdateScene).
    std::string controlPoints = ExtractArray(json, "\"control_points\"");
    static const std::regex pointRegex(R"(\[\s*(-?[0-9]*\.?[0-9]+)\s*,\s*(-?[0-9]*\.?[0-9]+)\s*\])");
    for (auto it = std::sregex_iterator(controlPoints.begin(), controlPoints.end(), pointRegex);
         it != std::sregex_iterator(); ++it)
    {
        float x = std::stof((*it)[1]);
        float z = std::stof((*it)[2]);
        spline.AddControlPoint(Vec3(x, 0.0f, z));
    }

    return spline;
}
