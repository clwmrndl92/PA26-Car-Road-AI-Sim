#pragma once
#include <string>
#include "Spline.h"

class DataParser
{
public:
    DataParser();
    ~DataParser();

    static Spline ParseSplineData(const std::string &filePath);
};
