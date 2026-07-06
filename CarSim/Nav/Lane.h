#include "Spline.h"
class Lane
{
public:
    Lane();
    ~Lane();

private:
    Spline m_Spline; // Spline representing the lane's path
};