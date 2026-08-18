#include "VehicleSegment.h"
#include "Car.h"

void SplineFollowSegment::Tick(Car &car)
{
    car.DriveControl();
}
