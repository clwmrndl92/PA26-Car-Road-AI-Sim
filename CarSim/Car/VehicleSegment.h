#pragma once

class Car;

class VehicleSegment
{
public:
    virtual ~VehicleSegment() = default;
    virtual void Tick(Car &car) = 0;
    virtual bool IsDone() const = 0;
};

class SplineFollowSegment : public VehicleSegment
{
public:
    void Tick(Car &car) override;
    bool IsDone() const override { return false; }
};
