#pragma once
#include <algorithm>
#include <memory>
#include <vector>
#include "Spline.h"

using namespace std;

class Road;
class Car;
struct RoadNode;

class Lane
{
public:
    Lane(int id, const Spline &spline, const shared_ptr<Road> &road);
    ~Lane();

    int GetId() const { return m_id; }
    const Spline &GetSpline() const { return m_spline; }
    const shared_ptr<Road> &GetRoad() const { return m_road; }

    float GetLength() const { return length; }
    float GetLimitSpeed() const;

    const vector<weak_ptr<Lane>> &GetSuccessors() const { return m_successors; }
    const vector<weak_ptr<Lane>> &GetPredecessors() const { return m_predecessors; }
    weak_ptr<Lane> GetLeft() const { return m_left; }
    weak_ptr<Lane> GetRight() const { return m_right; }

    void AddSuccessor(const shared_ptr<Lane> &lane) { m_successors.push_back(lane); }
    void AddPredecessor(const shared_ptr<Lane> &lane) { m_predecessors.push_back(lane); }
    void SetLeft(const shared_ptr<Lane> &lane) { m_left = lane; }
    void SetRight(const shared_ptr<Lane> &lane) { m_right = lane; }
    void ClearConnections()
    {
        m_successors.clear();
        m_predecessors.clear();
        m_left.reset();
        m_right.reset();
    }

    void RegisterCar(Car *car) { m_cars.push_back(car); }
    void UnregisterCar(Car *car) { m_cars.erase(std::remove(m_cars.begin(), m_cars.end(), car), m_cars.end()); }
    const vector<Car *> &GetCars() const { return m_cars; } // 지금 이 레인 위에 있는 차 목록

    const Vec3 &GetStartPoint() const { return m_spline.GetSplinePoints().front(); }
    const Vec3 &GetEndPoint() const { return m_spline.GetSplinePoints().back(); }

    void SetSignalNode(const shared_ptr<RoadNode> &node) { m_signalNode = node; }
    shared_ptr<RoadNode> GetSignalNode() const { return m_signalNode.lock(); }

private:
    Spline m_spline;
    int m_id;
    shared_ptr<Road> m_road;

    float length = 0.0f;
    vector<weak_ptr<Lane>> m_successors;
    vector<weak_ptr<Lane>> m_predecessors;
    weak_ptr<Lane> m_left;  // 같은 진행방향 좌측 인접 레인
    weak_ptr<Lane> m_right; // 같은 진행방향 우측 인접 레인
    vector<Car *> m_cars;   // 지금 이 레인 위에 있는 차들
    weak_ptr<RoadNode> m_signalNode;
};