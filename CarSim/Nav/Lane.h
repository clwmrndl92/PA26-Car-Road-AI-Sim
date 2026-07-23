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

    // 스플라인에서 미리 계산해둔 레인 전체 길이 (라우팅 간선 비용)
    float GetLength() const { return length; }
    // 이 레인의 제한속도 (현재는 소속 도로 제한속도, 추후 레인별 오버라이드 여지)
    float GetLimitSpeed() const;

    // 레인 그래프 위상: 소유는 RoadDataManager가 하므로 여기선 weak_ptr로만 참조해 순환참조 누수를 막는다.
    const vector<weak_ptr<Lane>> &GetSuccessors() const { return m_successors; }
    // 이 레인으로 들어오는 이전 레인들(합류 지점 등에서 여러 개일 수 있음). AddSuccessor와 대칭으로
    // RoadDataManager::BuildSuccessors가 채운다.
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

    // 지금 이 레인 위에 있는 차 목록(Car::SetCurrentLane이 관리). 소유권 없는 raw pointer —
    // 차가 런타임에 파괴되는 경우가 없어(앱 종료 시에만 GameObject 리스트가 비워짐) 수명 문제가 없다.
    // 다른 차 탐색(IIDM 실제 앞차 반영, MOBIL 이웃 탐색)에 쓴다.
    void RegisterCar(Car *car) { m_cars.push_back(car); }
    void UnregisterCar(Car *car) { m_cars.erase(std::remove(m_cars.begin(), m_cars.end(), car), m_cars.end()); }
    const vector<Car *> &GetCars() const { return m_cars; }

    // 끝점 매칭용 헬퍼: 스플라인 시작/끝 지점
    const Vec3 &GetStartPoint() const { return m_spline.GetSplinePoints().front(); }
    const Vec3 &GetEndPoint() const { return m_spline.GetSplinePoints().back(); }

    // 이 레인에 걸린 신호(있으면). BuildRoadData가 1회 세팅.
    void SetSignalNode(const shared_ptr<RoadNode> &node) { m_signalNode = node; }
    shared_ptr<RoadNode> GetSignalNode() const { return m_signalNode.lock(); }

private:
    Spline m_spline; // Spline representing the lane's path
    int m_id;
    shared_ptr<Road> m_road;

    float length = 0.0f;
    vector<weak_ptr<Lane>> m_successors;   // 진행 방향으로 이어지는 다음 레인들
    vector<weak_ptr<Lane>> m_predecessors; // 이 레인으로 이어지는 이전 레인들 (m_successors와 대칭)
    weak_ptr<Lane> m_left;               // 같은 진행방향 좌측 인접 레인 (차선변경)
    weak_ptr<Lane> m_right;              // 같은 진행방향 우측 인접 레인 (차선변경)
    vector<Car *> m_cars;                // 지금 이 레인 위에 있는 차들 (RegisterCar/UnregisterCar가 관리)
    weak_ptr<RoadNode> m_signalNode;     // 소유는 RoadDataManager(m_nodes)가 하므로 weak_ptr로만 참조
};