#pragma once
#include <memory>
#include <vector>
#include "Spline.h"

using namespace std;

class Road;

class Lane
{
public:
    enum class LaneType
    {
        Straight,
        Curve
    };

    Lane(int id, const Spline &spline, const shared_ptr<Road> &road);
    ~Lane();

    int GetId() const { return m_id; }
    const Spline &GetSpline() const { return m_spline; }
    const shared_ptr<Road> &GetRoad() const { return m_road; }
    LaneType GetLaneType() const { return m_laneType; }
    bool IsStraight() const { return m_laneType == LaneType::Straight; }

    // 스플라인에서 미리 계산해둔 레인 전체 길이 (라우팅 간선 비용)
    float GetLength() const { return m_length; }
    // 이 레인의 제한속도 (현재는 소속 도로 제한속도, 추후 레인별 오버라이드 여지)
    float GetLimitSpeed() const;

    // 레인 그래프 위상: 소유는 RoadDataManager가 하므로 여기선 weak_ptr로만 참조해 순환참조 누수를 막는다.
    const vector<weak_ptr<Lane>> &GetSuccessors() const { return m_successors; }
    weak_ptr<Lane> GetLeft() const { return m_left; }
    weak_ptr<Lane> GetRight() const { return m_right; }

    void AddSuccessor(const shared_ptr<Lane> &lane) { m_successors.push_back(lane); }
    void SetLeft(const shared_ptr<Lane> &lane) { m_left = lane; }
    void SetRight(const shared_ptr<Lane> &lane) { m_right = lane; }
    void ClearConnections()
    {
        m_successors.clear();
        m_left.reset();
        m_right.reset();
    }

    // 끝점 매칭용 헬퍼: 스플라인 시작/끝 지점
    const Vec3 &GetStartPoint() const { return m_spline.GetSplinePoints().front(); }
    const Vec3 &GetEndPoint() const { return m_spline.GetSplinePoints().back(); }

private:
    Spline m_spline; // Spline representing the lane's path
    int m_id;
    shared_ptr<Road> m_road;
    LaneType m_laneType;

    float m_length = 0.0f;
    vector<weak_ptr<Lane>> m_successors; // 진행 방향으로 이어지는 다음 레인들
    weak_ptr<Lane> m_left;               // 같은 진행방향 좌측 인접 레인 (차선변경)
    weak_ptr<Lane> m_right;              // 같은 진행방향 우측 인접 레인 (차선변경)
};