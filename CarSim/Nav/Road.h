#pragma once
#include <vector>
#include <memory>
#include "Spline.h"

// 링크 대상이 일반 도로인지 교차로인지.
enum class ElementType
{
    Road,
    Junction
};

// 대상 요소의 어느 끝에 붙는지(대상 참조선의 start=s0 / end=s최대).
enum class ContactPoint
{
    Start,
    End
};

// OpenDRIVE <link>의 predecessor/successor 하나.
struct RoadLink
{
    ElementType type = ElementType::Road;
    int elementId = -1;                        // 대상 road id 또는 junction id
    ContactPoint contact = ContactPoint::Start; // 대상의 어느 끝
    bool valid = false;
};

class Road
{
public:
    Road(int id, float speedLimit);
    ~Road() = default;

    int GetId() const { return m_id; }
    float GetSpeedLimit() const { return m_speedLimit; }

    const Spline &GetReferenceLine() const { return m_referenceLine; } // 도로 중앙 참조선(s 기준선)
    void SetReferenceLine(const Spline &line) { m_referenceLine = line; }
    float GetLength() const { return m_referenceLine.GetLength(); }

    int GetJunctionId() const { return m_junctionId; } // -1=일반 도로, 아니면 소속 junction(내부 연결도로)
    void SetJunctionId(int id) { m_junctionId = id; }

    const RoadLink &GetPredecessor() const { return m_predecessor; }
    const RoadLink &GetSuccessor() const { return m_successor; }
    void SetPredecessor(const RoadLink &link) { m_predecessor = link; }
    void SetSuccessor(const RoadLink &link) { m_successor = link; }

private:
    int m_id;
    float m_speedLimit;
    Spline m_referenceLine;
    int m_junctionId = -1;
    RoadLink m_predecessor;
    RoadLink m_successor;
};
