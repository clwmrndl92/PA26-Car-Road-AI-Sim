#ifndef CARSIM_H
#define CARSIM_H

#include "Core/GameApp.h"
#include "Car/Car.h"
#include "Nav/MarkingDataManager.h"
#include "Nav/SimulationState.h"

class CarSim : public GameApp
{
public:
    enum class CameraMode
    {
        Focus,
        Free
    };

    CarSim(HINSTANCE hInstance, const std::wstring &windowName, int initWidth, int initHeight);
    ~CarSim();

    bool Init();
    void InitCamera();
    void UpdateScene(float dt);
    void UpdateCamera(float dt);
    void UpdateUI(float dt);
    void DrawScene();

private:
    bool InitResource();
    void FocusOnObject(const std::shared_ptr<Car> &obj);
    void InitRoadRenderer();
    void InitMarkingRenderer();
    void InitRoadColliders();     // 고가(높이>0) 레인을 따라 lane 폭만큼 static 도로 GameObject 생성
    void InitObstacleColliders(); // data.json의 obstacle 위치/크기/회전으로 static 큐브(차량과 충돌) 생성
    void InitDynamicObstacleRenders(); // dynamic_obstacles 개수만큼 로컬박스 렌더 생성(위치는 매프레임 UpdateScene이 갱신)
    void SpawnCar(CarType type, CarPersonalityType personality);
    void RemoveCar(const std::shared_ptr<Car> &car);
    void UpdateSignalMarkers();

private:
    RoadDataManager m_RoadDataManager;
    SimulationState m_SimState;
    MarkingDataManager m_MarkingDataManager;

    std::vector<std::shared_ptr<Car>> m_CarObjects;

    CameraMode m_CameraMode = CameraMode::Focus;
    std::string m_PickedObjectName;
    std::weak_ptr<Car> m_pPickedObject;
    int m_carIDCounter = 1;
    int m_SpawnPersonalityIndex = 0;

    std::vector<RenderObject> m_RoadRenders;
    std::vector<RenderObject> m_RoadEdgeRenders;
    std::vector<RenderObject> m_MarkingRenders;
    std::vector<RenderObject> m_ObstacleRenders; // data.json의 obstacles를 파란 사각형 외곽선으로 시각화
    std::vector<RenderObject> m_DynamicObstacleRenders; // dynamic_obstacles를 주황 큐브로 시각화 (RoadDataManager::GetDynamicObstacles()와 같은 순서)
    std::vector<RenderObject> m_SignalRenders;   // traffic_light 노드 위치의 채워진 원 마커 (m_SignalMarkers와 같은 순서)

    struct SignalMarker
    {
        Model *model = nullptr;
        float phaseOffset = 0.0f;
    };
    std::vector<SignalMarker> m_SignalMarkers;
};

#endif
