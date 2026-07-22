#ifndef CARSIM_H
#define CARSIM_H

#include "Core/GameApp.h"
#include "Car/Car.h"
#include "Nav/MarkingDataManager.h"

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
    void SpawnCar(CarType type);
    void RemoveCar(const std::shared_ptr<Car> &car);
    void UpdateSignalMarkers();

private:
    RoadDataManager m_RoadDataManager;
    MarkingDataManager m_MarkingDataManager;

    std::vector<std::shared_ptr<Car>> m_CarObjects;

    CameraMode m_CameraMode = CameraMode::Focus;
    std::string m_PickedObjectName;
    std::weak_ptr<Car> m_pPickedObject;
    int m_carIDCounter = 1;

    std::vector<RenderObject> m_RoadRenders;
    std::vector<RenderObject> m_RoadEdgeRenders;
    std::vector<RenderObject> m_MarkingRenders;
    std::vector<RenderObject> m_ObstacleRenders; // data.json의 obstacles를 파란 사각형 외곽선으로 시각화
    std::vector<RenderObject> m_SignalRenders;   // traffic_light 노드 위치의 채워진 원 마커 (m_SignalMarkers와 같은 순서)

    struct SignalMarker
    {
        Model *model = nullptr;
        float phaseOffset = 0.0f;
    };
    std::vector<SignalMarker> m_SignalMarkers;
};

#endif
