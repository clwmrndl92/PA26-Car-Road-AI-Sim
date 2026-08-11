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
    void InitRoadColliders();
    void InitObstacleColliders();
    void InitDynamicObstacleRenders();
    void SpawnCar(CarType type, CarPersonalityType personality, bool roaming);
    std::shared_ptr<Car> SpawnCarAt(const Vec3 &position, const Vec3 &direction, CarType type,
                                    CarPersonalityType personality, bool roaming);
    void SpawnAllCars();
    void SpawnAllNodes();
    void RemoveCar(const std::shared_ptr<Car> &car);
    void UpdateSignalMarkers();
    void SpawnManualCar(CarType type);
    void RemoveManualCar();
    void DrawManualCarWindow();

private:
    static constexpr int kMaxSpawnAllCount = 30;

    RoadDataManager m_RoadDataManager;
    SimulationState m_SimState;
    MarkingDataManager m_MarkingDataManager;

    std::vector<std::shared_ptr<Car>> m_CarObjects;

    CameraMode m_CameraMode = CameraMode::Focus;
    std::string m_PickedObjectName;
    std::weak_ptr<Car> m_pPickedObject;
    int m_carIDCounter = 1;
    int m_SpawnPersonalityIndex = 0;
    bool m_SpawnRoaming = false;

    std::shared_ptr<Car> m_ManualCar;
    int m_ManualCarTypeIndex = 0;

    std::vector<RenderObject> m_RoadRenders;
    std::vector<RenderObject> m_RoadEdgeRenders;
    std::vector<RenderObject> m_MarkingRenders;
    std::vector<RenderObject> m_DynamicObstacleRenders;
    std::vector<RenderObject> m_SignalRenders;

    struct SignalMarker
    {
        Model *model = nullptr;
        shared_ptr<RoadNode> node;
    };
    std::vector<SignalMarker> m_SignalMarkers;
};

#endif
