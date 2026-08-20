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
    void RemoveAllCars();
    void UpdateSignalMarkers();
    void SpawnManualCar(CarType type);
    void RemoveManualCar();
    void DrawManualCarWindow();

    void DrawRaceWindow();
    bool EnsureRaceLines();           // 아직 없으면 공유 라인을 푼다(수백 ms, 1회)
    void SetRaceModeForAll(bool on);  // 등록된 모든 차의 레이스 모드 토글
    void SpawnRaceGrid(int carCount); // 레이싱 라인 위에 그리드로 세운다

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

    bool m_RaceMode = false;
    int m_RaceGridCount = 10;
    RenderObject m_RaceLineRender; // 차 선택과 무관하게 항상 보이는 기준 라인

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
