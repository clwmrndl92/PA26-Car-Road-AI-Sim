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
    // roaming=true면 목적지 없이 배회, false면 랜덤 목적지 노드를 잡아 그리로 주행.
    void SpawnCar(CarType type, CarPersonalityType personality, bool roaming);
    // SpawnCar/SpawnAllCars가 공유하는 실제 생성 로직.
    std::shared_ptr<Car> SpawnCarAt(const Vec3 &position, const Vec3 &direction, CarType type,
                                    CarPersonalityType personality, bool roaming);
    // "Spawn All" 버튼: 차종/성격을 지정된 비율로 무작위로 골라 맵 위 무작위 위치(x,z in [-150,150])에
    // kMaxSpawnAllCount대를 배회(roaming) 모드로 소환한다.
    void SpawnAllCars();
    void RemoveCar(const std::shared_ptr<Car> &car);
    void UpdateSignalMarkers();
    // 사용자 조작 차: 기존 것이 있으면 그 자리(위치/방향)에 새 차종으로 갈아끼우고 포커싱한다.
    void SpawnManualCar(CarType type);
    void RemoveManualCar();
    void DrawManualCarWindow();

private:
    static constexpr int kMaxSpawnAllCount = 30; // "Spawn All" 버튼이 소환하는 대수

    RoadDataManager m_RoadDataManager;
    SimulationState m_SimState;
    MarkingDataManager m_MarkingDataManager;

    std::vector<std::shared_ptr<Car>> m_CarObjects;

    CameraMode m_CameraMode = CameraMode::Focus;
    std::string m_PickedObjectName;
    std::weak_ptr<Car> m_pPickedObject;
    int m_carIDCounter = 1;
    int m_SpawnPersonalityIndex = 0;
    bool m_SpawnRoaming = false; // "Spawn Car" 창 체크박스: 체크 시 배회, 해제 시 랜덤 목적지 주행

    std::shared_ptr<Car> m_ManualCar;   // 사용자 조작 차 (없으면 nullptr)
    int m_ManualCarTypeIndex = 0;       // "User Car" 창 드롭다운 선택 인덱스

    std::vector<RenderObject> m_RoadRenders;
    std::vector<RenderObject> m_RoadEdgeRenders;
    std::vector<RenderObject> m_MarkingRenders;
    std::vector<RenderObject> m_DynamicObstacleRenders; // dynamic_obstacles를 주황 큐브로 시각화 (RoadDataManager::GetDynamicObstacles()와 같은 순서)
    std::vector<RenderObject> m_SignalRenders;   // traffic_light 노드 위치의 채워진 원 마커 (m_SignalMarkers와 같은 순서)

    struct SignalMarker
    {
        Model *model = nullptr;
        shared_ptr<RoadNode> node; // phaseOffset/green/yellow/red 지속시간을 노드에서 그대로 읽는다
    };
    std::vector<SignalMarker> m_SignalMarkers;
};

#endif
