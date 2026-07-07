#ifndef CARSIM_H
#define CARSIM_H

#include "GameApp.h"
#include "Entities/Car.h"

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
    void UpdateCamera(float dt);
    void UpdateUI(float dt);
    void DrawScene();

private:
    bool InitResource();
    void FocusOnObject(const std::shared_ptr<Car> &obj);
    void InitRoadRenderer();

private:
    RoadDataManager m_RoadDataManager;

    std::vector<std::shared_ptr<Car>> m_CarObjects;

    CameraMode m_CameraMode = CameraMode::Focus;
    std::string m_PickedObjectName;
    std::weak_ptr<Car> m_pPickedObject;

    std::vector<RenderObject> m_RoadRenders;
};

#endif
