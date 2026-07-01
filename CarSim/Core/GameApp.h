#ifndef GAMEAPP_H
#define GAMEAPP_H

#include <WinMin.h>
#include "d3dApp.h"
#include "Rendering/Effects.h"
#include <Camera.h>
#include <Collision.h>
#include <RenderStates.h>
#include <GameObject.h>
#include <Texture2D.h>
#include <Buffer.h>
#include <ModelManager.h>
#include <TextureManager.h>
#include <string>
#include "PhysicsWorld.h"

class GameApp : public D3DApp
{
public:
    enum class CameraMode { ThirdPerson, Free };

public:
    GameApp(HINSTANCE hInstance, const std::wstring& windowName, int initWidth, int initHeight);
    ~GameApp();

    bool Init();
    void OnResize();
    void UpdateScene(float dt);
    void DrawScene();

private:
    bool InitResource();
private:

    TextureManager m_TextureManager;
    ModelManager m_ModelManager;

    BasicEffect m_BasicEffect;                                  // Rendering effect manager

    std::unique_ptr<Depth2D> m_pDepthTexture;                   // Depth buffer

    GameObject m_Road;

    static constexpr int CAR_COUNT = 7;
    GameObject m_Cars[CAR_COUNT];

    static constexpr int ROAD_DASH_COUNT = 16;
    GameObject m_RoadDashes[ROAD_DASH_COUNT];

    PhysicsWorld m_Physics;

    std::shared_ptr<Camera> m_pCamera;							// Active camera
    CameraMode m_CameraMode = CameraMode::ThirdPerson;
    std::string m_PickedObjectName;								// Currently picked object
};


#endif