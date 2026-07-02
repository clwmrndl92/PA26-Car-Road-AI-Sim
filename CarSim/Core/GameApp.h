#ifndef GAMEAPP_H
#define GAMEAPP_H

#include <WinMin.h>
#include "d3dApp.h"
#include "Rendering/Effects.h"
#include <Camera.h>
#include <Collision.h>
#include <RenderStates.h>
#include <Texture2D.h>
#include <Buffer.h>
#include <ModelManager.h>
#include <TextureManager.h>
#include <string>
#include "PhysicsSystem.h"
#include "GameObject.h"

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
    ModelManager   m_ModelManager;

    BasicEffect m_BasicEffect;

    std::unique_ptr<Depth2D> m_pDepthTexture;

    std::vector<std::shared_ptr<GameObject>> m_GameObjects;

    PhysicsSystem m_Physics;

    std::shared_ptr<Camera>     m_pCamera;
    CameraMode                  m_CameraMode = CameraMode::ThirdPerson;
    std::string                 m_PickedObjectName;
    std::weak_ptr<GameObject>   m_pPickedObject;

    RenderObject m_GridXZ;
    RenderObject m_GridXY;
    RenderObject m_GridYZ;
    bool m_ShowGridXZ = true;
    bool m_ShowGridXY = false;
    bool m_ShowGridYZ = false;
};

#endif
