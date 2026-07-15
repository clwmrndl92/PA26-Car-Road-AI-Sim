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
#include <string>
#include "Core/Physics/PhysicsSystem.h"
#include "GameObject.h"
#include <TextureManager.h>

class GameApp : public D3DApp
{
public:
    GameApp(HINSTANCE hInstance, const std::wstring &windowName, int initWidth, int initHeight);
    ~GameApp();

    bool Init();
    virtual void InitCamera();
    void OnResize();
    virtual void UpdateScene(float dt);
    virtual void UpdateCamera(float dt);
    virtual void UpdateUI(float dt);
    void DrawScene();

protected:
    BasicEffect m_BasicEffect;
    TextureManager m_TextureManager;
    ModelManager m_ModelManager;

    std::unique_ptr<Depth2D> m_pDepthTexture;

    std::vector<std::shared_ptr<GameObject>> m_GameObjects;

    PhysicsSystem m_Physics;

    std::shared_ptr<Camera> m_pCamera;

    RenderObject m_GridXZ;
    RenderObject m_GridXY;
    RenderObject m_GridYZ;
    bool m_ShowGridXZ = false;
    bool m_ShowGridXY = false;
    bool m_ShowGridYZ = false;

private:
    bool InitResource();
    void InitLight();
};

#endif
