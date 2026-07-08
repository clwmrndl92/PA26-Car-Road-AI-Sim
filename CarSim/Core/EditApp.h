#ifndef EDITAPP_H
#define EDITAPP_H

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
#include "Nav/Spline.h"

class EditApp : public D3DApp
{
public:
    EditApp(HINSTANCE hInstance, const std::wstring &windowName, int initWidth, int initHeight);
    ~EditApp();

    bool Init();
    void OnResize();
    void UpdateScene(float dt);
    void DrawScene();

private:
    bool InitResource();
    void UpdateSplineRender(const Spline &spline);
    void UpdateRoadRender(const Spline &spline);

private:
    TextureManager m_TextureManager;
    ModelManager m_ModelManager;

    BasicEffect m_BasicEffect;

    std::unique_ptr<Depth2D> m_pDepthTexture;

    std::vector<std::shared_ptr<GameObject>> m_GameObjects;

    PhysicsSystem m_Physics;

    std::shared_ptr<FreeCamera> m_pCamera;
    float m_TopDownHeightMin = 6.0f;
    float m_TopDownHeightMax = 100.0f;

    RenderObject m_GridXZ;
    RenderObject m_GridXY;
    RenderObject m_GridYZ;
    bool m_ShowGridXZ = false;
    bool m_ShowGridXY = false;
    bool m_ShowGridYZ = false;

    std::vector<RenderObject> m_RoadRenders;

    std::vector<Vec3> m_SplineControlPoints;
    Spline m_Spline;
    RenderObject m_SplineCurve;
    bool m_SplineCurveVisible = false;
    Model *m_pSplineMarkerModel = nullptr;
    std::vector<RenderObject> m_SplineMarkers;
};

#endif
