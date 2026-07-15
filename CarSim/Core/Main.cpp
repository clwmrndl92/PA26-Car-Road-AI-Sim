#include "GameApp.h"
#include "EditApp.h"
#include "CarSim.h"
#include <cstring>

int WINAPI WinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE prevInstance,
                   _In_ LPSTR cmdLine, _In_ int showCmd)
{
    // Unused parameters
    UNREFERENCED_PARAMETER(prevInstance);
    UNREFERENCED_PARAMETER(showCmd);
    // Enable memory allocation tracking and leak detection in Debug builds
#if defined(DEBUG) | defined(_DEBUG)
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    bool editMode = strstr(cmdLine, "--edit") != nullptr;

    if (editMode)
    {
        EditApp theApp(hInstance, L"Edit", 1280, 720);

        if (!theApp.Init())
            return 0;

        return theApp.Run();
    }

    CarSim theApp(hInstance, L"CarSim", 1280, 720);

    if (!theApp.Init())
        return 0;

    return theApp.Run();
}
