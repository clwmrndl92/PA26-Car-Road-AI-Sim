cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 17)
add_compile_definitions(UNICODE _UNICODE NOMINMAX)

add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

# CarSim 전체(렌더링/ImGui/에셋 로딩)를 띄우지 않고, Nav 알고리즘 소스만 직접 컴파일해서
# 붙인 콘솔 실행 파일. VS에서 이 프로젝트를 시작 프로젝트로 잡고 프로파일러를 돌리면 된다.
add_executable(Tests
    TestHarness.cpp
    HybridAStarTests.cpp
    DebugConsoleStub.cpp
    ${CMAKE_SOURCE_DIR}/CarSim/Nav/HybridAStar.cpp
    ${CMAKE_SOURCE_DIR}/CarSim/Nav/ReedsShepp.cpp
)
target_include_directories(Tests PRIVATE ${CMAKE_SOURCE_DIR}/CarSim)

# Jolt Physics: Utill/MathUtil.h의 Vec3(JPH::Vec3) 때문에 필요.
target_link_libraries(Tests Jolt)
target_include_directories(Tests PRIVATE ${joltphysics_SOURCE_DIR})

set_target_properties(Tests PROPERTIES FOLDER "Tests")
