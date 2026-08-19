#pragma once

#include "Utill/MathUtil.h"
#include <vector>

// 트랙 한 바퀴분 최소곡률 레이싱 라인.
//
// QP가 비싸서(7000점 규모에서 수백 ms) 시작할 때 전략별로 한 번만 풀고 모든 차가 공유한다.
// 예전에는 차마다 "앞뒤 도로 몇 칸"을 문맥으로 잡아 다시 풀었는데, 도로 경계를 넘을 때마다
// 수십 ms짜리 solve가 차 수만큼 터졌다.
//
// 배열은 "한 바퀴 + 겹침 구간"을 들고 있고, 겹침 구간의 좌표는 랩 앞부분과 정확히 같게
// 맞춰 둔다. 그래야 결승선을 넘는 순간에도 lookahead/속도 미리보기가 끊기지 않는다.
struct RaceLine
{
    std::vector<Vec3> points; // 월드 좌표
    std::vector<float> arcLength;
    std::vector<float> curvature;
    std::vector<float> laneOffset; // QP가 고른 차선중심 기준 횡오프셋(추월 여유 계산용)

    float lapLength = 0.0f;   // 한 바퀴 호길이. s가 이걸 넘으면 되감는다
    size_t lapPoints = 0;     // 한 바퀴에 해당하는 점 개수(겹침 제외)

    // 도로가 바뀌는 지점의 호길이와 도로 id. 앞 도로 제한속도 미리보기에 쓴다.
    std::vector<float> roadStartS;
    std::vector<int> roadIds;

    bool IsValid() const { return points.size() >= 2 && lapPoints >= 2 && lapLength > 1.0f; }
};
