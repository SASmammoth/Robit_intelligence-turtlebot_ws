#ifndef tb_gui_WORLD_DATA_HPP_
#define tb_gui_WORLD_DATA_HPP_

// /lane/world (tb_interfaces/PathWorld)를 Qt 쪽으로 옮긴 것. 좌표는 로봇 기준 [m] (x 앞, y 왼쪽)
#include <QMetaType>
#include <QPointF>
#include <QPolygonF>
#include <QVector>

struct WorldData
{
  QVector<QPointF> white, yellow, red, ext; // 선 (줄마다 가운데 점)
  QVector<QPolygonF> bricks;                // 이번 프레임 벽돌 벽
  QVector<QPolygonF> held;                  // 기억 중인 지난 벽돌
  QVector<QPolygonF> bridges;               // 같은 쪽 벽돌 잇기
  QVector<QPolygonF> keep;                  // 경로를 떼어 놓은 벽돌 (con_keep)
  QPolygonF memory, trail;                  // 기억 경로, 지나온 경로
  double stop_x = -1.0, stop_cut_x = -1.0;
  bool construction = false;
  int turn = 0;
};
Q_DECLARE_METATYPE(WorldData)

#endif // tb_gui_WORLD_DATA_HPP_
