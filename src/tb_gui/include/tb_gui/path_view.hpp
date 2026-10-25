#ifndef tb_gui_PATH_VIEW_HPP_
#define tb_gui_PATH_VIEW_HPP_

// /lane/path를 로봇 좌표(base_link, m) 그대로 그리는 위젯
//  - 경로: /lane/path_source 색 (live 초록, memory 보라, color_rule 주황), 0.4초 넘게 안 오면 회색 점선
//  - 지난 경로 몇 개를 흐리게 겹쳐 그림 → 경로가 갑자기 선 밖으로 튀는 순간이 보임
//  - /drive/target 목표점 + lookahead 원, 로봇 외곽, 10cm 격자
//  - /drive/state에서 HOLD, stop 이유를 뽑아 배지로 표시
//  - 호 두 개: 목표점까지의 pure pursuit 호(빨강 점선) / drive가 실제로 낸 v, w의 호(하늘색)
//  - 좌우 바퀴 속도 화살표: v, w로 구한 바퀴 선속도 [m/s] + TB_Uart_RX로 실제 보낸 원시값
//  - BEV 범위(점선 네모)와 카메라가 실제로 보는 영역(옅은 면): GUI가 읽은 bev.*, geo.* 파라미터로 계산
//  계산은 전부 "그리기용"이고, 로봇 제어에는 쓰이지 않음 (값은 drive_node가 낸 것)

#include <QWidget>
#include <QPolygonF>
#include <QPointF>
#include <QString>
#include <QTimer>
#include <QElapsedTimer>
#include <deque>

class PathView : public QWidget
{
  Q_OBJECT
public:
  explicit PathView(QWidget *parent = nullptr);

  void setPath(const QPolygonF &pts);       // base_link [m], 가까운 점부터
  void setSource(const QString &src);       // /lane/path_source
  void setTarget(const QPointF &p);         // /drive/target
  void setDriveState(const QString &text);  // /drive/state
  void setUartCommand(const QString &text); // TB_Uart_RX ("velocity L R"만 씀)
  void setIntParam(const QString &name, int v);       // bev.*, geo.* (/lineDetect_node)
  void setDoubleParam(const QString &name, double v); // drive.uart.wheel_sep_m, pivot_sep_m (/drive_node)

protected:
  void paintEvent(QPaintEvent *) override;

private:
  QPointF toPx(const QPointF &m) const;     // 로봇 좌표 [m] → 위젯 픽셀
  void updateScale();
  void drawArc(QPainter &p, double kappa, double s0, double s1, const QPen &pen) const; // 원점에서 x축에 접하는 호
  void drawWheels(QPainter &p, qint64 now);
  void drawBev(QPainter &p);
  void rebuildBev();                          // 파라미터가 바뀌면 BEV 네모·보이는 영역 다시 계산
  double wheelSep(double v, double w) const;  // drive_node::publish_uart와 같은 바퀴 간격 섞기

  struct Old
  {
    QPolygonF pts;
    QString src;
  };

  QPolygonF path_;
  QString src_ = "-";
  QString path_src_at_recv_ = "-";          // 이 경로를 받을 때의 source
  std::deque<Old> history_;                 // 지난 경로 (최근 것이 뒤)
  QPointF target_;
  QString drive_state_;

  QElapsedTimer clock_;
  qint64 path_ms_ = -1, target_ms_ = -1, state_ms_ = -1;
  double hz_ = 0.0;                         // /lane/path 수신 주기 (지수평균)

  // drive 출력
  double v_ = 0.0, w_ = 0.0;                // /drive/state의 v [m/s], w [rad/s] (가감속 제한·피봇 후 실제 출력)
  qint64 vw_ms_ = -1;
  long raw_l_ = 0, raw_r_ = 0;              // TB_Uart_RX "velocity L R" (다이나믹셀 원시값)
  qint64 raw_ms_ = -1;

  // 바퀴 간격 [m] (drive.uart.*, 받기 전에는 drive.yaml 값)
  double wheel_sep_ = 0.193, pivot_sep_ = 0.165;

  // BEV (lineDetect_node 파라미터, 받기 전에는 노드 기본값)
  int bev_top_y_ = 40, bev_bot_y_ = 100, bev_top_w_ = 32, bev_bot_w_ = 100; // 원본 영상 사다리꼴 [%]
  int lane_w_px_ = 160, lane_w_mm_ = 260, bev_depth_mm_ = 520, bev_bottom_mm_ = 150;
  QPolygonF bev_rect_;    // BEV 네모 (로봇 좌표)
  QPolygonF bev_visible_; // 그중 카메라 영상이 닿는 영역 (나머지 = 사각지대)

  // 보이는 범위 [m]: x = 앞, y = 왼쪽
  double x_min_ = -0.20, x_max_ = 0.60, y_half_ = 0.35;
  double scale_ = 1.0;                      // px / m
  QPointF origin_;                          // 로봇 원점의 픽셀 위치

  QTimer repaint_timer_;
};

#endif // tb_gui_PATH_VIEW_HPP_
