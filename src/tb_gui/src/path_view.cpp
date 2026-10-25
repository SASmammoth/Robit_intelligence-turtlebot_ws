#include "../include/tb_gui/path_view.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QRegularExpression>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace
{
constexpr int HISTORY = 6;          // 흐리게 겹쳐 그릴 지난 경로 수
constexpr qint64 STALE_MS = 400;    // drive.path_timeout_ms 와 같게
constexpr qint64 TARGET_MS = 300;   // 이보다 오래된 목표점은 안 그림
constexpr qint64 DRIVE_MS = 500;    // 이보다 오래된 v, w·UART 값은 안 그림
constexpr int BEV_OUT = 320;        // lineDetect_node bev_out_ (BEV 한 변 px)
constexpr int CAM_W = 640, CAM_H = 480; // tb_camera 해상도
constexpr double ARROW_GAIN = 0.5;  // 바퀴 화살표 길이 [m] = 속도 [m/s] × 이 값 (0.3 m/s → 15cm)
const QColor CMD_COLOR(90, 200, 255); // 실제 출력(v, w) 색

QColor sourceColor(const QString &s)
{
  if (s == "live") return QColor(80, 220, 120);
  if (s == "memory") return QColor(175, 140, 255);
  if (s == "color_rule") return QColor(255, 160, 60);
  return QColor(150, 150, 150);
}
} // namespace

PathView::PathView(QWidget *parent) : QWidget(parent)
{
  setMinimumSize(220, 300);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  clock_.start();
  // 데이터가 안 와도 "오래됨" 표시가 바뀌도록 10Hz로 다시 그림
  repaint_timer_.setInterval(100);
  connect(&repaint_timer_, &QTimer::timeout, this, [this] { update(); });
  repaint_timer_.start();
  rebuildBev();
}

void PathView::setPath(const QPolygonF &pts)
{
  const qint64 t = clock_.elapsed();
  if (path_ms_ >= 0)
  {
    const double dt = (t - path_ms_) / 1000.0;
    if (dt > 1e-3)
      hz_ = hz_ <= 0.0 ? 1.0 / dt : 0.8 * hz_ + 0.2 / dt;
  }
  if (!path_.isEmpty())
  {
    history_.push_back({path_, path_src_at_recv_});
    while (history_.size() > HISTORY)
      history_.pop_front();
  }
  path_ = pts;
  path_src_at_recv_ = src_;
  path_ms_ = t;
}

void PathView::setSource(const QString &src)
{
  src_ = src;
  // path_node는 /lane/path 직후 /lane/path_source를 보내므로, 방금 받은 경로의 출처를 고쳐 줌
  if (path_ms_ >= 0 && clock_.elapsed() - path_ms_ < 50)
    path_src_at_recv_ = src;
}

void PathView::setTarget(const QPointF &p)
{
  target_ = p;
  target_ms_ = clock_.elapsed();
}

void PathView::setDriveState(const QString &text)
{
  drive_state_ = text;
  state_ms_ = clock_.elapsed();

  // "FOLLOW src=live v=0.123 w=0.45 ld=..." 에서 v, w
  static const QRegularExpression vw_re("\\bv=(-?[0-9.]+)\\s+w=(-?[0-9.]+)");
  const auto m = vw_re.match(text);
  if (m.hasMatch())
  {
    v_ = m.captured(1).toDouble();
    w_ = m.captured(2).toDouble();
    vw_ms_ = state_ms_;
  }
}

void PathView::setUartCommand(const QString &text)
{
  // "velocity L R" 만 (start, quit, psd ... 는 무시)
  const QStringList t = text.split(' ', Qt::SkipEmptyParts);
  if (t.size() != 3 || t[0] != "velocity")
    return;
  bool ok_l = false, ok_r = false;
  const long l = t[1].toLong(&ok_l), r = t[2].toLong(&ok_r);
  if (!ok_l || !ok_r)
    return;
  raw_l_ = l;
  raw_r_ = r;
  raw_ms_ = clock_.elapsed();
}

// 원점에서 x축(앞)에 접하고 곡률 kappa인 호를, 호 길이 s0 → s1 구간만 그림 (s < 0 이면 후진 쪽)
void PathView::drawArc(QPainter &p, double kappa, double s0, double s1, const QPen &pen) const
{
  QPolygonF px;
  const int n = 40;
  for (int i = 0; i <= n; ++i)
  {
    const double s = s0 + (s1 - s0) * i / n;
    if (std::abs(kappa) < 1e-6)
      px << toPx({s, 0.0});
    else
      px << toPx({std::sin(kappa * s) / kappa, (1.0 - std::cos(kappa * s)) / kappa});
  }
  p.setBrush(Qt::NoBrush);
  p.setPen(pen);
  p.drawPolyline(px);
}

// 좌우 바퀴 속도 화살표 + 글자. 바퀴 선속도는 drive와 같은 식 l = v − w·sep/2, r = v + w·sep/2
void PathView::drawWheels(QPainter &p, qint64 now)
{
  const bool vw_ok = vw_ms_ >= 0 && now - vw_ms_ < DRIVE_MS;
  const bool raw_ok = raw_ms_ >= 0 && now - raw_ms_ < DRIVE_MS;
  const double half = wheelSep(v_, w_) / 2.0;   // 바퀴 선속도 계산용 (drive와 같은 간격)
  const double draw_half = wheel_sep_ / 2.0;      // 바퀴 그림 위치 (실제 바퀴 간격)

  struct Wheel
  {
    const char *name;
    double y;       // 바퀴 위치 (왼쪽 +)
    double speed;   // [m/s]
    long raw;
    bool left;
  };
  const Wheel wheels[2] = {{"L", draw_half, v_ - w_ * half, raw_l_, true},
                           {"R", -draw_half, v_ + w_ * half, raw_r_, false}};

  QFont f = font();
  f.setPointSize(9);
  p.setFont(f);
  const QFontMetrics fm(f);

  for (const auto &wh : wheels)
  {
    // 바퀴 (지름 9.5cm, 폭 2cm)
    p.setPen(QPen(QColor(200, 200, 200), 1));
    p.setBrush(QColor(90, 90, 90));
    p.drawRect(QRectF(toPx({0.0475, wh.y + 0.01}), toPx({-0.0475, wh.y - 0.01})));

    // 화살표: 앞으로 굴러가면 위, 뒤로면 아래
    if (vw_ok && std::abs(wh.speed) > 1e-3)
    {
      const QPointF a = toPx({0.0, wh.y});
      const QPointF b = toPx({wh.speed * ARROW_GAIN, wh.y});
      const QColor c = wh.speed >= 0 ? CMD_COLOR : QColor(255, 140, 90);
      p.setPen(QPen(c, 3, Qt::SolidLine, Qt::RoundCap));
      p.drawLine(a, b);
      const double dir = b.y() < a.y() ? -1.0 : 1.0; // 화면에서 위(-) / 아래(+)
      p.setPen(Qt::NoPen);
      p.setBrush(c);
      p.drawPolygon(QPolygonF({b + QPointF(0, dir * 9), b + QPointF(-6, 0), b + QPointF(6, 0)}));
    }

    // 글자: 왼쪽 바퀴는 바퀴 왼쪽에, 오른쪽 바퀴는 오른쪽에
    const QString l1 = vw_ok ? QString("%1 %2 m/s").arg(wh.name).arg(wh.speed, 0, 'f', 3) : QString("%1 -").arg(wh.name);
    const QString l2 = raw_ok ? QString("raw %1").arg(wh.raw) : QString("raw -");
    const QPointF anchor = toPx({0.0, wh.y + (wh.left ? 0.02 : -0.02)});
    const double tw = std::max(fm.horizontalAdvance(l1), fm.horizontalAdvance(l2)) + 4;
    const double tx = wh.left ? anchor.x() - tw - 4 : anchor.x() + 4;
    p.setPen(vw_ok ? QColor(235, 235, 235) : QColor(130, 130, 130));
    p.drawText(QRectF(tx, anchor.y() - fm.height(), tw, fm.height()), wh.left ? Qt::AlignRight : Qt::AlignLeft, l1);
    p.setPen(raw_ok ? QColor(170, 170, 170) : QColor(110, 110, 110));
    p.drawText(QRectF(tx, anchor.y(), tw, fm.height()), wh.left ? Qt::AlignRight : Qt::AlignLeft, l2);
  }

  // 제자리 회전(피봇): 호 대신 원점 둘레 회전 화살표
  if (vw_ok && std::abs(v_) < 1e-3 && std::abs(w_) > 0.05)
  {
    const double r = 0.05 * scale_;
    const QPointF c = toPx({0, 0});
    const int start = 90 * 16, span = (w_ > 0 ? 1 : -1) * 270 * 16; // Qt: 반시계 +, 왼쪽 회전 = 반시계
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(CMD_COLOR, 2));
    p.drawArc(QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r), start, span);
    const double end = (90.0 + (w_ > 0 ? 270.0 : -270.0)) * M_PI / 180.0;
    const QPointF tip(c.x() + r * std::cos(end), c.y() - r * std::sin(end));
    const QPointF tang(-std::sin(end) * (w_ > 0 ? 1 : -1), -std::cos(end) * (w_ > 0 ? 1 : -1)); // 진행 방향 (화면 좌표)
    const QPointF nrm(-tang.y(), tang.x());
    p.setPen(Qt::NoPen);
    p.setBrush(CMD_COLOR);
    p.drawPolygon(QPolygonF({tip + tang * 8, tip + nrm * 5, tip - nrm * 5}));
  }
}

void PathView::updateScale()
{
  // BEV가 다 보이게 범위를 넓힘 (기본 앞 60cm, 좌우 35cm)
  x_max_ = 0.60;
  y_half_ = 0.35;
  for (const auto &q : bev_rect_)
  {
    x_max_ = std::max(x_max_, q.x() + 0.03);
    y_half_ = std::max(y_half_, std::abs(q.y()) + 0.03);
  }
  const double w_m = 2.0 * y_half_, h_m = x_max_ - x_min_;
  const double margin = 8.0, top_text = 54.0;
  const double sx = (width() - 2 * margin) / w_m;
  const double sy = (height() - 2 * margin - top_text) / h_m;
  scale_ = std::max(1.0, std::min(sx, sy));
  // 남는 세로 공간은 위아래로 나눠 가운데 정렬 (세로로 긴 칸에서도 그림이 아래에 붙지 않게)
  const double avail_h = height() - 2 * margin - top_text, draw_h = h_m * scale_;
  const double top = margin + top_text + std::max(0.0, (avail_h - draw_h) / 2.0);
  origin_ = QPointF(width() / 2.0, top + x_max_ * scale_);
}

QPointF PathView::toPx(const QPointF &m) const
{
  // x(앞) → 위쪽, y(왼쪽) → 왼쪽
  return QPointF(origin_.x() - m.y() * scale_, origin_.y() - m.x() * scale_);
}

void PathView::paintEvent(QPaintEvent *)
{
  updateScale();
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.fillRect(rect(), QColor(32, 32, 32));

  const qint64 now = clock_.elapsed();
  const qint64 age = path_ms_ < 0 ? -1 : now - path_ms_;
  const bool stale = age < 0 || age > STALE_MS;

  // ====== 10cm 격자
  p.setPen(QPen(QColor(60, 60, 60), 1));
  for (double x = std::ceil(x_min_ * 10) / 10; x <= x_max_ + 1e-6; x += 0.1)
    p.drawLine(toPx({x, y_half_}), toPx({x, -y_half_}));
  for (double y = -std::floor(y_half_ * 10) / 10; y <= y_half_ + 1e-6; y += 0.1)
    p.drawLine(toPx({x_min_, y}), toPx({x_max_, y}));
  p.setPen(QPen(QColor(95, 95, 95), 1, Qt::DashLine));
  p.drawLine(toPx({x_min_, 0}), toPx({x_max_, 0})); // 로봇 정면 축
  p.setPen(QColor(120, 120, 120));
  p.setFont(QFont(font().family(), 8));
  for (int i = 1; i * 0.1 <= x_max_ + 1e-6; ++i)
    p.drawText(toPx({i * 0.1, -y_half_}) + QPointF(-30, 4), QString("%1cm").arg(i * 10));

  // ====== BEV 범위
  drawBev(p);

  // ====== 지난 경로 (흐리게)
  for (size_t i = 0; i < history_.size(); ++i)
  {
    QColor c = sourceColor(history_[i].src);
    c.setAlpha(25 + int(70.0 * (i + 1) / history_.size()));
    p.setPen(QPen(c, 1.5));
    QPolygonF px;
    for (const auto &q : history_[i].pts)
      px << toPx(q);
    p.drawPolyline(px);
  }

  // ====== 현재 경로
  double len = 0.0;
  if (!path_.isEmpty())
  {
    QPolygonF px;
    for (int i = 0; i < path_.size(); ++i)
    {
      px << toPx(path_[i]);
      if (i > 0)
        len += std::hypot(path_[i].x() - path_[i - 1].x(), path_[i].y() - path_[i - 1].y());
    }
    const QColor c = stale ? QColor(130, 130, 130) : sourceColor(path_src_at_recv_);
    p.setPen(QPen(c, 2.5, stale ? Qt::DashLine : Qt::SolidLine, Qt::RoundCap));
    p.drawPolyline(px);
    p.setPen(Qt::NoPen);
    p.setBrush(c);
    for (const auto &q : px)
      p.drawEllipse(q, 2.2, 2.2);
    p.setBrush(QColor(255, 255, 255));
    p.drawEllipse(px.first(), 3.5, 3.5); // 시작점 (사각지대 채움이 안 되면 BEV 아랫줄 근처)
  }

  // ====== 목표점 + lookahead 원 + pure pursuit 호 (목표점을 지나고 로봇 정면에 접하는 원)
  double kappa_pp = 0.0;
  bool has_target = false;
  if (target_ms_ >= 0 && now - target_ms_ < TARGET_MS)
  {
    has_target = true;
    const double ld = std::hypot(target_.x(), target_.y());
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(QColor(255, 90, 90, 110), 1, Qt::DashLine));
    p.drawEllipse(toPx({0, 0}), ld * scale_, ld * scale_);
    p.setPen(QPen(QColor(255, 90, 90, 110), 1, Qt::DotLine));
    p.drawLine(toPx({0, 0}), toPx(target_)); // 현

    if (ld > 1e-3)
    {
      kappa_pp = 2.0 * target_.y() / (ld * ld); // κ = 2y / ld²
      const double alpha = std::atan2(target_.y(), target_.x());
      const double s_end = std::abs(kappa_pp) < 1e-6 ? ld : 2.0 * alpha / kappa_pp; // 호 길이 = 중심각 2α × R
      drawArc(p, kappa_pp, 0.0, s_end, QPen(QColor(255, 90, 90), 2, Qt::DashLine));
    }
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(255, 90, 90));
    p.drawEllipse(toPx(target_), 5, 5);
  }

  // ====== 실제 출력 호: drive가 낸 v, w (가감속 제한·피봇 반영) → κ = w / v, 1초 동안 갈 거리만큼
  const bool vw_ok = vw_ms_ >= 0 && now - vw_ms_ < DRIVE_MS;
  if (vw_ok && std::abs(v_) > 1e-3)
  {
    const double k = w_ / v_;
    const double s = std::copysign(std::clamp(std::abs(v_) * 1.0, 0.10, 0.60), v_);
    drawArc(p, k, 0.0, s, QPen(CMD_COLOR, 2.5, Qt::SolidLine, Qt::RoundCap));
  }

  // ====== 로봇 외곽 (바퀴 축 기준 앞 9cm, 뒤 14cm, 반폭 10.5cm)
  p.setBrush(QColor(255, 255, 255, 30));
  p.setPen(QPen(QColor(220, 220, 220), 1.2));
  p.drawPolygon(QPolygonF({toPx({0.09, 0.105}), toPx({0.09, -0.105}), toPx({-0.14, -0.105}), toPx({-0.14, 0.105})}));
  p.drawLine(toPx({0, 0}), toPx({0.06, 0}));
  p.setBrush(QColor(220, 220, 220));
  p.drawEllipse(toPx({0, 0}), 2.5, 2.5);

  // ====== 바퀴 속도
  drawWheels(p, now);

  // ====== 곡률 비교 (pure pursuit 목표 vs 실제 출력). 차이가 크면 가감속 제한·w_max·피봇 때문
  {
    QFont f = font();
    f.setPointSize(8);
    p.setFont(f);
    const QString kt = QString("κ  target %1   cmd %2  [1/m]")
                           .arg(has_target ? QString::number(kappa_pp, 'f', 2) : QString("-"))
                           .arg(vw_ok && std::abs(v_) > 1e-3 ? QString::number(w_ / v_, 'f', 2) : QString("-"));
    p.setPen(QColor(170, 170, 170));
    p.drawText(QPointF(8, 48), kt);
  }

  // ====== 글자: 경로 정보, 주행 상태
  p.setFont(QFont(font().family(), 9));
  QString line1 = QString("path %1  %2Hz  age %3  len %4m  n=%5")
                      .arg(path_src_at_recv_)
                      .arg(hz_, 0, 'f', 1)
                      .arg(age < 0 ? QString("-") : QString("%1ms").arg(age))
                      .arg(len, 0, 'f', 2)
                      .arg(path_.size());
  p.setPen(stale ? QColor(255, 120, 120) : QColor(230, 230, 230));
  p.drawText(QPointF(8, 16), line1);

  const bool state_fresh = state_ms_ >= 0 && now - state_ms_ < 500;
  p.setPen(QColor(170, 170, 170));
  p.drawText(QRectF(8, 20, width() - 16, 16), Qt::AlignLeft | Qt::TextSingleLine,
             state_fresh ? drive_state_ : QString("drive: -"));

  // 배지: STALE / HOLD / stop 이유
  QStringList badges;
  QList<QColor> colors;
  if (stale)
  {
    badges << "STALE";
    colors << QColor(150, 60, 60);
  }
  if (state_fresh && drive_state_.contains(" HOLD"))
  {
    badges << "HOLD";
    colors << QColor(170, 130, 20);
  }
  static const QRegularExpression stop_re("stop:(\\S+)");
  const auto m = stop_re.match(drive_state_);
  if (state_fresh && m.hasMatch())
  {
    badges << m.captured(1);
    colors << QColor(150, 60, 60);
  }
  double bx = width() - 8;
  for (int i = badges.size() - 1; i >= 0; --i)
  {
    const double w = p.fontMetrics().horizontalAdvance(badges[i]) + 12;
    bx -= w;
    const QRectF r(bx, height() - 26, w, 18);
    p.setPen(Qt::NoPen);
    p.setBrush(colors[i]);
    p.drawRoundedRect(r, 4, 4);
    p.setPen(Qt::white);
    p.drawText(r, Qt::AlignCenter, badges[i]);
    bx -= 6;
  }
}

void PathView::setIntParam(const QString &name, int v)
{
  int *dst = name == "bev.top_y" ? &bev_top_y_ : name == "bev.bot_y" ? &bev_bot_y_
           : name == "bev.top_w" ? &bev_top_w_ : name == "bev.bot_w" ? &bev_bot_w_
           : name == "bev.lane_w_px" ? &lane_w_px_ : name == "geo.lane_w_mm" ? &lane_w_mm_
           : name == "geo.bev_depth_mm" ? &bev_depth_mm_ : name == "geo.bev_bottom_mm" ? &bev_bottom_mm_
           : nullptr;
  if (!dst || *dst == v)
    return;
  *dst = v;
  rebuildBev();
}

void PathView::setDoubleParam(const QString &name, double v)
{
  if (name == "drive.uart.wheel_sep_m")
    wheel_sep_ = v;
  else if (name == "drive.uart.pivot_sep_m")
    pivot_sep_ = v;
}

// drive_node::publish_uart와 같은 식: 두 바퀴가 같은 방향이면 wheel_sep,
// 안쪽 바퀴가 거꾸로 돌수록(제자리 회전에 가까울수록) pivot_sep 쪽으로 섞음
double PathView::wheelSep(double v, double w) const
{
  const double half_turn = std::abs(w) * wheel_sep_ / 2.0;
  if (half_turn > std::abs(v))
    return pivot_sep_ + (wheel_sep_ - pivot_sep_) * (std::abs(v) / half_turn);
  return wheel_sep_;
}

// lineDetect_node와 같은 계산으로 BEV 네모와 보이는 영역을 로봇 좌표로
//  BEV 픽셀 → 로봇: x = bottom + (S-1-py)·my,  y = (S/2 - px)·mx   (lane_path::bev_to_robot)
void PathView::rebuildBev()
{
  const double S = BEV_OUT;
  const double mx = lane_w_mm_ / 1000.0 / std::max(1, lane_w_px_);
  const double my = bev_depth_mm_ / 1000.0 / S;
  const double bottom = bev_bottom_mm_ / 1000.0;
  auto toRobot = [&](double px, double py) { return QPointF(bottom + (S - 1 - py) * my, (S / 2.0 - px) * mx); };

  bev_rect_ = QPolygonF({toRobot(0, 0), toRobot(S, 0), toRobot(S, S - 1), toRobot(0, S - 1), toRobot(0, 0)});

  // 원본 영상 사다리꼴 → BEV 가운데 절반 폭 (lineDetect_node::update_bev_matrix)
  const float w = CAM_W, h = CAM_H, cx = w / 2.0f;
  const float ty = h * bev_top_y_ / 100.0f, by = h * bev_bot_y_ / 100.0f - 1;
  const float tw = w * bev_top_w_ / 200.0f, bw = w * bev_bot_w_ / 200.0f;
  const std::vector<cv::Point2f> src = {{cx - tw, ty}, {cx + tw, ty}, {cx + bw, by}, {cx - bw, by}};
  const float m = S * 0.25f;
  const std::vector<cv::Point2f> dst = {{m, 0}, {float(S) - m, 0}, {float(S) - m, float(S)}, {m, float(S)}};
  bev_visible_.clear();
  if (ty >= by || tw <= 0 || bw <= 0)
    return;
  const cv::Mat M = cv::getPerspectiveTransform(src, dst);

  // 원본 영상에서 사다리꼴 윗변 줄(ty)부터 아래 전체: 이 테두리를 펴면 BEV 안에서 보이는 영역
  // (ty 위쪽은 BEV 네모 바깥 = 더 먼 곳이라 필요 없음, 지평선 너머로 뒤집히는 것도 피함)
  std::vector<cv::Point2f> edge;
  const int n = 32;
  for (int i = 0; i <= n; ++i) edge.emplace_back(w * i / n, h - 1);              // 아랫변 (왼→오)
  for (int i = 1; i <= n; ++i) edge.emplace_back(w - 1, (h - 1) + (ty - (h - 1)) * i / n); // 오른쪽 변 (아래→위)
  for (int i = 1; i <= n; ++i) edge.emplace_back((w - 1) * (1.0 - double(i) / n), ty);    // ty 줄 (오→왼)
  for (int i = 1; i < n; ++i) edge.emplace_back(0, ty + ((h - 1) - ty) * i / n);  // 왼쪽 변 (위→아래)
  std::vector<cv::Point2f> out;
  cv::perspectiveTransform(edge, out, M);
  QPolygonF vis;
  for (const auto &q : out)
    vis << toRobot(q.x, q.y);
  vis << vis.first();
  bev_visible_ = vis.intersected(bev_rect_);
}

void PathView::drawBev(QPainter &p)
{
  if (bev_rect_.isEmpty())
    return;
  QPolygonF rect_px, vis_px;
  for (const auto &q : bev_rect_) rect_px << toPx(q);
  for (const auto &q : bev_visible_) vis_px << toPx(q);

  // 보이는 영역: 옅게 칠함. BEV 네모 안에서 칠해지지 않은 곳 = 사각지대 (카메라가 못 봄)
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(255, 255, 255, 16));
  p.drawPolygon(vis_px);

  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(QColor(200, 200, 120, 170), 1.2, Qt::DashLine));
  p.drawPolygon(rect_px);

  // BEV 아랫줄 (geo.bev_bottom_mm): 이보다 가까운 경로는 카메라가 아니라 기억으로 채운 것
  QFont f = font();
  f.setPointSize(8);
  p.setFont(f);
  p.setPen(QColor(200, 200, 120, 200));
  const QPointF tl = toPx(bev_rect_[0]);
  p.drawText(tl + QPointF(4, 12), QString("BEV  %1~%2 cm")
                                      .arg(bev_bottom_mm_ / 10)
                                      .arg((bev_bottom_mm_ + bev_depth_mm_) / 10));
}
