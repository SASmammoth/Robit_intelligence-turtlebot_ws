#include "../include/tb_gui/recorder.hpp"

#include <QDateTime>
#include <QDir>
#include <QFontDatabase>
#include <QTextStream>

// 영상 배치: 위 화면 3개 (각 640x480) / 시간 줄 / 로그 (왼쪽) | 시간·수신 표 (오른쪽)
static const int kPanelW = Recorder::kW / 3, kPanelH = 480;
static const int kBarY = kPanelH, kBarH = 44;
static const int kLogY = kBarY + kBarH;
static const int kDebugX = Recorder::kW * 3 / 5; // 로그 칸 끝 = 표 칸 시작
static const size_t kMaxLines = 60;
static const qint64 kNoSignalMs = 1000;

Recorder::Recorder(QObject *parent) : QObject(parent)
{
  timer_.setInterval(1000 / kFps);
  timer_.setTimerType(Qt::PreciseTimer);
  connect(&timer_, &QTimer::timeout, this, &Recorder::tick);
}

Recorder::~Recorder()
{
  stop();
}

bool Recorder::start(const QString &base_dir)
{
  if (recording_)
    return true;
  const QString dir = QDir(base_dir).filePath(QDateTime::currentDateTime().toString("yyyy-MM-dd.HH-mm-ss"));
  if (!QDir().mkpath(dir))
    return false;

  // H.264가 안 되는 OpenCV(ffmpeg 빌드)면 MPEG-4로
  const std::string video = QDir(dir).filePath("record.mp4").toStdString();
  codec_ = "avc1";
  if (!writer_.open(video, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), kFps, cv::Size(kW, kH)))
  {
    codec_ = "mp4v";
    writer_.open(video, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), kFps, cv::Size(kW, kH));
  }
  log_.setFileName(QDir(dir).filePath("events.log"));
  if (!writer_.isOpened() || !log_.open(QIODevice::WriteOnly | QIODevice::Text))
  {
    writer_.release();
    QDir(dir).removeRecursively(); // 실패하면 아무 파일도 남기지 않음
    return false;
  }

  dir_ = dir;
  usb_ = {};
  skel_ = {};
  lines_.clear();
  debug_text_.clear();
  frames_ = dropped_ = 0;
  queue_.clear();
  q_stop_ = false;
  encoder_ = std::thread(&Recorder::encodeLoop, this);
  recording_ = true;
  elapsed_.start();
  log_.write("# (녹화 경과) PC 시각 | 로그 찍은 쪽 시각 (unix, 로봇 시계) | 노드 | 레벨 | 내용\n"); // 소스가 UTF-8
  logEvent(QString("녹화 시작: %1 (%2x%3, %4 fps, %5)").arg(dir).arg(kW).arg(kH).arg(kFps).arg(codec_));
  timer_.start();
  tick(); // 첫 장
  return true;
}

void Recorder::stop()
{
  if (!recording_)
    return;
  timer_.stop();
  logEvent(QString("녹화 중지: %1초, %2장%3")
               .arg(elapsed_.elapsed() / 1000.0, 0, 'f', 1)
               .arg(frames_)
               .arg(dropped_ ? QString(" (인코딩 밀려 %1장 버림)").arg(dropped_) : QString()));
  {
    std::lock_guard<std::mutex> lk(q_mtx_);
    q_stop_ = true;
  }
  q_cv_.notify_one();
  encoder_.join(); // 대기열에 남은 장을 다 쓰고 끝남
  writer_.release();
  log_.close();
  recording_ = false;
}

void Recorder::setFrame(const QString &stream, const QImage &img)
{
  if (!recording_)
    return;
  Stream &s = stream == "usb" ? usb_ : skel_;
  s.img = img;
  s.ms = elapsed_.elapsed();
}

void Recorder::logEvent(const QString &text)
{
  if (!recording_)
    return;
  const QString t = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
  addLine(QString("%1 | %2 | %3 | %4 | %5").arg(t).arg(QString(), 27).arg("gui", -5).arg("-", -5).arg(text),
          QString("%1 gui   %2 %3").arg(t).arg("", -5).arg(text), QColor(0x6c, 0xb4, 0xff));
}

void Recorder::logNode(const QString &node, int level, qint64 stamp_ns, const QString &text)
{
  if (!recording_)
    return;
  // rcl_interfaces/Log: DEBUG 10, INFO 20, WARN 30, ERROR 40, FATAL 50
  const char *lv = level >= 50 ? "FATAL" : level >= 40 ? "ERROR" : level >= 30 ? "WARN" : level >= 20 ? "INFO" : "DEBUG";
  const QColor color = level >= 40 ? QColor(0xff, 0x6b, 0x6b) : level >= 30 ? QColor(0xf5, 0xa6, 0x23)
                       : node == "brain_node"                ? QColor(0xe0, 0xe0, 0xe0)
                                                             : QColor(0x99, 0xaa, 0xbb);
  const QString t = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
  const QString short_node = node.left(node.indexOf('_'));
  const QString file_line = QString("%1 | %2.%3 %4 | %5 | %6 | %7")
                                .arg(t)
                                .arg(stamp_ns / 1000000000)
                                .arg((stamp_ns / 1000000) % 1000, 3, 10, QChar('0'))
                                .arg(QDateTime::fromMSecsSinceEpoch(stamp_ns / 1000000).toString("HH:mm:ss.zzz"))
                                .arg(short_node, -5)
                                .arg(lv, -5)
                                .arg(text);
  addLine(file_line, QString("%1 %2 %3 %4").arg(t).arg(short_node, -5).arg(lv, -5).arg(text), color);
}

// 파일에는 녹화 경과 시간을 붙여 바로 flush (GUI가 죽어도 남게), 영상에는 최근 줄만 기억
void Recorder::addLine(const QString &file_line, const QString &video_line, const QColor &color)
{
  QTextStream out(&log_);
  out.setCodec("UTF-8");
  out << QString("(+%1s) ").arg(elapsed_.elapsed() / 1000.0, 8, 'f', 3) << file_line << '\n';
  out.flush();
  log_.flush();

  lines_.push_back({video_line, color});
  while (lines_.size() > kMaxLines)
    lines_.pop_front();
}

// 지금까지 나왔어야 할 장 수만큼 채움 (타이머가 늦으면 같은 장을 여러 번)
void Recorder::tick()
{
  const qint64 due = elapsed_.elapsed() * kFps / 1000 + 1;
  const qint64 n = due - frames_;
  if (n <= 0)
    return;
  const QImage frame = compose();
  {
    std::lock_guard<std::mutex> lk(q_mtx_);
    if (queue_.size() > size_t(kFps) * 2) // 인코딩이 2초 넘게 밀림 → 버림 (메모리 보호)
      dropped_ += n;
    else
      for (qint64 i = 0; i < n; ++i)
        queue_.push_back(frame); // QImage는 공유 복사라 n장이어도 메모리는 한 장
  }
  q_cv_.notify_one();
  frames_ = due;
}

QImage Recorder::compose()
{
  const qint64 now = elapsed_.elapsed();
  QImage frame(kW, kH, QImage::Format_RGB32);
  frame.fill(QColor(0x1b, 0x1b, 0x1f));
  QPainter p(&frame);
  p.setRenderHint(QPainter::SmoothPixmapTransform);

  // ====== 위: USB | Path | Skeleton
  drawPanel(p, QRect(0, 0, kPanelW, kPanelH), "USB", usb_.img, usb_.ms < 0 ? -1 : now - usb_.ms);
  QImage path;
  if (path_view_)
  {
    path = QImage(path_view_->size(), QImage::Format_RGB32);
    path.fill(Qt::black);
    QPainter pp(&path);
    path_view_->render(&pp); // 화면에 안 보여도 그려짐
  }
  drawPanel(p, QRect(kPanelW, 0, kPanelW, kPanelH), "Path", path, path.isNull() ? -1 : 0);
  drawPanel(p, QRect(2 * kPanelW, 0, kPanelW, kPanelH), "Skeleton", skel_.img, skel_.ms < 0 ? -1 : now - skel_.ms);

  // ====== 시간 줄: ● REC 경과 | PC 날짜·시각
  QFont mono = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  mono.setStyleHint(QFont::TypeWriter);
  mono.setPixelSize(26);
  mono.setBold(true);
  p.setFont(mono);
  p.fillRect(QRect(0, kBarY, kW, kBarH), QColor(0x2b, 0x2b, 0x31));
  const QRect bar(16, kBarY, kW - 32, kBarH);
  p.setPen(QColor(0xe7, 0x4c, 0x3c));
  p.drawText(bar, Qt::AlignLeft | Qt::AlignVCenter,
             QString("● REC %1:%2.%3")
                 .arg(now / 60000, 2, 10, QChar('0'))
                 .arg(now / 1000 % 60, 2, 10, QChar('0'))
                 .arg(now % 1000, 3, 10, QChar('0')));
  p.setPen(QColor(0xe0, 0xe0, 0xe0));
  p.drawText(bar, Qt::AlignRight | Qt::AlignVCenter, QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz"));

  // ====== 아래 왼쪽: 최근 로그 (아래가 최신, 칸보다 긴 줄은 … 으로. 전체는 events.log)
  mono.setPixelSize(16);
  mono.setBold(false);
  p.setFont(mono);
  const QFontMetrics fm = p.fontMetrics();
  const int lh = fm.height() + 2;
  const int log_w = kDebugX - 24;
  int y = kH - 8;
  for (auto it = lines_.rbegin(); it != lines_.rend() && y - lh >= kLogY + 4; ++it)
  {
    p.setPen(it->color);
    p.drawText(QRect(12, y - lh, log_w, lh), Qt::AlignLeft | Qt::AlignVCenter | Qt::TextSingleLine,
               fm.elidedText(it->text, Qt::ElideRight, log_w));
    y -= lh;
  }

  // ====== 아래 오른쪽: 시간·수신 표 (GUI 주행 탭 "시간·수신 (디버그)"와 같은 글자)
  p.fillRect(QRect(kDebugX, kLogY, kW - kDebugX, kH - kLogY), QColor(0x23, 0x23, 0x28));
  p.setPen(QColor(0x44, 0x44, 0x4a));
  p.drawLine(kDebugX, kLogY, kDebugX, kH);
  p.setPen(QColor(0x9a, 0xaa, 0xbb));
  p.drawText(QRect(kDebugX + 12, kLogY + 6, kW - kDebugX - 24, lh), Qt::AlignLeft | Qt::AlignVCenter, "시간·수신 (디버그)");
  const int dbg_w = kW - kDebugX - 24;
  y = kLogY + 6 + lh + 4;
  for (const QString &line : debug_text_.split('\n'))
  {
    if (y + lh > kH - 4)
      break;
    p.setPen(line.contains("끊김") ? QColor(0xff, 0x6b, 0x6b) : QColor(0xe0, 0xe0, 0xe0));
    p.drawText(QRect(kDebugX + 12, y, dbg_w, lh), Qt::AlignLeft | Qt::AlignVCenter | Qt::TextSingleLine,
               fm.elidedText(line, Qt::ElideRight, dbg_w));
    y += lh;
  }
  return frame;
}

// 비율 유지해 가운데, 왼쪽 위 제목. age_ms < 0 (아직 안 옴) 또는 kNoSignalMs 넘음 → NO SIGNAL
void Recorder::drawPanel(QPainter &p, const QRect &r, const QString &title, const QImage &img, qint64 age_ms)
{
  p.fillRect(r, Qt::black);
  if (!img.isNull())
  {
    const QSize s = img.size().scaled(r.size(), Qt::KeepAspectRatio);
    p.drawImage(QRect(r.x() + (r.width() - s.width()) / 2, r.y() + (r.height() - s.height()) / 2, s.width(), s.height()), img);
  }
  QFont f = QFontDatabase::systemFont(QFontDatabase::GeneralFont);
  f.setPixelSize(18);
  f.setBold(true);
  p.setFont(f);
  const QRect tr(r.x() + 6, r.y() + 6, p.fontMetrics().horizontalAdvance(title) + 16, p.fontMetrics().height() + 6);
  p.fillRect(tr, QColor(0, 0, 0, 160));
  p.setPen(Qt::white);
  p.drawText(tr, Qt::AlignCenter, title);

  if (age_ms < 0 || age_ms > kNoSignalMs)
  {
    p.fillRect(r, QColor(0, 0, 0, img.isNull() ? 0 : 140));
    f.setPixelSize(32);
    p.setFont(f);
    p.setPen(QColor(0xe7, 0x4c, 0x3c));
    p.drawText(r, Qt::AlignCenter, age_ms < 0 ? "NO SIGNAL" : QString("NO SIGNAL\n%1 s").arg(age_ms / 1000.0, 0, 'f', 1));
  }
  p.setPen(QColor(0x44, 0x44, 0x4a));
  p.drawRect(r.adjusted(0, 0, -1, -1));
}

// 인코딩 스레드: 대기열의 장을 BGR로 바꿔 씀. stop()이면 남은 장까지 쓰고 끝
void Recorder::encodeLoop()
{
  cv::Mat bgr;
  for (;;)
  {
    QImage img;
    {
      std::unique_lock<std::mutex> lk(q_mtx_);
      q_cv_.wait(lk, [this] { return q_stop_ || !queue_.empty(); });
      if (queue_.empty())
        return;
      img = queue_.front();
      queue_.pop_front();
    }
    // Format_RGB32 = 메모리상 B, G, R, 0xFF
    const cv::Mat bgra(img.height(), img.width(), CV_8UC4, const_cast<uchar *>(img.constBits()), img.bytesPerLine());
    cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
    writer_.write(bgr);
  }
}
