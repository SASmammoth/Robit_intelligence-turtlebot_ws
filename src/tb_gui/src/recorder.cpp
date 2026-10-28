#include "../include/tb_gui/recorder.hpp"

#include <QDateTime>
#include <QDir>
#include <QPainter>
#include <QTextStream>

Recorder::Recorder(QObject *parent) : QObject(parent)
{
  timer_.setInterval(1000 / kFps);
  timer_.setTimerType(Qt::PreciseTimer);
  connect(&timer_, &QTimer::timeout, this, &Recorder::writeFrames);
}

Recorder::~Recorder()
{
  stop();
}

bool Recorder::start(const QString &base_dir)
{
  if (recording_)
    return true;
  const QDateTime now = QDateTime::currentDateTime();
  const QString dir = QDir(base_dir).filePath(now.toString("yyyy-MM-dd.HH-mm-ss"));
  if (!QDir().mkpath(dir))
    return false;
  log_.setFileName(QDir(dir).filePath("events.log"));
  if (!log_.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append))
    return false;

  dir_ = dir;
  tracks_.clear();
  recording_ = true;
  elapsed_.start();
  writeLine("# PC 시각 (녹화 경과) | 로그 찍은 쪽 시각 (unix, 로봇 시계) | 노드 | 레벨 | 내용");
  logEvent(QString("녹화 시작: %1 (%2 fps)").arg(dir).arg(kFps));
  timer_.start();
  return true;
}

void Recorder::stop()
{
  if (!recording_)
    return;
  timer_.stop();
  QStringList counts;
  for (auto &[name, t] : tracks_)
  {
    t.writer.release();
    counts << QString("%1 %2프레임").arg(QString::fromStdString(name)).arg(t.frames);
  }
  logEvent(QString("녹화 중지: %1초, %2").arg(elapsed_.elapsed() / 1000.0, 0, 'f', 1).arg(counts.join(", ")));
  log_.close();
  tracks_.clear();
  recording_ = false;
}

void Recorder::setFrame(const QString &stream, const QImage &img)
{
  if (recording_)
    tracks_[stream.toStdString()].last = img;
}

void Recorder::logEvent(const QString &text)
{
  if (!recording_)
    return;
  writeLine(QString("%1 | %2 | %3 | %4 | %5")
                .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"))
                .arg(QString(), 27)
                .arg("gui", -5)
                .arg("-", -5)
                .arg(text));
}

void Recorder::logNode(const QString &node, int level, qint64 stamp_ns, const QString &text)
{
  if (!recording_)
    return;
  // rcl_interfaces/Log: DEBUG 10, INFO 20, WARN 30, ERROR 40, FATAL 50
  const char *lv = level >= 50 ? "FATAL" : level >= 40 ? "ERROR" : level >= 30 ? "WARN" : level >= 20 ? "INFO" : "DEBUG";
  const QDateTime dt = QDateTime::fromMSecsSinceEpoch(stamp_ns / 1000000);
  writeLine(QString("%1 | %2.%3 %4 | %5 | %6 | %7")
                .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"))
                .arg(stamp_ns / 1000000000)
                .arg((stamp_ns / 1000000) % 1000, 3, 10, QChar('0'))
                .arg(dt.toString("HH:mm:ss.zzz"))
                .arg(node.left(node.indexOf('_')), -5)
                .arg(lv, -5)
                .arg(text));
}

// 앞에 녹화 경과 시간을 붙여 한 줄 쓰고 바로 flush (GUI가 죽어도 남게)
void Recorder::writeLine(const QString &line)
{
  const QString t = QString("(+%1s) ").arg(elapsed_.elapsed() / 1000.0, 8, 'f', 3);
  QTextStream out(&log_);
  out.setCodec("UTF-8");
  out << (line.startsWith('#') ? line : t + line) << '\n';
  out.flush();
  log_.flush();
}

void Recorder::writeFrames()
{
  for (const char *s : {"usb", "skel"})
  {
    auto it = tracks_.find(s);
    if (it != tracks_.end() && !it->second.last.isNull())
      writeImage(s, it->second.last); // 새 프레임이 안 왔으면 지난 프레임을 다시 씀 (시간 맞춤)
  }
  if (path_view_)
  {
    QImage img(path_view_->size(), QImage::Format_RGB888);
    img.fill(Qt::black);
    QPainter p(&img);
    path_view_->render(&p); // 화면에 안 보여도 그려짐
    p.end();
    writeImage("path", img);
  }
}

void Recorder::writeImage(const std::string &stream, const QImage &img)
{
  Track &t = tracks_[stream];
  if (t.failed)
    return;
  const QImage bgr = img.convertToFormat(QImage::Format_BGR888);
  cv::Mat mat(bgr.height(), bgr.width(), CV_8UC3, const_cast<uchar *>(bgr.constBits()), bgr.bytesPerLine());
  if (!t.writer.isOpened())
  {
    t.size = cv::Size(mat.cols & ~1, mat.rows & ~1); // 짝수 크기 (일부 플레이어 호환)
    const std::string path = QDir(dir_).filePath(QString::fromStdString(stream) + ".mp4").toStdString();
    // H.264가 안 되는 OpenCV(ffmpeg 빌드)면 MPEG-4로
    const char *codec = "avc1";
    if (!t.writer.open(path, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), kFps, t.size))
    {
      codec = "mp4v";
      if (!t.writer.open(path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), kFps, t.size))
      {
        t.failed = true;
        logEvent(QString("%1.mp4 열기 실패").arg(QString::fromStdString(stream)));
        return;
      }
    }
    logEvent(QString("%1.mp4 시작 (%2x%3, %4)")
                 .arg(QString::fromStdString(stream))
                 .arg(t.size.width)
                 .arg(t.size.height)
                 .arg(codec));
  }
  if (mat.cols != t.size.width || mat.rows != t.size.height)
  {
    cv::Mat resized;
    cv::resize(mat, resized, t.size);
    t.writer.write(resized);
  }
  else
    t.writer.write(mat);
  ++t.frames;
}
