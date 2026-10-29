#ifndef tb_gui_RECORDER_HPP_
#define tb_gui_RECORDER_HPP_

// 녹화: USB / 경로 보기 / Skeleton 화면 + 시간 + 이벤트 로그를 한 영상(record.mp4)으로, 로그는 events.log로도 저장
//  - start(): <base>/<yyyy-MM-dd.HH-mm-ss>/ 폴더를 만들고 그 안에 record.mp4, events.log
//    (params.yaml은 MainWindow가 같은 폴더에 저장)
//  - 영상 배치 (kW x kH):  [ USB | Path | Skeleton ] / 시간 줄 / 최근 로그 | 시간·수신 표
//  - kFps 타이머로 한 장씩 그림. 타이머가 늦으면 같은 장을 더 써서 영상 길이 = 실제 녹화 시간
//  - 1초 넘게 프레임이 안 온 화면은 NO SIGNAL
//  - 그리기는 GUI 스레드, H.264 인코딩은 별도 스레드 (GUI가 안 버벅이게)
//  - mp4는 stop()으로 닫아야 재생됨 (GUI가 강제 종료되면 못 읽음)

#ifndef Q_MOC_RUN
#include <opencv2/opencv.hpp>
#endif
#include <QObject>
#include <QImage>
#include <QString>
#include <QColor>
#include <QFile>
#include <QTimer>
#include <QElapsedTimer>
#include <QWidget>
#include <QPainter>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

class Recorder : public QObject
{
  Q_OBJECT
public:
  static constexpr int kFps = 15;
  static constexpr int kW = 1920, kH = 1080;

  explicit Recorder(QObject *parent = nullptr);
  ~Recorder();

  bool start(const QString &base_dir); // 폴더·파일을 못 만들면 false (만든 폴더는 지움)
  void stop();
  bool isRecording() const { return recording_; }
  QString dir() const { return dir_; }
  qint64 elapsedMs() const { return recording_ ? elapsed_.elapsed() : 0; }

  void setFrame(const QString &stream, const QImage &img); // "usb", "skel": 최신 프레임만 기억
  void setPathView(QWidget *w) { path_view_ = w; }          // 매 장마다 render()로 캡처
  void setDebugText(const QString &t) { debug_text_ = t; }  // 시간·수신 표 (MainWindow가 5Hz로 줌)
  void logEvent(const QString &text);                       // GUI 쪽 이벤트 (녹화 시작/중지 등)
  void logNode(const QString &node, int level, qint64 stamp_ns, const QString &text); // /rosout

private:
  struct Stream
  {
    QImage img;
    qint64 ms = -1; // 받은 시각 (elapsed_)
  };
  struct LogLine
  {
    QString text;
    QColor color;
  };
  void tick();
  QImage compose();
  void drawPanel(QPainter &p, const QRect &r, const QString &title, const QImage &img, qint64 age_ms);
  void addLine(const QString &file_line, const QString &video_line, const QColor &color);
  void encodeLoop();

  bool recording_ = false;
  QString dir_;
  Stream usb_, skel_;
  QWidget *path_view_ = nullptr;
  std::deque<LogLine> lines_; // 영상에 보일 최근 로그
  QString debug_text_;        // 시간·수신 표
  QFile log_;
  QTimer timer_;
  QElapsedTimer elapsed_;
  qint64 frames_ = 0;  // 인코딩 대기열에 넣은 장 수
  qint64 dropped_ = 0; // 인코딩이 밀려 버린 장 수
  const char *codec_ = "";

  // 인코딩 스레드
  cv::VideoWriter writer_;
  std::thread encoder_;
  std::mutex q_mtx_;
  std::condition_variable q_cv_;
  std::deque<QImage> queue_;
  bool q_stop_ = false;
};

#endif // tb_gui_RECORDER_HPP_
