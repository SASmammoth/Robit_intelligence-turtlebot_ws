#ifndef tb_gui_RECORDER_HPP_
#define tb_gui_RECORDER_HPP_

// 녹화: GUI의 USB / Skeleton / 경로 보기 화면을 영상으로, 노드 로그를 시각과 함께 파일로 저장
//  - start(): <base>/<yyyy-MM-dd.HH-mm-ss>/ 폴더를 만들고 그 안에 usb.mp4, skel.mp4, path.mp4, events.log
//    (params.yaml은 MainWindow가 같은 폴더에 저장)
//  - 토픽마다 들어오는 주기가 달라서 kFps 타이머로 스트림별 최신 프레임을 씀 → 재생 속도 = 실제 시간
//  - 영상은 H.264 mp4 (안 되면 mp4v): stop()으로 닫아야 재생됨 (GUI가 강제 종료되면 못 읽음)
//  - 영상 크기는 스트림별 첫 프레임 크기로 고정 (USB·Skeleton은 첫 프레임이 와야 파일이 열림)

#ifndef Q_MOC_RUN
#include <opencv2/opencv.hpp>
#endif
#include <QObject>
#include <QImage>
#include <QString>
#include <QFile>
#include <QTimer>
#include <QElapsedTimer>
#include <QWidget>
#include <map>
#include <string>

class Recorder : public QObject
{
  Q_OBJECT
public:
  static constexpr int kFps = 15;

  explicit Recorder(QObject *parent = nullptr);
  ~Recorder();

  bool start(const QString &base_dir); // 폴더를 못 만들면 false
  void stop();
  bool isRecording() const { return recording_; }
  QString dir() const { return dir_; }
  qint64 elapsedMs() const { return recording_ ? elapsed_.elapsed() : 0; }

  void setFrame(const QString &stream, const QImage &img); // "usb", "skel": 최신 프레임만 기억
  void setPathView(QWidget *w) { path_view_ = w; }          // 매 타이머마다 render()로 캡처
  void logEvent(const QString &text);                       // GUI 쪽 이벤트 (녹화 시작/중지 등)
  void logNode(const QString &node, int level, qint64 stamp_ns, const QString &text); // /rosout

private:
  struct Track
  {
    cv::VideoWriter writer;
    cv::Size size;
    QImage last;
    int frames = 0;
    bool failed = false; // 파일 열기 실패 → 이 스트림은 더 시도 안 함
  };
  void writeFrames();
  void writeImage(const std::string &stream, const QImage &img);
  void writeLine(const QString &line);

  bool recording_ = false;
  QString dir_;
  std::map<std::string, Track> tracks_;
  QWidget *path_view_ = nullptr;
  QFile log_;
  QTimer timer_;
  QElapsedTimer elapsed_;
};

#endif // tb_gui_RECORDER_HPP_
