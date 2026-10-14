/**
 * @file /include/tb_gui/main_window.hpp
 *
 * @brief Qt based gui for tb_gui.
 **/
#ifndef tb_gui_MAIN_WINDOW_H
#define tb_gui_MAIN_WINDOW_H

#include <QMainWindow>
#include <QImage>
#include <QPixmap>
#include <QTimer>
#include <QSpinBox>
#include <QSlider>
#include <QLabel>
#include <QRadioButton>
#include <QCheckBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QGridLayout>
#include <vector>
#include <QIcon>
#include <QSet>
#include <QKeyEvent>
#include "qnode.hpp"
#include "ui_mainwindow.h"

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
  QNode *qnode;

protected:
  bool eventFilter(QObject *obj, QEvent *ev) override; // WASD 키 가로채기

private:
  Ui::MainWindowDesign *ui;
  void closeEvent(QCloseEvent *event);

  void showImage(QLabel *label, const QImage &img);
  void queueParam(const QString &name, int v); // 50ms 묶음 전송

  // HSV (대상별)
  struct HsvWidget
  {
    QSlider *slider;
    QLabel *label;
  };
  void loadTargetToSliders();
  QMap<QString, HsvWidget> hsv_widgets_;
  QMap<QString, int> hsv_cache_;
  QString current_target_ = "white";

  // 전역 파라미터 (bev., morph., skel., path.)
  struct GlobalWidget
  {
    QSlider *slider = nullptr;
    QLabel *label = nullptr;
    QSpinBox *spin = nullptr;
  };
  void applyGlobalParam(const QString &name, int v);
  QMap<QString, GlobalWidget> global_widgets_;

  // 상태 표시
  void showPathSource(const QString &src);
  void showMission();
  int turn_ = 0;
  bool parking_ = false;
  bool construction_ = false;     // GUI에서 켠 공사 구간 모드 (/mission/construction)
  bool construction_state_ = false; // path_node가 실제로 공사 구간 모드인지 (/lane/construction)

  // ====== View 탭: 카메라를 크게 + 키보드 조작 (setupViewTab 참고)
  enum class DriveMode { Off, Wasd, Auto };
  void setupViewTab();
  bool viewTabActive() const;
  void applyViewTabSize();             // View 탭이면 아래 탭 영역을 줄여 카메라를 크게
  DriveMode driveMode() const;
  void setDriveMode(DriveMode m);
  void refreshViewStatus();            // 모드 배지·미션 표시 갱신
  void setTurnMomentary(int key, bool press); // 자동 주행 중 A/D: 누르는 동안만 Left/Right
  void setBarrier(bool on);            // 차단바 정지: /drive/cmd STOP을 10Hz로 계속 보냄
  bool handleViewKey(QKeyEvent *ke, bool press);
  QWidget *view_tab_ = nullptr;
  QLabel *label_mode_ = nullptr, *label_keys_ = nullptr, *label_drive_brief_ = nullptr;
  QCheckBox *check_construction_ = nullptr, *check_barrier_ = nullptr;
  QLabel *label_con_ = nullptr;        // 공사 모드 상태 배지 (/lane/construction_info)
  void showConstructionInfo(const QString &info);
  void exitConstruction();             // 공사 모드 끝내기 (버튼, X키)
  QList<int> turn_keys_;               // 누르고 있는 A/D (나중에 누른 것이 우선)
  QTimer barrier_timer_;
  QTimer status_timeout_; // 경로 출처가 1초 넘게 안 오면 회색

  // 파라미터 파일 (Save/Load 대상, Save as/Open으로 바꿈)
  QString param_file_;
  void setParamFile(const QString &path); // 바꾸고 QSettings에 기억
  void saveParamsTo(const QString &path); // 폴더가 없으면 만들고 저장

  // 파라미터 스핀박스 (주행 탭, Params 탭 공통)
  struct ParamSpinDef
  {
    const char *name, *text, *suffix;
    int min, max, step;
  };
  void addParamSpin(QGridLayout *grid, QWidget *parent, int idx, int cols, const ParamSpinDef &d);
  void setupParamTab();

  // 주행 (WASD)
  void updateDrive();                               // 눌린 키 → 좌우 속도 계산
  void sendDrive(int l, int r, bool force = false); // 값이 바뀔 때만 전송
  void stopDrive(bool force = false);               // 키 상태 초기화 + 0 0
  void showUart(const QString &text);               // TB_Uart_TX 표시
  QSet<int> drive_keys_;
  int drive_l_ = 0, drive_r_ = 0; // 마지막으로 보낸 값

  // 자동 주행 (tb_drive, Tab 3)
  void setupDriveTab();                       // 파라미터 스핀박스 생성 + 연결
  void setAutoDrive(bool on);                 // drive.enable 전송 (실패하면 체크 해제)
  void showDriveState(const QString &text);   // /drive/state 표시
  QMap<QString, QCheckBox *> bool_widgets_;   // drive.require_cmd 등
  QTimer drive_state_timeout_;                // 1초 넘게 안 오면 회색

  // 공통
  QMap<QString, int> pending_;
  QTimer send_timer_;
  QTimer param_poll_timer_;
  QTimer param_refresh_timer_; // 노드 값을 주기적으로 다시 읽어 화면과 맞춤
  void makeTabsScrollable();   // 아래 탭 페이지를 스크롤 영역으로 감쌈
};

#endif // tb_gui_MAIN_WINDOW_H