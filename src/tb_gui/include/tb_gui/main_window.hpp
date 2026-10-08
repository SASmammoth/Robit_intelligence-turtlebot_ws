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
  QTimer status_timeout_; // 경로 출처가 1초 넘게 안 오면 회색

  QString paramFile() const; // 저장 파일 경로

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
};

#endif // tb_gui_MAIN_WINDOW_H