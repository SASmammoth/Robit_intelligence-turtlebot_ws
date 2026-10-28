/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 **/
#include "../include/tb_gui/main_window.hpp"

#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QGridLayout>
#include <QScrollArea>
#include <QStatusBar>
#include <QVariant>
#include <QSettings>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTabBar>
#include <QApplication>
#include <QDateTime>
#include <QFontDatabase>
#include <QPainter>
#include <QScrollBar>
#include <algorithm>
#include <cmath>
#include <memory>
#include <cstdlib>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();
  QObject::connect(qnode, &QNode::rosShutDown, this, &MainWindow::close);

  // ====== 영상 표시: 위쪽 영상 칸을 "주행 / 튜닝" 탭으로 나눔 (setupDisplayTabs)
  //  같은 영상이 두 탭에 다 있을 수 있어서, 영상 종류마다 라벨 목록에 보내고 보이는 라벨만 그림
  setupDisplayTabs();
  connect(qnode, &QNode::usbImageReceived, this, [this](const QImage &img) { showStream("usb", img); });
  connect(qnode, &QNode::maskImageReceived, this, [this](const QImage &img) { showStream("mask", img); });
  connect(qnode, &QNode::bevImageReceived, this, [this](const QImage &img) { showStream("bev", img); });
  connect(qnode, &QNode::roiImageReceived, this, [this](const QImage &img) { showStream("roi", img); });
  connect(qnode, &QNode::skelImageReceived, this, [this](const QImage &img) { showStream("skel", img); });
  setupRecorder();

  // ====== HSV 슬라이더 (흰/노랑 대상 전환)
  hsv_widgets_ = {
      {"h_min", {ui->sliderHueLow, ui->dispaly_Hue_Low}},
      {"h_max", {ui->sliderHueHigh, ui->dispaly_Hue_High}},
      {"s_min", {ui->sliderSaturationLow, ui->dispaly_Saturation_Low}},
      {"s_max", {ui->sliderSaturationHigh, ui->dispaly_Saturation_High}},
      {"v_min", {ui->sliderValueLow, ui->dispaly_Value_Low}},
      {"v_max", {ui->sliderValueHigh, ui->dispaly_Value_High}},
  };

  for (auto it = hsv_widgets_.begin(); it != hsv_widgets_.end(); ++it)
  {
    const QString key = it.key();
    QSlider *s = it->slider;
    QLabel *l = it->label;
    s->setRange(0, key.startsWith('h') ? 179 : 255);

    connect(s, &QSlider::valueChanged, this, [this, key, l](int v)
            {
      l->setNum(v);
      const QString param = current_target_ + "." + key;   // 예: "yellow.h_min"
      hsv_cache_[param] = v;
      queueParam(param, v); });
  }

  // ====== BEV 슬라이더
  struct SliderDef
  {
    const char *name;
    QSlider *slider;
    QLabel *label;
  };
  for (const auto &d : {SliderDef{"bev.top_y", ui->sliderBevTopY, ui->dispaly_Bev_Top_Y},
                        SliderDef{"bev.bot_y", ui->sliderBevBotY, ui->dispaly_Bev_Bot_Y},
                        SliderDef{"bev.top_w", ui->sliderBevTopW, ui->dispaly_Bev_Top_W},
                        SliderDef{"bev.bot_w", ui->sliderBevBotW, ui->dispaly_Bev_Bot_W}})
  {
    const QString name = d.name;
    QLabel *l = d.label;
    d.slider->setRange(0, 100);
    global_widgets_[name] = {d.slider, d.label, nullptr};

    connect(d.slider, &QSlider::valueChanged, this, [this, name, l](int v)
            {
      l->setNum(v);
      queueParam(name, v); });
  }

  // ====== 스핀박스 (morph → lineDetect_node, skel → path_node)
  struct SpinDef
  {
    const char *name;
    QSpinBox *spin;
    int min, max;
    bool odd;
  };
  for (const auto &d : {SpinDef{"morph.open", ui->spinMorphOpen, 1, 15, true},
                        SpinDef{"morph.close", ui->spinMorphClose, 1, 31, true},
                        SpinDef{"morph.min_area", ui->spinMinArea, 0, 5000, false},
                        SpinDef{"bev.lane_w_px", ui->spinSkelLaneW, 8, 320, false},
                        SpinDef{"skel.seal_len", ui->spinSkelSealLen, 0, 100, false},
                        SpinDef{"skel.prune_len", ui->spinSkelPruneLen, 0, 100, false},
                        SpinDef{"skel.dash_len", ui->spinSkelDashLen, 0, 160, false}})
  {
    const QString name = d.name;
    QSpinBox *sp = d.spin;
    const bool odd = d.odd;

    sp->setRange(d.min, d.max);
    sp->setSingleStep(odd ? 2 : 1);
    sp->setKeyboardTracking(false);
    global_widgets_[name] = {nullptr, nullptr, sp};

    connect(sp, qOverload<int>(&QSpinBox::valueChanged), this, [this, name, sp, odd](int v)
            {
      if (odd && v % 2 == 0)
      {
        v = std::min(v + 1, sp->maximum());
        QSignalBlocker block(sp);
        sp->setValue(v);
      }
      queueParam(name, v); });
  }

  setupWhiteModeBox();

  // 50ms마다 모아서 전송 (드래그 중 요청 폭주 방지)
  send_timer_.setSingleShot(true);
  send_timer_.setInterval(50);
  connect(&send_timer_, &QTimer::timeout, this, [this]
          {
    qnode->setParams(pending_);
    pending_.clear(); });

  // ====== HSV 대상 선택
  ui->radioBtnWhiteLine->setChecked(true);
  connect(ui->radioBtnWhiteLine, &QRadioButton::toggled, this, [this](bool checked)
          {
    if (!checked) return;
    current_target_ = "white";
    loadTargetToSliders(); });
  connect(ui->radioBtnYellowLine, &QRadioButton::toggled, this, [this](bool checked)
          {
    if (!checked) return;
    current_target_ = "yellow";
    loadTargetToSliders(); });
  connect(ui->radioBtn1, &QRadioButton::toggled, this, [this](bool checked)
          {
    if (!checked) return;
    current_target_ = "red";
    loadTargetToSliders(); });

  // ====== 노드에서 읽어온 값 반영
  connect(qnode, &QNode::paramLoaded, this, [this](const QString &name, int v)
          {
    if (pending_.contains(name))
      return;
    if (global_widgets_.contains(name))
    {
      applyGlobalParam(name, v);
      return;
    }
    if (name.startsWith("white.") || name.startsWith("yellow.") || name.startsWith("red."))
    {
      hsv_cache_[name] = v;
      if (name.startsWith(current_target_ + "."))
        loadTargetToSliders();
    } });

  // 노드가 늦게 떠도 연결되면 현재 값을 한 번 읽어옴
  param_poll_timer_.setInterval(500);
  connect(&param_poll_timer_, &QTimer::timeout, this, [this]
          {
    if (qnode->requestParams())
      param_poll_timer_.stop(); });
  param_poll_timer_.start();

  // 노드가 재시작되거나 다른 곳(ros2 param set, brain, launch)에서 값이 바뀌어도 2초 안에 화면에 반영
  // (입력 중인 스핀박스·끌고 있는 슬라이더·보내기 대기 중인 값은 덮어쓰지 않음)
  param_refresh_timer_.setInterval(2000);
  connect(&param_refresh_timer_, &QTimer::timeout, this, [this]
          { qnode->refreshParams(); });
  param_refresh_timer_.start();
  connect(qnode, &QNode::paramMessage, this, [this](const QString &t)
          { statusBar()->showMessage(t, 8000); });

  // ====== 상태 패널
  connect(qnode, &QNode::pathSourceReceived, this, &MainWindow::showPathSource);
  connect(qnode, &QNode::detectionsReceived, this, [this](const QString &t)
          { ui->labelDetections->setText("detections:\n" + t); });

  status_timeout_.setSingleShot(true);
  status_timeout_.setInterval(1000);
  connect(&status_timeout_, &QTimer::timeout, this, [this]
          { showPathSource(""); });

  // ====== 미션 (회전 방향, 주차)
  struct TurnDef
  {
    QRadioButton *btn;
    int turn;
  };
  for (const auto &d : {TurnDef{ui->radioTurnStraight, 0}, TurnDef{ui->radioTurnLeft, 1},
                        TurnDef{ui->radioTurnRight, 2}})
  {
    const int t = d.turn;
    connect(d.btn, &QRadioButton::toggled, this, [this, t](bool checked)
            {
      if (!checked) return;
      turn_ = t;
      qnode->publishTurn(t);
      showMission(); });
  }
  connect(ui->checkParking, &QCheckBox::toggled, this, [this](bool on)
          {
    parking_ = on;
    qnode->publishParking(on);
    showMission(); });

  // 다른 노드가 미션을 바꾸면 버튼 상태만 맞춤 (다시 발행하지 않음)
  connect(qnode, &QNode::turnReceived, this, [this](int t)
          {
    QRadioButton *b = t == 1 ? ui->radioTurnLeft : t == 2 ? ui->radioTurnRight : ui->radioTurnStraight;
    QSignalBlocker block(b);
    b->setChecked(true);
    turn_ = t;
    showMission(); });
  connect(qnode, &QNode::parkingReceived, this, [this](bool on)
          {
    QSignalBlocker block(ui->checkParking);
    ui->checkParking->setChecked(on);
    parking_ = on;
    showMission(); });
  showMission();

  // ====== 파라미터 저장 / 불러오기
  //  Save / Load      : 현재 파일에 바로 저장 / 현재 파일에서 불러오기
  //  Save as / Open   : 파일 선택 창으로 위치·이름을 정한 뒤 저장 / 불러오기 (현재 파일이 바뀜)
  //  마지막으로 쓴 파일은 QSettings에 기억 (~/.config/robit/tb_gui.conf)
  {
    QSettings s("robit", "tb_gui");
    param_file_ = s.value("param_file", QDir::homePath() + "/tb_params.yaml").toString();
  }
  ui->btnSaveParams->setToolTip("현재 파일에 저장");
  ui->btnLoadParams->setToolTip("현재 파일에서 불러오기");
  auto *btn_save_as = new QPushButton("Save as...", this);
  auto *btn_open = new QPushButton("Open...", this);
  btn_save_as->setToolTip("저장할 위치와 이름을 골라서 저장");
  btn_open->setToolTip("불러올 파일을 골라서 적용");
  ui->horizontalLayoutParams->addWidget(btn_save_as);
  ui->horizontalLayoutParams->addWidget(btn_open);
  ui->labelParamFile->setWordWrap(true);
  ui->labelParamFile->setText("params: " + param_file_);

  connect(ui->btnSaveParams, &QPushButton::clicked, this, [this]
          { saveParamsTo(param_file_); });
  connect(ui->btnLoadParams, &QPushButton::clicked, this, [this]
          { qnode->loadParams(param_file_); });
  connect(btn_save_as, &QPushButton::clicked, this, [this]
          {
    const QString f = QFileDialog::getSaveFileName(this, "파라미터 저장", param_file_,
                                                   "YAML (*.yaml *.yml);;All files (*)");
    if (f.isEmpty())
      return;
    setParamFile(f.endsWith(".yaml") || f.endsWith(".yml") ? f : f + ".yaml");
    saveParamsTo(param_file_); });
  connect(btn_open, &QPushButton::clicked, this, [this]
          {
    const QString f = QFileDialog::getOpenFileName(this, "파라미터 불러오기", QFileInfo(param_file_).absolutePath(),
                                                   "YAML (*.yaml *.yml);;All files (*)");
    if (f.isEmpty())
      return;
    setParamFile(f);
    qnode->loadParams(param_file_); });
  connect(qnode, &QNode::paramFileStatus, ui->labelParamFile, &QLabel::setText);

  // ======== 검출된 cv 객체 표시창 + reset버튼
  connect(qnode, &QNode::signStateChanged, this, [this](const QString &label, int state, const QString &lastTime)
          {
    QLabel *w = nullptr;
    QString name;
    if (label == "parking")
    {
      w = ui->signs_ParkingLabel;
      name = "주차";
    }
    else if (label == "left")
    {
      w = ui->signs_LeftLabel;
      name = "좌회전";
    }
    else if (label == "right")
    {
      w = ui->signs_RightLabel;
      name = "우회전";
    }
    else if (label == "construction")
    {
      w = ui->signs_ConstructionLabel;
      name = "공사장";
    }
    else if (label == "barrier")
    {
      w = ui->signs_BarrierStatusLabel;
      name = "차단바";
    }
    if (!w || state < 0 || state > 2)
      return;

    static const char *mark[] = {"X", "O", "△"};
    static const char *color[] = {"gray", "limegreen", "orange"};
    w->setText(QString("%1 : %2  (%3)").arg(name, mark[state], lastTime));
    w->setStyleSheet(QString("color: %1; font-weight: bold;").arg(color[state])); });

  connect(ui->signsResetButton, &QPushButton::clicked, this, [this]
          { qnode->detectionsReset(); });

  // ====== 주행 (WASD) + UART
  // 슬라이더/스핀박스에 포커스가 있어도 키를 받도록 앱 전체에 필터 설치
  qApp->installEventFilter(this);

  connect(ui->checkDriveEnable, &QCheckBox::toggled, this, [this](bool on)
          {
    if (!on)
      stopDrive();
    else
    {
      if (ui->checkAutoEnable->isChecked()) // 수동과 자동이 동시에 모터를 움직이지 않게
        setAutoDrive(false);
      setFocus();
    } });

  // 한계치/회전 비율을 바꾸면 누르고 있는 키 기준으로 바로 다시 계산
  connect(ui->spinMaxSpeed, qOverload<int>(&QSpinBox::valueChanged), this, [this](int)
          { updateDrive(); });
  connect(ui->spinTurnRatio, qOverload<int>(&QSpinBox::valueChanged), this, [this](int)
          { updateDrive(); });

  connect(ui->btnUartStart, &QPushButton::clicked, this, [this]
          { qnode->sendUart("start"); });
  connect(ui->btnUartQuit, &QPushButton::clicked, this, [this]
          {
    stopDrive(true);
    qnode->sendUart("quit"); });
  connect(ui->btnDriveStop, &QPushButton::clicked, this, [this]
          { stopDrive(true); });

  // 창이 포커스를 잃으면 KeyRelease를 못 받으므로 정지
  connect(qApp, &QGuiApplication::applicationStateChanged, this, [this](Qt::ApplicationState s)
          {
    if (s != Qt::ApplicationActive)
      stopDrive(); });

  connect(qnode, &QNode::uartReceived, this, &MainWindow::showUart);

  // ====== 자동 주행 (Tab 3)
  setupDriveTab();

  // ====== 거리·경로 파라미터 (Params 탭, 코드로 생성)
  setupParamTab();

  // ====== 아래 탭 페이지 스크롤 (창이 작아도 모든 항목에 닿게)
  makeTabsScrollable();

  // ====== View 탭 (카메라 크게 + 키보드 조작)
  setupViewTab();

  // ====== 영상 칸 ↔ 아래 탭 경계를 끌어서 높이 조절
  setupMainSplitter();
}

void MainWindow::setupMainSplitter()
{
  // .ui에서는 gridLayout_10 안에 영상 칸(gridLayout_7, 늘어남 10)과 아래 탭(tabWidget, 늘어남 0)이
  // 고정 비율로 들어 있어서 탭이 최소 높이만 받음 → 둘을 세로 QSplitter로 옮김
  QGridLayout *outer = ui->gridLayout_10;
  outer->removeItem(ui->gridLayout_7);
  outer->removeWidget(ui->tabWidget);

  auto *top = new QWidget;
  ui->gridLayout_7->setParent(nullptr); // 레이아웃은 부모가 없어야 다른 위젯에 붙일 수 있음
  top->setLayout(ui->gridLayout_7);
  video_area_ = top;

  main_split_ = new QSplitter(Qt::Vertical, ui->centralwidget);
  main_split_->addWidget(top);
  main_split_->addWidget(ui->tabWidget);
  main_split_->setChildrenCollapsible(false); // 끝까지 끌어도 한쪽이 사라지지 않게
  main_split_->setHandleWidth(8);
  main_split_->setStretchFactor(0, 3);        // 창 크기를 바꿀 때 영상 칸이 더 많이 늘어남
  main_split_->setStretchFactor(1, 2);
  outer->addWidget(main_split_, 0, 0);
  outer->setRowStretch(0, 1);
  outer->setRowStretch(1, 0);

  QSettings settings("tb_gui", "tb_gui");
  const QByteArray state = settings.value("main_split").toByteArray();
  if (!state.isEmpty())
    main_split_->restoreState(state);
  connect(main_split_, &QSplitter::splitterMoved, this, [this](int, int)
          { QSettings("tb_gui", "tb_gui").setValue("main_split", main_split_->saveState()); });

  // 아래 탭 이름 줄 오른쪽 끝: 영상 칸 숨기기/보이기 (켜 두면 다음 실행에도 유지)
  btn_hide_video_ = new QPushButton("영상 숨기기", ui->tabWidget);
  btn_hide_video_->setCheckable(true);
  btn_hide_video_->setFocusPolicy(Qt::NoFocus); // WASD·방향 키 입력을 뺏지 않게
  btn_hide_video_->setToolTip("위 영상 칸과 상태 상자를 숨기고 아래 탭(파라미터 등)을 창 전체로 씁니다.\n"
                              "View 탭에서는 영상이 필요해서 항상 보입니다.");
  ui->tabWidget->setCornerWidget(btn_hide_video_, Qt::TopRightCorner);
  btn_hide_video_->setChecked(settings.value("hide_video", false).toBool());
  connect(btn_hide_video_, &QPushButton::toggled, this, [this](bool on)
          {
    QSettings("tb_gui", "tb_gui").setValue("hide_video", on);
    updateVideoVisibility(); });
  updateVideoVisibility();
}

void MainWindow::updateVideoVisibility()
{
  if (!video_area_ || !btn_hide_video_)
    return;
  const bool hide = btn_hide_video_->isChecked() && !viewTabActive();
  video_area_->setVisible(!hide); // 숨긴 영상 라벨은 그리지 않음 (showStream이 보이는 것만 그림)
  btn_hide_video_->setText(btn_hide_video_->isChecked() ? "영상 보이기" : "영상 숨기기");
}

void MainWindow::makeTabsScrollable()
{
  QTabWidget *tw = ui->tabWidget;
  tw->setUsesScrollButtons(true); // 탭이 많으면 탭 이름 줄도 좌우로 넘김
  for (int i = 0; i < tw->count(); ++i)
  {
    QWidget *page = tw->widget(i);
    if (qobject_cast<QScrollArea *>(page) || qobject_cast<QTabWidget *>(page))
      continue; // Params 탭처럼 이미 스크롤 (하위 탭 페이지가 각자 스크롤)
    const QString title = tw->tabText(i);
    const QIcon icon = tw->tabIcon(i);
    tw->removeTab(i);
    auto *scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    page->setMinimumSize(page->minimumSizeHint()); // 줄어들면 찌그러지지 않고 스크롤
    scroll->setWidget(page);
    tw->insertTab(i, scroll, icon, title);
  }
}

// /lane/construction_info: "ON 표지판 brick=1 dist=0.42 sign_h>=0" / "off 끝: 끝내기 요청 ..."
void MainWindow::showConstructionInfo(const QString &info)
{
  if (!label_con_)
    return;
  const bool on = info.startsWith("ON");
  QString text;
  if (info.isEmpty())
    text = "공사 모드: (no data)";
  else if (on)
  {
    QString s = info.mid(3);
    s.replace("brick=1", "벽돌 봄").replace("brick=0", "벽돌 아직").replace("dist=", "진입 후 ");
    text = "공사 모드 ON  —  " + s;
  }
  else
    text = "공사 모드 off  —  " + info.mid(4);
  label_con_->setText(text);
  label_con_->setStyleSheet(QString("font-weight: bold; border-radius: 4px; padding: 3px; color: white; background: %1;")
                                .arg(info.isEmpty() ? "#555555" : on ? "#d35400" : "#7f8c8d"));
}

void MainWindow::resetAll()
{
  setBarrier(false);
  if (ui->radioTurnStraight->isChecked())
    qnode->publishTurn(0);
  else
    ui->radioTurnStraight->setChecked(true); // toggled에서 /mission/turn 0
  if (ui->checkParking->isChecked())
    ui->checkParking->setChecked(false); // toggled에서 /mission/parking false
  if (check_white_dash_ && check_white_dash_->isChecked())
  {
    QSignalBlocker block(check_white_dash_); // path_node는 reset에서 스스로 끔
    check_white_dash_->setChecked(false);
  }
  if (check_construction_->isChecked())
  {
    QSignalBlocker block(check_construction_);
    check_construction_->setChecked(false);
    construction_ = false;
  }
  qnode->publishReset(); // GUI가 보낸 turn·parking 뒤에 → 노드들이 초기화
  statusBar()->showMessage("초기화 보냄: path·drive·brain 상태를 처음처럼 (모드·파라미터는 그대로)", 4000);
  showMission();
}

void MainWindow::exitConstruction()
{
  // /mission/construction_exit (= false와 같음). 체크는 /lane/construction이 오면 맞춰짐
  if (check_construction_ && check_construction_->isChecked())
  {
    QSignalBlocker block(check_construction_);
    check_construction_->setChecked(false);
    construction_ = false;
  }
  qnode->publishConstructionExit();
  statusBar()->showMessage("공사 모드 끝내기 요청 보냄", 3000);
  showMission();
}

// ───────── View 탭 ─────────
// 이 탭을 고르면 아래 탭 영역이 탭 이름 줄만 남고 줄어들어, 위의 카메라 화면이 커짐.
// 이 탭에 있는 동안만 아래 키가 동작 (다른 탭에서 숫자 입력할 때 방해되지 않게)
//   Space : WASD ↔ 자동 주행 전환 (꺼져 있으면 WASD부터)      Esc : 모두 정지 (자동·WASD 끔)
//   WASD 모드 : W/A/S/D 수동 주행
//   자동 모드 : A/D 누르고 있는 동안 Left/Right, 떼면 Straight
//   P : 주차   C : 공사 구간   B : 차단바 정지 (켜 둔 동안 /drive/cmd STOP)
// 같은 조작은 상태 상자의 버튼·체크박스로도 할 수 있음
void MainWindow::setupViewTab()
{
  view_tab_ = new QWidget;
  auto *vl = new QVBoxLayout(view_tab_);
  vl->setContentsMargins(6, 2, 6, 2);
  auto *hint = new QLabel("View 탭: 키보드 조작 켜짐 (Space 모드 전환, Esc 정지 — 위 상태 상자 참고)", view_tab_);
  hint->setStyleSheet("color: gray;");
  vl->addWidget(hint);
  ui->tabWidget->insertTab(0, view_tab_, "View");

  // 상태 상자 맨 위에: 모드 배지, 모드 버튼, 미션 체크박스, 주행 요약, 키 안내
  auto *box = ui->verticalLayoutStatus;
  label_mode_ = new QLabel(ui->groupStatus);
  label_mode_->setAlignment(Qt::AlignCenter);
  box->insertWidget(0, label_mode_);

  auto *row = new QHBoxLayout;
  auto *btn_off = new QPushButton("정지 (Esc)", ui->groupStatus);
  auto *btn_wasd = new QPushButton("WASD", ui->groupStatus);
  auto *btn_auto = new QPushButton("자동", ui->groupStatus);
  auto *btn_brain = new QPushButton("Brain (Enter)", ui->groupStatus);
  btn_auto->setToolTip("drive.enable + require_cmd 끔: 명령 없이도 바로 경로 추종 (FOLLOW)");
  btn_brain->setToolTip("drive.enable + require_cmd 켬 + /brain/run start:\n"
                        "brain이 /drive/cmd(FOLLOW)를 줄 때만 움직임 (예: 초록불 보기 전까지 정지 stop:no_cmd)");
  for (QPushButton *b : {btn_off, btn_wasd, btn_auto, btn_brain})
  {
    b->setFocusPolicy(Qt::NoFocus); // 버튼이 Space를 먹지 않게
    row->addWidget(b);
  }
  box->insertLayout(1, row);
  connect(btn_off, &QPushButton::clicked, this, [this] { setDriveMode(DriveMode::Off); });
  connect(btn_wasd, &QPushButton::clicked, this, [this] { setDriveMode(DriveMode::Wasd); });
  connect(btn_auto, &QPushButton::clicked, this, [this] { setDriveMode(DriveMode::Auto); });
  connect(btn_brain, &QPushButton::clicked, this, [this] { setDriveMode(DriveMode::Brain); });

  label_drive_brief_ = new QLabel("drive: -", ui->groupStatus);
  label_drive_brief_->setWordWrap(true); // 긴 /drive/state 한 줄 때문에 상태 상자가 넓어지지 않게
  box->insertWidget(2, label_drive_brief_);

  // Brain 모드 동안 1초마다 /brain/run "start" 다시 보냄 (brain이 GUI보다 늦게 떠도 받게)
  // GUI가 켜질 때는 아무것도 안 보냄 (CLI로 켠 brain을 GUI가 켜지면서 끄지 않게)
  brain_timer_.setInterval(1000);
  connect(&brain_timer_, &QTimer::timeout, this, [this] { qnode->publishBrainRun(true); });

  // 기존 주차 체크박스 옆에 공사 구간, 차단바
  check_construction_ = new QCheckBox("공사 구간 (C)", ui->groupStatus);
  check_barrier_ = new QCheckBox("차단바 정지 (B)", ui->groupStatus);
  ui->checkParking->setText("Parking (P)");
  const int park_idx = box->indexOf(ui->checkParking);
  box->insertWidget(park_idx + 1, check_construction_);
  box->insertWidget(park_idx + 2, check_barrier_);
  check_white_dash_ = new QCheckBox("흰 점선 잇기", ui->groupStatus);
  check_white_dash_->setToolTip("체크 = path_node가 짧은 흰 조각(점선)을 이어서 실선으로 봄 (/mission/white_dash).\n"
                                "brain은 주차 시작부터 벽돌 옆에서 멈출 때까지 켬 (여기에도 그대로 보임).\n"
                                "조각·빈칸 기준은 Params 탭 스켈레톤의 skel.wdash_*");
  box->insertWidget(park_idx + 3, check_white_dash_);
  connect(check_white_dash_, &QCheckBox::toggled, this, [this](bool on)
          { qnode->publishWhiteDash(on); });
  connect(qnode, &QNode::whiteDashReceived, this, [this](bool on)
          {
    if (check_white_dash_->isChecked() == on)
      return;
    QSignalBlocker block(check_white_dash_); // brain이 바꾼 값: 화면만 맞추고 다시 보내지 않음
    check_white_dash_->setChecked(on); });
  for (QWidget *w : std::initializer_list<QWidget *>{check_construction_, check_barrier_, check_white_dash_, ui->checkParking,
                                                     ui->radioTurnLeft, ui->radioTurnRight, ui->radioTurnStraight})
    w->setFocusPolicy(Qt::NoFocus);

  // 공사 모드 상태 배지 + 끝내기 버튼 (표지판으로 자동 진입한 것도 끝낼 수 있음)
  label_con_ = new QLabel(ui->groupStatus);
  label_con_->setAlignment(Qt::AlignCenter);
  label_con_->setWordWrap(true);
  auto *btn_con_exit = new QPushButton("공사 끝내기 (X)", ui->groupStatus);
  btn_con_exit->setFocusPolicy(Qt::NoFocus);
  btn_con_exit->setToolTip("/mission/construction_exit: 수동·표지판 진입 모두 끔. 표지판이 3초 넘게 안 보여야 다시 자동 진입");
  box->insertWidget(park_idx + 3, label_con_);
  box->insertWidget(park_idx + 4, btn_con_exit);

  // 전체 초기화: path(기억 경로·공사·정지선), drive(추종·피봇 상태), brain(미션 상태) + GUI 미션 표시
  auto *btn_reset = new QPushButton("초기화 (R)", ui->groupStatus);
  btn_reset->setFocusPolicy(Qt::NoFocus);
  btn_reset->setToolTip("/mission/reset: path_node 경로 기억·공사·정지선, drive_node 추종 상태, brain 미션 상태를 처음처럼.\n"
                        "주행 모드·파라미터는 그대로 (로봇을 출발점에 다시 놓고 누르기)");
  box->insertWidget(park_idx + 5, btn_reset);
  connect(btn_reset, &QPushButton::clicked, this, [this] { resetAll(); });

  // 가짜 표지판 → tb_brain (/brain/inject). 필터 없이 바로 그 표지판 동작 (Brain 모드에서만 의미 있음)
  //   중간에서 시작할 때 초록불로 출발, 표지판을 놓쳤을 때 대신 보내기 등
  auto *inject_box = new QGroupBox("가짜 표지판 → brain", ui->groupStatus);
  auto *inject_grid = new QGridLayout(inject_box);
  inject_grid->setContentsMargins(4, 4, 4, 4);
  inject_grid->setSpacing(3);
  static const std::pair<const char *, const char *> fake_signs[] = {
      {"tl_green", "초록불 (출발)"}, {"left", "좌회전"}, {"right", "우회전"},
      {"construction", "공사"}, {"parking", "주차"}, {"barrier_closed", "차단바 닫힘"},
      {"barrier_open", "차단바 열림"}, {"stop_line", "정지선 켜기"}};
  int fi = 0;
  for (const auto &[label, text] : fake_signs)
  {
    auto *b = new QPushButton(text, inject_box);
    b->setFocusPolicy(Qt::NoFocus);
    b->setToolTip(QString("/brain/inject \"%1\"").arg(label));
    const QString l = label;
    connect(b, &QPushButton::clicked, this, [this, l]
            {
      qnode->publishBrainInject(l);
      statusBar()->showMessage(driveMode() == DriveMode::Brain
                                   ? "가짜 표지판 보냄: " + l
                                   : "가짜 표지판 보냄: " + l + " (Brain 모드가 아니면 brain이 무시함, Enter로 Brain 켜기)",
                               4000); });
    inject_grid->addWidget(b, fi / 3, fi % 3);
    ++fi;
  }
  box->insertWidget(park_idx + 6, inject_box);
  connect(btn_con_exit, &QPushButton::clicked, this, [this] { exitConstruction(); });
  connect(qnode, &QNode::constructionInfoReceived, this, &MainWindow::showConstructionInfo);
  showConstructionInfo("");

  check_construction_->setToolTip("체크 = 공사 모드 진입, 해제 = 끝 (/mission/construction).\n"
                                  "tb_brain이 표지판으로 켠 것, 벽돌로 자동 종료된 것도 여기 그대로 보임 (/lane/construction)");
  connect(check_construction_, &QCheckBox::toggled, this, [this](bool on)
          {
    construction_ = on;
    qnode->publishConstruction(on);
    showMission(); });
  // 체크 상태 = path_node의 실제 공사 모드 (brain 진입, 벽돌로 자동 종료 모두 반영)
  connect(qnode, &QNode::constructionStateReceived, this, [this](bool on)
          {
    construction_state_ = on;
    construction_ = on;
    if (check_construction_->isChecked() != on)
    {
      QSignalBlocker block(check_construction_);
      check_construction_->setChecked(on);
    }
    showMission(); });
  connect(check_barrier_, &QCheckBox::toggled, this, [this](bool on) { setBarrier(on); });
  barrier_timer_.setInterval(100); // drive.cmd_timeout_ms(500)보다 충분히 짧게
  connect(&barrier_timer_, &QTimer::timeout, this, [this] { qnode->publishDriveStop(); });

  label_keys_ = new QLabel(ui->groupStatus);
  label_keys_->setWordWrap(true);
  label_keys_->setStyleSheet("color: gray; font-size: 11px;");
  box->addWidget(label_keys_);

  // 모드가 바뀌는 모든 경로(버튼, 키, 다른 탭의 체크박스)에서 표시 갱신
  connect(ui->checkDriveEnable, &QCheckBox::toggled, this, [this](bool) { refreshViewStatus(); });
  connect(ui->checkAutoEnable, &QCheckBox::toggled, this, [this](bool) { refreshViewStatus(); });
  connect(qnode, &QNode::boolParamLoaded, this, [this](const QString &n, bool)
          { if (n == "drive.enable") refreshViewStatus(); });
  connect(qnode, &QNode::driveStateReceived, this, [this](const QString &t)
          { label_drive_brief_->setText(t.isEmpty() ? "drive: -" : t); });

  connect(ui->tabWidget, &QTabWidget::currentChanged, this, [this](int)
          {
    applyViewTabSize();
    refreshViewStatus();
    if (viewTabActive() && display_tabs_) // 아래 View 탭(키보드 주행)으로 오면 위 영상도 주행 탭으로
      display_tabs_->setCurrentIndex(0);
    if (!viewTabActive() && !turn_keys_.isEmpty()) // 탭을 떠나면 누르고 있던 A/D는 놓은 것으로
    {
      turn_keys_.clear();
      ui->radioTurnStraight->setChecked(true);
    } });

  ui->tabWidget->setCurrentWidget(view_tab_);
  applyViewTabSize();
  refreshViewStatus();
  showMission();
}

bool MainWindow::viewTabActive() const
{
  return view_tab_ && ui->tabWidget->currentWidget() == view_tab_;
}

void MainWindow::applyViewTabSize()
{
  updateVideoVisibility(); // View 탭으로 오면 영상 칸을 다시 보임, 다른 탭으로 가면 숨김 설정대로
  // QTabWidget은 가장 큰 페이지 크기를 차지하므로, View 탭일 때는 높이를 직접 줄임
  if (viewTabActive())
    ui->tabWidget->setMaximumHeight(ui->tabWidget->tabBar()->sizeHint().height() + 34);
  else
    ui->tabWidget->setMaximumHeight(QWIDGETSIZE_MAX);
}

MainWindow::DriveMode MainWindow::driveMode() const
{
  if (ui->checkAutoEnable->isChecked())
    return brain_run_ ? DriveMode::Brain : DriveMode::Auto;
  if (ui->checkDriveEnable->isChecked())
    return DriveMode::Wasd;
  return DriveMode::Off;
}

void MainWindow::setDriveMode(DriveMode m)
{
  // 기존 체크박스를 그대로 써서 Drive 탭과 항상 같은 상태 (서로 끄는 처리도 거기 있음)
  if (m != DriveMode::Auto && !turn_keys_.isEmpty())
  {
    turn_keys_.clear();
    ui->radioTurnStraight->setChecked(true);
  }
  if (m != DriveMode::Brain && brain_run_)
    setBrainRun(false); // brain 먼저 끔 (/mission/turn 0, /mission/pause false도 보냄)
  switch (m)
  {
  case DriveMode::Off:
    setAutoDrive(false); // 체크가 꺼져 보여도 항상 보냄 (화면과 노드가 어긋나 있어도 확실히 멈추게)
    ui->checkDriveEnable->setChecked(false); // stopDrive 호출됨
    stopDrive(true);
    break;
  case DriveMode::Wasd:
    if (ui->checkAutoEnable->isChecked())
      setAutoDrive(false);
    ui->checkDriveEnable->setChecked(true);
    break;
  case DriveMode::Auto:
    ui->checkDriveEnable->setChecked(false);
    setRequireCmd(false); // 명령 없어도 FOLLOW (enable보다 먼저)
    setAutoDrive(true);
    break;
  case DriveMode::Brain:
    ui->checkDriveEnable->setChecked(false);
    setRequireCmd(true); // brain 명령이 없으면 정지 (enable보다 먼저 → 켜지는 순간 FOLLOW로 튀지 않게)
    setBrainRun(true);
    setAutoDrive(true);
    if (!ui->checkAutoEnable->isChecked()) // drive_node 연결 안 됨
      setBrainRun(false);
    break;
  }
  refreshViewStatus();
}

void MainWindow::refreshViewStatus()
{
  if (!label_mode_)
    return;
  static const char *text[] = {"정지", "WASD 수동", "자동 주행", "Brain (대회)"};
  static const char *color[] = {"#555555", "#2980b9", "#2e9d4a", "#8e44ad"};
  const int i = static_cast<int>(driveMode());
  label_mode_->setText(QString("%1%2").arg(text[i], viewTabActive() ? "" : "   (키 조작은 View 탭에서)"));
  label_mode_->setStyleSheet(QString("font-size: 18px; font-weight: bold; border-radius: 4px; color: white; "
                                     "padding: 4px; background: %1;").arg(color[i]));
  const DriveMode dm = driveMode();
  label_keys_->setText(dm == DriveMode::Auto
                           ? "Space: WASD로   A/D 누르는 동안: Left/Right   P 주차  C 공사 켜기  X 공사 끝  B 차단바  R 초기화   Enter: Brain   Esc 정지"
                       : dm == DriveMode::Brain
                           ? "Brain이 /drive/cmd·/mission/* 를 냄 (수동 조작 자제)   Space: WASD로   B 차단바  R 초기화   Enter·Esc: 정지"
                           : "Space: 자동으로   W/A/S/D: 수동 주행   P 주차  C 공사 켜기  X 공사 끝  B 차단바  R 초기화   Enter: Brain   Esc 정지");
}

void MainWindow::setTurnMomentary(int key, bool press)
{
  turn_keys_.removeAll(key);
  if (press)
    turn_keys_.append(key);
  const int k = turn_keys_.isEmpty() ? 0 : turn_keys_.last();
  QRadioButton *b = k == Qt::Key_A ? ui->radioTurnLeft : k == Qt::Key_D ? ui->radioTurnRight : ui->radioTurnStraight;
  b->setChecked(true); // 라디오 버튼의 toggled에서 /mission/turn 발행
}

void MainWindow::setBrainRun(bool on)
{
  const bool changed = on != brain_run_;
  brain_run_ = on;
  qnode->publishBrainRun(on);
  if (on)
    brain_timer_.start();
  else
  {
    brain_timer_.stop();
    if (changed)
    {
      // brain이 멈추면 마지막으로 보낸 값이 그대로 남으므로 풀어 줌
      ui->radioTurnStraight->setChecked(true); // toggled에서 /mission/turn 0
      qnode->publishTurn(0);
      qnode->publishPause(false);
    }
  }
  refreshViewStatus();
}

void MainWindow::setRequireCmd(bool on)
{
  if (ui->checkRequireCmd->isChecked() != on)
    ui->checkRequireCmd->setChecked(on); // toggled에서 drive.require_cmd 설정
  else
    qnode->setBoolParam("drive.require_cmd", on); // 화면과 노드가 어긋나 있을 수도 있으니 한 번 더
}

void MainWindow::setBarrier(bool on)
{
  if (on)
  {
    qnode->publishDriveStop();
    barrier_timer_.start();
  }
  else
    barrier_timer_.stop(); // 0.5초(drive.cmd_timeout_ms) 뒤 원래대로 FOLLOW
  if (check_barrier_->isChecked() != on)
  {
    QSignalBlocker block(check_barrier_);
    check_barrier_->setChecked(on);
  }
  showMission();
}

// View 탭에서의 키. 처리했으면 true (다른 위젯으로 안 넘김)
bool MainWindow::handleViewKey(QKeyEvent *ke, bool press)
{
  const int k = ke->key();
  const bool repeat = ke->isAutoRepeat();
  const DriveMode m = driveMode();

  switch (k)
  {
  case Qt::Key_Space:
    if (press && !repeat)
      setDriveMode(m == DriveMode::Wasd ? DriveMode::Auto : DriveMode::Wasd);
    return true;
  case Qt::Key_Escape:
    if (press && !repeat)
    {
      setDriveMode(DriveMode::Off); // brain도 끔
      setBarrier(false);
    }
    return true;
  case Qt::Key_Return:
  case Qt::Key_Enter: // 키패드 Enter: Brain ↔ 정지
    if (press && !repeat)
      setDriveMode(m == DriveMode::Brain ? DriveMode::Off : DriveMode::Brain);
    return true;
  case Qt::Key_P:
    if (press && !repeat)
      ui->checkParking->toggle();
    return true;
  case Qt::Key_C:
    if (press && !repeat)
      check_construction_->toggle();
    return true;
  case Qt::Key_X:
    if (press && !repeat)
      exitConstruction();
    return true;
  case Qt::Key_B:
    if (press && !repeat)
      setBarrier(!check_barrier_->isChecked());
    return true;
  case Qt::Key_R:
    if (press && !repeat)
      resetAll();
    return true;
  case Qt::Key_A:
  case Qt::Key_D:
    if (m == DriveMode::Auto)
    {
      if (!repeat)
        setTurnMomentary(k, press);
      return true;
    }
    return false; // WASD 모드면 아래 기존 처리로
  default:
    return false;
  }
}

// ───────── 파라미터 스핀박스 공통 ─────────

// ───────── 파라미터 칸 공통 ─────────

// 스핀박스 하나 만들고 노드와 연결 (위치는 부르는 쪽이 정함)
QSpinBox *MainWindow::createParamSpin(QWidget *parent, const ParamSpinDef &d)
{
  const QString name = d.name;
  auto *sp = new QSpinBox(parent);
  sp->setRange(d.min, d.max);
  sp->setSingleStep(d.step);
  sp->setSuffix(d.suffix);
  sp->setKeyboardTracking(false);
  sp->setToolTip(name);
  global_widgets_[name] = {nullptr, nullptr, sp}; // paramLoaded → applyGlobalParam 로 동기화
  // 숫자를 쳤는데 아직 안 보낸 상태(Enter·다른 곳 클릭 전)면 노란색으로 표시
  // (입력 중인 칸은 노드 값으로 덮어쓰지 않고, 보낸 뒤에는 포커스가 남아 있어도 노드 값을 따라감)
  sp->setProperty("editing", false);
  auto done = [sp]
  {
    sp->setProperty("editing", false);
    sp->setStyleSheet("");
  };
  connect(sp, &QSpinBox::textChanged, this, [sp](const QString &)
          {
    sp->setProperty("editing", true);
    sp->setStyleSheet("QSpinBox { background: #fff3a0; color: black; }"); });
  connect(sp, qOverload<int>(&QSpinBox::valueChanged), this, [this, name, done](int v)
          {
    done();
    queueParam(name, v); });
  connect(sp, &QSpinBox::editingFinished, this, done);
  return sp;
}

// (이름 라벨, 스핀박스) 한 쌍을 grid의 idx번째 칸에 넣고 노드와 연결
void MainWindow::addParamSpin(QGridLayout *grid, QWidget *parent, int idx, int cols, const ParamSpinDef &d)
{
  auto *label = new QLabel(d.text, parent);
  label->setToolTip(d.name);
  grid->addWidget(label, idx / cols, (idx % cols) * 2);
  grid->addWidget(createParamSpin(parent, d), idx / cols, (idx % cols) * 2 + 1);
}

// 실수 파라미터 (YOLO conf·nms·max_hz). 노드에 바로 보냄 (정수 칸처럼 50ms 묶음 없음)
void MainWindow::addParamDouble(QGridLayout *grid, QWidget *parent, int idx, int cols, const ParamDoubleDef &d)
{
  const QString name = d.name;
  auto *label = new QLabel(d.text, parent);
  label->setToolTip(name);
  auto *sp = new QDoubleSpinBox(parent);
  sp->setDecimals(d.decimals);
  sp->setRange(d.min, d.max);
  sp->setSingleStep(d.step);
  sp->setSuffix(d.suffix);
  sp->setKeyboardTracking(false);
  sp->setToolTip(name);
  sp->setProperty("editing", false);
  grid->addWidget(label, idx / cols, (idx % cols) * 2);
  grid->addWidget(sp, idx / cols, (idx % cols) * 2 + 1);
  double_widgets_[name] = sp;
  connect(sp, &QDoubleSpinBox::textChanged, this, [sp](const QString &)
          {
    sp->setProperty("editing", true);
    sp->setStyleSheet("QDoubleSpinBox { background: #fff3a0; color: black; }"); });
  connect(sp, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this, name, sp](double v)
          {
    sp->setProperty("editing", false);
    sp->setStyleSheet("");
    qnode->setDoubleParam(name, v); });
  connect(sp, &QDoubleSpinBox::editingFinished, this, [sp]
          {
    sp->setProperty("editing", false);
    sp->setStyleSheet(""); });
}

// 묶음 상자: 제목 + (설명 한 줄) + 칸들을 cols쌍씩 한 줄
QGroupBox *MainWindow::makeParamGroup(QWidget *parent, const ParamGroup &g)
{
  auto *box = new QGroupBox(g.title, parent);
  auto *grid = new QGridLayout(box);
  int row0 = 0;
  if (g.note && *g.note)
  {
    auto *note = new QLabel(g.note, box);
    note->setWordWrap(true);
    note->setStyleSheet("color: gray;");
    grid->addWidget(note, 0, 0, 1, g.cols * 2);
    row0 = 1;
  }
  auto *inner = new QGridLayout;
  int i = 0;
  for (const auto &d : g.defs)
    addParamSpin(inner, box, i++, g.cols, d);
  inner->setColumnStretch(g.cols * 2, 1);
  grid->addLayout(inner, row0, 0, 1, g.cols * 2);
  return box;
}

// 묶음들을 세로로 쌓은 스크롤 페이지
QWidget *MainWindow::makeParamPage(const std::vector<QWidget *> &boxes, const QString &hint)
{
  auto *page = new QWidget;
  auto *vbox = new QVBoxLayout(page);
  for (QWidget *b : boxes)
  {
    b->setParent(page);
    vbox->addWidget(b);
  }
  if (!hint.isEmpty())
  {
    auto *h = new QLabel(hint, page);
    h->setWordWrap(true);
    h->setStyleSheet("color: gray;");
    vbox->addWidget(h);
  }
  vbox->addStretch(1);
  auto *scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setWidget(page);
  return scroll;
}

// 표지판별 필터: 행 = 표지판, 열 = 최소 높이·너비·면적·연속 프레임
QGroupBox *MainWindow::makeSignFilterTable(QWidget *parent)
{
  auto *box = new QGroupBox("표지판별 필터 (brain_node) — 원본 640×480 기준. 0 = 그 조건 안 봄", parent);
  auto *g = new QGridLayout(box);
  const char *cols[] = {"최소 높이", "최소 너비", "최소 면적 (w×h)", "연속 프레임 (0=공통)"};
  for (int c = 0; c < 4; ++c)
  {
    auto *h = new QLabel(QString("<b>%1</b>").arg(cols[c]), box);
    h->setAlignment(Qt::AlignCenter);
    g->addWidget(h, 0, c + 1);
  }
  struct Row
  {
    const char *key, *text;
  };
  // 트랙 순서 (신호등 → 차단바 → 좌/우 → 공사 → 주차)
  const Row rows[] = {{"tl_green", "초록불"},        {"barrier_closed", "차단바 닫힘"}, {"barrier_open", "차단바 열림"},
                      {"left", "좌회전"},           {"right", "우회전"},             {"construction", "공사"},
                      {"parking", "주차"}};
  int r = 1;
  for (const Row &row : rows)
  {
    g->addWidget(new QLabel(row.text, box), r, 0);
    const std::string k = std::string("sign.") + row.key;
    // createParamSpin이 이름을 바로 QString으로 복사하므로 지역 문자열로 충분
    const std::string n[4] = {k + ".min_h_px", k + ".min_w_px", k + ".min_area_px", k + ".confirm_frames"};
    const ParamSpinDef ds[4] = {{n[0].c_str(), "", " px", 0, 480, 5},
                                {n[1].c_str(), "", " px", 0, 640, 5},
                                {n[2].c_str(), "", " px²", 0, 307200, 500},
                                {n[3].c_str(), "", "", 0, 30, 1}};
    for (int c = 0; c < 4; ++c)
      g->addWidget(createParamSpin(box, ds[c]), r, c + 1);
    ++r;
  }
  g->setColumnStretch(5, 1);
  return box;
}

// YOLO (signs_node): 실수 3개 + 디버그 영상 체크
QGroupBox *MainWindow::makeYoloBox(QWidget *parent)
{
  auto *box = new QGroupBox("YOLO 검출 (signs_node)", parent);
  auto *g = new QGridLayout(box);
  const ParamDoubleDef defs[] = {
      {"max_hz", "초당 최대 추론 (0=프레임마다)", " Hz", 0.0, 30.0, 1.0, 1},
      {"conf", "검출 최소 점수", "", 0.0, 1.0, 0.05, 2},
      {"nms", "겹친 박스 합치기 (NMS IoU)", "", 0.0, 1.0, 0.05, 2},
  };
  int i = 0;
  for (const auto &d : defs)
    addParamDouble(g, box, i++, 3, d);
  check_yolo_debug_ = new QCheckBox("디버그 영상 발행 (publish_debug, 보는 사람 있을 때만 그림)", box);
  check_yolo_debug_->setToolTip("publish_debug");
  g->addWidget(check_yolo_debug_, 1, 0, 1, 6);
  bool_widgets_["publish_debug"] = check_yolo_debug_; // boolParamLoaded로 동기화
  connect(check_yolo_debug_, &QCheckBox::toggled, this, [this](bool on)
          {
    if (!qnode->setBoolParam("publish_debug", on))
    {
      QSignalBlocker block(check_yolo_debug_);
      check_yolo_debug_->setChecked(!on);
      statusBar()->showMessage("signs_node 연결 안 됨 (signs:=false로 띄웠거나 아직 안 뜸)", 5000);
    } });
  auto *note = new QLabel("max_hz를 낮추면 GPU·CPU 사용이 줄고, 표지판 인정(연속 프레임)은 그만큼 늦어집니다. "
                          "brain의 '최소 점수'는 YOLO 검출 최소 점수 뒤에 한 번 더 거르는 값입니다.",
                          box);
  note->setWordWrap(true);
  note->setStyleSheet("color: gray;");
  g->addWidget(note, 2, 0, 1, 6);
  g->setColumnStretch(6, 1);
  return box;
}

// odom 보정 (tb_uart_node): 바퀴 값·배율을 실행 중에 바꾸고, 멈춘 상태에서 0 → 밀거나 몰고 → 실제 거리로 배율 계산
QGroupBox *MainWindow::makeOdomBox(QWidget *parent)
{
  auto *box = new QGroupBox("odom 보정 (tb_uart_node)", parent);
  auto *g = new QGridLayout(box);
  const ParamDoubleDef defs[] = {
      {"odom.rpm_per_unit", "배율 (rpm/원시값)", "", 0.001, 5.0, 0.005, 4},
      {"odom.wheel_radius_m", "바퀴 반지름", " m", 0.01, 0.20, 0.0005, 4},
      {"odom.wheel_sep_m", "바퀴 간격 (직진·곡선)", " m", 0.05, 0.50, 0.001, 4},
      {"odom.pivot_sep_m", "바퀴 간격 (제자리 회전)", " m", 0.05, 0.50, 0.001, 4},
  };
  int i = 0;
  for (const auto &d : defs)
    addParamDouble(g, box, i++, 1, d);
  addParamSpin(g, box, i++, 1, {"odom.max_gap_ms", "피드백 끊김 허용", " ms", 1, 1000, 10});

  // 지금 odom 위치 + 0으로
  auto *pos = new QLabel("odom: -", box);
  pos->setStyleSheet("font-family: monospace;");
  g->addWidget(pos, i, 0, 1, 2);
  ++i;
  auto *row = new QHBoxLayout;
  auto *btn_zero = new QPushButton("odom 0으로", box);
  btn_zero->setToolTip("/odom/reset. 멈춘 상태에서만 (주행 중에 누르면 path_node 기억 경로가 튐)");
  btn_zero->setFocusPolicy(Qt::NoFocus);
  auto *real = new QDoubleSpinBox(box);
  real->setRange(0.05, 10.0);
  real->setDecimals(3);
  real->setSingleStep(0.05);
  real->setValue(1.0);
  real->setSuffix(" m");
  real->setToolTip("0으로 한 뒤 자로 잰 실제 직진 거리");
  auto *btn_fit = new QPushButton("실제 거리로 배율 맞추기", box);
  btn_fit->setToolTip("새 배율 = 지금 배율 × 실제 거리 ÷ odom 거리 (0으로 한 뒤 직진만 했을 때)");
  btn_fit->setFocusPolicy(Qt::NoFocus);
  row->addWidget(btn_zero);
  row->addWidget(new QLabel("실제", box));
  row->addWidget(real);
  row->addWidget(btn_fit, 1);
  g->addLayout(row, i, 0, 1, 2);
  ++i;
  auto *note = new QLabel("① 멈춘 상태에서 'odom 0으로' ② 자로 잰 거리만큼 직진 ③ 실제 거리 입력 → 배율 맞추기. "
                          "회전이 틀리면 제자리 360° 후 '제자리 회전 간격'을 조정. 값은 Save로 params 파일에 저장됨",
                          box);
  note->setWordWrap(true);
  note->setStyleSheet("color: gray;");
  g->addWidget(note, i, 0, 1, 2);

  auto odom_xy = std::make_shared<QPointF>(0.0, 0.0);
  connect(qnode, &QNode::odomReceived, pos, [pos, odom_xy](double x, double y, double th)
          {
    *odom_xy = QPointF(x, y);
    pos->setText(QString("odom: x %1  y %2 m  거리 %3 m  θ %4°")
                     .arg(x, 7, 'f', 3).arg(y, 7, 'f', 3).arg(std::hypot(x, y), 6, 'f', 3).arg(th * 180.0 / M_PI, 6, 'f', 1)); });
  connect(btn_zero, &QPushButton::clicked, this, [this]
          {
    qnode->publishOdomReset();
    statusBar()->showMessage("odom 0으로 (/odom/reset)", 3000); });
  connect(btn_fit, &QPushButton::clicked, this, [this, real, odom_xy]
          {
    const double d = std::hypot(odom_xy->x(), odom_xy->y());
    QDoubleSpinBox *sp = double_widgets_.value("odom.rpm_per_unit", nullptr);
    if (!sp || d < 0.05)
    {
      statusBar()->showMessage("odom 거리가 5cm보다 짧음 (0으로 한 뒤 직진했는지, tb_uart_node가 떠 있는지 확인)", 5000);
      return;
    }
    const double now = sp->value(), next = now * real->value() / d;
    sp->setValue(next); // valueChanged → 노드에 보냄
    statusBar()->showMessage(QString("배율 %1 → %2 (odom %3 m, 실제 %4 m). 다시 0으로 하고 확인하세요")
                                 .arg(now, 0, 'f', 4).arg(next, 0, 'f', 4).arg(d, 0, 'f', 3).arg(real->value(), 0, 'f', 3), 8000); });
  return box;
}

// ───────── Params 탭: 노드·주제별 하위 탭 ─────────
// Tab 1(차선 색·BEV)에 이미 있는 값(HSV, BEV 사다리꼴, morph, bev.lane_w_px, skel.seal/prune/dash)은 넣지 않음
// (한 파라미터에 위젯이 둘이면 동기화가 꼬임)

void MainWindow::setupParamTab()
{
  // 아래 탭 이름 (makeTabsScrollable이 이름을 옮겨 담기 전에)
  ui->tabWidget->setTabText(ui->tabWidget->indexOf(ui->tab), "차선 색·BEV");
  ui->tabWidget->setTabText(ui->tabWidget->indexOf(ui->tab_2), "센서·WASD");
  ui->tabWidget->setTabText(ui->tabWidget->indexOf(ui->tab_3), "주행 (drive)");

  auto *tabs = new QTabWidget;
  tabs->setDocumentMode(true);
  auto group = [this](const ParamGroup &g) -> QWidget * { return makeParamGroup(nullptr, g); };

  // ====== 1. 차선·경로 (lineDetect, path_node)
  tabs->addTab(makeParamPage({
      group({"실측 거리 (lineDetect_node) ★ 자로 잰 값",
             {
                 {"geo.lane_w_mm", "차선 안쪽 간격", " mm", 1, 1000, 1},
                 {"geo.bev_bottom_mm", "BEV 아랫줄 거리", " mm", 0, 2000, 1},
                 {"geo.bev_depth_mm", "BEV 세로 길이", " mm", 1, 3000, 1},
             }}),
      group({"스켈레톤 (path_node)",
             {
                 {"skel.max_lane_thick", "차선 최대 두께", " px", 0, 200, 1},
             },
             3, "끊긴 선 잇기·잔가지·노란 점선 길이는 '차선 색·BEV' 탭"}),
      group({"흰 점선 잇기 (path_node) — /mission/white_dash 켜져 있을 때만 (brain: 주차 시작 ~ 벽돌 옆 정지)",
             {
                 {"skel.wdash_max_mm", "조각 최대 길이 (0=끔)", " mm", 0, 500, 5},
                 {"skel.wdash_gap_mm", "이을 최대 빈칸", " mm", 0, 500, 5},
                 {"skel.wdash_angle_deg", "같은 줄 각도", " °", 0, 90, 1},
                 {"skel.wdash_offset_mm", "같은 줄 옆 어긋남", " mm", 0, 200, 5},
             },
             2}),
      group({"경로 기억 (path_node) — 화면이 애매할 때 직전 경로로 버티기",
             {
                 {"path.max_dev_mm", "기억과 차이 허용", " mm", 0, 500, 5},
                 {"path.override_frames", "새 경로 믿기까지", " 프레임", 0, 100, 1},
                 {"path.mem_travel_mm", "기억만으로 최대 이동", " mm", 0, 3000, 50},
                 {"path.min_len_mm", "최소 경로 길이", " mm", 0, 1000, 10},
                 {"path.turn_drop_memory", "좌/우 명령 들어오면 기억 버림 (1=켬)", "", 0, 1, 1},
             }}),
      group({"사각지대 (path_node) — 로봇 ~ BEV 아랫줄",
             {
                 {"path.fill_blind", "지나온 경로로 채움 (0끔/1직진만/2항상)", "", 0, 2, 1},
                 {"path.fill_join_mm", "채움 연결 허용", " mm", 0, 200, 5},
                 {"path.blind_max_mm", "사각지대 통과 최대", " mm", 0, 1000, 10},
             }}),
      group({"교차로 (path_node)",
             {
                 {"path.turn_radius_mm", "회전 반지름 (0=끔)", " mm", 0, 500, 10},
                 {"path.turn_min_deg", "둥글게 할 꺾임", " °", 0, 180, 5},
                 {"path.branch_look_mm", "분기 미리보기", " mm", 0, 300, 5},
             }}),
      group({"정지선 (path_node) — /mission/stop_line 켜져 있을 때만 (brain: 차단바 끝나면 켬)",
             {
                 {"path.sl_margin_mm", "경로 끝 = 빨간선 - (바퀴 축 기준)", " mm", 0, 500, 5},
                 {"path.sl_min_w_px", "한 줄 빨강 최소 폭 (BEV)", " px", 0, 320, 5},
             },
             2}),
  }, "이름에 마우스를 올리면 ROS 파라미터 이름이 보입니다. 값은 바로 노드에 적용되고, 유지하려면 Save로 파일에 저장하세요."),
               "차선·경로");

  // ====== 2. 벽돌·공사 구간 (path_node + brain 종료 판단)
  tabs->addTab(makeParamPage({
      group({"벽돌 공통 (path_node)",
             {
                 {"path.brick_always", "공사 모드 밖에서도 벽돌 피함 (0=공사 모드만)", "", 0, 1, 1},
                 {"path.det_max_age_ms", "탐지 유효", " ms", 0, 2000, 50},
                 {"path.brick_margin_mm", "좌우 여유 (공사 밖)", " mm", 0, 300, 5},
                 {"path.brick_depth_mm", "앞뒤 길이 (공사 밖)", " mm", 0, 500, 5},
                 {"path.brick_gap_mm", "선까지 틈 막기", " mm", 0, 300, 10},
             }}),
      group({"공사: 벽돌 벽 크기",
             {
                 {"path.con_brick_depth_mm", "앞뒤", " mm", 0, 500, 10},
                 {"path.con_brick_margin_mm", "좌우 여유", " mm", 0, 300, 5},
                 {"path.con_brick_w_mm", "벽돌 실제 가로", " mm", 0, 600, 10},
                 {"path.con_attach_mm", "붙은 쪽 선까지 막기", " mm", 0, 600, 10},
                 {"path.con_edge_mm", "화면 끝까지 막기", " mm", 0, 300, 10},
                 {"path.con_extend_mm", "가려진 선 잇기", " mm", 0, 1000, 10},
             }}),
      group({"공사: 경로",
             {
                 {"path.con_smooth_mm", "경로 모서리 둥글게", " mm", 0, 300, 10},
                 {"path.con_dead_end_mm", "막다른 가지 지우기", " mm", 0, 1000, 10},
                 {"path.con_override_frames", "새 경로 믿기까지", " 프레임", 0, 100, 1},
                 {"path.con_cross_override_frames", "선 너머 경로 믿기까지", " 프레임", 0, 100, 1},
                 {"path.con_straight_by_lane", "직진 가지를 차선 방향으로 (1=켬)", "", 0, 1, 1},
                 {"path.con_offscreen_side", "화면 끝 벽돌은 화면 밖 선에 붙음 (1=켬)", "", 0, 1, 1},
             }}),
      group({"공사: 지난 벽돌 (화면 아래로 빠진 벽돌 기억)",
             {
                 {"path.con_bridge_mm", "같은 쪽 벽돌 잇기 거리 (0=끔)", " mm", 0, 1500, 50},
                 {"path.con_bridge_margin_mm", "잇기 벽 길 쪽 여유", " mm", 0, 300, 5},
                 {"path.con_bridge_dy_mm", "잇기: 안쪽 끝 차이 허용 (0=안 봄)", " mm", 0, 500, 10},
                 {"path.con_bridge_from_back", "잇기: 앞 벽돌 뒷면부터 (1=켬)", "", 0, 1, 1},
                 {"path.con_keep", "경로 밀기 (0끔 1약 2강)", "", 0, 2, 1},
                 {"path.con_keep_rear_mm", "축 뒤 길이", " mm", 0, 300, 10},
                 {"path.con_hold_mm", "더 피하기", " mm", 0, 500, 10},
                 {"path.con_clear_mm", "옆 여유", " mm", 0, 200, 5},
             },
             2}),
      group({"공사: 끝내기",
             {
                 {"mission.con_end_mode", "끝 판단: 0 벽돌(path) 1 시간 2 주차표지판", "", 0, 2, 1},
                 {"mission.con_hold_ms", "유지 시간 (모드 1)", " ms", 0, 120000, 500},
                 {"mission.con_once", "한 번만 (주차 표지판까지 재진입 막기)", "", 0, 1, 1},
                 {"path.con_exit_mm", "벽돌 안 보이고 이만큼 가면 끝 (0=끔, 모드 0)", " mm", 0, 3000, 50},
                 {"path.con_no_brick_mm", "벽돌 못 보고 이만큼 가면 끝 (0=끔, 모드 0)", " mm", 0, 10000, 100},
             },
             2, "모드 1·2를 쓰면 '벽돌 안 보이고…'·'벽돌 못 보고…'(path_node)를 0으로 — 안 그러면 path_node가 먼저 끝냄"}),
  }, "주행 속도·회전 세기 같은 공사 구간 주행 프로필은 '주행 (drive)' 탭의 '공사 구간' 묶음."),
               "벽돌·공사");

  // ====== 3. 표지판·YOLO (signs_node + brain 필터)
  tabs->addTab(makeParamPage({
      makeYoloBox(nullptr),
      group({"표지판 공통 (brain_node)",
             {
                 {"sign.confirm_frames", "연속 프레임 (한 프레임 오검출 거르기)", "", 1, 30, 1},
                 {"sign.min_score_pct", "최소 점수", " %", 0, 100, 5},
                 {"mission.rearm_ms", "같은 표지판 다시 받기까지", " ms", 0, 30000, 250},
             }}),
      makeSignFilterTable(nullptr),
  }, "높이·너비·면적을 모두 넘어야 그 표지판으로 동작합니다 (멀리 보인 작은 박스 거르기). "
     "GUI '탐지' 목록의 박스 크기를 보고 맞추세요."),
               "표지판·YOLO");

  // ====== 4. 미션 (brain): 트랙 순서
  tabs->addTab(makeParamPage({
      group({"출발·차단바·정지선",
             {
                 {"mission.start_follow", "시작하자마자 출발 (1=초록불 안 기다림)", "", 0, 1, 1},
                 {"mission.barrier_release_ms", "차단바 안 보이면 정지 풀기 (0=끔)", " ms", 0, 30000, 250},
                 {"mission.stop_line", "차단바 끝나면 정지선 앞 정지 (1=켬)", "", 0, 1, 1},
             }}),
      group({"좌/우 표지판",
             {
                 {"mission.turn_ms", "turn 유지 시간", " ms", 0, 30000, 500},
             }}),
      group({"주차 — 진행 순서대로",
             {
                 {"mission.parking.time_enter", "① 표지판 → 좌회전 유지", " s", 0, 60, 1},
                 {"mission.parking.psd_L", "② PSD 왼쪽 기준 (벽돌 왼쪽)", "", 0, 65535, 50},
                 {"mission.parking.psd_R", "② PSD 오른쪽 기준 (벽돌 오른쪽)", "", 0, 65535, 50},
                 {"mission.parking.time_break_ms", "③ 기준 넘은 뒤 더 가고 정지", " ms", 0, 10000, 50},
                 {"mission.parking.turn_vel", "④⑥ 제자리 회전 속도 (원시값)", "", 0, 1000, 5},
                 {"mission.parking.turn_deg", "④⑥ 제자리 회전 각도", " °", 0, 360, 5},
                 {"mission.parking.move_vel", "⑤ 전후진 속도 (원시값)", "", 1, 1000, 5},
                 {"mission.parking.dist_mm", "⑤ 후진·직진 거리", " mm", 0, 2000, 10},
                 {"mission.parking_pause", "⑤ 후진 뒤 정지 시간", " s", 0, 60, 1},
                 {"mission.parking.time_exit", "⑦ 끝 → 좌회전 유지", " s", 0, 60, 1},
                 {"mission.parking.psd_F", "PSD 앞 기준 (지금 코드에서는 안 씀)", "", 0, 65535, 50},
             },
             2}),
  }, "공사 구간 시작·끝 판단은 '벽돌·공사' 탭, 표지판 인정 조건은 '표지판·YOLO' 탭."),
               "미션 (brain)");

  // 마지막으로 본 하위 탭 기억
  QSettings settings("tb_gui", "tb_gui");
  tabs->setCurrentIndex(settings.value("param_subtab", 0).toInt());
  connect(tabs, &QTabWidget::currentChanged, this, [](int i)
          { QSettings("tb_gui", "tb_gui").setValue("param_subtab", i); });

  ui->tabWidget->addTab(tabs, "Params");

  // 실수 파라미터 (YOLO) 노드 값 → 화면
  connect(qnode, &QNode::doubleParamLoaded, this, [this](const QString &name, double v)
          {
    QDoubleSpinBox *sp = double_widgets_.value(name, nullptr);
    if (!sp || sp->property("editing").toBool()) // 입력 중인 칸은 덮어쓰지 않음
      return;
    QSignalBlocker block(sp);
    sp->setValue(v); });
}

// ───────── 파라미터 파일 ─────────

void MainWindow::setParamFile(const QString &path)
{
  param_file_ = path;
  QSettings("robit", "tb_gui").setValue("param_file", path);
  ui->labelParamFile->setText("params: " + path);
}

void MainWindow::saveParamsTo(const QString &path)
{
  // 폴더가 없으면 만들어 둠 (예: ~/ros2_ws/params/ 처음 저장할 때)
  if (!QDir().mkpath(QFileInfo(path).absolutePath()))
  {
    ui->labelParamFile->setText("저장 실패: 폴더를 만들 수 없음 (" + path + ")");
    return;
  }
  qnode->saveParams(path);
}

// ───────── 자동 주행 (tb_drive) ─────────

void MainWindow::setupDriveTab()
{
  // 파라미터 묶음: 이름, 표시 이름, 단위, 범위 (tb_drive의 declare_int 범위와 같게)
  // 순서 = 속도가 정해지는 순서 (상한 → 코너 → 가감속) 다음 조향, 구간별 프로필, 안전장치
  const std::vector<ParamGroup> groups = {
      {"속도",
       {
           {"drive.v_max_mm_s", "최고 속도", " mm/s", 0, 1000, 10},
           {"drive.v_curve_mm_s", "코너 속도 (0=끔)", " mm/s", 0, 1000, 10},
           {"drive.curve_start_deg", "코너 감속 시작 각도", " °", 0, 180, 5},
           {"drive.curve_full_deg", "코너 속도 도달 각도", " °", 1, 180, 5},
           {"drive.curv_slow_mm", "곡률 감속", " mm", 0, 1000, 10},
           {"drive.w_max_deg_s", "최대 회전 (부족하면 속도↓)", " °/s", 0, 720, 5},
           {"drive.v_min_mm_s", "최저 속도", " mm/s", 0, 500, 5},
       },
       1, "코너 속도 ≈ v_curve / (1 + curv_slow × 곡률)"},
      {"가감속",
       {
           {"drive.accel_mm_s2", "가속", " mm/s²", 10, 5000, 50},
           {"drive.decel_mm_s2", "감속 (코너 앞 제동거리)", " mm/s²", 10, 5000, 50},
           {"drive.w_accel_deg_s2", "회전 가속", " °/s²", 10, 5000, 30},
       },
       1},
      {"lookahead·조향 (일반)",
       {
           {"drive.ld_time_ms", "lookahead = 속도 × 이 시간", " ms", 0, 5000, 100},
           {"drive.ld_min_mm", "lookahead 최소", " mm", 30, 1000, 10},
           {"drive.ld_max_mm", "lookahead 최대", " mm", 30, 1500, 10},
           {"drive.turn_gain_pct", "회전 세기", " %", 50, 300, 5},
           {"drive.curve_x_offset_mm", "곡선 거리 보정 (+ 늦게 꺾음)", " mm", -200, 200, 5},
       },
       1},
      {"급한 곳 (lookahead 줄이기)",
       {
           {"drive.ld_sharp_mm", "급한 곳 lookahead", " mm", 0, 1000, 10},
           {"drive.sharp_curve_start_deg", "일반: 시작 각도", " °", 0, 180, 5},
           {"drive.sharp_curve_full_deg", "일반: 최대 각도", " °", 1, 180, 5},
           {"drive.sharp_turn_deg", "교차로: 최대 각도 (0°부터)", " °", 1, 180, 5},
           {"drive.sharp_hold_mm", "유지 거리 (0=시간)", " mm", 0, 2000, 10},
           {"drive.sharp_hold_ms", "유지 시간", " ms", 0, 5000, 100},
       },
       1, "앞 350mm 경로 방향이 로봇 정면과 틀어진 최대 각도로 판단"},
      {"교차로 프로필 (/mission/turn ≠ 0)",
       {
           {"drive.cross_v_mm_s", "속도 상한 (0=끔)", " mm/s", 0, 1000, 10},
           {"drive.cross_turn_gain_pct", "회전 세기", " %", 50, 300, 5},
           {"drive.cross_ld_min_mm", "ld_min (0=일반 값)", " mm", 0, 1000, 10},
           {"drive.cross_ld_max_mm", "ld_max (0=일반 값)", " mm", 0, 1500, 10},
           {"drive.cross_v_min_mm_s", "최저 속도 (0=일반 값)", " mm/s", 0, 500, 5},
           {"drive.cross_v_curve_mm_s", "코너 속도 (0=일반 값)", " mm/s", 0, 1000, 10},
       },
       1},
      {"공사 구간 프로필",
       {
           {"drive.con_v_mm_s", "속도 상한", " mm/s", 0, 1000, 10},
           {"drive.con_turn_gain_pct", "회전 세기", " %", 50, 300, 5},
           {"drive.con_ld_min_mm", "ld_min", " mm", 0, 1000, 10},
           {"drive.con_ld_max_mm", "ld_max (0=일반 값)", " mm", 0, 1500, 10},
           {"drive.con_ld_sharp_mm", "ld_sharp (0=끔)", " mm", 0, 1000, 10},
           {"drive.con_v_min_mm_s", "최저 속도 (0=일반 값)", " mm/s", 0, 500, 5},
           {"drive.con_v_curve_mm_s", "코너 속도 (0=일반 값)", " mm/s", 0, 1000, 10},
           {"drive.con_pivot_enter_deg", "피봇 시작", " °", 0, 180, 5},
           {"drive.con_pivot_keep_deg", "피봇 맡김", " °", 0, 180, 5},
       },
       1},
      {"경로 끊김·안전",
       {
           {"drive.path_timeout_ms", "경로 유효", " ms", 50, 3000, 50},
           {"drive.hold_ms", "끊긴 뒤 마지막 경로 유지", " ms", 0, 3000, 50},
           {"drive.weak_src_pct", "약한 경로(memory 등) 속도", " %", 0, 100, 5},
           {"drive.stop_dist_mm", "경로 끝 정지", " mm", 0, 300, 5},
           {"drive.cmd_timeout_ms", "brain 명령 유효", " ms", 50, 3000, 50},
           {"drive.psd.stop_mm", "PSD 정지 거리", " mm", 0, 1000, 10},
           {"drive.control_hz", "제어 주기", " Hz", 5, 100, 5},
       },
       1},
      {"피봇 (제자리 회전)",
       {
           {"drive.pivot_enter_deg", "시작 각도 (180=끔)", " °", 0, 180, 5},
           {"drive.pivot_keep_deg", "남길 각도", " °", 0, 180, 5},
           {"drive.pivot_exit_deg", "종료 각도", " °", 0, 90, 1},
           {"drive.pivot_w_deg_s", "회전 속도", " °/s", 0, 360, 5},
           {"drive.pivot_confirm", "확인 횟수", " 회", 1, 30, 1},
           {"drive.pivot_fallback_deg", "안전장치 각도", " °", 0, 180, 5},
           {"drive.corner_min_x_mm", "코너 검출 시작 거리", " mm", 0, 1000, 10},
       },
       1},
  };
  constexpr int GROUP_COLS = 3; // 묶음 3개씩 한 줄

  auto *wrap = new QWidget(ui->groupDriveParams);
  auto *grid = new QGridLayout(wrap);
  grid->setContentsMargins(0, 0, 0, 0);
  int i = 0;
  for (const auto &g : groups)
  {
    QGroupBox *box = makeParamGroup(wrap, g);
    grid->addWidget(box, i / GROUP_COLS, i % GROUP_COLS, Qt::AlignTop);
    ++i;
  }
  for (int c = 0; c < GROUP_COLS; ++c)
    grid->setColumnStretch(c, 1);
  grid->addWidget(makeOdomBox(wrap), i / GROUP_COLS, i % GROUP_COLS, Qt::AlignTop); // odom 보정 (tb_uart_node)
  ui->gridDriveParams->addWidget(wrap, 0, 0);
  ui->gridDriveParams->setRowStretch(1, 1);

  // bool 파라미터 체크박스
  bool_widgets_ = {{"drive.require_cmd", ui->checkRequireCmd},
                   {"drive.latency_comp", ui->checkLatencyComp},
                   {"drive.psd.enable", ui->checkPsdEnable}};
  for (auto it = bool_widgets_.begin(); it != bool_widgets_.end(); ++it)
  {
    const QString name = it.key();
    QCheckBox *c = it.value();
    connect(c, &QCheckBox::toggled, this, [this, name, c](bool on)
            {
      if (!qnode->setBoolParam(name, on))
      {
        QSignalBlocker block(c);
        c->setChecked(!on);
        ui->labelDriveInfo->setText("drive_node 연결 안 됨");
      } });
  }

  // 활성화 체크박스 / 정지 버튼
  connect(ui->checkAutoEnable, &QCheckBox::toggled, this, [this](bool on)
          { setAutoDrive(on); });
  connect(ui->btnAutoStop, &QPushButton::clicked, this, [this]
          { setAutoDrive(false); });

  // 노드 값 → 화면 (다른 곳에서 ros2 param set 해도 반영)
  connect(qnode, &QNode::boolParamLoaded, this, [this](const QString &name, bool on)
          {
    QCheckBox *c = name == "drive.enable" ? ui->checkAutoEnable : bool_widgets_.value(name, nullptr);
    if (!c)
      return;
    QSignalBlocker block(c);
    c->setChecked(on);
    if (name == "drive.enable" && on && ui->checkDriveEnable->isChecked())
      ui->checkDriveEnable->setChecked(false); });

  // 상태 표시
  connect(qnode, &QNode::driveStateReceived, this, &MainWindow::showDriveState);
  drive_state_timeout_.setSingleShot(true);
  drive_state_timeout_.setInterval(1000);
  connect(&drive_state_timeout_, &QTimer::timeout, this, [this]
          { showDriveState(""); });
  showDriveState("");
}

void MainWindow::setAutoDrive(bool on)
{
  if (!on && brain_run_)
    setBrainRun(false); // 주행을 끄면 brain도 (Drive 탭 체크박스, 정지 버튼 등 어느 경로든)
  if (on && ui->checkDriveEnable->isChecked())
    ui->checkDriveEnable->setChecked(false); // WASD 끄기 (stopDrive 호출됨)

  const bool ok = qnode->setBoolParam("drive.enable", on);
  QSignalBlocker block(ui->checkAutoEnable);
  // 켜기에 실패하면 체크 해제. 끄기는 실패해도 화면은 꺼짐으로 두고 경고만
  ui->checkAutoEnable->setChecked(on && ok);
  if (!ok)
    ui->labelDriveInfo->setText(on ? "켜기 실패: drive_node 연결 안 됨"
                                   : "끄기 실패: drive_node 연결 안 됨 (로봇 확인!)");
}

// "FOLLOW src=live v=0.142 w=0.31 ld=0.25 end=0.48 HOLD stop:path_end"
void MainWindow::showDriveState(const QString &text)
{
  static const QMap<QString, QString> colors = {
      {"FOLLOW", "#2e9d4a"}, {"STOP", "#e67e22"}, {"TWIST", "#2980b9"}, {"DISABLED", "#555555"}};

  if (text.isEmpty())
  {
    ui->labelDriveMode->setText("drive: (no data)");
    ui->labelDriveMode->setStyleSheet(
        "font-size: 16px; font-weight: bold; border-radius: 4px; color: white; background: #555555;");
    ui->labelDriveVel->setText("v: -   w: -");
    return;
  }
  drive_state_timeout_.start();

  const QStringList t = text.split(' ', Qt::SkipEmptyParts);
  const QString mode = t.value(0);
  QMap<QString, QString> kv;
  QString stop;
  bool hold = false;
  for (int i = 1; i < t.size(); ++i)
  {
    if (t[i] == "HOLD")
      hold = true;
    else if (t[i].startsWith("stop:"))
      stop = t[i].mid(5);
    else if (const int eq = t[i].indexOf('='); eq > 0)
      kv[t[i].left(eq)] = t[i].mid(eq + 1);
  }

  QString title = mode;
  if (!stop.isEmpty())
    title += "  (" + stop + ")";
  else if (hold)
    title += "  (HOLD)";
  ui->labelDriveMode->setText(title);
  ui->labelDriveMode->setStyleSheet(
      QString("font-size: 16px; font-weight: bold; border-radius: 4px; color: white; background: %1;")
          .arg(colors.value(mode, "#555555")));

  ui->labelDriveVel->setText(QString("v: %1 m/s   w: %2 rad/s").arg(kv.value("v", "-"), kv.value("w", "-")));
  static const QMap<QString, QString> prof_name = {{"curve", "일반"}, {"cross", "교차로"}, {"con", "공사"}};
  ui->labelDriveInfo->setText(QString("src: %1   ld: %2 m   end: %3 m   프로필: %4")
                                  .arg(kv.value("src", "-"), kv.value("ld", "-"), kv.value("end", "-"),
                                       prof_name.value(kv.value("prof"), kv.value("prof", "-"))));
}

// ───────── 주행 (WASD) ─────────

bool MainWindow::eventFilter(QObject *obj, QEvent *ev)
{
  const bool press = ev->type() == QEvent::KeyPress;

  if ((press || ev->type() == QEvent::KeyRelease) && viewTabActive() &&
      handleViewKey(static_cast<QKeyEvent *>(ev), press))
    return true;

  // 자동 주행 중 Space → 자동 주행 끄기
  if (press && ui->checkAutoEnable->isChecked())
  {
    auto *ke = static_cast<QKeyEvent *>(ev);
    if (ke->key() == Qt::Key_Space && !ke->isAutoRepeat())
    {
      setAutoDrive(false);
      return true;
    }
  }

  if ((press || ev->type() == QEvent::KeyRelease) && ui->checkDriveEnable->isChecked())
  {
    auto *ke = static_cast<QKeyEvent *>(ev);
    const int k = ke->key();
    if (k == Qt::Key_W || k == Qt::Key_A || k == Qt::Key_S || k == Qt::Key_D || k == Qt::Key_Space)
    {
      if (!ke->isAutoRepeat()) // 키를 누르고 있을 때의 반복 이벤트는 무시
      {
        if (k == Qt::Key_Space)
        {
          if (press)
            stopDrive(true);
        }
        else
        {
          if (press)
            drive_keys_.insert(k);
          else
            drive_keys_.remove(k);
          updateDrive();
        }
      }
      return true; // 다른 위젯(체크박스, 슬라이더 등)으로 넘기지 않음
    }
  }
  return QMainWindow::eventFilter(obj, ev);
}

// W/S: 전진/후진, A/D: 좌/우 회전 (같이 누르면 곡선 주행)
// L = v + t, R = v - t  →  둘 중 큰 값이 한계치를 넘으면 비율 유지한 채 축소
void MainWindow::updateDrive()
{
  if (!ui->checkDriveEnable->isChecked())
    return;

  const int fwd = int(drive_keys_.contains(Qt::Key_W)) - int(drive_keys_.contains(Qt::Key_S));
  const int turn = int(drive_keys_.contains(Qt::Key_D)) - int(drive_keys_.contains(Qt::Key_A));
  const int vmax = ui->spinMaxSpeed->value();

  const int v = fwd * vmax;
  const int t = turn * vmax * ui->spinTurnRatio->value() / 100;
  int l = v + t;
  int r = v - t;

  const int peak = std::max(std::abs(l), std::abs(r));
  if (peak > vmax && peak > 0)
  {
    l = l * vmax / peak;
    r = r * vmax / peak;
  }
  sendDrive(l, r);
}

void MainWindow::sendDrive(int l, int r, bool force)
{
  if (!force && l == drive_l_ && r == drive_r_)
    return;
  drive_l_ = l;
  drive_r_ = r;
  qnode->publishVelocity(l, r);
  ui->labelDriveCmd->setText(QString("cmd  L: %1   R: %2").arg(l).arg(r));
}

void MainWindow::stopDrive(bool force)
{
  drive_keys_.clear();
  sendDrive(0, 0, force);
}

// tb_uart_node가 보내는 문자열 (velocity / psd / stm32 / error)
void MainWindow::showUart(const QString &text)
{
  const QStringList t = text.split(' ', Qt::SkipEmptyParts);
  if (t.isEmpty())
    return;

  if (t[0] == "velocity")
  {
    if (t.size() == 3 && (t[1] == "L" || t[1] == "R"))
      (t[1] == "L" ? ui->moterL_speedLabel : ui->moterR_speedLabel)->setText(t[1] + " : " + t[2]);
    else if (t.size() == 3)
    {
      ui->moterL_speedLabel->setText("L : " + t[1]);
      ui->moterR_speedLabel->setText("R : " + t[2]);
    }
  }
  else if (t[0] == "psd")
  {
    if (t.size() == 4)
    {
      ui->frontPSDLabel->setText("정면 : " + t[1]);
      ui->leftPSDLabel_3->setText("좌측 : " + t[2]);
      ui->rightPSDLabel_2->setText("우측 : " + t[3]);
    }
    else if (t.size() == 3)
    {
      if (t[1] == "F")
        ui->frontPSDLabel->setText("정면 : " + t[2]);
      else if (t[1] == "L")
        ui->leftPSDLabel_3->setText("좌측 : " + t[2]);
      else if (t[1] == "R")
        ui->rightPSDLabel_2->setText("우측 : " + t[2]);
    }
  }
  else if (t[0] == "stm32" && t.size() >= 2 && (t[1] == "start" || t[1] == "quit"))
  {
    const bool on = t[1] == "start";
    ui->labelUartState->setText(on ? "STM32: running" : "STM32: stopped");
    ui->labelUartState->setStyleSheet(on ? "color: limegreen; font-weight: bold;" : "color: gray; font-weight: bold;");
  }
  else if (t[0] == "error")
  {
    ui->labelUartLog->setText(text);
    ui->labelUartLog->setStyleSheet("color: #c0392b;");
  }
}

// ───────── 위쪽 영상 칸: 주행 / 튜닝 탭 ─────────
//
//  [주행]  USB(YOLO) | 경로 보기 | Skeleton          ← 달릴 때 볼 것만 크게
//  [튜닝]  USB  | Mask | BEV                         ← 차선·BEV·경로 파라미터 맞출 때
//          ROI  | Skeleton | 경로 보기
//  Status / Mission 상자는 탭 밖 오른쪽에 그대로 (어느 탭이든 미션 조작 가능)
//  .ui 파일은 그대로 두고, 원래 영상 라벨은 숨긴 뒤 코드로 새로 배치함
void MainWindow::setupDisplayTabs()
{
  // 원래 .ui의 영상 라벨·제목은 숨김 (빈 칸이 되어 자리를 차지하지 않음)
  for (QWidget *w : std::initializer_list<QWidget *>{ui->displayUsb, ui->labelUsb, ui->displayMask, ui->labelMask,
                                                     ui->displayBev, ui->labelDisplay_4, ui->displayRoi, ui->labelDisplay_5,
                                                     ui->displaySkel, ui->labelDisplay_6})
    w->hide();

  // 칸 하나 = 내용 + 아래 제목
  auto cell = [](QWidget *content, const QString &title)
  {
    auto *box = new QWidget;
    auto *vl = new QVBoxLayout(box);
    vl->setContentsMargins(0, 0, 0, 0);
    vl->setSpacing(2);
    content->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored); // 영상 크기로 레이아웃이 커지지 않게
    content->setMinimumSize(1, 1);
    vl->addWidget(content, 1);
    auto *t = new QLabel(title, box);
    t->setAlignment(Qt::AlignCenter);
    vl->addWidget(t, 0);
    return box;
  };
  // 영상 칸 (stream 이름으로 등록 → showStream이 보이는 것만 그림)
  auto image = [this, &cell](const QString &stream, const QString &title)
  {
    auto *l = new QLabel(stream);
    l->setAlignment(Qt::AlignCenter);
    l->setStyleSheet("color: gray;");
    streams_[stream].append(l);
    return cell(l, title);
  };
  auto path = [this, &cell]()
  {
    PathView *v = makePathView();
    v->setMinimumSize(200, 1);
    return cell(v, "Path (/lane/path → drive)");
  };

  display_tabs_ = new QTabWidget(ui->groupStatus->parentWidget());
  display_tabs_->setDocumentMode(true);

  // ====== 주행: 한 줄로 크게
  auto *drive_page = new QWidget;
  auto *dg = new QGridLayout(drive_page);
  dg->setContentsMargins(2, 2, 2, 2);
  dg->setSpacing(5);
  dg->addWidget(image("usb", "USB (YOLO)"), 0, 0);
  dg->addWidget(path(), 0, 1);
  dg->addWidget(image("skel", "Skeleton"), 0, 2);
  dg->addWidget(makeLogPanel(), 1, 0);      // USB 밑: brain 로그
  dg->addWidget(makeDebugPanel(), 1, 1, 1, 2); // 시간·수신 디버그
  dg->setColumnStretch(0, 4);
  dg->setColumnStretch(1, 3);
  dg->setColumnStretch(2, 3);
  dg->setRowStretch(0, 3);
  dg->setRowStretch(1, 2);
  display_tabs_->addTab(drive_page, "주행");

  // ====== 주행+: 경로 보기에 path_node가 쓴 선·벽돌·기억(/lane/world)을 겹쳐 크게
  auto *world_page = new QWidget;
  auto *wg = new QGridLayout(world_page);
  wg->setContentsMargins(2, 2, 2, 2);
  wg->setSpacing(5);
  {
    PathView *v = makePathView();
    v->setWorldMode(true);
    connect(qnode, &QNode::worldReceived, v, &PathView::setWorld);
    v->setMinimumSize(200, 1);
    wg->addWidget(cell(v, "Path + 선·벽돌·기억 (/lane/world)"), 0, 1, 2, 1);
  }
  wg->addWidget(image("usb", "USB (YOLO)"), 0, 0);
  wg->addWidget(image("skel", "Skeleton"), 1, 0);
  wg->setColumnStretch(0, 2);
  wg->setColumnStretch(1, 3);
  display_tabs_->addTab(world_page, "주행+");

  // ====== 튜닝: 2 x 3
  auto *tune_page = new QWidget;
  auto *tg = new QGridLayout(tune_page);
  tg->setContentsMargins(2, 2, 2, 2);
  tg->setSpacing(5);
  tg->addWidget(image("usb", "USB"), 0, 0);
  tg->addWidget(image("mask", "Lane mask"), 0, 1);
  tg->addWidget(image("bev", "BEV"), 0, 2);
  tg->addWidget(image("roi", "ROI"), 1, 0);
  tg->addWidget(image("skel", "Skeleton"), 1, 1);
  tg->addWidget(path(), 1, 2);
  for (int c = 0; c < 3; ++c)
    tg->setColumnStretch(c, 1);
  display_tabs_->addTab(tune_page, "튜닝");

  // 원래 3열 그리드 자리에 탭을 넣고, Status 상자는 오른쪽 열에 세로로 길게
  ui->gridLayout_7->removeWidget(ui->groupStatus);
  ui->gridLayout_7->addWidget(display_tabs_, 0, 0, 2, 2);
  ui->gridLayout_7->addWidget(ui->groupStatus, 0, 2, 2, 1);
  ui->gridLayout_7->setColumnStretch(0, 1);
  ui->gridLayout_7->setColumnStretch(1, 1);
  ui->gridLayout_7->setColumnStretch(2, 0);
  ui->groupStatus->setMaximumWidth(420); // 영상 탭에 자리를 더 줌

  // 마지막으로 본 탭 기억
  QSettings settings("tb_gui", "tb_gui");
  display_tabs_->setCurrentIndex(settings.value("display_tab", 0).toInt());
  connect(display_tabs_, &QTabWidget::currentChanged, this, [](int i)
          { QSettings("tb_gui", "tb_gui").setValue("display_tab", i); });
}

// 경로 보기 하나 만들고 토픽·파라미터 연결 (주행·튜닝 탭에 하나씩)
PathView *MainWindow::makePathView()
{
  auto *v = new PathView;
  connect(qnode, &QNode::pathReceived, v, &PathView::setPath);
  connect(qnode, &QNode::pathSourceReceived, v, &PathView::setSource);
  connect(qnode, &QNode::targetReceived, v, &PathView::setTarget);
  connect(qnode, &QNode::driveStateReceived, v, &PathView::setDriveState);
  connect(qnode, &QNode::uartCommandSeen, v, &PathView::setUartCommand);
  connect(qnode, &QNode::paramLoaded, v, &PathView::setIntParam);      // bev.*, geo.* (나머지 이름은 무시)
  connect(qnode, &QNode::doubleParamLoaded, v, &PathView::setDoubleParam);
  return v;
}

// 녹화 버튼: 영상 탭(주행/튜닝) 이름 줄 오른쪽 끝
//  녹화는 화면 표시와 따로 qnode 신호를 받음 → 영상을 숨기거나 다른 탭을 봐도 녹화됨
void MainWindow::setupRecorder()
{
  recorder_ = new Recorder(this);
  connect(qnode, &QNode::usbImageReceived, recorder_, [this](const QImage &img) { recorder_->setFrame("usb", img); });
  connect(qnode, &QNode::skelImageReceived, recorder_, [this](const QImage &img) { recorder_->setFrame("skel", img); });
  connect(qnode, &QNode::nodeLogReceived, recorder_, &Recorder::logNode); // 로그 필터와 상관없이 전부
  connect(qnode, &QNode::paramFileStatus, recorder_, [this](const QString &text)
          { recorder_->logEvent("파라미터 파일: " + QString(text).replace('\n', ' ')); }); // params.yaml 저장 결과 등

  rec_path_view_ = makePathView();
  rec_path_view_->setWorldMode(true); // 녹화에는 선·벽돌·기억까지 (주행+ 탭과 같은 그림)
  connect(qnode, &QNode::worldReceived, rec_path_view_, &PathView::setWorld);
  rec_path_view_->setParent(this);
  rec_path_view_->hide();
  rec_path_view_->resize(640, 480);
  recorder_->setPathView(rec_path_view_);

  btn_record_ = new QPushButton(display_tabs_);
  btn_record_->setCheckable(true);
  btn_record_->setFocusPolicy(Qt::NoFocus); // WASD·방향 키 입력을 뺏지 않게
  btn_record_->setToolTip(QString("USB · 경로 보기 · Skeleton + 시간 + 로그를 한 영상(record.mp4, %1x%2, %3 fps)으로,\n"
                                  "노드 로그를 events.log로, 시작할 때 노드의 현재 파라미터 값을 params.yaml로 저장\n"
                                  "파라미터 노드가 하나도 연결되어 있지 않으면 녹화하지 않음\n"
                                  "저장 위치: ~/tb_records/<날짜.시작시각>/")
                              .arg(Recorder::kW)
                              .arg(Recorder::kH)
                              .arg(Recorder::kFps));
  display_tabs_->setCornerWidget(btn_record_, Qt::TopRightCorner);
  connect(btn_record_, &QPushButton::toggled, this, &MainWindow::setRecording);
  connect(&debug_timer_, &QTimer::timeout, this, &MainWindow::updateRecordButton);
  updateRecordButton();
}

void MainWindow::setRecording(bool on)
{
  if (on && !recorder_->isRecording())
  {
    // 파라미터 노드가 하나도 없으면 녹화 안 함 (폴더·파일도 안 만듦)
    const QStringList nodes = qnode->paramNodesReady();
    QString fail;
    if (nodes.isEmpty())
      fail = "녹화 안 함: 연결된 노드 없음 (lineDetect_node, path_node, drive_node, brain_node)";
    else if (!recorder_->start(QDir::home().filePath("tb_records")))
      fail = "녹화 실패: 폴더/파일을 만들 수 없음";
    if (!fail.isEmpty())
    {
      statusBar()->showMessage(fail, 5000);
      QSignalBlocker block(btn_record_);
      btn_record_->setChecked(false);
    }
    else
    {
      // 녹화 당시 노드의 현재 파라미터 값 (Save 버튼과 같은 형식, 결과는 파라미터 파일 표시 줄에 나옴)
      const QString params = QDir(recorder_->dir()).filePath("params.yaml");
      qnode->saveParams(params);
      recorder_->logEvent(QString("파라미터 저장 요청: %1 (연결된 노드: %2)").arg(params, nodes.join(", ")));
      statusBar()->showMessage("녹화 시작: " + recorder_->dir(), 5000);
    }
  }
  else if (!on && recorder_->isRecording())
  {
    const QString dir = recorder_->dir();
    recorder_->stop();
    statusBar()->showMessage("녹화 저장됨: " + dir, 10000);
  }
  updateRecordButton();
}

void MainWindow::updateRecordButton()
{
  if (!btn_record_)
    return;
  if (recorder_->isRecording())
  {
    const qint64 s = recorder_->elapsedMs() / 1000;
    btn_record_->setText(QString("■ 녹화 중지 (%1:%2)").arg(s / 60, 2, 10, QChar('0')).arg(s % 60, 2, 10, QChar('0')));
    btn_record_->setStyleSheet("QPushButton { color: white; background: #c0392b; font-weight: bold; padding: 2px 8px; }");
  }
  else
  {
    btn_record_->setText("● 녹화");
    btn_record_->setStyleSheet("QPushButton { color: #c0392b; font-weight: bold; padding: 2px 8px; }");
  }
}

// 같은 영상을 보여주는 라벨 중 지금 화면에 보이는 것만 그림 (숨은 탭은 크기 조정 안 함)
void MainWindow::showStream(const QString &stream, const QImage &img)
{
  stream_ms_[stream] = clock_.elapsed(); // NO SIGNAL 판단용 (안 보이는 탭이어도 기록)
  for (QLabel *l : streams_.value(stream))
    if (l->isVisible())
      showImage(l, img);
}

void MainWindow::showImage(QLabel *label, const QImage &img)
{
  label->setPixmap(QPixmap::fromImage(img).scaled(
      label->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
}

void MainWindow::queueParam(const QString &name, int v)
{
  pending_[name] = v;
  if (!send_timer_.isActive())
    send_timer_.start();
}

void MainWindow::applyGlobalParam(const QString &name, int v)
{
  const GlobalWidget &w = global_widgets_[name];
  if (w.combo)
  {
    const int i = w.combo->findData(v);
    if (i >= 0 && i != w.combo->currentIndex())
    {
      QSignalBlocker block(w.combo);
      w.combo->setCurrentIndex(i);
    }
    return;
  }
  if (w.slider && !w.slider->isSliderDown())
  {
    QSignalBlocker block(w.slider);
    w.slider->setValue(v);
    if (w.label)
      w.label->setNum(v);
  }
  // addParamSpin으로 만든 칸은 'editing'으로, 나머지(Tab 1 스핀박스)는 포커스로 입력 중인지 판단
  const QVariant editing = w.spin ? w.spin->property("editing") : QVariant();
  const bool busy = w.spin && (editing.isValid() ? editing.toBool() : w.spin->hasFocus());
  if (w.spin && !busy)
  {
    QSignalBlocker block(w.spin);
    w.spin->setValue(v);
  }
}

void MainWindow::loadTargetToSliders()
{
  for (auto it = hsv_widgets_.begin(); it != hsv_widgets_.end(); ++it)
  {
    const QString param = current_target_ + "." + it.key();
    if (!hsv_cache_.contains(param) || it->slider->isSliderDown())
      continue;
    const int v = hsv_cache_[param];
    QSignalBlocker block(it->slider);
    it->slider->setValue(v);
    it->label->setNum(v);
  }
}

// live 초록 / memory 보라 / color_rule 주황 / lost 빨강 / 끊김 회색
void MainWindow::showPathSource(const QString &src)
{
  static const QMap<QString, QString> colors = {
      {"live", "#2e9d4a"}, {"memory", "#8e44ad"}, {"color_rule", "#e67e22"}, {"lost", "#c0392b"},
      {"construction", "#16a085"}};
  const QString bg = colors.value(src, "#555555");
  ui->labelPathSource->setText(src.isEmpty() ? "path: (no data)" : "path: " + src);
  ui->labelPathSource->setStyleSheet(
      QString("font-size: 16px; font-weight: bold; border-radius: 4px; color: white; background: %1;").arg(bg));
  if (!src.isEmpty())
    status_timeout_.start();
}

void MainWindow::showMission()
{
  static const char *names[] = {"straight", "left", "right"};
  QString cons = construction_state_ ? "ON" : "off";
  if (construction_state_ != construction_)
    cons += construction_ ? " (진입 요청 보냄)" : " (끝 요청 보냄)";
  ui->labelMission->setText(QString("turn: %1 | parking: %2 | 공사: %3%4")
                                .arg(names[std::clamp(turn_, 0, 2)])
                                .arg(parking_ ? "ON" : "off")
                                .arg(cons)
                                .arg(check_barrier_ && check_barrier_->isChecked() ? " | 차단바 정지" : ""));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  stopDrive(); // 주행 중에 창을 닫으면 정지 명령 전송
  barrier_timer_.stop();
  if (recorder_)
    recorder_->stop(); // 녹화 중에 닫으면 파일을 정상적으로 닫음
  if (ui->checkAutoEnable->isChecked())
    qnode->setBoolParam("drive.enable", false); // 테스트 중 GUI를 닫으면 자동 주행도 끔
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete qnode;
  delete ui;
}
// ───────── Tab 1: 흰색 판단 방식 ─────────
// white.mode 0: HSV 절대 밝기 (위 Value Low 이상)
// white.mode 1: 대비 — 주변 바닥 밝기(배경 상자로 opening)보다 '대비'만큼 밝고 'V 바닥' 이상이면 흰선
//               H·S 범위와 Value High는 두 모드 공통, Value Low는 대비 모드에서 안 씀
void MainWindow::setupWhiteModeBox()
{
  auto *box = new QGroupBox("흰색 판단 방식 (lineDetect_node)", ui->tab);
  auto *g = new QGridLayout(box);

  auto *combo = new QComboBox(box);
  combo->addItem("HSV 절대 밝기 (Value Low 이상)", 0);
  combo->addItem("대비 (주변 바닥보다 밝은 만큼)", 1);
  combo->setToolTip("대비: 빛반사가 넓게 퍼진 곳·조명이 어두운 곳에서도 같은 값으로 잡힘\n"
                    "H·S 범위와 Value High는 두 모드 공통, Value Low는 대비 모드에서 안 씀");
  g->addWidget(new QLabel("모드", box), 0, 0);
  g->addWidget(combo, 0, 1, 1, 2);
  global_widgets_["white.mode"].combo = combo;
  connect(combo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this, combo](int)
          { queueParam("white.mode", combo->currentData().toInt()); });

  struct Def
  {
    const char *name, *text, *tip;
    int min, max;
    bool odd;
  };
  const Def defs[] = {
      {"white.bg_px", "배경 상자(px)", "주변 바닥 밝기를 구할 상자 크기 (홀수). 선 폭(약 13px)의 3배쯤.\n"
                                       "이보다 넓은 반사 덩어리는 빠지고, 너무 크면 위치별 밝기 차이를 못 따라감",
       3, 151, true},
      {"white.contrast", "대비", "주변 바닥보다 이만큼 이상 밝아야 흰선.\n올리면 반사·잡티 감소, 내리면 어두운 곳 선이 잘 잡힘",
       0, 255, false},
      {"white.v_floor", "V 바닥", "대비 모드에서 이보다 어두운 픽셀은 무시 (어두운 바닥의 잡티 방지)", 0, 255, false},
  };
  int row = 1;
  for (const Def &d : defs)
  {
    const QString name = d.name;
    const bool odd = d.odd;
    auto *text = new QLabel(d.text, box);
    auto *slider = new QSlider(Qt::Horizontal, box);
    auto *value = new QLabel("000", box);
    value->setMinimumWidth(40);
    slider->setRange(d.min, d.max);
    slider->setSingleStep(odd ? 2 : 1);
    slider->setPageStep(odd ? 10 : 10);
    text->setToolTip(d.tip);
    slider->setToolTip(d.tip);
    g->addWidget(text, row, 0);
    g->addWidget(slider, row, 1);
    g->addWidget(value, row, 2);
    g->setColumnStretch(1, 1);
    ++row;
    global_widgets_[name] = {slider, value, nullptr};

    connect(slider, &QSlider::valueChanged, this, [this, name, slider, value, odd](int v)
            {
      if (odd && v % 2 == 0)
      {
        v = std::min(v + 1, slider->maximum());
        QSignalBlocker block(slider);
        slider->setValue(v);
      }
      value->setNum(v);
      queueParam(name, v); });
  }

  ui->gridLayout_3->addWidget(box, 2, 0);
}

// ───────── 주행 탭 아래: 노드 로그 / 시간·수신 디버그 / NO SIGNAL ─────────

QWidget *MainWindow::makeLogPanel()
{
  auto *box = new QGroupBox("노드 로그 (/rosout)");
  auto *vl = new QVBoxLayout(box);
  vl->setContentsMargins(4, 4, 4, 4);
  vl->setSpacing(2);

  auto *row = new QHBoxLayout;
  log_all_ = new QCheckBox("path·drive도", box);
  log_all_->setToolTip("끄면 brain_node 로그만. 켜면 path_node(정지선 찾음 등)·drive_node 로그도");
  log_all_->setFocusPolicy(Qt::NoFocus);
  log_all_->setChecked(QSettings("tb_gui", "tb_gui").value("log_all", false).toBool());
  connect(log_all_, &QCheckBox::toggled, this, [](bool on) { QSettings("tb_gui", "tb_gui").setValue("log_all", on); });
  auto *btn_clear = new QPushButton("지우기", box);
  btn_clear->setFocusPolicy(Qt::NoFocus);
  row->addWidget(new QLabel("시각: 로그 찍은 쪽 unix 시간 (로봇 시계)", box));
  row->addStretch(1);
  row->addWidget(log_all_);
  row->addWidget(btn_clear);
  vl->addLayout(row);

  log_view_ = new QPlainTextEdit(box);
  log_view_->setReadOnly(true);
  log_view_->setFocusPolicy(Qt::NoFocus); // View 탭 키 조작을 뺏지 않게
  log_view_->setMaximumBlockCount(2000);
  log_view_->setLineWrapMode(QPlainTextEdit::NoWrap);
  QFont mono = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  mono.setStyleHint(QFont::TypeWriter); // 고정폭 글꼴이 없으면 비슷한 것으로
  mono.setPointSizeF(std::max(7.0, mono.pointSizeF() - 1.0));
  log_view_->setFont(mono);
  log_view_->setStyleSheet("QPlainTextEdit { background: #1b1b1f; color: #e0e0e0; }"); // 밝은 테마에서도 색이 보이게
  log_view_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  vl->addWidget(log_view_, 1);

  connect(btn_clear, &QPushButton::clicked, log_view_, &QPlainTextEdit::clear);
  connect(qnode, &QNode::nodeLogReceived, this, &MainWindow::appendNodeLog);
  return box;
}

void MainWindow::appendNodeLog(const QString &node, int level, qint64 stamp_ns, const QString &text)
{
  if (!log_view_ || (node != "brain_node" && !log_all_->isChecked()))
    return;
  // rcl_interfaces/Log: DEBUG 10, INFO 20, WARN 30, ERROR 40, FATAL 50
  const char *color = level >= 40 ? "#ff6b6b" : level >= 30 ? "#f5a623" : node == "brain_node" ? "#e0e0e0" : "#9ab";
  const QDateTime dt = QDateTime::fromMSecsSinceEpoch(stamp_ns / 1000000);
  const QString line = QString("%1.%2 %3 %4 │ %5")
                           .arg(stamp_ns / 1000000000)
                           .arg((stamp_ns / 1000000) % 1000, 3, 10, QChar('0'))
                           .arg(dt.toString("HH:mm:ss.zzz"))
                           .arg(node.left(node.indexOf('_')).leftJustified(5))
                           .arg(text);
  QScrollBar *sb = log_view_->verticalScrollBar();
  const bool at_bottom = sb->value() >= sb->maximum() - 2;
  const int keep = sb->value();
  log_view_->appendHtml(QString("<span style=\"color:%1; white-space:pre;\">%2</span>").arg(color, line.toHtmlEscaped()));
  if (at_bottom)
    sb->setValue(sb->maximum()); // 맨 아래 보고 있을 때만 따라감 (위로 올려 읽는 중이면 그대로)
  else
    sb->setValue(keep);
}

QWidget *MainWindow::makeDebugPanel()
{
  auto *box = new QGroupBox("시간·수신 (디버그)");
  auto *vl = new QVBoxLayout(box);
  vl->setContentsMargins(4, 4, 4, 4);
  vl->setSpacing(2);

  auto *row = new QHBoxLayout;
  row->addWidget(new QLabel("영상이 이만큼 안 오면 NO SIGNAL", box));
  spin_nosignal_ = new QDoubleSpinBox(box);
  spin_nosignal_->setRange(0.2, 30.0);
  spin_nosignal_->setSingleStep(0.5);
  spin_nosignal_->setDecimals(1);
  spin_nosignal_->setSuffix(" s");
  spin_nosignal_->setValue(QSettings("tb_gui", "tb_gui").value("nosignal_s", 1.0).toDouble());
  spin_nosignal_->setKeyboardTracking(false);
  connect(spin_nosignal_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          [](double v) { QSettings("tb_gui", "tb_gui").setValue("nosignal_s", v); });
  row->addWidget(spin_nosignal_);
  row->addStretch(1);
  vl->addLayout(row);

  label_debug_ = new QLabel(box);
  label_debug_->setTextFormat(Qt::PlainText);
  label_debug_->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  label_debug_->setTextInteractionFlags(Qt::TextSelectableByMouse); // 값 복사용
  QFont mono = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  mono.setStyleHint(QFont::TypeWriter);
  mono.setPointSizeF(std::max(7.0, mono.pointSizeF() - 1.0));
  label_debug_->setFont(mono);
  label_debug_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  vl->addWidget(label_debug_, 1);

  clock_.start();
  debug_timer_.setInterval(200);
  connect(&debug_timer_, &QTimer::timeout, this, &MainWindow::updateDebug);
  debug_timer_.start();
  return box;
}

void MainWindow::updateDebug()
{
  // ====== NO SIGNAL: 영상마다 마지막 프레임이 기준보다 오래되면 (한 번도 안 왔으면 바로)
  const double thr = spin_nosignal_ ? spin_nosignal_->value() : 1.0;
  const qint64 now_ms = clock_.elapsed();
  for (auto it = streams_.begin(); it != streams_.end(); ++it)
  {
    const bool seen = stream_ms_.contains(it.key());
    const double age = seen ? (now_ms - stream_ms_.value(it.key())) / 1000.0 : -1.0;
    if (seen && age <= thr)
      continue;
    for (QLabel *l : it.value())
      if (l->isVisible())
        drawNoSignal(l, it.key(), age);
  }

  // ====== 시간·수신 표
  if (!label_debug_ || !label_debug_->isVisible())
    return;
  const qint64 unix_ms = QDateTime::currentMSecsSinceEpoch();
  QString t;
  t += QString("PC  unix %1.%2  %3\n")
           .arg(unix_ms / 1000)
           .arg(unix_ms % 1000, 3, 10, QChar('0'))
           .arg(QDateTime::fromMSecsSinceEpoch(unix_ms).toString("yyyy-MM-dd HH:mm:ss.zzz"));
  t += QString("ROS now  %1   GUI uptime %2 s\n\n")
           .arg(qnode->rosNowSec(), 0, 'f', 3)
           .arg(now_ms / 1000.0, 0, 'f', 1);

  struct Row
  {
    const char *key, *name;
  };
  static const Row rows[] = {
      {"/camera/image_raw/compressed", "camera"},
      {"/signs/debug/compressed", "yolo img"},
      {"/signs/detections", "detections"},
      {"/lane/debug/skel/compressed", "skel img"},
      {"/lane/debug/mask/compressed", "mask img"},
      {"/lane/path", "lane/path"},
      {"/lane/world", "lane/world"},
      {"/drive/state", "drive/state"},
      {"/rosout brain_node", "log brain"},
      {"/rosout path_node", "log path"},
      {"/rosout drive_node", "log drive"},
  };
  t += QString("%1 %2 %3 %4\n").arg("topic", -12).arg("last", 8).arg("Hz", 6).arg("stamp lag", 11);
  const auto rx = qnode->rxSnapshot();
  for (const Row &r : rows)
  {
    const auto it = rx.find(r.key);
    if (it == rx.end())
    {
      t += QString("%1 %2\n").arg(r.name, -12).arg("-", 8);
      continue;
    }
    const QNode::RxInfo &i = it.value();
    const QString age = i.age_s < 10.0 ? QString::number(i.age_s, 'f', 2) + "s" : QString::number(i.age_s, 'f', 0) + "s";
    const QString lag = i.has_stamp ? QString::number(i.lat_ms, 'f', 0) + " ms" : QString("-");
    t += QString("%1 %2 %3 %4%5\n")
             .arg(r.name, -12)
             .arg(age, 8)
             .arg(i.hz, 6, 'f', 1)
             .arg(lag, 11)
             .arg(i.age_s > thr ? "  ← 끊김" : "");
  }
  t += "\nstamp lag = 받은 시각(PC) - 메시지 stamp(로봇). 로봇·PC 시계 차이 포함 (음수면 PC 시계가 느림)";
  label_debug_->setText(t);
}

void MainWindow::drawNoSignal(QLabel *label, const QString &stream, double age_s)
{
  const QSize sz = label->size();
  if (sz.width() < 8 || sz.height() < 8)
    return;
  QPixmap pm(sz);
  pm.fill(QColor(24, 24, 28));
  QPainter p(&pm);
  p.setRenderHint(QPainter::Antialiasing);
  // 가는 테두리 + 가운데 글자
  p.setPen(QPen(QColor(200, 60, 60), 2));
  p.drawRect(pm.rect().adjusted(1, 1, -2, -2));
  QFont f = p.font();
  f.setBold(true);
  f.setPixelSize(std::max(12, std::min(sz.width() / 8, sz.height() / 5)));
  p.setFont(f);
  p.setPen(QColor(220, 220, 220));
  QRect top = pm.rect();
  top.setBottom(sz.height() / 2 + f.pixelSize() / 2);
  p.drawText(top, Qt::AlignHCenter | Qt::AlignBottom, "NO SIGNAL");
  f.setBold(false);
  f.setPixelSize(std::max(10, f.pixelSize() / 3));
  p.setFont(f);
  p.setPen(QColor(150, 150, 150));
  QRect bottom = pm.rect();
  bottom.setTop(sz.height() / 2 + f.pixelSize() * 3);
  p.drawText(bottom, Qt::AlignHCenter | Qt::AlignTop,
             age_s < 0 ? stream + " · 수신 없음" : QString("%1 · 마지막 %2 s 전").arg(stream).arg(age_s, 0, 'f', 1));
  p.end();
  label->setPixmap(pm);
}
