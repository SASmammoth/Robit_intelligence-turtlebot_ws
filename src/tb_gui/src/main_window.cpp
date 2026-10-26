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
#include <algorithm>
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
}

void MainWindow::makeTabsScrollable()
{
  QTabWidget *tw = ui->tabWidget;
  tw->setUsesScrollButtons(true); // 탭이 많으면 탭 이름 줄도 좌우로 넘김
  for (int i = 0; i < tw->count(); ++i)
  {
    QWidget *page = tw->widget(i);
    if (qobject_cast<QScrollArea *>(page))
      continue; // Params 탭처럼 이미 스크롤
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
  for (QWidget *w : std::initializer_list<QWidget *>{check_construction_, check_barrier_, ui->checkParking,
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

  // 가짜 표지판 → tb_brain (/brain/inject). 필터 없이 바로 그 표지판 동작 (Brain 모드에서만 의미 있음)
  //   중간에서 시작할 때 초록불로 출발, 표지판을 놓쳤을 때 대신 보내기 등
  auto *inject_box = new QGroupBox("가짜 표지판 → brain", ui->groupStatus);
  auto *inject_grid = new QGridLayout(inject_box);
  inject_grid->setContentsMargins(4, 4, 4, 4);
  inject_grid->setSpacing(3);
  static const std::pair<const char *, const char *> fake_signs[] = {
      {"tl_green", "초록불 (출발)"}, {"left", "좌회전"}, {"right", "우회전"},
      {"construction", "공사"}, {"parking", "주차"}, {"barrier_closed", "차단바 닫힘"},
      {"barrier_open", "차단바 열림"}};
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
  box->insertWidget(park_idx + 5, inject_box);
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
                           ? "Space: WASD로   A/D 누르는 동안: Left/Right   P 주차  C 공사 켜기  X 공사 끝  B 차단바   Enter: Brain   Esc 정지"
                       : dm == DriveMode::Brain
                           ? "Brain이 /drive/cmd·/mission/* 를 냄 (수동 조작 자제)   Space: WASD로   B 차단바   Enter·Esc: 정지"
                           : "Space: 자동으로   W/A/S/D: 수동 주행   P 주차  C 공사 켜기  X 공사 끝  B 차단바   Enter: Brain   Esc 정지");
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

// (이름 라벨, 스핀박스) 한 쌍을 grid의 idx번째 칸에 넣고 노드와 연결
void MainWindow::addParamSpin(QGridLayout *grid, QWidget *parent, int idx, int cols, const ParamSpinDef &d)
{
  const QString name = d.name;
  auto *label = new QLabel(d.text, parent);
  label->setToolTip(name);
  auto *sp = new QSpinBox(parent);
  sp->setRange(d.min, d.max);
  sp->setSingleStep(d.step);
  sp->setSuffix(d.suffix);
  sp->setKeyboardTracking(false);
  sp->setToolTip(name);
  grid->addWidget(label, idx / cols, (idx % cols) * 2);
  grid->addWidget(sp, idx / cols, (idx % cols) * 2 + 1);
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
}

// ───────── Params 탭 (실측 거리, 경로 계산) ─────────
// Tab 1에 이미 있는 값(HSV, BEV 사다리꼴, morph, bev.lane_w_px, skel.seal/prune/dash)은 넣지 않음
// (한 파라미터에 위젯이 둘이면 동기화가 꼬임)

void MainWindow::setupParamTab()
{
  struct Group
  {
    const char *title;
    std::vector<ParamSpinDef> defs;
    int cols = 3; // (이름, 스핀박스) 묶음 몇 개씩 한 줄
  };
  const std::vector<Group> groups = {
      {"실측 거리 (lineDetect_node) ★ 자로 잰 값",
       {
           {"geo.lane_w_mm", "차선 안쪽 간격", " mm", 1, 1000, 1},
           {"geo.bev_bottom_mm", "BEV 아랫줄 거리", " mm", 0, 2000, 1},
           {"geo.bev_depth_mm", "BEV 세로 길이", " mm", 1, 3000, 1},
       }},
      {"스켈레톤 (path_node)",
       {
           {"skel.max_lane_thick", "차선 최대 두께", " px", 0, 200, 1},
       }},
      {"벽돌 (path_node)",
       {
           {"path.brick_margin_mm", "좌우 여유", " mm", 0, 300, 5},
           {"path.brick_depth_mm", "앞뒤 길이", " mm", 0, 500, 5},
           {"path.brick_gap_mm", "선까지 틈 막기", " mm", 0, 300, 10},
           {"path.det_max_age_ms", "탐지 유효", " ms", 0, 2000, 50},
       }},
      {"경로 기억·판단 (path_node)",
       {
           {"path.mem_travel_mm", "기억 최대 이동", " mm", 0, 3000, 50},
           {"path.max_dev_mm", "기억과 차이 허용", " mm", 0, 500, 5},
           {"path.override_frames", "새 경로 믿기까지", " 프레임", 0, 100, 1},
           {"path.turn_drop_memory", "좌/우 명령 들어오면 기억 버림 (1=켬)", "", 0, 1, 1},
           {"path.min_len_mm", "최소 경로 길이", " mm", 0, 1000, 10},
           {"path.branch_look_mm", "분기 미리보기", " mm", 0, 300, 5},
           {"path.blind_max_mm", "사각지대 통과 최대", " mm", 0, 1000, 10},
           {"path.fill_blind", "사각지대 채움 (0끔/1직진만/2항상)", "", 0, 2, 1},
           {"path.fill_join_mm", "채움 연결 허용", " mm", 0, 200, 5},
           {"path.turn_radius_mm", "교차로 회전 반지름 (0=끔)", " mm", 0, 500, 10},
           {"path.turn_min_deg", "둥글게 할 꺾임", " °", 0, 180, 5},
       }},
      {"공사 구간 (path_node) — 표지판 또는 View 탭 C키",
       {
           {"path.con_brick_depth_mm", "벽돌 벽 앞뒤", " mm", 0, 500, 10},
           {"path.con_brick_margin_mm", "벽돌 벽 좌우 여유", " mm", 0, 300, 5},
           {"path.con_brick_w_mm", "벽돌 실제 가로", " mm", 0, 600, 10},
           {"path.con_smooth_mm", "경로 모서리 둥글게", " mm", 0, 300, 10},
           {"path.con_attach_mm", "붙은 쪽 선까지 막기", " mm", 0, 600, 10},
           {"path.con_extend_mm", "가려진 선 잇기", " mm", 0, 1000, 10},
           {"path.con_edge_mm", "화면 끝까지 막기", " mm", 0, 300, 10},
           {"path.con_dead_end_mm", "막다른 가지 지우기", " mm", 0, 1000, 10},
           {"path.con_keep", "지난 벽돌 (0끔 1약 2강)", "", 0, 2, 1},
           {"path.con_keep_rear_mm", "지난 벽돌 축 뒤 길이", " mm", 0, 300, 10},
           {"path.con_hold_mm", "지난 벽돌 더 피하기", " mm", 0, 500, 10},
           {"path.con_clear_mm", "지난 벽돌 옆 여유", " mm", 0, 200, 5},
           {"path.con_exit_mm", "벽돌 본 뒤 안 보이고 이만큼 가면 끝 (0=끔)", " mm", 0, 3000, 50},
           {"path.con_no_brick_mm", "벽돌 못 보고 이만큼 가면 끝 (0=끔)", " mm", 0, 10000, 100},
           {"path.con_override_frames", "공사: 새 경로 믿기까지", " 프레임", 0, 100, 1},
           {"path.con_straight_by_lane", "공사: 직진 가지를 차선 방향으로 (1=켬)", "", 0, 1, 1},
           {"path.con_offscreen_side", "공사: 화면 끝 벽돌은 화면 밖 선에 붙음 (1=켬)", "", 0, 1, 1},
       }},
      {"미션 동작 (brain_node) — 공사 끝 모드 1·2를 쓰면 위 path_node의 '벽돌 본 뒤…끝', '벽돌 못 보고…끝'을 0으로",
       {
           {"mission.turn_ms", "좌/우 표지판 → turn 유지", " ms", 0, 30000, 500},
           {"mission.rearm_ms", "같은 표지판 다시 받기까지", " ms", 0, 30000, 250},
           {"mission.con_end_mode", "공사 끝: 0 벽돌(path) 1 시간 2 주차표지판", "", 0, 2, 1},
           {"mission.con_hold_ms", "공사 유지 시간 (모드 1)", " ms", 0, 120000, 500},
           {"mission.con_once", "공사 한 번만 (주차 표지판까지 재진입 막기)", "", 0, 1, 1},
           {"mission.parking_pause", "주차 표지판에서 정지 (1=정지)", "", 0, 1, 1},
           {"mission.barrier_release_ms", "차단바 안 보이면 정지 풀기 (0=끔)", " ms", 0, 30000, 250},
           {"mission.start_follow", "시작하자마자 출발 (1=초록불 안 기다림)", "", 0, 1, 1},
       },
       2},
      {"표지판 검출 필터 (brain_node) — 원본 640x480 기준, 0 = 끔. 높이·너비·면적을 모두 넘어야 동작",
       {
           {"sign.barrier_closed.min_h_px", "차단바 닫힘 최소 높이", " px", 0, 480, 5},
           {"sign.barrier_closed.min_w_px", "차단바 닫힘 최소 너비", " px", 0, 640, 5},
           {"sign.barrier_closed.min_area_px", "차단바 닫힘 최소 면적 (w×h)", " px²", 0, 307200, 500},
           {"sign.barrier_open.min_h_px", "차단바 열림 최소 높이", " px", 0, 480, 5},
           {"sign.barrier_open.min_w_px", "차단바 열림 최소 너비", " px", 0, 640, 5},
           {"sign.barrier_open.min_area_px", "차단바 열림 최소 면적 (w×h)", " px²", 0, 307200, 500},
           {"sign.construction.min_h_px", "공사 최소 높이", " px", 0, 480, 5},
           {"sign.construction.min_w_px", "공사 최소 너비", " px", 0, 640, 5},
           {"sign.construction.min_area_px", "공사 최소 면적 (w×h)", " px²", 0, 307200, 500},
           {"sign.left.min_h_px", "좌회전 최소 높이", " px", 0, 480, 5},
           {"sign.left.min_w_px", "좌회전 최소 너비", " px", 0, 640, 5},
           {"sign.left.min_area_px", "좌회전 최소 면적 (w×h)", " px²", 0, 307200, 500},
           {"sign.right.min_h_px", "우회전 최소 높이", " px", 0, 480, 5},
           {"sign.right.min_w_px", "우회전 최소 너비", " px", 0, 640, 5},
           {"sign.right.min_area_px", "우회전 최소 면적 (w×h)", " px²", 0, 307200, 500},
           {"sign.parking.min_h_px", "주차 최소 높이", " px", 0, 480, 5},
           {"sign.parking.min_w_px", "주차 최소 너비", " px", 0, 640, 5},
           {"sign.parking.min_area_px", "주차 최소 면적 (w×h)", " px²", 0, 307200, 500},
           {"sign.tl_green.min_h_px", "초록불 최소 높이", " px", 0, 480, 5},
           {"sign.tl_green.min_w_px", "초록불 최소 너비", " px", 0, 640, 5},
           {"sign.tl_green.min_area_px", "초록불 최소 면적 (w×h)", " px²", 0, 307200, 500},
           {"sign.confirm_frames", "연속 프레임 (한 프레임 오검출 거르기)", "", 1, 30, 1},
           {"sign.min_score_pct", "최소 점수", " %", 0, 100, 5},
       },
       3},
  };

  auto *page = new QWidget;
  auto *vbox = new QVBoxLayout(page);
  for (const auto &g : groups)
  {
    auto *box = new QGroupBox(g.title, page);
    auto *grid = new QGridLayout(box);
    int i = 0;
    for (const auto &d : g.defs)
      addParamSpin(grid, box, i++, g.cols, d);
    grid->setColumnStretch(g.cols * 2, 1);
    vbox->addWidget(box);
  }
  auto *hint = new QLabel("이름에 마우스를 올리면 ROS 파라미터 이름이 보입니다. "
                          "값은 바로 노드에 적용되고, 유지하려면 Save로 파일에 저장하세요.",
                          page);
  hint->setWordWrap(true);
  hint->setStyleSheet("color: gray;");
  vbox->addWidget(hint);
  vbox->addStretch(1);

  auto *scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setWidget(page);
  ui->tabWidget->addTab(scroll, "Params");
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
  // 파라미터 스핀박스: 이름, 표시 이름, 단위, 범위 (tb_drive의 declare_int 범위와 같게)
  static const ParamSpinDef defs[] = {
      {"drive.v_max_mm_s", "최고 속도", " mm/s", 0, 1000, 10},
      {"drive.v_curve_mm_s", "코너 속도 (0=끔)", " mm/s", 0, 1000, 10},
      {"drive.curve_start_deg", "코너 감속 시작 각도", " °", 0, 180, 5},
      {"drive.curve_full_deg", "코너 속도 도달 각도", " °", 1, 180, 5},
      {"drive.v_min_mm_s", "최저 속도", " mm/s", 0, 500, 5},
      {"drive.w_max_deg_s", "최대 회전", " °/s", 0, 720, 5},
      {"drive.accel_mm_s2", "가속", " mm/s²", 10, 5000, 50},
      {"drive.decel_mm_s2", "감속", " mm/s²", 10, 5000, 50},
      {"drive.w_accel_deg_s2", "회전 가속", " °/s²", 10, 5000, 30},
      {"drive.ld_min_mm", "lookahead 최소", " mm", 30, 1000, 10},
      {"drive.ld_max_mm", "lookahead 최대", " mm", 30, 1500, 10},
      {"drive.ld_time_ms", "lookahead 시간", " ms", 0, 5000, 100},
      {"drive.ld_sharp_mm", "급한 곳 lookahead", " mm", 0, 1000, 10},
      {"drive.sharp_turn_deg", "교차로: 급한 곳 판단 각도", " °", 1, 180, 5},
      {"drive.sharp_hold_ms", "급한 곳 유지 시간", " ms", 0, 5000, 100},
      {"drive.sharp_hold_mm", "급한 곳 유지 거리 (0=시간)", " mm", 0, 2000, 10},
      {"drive.sharp_curve_start_deg", "일반 곡선: 급한 곳 시작 각도", " °", 0, 180, 5},
      {"drive.sharp_curve_full_deg", "일반 곡선: 급한 곳 최대 각도", " °", 1, 180, 5},
      {"drive.curve_x_offset_mm", "일반 곡선 거리 보정", " mm", -200, 200, 5},
      {"drive.curv_slow_mm", "커브 감속", " mm", 0, 1000, 10},
      {"drive.stop_dist_mm", "경로 끝 정지", " mm", 0, 300, 5},
      {"drive.hold_ms", "경로 유지", " ms", 0, 3000, 50},
      {"drive.path_timeout_ms", "경로 유효", " ms", 50, 3000, 50},
      {"drive.cmd_timeout_ms", "명령 유효", " ms", 50, 3000, 50},
      {"drive.weak_src_pct", "약한 경로 속도", " %", 0, 100, 5},
      {"drive.con_v_mm_s", "공사 구간 속도", " mm/s", 0, 1000, 10},
      {"drive.con_ld_min_mm", "공사 ld_min", " mm", 0, 1000, 10},
      {"drive.con_ld_sharp_mm", "공사 ld_sharp (0=끔)", " mm", 0, 1000, 10},
      {"drive.con_turn_gain_pct", "공사 회전 세기", " %", 50, 300, 5},
      {"drive.con_pivot_enter_deg", "공사 피봇 시작", " °", 0, 180, 5},
      {"drive.con_pivot_keep_deg", "공사 피봇 맡김", " °", 0, 180, 5},
      {"drive.psd.stop_mm", "PSD 정지 거리", " mm", 0, 1000, 10},
      {"drive.control_hz", "제어 주기", " Hz", 5, 100, 5},
      {"drive.pivot_enter_deg", "피봇 시작 각도", " °", 0, 180, 5},
      {"drive.pivot_exit_deg", "피봇 종료 각도", " °", 0, 90, 1},
      {"drive.pivot_w_deg_s", "피봇 회전 속도", " °/s", 0, 360, 5},
      {"drive.pivot_fallback_deg", "피봇 안전장치 각도", " °", 0, 180, 5},
      {"drive.corner_min_x_mm", "코너 검출 시작 거리", " mm", 0, 1000, 10},
      {"drive.pivot_keep_deg", "피봇 남길 각도", " °", 0, 180, 5},
      {"drive.pivot_confirm", "피봇 확인 횟수", " 회", 1, 30, 1},
  };
  constexpr int COLS = 3; // (이름, 스핀박스) 묶음 3개씩 한 줄

  int i = 0;
  for (const auto &d : defs)
    addParamSpin(ui->gridDriveParams, ui->groupDriveParams, i++, COLS, d);
  ui->gridDriveParams->setRowStretch((i + COLS - 1) / COLS, 1);

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
  dg->setColumnStretch(0, 4);
  dg->setColumnStretch(1, 3);
  dg->setColumnStretch(2, 3);
  display_tabs_->addTab(drive_page, "주행");

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

// 같은 영상을 보여주는 라벨 중 지금 화면에 보이는 것만 그림 (숨은 탭은 크기 조정 안 함)
void MainWindow::showStream(const QString &stream, const QImage &img)
{
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
  if (ui->checkAutoEnable->isChecked())
    qnode->setBoolParam("drive.enable", false); // 테스트 중 GUI를 닫으면 자동 주행도 끔
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete qnode;
  delete ui;
}