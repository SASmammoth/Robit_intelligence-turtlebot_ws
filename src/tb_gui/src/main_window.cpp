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
#include <QScrollArea>
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

  // ====== 영상 표시 (라벨이 영상 크기로 늘어나 레이아웃이 커지는 것 방지)
  for (QLabel *d : {ui->displayUsb, ui->displayMask, ui->displayBev, ui->displayRoi, ui->displaySkel})
  {
    d->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    d->setMinimumSize(1, 1);
  }

  connect(qnode, &QNode::usbImageReceived, this, [this](const QImage &img)
          { showImage(ui->displayUsb, img); });
  connect(qnode, &QNode::maskImageReceived, this, [this](const QImage &img)
          { showImage(ui->displayMask, img); });
  connect(qnode, &QNode::bevImageReceived, this, [this](const QImage &img)
          { showImage(ui->displayBev, img); });
  connect(qnode, &QNode::roiImageReceived, this, [this](const QImage &img)
          { showImage(ui->displayRoi, img); });
  connect(qnode, &QNode::skelImageReceived, this, [this](const QImage &img)
          { showImage(ui->displaySkel, img); });

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
    if (name.startsWith("white.") || name.startsWith("yellow."))
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

  // ====== View 탭 (카메라 크게 + 키보드 조작)
  setupViewTab();
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
  for (QPushButton *b : {btn_off, btn_wasd, btn_auto})
  {
    b->setFocusPolicy(Qt::NoFocus); // 버튼이 Space를 먹지 않게
    row->addWidget(b);
  }
  box->insertLayout(1, row);
  connect(btn_off, &QPushButton::clicked, this, [this] { setDriveMode(DriveMode::Off); });
  connect(btn_wasd, &QPushButton::clicked, this, [this] { setDriveMode(DriveMode::Wasd); });
  connect(btn_auto, &QPushButton::clicked, this, [this] { setDriveMode(DriveMode::Auto); });

  label_drive_brief_ = new QLabel("drive: -", ui->groupStatus);
  box->insertWidget(2, label_drive_brief_);

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

  connect(check_construction_, &QCheckBox::toggled, this, [this](bool on)
          {
    construction_ = on;
    qnode->publishConstruction(on);
    showMission(); });
  connect(qnode, &QNode::constructionStateReceived, this, [this](bool on)
          {
    construction_state_ = on;
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
    return DriveMode::Auto;
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
  switch (m)
  {
  case DriveMode::Off:
    if (ui->checkAutoEnable->isChecked())
      setAutoDrive(false);
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
    setAutoDrive(true);
    break;
  }
  refreshViewStatus();
}

void MainWindow::refreshViewStatus()
{
  if (!label_mode_)
    return;
  static const char *text[] = {"정지", "WASD 수동", "자동 주행"};
  static const char *color[] = {"#555555", "#2980b9", "#2e9d4a"};
  const int i = static_cast<int>(driveMode());
  label_mode_->setText(QString("%1%2").arg(text[i], viewTabActive() ? "" : "   (키 조작은 View 탭에서)"));
  label_mode_->setStyleSheet(QString("font-size: 18px; font-weight: bold; border-radius: 4px; color: white; "
                                     "padding: 4px; background: %1;").arg(color[i]));
  label_keys_->setText(driveMode() == DriveMode::Auto
                           ? "Space: WASD로   A/D 누르는 동안: Left/Right   P 주차  C 공사  B 차단바   Esc 정지"
                           : "Space: 자동으로   W/A/S/D: 수동 주행   P 주차  C 공사  B 차단바   Esc 정지");
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
      setDriveMode(DriveMode::Off);
      setBarrier(false);
    }
    return true;
  case Qt::Key_P:
    if (press && !repeat)
      ui->checkParking->toggle();
    return true;
  case Qt::Key_C:
    if (press && !repeat)
      check_construction_->toggle();
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
  connect(sp, qOverload<int>(&QSpinBox::valueChanged), this, [this, name](int v)
          { queueParam(name, v); });
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
           {"path.min_len_mm", "최소 경로 길이", " mm", 0, 1000, 10},
           {"path.branch_look_mm", "분기 미리보기", " mm", 0, 300, 5},
           {"path.blind_max_mm", "사각지대 통과 최대", " mm", 0, 1000, 10},
           {"path.fill_blind", "사각지대 채움 (0끔/1직진만/2항상)", "", 0, 2, 1},
           {"path.fill_join_mm", "채움 연결 허용", " mm", 0, 200, 5},
       }},
      {"공사 구간 (path_node) — 표지판 또는 View 탭 C키",
       {
           {"path.con_brick_depth_mm", "벽돌 벽 앞뒤", " mm", 0, 500, 10},
           {"path.con_brick_margin_mm", "벽돌 벽 좌우 여유", " mm", 0, 300, 5},
           {"path.con_brick_w_mm", "벽돌 실제 가로", " mm", 0, 600, 10},
           {"path.con_edge_mm", "화면 끝까지 막기", " mm", 0, 300, 10},
           {"path.con_exit_mm", "벽돌 없이 이만큼 가면 끝", " mm", 0, 3000, 50},
           {"path.con_sign_frames", "표지판 연속 프레임", "", 1, 30, 1},
       }},
  };
  constexpr int COLS = 3;

  auto *page = new QWidget;
  auto *vbox = new QVBoxLayout(page);
  for (const auto &g : groups)
  {
    auto *box = new QGroupBox(g.title, page);
    auto *grid = new QGridLayout(box);
    int i = 0;
    for (const auto &d : g.defs)
      addParamSpin(grid, box, i++, COLS, d);
    grid->setColumnStretch(COLS * 2, 1);
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
      {"drive.v_min_mm_s", "최저 속도", " mm/s", 0, 500, 5},
      {"drive.w_max_deg_s", "최대 회전", " °/s", 0, 720, 5},
      {"drive.accel_mm_s2", "가속", " mm/s²", 10, 5000, 50},
      {"drive.decel_mm_s2", "감속", " mm/s²", 10, 5000, 50},
      {"drive.w_accel_deg_s2", "회전 가속", " °/s²", 10, 5000, 30},
      {"drive.ld_min_mm", "lookahead 최소", " mm", 30, 1000, 10},
      {"drive.ld_max_mm", "lookahead 최대", " mm", 30, 1500, 10},
      {"drive.ld_time_ms", "lookahead 시간", " ms", 0, 5000, 100},
      {"drive.ld_sharp_mm", "급한 곳 lookahead", " mm", 0, 1000, 10},
      {"drive.sharp_turn_deg", "급한 곳 판단 각도", " °", 1, 180, 5},
      {"drive.sharp_hold_ms", "급한 곳 유지 시간", " ms", 0, 5000, 100},
      {"drive.curv_slow_mm", "커브 감속", " mm", 0, 1000, 10},
      {"drive.stop_dist_mm", "경로 끝 정지", " mm", 0, 300, 5},
      {"drive.hold_ms", "경로 유지", " ms", 0, 3000, 50},
      {"drive.path_timeout_ms", "경로 유효", " ms", 50, 3000, 50},
      {"drive.cmd_timeout_ms", "명령 유효", " ms", 50, 3000, 50},
      {"drive.weak_src_pct", "약한 경로 속도", " %", 0, 100, 5},
      {"drive.con_v_mm_s", "공사 구간 속도", " mm/s", 0, 1000, 10},
      {"drive.con_ld_min_mm", "공사 ld_min", " mm", 0, 1000, 10},
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
  ui->labelDriveInfo->setText(QString("src: %1   ld: %2 m   end: %3 m")
                                  .arg(kv.value("src", "-"), kv.value("ld", "-"), kv.value("end", "-")));
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
  if (w.spin && !w.spin->hasFocus())
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
  QString cons = construction_ ? "ON" : "off";
  if (construction_state_ != construction_)
    cons += construction_state_ ? " (자동 진입)" : " (path_node 꺼짐)";
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