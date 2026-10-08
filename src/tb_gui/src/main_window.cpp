/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 **/
#include "../include/tb_gui/main_window.hpp"

#include <QDir>
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
  ui->labelParamFile->setText("params: " + paramFile());
  connect(ui->btnSaveParams, &QPushButton::clicked, this, [this]
          { qnode->saveParams(paramFile()); });
  connect(ui->btnLoadParams, &QPushButton::clicked, this, [this]
          { qnode->loadParams(paramFile()); });
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
      setFocus(); });

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
}

// ───────── 주행 (WASD) ─────────

bool MainWindow::eventFilter(QObject *obj, QEvent *ev)
{
  const bool press = ev->type() == QEvent::KeyPress;
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
      {"live", "#2e9d4a"}, {"memory", "#8e44ad"}, {"color_rule", "#e67e22"}, {"lost", "#c0392b"}};
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
  ui->labelMission->setText(QString("turn: %1 | parking: %2")
                                .arg(names[std::clamp(turn_, 0, 2)])
                                .arg(parking_ ? "ON" : "off"));
}

QString MainWindow::paramFile() const
{
  return QDir::homePath() + "/tb_params.yaml";
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  stopDrive(); // 주행 중에 창을 닫으면 정지 명령 전송
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete qnode;
  delete ui;
}