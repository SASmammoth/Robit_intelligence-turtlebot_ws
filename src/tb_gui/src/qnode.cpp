/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 **/
#include "../include/tb_gui/qnode.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
#include <QDateTime>
#include <QStringList>

// 압축 이미지 → QImage (실패 시 null)
static QImage toQImage(const sensor_msgs::msg::CompressedImage &msg)
{
  cv::Mat mat = cv::imdecode(msg.data, cv::IMREAD_COLOR);
  if (mat.empty())
    return {};
  return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_BGR888).copy();
}

static int64_t now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

static const char *LINE_NODE = "/lineDetect_node";
static const char *PATH_NODE = "/path_node";
static const char *DRIVE_NODE = "/drive_node";

// 파일 저장/불러오기에서 빼는 파라미터 (불러오자마자 로봇이 움직이면 안 됨)
static bool isUnsavedParam(const std::string &name)
{
  return name == "drive.enable";
}

QNode::QNode()
{
  int argc = 0;
  char **argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("tb_gui");

  // ====== 영상 구독 (토픽 → 시그널)
  auto sub = [this](const std::string &topic, void (QNode::*sig)(const QImage &))
  {
    return node->create_subscription<sensor_msgs::msg::CompressedImage>(
        topic, rclcpp::SensorDataQoS(),
        [this, sig](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
        {
          QImage img = toQImage(*msg);
          if (!img.isNull())
            Q_EMIT(this->*sig)(img);
        });
  };

  // USB 화면: YOLO 결과 영상이 오면 그걸, 0.5초 이상 끊기면 원본 카메라
  sign_sub_ = node->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/signs/debug/compressed", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
      {
        last_sign_ns_ = now_ns();
        QImage img = toQImage(*msg);
        if (!img.isNull())
          Q_EMIT usbImageReceived(img);
      });
  usb_sub_ = node->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image_raw/compressed", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
      {
        if (now_ns() - last_sign_ns_ < 500'000'000)
          return;
        QImage img = toQImage(*msg);
        if (!img.isNull())
          Q_EMIT usbImageReceived(img);
      });

  mask_sub_ = sub("/lane/debug/mask/compressed", &QNode::maskImageReceived);
  bev_sub_ = sub("/lane/debug/bev/compressed", &QNode::bevImageReceived);
  roi_sub_ = sub("/lane/debug/roi/compressed", &QNode::roiImageReceived);
  skel_sub_ = sub("/lane/debug/skel/compressed", &QNode::skelImageReceived);

  // ====== 상태
  path_src_sub_ = node->create_subscription<std_msgs::msg::String>(
      "/lane/path_source", 10, [this](const std_msgs::msg::String::ConstSharedPtr m)
      { Q_EMIT pathSourceReceived(QString::fromStdString(m->data)); });

  det_sub_ = node->create_subscription<tb_interfaces::msg::DetectionArray>(
      "/signs/detections", 10, [this](const tb_interfaces::msg::DetectionArray::ConstSharedPtr m)
      { onDetections(*m); });

  // 다른 노드(나중의 판단 노드)가 미션을 바꿔도 화면에 반영
  turn_sub_ = node->create_subscription<std_msgs::msg::UInt8>(
      "/mission/turn", 10, [this](const std_msgs::msg::UInt8::ConstSharedPtr m)
      { Q_EMIT turnReceived(m->data); });
  parking_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/mission/parking", 10, [this](const std_msgs::msg::Bool::ConstSharedPtr m)
      { Q_EMIT parkingReceived(m->data); });

  turn_pub_ = node->create_publisher<std_msgs::msg::UInt8>("/mission/turn", 10);
  parking_pub_ = node->create_publisher<std_msgs::msg::Bool>("/mission/parking", 10);
  construction_pub_ = node->create_publisher<std_msgs::msg::Bool>("/mission/construction", 10);
  construction_state_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/lane/construction", 10, [this](const std_msgs::msg::Bool::ConstSharedPtr m)
      { Q_EMIT constructionStateReceived(m->data); });
  construction_exit_pub_ = node->create_publisher<std_msgs::msg::Bool>("/mission/construction_exit", 10);
  construction_info_sub_ = node->create_subscription<std_msgs::msg::String>(
      "/lane/construction_info", 10, [this](const std_msgs::msg::String::ConstSharedPtr m)
      { Q_EMIT constructionInfoReceived(QString::fromStdString(m->data)); });
  drive_cmd_pub_ = node->create_publisher<tb_interfaces::msg::DriveCmd>("/drive/cmd", 10);

  // ====== 주행 상태 (tb_drive, 10Hz 문자열)
  drive_state_sub_ = node->create_subscription<std_msgs::msg::String>(
      "/drive/state", 10, [this](const std_msgs::msg::String::ConstSharedPtr m)
      { Q_EMIT driveStateReceived(QString::fromStdString(m->data)); });

  // ====== 경로 보기: path_node가 낸 경로, drive가 고른 목표점
  //  (/drive/target은 구독자가 있을 때만 발행되므로 여기서 구독하면 켜짐)
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      "/lane/path", 10, [this](const nav_msgs::msg::Path::ConstSharedPtr m)
      {
        QPolygonF pts;
        pts.reserve(static_cast<int>(m->poses.size()));
        for (const auto &ps : m->poses)
          pts << QPointF(ps.pose.position.x, ps.pose.position.y);
        Q_EMIT pathReceived(pts); });
  target_sub_ = node->create_subscription<geometry_msgs::msg::PointStamped>(
      "/drive/target", 10, [this](const geometry_msgs::msg::PointStamped::ConstSharedPtr m)
      { Q_EMIT targetReceived(QPointF(m->point.x, m->point.y)); });
  // 모터로 실제 나간 "velocity L R" (drive_node가 보낸 것 + GUI WASD 것 모두)
  uart_cmd_sub_ = node->create_subscription<std_msgs::msg::String>(
      "TB_Uart_RX", 10, [this](const std_msgs::msg::String::ConstSharedPtr m)
      { Q_EMIT uartCommandSeen(QString::fromStdString(m->data)); });

  // ====== UART (tb_uart_node)
  uart_pub_ = node->create_publisher<std_msgs::msg::String>("TB_Uart_RX", 10);
  uart_sub_ = node->create_subscription<std_msgs::msg::String>(
      "TB_Uart_TX", 10, [this](const std_msgs::msg::String::ConstSharedPtr m)
      { Q_EMIT uartReceived(QString::fromStdString(m->data)); });

  // ====== 파라미터 클라이언트 (노드별)
  line_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, LINE_NODE);
  path_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, PATH_NODE);
  drive_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, DRIVE_NODE);

  // 세 노드의 파라미터 변경을 실시간으로 받음
  param_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(node);
  param_event_cb_handle_ = param_event_handler_->add_parameter_event_callback(
      [this](const rcl_interfaces::msg::ParameterEvent &event)
      {
        if (event.node != LINE_NODE && event.node != PATH_NODE && event.node != DRIVE_NODE)
          return;
        for (const auto *list : {&event.new_parameters, &event.changed_parameters})
          for (const auto &p : *list)
          {
            const QString name = QString::fromStdString(p.name);
            if (p.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
              Q_EMIT paramLoaded(name, static_cast<int>(p.value.integer_value));
            else if (p.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
              Q_EMIT boolParamLoaded(name, p.value.bool_value);
            else if (p.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
              Q_EMIT doubleParamLoaded(name, p.value.double_value);
          }
      });

  judge_timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this]
                                         { judgeSigns(); });
  this->start();
}

QNode::~QNode()
{
  if (rclcpp::ok())
    rclcpp::shutdown();
  wait();
}

void QNode::run()
{
  rclcpp::spin(node);
  Q_EMIT rosShutDown();
}

// ───────── 파라미터 ─────────

bool QNode::isPathParam(const QString &n)
{
  return n.startsWith("skel.") || n.startsWith("path.");
}

bool QNode::isDriveParam(const QString &n)
{
  return n.startsWith("drive.");
}

std::vector<std::string> QNode::lineParamNames()
{
  std::vector<std::string> names;
  for (const char *t : {"white", "yellow", "red"})
    for (const char *k : {"h_min", "h_max", "s_min", "s_max", "v_min", "v_max"})
      names.push_back(std::string(t) + "." + k);
  for (const char *n : {"morph.open", "morph.close", "morph.min_area",
                        "bev.top_y", "bev.bot_y", "bev.top_w", "bev.bot_w", "bev.lane_w_px",
                        "geo.lane_w_mm", "geo.bev_depth_mm", "geo.bev_bottom_mm"})
    names.push_back(n);
  return names;
}

std::vector<std::string> QNode::pathParamNames()
{
  return {"skel.seal_len", "skel.prune_len", "skel.dash_len", "skel.max_lane_thick",
          "path.brick_depth_mm", "path.brick_margin_mm", "path.brick_gap_mm", "path.det_max_age_ms", "path.mem_travel_mm",
          "path.max_dev_mm", "path.override_frames", "path.min_len_mm",
          "path.branch_look_mm", "path.blind_max_mm", "path.fill_blind", "path.fill_join_mm", "path.turn_radius_mm", "path.turn_min_deg",
          "path.con_brick_depth_mm", "path.con_brick_margin_mm", "path.con_brick_w_mm", "path.con_smooth_mm", "path.con_attach_mm", "path.con_extend_mm", "path.con_edge_mm", "path.con_dead_end_mm", "path.con_keep", "path.con_keep_rear_mm", "path.con_hold_mm", "path.con_clear_mm",
          "path.con_exit_mm", "path.con_sign_frames", "path.con_sign_min_h_px", "path.con_no_brick_mm"};
}

std::vector<std::string> QNode::driveParamNames(bool with_enable)
{
  std::vector<std::string> names = {
      "drive.require_cmd", "drive.latency_comp", "drive.psd.enable", // bool
      "drive.control_hz",
      "drive.v_max_mm_s", "drive.v_curve_mm_s", "drive.curve_start_deg", "drive.curve_full_deg",
      "drive.v_min_mm_s", "drive.w_max_deg_s",
      "drive.accel_mm_s2", "drive.decel_mm_s2", "drive.w_accel_deg_s2",
      "drive.ld_min_mm", "drive.ld_max_mm", "drive.ld_time_ms", "drive.ld_sharp_mm", "drive.sharp_turn_deg", "drive.sharp_hold_ms", "drive.sharp_hold_mm", "drive.curv_slow_mm", "drive.stop_dist_mm",
      "drive.sharp_curve_start_deg", "drive.sharp_curve_full_deg", "drive.curve_x_offset_mm",
      "drive.path_timeout_ms", "drive.hold_ms", "drive.cmd_timeout_ms", "drive.weak_src_pct", "drive.con_v_mm_s",
      "drive.con_ld_min_mm", "drive.con_ld_sharp_mm", "drive.con_turn_gain_pct", "drive.con_pivot_enter_deg", "drive.con_pivot_keep_deg",
      "drive.pivot_enter_deg", "drive.pivot_keep_deg", "drive.pivot_confirm", "drive.pivot_exit_deg", "drive.pivot_w_deg_s", "drive.pivot_fallback_deg", "drive.corner_min_x_mm",
      "drive.psd.stop_mm"};
  if (with_enable)
    names.insert(names.begin(), "drive.enable");
  return names;
}

// 하드웨어 값 (drive.yaml에서 정함). 경로 보기 창의 바퀴 그림·속도 계산에만 씀
std::vector<std::string> QNode::driveDoubleNames()
{
  return {"drive.uart.wheel_sep_m", "drive.uart.pivot_sep_m"};
}

void QNode::setParams(const QMap<QString, int> &params)
{
  std::vector<rclcpp::Parameter> line, path, drive;
  for (auto it = params.begin(); it != params.end(); ++it)
  {
    auto &dst = isDriveParam(it.key()) ? drive : isPathParam(it.key()) ? path
                                                                       : line;
    dst.emplace_back(it.key().toStdString(), it.value());
  }

  sendParams(line_client_, LINE_NODE, line);
  sendParams(path_client_, PATH_NODE, path);
  sendParams(drive_client_, DRIVE_NODE, drive);
}

// 보내고 결과 확인: 노드가 거절하면(범위 밖, 없는 이름 등) 알리고 노드 값을 다시 읽어 화면을 되돌림
void QNode::sendParams(rclcpp::AsyncParametersClient::SharedPtr client, const char *node_name,
                       const std::vector<rclcpp::Parameter> &ps)
{
  if (ps.empty())
    return;
  if (!client->service_is_ready())
  {
    Q_EMIT paramMessage(QString("%1 연결 안 됨: 값이 적용되지 않았습니다").arg(QString(node_name)));
    return;
  }
  std::vector<std::string> names;
  for (const auto &p : ps)
    names.push_back(p.get_name());
  client->set_parameters(ps, [this, client, names, node_name](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f)
                         {
    QStringList bad;
    try {
      const auto res = f.get();
      for (size_t i = 0; i < res.size() && i < names.size(); ++i)
        if (!res[i].successful)
          bad << QString("%1 (%2)").arg(QString::fromStdString(names[i]), QString::fromStdString(res[i].reason));
    } catch (const std::exception &e) {
      bad << QString("응답 없음: %1").arg(e.what());
    }
    if (!bad.isEmpty())
      Q_EMIT paramMessage(QString("%1 설정 거절: %2").arg(QString(node_name), bad.join(", ")));
    requestFrom(client, names); }); // 노드가 실제로 가진 값으로 화면을 맞춤
}

void QNode::requestFrom(rclcpp::AsyncParametersClient::SharedPtr client, const std::vector<std::string> &names)
{
  client->get_parameters(names, [this, client, names](std::shared_future<std::vector<rclcpp::Parameter>> f)
                         {
    size_t got = 0;
    try {
      const auto ps = f.get();
      got = ps.size();
      for (const auto &p : ps)
      {
        const QString name = QString::fromStdString(p.get_name());
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
          Q_EMIT paramLoaded(name, static_cast<int>(p.as_int()));
        else if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
          Q_EMIT boolParamLoaded(name, p.as_bool());
        else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          Q_EMIT doubleParamLoaded(name, p.as_double());
      }
    } catch (const std::exception &) {
    }
    // 노드에 없는 이름(Nano가 구버전 등)이 하나라도 섞이면 노드가 빈 응답을 줌 → 하나씩 다시 (나머지는 보이게)
    if (got < names.size() && names.size() > 1)
      for (const auto &n : names)
        requestFrom(client, {n}); });
}

void QNode::refreshParams()
{
  // 노드 재시작, ros2 param set, launch 파일로 바뀐 값도 화면에 반영 (노드마다 요청 한 번)
  auto each = [this](rclcpp::AsyncParametersClient::SharedPtr c, const std::vector<std::string> &names)
  {
    if (c->service_is_ready())
      requestFrom(c, names);
  };
  each(line_client_, lineParamNames());
  each(path_client_, pathParamNames());
  each(drive_client_, driveParamNames(true));
  each(drive_client_, driveDoubleNames());
}

bool QNode::setBoolParam(const QString &name, bool on)
{
  if (!isDriveParam(name))
    return false;
  if (!drive_client_->service_is_ready())
  {
    RCLCPP_WARN(node->get_logger(), "%s 설정 실패: /drive_node 연결 안 됨", name.toStdString().c_str());
    return false;
  }
  drive_client_->set_parameters({rclcpp::Parameter(name.toStdString(), on)});
  return true;
}

bool QNode::requestParams()
{
  auto request = [this](rclcpp::AsyncParametersClient::SharedPtr client, const std::vector<std::string> &names)
  {
    client->get_parameters(names, [this](std::shared_future<std::vector<rclcpp::Parameter>> f)
                           {
      try {
        for (const auto &p : f.get())
        {
          const QString name = QString::fromStdString(p.get_name());
          if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            Q_EMIT paramLoaded(name, static_cast<int>(p.as_int()));
          else if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            Q_EMIT boolParamLoaded(name, p.as_bool());
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN(node->get_logger(), "파라미터 읽기 실패: %s", e.what());
      } });
  };

  if (!line_requested_ && line_client_->service_is_ready())
  {
    request(line_client_, lineParamNames());
    line_requested_ = true;
  }
  if (!path_requested_ && path_client_->service_is_ready())
  {
    request(path_client_, pathParamNames());
    path_requested_ = true;
  }
  if (!drive_requested_ && drive_client_->service_is_ready())
  {
    request(drive_client_, driveParamNames(true));
    drive_requested_ = true;
  }
  return line_requested_ && path_requested_ && drive_requested_;
}

// 형식 (ROS 파라미터 파일과 같아서 launch에서도 그대로 읽을 수 있음)
// /lineDetect_node:
//   ros__parameters:
//     white.h_min: 75
//     drive.require_cmd: true
// 연결된 노드만 저장한다 (예: tb_drive를 안 띄웠으면 line/path만)
void QNode::saveParams(const QString &path)
{
  struct Target
  {
    const char *name;
    rclcpp::AsyncParametersClient::SharedPtr client;
    std::vector<std::string> names;
  };
  std::vector<Target> targets;
  for (const auto &t : {Target{LINE_NODE, line_client_, lineParamNames()},
                        Target{PATH_NODE, path_client_, pathParamNames()},
                        Target{DRIVE_NODE, drive_client_, driveParamNames(false)}})
    if (t.client->service_is_ready())
      targets.push_back(t);

  if (targets.empty())
  {
    Q_EMIT paramFileStatus("저장 실패: 연결된 노드 없음");
    return;
  }

  // 각 노드에서 값을 받아 모은 뒤 한 번에 파일로 씀 (콜백은 ROS 스레드에서 차례로 실행)
  struct Pending
  {
    std::string file;
    std::vector<std::pair<std::string, std::vector<rclcpp::Parameter>>> nodes;
    size_t left = 0;
  };
  auto st = std::make_shared<Pending>();
  st->file = path.toStdString();
  st->left = targets.size();
  for (const auto &t : targets)
    st->nodes.push_back({t.name, {}});

  auto finish = [this, st]()
  {
    if (--st->left > 0)
      return;
    std::ofstream f(st->file);
    if (!f)
    {
      Q_EMIT paramFileStatus("저장 실패: 파일을 열 수 없음\n" + QString::fromStdString(st->file));
      return;
    }
    size_t count = 0;
    QStringList saved;
    for (const auto &[node_name, ps] : st->nodes)
    {
      f << node_name << ":\n  ros__parameters:\n";
      for (const auto &p : ps)
      {
        if (isUnsavedParam(p.get_name()))
          continue;
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
          f << "    " << p.get_name() << ": " << p.as_int() << "\n";
        else if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
          f << "    " << p.get_name() << ": " << (p.as_bool() ? "true" : "false") << "\n";
        else
          continue;
        ++count;
      }
      saved << QString::fromStdString(node_name).mid(1);
    }
    Q_EMIT paramFileStatus(QString("저장됨: %1\n%2개 (%3)")
                               .arg(QString::fromStdString(st->file))
                               .arg(count)
                               .arg(saved.join(", ")));
  };

  // 노드에 실제로 있는 이름만 요청 (없는 이름이 하나라도 섞이면 노드가 빈 응답을 줘서 그 노드 값이 통째로 빠짐)
  for (size_t i = 0; i < targets.size(); ++i)
  {
    auto client = targets[i].client;
    const auto wanted = targets[i].names;
    client->list_parameters({}, 0, [st, finish, i, client, wanted](std::shared_future<rcl_interfaces::msg::ListParametersResult> lf)
                            {
      std::vector<std::string> names;
      try {
        const auto have = lf.get().names;
        for (const auto &n : wanted)
          if (std::find(have.begin(), have.end(), n) != have.end())
            names.push_back(n);
      } catch (...) {
        names = wanted;
      }
      client->get_parameters(names, [st, finish, i](std::shared_future<std::vector<rclcpp::Parameter>> f)
                             {
        try { st->nodes[i].second = f.get(); } catch (...) {}
        finish(); }); });
  }
}

void QNode::loadParams(const QString &path)
{
  std::ifstream f(path.toStdString());
  if (!f)
  {
    Q_EMIT paramFileStatus("불러오기 실패: 파일 없음\n" + path);
    return;
  }

  // "/노드:" 줄로 대상 노드를 정하고, "이름: 값" 줄을 모음 (값은 정수 또는 true/false)
  std::vector<rclcpp::Parameter> line, pth, drv;
  std::vector<rclcpp::Parameter> *cur = nullptr;
  std::string s;
  while (std::getline(f, s))
  {
    const auto b = s.find_first_not_of(" \t");
    if (b == std::string::npos || s[b] == '#')
      continue;
    const std::string t = s.substr(b);
    if (b == 0)
    {
      cur = (t.rfind(LINE_NODE, 0) == 0)    ? &line
            : (t.rfind(PATH_NODE, 0) == 0)  ? &pth
            : (t.rfind(DRIVE_NODE, 0) == 0) ? &drv
                                            : nullptr;
      continue;
    }
    const auto colon = t.find(':');
    if (!cur || colon == std::string::npos || t.rfind("ros__parameters", 0) == 0)
      continue;

    const std::string name = t.substr(0, colon);
    if (isUnsavedParam(name))
      continue;
    std::string val = t.substr(colon + 1);
    val.erase(0, val.find_first_not_of(" \t"));
    val.erase(val.find_last_not_of(" \t\r") + 1);
    if (val == "true" || val == "false")
    {
      cur->emplace_back(name, val == "true");
      continue;
    }
    try
    {
      cur->emplace_back(name, static_cast<int64_t>(std::stoll(val)));
    }
    catch (...)
    {
    }
  }

  int sent = 0;
  auto send = [&sent](rclcpp::AsyncParametersClient::SharedPtr c, const std::vector<rclcpp::Parameter> &ps)
  {
    if (!ps.empty() && c->service_is_ready())
    {
      c->set_parameters(ps);
      sent += static_cast<int>(ps.size());
    }
  };
  send(line_client_, line);
  send(path_client_, pth);
  send(drive_client_, drv);
  Q_EMIT paramFileStatus(QString("불러옴: %1 (%2개 적용)").arg(path).arg(sent));
}

// ───────── 미션 ─────────

void QNode::publishTurn(int turn)
{
  std_msgs::msg::UInt8 m;
  m.data = static_cast<uint8_t>(turn);
  turn_pub_->publish(m);
}

void QNode::publishConstruction(bool on)
{
  std_msgs::msg::Bool m;
  m.data = on;
  construction_pub_->publish(m);
}

void QNode::publishConstructionExit()
{
  std_msgs::msg::Bool m;
  m.data = true;
  construction_exit_pub_->publish(m);
}

void QNode::publishDriveStop()
{
  tb_interfaces::msg::DriveCmd m;
  m.header.stamp = node->now();
  m.mode = tb_interfaces::msg::DriveCmd::STOP;
  drive_cmd_pub_->publish(m);
}

void QNode::publishParking(bool on)
{
  std_msgs::msg::Bool m;
  m.data = on;
  parking_pub_->publish(m);
}

// ───────── UART ─────────

void QNode::sendUart(const QString &cmd)
{
  std_msgs::msg::String m;
  m.data = cmd.toStdString();
  uart_pub_->publish(m);
}

void QNode::publishVelocity(int l, int r)
{
  sendUart(QString("velocity %1 %2").arg(l).arg(r));
}

// ------------------ 객체감지--------------------

void QNode::onDetections(const tb_interfaces::msg::DetectionArray &msg)
{
  QString text;
  std::lock_guard<std::mutex> lk(seen_mtx_);
  for (const auto &d : msg.detections)
  {
    text += QString("%1  %2  h=%3\n").arg(QString::fromStdString(d.label)).arg(d.score, 0, 'f', 2).arg(d.h); // h: 박스 높이 (가까울수록 큼)
    last_seen_[QString::fromStdString(d.label)] = node->now();
  }
  Q_EMIT detectionsReceived(text.isEmpty() ? QString("(없음)") : text.trimmed());
}

void QNode::detectionsReset()
{
  std::lock_guard<std::mutex> lk(seen_mtx_);
  last_seen_.clear();
  sign_state_.clear(); // 다음 판정 때 전부 X로 다시 전송됨
  sign_time_.clear();
}

void QNode::judgeSigns()
{
  static const QStringList labels = {"parking", "left", "right", "construction", "barrier"};
  constexpr double HOLD_SEC = 2.0;

  const rclcpp::Time now = node->now();
  std::lock_guard<std::mutex> lk(seen_mtx_);
  for (const auto &l : labels)
  {
    int st = 0;
    QString t = "-";
    auto it = last_seen_.find(l);
    if (it != last_seen_.end())
    {
      const double ago = (now - it.value()).seconds();
      st = (ago < HOLD_SEC) ? 1 : 2;
      t = QString("%1, %2초 전")
              .arg(QDateTime::fromMSecsSinceEpoch(it.value().nanoseconds() / 1'000'000)
                       .toString("hh:mm:ss"))
              .arg(ago, 0, 'f', 1);
    }

    if (sign_state_.value(l, -1) != st || sign_time_.value(l) != t)
    {
      sign_state_[l] = st;
      sign_time_[l] = t;
      Q_EMIT signStateChanged(l, st, t);
    }
  }
}