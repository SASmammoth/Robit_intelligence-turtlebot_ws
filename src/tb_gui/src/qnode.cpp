/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 **/
#include "../include/tb_gui/qnode.hpp"

#include <chrono>
#include <fstream>
#include <sstream>
#include <QDateTime>

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

  // ====== UART (tb_uart_node)
  uart_pub_ = node->create_publisher<std_msgs::msg::String>("TB_Uart_RX", 10);
  uart_sub_ = node->create_subscription<std_msgs::msg::String>(
      "TB_Uart_TX", 10, [this](const std_msgs::msg::String::ConstSharedPtr m)
      { Q_EMIT uartReceived(QString::fromStdString(m->data)); });

  // ====== 파라미터 클라이언트 (노드별)
  line_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, LINE_NODE);
  path_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, PATH_NODE);

  // 두 노드의 파라미터 변경을 실시간으로 받음
  param_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(node);
  param_event_cb_handle_ = param_event_handler_->add_parameter_event_callback(
      [this](const rcl_interfaces::msg::ParameterEvent &event)
      {
        if (event.node != LINE_NODE && event.node != PATH_NODE)
          return;
        for (const auto *list : {&event.new_parameters, &event.changed_parameters})
          for (const auto &p : *list)
            if (p.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
              Q_EMIT paramLoaded(QString::fromStdString(p.name), static_cast<int>(p.value.integer_value));
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

std::vector<std::string> QNode::lineParamNames()
{
  std::vector<std::string> names;
  for (const char *t : {"white", "yellow"})
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
  return {"skel.seal_len", "skel.prune_len", "skel.dash_len",
          "path.brick_depth_mm", "path.brick_margin_mm", "path.det_max_age_ms", "path.mem_travel_mm"};
}

void QNode::setParams(const QMap<QString, int> &params)
{
  std::vector<rclcpp::Parameter> line, path;
  for (auto it = params.begin(); it != params.end(); ++it)
    (isPathParam(it.key()) ? path : line).emplace_back(it.key().toStdString(), it.value());

  if (!line.empty() && line_client_->service_is_ready())
    line_client_->set_parameters(line);
  if (!path.empty() && path_client_->service_is_ready())
    path_client_->set_parameters(path);
}

bool QNode::requestParams()
{
  auto request = [this](rclcpp::AsyncParametersClient::SharedPtr client, const std::vector<std::string> &names)
  {
    client->get_parameters(names, [this](std::shared_future<std::vector<rclcpp::Parameter>> f)
                           {
      try {
        for (const auto &p : f.get())
          if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            Q_EMIT paramLoaded(QString::fromStdString(p.get_name()), static_cast<int>(p.as_int()));
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
  return line_requested_ && path_requested_;
}

// 형식 (ROS 파라미터 파일과 같아서 launch에서도 그대로 읽을 수 있음)
// /lineDetect_node:
//   ros__parameters:
//     white.h_min: 75
void QNode::saveParams(const QString &path)
{
  if (!line_client_->service_is_ready() || !path_client_->service_is_ready())
  {
    Q_EMIT paramFileStatus("저장 실패: 노드 연결 안 됨");
    return;
  }

  // 두 노드에서 값을 받아 모은 뒤 한 번에 파일로 씀
  struct Pending
  {
    std::string file;
    std::vector<rclcpp::Parameter> line, path;
    int done = 0;
  };
  auto st = std::make_shared<Pending>();
  st->file = path.toStdString();

  auto finish = [this, st]()
  {
    if (++st->done < 2)
      return;
    std::ofstream f(st->file);
    if (!f)
    {
      Q_EMIT paramFileStatus("저장 실패: 파일을 열 수 없음");
      return;
    }
    auto write = [&f](const char *node_name, const std::vector<rclcpp::Parameter> &ps)
    {
      f << node_name << ":\n  ros__parameters:\n";
      for (const auto &p : ps)
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
          f << "    " << p.get_name() << ": " << p.as_int() << "\n";
    };
    write(LINE_NODE, st->line);
    write(PATH_NODE, st->path);
    Q_EMIT paramFileStatus(QString("저장됨: %1 (%2개)")
                               .arg(QString::fromStdString(st->file))
                               .arg(st->line.size() + st->path.size()));
  };

  line_client_->get_parameters(lineParamNames(),
                               [st, finish](std::shared_future<std::vector<rclcpp::Parameter>> f)
                               {
                                 try
                                 {
                                   st->line = f.get();
                                 }
                                 catch (...)
                                 {
                                 }
                                 finish();
                               });
  path_client_->get_parameters(pathParamNames(),
                               [st, finish](std::shared_future<std::vector<rclcpp::Parameter>> f)
                               {
                                 try
                                 {
                                   st->path = f.get();
                                 }
                                 catch (...)
                                 {
                                 }
                                 finish();
                               });
}

void QNode::loadParams(const QString &path)
{
  std::ifstream f(path.toStdString());
  if (!f)
  {
    Q_EMIT paramFileStatus("불러오기 실패: 파일 없음");
    return;
  }

  // "/노드:" 줄로 대상 노드를 정하고, "이름: 정수" 줄을 모음
  std::vector<rclcpp::Parameter> line, pth;
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
      cur = (t.rfind(LINE_NODE, 0) == 0) ? &line : (t.rfind(PATH_NODE, 0) == 0) ? &pth
                                                                                : nullptr;
      continue;
    }
    const auto colon = t.find(':');
    if (!cur || colon == std::string::npos || t.rfind("ros__parameters", 0) == 0)
      continue;
    try
    {
      cur->emplace_back(t.substr(0, colon), static_cast<int64_t>(std::stoll(t.substr(colon + 1))));
    }
    catch (...)
    {
    }
  }

  int sent = 0;
  if (!line.empty() && line_client_->service_is_ready())
  {
    line_client_->set_parameters(line);
    sent += static_cast<int>(line.size());
  }
  if (!pth.empty() && path_client_->service_is_ready())
  {
    path_client_->set_parameters(pth);
    sent += static_cast<int>(pth.size());
  }
  Q_EMIT paramFileStatus(QString("불러옴: %1 (%2개 적용)").arg(path).arg(sent));
}

// ───────── 미션 ─────────

void QNode::publishTurn(int turn)
{
  std_msgs::msg::UInt8 m;
  m.data = static_cast<uint8_t>(turn);
  turn_pub_->publish(m);
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
    text += QString("%1  %2\n").arg(QString::fromStdString(d.label)).arg(d.score, 0, 'f', 2);
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