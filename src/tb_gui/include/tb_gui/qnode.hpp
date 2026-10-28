/**
 * @file /include/tb_gui/qnode.hpp
 *
 * @brief Communications central!
 **/
#ifndef tb_gui_QNODE_HPP_
#define tb_gui_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <opencv2/opencv.hpp>
#include "tb_interfaces/msg/detection_array.hpp"
#include "tb_interfaces/msg/drive_cmd.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rcl_interfaces/msg/log.hpp>
#endif
#include <QThread>
#include <QImage>
#include <QPolygonF>
#include <QPointF>
#include <QMap>
#include <QString>
#include <QStringList>
#include <atomic>
#include <mutex>
#include <map>
#include <chrono>
#include <string>

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();

  // ====== 파라미터 (이름으로 /lineDetect_node, /path_node, /drive_node 중 보낼 곳을 고름)
  void setParams(const QMap<QString, int> &params);
  bool setBoolParam(const QString &name, bool on); // drive.enable 등 (/drive_node). 연결 안 됐으면 false
  bool requestParams();                 // 세 노드 모두 요청했으면 true
  void refreshParams();                 // 연결된 노드의 현재 값을 다시 읽음 (주기적으로 호출 → 화면이 항상 노드 값)
  void saveParams(const QString &path); // 연결된 노드의 현재 값 → YAML (drive.enable 제외)
  void loadParams(const QString &path); // YAML → 각 노드에 설정 (drive.enable 제외)
  QStringList paramNodesReady();        // 파라미터 서비스가 연결된 노드 이름 (saveParams 대상과 같은 기준)

  // ====== 미션
  void publishTurn(int turn); // 0 직진, 1 좌, 2 우
  void publishParking(bool on);
  void publishWhiteDash(bool on); // /mission/white_dash (path_node 흰 점선 잇기)
  void publishConstruction(bool on); // /mission/construction (공사 구간 모드 수동 켜기/끄기)
  void publishConstructionExit();    // /mission/construction_exit (지금 공사 모드 끝내기: 표지판 자동 진입도)
  void publishDriveStop();           // /drive/cmd STOP 한 번 (차단바 흉내: 10Hz 이상으로 계속 불러야 유지됨)
  void publishBrainRun(bool start);  // /brain/run "start" / "stop" (tb_brain 켜기/끄기)
  void publishReset();               // /mission/reset (path·drive·brain 상태 초기화)
  void publishPause(bool on);        // /mission/pause (tb_brain이 거는 일시정지. brain 끌 때 풀어 줌)
  void publishBrainInject(const QString &label); // /brain/inject: 가짜 표지판 (brain이 바로 그 표지판 동작)

  // ====== UART (tb_uart_node: TB_Uart_RX로 명령 문자열 전송)
  void sendUart(const QString &cmd);  // "start", "quit" 등
  void publishVelocity(int l, int r); // "velocity <L> <R>"

  // ====== 객체 감지 리셋버튼 콜백
  void detectionsReset();

  // ====== 수신 통계 (디버그 시간 표시): 토픽마다 마지막 수신 시각, 헤더 stamp 지연, 초당 개수
  struct RxInfo
  {
    double age_s = -1.0;  // 마지막 수신 후 지난 시간 (-1: 한 번도 안 옴)
    double lat_ms = 0.0;  // 수신 시각(이 PC 벽시계) - 헤더 stamp (로봇·PC 시계 차이 포함, stamp 없으면 0)
    bool has_stamp = false;
    double hz = 0.0;
  };
  QMap<QString, RxInfo> rxSnapshot();   // GUI 스레드에서 주기적으로 호출
  double rosNowSec();                   // 이 노드의 ROS 시간 [s]

protected:
  void run();

private:
  static bool isPathParam(const QString &name);
  static bool isDriveParam(const QString &name);
  static bool isBrainParam(const QString &name); // sign.*, mission.* → /brain_node (표지판 필터, 미션 동작)
  static std::vector<std::string> brainParamNames();
  static std::vector<std::string> lineParamNames();
  static std::vector<std::string> pathParamNames();
  static std::vector<std::string> driveParamNames(bool with_enable); // 정수 + bool
  static std::vector<std::string> driveDoubleNames();                // 실수, 읽기만 (GUI에서 안 바꿈, 파일 저장 안 함)
  void requestFrom(rclcpp::AsyncParametersClient::SharedPtr client, const std::vector<std::string> &names);
  void sendParams(rclcpp::AsyncParametersClient::SharedPtr client, const char *node_name,
                  const std::vector<rclcpp::Parameter> &ps);

  std::shared_ptr<rclcpp::Node> node;

  // 영상
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr usb_sub_, sign_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr mask_sub_, bev_sub_, roi_sub_, skel_sub_;
  std::atomic<int64_t> last_sign_ns_{0};

  // 상태
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_src_sub_;
  rclcpp::Subscription<tb_interfaces::msg::DetectionArray>::SharedPtr det_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr turn_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr parking_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr white_dash_sub_;

  // 미션
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr turn_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parking_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr white_dash_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr construction_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr construction_state_sub_; // path_node가 실제로 공사 구간 모드인지
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr construction_exit_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr construction_info_sub_; // /lane/construction_info
  rclcpp::Publisher<tb_interfaces::msg::DriveCmd>::SharedPtr drive_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr brain_run_pub_; // /brain/run
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_pub_;      // /mission/reset
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_pub_;       // /mission/pause
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr inject_pub_;    // /brain/inject

  // UART
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr uart_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr uart_sub_;

  // 주행 상태 (/drive/state)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drive_state_sub_;

  // 경로 보기 (/lane/path, /drive/target)
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr uart_cmd_sub_; // TB_Uart_RX (모터로 실제 보낸 명령)

  // 파라미터
  rclcpp::AsyncParametersClient::SharedPtr line_client_, path_client_, drive_client_, brain_client_;
  // setBoolParam으로 보낸 시각. 그 직후 주기 읽기(refreshParams)가 보내기 전 값을 돌려주면
  // 화면이 되돌아감(자동→정지 표시인데 로봇은 주행) → 잠깐 동안은 읽은 bool 값을 무시
  std::map<std::string, std::chrono::steady_clock::time_point> bool_set_time_;
  std::mutex bool_set_mtx_;
  bool recentlySet(const std::string &name);
  bool line_requested_ = false, path_requested_ = false, drive_requested_ = false;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler_;
  rclcpp::ParameterEventCallbackHandle::SharedPtr param_event_cb_handle_;

  // 수신 통계
  struct Rx
  {
    int64_t last_ns = 0;   // steady clock
    int64_t lat_ns = 0;
    bool has_stamp = false;
    int count = 0;         // 이번 창에서 받은 개수
    int64_t win_ns = 0;    // 창 시작
    double hz = 0.0;
  };
  std::mutex rx_mtx_;
  std::map<std::string, Rx> rx_;
  void markRx(const std::string &key, const builtin_interfaces::msg::Time *stamp = nullptr);
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;

  // signs 감지 콜백
  void onDetections(const tb_interfaces::msg::DetectionArray &msg);
  QMap<QString, rclcpp::Time> last_seen_;
  QMap<QString, int> sign_state_;    // 마지막으로 보낸 상태
  QMap<QString, QString> sign_time_; // 마지막으로 보낸 시각 문자열
  std::mutex seen_mtx_;              // ROS 스레드 ↔ GUI(리셋) 보호
  rclcpp::TimerBase::SharedPtr judge_timer_;
  void judgeSigns();

Q_SIGNALS:
  void rosShutDown();

  // 영상
  void usbImageReceived(const QImage &img);
  void maskImageReceived(const QImage &img);
  void bevImageReceived(const QImage &img);
  void roiImageReceived(const QImage &img);
  void skelImageReceived(const QImage &img);

  // 노드 로그 (/rosout: brain_node, path_node, drive_node). stamp_ns: 로그를 찍은 쪽 시각 (unix ns)
  void nodeLogReceived(const QString &node, int level, qint64 stamp_ns, const QString &text);

  // 상태
  void pathSourceReceived(const QString &src);
  void detectionsReceived(const QString &text);
  void turnReceived(int turn);
  void parkingReceived(bool on);
  void whiteDashReceived(bool on);
  void constructionStateReceived(bool on); // /lane/construction (path_node 상태: 표지판 자동 진입 포함)
  void constructionInfoReceived(const QString &text); // "ON 표지판 brick=1 dist=0.42 ..." / "off ..."

  // 파라미터
  void paramLoaded(const QString &name, int value);
  void boolParamLoaded(const QString &name, bool value);
  void doubleParamLoaded(const QString &name, double value); // 실수 파라미터 (표시만: drive.uart.* 등)
  void paramFileStatus(const QString &text);
  void paramMessage(const QString &text); // 설정 실패 등 (상태 줄에 잠깐 표시)

  // 주행 (tb_drive)
  void driveStateReceived(const QString &text);
  void pathReceived(const QPolygonF &pts);   // /lane/path (base_link, m)
  void targetReceived(const QPointF &p);     // /drive/target (base_link, m)
  void uartCommandSeen(const QString &text); // TB_Uart_RX (drive, WASD 등 누가 보냈든)

  // UART (TB_Uart_TX 수신 문자열)
  void uartReceived(const QString &text);

  // 객체탐지
  void signStateChanged(const QString &label, int state, const QString &lastTime);
};

#endif /* tb_gui_QNODE_HPP_ */