/*! \file   janus_mqtt.c
 * \author Morgan Quigley <morgan@openrobotics.org>
 * \copyright GNU General Public License v3
 * \brief  Janus ROS2 transport plugin
 * \details  This is an implementation of a ROS2 transport for the Janus API.
 *           The code skeleton follows the overall structure of the MQTT transport
 * \ingroup transports
 * \ref transports
 */

extern "C" {
#include "transports/transport.h"
#include "debug.h"
#include "config.h"
#include "utils.h"
}

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/* Transport plugin information */
#define JANUS_ROS2_VERSION        1
#define JANUS_ROS2_VERSION_STRING "0.0.1"
#define JANUS_ROS2_DESCRIPTION    "This transport plugin adds ROS2 support to the Janus API"
#define JANUS_ROS2_NAME           "JANUS ROS2 transport plugin"
#define JANUS_ROS2_AUTHOR         "Morgan Quigley <morgan@openrobotics.org>"
#define JANUS_ROS2_PACKAGE        "janus.transport.ros2"

/* Transport methods */
extern "C" {
janus_transport *create(void);
int janus_ros2_init(janus_transport_callbacks *callback, const char *config_path);
void janus_ros2_destroy(void);
int janus_ros2_get_api_compatibility(void);
int janus_ros2_get_version(void);
const char *janus_ros2_get_version_string(void);
const char *janus_ros2_get_description(void);
const char *janus_ros2_get_name(void);
const char *janus_ros2_get_author(void);
const char *janus_ros2_get_package(void);
gboolean janus_ros2_is_janus_api_enabled(void);
gboolean janus_ros2_is_admin_api_enabled(void);
int janus_ros2_send_message(janus_transport_session *transport, void *request_id, gboolean admin, json_t *message);
void janus_ros2_session_created(janus_transport_session *transport, guint64 session_id);
void janus_ros2_session_over(janus_transport_session *transport, guint64 session_id, gboolean timeout, gboolean claimed);
void janus_ros2_session_claimed(janus_transport_session *transport, guint64 session_id);
}

// Transport setup
// have to populate the transport object by place order, since named initializer lists
// are not allowed in C++11  :(
static janus_transport janus_ros2_transport_ = {
		janus_ros2_init,
    janus_ros2_destroy,

		janus_ros2_get_api_compatibility,
    janus_ros2_get_version,
		janus_ros2_get_version_string,
		janus_ros2_get_description,
		janus_ros2_get_name,
		janus_ros2_get_author,
		janus_ros2_get_package,

		janus_ros2_is_janus_api_enabled,
		janus_ros2_is_admin_api_enabled,

		janus_ros2_send_message,
		janus_ros2_session_created,
		janus_ros2_session_over,
		janus_ros2_session_claimed,
};

// Transport creator
janus_transport *create(void) {
	JANUS_LOG(LOG_VERB, "%s created!\n", JANUS_ROS2_NAME);
  JANUS_LOG(LOG_INFO, "ros2 transport created\n");
	return &janus_ros2_transport_;
}

static janus_transport_session *g_ros2_session = NULL;

class JanusNode : public rclcpp::Node
{
public:
  JanusNode(janus_transport_callbacks *transport_callbacks)
  : Node("janus")
  {
    JANUS_LOG(LOG_INFO, "JanusNode::JanusNode()\n");
    pub_ = this->create_publisher<std_msgs::msg::String>("janus_pub");
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "janus_sub",
        std::bind(&JanusNode::sub_cb,
        this,
        std::placeholders::_1));
    transport_callbacks_ = transport_callbacks;
  }
  ~JanusNode()
  {
    JANUS_LOG(LOG_INFO, "JanusNode::~JanusNode()\n");
  }
  void send(const char * const msg)
  {
    // send() is called from the Janus thread.
    // todo: safely cross thread boundary, if needed (?)
    JANUS_LOG(LOG_INFO, "JanusNode::send called: %s", msg);
    auto ros2_msg = std_msgs::msg::String();
    ros2_msg.data = std::string(msg);
    pub_->publish(ros2_msg);
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  janus_transport_callbacks *transport_callbacks_;
  void sub_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    // called from the ROS 2 thread. Janus implements thread-safe queue
    JANUS_LOG(LOG_INFO, "janus msg rx: %s\n", msg->data.c_str());
    json_error_t json_error;
    json_t *json_root = json_loadb(
        msg->data.c_str(), msg->data.length(), 0, &json_error);
    transport_callbacks_->incoming_request(&janus_ros2_transport_,
        g_ros2_session, NULL, FALSE, json_root, &json_error);
  }
};
static std::shared_ptr<JanusNode> janus_ros2_node_ptr;
static std::thread janus_ros2_thread;

void janus_ros2_thread_fn()
{
  JANUS_LOG(LOG_INFO, "entering JanusNode spin\n");
  rclcpp::spin(janus_ros2_node_ptr);
  JANUS_LOG(LOG_INFO, "exited JanusNode spin\n");
}

int janus_ros2_init(
    janus_transport_callbacks *transport_callbacks,
    const char *config_path)
{
	if (transport_callbacks == NULL || config_path == NULL) {
    return -1;
  }

  rclcpp::init(0, nullptr);
  auto node = std::make_shared<JanusNode>(transport_callbacks);
  janus_ros2_node_ptr = node;  // AHHH global... raptors start attacking
  //g_ros2_session = janus_transport_session_create(NULL, janus_ros2_destroy_node);
  g_ros2_session = janus_transport_session_create(NULL, NULL);
  janus_ros2_thread = std::thread(janus_ros2_thread_fn);
	return 0;
}

void janus_ros2_destroy(void)
{
  JANUS_LOG(LOG_INFO, "Disconnecting ROS2 client...\n");
  janus_transport_session_destroy(g_ros2_session);
  JANUS_LOG(LOG_INFO, "finished destroying ros2_session\n");
  janus_ros2_thread.join();
  JANUS_LOG(LOG_INFO, "joined ros2 thread\n");
  rclcpp::shutdown();
  JANUS_LOG(LOG_INFO, "called rclcpp::shutdown()\n");
  janus_ros2_node_ptr = nullptr;
  JANUS_LOG(LOG_INFO, "deleted JanusNode shared_ptr reference\n");
}

/*
void janus_ros2_destroy_node(void *)
{
}
*/

int janus_ros2_send_message(
    janus_transport_session *transport,
    void *request_id,
    gboolean admin,
    json_t *message)
{
  (void)request_id;  // unused for now...
  (void)admin;  // unused for now...
  if(message == NULL) {
    return -1;
  }

  const size_t json_format_ = JSON_INDENT(3) | JSON_PRESERVE_ORDER;
  char *payload = json_dumps(message, json_format_);
  if (!admin) {
	  JANUS_LOG(LOG_INFO, "janus_ros2_send_message: %s\n", payload);
    janus_ros2_node_ptr->send(payload);
  }
  else {
	  JANUS_LOG(LOG_INFO,
        "janus_ros2_send_message ignoring admin message: %s\n", payload);
  }
  json_decref(message);
  free(payload);
	return 0;
}


int janus_ros2_get_api_compatibility(void)
{
  return JANUS_TRANSPORT_API_VERSION;
}

int janus_ros2_get_version(void)
{
	return JANUS_ROS2_VERSION;
}

const char *janus_ros2_get_version_string(void)
{
  return JANUS_ROS2_VERSION_STRING;
}

const char *janus_ros2_get_description(void)
{
  return JANUS_ROS2_DESCRIPTION;
}

const char *janus_ros2_get_name(void)
{
  return JANUS_ROS2_NAME;
}

const char *janus_ros2_get_author(void)
{
  return JANUS_ROS2_AUTHOR;
}

const char *janus_ros2_get_package(void)
{
  return JANUS_ROS2_PACKAGE;
}

gboolean janus_ros2_is_janus_api_enabled(void)
{
  return TRUE;
}

gboolean janus_ros2_is_admin_api_enabled(void)
{
  return FALSE;
}

void janus_ros2_session_created(
    janus_transport_session *transport,
    guint64 session_id)
{
  // not currently needed
  (void)transport;
  (void)session_id;
}

void janus_ros2_session_over(
    janus_transport_session *transport,
    guint64 session_id,
    gboolean timeout,
    gboolean claimed)
{
  // not currently needed
  (void)transport;
  (void)session_id;
  (void)timeout;
  (void)claimed;
}

void janus_ros2_session_claimed(janus_transport_session *transport, guint64 session_id)
{
  // not currently needed
  (void)transport;
  (void)session_id;
}
