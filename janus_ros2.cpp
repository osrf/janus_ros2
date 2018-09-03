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
#include "janus/transports/transport.h"
#include "janus/debug.h"
#include "janus/config.h"
#include "janus/utils.h"
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

/* Transport setup */
// ahve to populate the transport object by place order, since named initializer lists
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

/* Transport creator */
janus_transport *create(void) {
	JANUS_LOG(LOG_VERB, "%s created!\n", JANUS_ROS2_NAME);
  JANUS_LOG(LOG_INFO, "ros2 transport created\n");
	return &janus_ros2_transport_;
}

#if 0
/* Event handlers */
static gboolean notify_events = TRUE;
#endif

/* JSON serialization options */
static size_t json_format_ = JSON_INDENT(3) | JSON_PRESERVE_ORDER;

#if 0
/* ROS2 client context */
typedef struct janus_ros2_context {
	janus_transport_callbacks *gateway;
	MQTTAsync client;
	struct {
		int keep_alive_interval;
		int cleansession;
		char *username;
		char *password;
	} connect;
	struct {
		int timeout;
	} disconnect;
	struct {
		char *topic;
		int qos;
	} subscribe;
	struct {
		char *topic;
		int qos;
	} publish;
	struct {
		struct {
			char *topic;
			int qos;
		} subscribe;
		struct {
			char *topic;
			int qos;
		} publish;
	} admin;
	/* SSL config, if needed */
	gboolean ssl_enable;
	char *cacert_file;
	char *cert_file;
	char *key_file;
	gboolean verify_peer;
} janus_ros2_context;

/* Transport client methods */
void janus_mqtt_client_connection_lost(void *context, char *cause);
int janus_mqtt_client_message_arrived(void *context, char *topicName, int topicLen, MQTTAsync_message *message);
void janus_mqtt_client_delivery_complete(void *context, MQTTAsync_token token);
int janus_mqtt_client_connect(janus_mqtt_context *ctx);
void janus_mqtt_client_connect_success(void *context, MQTTAsync_successData *response);
void janus_mqtt_client_connect_failure(void *context, MQTTAsync_failureData *response);
int janus_mqtt_client_reconnect(janus_mqtt_context *ctx);
void janus_mqtt_client_reconnect_success(void *context, MQTTAsync_successData *response);
void janus_mqtt_client_reconnect_failure(void *context, MQTTAsync_failureData *response);
int janus_mqtt_client_disconnect(janus_mqtt_context *ctx);
void janus_mqtt_client_disconnect_success(void *context, MQTTAsync_successData *response);
void janus_mqtt_client_disconnect_failure(void *context, MQTTAsync_failureData *response);
int janus_mqtt_client_subscribe(janus_mqtt_context *ctx, gboolean admin);
void janus_mqtt_client_subscribe_success(void *context, MQTTAsync_successData *response);
void janus_mqtt_client_subscribe_failure(void *context, MQTTAsync_failureData *response);
void janus_mqtt_client_admin_subscribe_success(void *context, MQTTAsync_successData *response);
void janus_mqtt_client_admin_subscribe_failure(void *context, MQTTAsync_failureData *response);
int janus_mqtt_client_publish_message(janus_mqtt_context *ctx, char *payload, gboolean admin);
void janus_mqtt_client_publish_janus_success(void *context, MQTTAsync_successData *response);
void janus_mqtt_client_publish_janus_failure(void *context, MQTTAsync_failureData *response);
void janus_mqtt_client_publish_admin_success(void *context, MQTTAsync_successData *response);
void janus_mqtt_client_publish_admin_failure(void *context, MQTTAsync_failureData *response);
void janus_mqtt_client_destroy_context(janus_mqtt_context **ctx);
#endif

/* We only handle a single client */
#if 0
static janus_ros2_context *context_ = NULL;
#endif
static janus_transport_session *ros2_session = NULL;

class JanusNode : public rclcpp::Node
{
public:
  JanusNode()
  : Node("janus")
  {
    printf("hello from JanusNode\n");
    pub_ = this->create_publisher<std_msgs::msg::String>("janus_pub");
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "janus_sub",
        std::bind(&JanusNode::sub_cb,
        this,
        std::placeholders::_1));
  }
  void send(const char * const msg)
  {
    // send() is called from the Janus thread
    // todo: safely cross thread boundary...
    auto ros2_msg = std_msgs::msg::String();
    ros2_msg.data = std::string(msg);
    pub_->publish(ros2_msg);
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  void sub_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    JANUS_LOG(LOG_INFO, "janus msg rx: %s\n", msg->data.c_str());
    // this is called from the ROS 2 thread need to figure
    // out how to cross back to the Janus thread, if needed
  }
};
static std::shared_ptr<JanusNode> janus_ros2_node_ptr;

/////////////////////////////
static std::thread janus_ros2_thread;

void janus_ros2_thread_fn()
{
  JANUS_LOG(LOG_INFO, "entering JanusNode spin\n");
  rclcpp::spin(janus_ros2_node_ptr);
  JANUS_LOG(LOG_INFO, "exited JanusNode spin\n");
}
/////////////////////////////

void janus_ros2_destroy_node(void *)
{
  janus_ros2_thread.join();
  rclcpp::shutdown();
  janus_ros2_node_ptr = nullptr;
}

int janus_ros2_init(janus_transport_callbacks *callback, const char *config_path) {
	if(callback == NULL || config_path == NULL) {
		return -1;
	}

#if 0
	/* Initializing context */
	janus_mqtt_context *ctx = g_malloc0(sizeof(struct janus_mqtt_context));
	ctx->gateway = callback;
	context_ = ctx;
#endif
	/* Prepare the transport session (again, just one) */
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<JanusNode>();
  janus_ros2_node_ptr = node;  // AHHH global... raptors start attacking
	ros2_session = janus_transport_session_create(NULL, janus_ros2_destroy_node);
  janus_ros2_thread = std::thread(janus_ros2_thread_fn);

#if 0
	/* Read configuration */
	char filename[255];
	g_snprintf(filename, 255, "%s/%s.cfg", config_path, JANUS_MQTT_PACKAGE);
	JANUS_LOG(LOG_VERB, "Configuration file: %s\n", filename);
	janus_config *config = janus_config_parse(filename);
	if(config != NULL) {
		janus_config_print(config);
	}

	/* Handle configuration */
	janus_config_item *url_item = janus_config_get_item_drilldown(config, "general", "url");
	const char *url = g_strdup((url_item && url_item->value) ? url_item->value : "tcp://localhost:1883");

	janus_config_item *client_id_item = janus_config_get_item_drilldown(config, "general", "client_id");
	const char *client_id = g_strdup((client_id_item && client_id_item->value) ? client_id_item->value : "guest");

	janus_config_item *username_item = janus_config_get_item_drilldown(config, "general", "username");
	ctx->connect.username = g_strdup((username_item && username_item->value) ? username_item->value : "guest");

	janus_config_item *password_item = janus_config_get_item_drilldown(config, "general", "password");
	ctx->connect.password = g_strdup((password_item && password_item->value) ? password_item->value : "guest");

	janus_config_item *json_item = janus_config_get_item_drilldown(config, "general", "json");
	if(json_item && json_item->value) {
		/* Check how we need to format/serialize the JSON output */
		if(!strcasecmp(json_item->value, "indented")) {
			/* Default: indented, we use three spaces for that */
			json_format_ = JSON_INDENT(3) | JSON_PRESERVE_ORDER;
		} else if(!strcasecmp(json_item->value, "plain")) {
			/* Not indented and no new lines, but still readable */
			json_format_ = JSON_INDENT(0) | JSON_PRESERVE_ORDER;
		} else if(!strcasecmp(json_item->value, "compact")) {
			/* Compact, so no spaces between separators */
			json_format_ = JSON_COMPACT | JSON_PRESERVE_ORDER;
		} else {
			JANUS_LOG(LOG_WARN, "Unsupported JSON format option '%s', using default (indented)\n", json_item->value);
			json_format_ = JSON_INDENT(3) | JSON_PRESERVE_ORDER;
		}
	}

	/* Check if we need to send events to handlers */
	janus_config_item *events = janus_config_get_item_drilldown(config, "general", "events");
	if(events != NULL && events->value != NULL)
		notify_events = janus_is_true(events->value);
	if(!notify_events && callback->events_is_enabled()) {
		JANUS_LOG(LOG_WARN, "Notification of events to handlers disabled for %s\n", JANUS_MQTT_NAME);
	}

	/* Check if we need to enable SSL support */
	janus_config_item *ssl = janus_config_get_item_drilldown(config, "general", "ssl_enable");
	if(ssl && ssl->value && janus_is_true(ssl->value)) {
		if(strstr(url, "ssl://") != url)
			JANUS_LOG(LOG_WARN, "SSL enabled, but MQTT url doesn't start with ssl://...\n");

		ctx->ssl_enable = TRUE;

		janus_config_item *cacertfile = janus_config_get_item_drilldown(config, "general", "cacertfile");
		if(!cacertfile || !cacertfile->value) {
			JANUS_LOG(LOG_FATAL, "Missing CA certificate for MQTT integration...\n");
			goto error;
		}
		ctx->cacert_file = g_strdup(cacertfile->value);

		janus_config_item *certfile = janus_config_get_item_drilldown(config, "general", "certfile");
		ctx->cert_file = (certfile && certfile->value) ? g_strdup(certfile->value) : NULL;

		janus_config_item *keyfile = janus_config_get_item_drilldown(config, "general", "keyfile");
		ctx->key_file = (keyfile && keyfile->value) ? g_strdup(keyfile->value) : NULL;

		if(ctx->cert_file && !ctx->key_file) {
			JANUS_LOG(LOG_FATAL, "Certificate is set but key isn't for MQTT integration...\n");
			goto error;
		}
		if(!ctx->cert_file && ctx->key_file) {
			JANUS_LOG(LOG_FATAL, "Key is set but certificate isn't for MQTT integration...\n");
			goto error;
		}

		janus_config_item *verify = janus_config_get_item_drilldown(config, "general", "verify_peer");
		ctx->verify_peer = (verify && verify->value && janus_is_true(verify->value)) ? TRUE : FALSE;
	} else {
		JANUS_LOG(LOG_INFO, "MQTT SSL support disabled\n");
		if(strstr(url, "ssl://") == url)
			JANUS_LOG(LOG_WARN, "SSL disabled, but MQTT url starts with ssl:// instead of tcp://...\n");
	}

	/* Connect configuration */
	janus_config_item *keep_alive_interval_item = janus_config_get_item_drilldown(config, "general", "keep_alive_interval");
	ctx->connect.keep_alive_interval = (keep_alive_interval_item && keep_alive_interval_item->value) ? atoi(keep_alive_interval_item->value) : 20;

	janus_config_item *cleansession_item = janus_config_get_item_drilldown(config, "general", "cleansession");
	ctx->connect.cleansession = (cleansession_item && cleansession_item->value) ? atoi(cleansession_item->value) : 0;

	/* Disconnect configuration */
	janus_config_item *disconnect_timeout_item = janus_config_get_item_drilldown(config, "general", "disconnect_timeout");
	ctx->disconnect.timeout = (disconnect_timeout_item && disconnect_timeout_item->value) ? atoi(disconnect_timeout_item->value) : 100;

	janus_config_item *enable_item = janus_config_get_item_drilldown(config, "general", "enable");
	if(enable_item && enable_item->value && janus_is_true(enable_item->value)) {
		janus_mqtt_api_enabled_ = TRUE;

		/* Subscribe configuration */
		{
			janus_config_item *topic_item = janus_config_get_item_drilldown(config, "general", "subscribe_topic");
			if(!topic_item || !topic_item->value) {
				JANUS_LOG(LOG_FATAL, "Missing topic for incoming messages for MQTT integration...\n");
				goto error;
			}
			ctx->subscribe.topic = g_strdup(topic_item->value);

			janus_config_item *qos_item = janus_config_get_item_drilldown(config, "general", "subscribe_qos");
			ctx->subscribe.qos = (qos_item && qos_item->value) ? atoi(qos_item->value) : 1;
		}

		/* Publish configuration */
		{
			janus_config_item *topic_item = janus_config_get_item_drilldown(config, "general", "publish_topic");
			if(!topic_item || !topic_item->value) {
				JANUS_LOG(LOG_FATAL, "Missing topic for outgoing messages for MQTT integration...\n");
				goto error;
			}
			ctx->publish.topic = g_strdup(topic_item->value);

			janus_config_item *qos_item = janus_config_get_item_drilldown(config, "general", "publish_qos");
			ctx->publish.qos = (qos_item && qos_item->value) ? atoi(qos_item->value) : 1;
		}
	} else {
		janus_mqtt_api_enabled_ = FALSE;
		ctx->subscribe.topic = NULL;
		ctx->publish.topic = NULL;
	}

	/* Admin configuration */
	janus_config_item *admin_enable_item = janus_config_get_item_drilldown(config, "admin", "admin_enable");
	if(admin_enable_item && admin_enable_item->value && janus_is_true(admin_enable_item->value)) {
		janus_mqtt_admin_api_enabled_ = TRUE;

		/* Admin subscribe configuration */
		{
			janus_config_item *topic_item = janus_config_get_item_drilldown(config, "admin", "subscribe_topic");
			if(!topic_item || !topic_item->value) {
				JANUS_LOG(LOG_FATAL, "Missing topic for incoming admin messages for MQTT integration...\n");
				goto error;
			}
			ctx->admin.subscribe.topic = g_strdup(topic_item->value);

			janus_config_item *qos_item = janus_config_get_item_drilldown(config, "admin", "subscribe_qos");
			ctx->admin.subscribe.qos = (qos_item && qos_item->value) ? atoi(qos_item->value) : 1;
		}

		/* Admin publish configuration */
		{
			janus_config_item *topic_item = janus_config_get_item_drilldown(config, "admin", "publish_topic");
			if(!topic_item || !topic_item->value) {
				JANUS_LOG(LOG_FATAL, "Missing topic for outgoing admin messages for MQTT integration...\n");
				goto error;
			}
			ctx->admin.publish.topic = g_strdup(topic_item->value);

			janus_config_item *qos_item = janus_config_get_item_drilldown(config, "admin", "publish_qos");
			ctx->admin.publish.qos = (qos_item && qos_item->value) ? atoi(qos_item->value) : 1;
		}
	} else {
		janus_mqtt_admin_api_enabled_ = FALSE;
		ctx->admin.subscribe.topic = NULL;
		ctx->admin.publish.topic = NULL;
	}

	if(!janus_mqtt_api_enabled_ && !janus_mqtt_admin_api_enabled_) {
		JANUS_LOG(LOG_WARN, "MQTT support disabled for both Janus and Admin API, giving up\n");
		goto error;
	}

	/* Creating a client */
	if(MQTTAsync_create(
			&ctx->client,
			url,
			client_id,
			MQTTCLIENT_PERSISTENCE_NONE,
			NULL) != MQTTASYNC_SUCCESS) {
		JANUS_LOG(LOG_FATAL, "Can't connect to MQTT broker: error creating client...\n");
		goto error;
	}
	if(MQTTAsync_setCallbacks(
			ctx->client,
			ctx,
			janus_mqtt_client_connection_lost,
			janus_mqtt_client_message_arrived,
			janus_mqtt_client_delivery_complete) != MQTTASYNC_SUCCESS) {
		JANUS_LOG(LOG_FATAL, "Can't connect to MQTT broker: error setting up callbacks...\n");
		goto error;
	}

	/* Connecting to the broker */
	int rc = janus_mqtt_client_connect(ctx);
	if(rc != MQTTASYNC_SUCCESS) {
		JANUS_LOG(LOG_FATAL, "Can't connect to MQTT broker, return code: %d\n", rc);
		goto error;
	}

	g_free((char *)url);
	g_free((char *)client_id);
	janus_config_destroy(config);
#endif
	return 0;

#if 0
error:
	/* If we got here, something went wrong */
	janus_transport_session_destroy(mqtt_session);
	janus_mqtt_client_destroy_context(&ctx);
	g_free((char *)url);
	g_free((char *)client_id);
	janus_config_destroy(config);

	return -1;
#endif
}

void janus_ros2_destroy(void) {
	JANUS_LOG(LOG_INFO, "Disconnecting ROS2 client...\n");

	janus_transport_session_destroy(ros2_session);
  JANUS_LOG(LOG_INFO, "finished destroying ros2_session\n");
#if 0
	janus_mqtt_client_disconnect(context_);
#endif
}

int janus_ros2_get_api_compatibility(void) {
	return JANUS_TRANSPORT_API_VERSION;
}

int janus_ros2_get_version(void) {
	return JANUS_ROS2_VERSION;
}

const char *janus_ros2_get_version_string(void) {
	return JANUS_ROS2_VERSION_STRING;
}

const char *janus_ros2_get_description(void) {
	return JANUS_ROS2_DESCRIPTION;
}

const char *janus_ros2_get_name(void) {
	return JANUS_ROS2_NAME;
}

const char *janus_ros2_get_author(void) {
	return JANUS_ROS2_AUTHOR;
}

const char *janus_ros2_get_package(void) {
	return JANUS_ROS2_PACKAGE;
}

gboolean janus_ros2_is_janus_api_enabled(void)
{
  return FALSE;
}

gboolean janus_ros2_is_admin_api_enabled(void)
{
  return FALSE;
}

int janus_ros2_send_message(
    janus_transport_session *transport,
    void *request_id,
    gboolean admin,
    json_t *message)
{
  (void)transport;  // unused for now...
  (void)request_id;  // unused for now...
  (void)admin;  // unused for now...
  if(message == NULL) {
    return -1;
  }

  char *payload = json_dumps(message, json_format_);
  json_decref(message);
  ((JanusNode *)transport)->send(payload);
  free(payload);


#if 0
	/* Not really needed as we always only have a single context, but that's fine */
	janus_mqtt_context *ctx = (janus_mqtt_context *)transport->transport_p;
	if(ctx == NULL) {
		json_decref(message);
		return -1;
	}

	char *payload = json_dumps(message, json_format_);
	json_decref(message);
	JANUS_LOG(LOG_HUGE, "Sending %s API message via MQTT: %s\n", admin ? "admin" : "Janus", payload);

	int rc = janus_mqtt_client_publish_message(ctx, payload, admin);
	if(rc != MQTTASYNC_SUCCESS) {
		JANUS_LOG(LOG_ERR, "Can't publish to MQTT topic: %s, return code: %d\n", admin ? ctx->admin.publish.topic : ctx->publish.topic, rc);
	}
	free(payload);
#endif
	return 0;
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

#if 0
void janus_mqtt_client_connection_lost(void *context, char *cause) {
	JANUS_LOG(LOG_INFO, "MQTT connection lost cause of %s. Reconnecting...\n", cause);
	/* Automatic reconnect */

	/* Notify handlers about this transport being gone */
	janus_mqtt_context *ctx = (janus_mqtt_context *)context;
	if(notify_events && ctx && ctx->gateway && ctx->gateway->events_is_enabled()) {
		json_t *info = json_object();
		json_object_set_new(info, "event", json_string("reconnecting"));
		ctx->gateway->notify_event(&janus_mqtt_transport_, mqtt_session, info);
	}
}

int janus_mqtt_client_message_arrived(void *context, char *topicName, int topicLen, MQTTAsync_message *message) {
	janus_mqtt_context *ctx = (janus_mqtt_context *)context;
	gchar *topic = g_strndup(topicName, topicLen);
	const gboolean janus = janus_mqtt_api_enabled_ &&  !strcasecmp(topic, ctx->subscribe.topic);
	const gboolean admin = janus_mqtt_admin_api_enabled_ && !strcasecmp(topic, ctx->admin.subscribe.topic);
	g_free(topic);

	if((janus || admin) && message->payloadlen) {
		JANUS_LOG(LOG_HUGE, "Receiving %s API message over MQTT: %s\n", admin ? "admin" : "Janus", (char *)message->payload);

		json_error_t error;
		json_t *root = json_loadb(message->payload, message->payloadlen, 0, &error);
		ctx->gateway->incoming_request(&janus_mqtt_transport_, mqtt_session, NULL, admin, root, &error);
	}

	MQTTAsync_freeMessage(&message);
	MQTTAsync_free(topicName);
	return TRUE;
}

void janus_mqtt_client_delivery_complete(void *context, MQTTAsync_token token) {
}

int janus_mqtt_client_connect(janus_mqtt_context *ctx) {
	MQTTAsync_connectOptions options = MQTTAsync_connectOptions_initializer;
	options.keepAliveInterval = ctx->connect.keep_alive_interval;
	options.cleansession = ctx->connect.cleansession;
	options.username = ctx->connect.username;
	options.password = ctx->connect.password;
	options.automaticReconnect = TRUE;
	options.onSuccess = janus_mqtt_client_connect_success;
	options.onFailure = janus_mqtt_client_connect_failure;
	/* Is SSL enabled? */
	MQTTAsync_SSLOptions ssl_opts = MQTTAsync_SSLOptions_initializer;
	if(ctx->ssl_enable) {
		ssl_opts.trustStore = ctx->cacert_file;
		ssl_opts.keyStore = ctx->cert_file;
		ssl_opts.privateKey = ctx->key_file;
		ssl_opts.enableServerCertAuth = ctx->verify_peer;
		options.ssl = &ssl_opts;
	}
	/* Connect now */
	options.context = ctx;
	return MQTTAsync_connect(ctx->client, &options);
}

void janus_mqtt_client_connect_success(void *context, MQTTAsync_successData *response) {
	JANUS_LOG(LOG_INFO, "MQTT client has been successfully connected to the broker\n");

	/* Subscribe to one (janus or admin) topic at the time */
	janus_mqtt_context *ctx = (janus_mqtt_context *)context;
	if(janus_mqtt_api_enabled_) {
		int rc = janus_mqtt_client_subscribe(context, FALSE);
		if(rc != MQTTASYNC_SUCCESS) {
			JANUS_LOG(LOG_ERR, "Can't subscribe to MQTT topic: %s, return code: %d\n", ctx->subscribe.topic, rc);
		}
	}
	else if(janus_mqtt_admin_api_enabled_) {
		int rc = janus_mqtt_client_subscribe(context, TRUE);
		if(rc != MQTTASYNC_SUCCESS) {
			JANUS_LOG(LOG_ERR, "Can't subscribe to MQTT admin topic: %s, return code: %d\n", ctx->admin.subscribe.topic, rc);
		}
	}

	/* Notify handlers about this new transport */
	if(notify_events && ctx->gateway && ctx->gateway->events_is_enabled()) {
		json_t *info = json_object();
		json_object_set_new(info, "event", json_string("connected"));
		ctx->gateway->notify_event(&janus_mqtt_transport_, mqtt_session, info);
	}
}

void janus_mqtt_client_connect_failure(void *context, MQTTAsync_failureData *response) {
	int rc = response ? response->code : 0;
	JANUS_LOG(LOG_ERR, "MQTT client has failed connecting to the broker, return code: %d. Reconnecting...\n", rc);
	/* Automatic reconnect */

	/* Notify handlers about this transport failure */
	janus_mqtt_context *ctx = (janus_mqtt_context *)context;
	if(notify_events && ctx && ctx->gateway && ctx->gateway->events_is_enabled()) {
		json_t *info = json_object();
		json_object_set_new(info, "event", json_string("failed"));
		json_object_set_new(info, "code", json_integer(rc));
		ctx->gateway->notify_event(&janus_mqtt_transport_, mqtt_session, info);
	}
}

int janus_mqtt_client_reconnect(janus_mqtt_context *ctx) {
	MQTTAsync_disconnectOptions options = MQTTAsync_disconnectOptions_initializer;
	options.onSuccess = janus_mqtt_client_reconnect_success;
	options.onFailure = janus_mqtt_client_reconnect_failure;
	options.context = ctx;
	options.timeout = ctx->disconnect.timeout;
	return MQTTAsync_disconnect(ctx->client, &options);
}

void janus_mqtt_client_reconnect_success(void *context, MQTTAsync_successData *response) {
	JANUS_LOG(LOG_INFO, "MQTT client has been successfully disconnected. Reconnecting...\n");

	int rc = janus_mqtt_client_connect(context);
	if(rc != MQTTASYNC_SUCCESS) {
		JANUS_LOG(LOG_FATAL, "Can't connect to MQTT broker, return code: %d\n", rc);
	}
}

void janus_mqtt_client_reconnect_failure(void *context, MQTTAsync_failureData *response) {
	int rc = response ? response->code : 0;
	JANUS_LOG(LOG_ERR, "MQTT client has failed reconnecting from MQTT broker, return code: %d\n", rc);
}

int janus_mqtt_client_disconnect(janus_mqtt_context *ctx) {
	MQTTAsync_disconnectOptions options = MQTTAsync_disconnectOptions_initializer;
	options.onSuccess = janus_mqtt_client_disconnect_success;
	options.onFailure = janus_mqtt_client_disconnect_failure;
	options.context = ctx;
	options.timeout = ctx->disconnect.timeout;
	return MQTTAsync_disconnect(ctx->client, &options);
}

void janus_mqtt_client_disconnect_success(void *context, MQTTAsync_successData *response) {
	JANUS_LOG(LOG_INFO, "MQTT client has been successfully disconnected. Destroying the client...\n");

	/* Notify handlers about this transport being gone */
	janus_mqtt_context *ctx = (janus_mqtt_context *)context;
	if(notify_events && ctx && ctx->gateway && ctx->gateway->events_is_enabled()) {
		json_t *info = json_object();
		json_object_set_new(info, "event", json_string("disconnected"));
		ctx->gateway->notify_event(&janus_mqtt_transport_, mqtt_session, info);
	}

	janus_mqtt_client_destroy_context(&ctx);
}

void janus_mqtt_client_disconnect_failure(void *context, MQTTAsync_failureData *response) {
	int rc = response ? response->code : 0;
	JANUS_LOG(LOG_ERR, "Can't disconnect from MQTT broker, return code: %d\n", rc);

	janus_mqtt_context *ctx = (janus_mqtt_context *)context;
	janus_mqtt_client_destroy_context(&ctx);
}

int janus_mqtt_client_subscribe(janus_mqtt_context *ctx, gboolean admin) {
	MQTTAsync_responseOptions options = MQTTAsync_responseOptions_initializer;
	options.context = ctx;
	if(admin) {
		options.onSuccess = janus_mqtt_client_admin_subscribe_success;
		options.onFailure = janus_mqtt_client_admin_subscribe_failure;
		return MQTTAsync_subscribe(ctx->client, ctx->admin.subscribe.topic, ctx->admin.subscribe.qos, &options);
	} else {
		options.onSuccess = janus_mqtt_client_subscribe_success;
		options.onFailure = janus_mqtt_client_subscribe_failure;
		return MQTTAsync_subscribe(ctx->client, ctx->subscribe.topic, ctx->subscribe.qos, &options);
	}
}

void janus_mqtt_client_subscribe_success(void *context, MQTTAsync_successData *response) {
	janus_mqtt_context *ctx = (janus_mqtt_context *)context;
	JANUS_LOG(LOG_INFO, "MQTT client has been successfully subscribed to MQTT topic: %s\n", ctx->subscribe.topic);

	/* Subscribe to admin topic if we haven't done it yet */
	if(janus_mqtt_admin_api_enabled_ && (!janus_mqtt_api_enabled_ || strcasecmp(ctx->subscribe.topic, ctx->admin.subscribe.topic))) {
		int rc = janus_mqtt_client_subscribe(context, TRUE);
		if(rc != MQTTASYNC_SUCCESS) {
			JANUS_LOG(LOG_ERR, "Can't subscribe to MQTT topic: %s, return code: %d\n", ctx->subscribe.topic, rc);
		}
	}
}

void janus_mqtt_client_subscribe_failure(void *context, MQTTAsync_failureData *response) {
	janus_mqtt_context *ctx = (janus_mqtt_context *)context;
	int rc = response ? response->code : 0;
	JANUS_LOG(LOG_ERR, "MQTT client has failed subscribing to MQTT topic: %s, return code: %d. Reconnecting...\n", ctx->subscribe.topic, rc);

	/* Reconnect */
	{
		int rc = janus_mqtt_client_reconnect(context);
		if(rc != MQTTASYNC_SUCCESS) {
			JANUS_LOG(LOG_FATAL, "Can't reconnect to MQTT broker, return code: %d\n", rc);
		}
	}
}

void janus_mqtt_client_admin_subscribe_success(void *context, MQTTAsync_successData *response) {
	janus_mqtt_context *ctx = (janus_mqtt_context *)context;
	JANUS_LOG(LOG_INFO, "MQTT client has been successfully subscribed to MQTT topic: %s\n", ctx->admin.subscribe.topic);
}

void janus_mqtt_client_admin_subscribe_failure(void *context, MQTTAsync_failureData *response) {
	janus_mqtt_context *ctx = (janus_mqtt_context *)context;
	int rc = response ? response->code : 0;
	JANUS_LOG(LOG_ERR, "MQTT client has failed subscribing to MQTT topic: %s, return code: %d. Reconnecting...\n", ctx->admin.subscribe.topic, rc);

	/* Reconnect */
	{
		int rc = janus_mqtt_client_reconnect(context);
		if(rc != MQTTASYNC_SUCCESS) {
			JANUS_LOG(LOG_FATAL, "Can't reconnect to MQTT broker, return code: %d\n", rc);
		}
	}
}

int janus_mqtt_client_publish_message(janus_mqtt_context *ctx, char *payload, gboolean admin) {
	MQTTAsync_message msg = MQTTAsync_message_initializer;
	msg.payload = payload;
	msg.payloadlen = strlen(payload);
	msg.qos = ctx->publish.qos;
	msg.retained = 0;

	MQTTAsync_responseOptions options;
	options.context = ctx;
	if(admin) {
		options.onSuccess = janus_mqtt_client_publish_admin_success;
		options.onFailure = janus_mqtt_client_publish_admin_failure;
		return MQTTAsync_sendMessage(ctx->client, ctx->admin.publish.topic, &msg, &options);
	} else {
		options.onSuccess = janus_mqtt_client_publish_janus_success;
		options.onFailure = janus_mqtt_client_publish_janus_failure;
		return MQTTAsync_sendMessage(ctx->client, ctx->publish.topic, &msg, &options);
	}
}

void janus_mqtt_client_publish_janus_success(void *context, MQTTAsync_successData *response) {
	janus_mqtt_context *ctx = (janus_mqtt_context *)context;
	JANUS_LOG(LOG_HUGE, "MQTT client has been successfully published to MQTT topic: %s\n", ctx->publish.topic);
}

void janus_mqtt_client_publish_janus_failure(void *context, MQTTAsync_failureData *response) {
	janus_mqtt_context *ctx = (janus_mqtt_context *)context;
	int rc = response ? response->code : 0;
	JANUS_LOG(LOG_ERR, "MQTT client has failed publishing to MQTT topic: %s, return code: %d\n", ctx->publish.topic, rc);
}

void janus_mqtt_client_publish_admin_success(void *context, MQTTAsync_successData *response) {
	janus_mqtt_context *ctx = (janus_mqtt_context *)context;
	JANUS_LOG(LOG_HUGE, "MQTT client has been successfully published to MQTT topic: %s\n", ctx->admin.publish.topic);
}

void janus_mqtt_client_publish_admin_failure(void *context, MQTTAsync_failureData *response) {
	janus_mqtt_context *ctx = (janus_mqtt_context *)context;
	int rc = response ? response->code : 0;
	JANUS_LOG(LOG_ERR, "MQTT client has failed publishing to MQTT topic: %s, return code: %d\n", ctx->admin.publish.topic, rc);
}

void janus_mqtt_client_destroy_context(janus_mqtt_context **ptr) {
	janus_mqtt_context *ctx = (janus_mqtt_context *)*ptr;
	if(ctx) {
		MQTTAsync_destroy(&ctx->client);
		g_free(ctx->subscribe.topic);
		g_free(ctx->publish.topic);
		g_free(ctx->connect.username);
		g_free(ctx->connect.password);
		g_free(ctx->admin.subscribe.topic);
		g_free(ctx->admin.publish.topic);
		g_free(ctx);
		*ptr = NULL;
	}

	JANUS_LOG(LOG_INFO, "%s destroyed!\n", JANUS_MQTT_NAME);
}
#endif
