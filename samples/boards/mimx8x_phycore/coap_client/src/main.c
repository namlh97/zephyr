/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_coap_client_sample, LOG_LEVEL_DBG);

#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/net/socket.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/udp.h>
#include <zephyr/net/coap.h>
#include <zephyr/net/coap_client.h>

#include <zephyr/shell/shell.h>

#include "net_private.h"

#ifndef CONFIG_NET_CONFIG_MY_IPV6_ADDR
#define CONFIG_NET_CONFIG_MY_IPV6_ADDR NULL
#endif 

#ifndef CONFIG_NET_CONFIG_MY_IPV4_ADDR
#define CONFIG_NET_CONFIG_MY_IPV4_ADDR NULL
#endif 

#ifndef CONFIG_NET_CONFIG_MY_IPV4_NETMASK
#define CONFIG_NET_CONFIG_MY_IPV4_NETMASK NULL
#endif

#define COAP_DEFAULT_PORT  5683  /* CoAP default UDP/TCP port */
#define COAP_CLIENT_PORT   1307

/* CoAP socket fd */
static int sock;
static struct coap_client client;

struct coap_uri_info {
  	const char *scheme_name;     /**< scheme name */
	union {
		struct sockaddr sa;		 /** Address */
		struct sockaddr_in sin;
		struct sockaddr_in6 sin6;
	};
  	const uint8_t *path;		 /** Uri-Path */
  	uint8_t        scheme;       /**< scheme */
};

static struct coap_uri_info coap_uri_scheme[1] = {
  { "coap", .sa = {.sa_family = AF_INET, .data = {0},}, NULL, COAP_OPTION_PROXY_SCHEME },
};

static uint8_t method = 0;
static struct coap_uri_info uri_info = {0};
static struct coap_client_request client_request;

/* CoAP Options */
#define BLOCK_WISE_TRANSFER_SIZE_GET 2048
#define ENET_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(nxp_enet_mac)

static K_SEM_DEFINE(iface_up, 0, 1);
static K_MUTEX_DEFINE(wait_mutex);
static struct k_condvar wait_condvar;

static int setup_iface(struct net_if *iface, const char *ipv6_addr,
		       const char *ipv4_addr, const char *netmask)
{
	struct net_if_addr *ifaddr;
	struct in_addr addr4;
	struct in6_addr addr6;

	LOG_INF("Configuring iface %d (%p)", net_if_get_by_iface(iface), iface);

	if (IS_ENABLED(CONFIG_NET_IPV6) && net_if_flag_is_set(iface, NET_IF_IPV6)) {
		if (net_addr_pton(AF_INET6, ipv6_addr, &addr6)) {
			LOG_ERR("Invalid address: %s", ipv6_addr);
			return -EINVAL;
		}

		ifaddr = net_if_ipv6_addr_add(iface, &addr6, NET_ADDR_MANUAL, 0);
		if (!ifaddr) {
			LOG_ERR("Cannot add %s to interface %p", ipv6_addr, iface);
			return -EINVAL;
		}
		LOG_INF("IPv6 address: %s", ipv6_addr);
	}

	if (IS_ENABLED(CONFIG_NET_IPV4) && net_if_flag_is_set(iface, NET_IF_IPV4)) {
		if (net_addr_pton(AF_INET, ipv4_addr, &addr4)) {
			LOG_ERR("Invalid address: %s", ipv4_addr);
			return -EINVAL;
		}

		ifaddr = net_if_ipv4_addr_add(iface, &addr4, NET_ADDR_MANUAL, 0);
		if (!ifaddr) {
			LOG_ERR("Cannot add %s to interface %p", ipv4_addr, iface);
			return -EINVAL;
		}
		LOG_INF("IPv4 address: %s", ipv4_addr);

		if (netmask && netmask[0]) {
			struct in_addr nm;
			if (net_addr_pton(AF_INET, netmask, &nm)) {
				LOG_ERR("Invalid netmask: %s", netmask);
				return -EINVAL;
			}

			net_if_ipv4_set_netmask_by_addr(iface, &addr4, &nm);
		}
	}

	return 0;
}

static void iface_up_handler(struct net_mgmt_event_callback *cb,
			     uint32_t mgmt_event, struct net_if *iface)
{
	if (mgmt_event == NET_EVENT_IF_UP) {
		k_sem_give(&iface_up);
	}
}

static void wait_for_iface(struct net_if *iface)
{
	struct net_mgmt_event_callback iface_up_cb;

	if (net_if_is_up(iface)) {
		return;
	}

	net_mgmt_init_event_callback(&iface_up_cb, iface_up_handler, NET_EVENT_IF_UP);
	net_mgmt_add_event_callback(&iface_up_cb);

	LOG_INF("Waiting for iface %d to come up", net_if_get_by_iface(iface));
	k_sem_take(&iface_up, K_FOREVER);

	net_mgmt_del_event_callback(&iface_up_cb);
}

static int parse_method(const uint8_t *input_string, uint8_t *method)
{
	int i;
	const char *method_string[] = {
		0, "get", "post", "put", "delete", "fetch", "patch", "ipatch", 0
	};

	for (i = 1; method_string[i]; i++) {
		if (!memcmp(input_string, method_string[i], strlen(method_string[i]))) {
			*method = i;
			return 0;
		}
	}

	return -EINVAL;
}

int parse_uri(const uint8_t *input_string, struct coap_uri_info *uri)
{
	const uint8_t *p, *q;
	uint8_t i;
	size_t len = strlen(input_string);

	if (len == 0) {
		return -EINVAL;
	}

	p = input_string;
	/* Find scheme terminating :// */
	while (len >= 3 && !(p[0] == ':' && p[1] == '/' && p[2] == '/')) {
		p++;
		len--;
 	}

	for (i = 0; i < (sizeof(coap_uri_scheme) / sizeof(coap_uri_scheme[0])); i++) {
		if ((p - input_string) == (int)strlen(coap_uri_scheme[i].scheme_name) && \
			!memcmp(input_string, coap_uri_scheme[i].scheme_name, p - input_string)) {
			uri->scheme = coap_uri_scheme[i].scheme;
			uri->sin.sin_port = htons(COAP_DEFAULT_PORT);
			uri->sin.sin_family = coap_uri_scheme[i].sin.sin_family;
			break;
		}
	}

	p += 3;
	len -= 3;

	/* Find Uri-Host */
	/* q points to beginning of Uri-Host */
	q = p;

	/* IPv4 */
	while (len && *q != ':' && *q != '/' && *q != '?') {
		q++;
		len--;
	}

	if (p == q) {
		return -EINVAL;
	}
	uint8_t ipv4_addr[15] = {0};
	/* Get host */
	memcpy(ipv4_addr, p, (size_t)(q - p));
	/* Convert string to ip address */
	inet_pton(AF_INET, ipv4_addr, &uri->sin.sin_addr);

	/* Find Uri-Host */
	if (len && *q == ':') {
		p = ++q;
		len--;

		while (len && (*q >= '0' && *q <= '9')) {
			q++;
			len--;
		}

		/* explicit port number given */
		if (p < q) {  
			uint16_t uri_port = 0;
			while ((p < q) && (uri_port <= UINT16_MAX)) {
				uri_port = uri_port * 10 + (*p++ - '0');
			}

			/* check if port number is in allowed range */
			if (uri_port > UINT16_MAX) {
				return -EINVAL;
			}
			uri->sin.sin_port = htons(uri_port);
		}
	}

	/* Find Uri-Path */
	if (*q == '/') {
		p = ++q;
		len--;
	
		while (len && *q != '\0') {
			q++;
			len--;
		}

		if (p < q) {
			uri->path = p;
		}
	}

	return 0;
}

static void wait_task(void) {
	k_condvar_init(&wait_condvar);

	k_mutex_lock(&wait_mutex, K_FOREVER);
	k_condvar_wait(&wait_condvar, &wait_mutex, K_FOREVER);
	k_mutex_unlock(&wait_mutex);
}

void coap_callback(int16_t code, size_t offset, const uint8_t *payload, size_t payload_len, bool last_block,
		   void *user_data)
{
	LOG_INF("CoAP response callback, %d\n", code);

	if (payload_len > 0) {
		LOG_INF("Received data with length: %ld", payload_len);
	}

	if (last_block) {
		k_condvar_signal(&wait_condvar);
	}
}

static int cmd_coap_client(const struct shell *sh, size_t argc, char *argv[])
{
	ARG_UNUSED(sh);
	int ret;

	for (size_t i = 1; i < argc; i++) {

		if (*argv[i] != '-') {
			ret = parse_uri(argv[i], &uri_info);
			if (ret < 0) {
				return ret;
			}
			continue;
		}

		switch (argv[i][1]) {
			case 'm':
			ret = parse_method(&argv[i][2], &method);
			if (ret < 0) {
				return ret;
			}
			break;
			default:
			break;
		}
	}

	client_request.method = method;
	client_request.confirmable = true;
	client_request.path = uri_info.path;
	client_request.fmt = COAP_CONTENT_FORMAT_TEXT_PLAIN;
	client_request.cb = coap_callback;
	client_request.payload = NULL;
	client_request.len = 0;

	if (sock >= 0) {
		ret = coap_client_req(&client, sock, &uri_info.sa, &client_request, NULL);
		if (ret < 0) {
			return ret;
		}

		wait_task();
	}
	else {
		LOG_ERR("Sock is not available !");
	}

	return 0;
}

static int start_coap_client(void)
{
	struct net_if *iface = NULL;
	struct sockaddr_in addr;
	int ret = 0;

	/* Setup iface */
	iface = net_if_lookup_by_dev(DEVICE_DT_GET(ENET_NODE));

	if (!iface) {
		return -EIO;
	}

	wait_for_iface(iface);
	setup_iface(iface,
			CONFIG_NET_CONFIG_MY_IPV6_ADDR,
			CONFIG_NET_CONFIG_MY_IPV4_ADDR,
			CONFIG_NET_CONFIG_MY_IPV4_NETMASK);

	/* Setup sockets */
	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("Failed to create UDP socket %d", errno);
		return -errno;
	}

	addr.sin_family = AF_INET;
	addr.sin_port = htons(COAP_CLIENT_PORT);

	ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	if (ret < 0) {
		LOG_ERR("Cannot bind address : %d", errno);
		return -errno;
	}

	ret = coap_client_init(&client, NULL);
	if (ret < 0) {
		LOG_ERR("Cannot initialize COAP client : %d", errno);
		return -errno;
	}

	LOG_INF("CoAP-client started");

	return 0;
}

int main(void)
{
	int r;

 	LOG_DBG("Start CoAP-client sample");
	r = start_coap_client();
	if (r < 0) {
		goto quit;
	}

	LOG_DBG("Done");
	return 0;

quit:
	(void)close(sock);

	LOG_ERR("quit");
	return 0;
}

SHELL_CMD_REGISTER(coap_client, NULL, "CoAP Client Service commands", cmd_coap_client);