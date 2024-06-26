/*
 * Copyright (c) 2018 Intel Corporation
 * Copyright (c) 2023 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(net_coap_service_sample);

#include <zephyr/sys/printk.h>
#include <zephyr/net/coap_service.h>

static char *short_payload = " Ut enim ad minim veniam, quis nostrud exercitation "
				" ullamco laboris nisi ut aliquip ex ea commodo consequat";

static char *long_payload = "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do "
				  "eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut "
				  "enim ad minim veniam, quis nostrud exercitation ullamco laboris "
				  "nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor "
				  "in reprehenderit in voluptate velit esse cillum dolore eu fugiat"
				  " nulla pariatur. Excepteur sint occaecat cupidatat non proident,"
				  " sunt in culpa qui officia deserunt mollit anim id est laborum.";

static int large_get(struct coap_resource *resource,
		     struct coap_packet *request,
		     struct sockaddr *addr, socklen_t addr_len)

{
	uint8_t data[CONFIG_COAP_SERVER_MESSAGE_SIZE];
	static struct coap_block_context ctx;
	struct coap_packet response;
	static uint8_t *payload;
	static uint16_t payload_size;
	struct coap_option query;
	uint8_t token[COAP_TOKEN_MAX_LEN];
	uint16_t size;
	uint16_t id;
	uint8_t code;
	uint8_t type;
	uint8_t tkl;
	int r;

	if (ctx.total_size == 0) {
		r = coap_find_options(request, COAP_OPTION_URI_QUERY, &query, 1);

		if (!memcmp(query.value, "m=long", strlen("m=long")))	{
			payload = long_payload;
			payload_size = strlen(long_payload);
		} else {
			payload = short_payload;
			payload_size = strlen(short_payload);
		}

		coap_block_transfer_init(&ctx, COAP_BLOCK_64, payload_size);
	}

	r = coap_update_from_block(request, &ctx);
	if (r < 0) {
		return -EINVAL;
	}

	code = coap_header_get_code(request);
	type = coap_header_get_type(request);
	id = coap_header_get_id(request);
	tkl = coap_header_get_token(request, token);

	LOG_INF("*******");
	LOG_INF("type: %u code %u id %u", type, code, id);
	LOG_INF("*******");

	r = coap_packet_init(&response, data, sizeof(data),
			     COAP_VERSION_1, COAP_TYPE_ACK, tkl, token,
			     COAP_RESPONSE_CODE_CONTENT, id);
	if (r < 0) {
		return -EINVAL;
	}

	r = coap_append_option_int(&response, COAP_OPTION_CONTENT_FORMAT,
				   COAP_CONTENT_FORMAT_TEXT_PLAIN);
	if (r < 0) {
		return r;
	}

	r = coap_append_block2_option(&response, &ctx);
	if (r < 0) {
		return r;
	}

	r = coap_packet_append_payload_marker(&response);
	if (r < 0) {
		return r;
	}

	size = MIN(coap_block_size_to_bytes(ctx.block_size),
		   ctx.total_size - ctx.current);

	r = coap_packet_append_payload(&response, payload, size);
	if (r < 0) {
		return r;
	}

	payload += size;

	r = coap_next_block(&response, &ctx);
	if (!r) {
		/* Will return 0 when it's the last block. */
		memset(&ctx, 0, sizeof(ctx));
		payload = NULL;
	}

	r = coap_resource_send(resource, &response, addr, addr_len, NULL);

	return r;
}

static const char * const large_path[] = { "example_data", NULL };
COAP_RESOURCE_DEFINE(large, coap_server,
{
	.get = large_get,
	.path = large_path,
});

