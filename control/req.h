/**
 * @file req.h
 * @brief Utilities for reqest handling (on tom of IPC)
 * @author Linas Perkauskas
 * @date 2023-02-13
 */

#pragma once

#include <json-c/json.h>

#include "ipc.h"

enum req_status {
	RQS_OK,
	RQS_ERR,
	RQS_NOT_INIT,
	RQS_NO_DATA,
	RQS_PARSE,
	RQS_SUPPORT,
	RQS_MAX,
};

int req_model(struct json_object **j);
int req_sens(struct json_object **j);
int req_setup();
int req_step(struct json_object **j);

/**
 * @brief Funcion creates request template
 * @param[in] 	*act 	Request `action` name.
 * @param[out] 	**root 	This is json root node. Must be freed using `json_object_put`.
 * @param[out] 	**data 	This is pointer to `data` json node.
 * @return Nothing.
 */
void get_request_template(const char *act, struct json_object **root,
						  struct json_object **data);

int exec_request(ipc_conn_t *conn, struct json_object *jreq,
				 struct json_object **jrsp);

