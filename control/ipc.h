/**
 * @file ipc.h
 * @brief Utilities fir IPC
 * @author Linas Perkauskas
 * @date 2023-02-08
 */

#pragma once

#include <stdlib.h>
#include <json-c/json.h>

#define IPC_BUF_LEN 2048

enum conn_addr {
	CONN_ADDR_SIM,
	CONN_ADDR_VIEW,
	CONN_ADDR_AUX,
	MAX_ADDR_TP,
};

/**
 * @brief Function sends request to client and waits for response.
 * @param[in] 		addr 	Connection addressee that identifies which connection will be used.
 * @param[in, out] 	*j 		`json_object` to send and retreive data.
 * @param[in] 		n 		Size of `buf`.
 * @return 0 on sucess, 1 - otherwise.
 */
int ipc_conn_request(enum conn_addr addr, struct json_object **j, size_t n);

	/**
 * @brief Funcion creates request template
 * @param[in] 	*act 	Request `action` name.
 * @param[out] 	**root 	This is json root node. Must be freed using `json_object_put`.
 * @param[out] 	**data 	This is pointer to `data` json node.
 * @return Nothing.
 */
	void get_request_template(const char *act, struct json_object **root,
							  struct json_object **data);

/**
 * @brief Function setups IPC comunication.
 * @return 0 on sucess, 1 - otherwise.
 */
int ipc_setup();

/**
 * @brief Function destroys IPC comunication.
 * @return Nothing.
 */
void ipc_release();
