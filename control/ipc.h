/**
 * @file ipc.h
 * @brief Utilities for IPC
 * @author Linas Perkauskas
 * @date 2023-02-08
 */

#pragma once

#include <json-c/json_types.h>
#include <stdbool.h>
#include <stdlib.h>
#include <json-c/json.h>

#define IPC_BUF_LEN 2048

enum conn_addr {
	CONN_ADDR_SIM,
	CONN_ADDR_VIEW,
	CONN_ADDR_AUX,
	MAX_ADDR_TP,
};

enum conn_tp {
	CONN_TP_FIFO,
};

enum conn_flags {
	CONN_FL_ASYNC_READ = 1 << 0,
	CONN_FL_LAZY_CONNECT = 1 << 1,
};

typedef struct ipc_conn {
	enum conn_tp tp;
	enum conn_flags flags;
	void *priv;
	void (*free)(struct ipc_conn *);
	int (*req)(struct ipc_conn *, char *buf, bool no_recv);
	void (*async)(struct ipc_conn *);
	void (*complete)(struct ipc_conn *);
	void (*suspend)(struct ipc_conn *, struct json_object **);
} ipc_conn_t;

/**
 * @brief Function sends request to client and waits for response.
 * @param[in] 		addr 		Connection addressee that identifies which connection will be used.
 * @param[in, out] 	*j 			`json_object` to send and retreive data.
 * @param[in] 		no_recv 	Do not wait for rsponse.
 * @return 0 on sucess, 1 - otherwise.
 */
int ipc_conn_request_ex(enum conn_addr addr, struct json_object **j,
						bool no_recv);

inline int ipc_conn_request(enum conn_addr addr, struct json_object **j)
{
	return ipc_conn_request_ex(addr, j, false);
}

/**
 * @brief Function processes asyn chronous messages retreived for all connections
 * @return Nothing.
 */
void process_async();

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
