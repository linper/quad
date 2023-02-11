/**
 * @file ipc.h
 * @brief Utilities fir IPC
 * @author Linas Perkauskas
 * @date 2023-02-08
 */

#include <stdlib.h>

#pragma once

enum conn_addr{
	CONN_ADDR_SIM,
	CONN_ADDR_VIEW,
	CONN_ADDR_AUX,
	MAX_ADDR_TP,
};

/**
 * @brief Function sends request to client and waits for response.
 * @param[in] 		addr 		Connection addressee that identifies which connection will be used.
 * @param[in, out] 	*buf 	Buffer with data to send, and to receive into
 * @param[in] 		n 		Size of `buf`.
 * @return 0 on sucess, 1 - otherwise.
 */
int ipc_conn_request(enum conn_addr tp, char *buf, size_t n);

int ipc_setup();

void ipc_release();
