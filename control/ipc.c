/**
 * @file ipc.c
 * @brief Utilities fir IPC
 * @author Linas Perkauskas
 * @date 2023-02-08
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <errno.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "log.h"
#include "mth.h"
#include "ipc.h"

#define N_CONNS 2
#define FIFO_PERM (S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH)

#define VC_PATH "/tmp/view_ctl_pipe"
#define CV_PATH "/tmp/ctl_view_pipe"
#define AC_PATH "/tmp/aux_ctl_pipe"
#define CA_PATH "/tmp/ctl_aux_pipe"
#define SC_PATH "/tmp/sim_ctl_pipe"
#define CS_PATH "/tmp/ctl_sim_pipe"

enum conn_tp {
	CONN_TP_FIFO,
};

struct fifo_conn {
	int in_fd;
	int out_fd;
	const char *in_path;
	const char *out_path;
	int in_oflags;
	int out_oflags;
};

struct ipc_conn {
	union {
		struct fifo_conn fifo;
	} conn;
	enum conn_tp tp;
};

static struct ipc_conn *conns[MAX_ADDR_TP];

static struct ipc_conn *fifo_conn_create(const char *in_path,
										 const char *out_path, int in_oflags,
										 int out_oflags)
{
	if (!in_path || !out_path) {
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	struct ipc_conn *ic = calloc(1, sizeof(struct ipc_conn));
	if (!ic) {
		FATAL(ERR_MALLOC_FAIL);
	}

	struct fifo_conn *c = &ic->conn.fifo;

	if (access(in_path, F_OK)) {
		mkfifo(in_path, FIFO_PERM);
	}

	c->in_fd = open(in_path, in_oflags);
	if (c->in_fd == -1) {
		ERR(NLS, strerror(errno));
		return NULL;
	}

	if (access(out_path, F_OK)) {
		mkfifo(out_path, FIFO_PERM);
	}

	c->out_fd = open(out_path, out_oflags);
	if (c->out_fd == -1) {
		ERR(NLS, strerror(errno));
		close(c->in_fd);
		return NULL;
	}

	c->in_path = in_path;
	c->in_oflags = in_oflags;
	c->out_path = out_path;
	c->out_oflags = out_oflags;

	return ic;
}

static void fifo_conn_release(struct fifo_conn *c)
{
	if (c) {
		close(c->in_fd);
		close(c->out_fd);
	}
}

static int fifo_conn_request(struct fifo_conn *conn, char *buf, size_t n)
{
	int ret;

	DBG("Sent: %s", buf);
	/*HEX(buf, strlen(buf));*/

	if (write(conn->out_fd, buf, strlen(buf)) == -1) {
		ERR(NLS, strerror(errno));
		return 1;
	}

	buf[0] = 0;

	ret = read(conn->in_fd, buf, n);
	if (ret == -1) {
		ERR(NLS, strerror(errno));
		return 1;
	}

	buf[MIN(n - 1, (uint)ret)] = 0;

	DBG("Result: %s\n", buf);

	return 0;
}

int ipc_conn_request(enum conn_addr addr, char *buf, size_t n)
{
	struct ipc_conn *conn;

	if (addr < 0 || addr > MAX_ADDR_TP) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	conn = conns[addr];
	if (!conn) {
		ERR("Connection is not setup\n");
		return 1;
	}

	switch (conn->tp) {
	case CONN_TP_FIFO:
		return fifo_conn_request(&conn->conn.fifo, buf, n);
		break;
	}

	return 0;
}

int ipc_setup()
{
	if (!(conns[CONN_ADDR_SIM] =
			  fifo_conn_create(SC_PATH, CS_PATH, O_RDONLY, O_WRONLY))) {
		ERR(ERR_ERROR);
		goto err;
	}

	if (!(conns[CONN_ADDR_VIEW] = fifo_conn_create(
			  VC_PATH, CV_PATH, O_RDONLY | O_NONBLOCK, O_RDWR))) {
		ERR(ERR_ERROR);
		goto err;
	}

	if (!(conns[CONN_ADDR_AUX] = fifo_conn_create(
			  AC_PATH, CA_PATH, O_RDONLY | O_NONBLOCK, O_RDWR))) {
		ERR(ERR_ERROR);
		goto err;
	}

	return 0;
err:
	ipc_release();
	return 1;
}

void ipc_release()
{
	for (int i = 0; i < N_CONNS; i++) {
		if (conns[i]) {
			switch (conns[i]->tp) {
			case CONN_TP_FIFO:
				fifo_conn_release(&conns[i]->conn.fifo);
				break;
			}
		}
	}
}
