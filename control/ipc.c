/**
 * @file ipc.c
 * @brief Utilities for IPC
 * @author Linas Perkauskas
 * @date 2023-02-08
 */

#include <json-c/json_types.h>
#include <req.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <json-c/json_object.h>

#include "log.h"
#include "mth.h"
#include "ipc.h"
#include "req.h"

#define N_CONNS 2
#define FIFO_PERM (S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH)

#define VC_PATH "/tmp/view_ctl_pipe"
#define CV_PATH "/tmp/ctl_view_pipe"
#define AC_PATH "/tmp/aux_ctl_pipe"
#define CA_PATH "/tmp/ctl_aux_pipe"
#define SC_PATH "/tmp/sim_ctl_pipe"
#define CS_PATH "/tmp/ctl_sim_pipe"

typedef struct fifo_conn {
	int in_fd;
	int out_fd;
	const char *in_path;
	const char *out_path;
	int in_oflags;
	int out_oflags;
} fifo_conn_t;

static ipc_conn_t *conns[MAX_ADDR_TP];

/**********
*  FIFO  *
**********/

static void fifo_conn_complete(struct ipc_conn *self)
{
	if (!self) {
		return;
	}

	fifo_conn_t *c = (fifo_conn_t *)self->priv;
	if (c && !c->out_fd && self->flags & CONN_FL_LAZY_CONNECT) {
		c->out_fd = open(c->out_path, c->out_oflags);
		if (c->out_fd == -1) {
			ERR(NLS, strerror(errno));
			close(c->in_fd);
			c->out_fd = 0;
		}
	}
}

static void fifo_conn_suspend(struct ipc_conn *self, struct json_object **msg)
{
	char buf[IPC_BUF_LEN];
	const char *p;

	if (!self) {
		return;
	}

	fifo_conn_t *c = (fifo_conn_t *)self->priv;

	if (*msg) {
		p = json_object_to_json_string(*msg);
		strncpy(buf, p, IPC_BUF_LEN);
		json_object_put(*msg);
		*msg = NULL;

		if (write(c->out_fd, buf, strlen(buf)) == -1) {
			ERR(NLS, strerror(errno));
			return;
		}

		DBG("Sent[%lu]:%s\n", strlen(buf), buf);
	}

	if (c && c->out_fd && self->flags & CONN_FL_LAZY_CONNECT) {
		close(c->out_fd);
		c->out_fd = 0;
	}
}

static void fifo_conn_free(struct ipc_conn *self)
{
	if (!self) {
		return;
	}

	fifo_conn_t *c = (fifo_conn_t *)self->priv;
	if (c) {
		close(c->in_fd);
		if (c->out_fd)
			close(c->out_fd);

		free(c);
	}

	free(self);
}

static int fifo_conn_request(struct ipc_conn *self, char *buf, bool no_recv)
{
	int ret;

	DBG("Sent: %s\n", buf);
	/*HEX(buf, strlen(buf));*/
	if (!self || !self->priv) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	fifo_conn_t *c = (fifo_conn_t *)self->priv;

	if (write(c->out_fd, buf, strlen(buf)) == -1) {
		ERR(NLS, strerror(errno));
		return 1;
	}

	if (no_recv) {
		return 0;
	}

	buf[0] = 0;

	ret = read(c->in_fd, buf, IPC_BUF_LEN);
	if (ret == -1) {
		ERR(NLS, strerror(errno));
		return 1;
	}

	buf[MIN(IPC_BUF_LEN - 1, (uint)ret)] = 0;

	DBG("Result: %s\n", buf);
	return 0;
}

static void fifo_conn_process_async(struct ipc_conn *self)
{
	int ret;
	char buf[IPC_BUF_LEN];
	const char *p;
	struct json_object *jreq, *jres = NULL;

	buf[0] = 0;

	if (!self || !self->priv) {
		ERR(ERR_INVALID_INPUT);
		return;
	}

	/*DBG("Path:%s\n", ((fifo_conn_t *)(self->priv))->in_path);*/
	fifo_conn_t *c = (fifo_conn_t *)self->priv;

	ret = read(c->in_fd, buf, IPC_BUF_LEN - 1);
	if (ret == -1) {
		ERR(NLS, strerror(errno));
		return;
	} else if (!ret) {
		return;
	}

	buf[MIN(IPC_BUF_LEN - 1, (uint)ret)] = 0;

	DBG("Received[%d]:%s\n", ret, buf);

	jreq = json_tokener_parse(buf);
	if (!jreq) {
		ERR(ERR_PARSE_FAIL);
		return;
	}

	if (exec_request(self, jreq, &jres)) {
		json_object_put(jreq);
		ERR(ERR_PARSE_FAIL);
		return;
	}

	json_object_put(jreq);
	if (!jres) {
		return;
	}

	p = json_object_to_json_string(jres);
	strncpy(buf, p, IPC_BUF_LEN);
	json_object_put(jres);

	if (write(c->out_fd, buf, strlen(buf)) == -1) {
		ERR(NLS, strerror(errno));
		return;
	}

	DBG("Sent[%lu]:%s\n", strlen(buf), buf);

	return;
}

static ipc_conn_t *fifo_conn_create(const char *in_path, const char *out_path,
									int in_oflags, int out_oflags,
									enum conn_flags flags)
{
	if (!in_path || !out_path) {
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	struct ipc_conn *ic = calloc(1, sizeof(struct ipc_conn));
	struct fifo_conn *c = calloc(1, sizeof(struct fifo_conn));
	if (!ic || !c) {
		FATAL(ERR_MALLOC_FAIL);
	}

	ic->priv = c;

	ic->tp = CONN_TP_FIFO;
	ic->flags = flags;
	ic->free = fifo_conn_free;
	ic->req = fifo_conn_request;
	ic->async = fifo_conn_process_async;
	ic->complete = fifo_conn_complete;
	ic->suspend = fifo_conn_suspend;

	if (access(in_path, F_OK)) {
		mkfifo(in_path, FIFO_PERM);
	}

	if (flags & CONN_FL_ASYNC_READ) {
		// Must not set O_ASYNC when opening fifo. Because signals won't work
		c->in_fd = open(in_path, in_oflags);
		if (c->in_fd == -1) {
			goto err;
		}

		if (fcntl(c->in_fd, F_SETOWN, getpid()) == -1) {
			goto err;
		}

		/*if (fcntl(c->in_fd, F_SETFL, O_ASYNC) == -1) {*/
		if (fcntl(c->in_fd, F_SETFL, O_ASYNC | O_NONBLOCK) == -1) {
			goto err;
		}
	} else {
		c->in_fd = open(in_path, in_oflags);
		if (c->in_fd == -1) {
			goto err;
		}
	}

	if (access(out_path, F_OK)) {
		mkfifo(out_path, FIFO_PERM);
	}

	if (!(flags & CONN_FL_LAZY_CONNECT)) {
		c->out_fd = open(out_path, out_oflags);
		if (c->out_fd == -1) {
			goto err;
		}
	}

	c->in_path = in_path;
	c->in_oflags = in_oflags;
	c->out_path = out_path;
	c->out_oflags = out_oflags;

	return ic;
err:

	ERR(NLS, strerror(errno));
	fifo_conn_free(ic);
	return NULL;
}

/**********
*  BASE  *
**********/

int ipc_conn_request_ex(enum conn_addr addr, struct json_object **j,
						bool no_recv)
{
	struct ipc_conn *c;
	char buf[IPC_BUF_LEN];
	const char *p;
	struct json_object *rsp, *tmp;

	if (!*j || addr < 0 || addr > MAX_ADDR_TP) {
		ERR(ERR_INVALID_INPUT);
		return -1;
	}

	p = json_object_to_json_string(*j);
	strncpy(buf, p, IPC_BUF_LEN);

	c = conns[addr];
	if (!c) {
		ERR("Connection is not setup\n");
		return -1;
	}

	if (c->req(c, buf, no_recv)) {
		ERR(ERR_PARSE_FAIL);
		return -1;
	}

	if (no_recv) {
		return 0;
	}

	json_object_put(*j);

	rsp = json_tokener_parse(buf);
	if (!rsp) {
		ERR(ERR_PARSE_FAIL);
		return -1;
	}

	*j = rsp;

	// Status
	if (!json_object_object_get_ex(rsp, "status", &tmp)) {
		ERR(ERR_PARSE_FAIL);
		return 1;
	}

	int status = json_object_get_int(tmp);

	return status;
}

void process_async()
{
	struct ipc_conn *c;

	for (int i = 0; i < MAX_ADDR_TP; i++) {
		c = conns[i];
		if (!c) {
			ERR("Connection is not setup\n");
			continue;
		}

		if (c->flags & CONN_FL_ASYNC_READ) {
			c->async(c);
		}
	}
}

int ipc_setup()
{
	if (!(conns[CONN_ADDR_SIM] =
			  fifo_conn_create(SC_PATH, CS_PATH, O_RDONLY, O_WRONLY, 0))) {
		ERR(ERR_ERROR);
		goto err;
	}

	if (!(conns[CONN_ADDR_VIEW] = fifo_conn_create(
			  VC_PATH, CV_PATH, O_RDONLY | O_NONBLOCK, O_WRONLY,
			  CONN_FL_LAZY_CONNECT | CONN_FL_ASYNC_READ))) {
		ERR(ERR_ERROR);
		goto err;
	}

	if (!(conns[CONN_ADDR_AUX] = fifo_conn_create(
			  AC_PATH, CA_PATH, O_RDONLY | O_NONBLOCK, O_WRONLY,
			  CONN_FL_LAZY_CONNECT | CONN_FL_ASYNC_READ))) {
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
	struct ipc_conn *c;

	for (int i = 0; i < N_CONNS; i++) {
		c = conns[i];
		if (!c) {
			ERR("Connection is not setup\n");
			continue;
		}

		c->free(c);
	}
}
