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
#define N_RSP_TRIES 3
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
static int64_t req_id = 0;

/**********
*  MISC  *
**********/

static int64_t get_id(struct json_object *j)
{
	struct json_object *tmp;

	if (!json_object_object_get_ex(j, "id", &tmp)) {
		ERR(ERR_PARSE_FAIL);
		return -1;
	}

	int64_t res = json_object_get_int64(tmp);

	return res;
}

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
	if (!self) {
		return;
	}

	fifo_conn_t *c = (fifo_conn_t *)self->priv;

	if (*msg) {
		char buf[IPC_BUF_LEN];
		const char *p = json_object_to_json_string(*msg);
		strncpy(buf, p, IPC_BUF_LEN);
		json_object_put(*msg);
		*msg = NULL;

		if (write(c->out_fd, buf, strlen(buf)) == -1) {
			ERR(NLS, strerror(errno));
			return;
		}

		DBG("Sent[%zu]:%s\n", strlen(buf), buf);
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

static int fifo_conn_request(struct ipc_conn *self, char *buf, bool no_req,
							 bool no_recv)
{
	int ret;

	/*HEX(buf, strlen(buf));*/
	if (!self || !self->priv) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	fifo_conn_t *c = (fifo_conn_t *)self->priv;

	if (!no_req) {
		DBG("Sent: %s\n", buf);
		if (write(c->out_fd, buf, strlen(buf)) == -1) {
			ERR(NLS, strerror(errno));
			return 1;
		}
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
	char rbuf[IPC_BUF_LEN], wbuf[IPC_BUF_LEN], *bptr, *ptr;
	const char *p;
	struct json_object *jreq, *jres;

	rbuf[0] = 0;
	bptr = rbuf;

	if (!self || !self->priv) {
		ERR(ERR_INVALID_INPUT);
		return;
	}

	/*DBG("Path:%s\n", ((fifo_conn_t *)(self->priv))->in_path);*/
	fifo_conn_t *c = (fifo_conn_t *)self->priv;

	ret = read(c->in_fd, rbuf, IPC_BUF_LEN - 1);
	if (ret == -1) {
		ERR(NLS, strerror(errno));
		return;
	} else if (!ret) {
		return;
	}

	rbuf[MIN(IPC_BUF_LEN - 1, (uint)ret)] = 0;

	DBG("Received[%d]:%s\n", ret, rbuf);
	/*if (strstr(rbuf, "\"go\"")) {*/
	/*printf("whatever\n");*/
	/*}*/

	while ((ptr = strsep(&bptr, "\n"))) {
		if (strlen(ptr) < 2) {
			continue;
		}

		jreq = json_tokener_parse(ptr);
		if (!jreq) {
			ERR(ERR_PARSE_FAIL);
			return;
		}

		jres = NULL;
		if (exec_request(self, jreq, &jres)) {
			json_object_put(jreq);
			ERR(ERR_PARSE_FAIL);
			return;
		}

		json_object_put(jreq);
		if (!jres) {
			continue;
		}

		p = json_object_to_json_string(jres);
		strncpy(wbuf, p, IPC_BUF_LEN);
		json_object_put(jres);

		if (write(c->out_fd, wbuf, strlen(wbuf)) == -1) {
			ERR(NLS, strerror(errno));
			return;
		}

		DBG("Sent[%zu]:%s\n", strlen(wbuf), wbuf);
	}
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
	int status = -1;

	if (!*j || addr < 0 || addr >= MAX_ADDR_TP) {
		ERR(ERR_INVALID_INPUT);
		return -1;
	}

	json_object_object_add(*j, "id", json_object_new_int64(req_id++));

	p = json_object_to_json_string(*j);
	strncpy(buf, p, IPC_BUF_LEN);

	c = conns[addr];
	if (!c) {
		ERR("Connection is not setup\n");
		return -1;
	}

	if (c->req(c, buf, false, no_recv)) {
		ERR(ERR_PARSE_FAIL);
		return -1;
	}

	if (no_recv) {
		return 0;
	}

	json_object_put(*j);
	for (int i = 0; i < N_RSP_TRIES; i++) {
		rsp = json_tokener_parse(buf);
		if (!rsp) {
			ERR(ERR_PARSE_FAIL);
			return -1;
		}

		int64_t id = get_id(rsp);
		if (id != req_id - 1) {
			DBG("Invalid response tries:%d\n", i);
			if (c->req(c, buf, true, no_recv)) {
				ERR(ERR_PARSE_FAIL);
				json_object_put(rsp);
				return -1;
			}

			json_object_put(rsp);
			continue;
		}

		// Status
		if (!json_object_object_get_ex(rsp, "status", &tmp)) {
			ERR(ERR_PARSE_FAIL);
			json_object_put(rsp);
			return -1;
		}

		status = json_object_get_int(tmp);

		*j = rsp;
		break;
	}

	return status;
}

void process_async()
{
	for (int i = 0; i < MAX_ADDR_TP; i++) {
		struct ipc_conn *c = conns[i];
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
	for (int i = 0; i < N_CONNS; i++) {
		struct ipc_conn *c = conns[i];
		if (!c) {
			ERR("Connection is not setup\n");
			continue;
		}

		c->free(c);
	}
}
