/**
 * @file req.c
 * @brief Utilities for reqest handling (on tom of IPC)
 * @author Linas Perkauskas
 * @date 2023-02-13
 */

#include <gsl/gsl_vector_double.h>
#include <mth.h>
#include <stance.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/*#include <errno.h>*/
/*#include <unistd.h>*/
/*#include <fcntl.h>*/

#include <sys/stat.h>
#include <sys/types.h>

#include <json-c/json_object.h>
#include <json-c/json_types.h>

#include "log.h"
#include "model.h"
#include "req.h"
#include "ipc.h"

#define STEP_ACT "step"
#define SETUP_ACT "setup"
#define MODEL_ACT "model"
#define SENS_ACT "sens"
#define ACT_LABEL "act"
#define DATA_LABEL "data"
#define ANGLES_LABEL "angles"

#define ARRAY_SIZE(x) ((sizeof x) / (sizeof *x))

struct cb_map {
	const char *act;
	int (*cb)(ipc_conn_t *, struct json_object *, struct json_object **);
};

void get_request_template(const char *act, struct json_object **root,
						  struct json_object **data)
{
	json_object *r, *d;

	r = json_object_new_object();
	d = json_object_new_object();

	if (!r || !d) {
		FATAL(ERR_MALLOC_FAIL);
	} else if (!act) {
		FATAL(ERR_INVALID_INPUT);
	}

	json_object_object_add(r, "act", json_object_new_string(act));
	json_object_object_add(r, "data", d);

	*root = r;
	*data = d;
}

int req_model(struct json_object **j)
{
	json_object *d;

	get_request_template(MODEL_ACT, j, &d);
	if (!*j) {
		ERR(ERR_ERROR);
		return 1;
	}

	return ipc_conn_request(CONN_ADDR_SIM, j);
}

int req_sens(struct json_object **j)
{
	json_object *d;

	get_request_template(SENS_ACT, j, &d);
	if (!g_model || !*j) {
		ERR(ERR_ERROR);
		return 1;
	}

	return ipc_conn_request(CONN_ADDR_SIM, j);
}

int req_setup()
{
	struct json_object *j, *d;

	get_request_template(SETUP_ACT, &j, &d);
	if (!j) {
		ERR(ERR_ERROR);
		return 1;
	}

	int ret = ipc_conn_request(CONN_ADDR_SIM, &j);

	json_object_put(j);

	return ret;
}

int req_step(struct json_object **j)
{
	json_object *d;

	get_request_template(STEP_ACT, j, &d);
	if (!g_model || !*j) {
		ERR(ERR_ERROR);
		return 1;
	}

	json_object_object_add(d, ANGLES_LABEL, matrix_to_json(g_model->angles));

	return ipc_conn_request(CONN_ADDR_SIM, j);
}

/**
 * @brief Funcion that adds aditional leg info to json object 
 * (in addition to sens info) and then notifies `view`.
 * Does not wait for response.
 * @return 0 on success, 1 - otherwise.
 */
static int rsp_vstep(ipc_conn_t *conn, struct json_object *jreq,
					 struct json_object **jrsp)
{
	(void)jreq;
	(void)conn;

	json_object *j, *d, *lgs;

	j = json_object_new_object();
	d = json_object_new_object();
	json_object_object_add(j, "act", json_object_new_string("vstep"));

	*jrsp = j;

	json_object_object_add(d, "leg_avg_h",
						   json_object_new_double(g_model->sens->avg_leg_h));
	json_object_object_add(d, "touch_force",
						   block_to_json(g_model->sens->touch_f));
	json_object_object_add(d, "damp", block_to_json(g_model->sens->damp));
	json_object_object_add(d, "tf_pos", vector_to_json(g_model->sens->tf_pos));
	json_object_object_add(d, "tf_type",
						   json_object_new_int(g_model->sens->tf_type));

	lgs = json_object_new_array_ext(N_LEGS);
	for (int i = 0; i < N_LEGS; i++) {
		json_object *l = json_object_new_object();
		leg_t *leg = g_model->legs[i];
		json_object_object_add(l, "bal", json_object_new_int(leg->bal));
		json_object_object_add(l, "idx", json_object_new_int(leg->idx));
		json_object_object_add(l, "name", json_object_new_string(leg->name));
		json_object_object_add(l, "pos", vector_to_json(leg->pos));
		json_object_object_add(l, "def_pos", vector_to_json(leg->def_pos));

		json_object_array_put_idx(lgs, i, l);
	}

	json_object_object_add(d, "legs", lgs);

	json_object_object_add(j, "status", json_object_new_int(RQS_OK));
	json_object_object_add(j, "rsp", d);

	return 0;
}

static int rsp_go(ipc_conn_t *conn, struct json_object *jreq,
				  struct json_object **jrsp)
{
	(void)jrsp;
	(void)conn;

	int n_tasks, n_points, lidx;
	double *pos_arr, dir_arr[2];
	bool do_lift;

	json_object *jtasks, *jtask, *jpts, *jpt, *tmp1, *jdir;

	if (!json_object_object_get_ex(jreq, "n_tasks", &tmp1))
		goto err;

	n_tasks = json_object_get_int(tmp1);

	if (!json_object_object_get_ex(jreq, "tasks", &jtasks))
		goto err;

	for (int i = 0; i < n_tasks; i++) {
		if (!(jtask = json_object_array_get_idx(jtasks, i)))
			goto err;

		if (!json_object_object_get_ex(jtask, "do_lift", &tmp1))
			goto err;

		do_lift = json_object_get_boolean(tmp1);

		if (!json_object_object_get_ex(jtask, "idx", &tmp1))
			goto err;

		lidx = json_object_get_int(tmp1);

		/*if (lidx < 0)*/
		/*continue;*/

		if (!json_object_object_get_ex(jtask, "direction", &jdir))
			goto err;

		// TODO currenty uses directionfrom last task
		for (int j = 0; j < 2; j++) {
			if (!(tmp1 = json_object_array_get_idx(jdir, j)))
				goto err;

			dir_arr[j] = json_object_get_double(tmp1);
		}

		if (!json_object_object_get_ex(jtask, "n_points", &tmp1))
			goto err;

		n_points = json_object_get_int(tmp1);

		pos_arr = malloc(3 * n_points * sizeof(double));
		if (!pos_arr) {
			FATAL(ERR_MALLOC_FAIL);
		}

		if (!json_object_object_get_ex(jtask, "points", &jpts))
			goto err;

		for (int j = 0; j < n_points; j++) {
			if (!(jpt = json_object_array_get_idx(jpts, j)))
				goto err;

			pos_arr[3 * j + 0] =
				json_object_get_double(json_object_array_get_idx(jpt, 0));
			pos_arr[3 * j + 1] =
				json_object_get_double(json_object_array_get_idx(jpt, 1));
			pos_arr[3 * j + 2] =
				json_object_get_double(json_object_array_get_idx(jpt, 2));
		}

		gsl_vector *dir = vector_from_array(2, dir_arr);

		if (lidx != -1) {
			plan_make_movement(&g_model->legs[lidx]->plan, pos_arr, n_points,
							   do_lift);
		} else {
			gsl_vector *pt = vector_from_array(2, pos_arr);
			get_movement(dir, pt);
			gsl_vector_free(pt);
		}

		free(pos_arr);
	}

	return 0;
err:

	ERR("Parsing go command failed \n");
	return 1;
}

static int rsp_hello(ipc_conn_t *conn, struct json_object *jreq,
					 struct json_object **jrsp)
{
	(void)jreq;

	conn->complete(conn);

	struct json_object *j = json_object_new_object();
	json_object_object_add(j, "act", json_object_new_string("hello"));
	json_object_object_add(j, "status", json_object_new_int(RQS_OK));
	json_object_object_add(j, "rsp", json_object_new_object());

	*jrsp = j;

	return 0;
}

static int rsp_bye(ipc_conn_t *conn, struct json_object *jreq,
				   struct json_object **jrsp)
{
	(void)jreq;

	struct json_object *j = json_object_new_object();

	json_object_object_add(j, "act", json_object_new_string("bye"));
	json_object_object_add(j, "status", json_object_new_int(RQS_OK));
	json_object_object_add(j, "rsp", json_object_new_object());
	*jrsp = j;

	conn->suspend(conn, jrsp);

	return 0;
}

static struct cb_map func_map[] = {
	{ .act = "vstep", .cb = rsp_vstep },
	{ .act = "go", .cb = rsp_go },
	{ .act = "hello", .cb = rsp_hello },
	{ .act = "bye", .cb = rsp_bye },
};

int exec_request(ipc_conn_t *conn, struct json_object *jreq,
				 struct json_object **jrsp)
{
	struct json_object *tmp;
	const char *act_str;

	if (!jreq || !jrsp) {
		goto err;
	}

	if (!json_object_object_get_ex(jreq, "act", &tmp)) {
		goto err;
	}

	act_str = json_object_get_string(tmp);

	if (!json_object_object_get_ex(jreq, "data", &tmp)) {
		goto err;
	}

	for (size_t i = 0; i < ARRAY_SIZE(func_map); i++) {
		if (!strcmp(func_map[i].act, act_str)) {
			if (func_map[i].cb(conn, tmp, jrsp)) {
				DBG("Failed to respond to :%s\n", act_str);
			}
			return 0;
		}
	}

	DBG(ERR_NOT_FOUND);
err:
	*jrsp = NULL;

	return 1;
}
