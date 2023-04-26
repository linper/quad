/**
 * @file control_main.c
 * @brief Main file and entry point for "control" program
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

#include <sys/types.h>
#include <sys/stat.h>
/*#include <sys/un.h>*/

/*#include <readline/readline.h>*/
/*#include <readline/history.h>*/
#include <json-c/json_object.h>
#include <json-c/json_types.h>

#include "log.h"
#include "mth.h"
#include "model.h"
#include "req.h"
#include "ipc.h"

#define CTL_VER "3.0"

static const char *model_str =
	"{\"id\": -1, \"act\": \"model\", \"rsp\": {\"base\": {\"max_dip\": -0.35, \"min_dip\": -0.1, \"max_walk_h\": -0.13, \"min_walk_h\": -0.2, \"leg_tar_h\": -0.26, \"t_rad\": 0.012, \"cw\": [0, 1, 3, 2], \"link_len\": 0.1, \"soft_hit_thr\": 0.02}, \"legs\": [{\"name\": \"front_left\", \"idx\": 0, \"pos\": [-0.125, -0.1, -0.26], \"def_pos\": [-0.125, -0.1, -0.26], \"base_off\": [-0.125, -0.05, -0.03], \"dir\": [-1, 1, -1], \"joint_lims\": [-0.5236, 0.245, -1.309, 1.309, -0.5236, -2.618]}, {\"name\": \"front_right\", \"idx\": 1, \"pos\": [-0.125, 0.1, -0.26], \"def_pos\": [-0.125, 0.1, -0.26], \"base_off\": [-0.125, 0.05, -0.03], \"dir\": [-1, 1, 1], \"joint_lims\": [-0.245, 0.5236, -1.309, 1.309, -0.5236, -2.618]}, {\"name\": \"back_left\", \"idx\": 2, \"pos\": [0.125, -0.1, -0.26], \"def_pos\": [0.125, -0.1, -0.26], \"base_off\": [0.125, -0.05, -0.03], \"dir\": [-1, 1, -1], \"joint_lims\": [-0.5236, 0.245, -1.309, 1.309, -0.5236, -2.618]}, {\"name\": \"back_right\", \"idx\": 3, \"pos\": [0.125, 0.1, -0.26], \"def_pos\": [0.125, 0.1, -0.26], \"base_off\": [0.125, 0.05, -0.03], \"dir\": [-1, 1, 1], \"joint_lims\": [-0.245, 0.5236, -1.309, 1.309, -0.5236, -2.618]}]}, \"status\": 0}";

static const char *sens_str =
	"{\"id\": -1, \"act\": \"step\", \"rsp\": {\"avg_leg_h\": -0.26, \"touch_force\": [3, 13, -2, -2], \"abs_std_leg_h\": 0.0, \"damp\": [3.3617, 3.3446, 0.0542, 0.0539], \"bf_vec\": [1.1619, 0.0116, -4.1236], \"bfo_mat\": [[0.9975, 0.0001, -0.0709], [-0.0, 1.0, 0.0015], [0.0709, -0.0015, 0.9975]], \"t_force\": {\"type\": 1, \"pos\": [0.07326, 0.00073, -0.26]}}, \"status\": 0}";

int quiet = 0;
int debug = 0;

static volatile int got_msg = 0;

static void sig_msg(int signo)
{
	(void)signo;

	DBG("GOT SIGNAL:%d\n", signo);

	got_msg = 1;
}

static void help()
{
	const char *usage = "\
Usage: contol [OPTION...]\n\
  -q 		Do not output anything (except fatal mesages)\n\
  -d 		Enable debug mesages\n\
  -?, -h 	Give this help list\n\
  -V 		Print program version\n\
";
	puts(usage);
}

static int start_nosim()
{
	struct json_object *j;

	DBG("PARSING MODEL...\n");
	j = json_tokener_parse(model_str);
	if (!j) {
		ERR("Parsing model string failed\n");
		return 1;
	}

	if (model_from_json(j)) {
		ERR("Parsing model failed\n");
		json_object_put(j);
		return 1;
	}

	json_object_put(j);

	DBG("PARSING SENS...\n");
	j = json_tokener_parse(sens_str);
	if (!j) {
		ERR("Parsing sens string failed\n");
		return 1;
	}
	if (set_sens_from_json(j)) {
		g_model->sens = NULL;
		ERR("Parsing sens failed\n");
		json_object_put(j);
		return 1;
	}

	json_object_put(j);

	return 0;
}

/**
 * @brief This function starts `sim` and queries initial model data.
 * @return 0 on succes, 1 - otherwise.
 */
static int start_sim()
{
#ifdef NOSIM
	return start_nosim();
#else
	(void)start_nosim();
#endif

	struct json_object *j;

	DBG("SETTING UP...\n");
	if (req_setup()) {
		ERR("Setting up failed\n");
		return 1;
	}

	DBG("GETTING MODEL...\n");
	if (req_model(&j)) {
		ERR("Geting model failed\n");
		return 1;
	}

	DBG("PARSING MODEL...\n");
	if (model_from_json(j)) {
		ERR("Parsing model failed\n");
		json_object_put(j);
		return 1;
	}

	json_object_put(j);

	DBG("GETTING SENS...\n");
	if (req_sens(&j)) {
		ERR("Getting sensor info failed\n");
		return 1;
	}

	DBG("PARSING SENS...\n");
	if (set_sens_from_json(j)) {
		g_model->sens = NULL;
		ERR("Parsing sens failed\n");
		json_object_put(j);
		return 1;
	}

	json_object_put(j);

	return 0;
}

/**
 * @brief Main execution loop of the program.
 * @return 0 on succes, 1 otherwise.
 */
static int main_loop()
{
	struct json_object *j;

	// Calc iinitial step
	model_step();

	for (;;) {
#ifndef NOSIM
		// Make `step` request to `sim` and save response
		req_step(&j);
#else
		j = json_tokener_parse(sens_str);
#endif
		// Update sensors
		set_sens_from_json(j);
		json_object_put(j);

		// Calc next step
		model_step();

		// Check and handle AIO
		if (got_msg) {
			got_msg = 0;
			process_async();
		}
	}

	return 0;
}

int main(int argc, char *argv[])
{
	/*char buf[IPC_BUF_LEN];*/
	int choice;

	/* Argument parameters:
		no_argument: " "
		required_argument: ":"
		optional_argument: "::" */
	while ((choice = getopt(argc, argv, "Vhqd")) != -1) {
		switch (choice) {
		case 'V':
			puts(CTL_VER);
			return 0;

		case '?':
		case 'h':
			help();
			return 0;

		case 'q':
			quiet = 1;
			break;

		case 'd':
			debug = 1;
			break;

		default:
			/* Not sure how to get here... */
			return EXIT_FAILURE;
		}
	}

	signal(SIGIO, sig_msg);

	json_c_set_serialization_double_format("%.5g", JSON_C_OPTION_THREAD);

#ifndef NOSIM
	if (ipc_setup()) {
		FATAL("Failed to setup IPC\n");
	}
#endif

	if (start_sim()) {
		FATAL("Failed to start simulationi\n");
	}

	main_loop();

	/*interactive_loop(buf);*/

	ipc_release();

	exit(EXIT_SUCCESS);
}

