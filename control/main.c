/**
 * @file control_main.c
 * @brief Main file and entry point for "control" program
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <readline/readline.h>
#include <readline/history.h>

#include "log.h"
#include "mth.h"
#include "model.h"
#include "ipc.h"

#define CTL_VER "3.0"

#define EMPTY_DATA "empty"
#define REQ_FMT "{\"act\": \"%s\", \"data\": \"%s\"}\n"
#define SETUP_REQ "{\"act\": \"setup\", \"data\": \"empty\"}\n"
#define MODEL_REQ "{\"act\": \"model\", \"data\": \"empty\"}\n"
#define SENS_REQ "{\"act\": \"sens\", \"data\": \"empty\"}\n"
#define STEP_REQ "{\"act\": \"step\", \"data\": {\"angles\": %s}}\n"

#define READ_BUF_LEN 2048

int quiet = 0;
int debug = 0;

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

/**
 * @brief strip whitespace from the start and end of string.
 * @param[in] *string string to strip
 * @return pointer into string.
 */
static char *stripwhite(char *string)
{
	register char *s, *t;

	for (s = string; whitespace(*s); s++)
		;

	if (*s == 0)
		return (s);

	t = s + strlen(s) - 1;
	while (t > s && whitespace(*t))
		t--;
	*++t = '\0';

	return s;
}

static int make_model_request(char *buf)
{
	strcpy(buf, MODEL_REQ);
	return ipc_conn_request(CONN_ADDR_SIM, buf, READ_BUF_LEN);
}

static int make_sens_request(char *buf)
{
	strcpy(buf, SENS_REQ);
	return ipc_conn_request(CONN_ADDR_SIM, buf, READ_BUF_LEN);
}

static int make_setup_request(char *buf)
{
	strcpy(buf, SETUP_REQ);
	return ipc_conn_request(CONN_ADDR_SIM, buf, READ_BUF_LEN);
}

static int make_step_request(char *buf)
{
	char *ang = mat_to_str(g_model->angles);
	if (!ang) {
		ERR("Failed to format 'angles'\n");
		return 1;
	}

	snprintf(buf, READ_BUF_LEN, STEP_REQ, ang);
	free(ang);

	return ipc_conn_request(CONN_ADDR_SIM, buf, READ_BUF_LEN);
}

/**
 * @brief It does exactly that what it is called.
 * @param[in, out] *buf 	Buffer pointer to read and write in IPC
 * @return 0 on succes, 1 otherwise.
 */
static void interactive_loop(char *buf)
{
	char *line_act, *line_data, *s_act, *s_data;

	for (;;) {
		if (!(line_act = readline("act > "))) {
			continue;
		}

		s_act = stripwhite(line_act);

		if (*s_act) {
			add_history(s_act);
		} else {
			s_act = EMPTY_DATA;
		}

		if (!strncmp(s_act, "quit", 4)) {
			free(line_act);
			break;
		}

		if (!(line_data = readline("data > "))) {
			free(line_act);
			continue;
		}

		s_data = stripwhite(line_data);

		if (*s_data) {
			add_history(s_data);
		} else {
			s_data = EMPTY_DATA;
		}

		sprintf(buf, REQ_FMT, s_act, s_data);

		free(line_act);
		free(line_data);

		if (ipc_conn_request(CONN_ADDR_SIM, buf, READ_BUF_LEN)) {
			ERR("Failed to request\n");
			continue;
		}

		if (!strncmp(buf, "exit", 4)) {
			break;
		}
	}
}

/**
 * @brief This function starts `sim` and queries initial model data.
 * @param[in. out] *buf 	Buffer ptr to be used in IPC.
 * @return 0 on succes, 1 - otherwise.
 */
static int start_sim(char *buf)
{
	DBG("SETTING UP...\n");
	if (make_setup_request(buf)) {
	ERR("Setting up failed\n");
	return 1;
	}

	DBG("GETTING MODEL...\n");
	if (make_model_request(buf)) {
	ERR("Geting model failed\n");
	return 1;
	}

	/*strcpy(*/
		/*buf,*/
		/*"{\"legs\": [{\"name\": \"front_left\", \"idx\": 0, \"pos\": [-0.125, -0.1, -0.26], \"def_pos\": [-0.125, -0.1, -0.26], \"base_off\": [-0.125, -0.05, -0.03], \"dir\": [-1, 1, -1], \"joint_lims\": [-0.5236, 0.245, -1.309, 1.309, -0.5236, -2.618]}, {\"name\": \"front_right\", \"idx\": 1, \"pos\": [-0.125, 0.1, -0.26], \"def_pos\": [-0.125, 0.1, -0.26], \"base_off\": [-0.125, 0.05, -0.03], \"dir\": [-1, 1, 1], \"joint_lims\": [-0.245, 0.5236, -1.309, 1.309, -0.5236, -2.618]}, {\"name\": \"back_left\", \"idx\": 2, \"pos\": [0.125, -0.1, -0.26], \"def_pos\": [0.125, -0.1, -0.26], \"base_off\": [0.125, -0.05, -0.03], \"dir\": [-1, 1, -1], \"joint_lims\": [-0.5236, 0.245, -1.309, 1.309, -0.5236, -2.618]}, {\"name\": \"back_right\", \"idx\": 3, \"pos\": [0.125, 0.1, -0.26], \"def_pos\": [0.125, 0.1, -0.26], \"base_off\": [0.125, 0.05, -0.03], \"dir\": [-1, 1, 1], \"joint_lims\": [-0.245, 0.5236, -1.309, 1.309, -0.5236, -2.618]}], \"base\": {\"max_dip\": -0.35, \"leg_tar_h\": -0.26, \"t_rad\": 0.012, \"cw\": [0, 1, 3, 2], \"link_len\": 0.1}}");*/

	DBG("PARSING MODEL...\n");
	if (model_from_json(buf)) {
		ERR("Parsing model failed\n");
		return 1;
	}

	DBG("GETTING SENS...\n");
	if (make_sens_request(buf)) {
	ERR("Getting sensor info failed\n");
	return 1;
	}

	/*strcpy(*/
		/*buf,*/
		/*"{\"avg_leg_h\": -0.26, \"touch_force\": [0, 0, 0, 0], \"damp\": [0.0, 0.0, 0.0, 0.0], \"bf_vec\": [0.0, 0.0, 0.0], \"bfo_mat\": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], \"t_force\": {\"type\": 3, \"pos\": [0.0, 0.0, 0.0]}}");*/

	DBG("PARSING SENS...\n");
	if (set_sens_from_json(buf)) {
		ERR("Parsing sens failed\n");
		return 1;
	}

	return 0;
}

/**
 * @brief Main execution loop of the program.
 * @param[in, out] *buf 	Buffer pointer to read and write in IPC
 * @return 0 on succes, 1 otherwise.
 */
static int main_loop(char *buf)
{
	for (;;) {
		// Check and handle IO
		// Calc next step
		// Send current state to `view`
		// Get angles
		model_get_angles();
		// Make `step` request to `sim` and save response
		make_step_request(buf);
	}

	return 0;
}

int main(int argc, char *argv[])
{
	char buf[READ_BUF_LEN];
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

	if (ipc_setup()) {
	FATAL("Failed to setup IPC\n");
	}

	if (start_sim(buf)) {
		FATAL("Failed to start simulationi\n");
	}

	main_loop(buf);

	interactive_loop(buf);

	ipc_release();

	exit(EXIT_SUCCESS);
}

