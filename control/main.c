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
#include "matrix.h"
#include "req.h"
#include "ipc.h"

#define CTL_VER "3.0"

int quiet = 0;
int debug = 0;

static volatile int got_msg = 0;

static void sig_msg(int signo)
{
	(void)signo;

	printf("GOT SIGNAL:%d\n", signo);

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

/**
 * @brief strip whitespace from the start and end of string.
 * @param[in] *string string to strip
 * @return pointer into string.
 */
/*static char *stripwhite(char *string)*/
/*{*/
/*register char *s, *t;*/

/*for (s = string; whitespace(*s); s++)*/
/*;*/

/*if (*s == 0)*/
/*return (s);*/

/*t = s + strlen(s) - 1;*/
/*while (t > s && whitespace(*t))*/
/*t--;*/
/**++t = '\0';*/

/*return s;*/
/*}*/

/**
 * @brief It does exactly that what it is called.
 * @param[in, out] *buf 	Buffer pointer to read and write in IPC
 * @return 0 on succes, 1 otherwise.
 */
/*static void interactive_loop(char *buf)*/
/*{*/
/*char *line_act, *line_data, *s_act, *s_data;*/
/*json_object *j, *d;*/

/*for (;;) {*/
/*if (!(line_act = readline("act > "))) {*/
/*continue;*/
/*}*/

/*s_act = stripwhite(line_act);*/

/*if (*s_act) {*/
/*add_history(s_act);*/
/*} else {*/
/*s_act = EMPTY_DATA;*/
/*}*/

/*if (!strncmp(s_act, "quit", 4)) {*/
/*free(line_act);*/
/*break;*/
/*}*/

/*if (!(line_data = readline("data > "))) {*/
/*free(line_act);*/
/*continue;*/
/*}*/

/*s_data = stripwhite(line_data);*/

/*if (*s_data) {*/
/*add_history(s_data);*/
/*} else {*/
/*s_data = EMPTY_DATA;*/
/*}*/

/*sprintf(buf, REQ_FMT, s_act, s_data);*/

/*free(line_act);*/
/*free(line_data);*/

/*if (ipc_conn_request(CONN_ADDR_SIM, buf)) {*/
/*ERR("Failed to request\n");*/
/*continue;*/
/*}*/

/*if (!strncmp(buf, "exit", 4)) {*/
/*break;*/
/*}*/
/*}*/
/*}*/

/**
 * @brief This function starts `sim` and queries initial model data.
 * @return 0 on succes, 1 - otherwise.
 */
static int start_sim()
{
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

	for (;;) {
		// Calc next step
		model_step();
		// Make `step` request to `sim` and save response
		req_step(&j);
		// Update sensors
		set_sens_from_json(j);
		json_object_put(j);

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

	if (ipc_setup()) {
		FATAL("Failed to setup IPC\n");
	}

	if (start_sim()) {
		FATAL("Failed to start simulationi\n");
	}

	main_loop();

	/*interactive_loop(buf);*/

	ipc_release();

	exit(EXIT_SUCCESS);
}

