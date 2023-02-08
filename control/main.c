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
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <readline/readline.h>
#include <readline/history.h>

#include "log.h"
#include "mth.h"
#include "model.h"

#define EMPTY_DATA "empty"
#define REQ_FMT "{\"act\": \"%s\", \"data\": \"%s\"}\n"
#define SETUP_REQ "{\"act\": \"setup\", \"data\": \"empty\"}\n"
#define MODEL_REQ "{\"act\": \"model\", \"data\": \"empty\"}\n"
#define SENS_REQ "{\"act\": \"sens\", \"data\": \"empty\"}\n"

#define SOCK_PATH "/tmp/pe_sock"
#define READ_BUF_LEN 2048

#define VC_PATH "/tmp/view_ctl_pipe"
#define SC_PATH "/tmp/sim_ctl_pipe"
#define CS_PATH "/tmp/ctl_sim_pipe"

static int vc_fd, sc_fd, cs_fd;

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

/**
 * @brief Function sends request to `sim` and waits for response.
 * @param[in, out] *buf 	Buffer with data to send, and to receive into
 * @param[in] 		sock	Socket 
 * @return 0 on sucess, 1 - otherwise.
 */
static int make_request(char *buf)
{
	int ret;
	DBG("Sent: %s", buf);
	/*HEX(buf, strlen(buf));*/

	if (write(cs_fd, buf, strlen(buf)) == -1) {
		ERR(NLS, strerror(errno));
		return 1;
	}

	buf[0] = 0;

	ret = read(sc_fd, buf, READ_BUF_LEN);
	if (ret == -1) {
		ERR(NLS, strerror(errno));
		return 1;
	}

	buf[MIN(READ_BUF_LEN - 1, (uint)ret)] = 0;

	DBG("Result: %s\n", buf);

	return 0;
}

static int make_model_request(char *buf)
{
	strcpy(buf, MODEL_REQ);
	return make_request(buf);
}

static int make_sens_request(char *buf)
{
	strcpy(buf, SENS_REQ);
	return make_request(buf);
}

static int make_setup_request(char *buf)
{
	strcpy(buf, SETUP_REQ);
	return make_request(buf);
}

/**
 * @brief It does exactly that what it is called.
 * @param[in] sock 		Socket to communicate with `sim`
 * @return 0 on succes, 1 otherqise.
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

		if (make_request(buf)) {
			ERR("Failed to request\n");
			continue;
		}

		if (!strncmp(buf, "exit", 4)) {
			break;
		}
	}
}

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
	/*"{\"legs\": [{\"name\": \"front_left\", \"idx\": 0, \"pos\": [-0.125, -0.1, -0.26], \"def_pos\": [-0.125, -0.1, -0.26], \"base_off\": [-0.125, -0.05, -0.03], \"dir\": [-1, 1, -1], \"joint_lims\": [-0.5236, 0.245, -1.309, 1.309, -0.5236, -2.618]}, {\"name\": \"front_right\", \"idx\": 1, \"pos\": [-0.125, 0.1, -0.26], \"def_pos\": [-0.125, 0.1, -0.26], \"base_off\": [-0.125, 0.05, -0.03], \"dir\": [-1, 1, 1], \"joint_lims\": [-0.245, 0.5236, -1.309, 1.309, -0.5236, -2.618]}, {\"name\": \"back_left\", \"idx\": 2, \"pos\": [0.125, -0.1, -0.26], \"def_pos\": [0.125, -0.1, -0.26], \"base_off\": [0.125, -0.05, -0.03], \"dir\": [-1, 1, -1], \"joint_lims\": [-0.5236, 0.245, -1.309, 1.309, -0.5236, -2.618]}, {\"name\": \"back_right\", \"idx\": 3, \"pos\": [0.125, 0.1, -0.26], \"def_pos\": [0.125, 0.1, -0.26], \"base_off\": [0.125, 0.05, -0.03], \"dir\": [-1, 1, 1], \"joint_lims\": [-0.245, 0.5236, -1.309, 1.309, -0.5236, -2.618]}], \"base\": {\"max_dip\": -0.35, \"leg_tar_h\": -0.26, \"t_rad\": 0.012, \"cw\": [0, 1, 3, 2]}}");*/

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

	/*(void)sock;*/
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

int main(int argc, char *argv[])
{
	char buf[READ_BUF_LEN];

	/*start_sim(buf, 0); // test*/

	// CONTROL TO SIM PIPE
	if (access(CS_PATH, F_OK)) {
		mkfifo(CS_PATH, 0666);
	}

	cs_fd = open(CS_PATH, O_RDWR);
	/*cs_fd = open(CS_PATH, O_WRONLY | O_NONBLOCK);*/
	if (cs_fd == -1) {
		FATAL(NLS, strerror(errno));
	}

	// SIM TO CONTROL PIPE
	if (access(SC_PATH, F_OK)) {
		mkfifo(SC_PATH, 0666);
	}

	sc_fd = open(SC_PATH, O_RDONLY);
	/*sc_fd = open(SC_PATH, O_RDONLY | O_NONBLOCK);*/
	if (sc_fd == -1) {
		FATAL(NLS, strerror(errno));
	}

	// VIEW TO CONTROL PIPE
	if (access(VC_PATH, F_OK)) {
		mkfifo(VC_PATH, 0666);
	}

	/*vc_fd = open(VC_PATH, O_RDONLY | O_NONBLOCK);*/
	/* TODO: implement SIGIO handling (inf O_ASYNC is used) <08-02-23, yourname> */
	vc_fd = open(VC_PATH, O_RDONLY | O_NONBLOCK | O_ASYNC);
	if (vc_fd == -1) {
		FATAL(NLS, strerror(errno));
	}

	if (argc == 2 && !strcmp(argv[1], "start_sim") && start_sim(buf)) {
		ERR("Failed to start simulationi\n");
	}

	interactive_loop(buf);

	close(cs_fd);
	close(sc_fd);
	close(vc_fd);

	exit(EXIT_SUCCESS);
}

