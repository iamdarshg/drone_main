#include "hw_sim.h"
#include "logging.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#if !defined(_WIN32)
# include <sys/wait.h>
#endif

/*
 * hw_sim.c
 *
 * VM-driven hardware self-test runner.
 * When available, this runner will invoke an external ARM user-mode emulator
 * (for example `qemu-arm`) to execute a compiled ARM ELF containing the
 * firmware/tests. The emulator stdout/stderr is captured and parsed for
 * standardized test markers (e.g. "TEST: PASS" / "TEST: FAIL").
 *
 * This allows exercising the compiled ARM code in a host-side environment
 * without flashing hardware.
 *
 * Notes / assumptions:
 * - The test ELF should print lines with the prefix "TEST: " followed by
 *   a human-readable message and PASS/FAIL status for each test case.
 * - The host system must have an ARM user-mode emulator in PATH (qemu-arm
 *   or equivalent) or the path can be provided via hw_sim_set_emulator_path().
 * - If no emulator is available, the runner will log an error and return
 *   a non-zero result. This avoids creating stubs or silent success paths.
 */

static int imu_ok = 1, rf_ok = 1, mem_ok = 1;
static char emulator_path[512] = "qemu-arm"; /* default executable name */
static int emulator_timeout_sec = 30;

void hw_sim_init(void)
{
	log_info("HW sim init: VM-driven self-test runner initialized");
}

void hw_sim_set_imu_response(int ok) { imu_ok = ok; }
void hw_sim_set_rf_response(int ok) { rf_ok = ok; }
void hw_sim_set_memory_response(int ok) { mem_ok = ok; }

void hw_sim_set_emulator_path(const char *path)
{
	if (!path) return;
	strncpy(emulator_path, path, sizeof(emulator_path) - 1);
	emulator_path[sizeof(emulator_path) - 1] = '\0';
	log_info("HW sim: emulator path set to %s", emulator_path);
}

void hw_sim_set_timeout(int seconds)
{
	if (seconds <= 0) return;
	emulator_timeout_sec = seconds;
	log_info("HW sim: emulator timeout set to %d seconds", emulator_timeout_sec);
}

/*
 * Run a compiled ARM ELF under the configured emulator and capture output.
 * elf_path: path to the ELF or binary image built for ARM user-mode.
 * Returns 0 on success (all tests passed), non-zero on failure or error.
 */
int hw_sim_run_vm_tests(const char *elf_path)
{
	if (!elf_path) {
		log_error("HW sim: elf_path is NULL");
		return -1;
	}

	char cmd[1024];
	/* Use -E to keep environment minimal on some qemu builds if needed; leave flexible */
	snprintf(cmd, sizeof(cmd), "\"%s\" \"%s\"", emulator_path, elf_path);

	log_info("HW sim: running VM command: %s", cmd);

	/* Open a pipe to capture emulator output. popen returns a FILE*; on Windows
	 * this uses _popen internally in many toolchains. We keep this portable by
	 * using the stdlib function. */
	FILE *fp = popen(cmd, "r");
	if (!fp) {
		log_error("HW sim: failed to start emulator '%s'", emulator_path);
		return -2;
	}

	char line[1024];
	int tests_run = 0;
	int tests_passed = 0;
	time_t start = time(NULL);

	while (fgets(line, sizeof(line), fp) != NULL) {
		/* Simple echo of VM output into logs so the build/test harness keeps full trace */
		/* Trim trailing newline for logging API */
		size_t len = strlen(line);
		if (len && line[len - 1] == '\n') line[len - 1] = '\0';
		log_info("VM: %s", line);

		/* Parse test markers */
		if (strncmp(line, "TEST:", 5) == 0) {
			tests_run++;
			/* Accept either PASS or FAIL after the marker */
			if (strstr(line, "PASS") != NULL) {
				tests_passed++;
			}
		}

		/* Respect timeout */
		if (difftime(time(NULL), start) > emulator_timeout_sec) {
			log_error("HW sim: emulator timed out after %d seconds", emulator_timeout_sec);
			break;
		}
	}

	int rc = pclose(fp);
	if (rc == -1) {
		log_error("HW sim: error waiting for emulator process");
		return -3;
	}

	/* pclose return semantics vary between platforms: on POSIX it returns a
	 * wait status (needs WIFEXITED/WEXITSTATUS), on Windows it typically
	 * returns the process exit code directly. Normalize for logging. */
#if defined(_WIN32)
	int exitcode = rc;
#else
	int exitcode = -1;
	if (WIFEXITED(rc)) {
		exitcode = WEXITSTATUS(rc);
	}
#endif
	log_info("HW sim: emulator exited with code %d", exitcode);

	log_info("HW sim: tests run=%d passed=%d", tests_run, tests_passed);

	if (tests_run == 0) {
		log_warn("HW sim: no tests were detected in VM output; ensure the ELF prints lines starting with 'TEST:'");
		return -4;
	}

	if (tests_run == tests_passed) {
		log_info("HW sim: all VM tests passed");
		return 0;
	}

	log_error("HW sim: some VM tests failed (%d/%d)", tests_passed, tests_run);
	return 1;
}

/* Note: the functions below are kept for compatibility with existing test hooks
 * in the codebase. They allow parts of the firmware to be conditionally exercised
 * by host-side test runners which may query these values. */

