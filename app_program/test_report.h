/*
 *	test_report.h
 *	Minimal on-target test helpers.
 *
 *	Every check prints ONE line in a fixed format, so a saved console log
 *	can be searched for results ("grep '\[TEST\]' log.txt"):
 *
 *	    [TEST] S1-U01 PASS  ADC reads in range
 *	    [TEST] S1-U02 FAIL  queue rejects overflow   (task_x.c:57)
 *	    [TEST] subsystem1 SUMMARY 1 passed, 1 failed
 *
 *	Usage, inside a test function:
 *	    TEST_INIT();
 *	    TEST_CHECK("S1-U01", value >= 0 && value <= 4095, "ADC reads in range");
 *	    TEST_SUMMARY("subsystem1");
 *
 *	Test IDs must match the ones in testing/<subsystem>/TEST_LOG.md.
 */

#ifndef TEST_REPORT_H
#define TEST_REPORT_H

#include <tm/tmonitor.h>

#define TEST_INIT()	INT test_pass_ = 0, test_fail_ = 0

#define TEST_CHECK(id, cond, desc)					\
	do {								\
		if (cond) {						\
			test_pass_++;					\
			tm_printf((UB*)"[TEST] %s PASS  %s\n", (id), (desc)); \
		} else {						\
			test_fail_++;					\
			tm_printf((UB*)"[TEST] %s FAIL  %s   (%s:%d)\n",	\
				  (id), (desc), __FILE__, __LINE__);	\
		}							\
	} while (0)

#define TEST_SUMMARY(name)						\
	tm_printf((UB*)"[TEST] %s SUMMARY %d passed, %d failed\n",	\
		  (name), test_pass_, test_fail_)

#endif /* TEST_REPORT_H */
