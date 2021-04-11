#include "stdint.h"

#define MAX_N_TASKS 32

struct task {
	uint8_t prio, ref_prio;
	void (*func)(void);
	int ref_count;
};

struct queue {
	struct task* que[MAX_N_TASKS];
	int head;
	int tail;
	int max;
	int len;
};