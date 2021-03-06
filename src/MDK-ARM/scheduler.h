#define MAX_N_TASKS 32
#include "stdint.h"

struct task {
	uint16_t prio, ref_prio;
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
