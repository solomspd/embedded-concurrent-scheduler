#include "../../sched_core.h"

extern void wrap_around(int *x,int wrap_val);
extern struct task* queue_pop(struct queue* que);
extern void queue_push_back(struct queue* que, struct task* new_task);
extern struct queue delay_que, rdy_que;
extern void QueTask(void (*func_in)(void), uint8_t priority_in);
extern void set_temp_thresh(void);
extern struct task* get_head(struct queue *que);