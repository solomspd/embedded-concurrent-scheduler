# A lightweight scheduler for embedded applications

This library sets out to create simple cooperative task scheduler for embedded C applications for STM32L4 devices. 

## Integrating into your project

To build the project, use your preferred pinout and configurations and the resulting project files from CubeMX.

Simply include the `sched.h` file in main and the `itter.h` file in the interrupts source file.

Make sure to define the number of tasks expected to be run by a compiler directive; for example this here sets the max number of tasks as 32: `#define MAX_N_TASKS 32`. Make sure to add this before including the header files not after.

Add tasks to the scheduler to be run once via `QueTask(function, priority)` and run periodically by adding a `ReRunMe(delay_in_ticks)` at the end of this function.

Have the main loop as simply an infinite while look with a single `Dispatch()` in it.

Priorities are made so that priority 1 is the hight priority and priority 8 is the lowest priority.

## Scheduler architecture

### Priority queue and core

The scheduler works by relying on a priority queue. Tasks are ordered in it according to their priority, having the hight priority at the head. 

The main function simply continuously dequeues the priority queue. You add tasks to the priority queue through the `QueTask` API and you run tasks after a period by adding `ReRunMe` API to the end of a queued function.

As it is configured, each tick is 50ms long.

### Rerunning a task

When a Task finishes execution and is not designated to be reran, then it will simply leave the queue and its memory freed. If it is not meant to be rerun, then it will be moved to another queue that stores tasks that are meant to be rerun with their priority modified to be the number of ticks the task should wait before needing to be run again.
Each tick, the priority (ticks to wait to run) are decremented by 1.
When a tasks priority reaches zero, it is popped from this queue and places back in the first queue that holds tasks that are meant to execute.

## Queue implementation

The queue is a simple 1D array of task pointers.
It behaves as a circular queue by keeping track where its head and tail are and "wrapping around" the array end and beginning.

It does not use an array of tasks since it is a common use case for tasks to be very frequently passed from one queue to another so simply the pointer is passed. In other words only a 2 bytes are passed around instead of a 10 byte struct.

Tasks are created dynamically through `malloc`. When a task is passed to another queue, a counter presenting the number of active references is incremented but when it is dequeued the reference counter is decremented. When the reference counter is 0, the task allocated memory is freed.

## API

### `void QueTask(void (*func_in)(void), uint8_t priority_in)`

- `func_in`: Function pointer of task to run.
- `priority_in`: Priority of new queue entry.

This method queues a task to the queue that is sorts tasks according to their priorities. Also allocated the memory of a new task and actually creates it.

### `void ReRunMe(unsigned int delay_in)`

- `delay_in`: Delay in ticks to wait before the task is run again.

This method takes the currently running task and adds it to the queue that sorts tasks inversely proportionally to the number of ticks they have to wait.
It pops it from one queue and pushes it back into the other.

### `void Dispatch(void)`

This method does not take any arguments, it simply dequeues the top of the queue with the tasks ready to run and runs it.

### `void wrap_around(int *x,int wrap_val);`

Used to treat the queue as a continous queue. Wraps around the values so that index of `-1` would be the end of the que and index of `wrap_val` would be the start of the array.

### `struct task* get_tail(struct queue *que);`

Read the last task in the queue.

### `struct task* get_head(struct queue *que);`

Read the first task in the queue.

### `struct task* que_pop(struct queue *que);`

Remove the first task in the queue and return its pointer.

### `struct task* que_pop_back(struct queue *que);`

Remove the last task in the queue and return its pointer.

### `void swap_task(struct task **a, struct task **b);`

Swap two task pointers. Critical for inserting a task into the queue.

### `void queue_push_back(struct queue *que, struct task *new_task);`

Add the task pointer to the queue pointed at by `que`.

### `uint8_t eq_tasks(struct task *a, struct task *b);`

Compare 2 tasks. If all elements are equivalent, then it returns a bytes of all 1 otherwise, it returns a bytes of all 0.

### `void init_que(struct queue *que);`

Initialize the queue pointed to by que to default values.

# Sample applications

Here a few sample application that demonstrate the power and capabilities of this scheduler.

## Temperature alarm

This application triggers an alarm in the form of flickering an LED when the temperature from a sensor crosses a certain threshold. It reads an input from the user over UART to set the temperature threshold.

The input is expected in the form of `xx.xx\n` where `x` can be any number. The decimal places have to be 2 numbers while the integral part of the number can either be 1 or 2 numbers. This decimal number must be followed by a new line.

This application is comprised of 3 tasks.

- `read_temp()`: Reads the temperature over I2C from a DS3231 temperature sensor.

- `trig_alarm()`: Checks if a global variable containing the read temperature crosses a threshold.

- `set_temp_thresh()`: Reads a single character from the UART connection, adding it to a buffer. When a new line character is received, it parses the string and produces a new threshold.

A UART interrupt is set up to trigger when a new character is received. All this interrupt does `QueTask(set_temp_thresh,1)` to trigger the `set_temp_thresh` task and process the new character.

## Parking sensor

This application we read a distance from an ultrasonic sensor and according to this distance, we inversely proportionally increase the rate of beeping of a buzzer.

- `read_dist()`: Read the distance from an HC-SR04 ultrasonic sensor and scale it.

- `beep()`: Trigger a 1ms beep and rerun according to the scaled distance from the ultrasonic sensor. We use the scheduler to set up the beep period.
