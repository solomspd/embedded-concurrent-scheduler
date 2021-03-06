# A lightweight scheduler for embedded applications

This library sets out to create simple cooperative task scheduler for embedded C applications for STM32L4 devices. 

## Running this repo

To run this repository to develop or run any of the sample applications, simply clone it and open it in keil. Then you can uncomment the compiler flags in the beginning of the `main.c` to print debug values over UART2.

To select any of the sample applications to run, simply assign `TASK_SET` to the appropriate value (either `DEMO1` or `DEMO2` or `INTERNAL`). Internal is the simplest task set that involves toggling an LED and writing to UART2.

No additional configuration required through CubeMx since no pin designations interfere between sample applications.

## Integrating into your project

Simply use the CubeMx project as a template and configure it to the desired pinouts. The sechuler is all in the `USER CODE 0` section.

Make sure to define the number of tasks expected to be run by a compiler directive; for example this here sets the max number of tasks as 32: `#define MAX_N_TASKS 32`. Make sure to add this before including the header file `scheduler.h` not after.

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

It does not use an array of tasks since it is a common use case for tasks to be very frequently passed from one queue to another so simply the pointer is passed. In other words only a 4 bytes are passed around instead of a 12 byte struct.

Tasks are created dynamically through `malloc`. When a task is passed to another queue, a counter presenting the number of active references is incremented but when it is dequeued the reference counter is decremented. When the reference counter is 0, the task allocated memory is freed.

## User facing API

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

## Queue internal API

### `void wrap_around(int *x,int wrap_val);`

Used to treat the queue as a continous queue. Wraps around the values so that index of `-1` would be the end of the que and index of `wrap_val` would be the start of the array.

### `struct task* get_tail(struct queue *que);`

Read the last task in the queue.

- `que`: Pointer to queue to get tail from.

### `struct task* get_head(struct queue *que);`

- `que`: Pointer to queue to get head from.

Read the first task in the queue.

### `struct task* que_pop(struct queue *que);`

- `que`: Pointer to queue to pop the head from.

Remove the first task in the queue and return its pointer.

### `struct task* que_pop_back(struct queue *que);`

- `que`: Pointer to queue to pop the tail from.

Remove the last task in the queue and return its pointer.

### `void swap_task(struct task **a, struct task **b);`

- `a`: One of the two task pointer reference to swap.
- `b`: The other task pointer reference to swap.

Swap two task pointers. Critical for inserting a task into the queue.

### `void queue_push_back(struct queue *que, struct task *new_task);`

- `que`: Pointer to que to push back the task back to.
- `new_task`: Pointer to task to add to que.

Add the task pointer to the queue pointed at by `que`.

### `uint8_t eq_tasks(struct task *a, struct task *b);`

- `a`: Pointer to one of the two tasks to compare.
- `b`: Pointer to the other task to compare.

Compare 2 tasks. If all elements are equivalent, then it returns a bytes of all 1 otherwise, it returns a bytes of all 0.

### `void init_que(struct queue *que);`

- `que`: Pointer to que to initalize.

Initialize the queue pointed to by que to default values.

# Sample applications

Here a few sample application that demonstrate the power and capabilities of this scheduler.

Note: Pinouts mentioned are according to STM32L4 internal pinout names not the arduino convention.

## Temperature alarm

This application triggers an alarm in the form of flickering an LED when the temperature from a sensor crosses a certain threshold. It reads an input from the user over UART to set the temperature threshold.

The input is expected in the form of `xx.xx\n` where `x` can be any number. The decimal places have to be 2 numbers while the integral part of the number can either be 1 or 2 numbers. This decimal number must be followed by a new line.

This application is comprised of 3 tasks.

- `read_temp()`: Reads the temperature over I2C from a DS3231 temperature sensor.

- `trig_alarm()`: Checks if a global variable containing the read temperature crosses a threshold.

- `set_temp_thresh()`: Reads a single character from the UART connection, adding it to a buffer. When a new line character is received, it parses the string and produces a new threshold.

A UART interrupt is set up to trigger when a new character is received. To keep the intterupt code to a minimum, all this interrupt does `QueTask(set_temp_thresh,1)` to trigger the `set_temp_thresh` task and process the new character.

### Connections

- Connect the temperature sensor VCC to 3v3 and ground to ground.
- Connect sensor SCL to A9 and sensor SDA to A10.

[![Demo1](https://yt-embed.herokuapp.com/embed?v=0WEf4JgkcQE)](https://youtu.be/0WEf4JgkcQE)

## Parking sensor

This application we read a distance from an ultrasonic sensor and according to this distance, we inversely proportionally increase the rate of beeping of a buzzer.

- `read_dist()`: Request and read the distance from an HC-SR04 ultrasonic sensor by polling it and scale the result.

- `beep()`: Trigger a 1ms beep and rerun according to the scaled distance from the ultrasonic sensor. We use the scheduler to set up the beep period.

### Connections

- Connect the ultrasonic sensor VCC to 5v and ground to ground.
- Connect sensor trig to A0 and echo to A1
- Connect the buzzer ground to ground and VCC to B3.

Since STM32L4 seems to have issues with interrupts over UART over UART1, we use UART2 with a USB to TTL module.

- Connect USB to TTL Rx to STM32 Tx and Tx to STM32 Rx. And ground to ground.

[![Demo2](https://yt-embed.herokuapp.com/embed?v=0X_GfUMAeQ0)](https://youtu.be/0X_GfUMAeQ0)
