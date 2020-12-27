/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

//#include <px4_platform_common/px4_config.h>
//#include <px4_platform_common/tasks.h>
//#include <px4_platform_common/posix.h>
//#include <unistd.h>
//#include <stdio.h>
//#include <poll.h>
//#include <string.h>
//#include <math.h>

//#include <uORB/uORB.h>
//#include <uORB/topics/vehicle_acceleration.h>
//#include <uORB/topics/vehicle_attitude.h>

//__EXPORT int px4_simple_app_main(int argc, char *argv[]);

//int px4_simple_app_main(int argc, char *argv[])
//{
//	PX4_INFO("Hello Sky!");

//	/* subscribe to vehicle_acceleration topic */
//	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_acceleration));
//	/* limit the update rate to 5 Hz */
//	orb_set_interval(sensor_sub_fd, 200);

//	/* advertise attitude topic */
//	struct vehicle_attitude_s att;
//	memset(&att, 0, sizeof(att));
//	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

//	/* one could wait for multiple topics with this technique, just using one here */
//	px4_pollfd_struct_t fds[] = {
//		{ .fd = sensor_sub_fd,   .events = POLLIN },
//		/* there could be more file descriptors here, in the form like:
//		 * { .fd = other_sub_fd,   .events = POLLIN },
//		 */
//	};

//	int error_counter = 0;

//	for (int i = 0; i < 5; i++) {
//		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
//		int poll_ret = px4_poll(fds, 1, 1000);

//		/* handle the poll result */
//		if (poll_ret == 0) {
//			/* this means none of our providers is giving us data */
//			PX4_ERR("Got no data within a second");

//		} else if (poll_ret < 0) {
//			/* this is seriously bad - should be an emergency */
//			if (error_counter < 10 || error_counter % 50 == 0) {
//				/* use a counter to prevent flooding (and slowing us down) */
//				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
//			}

//			error_counter++;

//		} else {

//			if (fds[0].revents & POLLIN) {
//				/* obtained data for the first file descriptor */
//				struct vehicle_acceleration_s accel;
//				/* copy sensors raw data into local buffer */
//				orb_copy(ORB_ID(vehicle_acceleration), sensor_sub_fd, &accel);
//				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
//					 (double)accel.xyz[0],
//					 (double)accel.xyz[1],
//					 (double)accel.xyz[2]);

//				/* set att and publish this information for other apps
//				 the following does not have any meaning, it's just an example
//				*/
//				att.q[0] = accel.xyz[0];
//				att.q[1] = accel.xyz[1];
//				att.q[2] = accel.xyz[2];

//				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
//			}

//			/* there could be more file descriptors here, in the form like:
//			 * if (fds[1..n].revents & POLLIN) {}
//			 */
//		}
//	}

//	PX4_INFO("exiting");

//	return 0;
//}






/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>

// заголовочные файлы
// для потоков
#include <pthread.h>
#include <unistd.h> // для функции sleep
#include <mqueue.h>

// максимальное количество сообщений
#define MAX_MESSAGES 25
// емкость очереди
#define CAPACITY 5
#define QUEUE_PERMISSIONS 0660
#define QUEUE_NAME "Producer_Consumer_App"
int is_done = 0; // сигнал о завершении

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

void* produce(void *arg){
	char counter = 0;

	// 1. Создать разделяемую очередь
	// вот ссылка https://nuttx.apache.org/docs/latest/reference/user/04_message_queue.html#c.mq_open
	// как-то 4 аргументом надо установить максимальную длину очереди как 5
	// в ссылке [2] сделали так:
	struct mq_attr attr;

	attr.mq_flags = 0;
	attr.mq_maxmsg = CAPACITY;
	// вес одного сообщения, в нашем случае одно целое число
	attr.mq_msgsize = sizeof(char);
	attr.mq_curmsgs = 0;

	mqd_t Queue = mq_open(QUEUE_NAME, O_WRONLY | O_CREAT, QUEUE_PERMISSIONS, &attr );


	for (counter = 0 ; counter < MAX_MESSAGES; counter++)
	{
		sleep(1); // заснуть на секунду
		// https://nuttx.apache.org/docs/latest/reference/user/04_message_queue.html#c.mq_send
		mq_send(Queue, &counter, sizeof(char), 2);
		PX4_INFO("Sending %d\n", counter);

	}

	is_done = 1;

	// закрыть очередь
	// https://nuttx.apache.org/docs/latest/reference/user/04_message_queue.html#c.mq_close

	mq_close(Queue);

	return NULL;


}

void* consume(void *arg){

	char digit;
	unsigned int prior;

	struct mq_attr attr;

	attr.mq_flags = 0;
	attr.mq_maxmsg = CAPACITY;
	// вес одного сообщения, в нашем случае одно целое число
	attr.mq_msgsize = sizeof(char);
	attr.mq_curmsgs = 0;

	mqd_t Queue = mq_open(QUEUE_NAME, O_RDONLY | O_CREAT , QUEUE_PERMISSIONS, &attr );

	//mqd_t Queue = mq_open(QUEUE_NAME, O_RDONLY);

	while(!is_done ){
		mq_receive(Queue, &digit, sizeof(char), &prior);
		PX4_INFO("Receiving %d with priority %d\n", digit, prior);
		sleep(1);
	}

// получаем все оставшиеся сообщения
// https://nuttx.apache.org/docs/latest/reference/user/structures.html#c.timespec
	struct timespec time_to_wait;
	time_to_wait.tv_sec = 1;
	time_to_wait.tv_nsec = 0;

	// здесь используем функцию получения сообщения с ожиданием 2 секунды, чтобы выйти в случае пустой очереди
	while(mq_timedreceive(Queue, &digit, sizeof(char), &prior, &time_to_wait) != EAGAIN){
		PX4_INFO("Receiving %d with priority %d\n", digit, prior);
		sleep(1);
	}


	// закрыть очередь
	// https://nuttx.apache.org/docs/latest/reference/user/04_message_queue.html#c.mq_close
	mq_close(Queue);

	return NULL;

}

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Producer Consumer App");

	// Взяла за основу пример отсюда
	// [1] https://github.com/oflynned/Producer-Consumer-Queue/blob/master/ProducerConsumerQueue.c

// и вот отсюда
	// [2] https://www.softprayog.in/programming/interprocess-communication-using-posix-message-queues-in-linux


	// создание потоков task_create или pthread_create
	// судя по фразе
	// отсюда https://nuttx.apache.org/docs/latest/reference/user/01_task_control.html#c.task_create
	// - tasks are threads which have a degree of independence
	// - pthreads share some resources.

	// надо использовать pthreads, так как мы имеем разделяемый ресурс

	pthread_t consumer_thread;
	pthread_t producer_thread;

	// создать и запустить поток  производителя
	pthread_create(&producer_thread, NULL, produce, NULL);
	// создать и запустить поток потребителя
	pthread_create(&consumer_thread, NULL, consume, NULL);

	// ждать завершения
	pthread_join(producer_thread, NULL);
	pthread_join(consumer_thread, NULL);






	PX4_INFO("exiting app");

	return 0;
}
