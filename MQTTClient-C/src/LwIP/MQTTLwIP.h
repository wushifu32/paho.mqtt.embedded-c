/*******************************************************************************
 * Copyright (c) 2014 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Allan Stockdill-Mander - initial API and implementation and/or initial documentation
 *******************************************************************************/

#if !defined(__MQTT_LWIP_)
#define __MQTT_LWIP_

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "lwip/tcp.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include <string.h>

#define MQTT_TASK 1

typedef struct Timer
{
    TickType_t xTicksToWait;
	TimeOut_t xTimeOut;
} Timer;

typedef struct Mutex
{
	SemaphoreHandle_t sem;
} Mutex;

typedef struct Thread
{
	TaskHandle_t task;
} Thread;

typedef struct Network
{
	int my_socket;
	int (*mqttread) (struct Network*, unsigned char*, int, int);
	int (*mqttwrite) (struct Network*, unsigned char*, int, int);
} Network;

void MutexInit(Mutex*);
int MutexLock(Mutex*);
int MutexUnlock(Mutex*);
int ThreadStart(Thread*, void (*fn)(void*), void* arg);

void TimerInit(Timer*);
char TimerIsExpired(Timer*);
void TimerCountdownMS(Timer*, unsigned int);
void TimerCountdown(Timer*, unsigned int);
int TimerLeftMS(Timer*);

int mqtt_lwip_read(Network*, unsigned char*, int, int);
int mqtt_lwip_write(Network*, unsigned char*, int, int);

void NetworkInit(Network*);
int NetworkConnect(Network*, char*, int);
void NetworkDisconnect(Network*);

#endif
