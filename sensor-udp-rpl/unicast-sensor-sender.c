/*
 * Copyright (c) 2011, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**********************************************************************
 *                      Update: September 2015 
 *          This file has been modified by STclab team to use
 *                  stm32nucleo-spirit1 platform
 **********************************************************************/

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-debug.h"

#include "sys/node-id.h"

#include "simple-udp.h"
#include "servreg-hack.h"

#include <stdio.h>
#include <string.h>

#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "dev/radio-sensor.h"

#ifdef COMPILE_SENSORS
#include "dev/sensor-common.h"
#include "dev/temperature-sensor.h"
#include "dev/humidity-sensor.h"
#include "dev/pressure-sensor.h"
#include "dev/magneto-sensor.h"
#include "dev/acceleration-sensor.h"
#include "dev/gyroscope-sensor.h"
#endif /*COMPILE_SENSORS*/

#define UDP_PORT 1234
#define SERVICE_ID 199

#define SEND_INTERVAL		(5 * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))

static struct simple_udp_connection unicast_connection;

/*---------------------------------------------------------------------------*/
PROCESS(unicast_sender_process, "Unicast sensor sender example process");
AUTOSTART_PROCESSES(&unicast_sender_process);
/*---------------------------------------------------------------------------*/
static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  printf("Data received on port %d from port %d with length %d\n",
         receiver_port, sender_port, datalen);
}
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;
  int i;
  uint8_t state;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  printf("IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      uip_debug_ipaddr_print(&uip_ds6_if.addr_list[i].ipaddr);
      printf("\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(unicast_sender_process, ev, data)
{
  static struct etimer periodic_timer;
  static struct etimer send_timer;
  uip_ipaddr_t *addr;
  static int  sensor_value = 0;

  PROCESS_BEGIN();

  SENSORS_ACTIVATE(button_sensor);
  SENSORS_ACTIVATE(radio_sensor);
  
#ifdef COMPILE_SENSORS
  SENSORS_ACTIVATE(temperature_sensor);
  SENSORS_ACTIVATE(humidity_sensor);
  SENSORS_ACTIVATE(pressure_sensor);
  SENSORS_ACTIVATE(magneto_sensor);
  SENSORS_ACTIVATE(acceleration_sensor);
  SENSORS_ACTIVATE(gyroscope_sensor);
#endif /*COMPILE_SENSORS*/

  servreg_hack_init();

  set_global_address();

  simple_udp_register(&unicast_connection, UDP_PORT,
                      NULL, UDP_PORT, receiver);

  etimer_set(&periodic_timer, SEND_INTERVAL);
  while(1) 
  {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
    etimer_reset(&periodic_timer);
    etimer_set(&send_timer, SEND_TIME);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_timer));
    addr = servreg_hack_lookup(SERVICE_ID);
    if(addr != NULL) {
      static unsigned int message_number = 0;
      char buf[40];

      printf("Sending unicast to ");
      uip_debug_ipaddr_print(addr);
      printf("\n");
	  
#ifdef COMPILE_SENSORS
      switch(message_number) {
	case 0:
          sensor_value = temperature_sensor.value(0);
	  sprintf(buf, "Temperature:\t%d.%d C", sensor_value/10, ABS_VALUE(sensor_value)%10);
	  break;
	case 1:
	  sprintf(buf, "Humidity:\t%d.%d rH", humidity_sensor.value(0)/10, humidity_sensor.value(0)%10);
	  break;
	case 2:
	  sprintf(buf, "Pressure:\t%d.%d mbar", pressure_sensor.value(0)/10, pressure_sensor.value(0)%10);
	  break;
	case 3:
	  sprintf(buf, "Magneto:\t\t%d/%d/%d (X/Y/Z) mgauss", magneto_sensor.value(X_AXIS),
                                                              magneto_sensor.value(Y_AXIS),
                                                              magneto_sensor.value(Z_AXIS));
	  break;
	case 4:
	  sprintf(buf, "Acceleration:\t%d/%d/%d (X/Y/Z) mg", acceleration_sensor.value(X_AXIS),
                                                               acceleration_sensor.value(Y_AXIS),
                                                               acceleration_sensor.value(Z_AXIS));
	  break;
	case 5:
	  sprintf(buf, "Gyroscope:\t%d/%d/%d (X/Y/Z) mdps", gyroscope_sensor.value(X_AXIS),
                                                              gyroscope_sensor.value(Y_AXIS),
                                                              gyroscope_sensor.value(Z_AXIS));
	  break;
	case 6:
          sensor_value = radio_sensor.value(RADIO_SENSOR_LAST_PACKET);
          sprintf(buf, "Radio (RSSI):\t%d.%d dBm", sensor_value/10, ABS_VALUE(sensor_value)%10);
	  break;
	case 7:
          sprintf(buf, "Radio (LQI):\t%d", radio_sensor.value(RADIO_SENSOR_LAST_VALUE));
	  break;
	default:
	  break;
     }
     simple_udp_sendto(&unicast_connection, buf, strlen(buf) + 1, addr);

     message_number++;
     if(message_number > 7) {
       message_number = 0;
     }
#endif /*COMPILE_SENSORS*/
    } else {
      printf("Service %d not found\n", SERVICE_ID);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
