Examples to be put in the contiki/examples/stm32nucleo-spirit1 folder.

* 'sensor-demo' is the same example already included in the stm32nucleo-spirit1 branch of the Contiki repository.
* 'sensor-er-rest-example' is a modified version of the er-rest-example, so it implements a CoAP server that exposes the sensors of the STM32 Nucleo expansion board.
* 'sensor-udp-rpl' is a modified version of the ipv6/simple-udp-rpl example of Contiki, so it implements the sending of UDP messages carrying  sensor data, from a UDP client to a UDP server.
* 'sniffer' is a simple utility that can be used to sniff on-air packets and analyze them with wireshark.
* 'binaries' folder contains some pre-compiled firmware images for these examples and for the rpl-border-router.

