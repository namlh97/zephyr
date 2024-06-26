.. zephyr:code-sample:: coap-client
   :name: CoAP client
   :relevant-api: coap udp

   Use the CoAP library to implement a client that fetches a resource on iMX8X-Phycore board.

Overview
********

This sample is a simple CoAP client showing how to retrieve information
from a resource using iMX8X-Phycore board.

This demo assumes that the platform of choice has networking support,
some adjustments to the configuration may be needed.

Currently, only IPv4 is supported.

This sample will use the libcoap for testing. 

Building and Running
********************

This project will print all the octets of the response received, more context can
be obtained by using a tool such as tcpdump or wireshark.

Use following command for building:
  west build -p always -b mimx8x_phycore/mimx8qx6/a35 samples/boards/mimx8x_phycore/coap_client \
  -DDTC_OVERLAY_FILE="nxp-enet-experimental.overlay"

Setting up Host
***************

On Host machine, configure the host network interface IP's and default routes assuming are 
  `192.0.2.2` and `255.255.255.0` respectively. 
.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

        ip -4 addr add 192.0.2.2/24 dev enp1s0

  .. group-tab:: Windows

    .. code-block:: console

        netsh interface ipv4 set address "Ethernet" static 192.0.2.2 255.255.255.0

.. note::
   The above commands must be run as priviledged user.

Then, you will need to set-up COAP-server on host manchine.
  cd libcoap/build
  sudo ./coap-server -A 192.0.2.2 -v 7

If everything is configured correctly, you will be able to successfully execute
the following commands from the Zephyr shell:

.. code-block:: console

   coap_client -mget coap://192.0.2.2/example_data

Sample output
=============

.. code-block:: console

  <inf> net_coap_client_sample: CoAP response callback,                                                              
  <inf> net_coap_client_sample: Received data with length: 1024
  <inf> net_coap_client_sample: CoAP response callback,                                                              
  <inf> net_coap_client_sample: Received data with length: 476

.. _`libcoap`: https://github.com/obgm/libcoap
