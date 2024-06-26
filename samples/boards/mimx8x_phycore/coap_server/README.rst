.. zephyr:code-sample:: coap-server
   :name: CoAP service
   :relevant-api: coap coap_service udp

   Use the CoAP server subsystem to register CoAP resources.

Overview
********

This sample shows how to register CoAP resources to a main CoAP service.
The CoAP server implementation expects all services and resources to be
available at compile time, as they are put into dedicated sections.

The resource is placed into the correct linker section based on the owning
service's name. A linker file is required, see ``sections-ram.ld`` for an example.

This demo assumes that the platform of choice has networking support,
some adjustments to the configuration may be needed.

The sample will listen for requests in the CoAP UDP port (5683) in the
site-local IPv6 multicast address reserved for CoAP nodes.

Currently, only IPv4 is supported.

The sample exports the following resources:

.. code-block:: none

   /example_data


These resources allow a good part of the ETSI test cases to be run
against coap-server.

Building And Running
********************

This project will print all the octets of the response received, more context can
be obtained by using a tool such as tcpdump or wireshark.

Use following command for building:
  west build -p always -b mimx8x_phycore/mimx8qx6/a35 samples/boards/mimx8x_phycore/coap_server \
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

Then, you can run following command on Host machine, to get example data.

.. code-block:: console

   sudo ./coap-client -m get coap://192.0.2.1/example_data

or, adding query to get long message.

.. code-block:: console

   sudo ./coap-client -m get coap://192.0.2.1/example_data?m=long

Sample output
=============

.. code-block:: console

   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: type: 0 code 1 id 62928
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: type: 0 code 1 id 62929
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: type: 0 code 1 id 46044
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: type: 0 code 1 id 46045
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: type: 0 code 1 id 46046
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: type: 0 code 1 id 46047
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: type: 0 code 1 id 46048
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: type: 0 code 1 id 46049
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: *******
   <inf> net_coap_service_sample: type: 0 code 1 id 46050
   <inf> net_coap_service_sample: *******

.. _`libcoap`: https://github.com/obgm/libcoap
