.. _nxp_imx8x_net-samples:

NXP IMX8X Sample Application
###############################

Overview
********


Requirements
************

To run this sample is needed to set-up a host machine running GNU/Linux or Windows
with a network adapter connected to the target board ETH0 port through an Ethernet
cable.

Building and Running
********************



Setting up Host
***************

To be able to reach the board from the host, it's needed to configure the host
network interface IP's and default routes. This guide assumes the host IPv4 and
IPv6 addresses are `192.0.2.3` and `2001:db8::3`, respectively. For example,
using a network interface named `enp1s0` in a GNU/Linux host or `Ethernet` in
a Windows host, this can be done with the following commands:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

         ip -4 addr add 192.0.2.3/24 dev enp1s0
         ip -6 addr add 2001:db8::3/128 dev enp1s0
         route -A inet6 add default dev enp1s0

   .. group-tab:: Windows

      .. code-block:: console

         netsh interface ipv4 set address "Ethernet" static 192.0.2.3 255.255.255.0
         netsh interface ipv6 add address "Ethernet" 2001:db8::3/128
         netsh interface ipv6 add route ::/0 "Ethernet" ::

.. note::
   The above commands must be run as priviledged user.

If everything is configured correctly, you will be able to successfully execute
the following commands from the Zephyr shell:

.. code-block:: console

   net ping -I<iface> 192.0.2.3
   net ping -I<iface> 2001:db8::3

Where `<iface>` is the interface number starting from 1.
