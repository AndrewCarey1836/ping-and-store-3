
Ping some towers and store the info
###################################

Overview
********

This program pings LTE towers and stores the ouput to a .txt file in an attached SD
card.

Requirements
************

This project requires SD card support and microSD card formatted with FAT filesystem.
See the :ref:`disk_access_api` documentation for Zephyr implementation details.

Also requires LTE connection.

Building and Running
********************

This sample can be built for the nrf9160dk_nrf9160_ns

To run this, a FAT formatted microSD card should be present in the
microSD slot. If there are any files or directories present in the card, the
sample lists them out on the debug serial output.
