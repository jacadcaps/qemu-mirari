Mirari board (``mirari``)
=========================

The ``mirari`` machine models the Mirari board, an NXP QorIQ T1042 system
on chip with four e5500 cores.  With ``-cpu e6500`` (or as ``-M mirari2``)
it is the same board around the T2081, the four-core e6500 sibling, which
adds AltiVec and the memory-mapped cluster L2.

Supported devices
-----------------

* PowerPC e5500 or e6500 core (up to four)
* CoreNet coherency manager: CCSRBAR, boot space translation and the local
  access windows
* DDR controller registers, enough for the guest to size RAM
* Freescale MPIC 4.2 interrupt controller
* Device configuration (DCFG) unit, including the boot release register
* Clocking and run control / power management (RCPM) registers
* Four MPC I2C controllers, with a 24AA256UID EEPROM carrying the board
  EUI, a DS1339 RTC and an ADT7461 temperature sensor on the first bus
* Two ns16550 DUART serial ports
* Two QorIQ SATA controllers, one port each
* Four PCI Express controllers; a PCI-to-PCI bridge root complex on each,
  with a device slot behind the first one
* Board CPLD: revision, slot occupancy, power and reset control

Running
-------

The machine has no on-board firmware.  The board boot image is loaded as a
32-bit big endian PowerPC ELF, the way u-boot's ``bootelf`` command loads
it, and the machine sets up the register and TLB state u-boot leaves
behind::

  qemu-system-ppc64 -M mirari -m 1G \
      -kernel boot.img \
      -cdrom install.iso

The image is taken from ``-kernel``, from ``-bios``, or from
``bootloadermirarirom.img`` on the firmware search path, in that order.
``-append`` passes a command line through the Hyperbootloader tag list.

RAM is a multiple of 16 MB, up to 4 GB.  The DDR local access windows are
sized the way u-boot sizes them -- a power of two each, aligned to its own
size and no larger than 2 GB, so 3 GB takes two windows -- and there are
five of them, which rejects a handful of awkward sizes such as ``1520M``.

An SM501 display, an OHCI controller with a USB keyboard and mouse, and an
RTL8139 network card are created by default; ``-nodefaults``, ``-vga
none``, ``-usb off`` and ``-nic none`` opt out of each.  ``-cdrom``,
``-hda``, ``-hdb`` and ``-drive if=ide`` are attached to the two SoC SATA
ports in index order.
