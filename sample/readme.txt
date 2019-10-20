The folder contains 2 versions of the same sample program.
It executes all QPU operators against a bunch of constants.

* porting
  This version is shows how existing programs that use the mailbox.c interface
  of /opt/vc/src/hello_pi/hello_fft for QPU access.

  Replacing mailbox.c and mailbox.h by the files in the proting subfolder
  will switch the application to vcio2.
  There is one incompatible change in the API:
  mapmem requires an additional parameter file_desc from mbox_open like all
  other functions too.

* hybrid
  This version cann use the vcio2 driver as well as the old vcio driver
  if the first is not available. But you need to be root to use the vcio
  fallback.
  Replacing mailbox.c and mailbox.h by the files in the hybrid subfolder
  will switch the application to vcio2 with vcio fallback..
  It has the same incompatible API change than the porting version.
