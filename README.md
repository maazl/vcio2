# Kernel driver to access Rapsperry Pi's VideoCore IV GPU from user space

When using Raspberry Pi's GPU from user space (e.g. hello_fft) with the built-in vcio driver in the RPi kernel
you need to have _root privileges_ for every application.
The goal of the vcio2 driver is to overcome this restriction.
Furthermore vcio2 keeps track of the resources and performs some clean-up if the application is interrupted.

<a href="http://www.maazl.de/project/vcio2/doc/index.html">&rarr; Homepage & Download</a>

<a href="http://www.maazl.de/project/vcio2/doc/index.html#features">&rarr; Features and limitations</a>

<a href="http://www.maazl.de/project/vcio2/doc/index.html#features">&rarr; Installation instructions</a>

<a href="sample">&rarr; Sample code</a>

<a href="http://www.maazl.de/project/vcio2/doc/APIref.html">&rarr; API reference</a>

<a href="http://www.maazl.de/project/vcio2/doc/guide.html">&rarr; Programming and migration guide</a>
