
# GETTING STARTED GUIDE

## Freescale Kinetis / ARM Cortex M4 Project

Rob Laswick    
April 20 2012    
[www.laswick.net](http://www.laswick.net)


***
## GETTING STARTED

Read the [project page](http://www.laswick.net/kinetis.html).


***
## THE TOOLCHAIN

We'll be using the GNU tools, newlib, and openocd.

You can download a free pre-built toolchain from Mentor Graphics
[here](http://www.mentor.com/embedded-software/sourcery-tools/sourcery-codebench), or build your own toolchain from scratch, _which is much more fun :)_  In fact, I highly recommend building your own toolchain if you're up to it.  To help you get started checkout my [ARM Toolchain Project](http://www.laswick.net/arm-toolchain.html) for a complete set of scripts that will actually download, configure, patch, build, and install the entire toolchain for you _(binutils, gcc, newlib, gdb, and openocd)_!  These scripts are relatively simple and easy to modify should you feel the need to customize them.

#### WARNING

_One thing to note is that regardless which toolchain you choose to use, you will likely have to modify the makefiles in this project slightly to point to the correct tools and libraries.  Typically it's just a matter of modifying the PATH settings to match where you've installed your tools._

#### NOTE

Typically non-commercial tools only provide an assembler, compiler, and linker.  They do not provide any processor specific support files, such as `start code`, `linker script`(s), and `processor header files` -- BUT FEAR NOT ... _I've got you covered ;)_


***
## THE MENTOR GRAPHICS CODESOURCERY TOOLCHAIN

The CodeSourcery tools use a graphical installer, and installs smoothly.

I typically install all my tools in `~/opt` unless they need to be system wide, in which case I install them in `/opt`.

During the install, the tools will _attempt_ to modify your PATH environment variable automatically.  At the time of writing this the tools could only modify your PATH environment variable if your system uses a `.bash_profile` file, which is the case on Redhat based distros like Fedora.  If you're running a Debian based system, like Ubuntu or Mint, etc. your system uses a `.profile` file instead so you'll have to modify it manually.  Simply add the following:

        PATH=$HOME/opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI/bin:$PATH

_You'll need to run `. ~/.profile` in each shell for this update to take effect, or log out and back in again._


***
## THE OSBDM USB DRIVER

The USB to JTAG bridge on the tower board uses the OSBDM protocol, and as such you'll need to install an OSBDM USB driver.

The software and drivers in the tower kit are all garbage.  You'll need to download and install the latest driver from [P&E Micro](http://www.pemicro.com).

_I found the P&E Micro site very difficult to navigate and find what I was looking for.  To save you the near death experience of taking your own life I've posted the Linux driver [here](http://www.http://laswick.net/public/kinetis/drivers)_.

      
#### INSTALLING THE OSBDM USB DRIVER

Extract the driver:

        tar xvfz PemicroLinuxDrivers_2012_01_19.tar.gz

Enter the `libusb_64_32` directory and run the `setup` script (_as root_).  It should install painlessly and without errors (_honestly_).


***
## THE GDB SERVER

We'll be using [openocd](http://openocd.sourceforge.net) as our gdb server.

#### DOWNLOADING OPENOCD

The best way to get the latest and greatest version of openocd is to clone their `git` repo:

        git clone git://git.code.sf.net/p/openocd/code openocd.git

Enter the repo and run:

        ./bootstrap
        mkdir build
        cd build
        ../configure --prefix=/home/rlaswick/opt/openocd --enable-osbdm \
                    --enable-maintainer-mode
        make
        make install

#### NOTE

As long as your development host is configured correctly, and contains all the _required_ packages, you should be fine and everything will build without error.  However, this is rarely the case so you may need to install other libraries in order to complete the build successfully, depending on which Linux distro you're using (i.e. at the time of writing this I'm running Mint 13 (32 bit), and I needed install `libtool`, `libusb-1.0`, `automake`, and `texinfo` before openOCD would build properly).

#### A LITTLE TLC...

The kinetis tower includes a USB JTAG interface for programming and debugging. As mentioned above, the particular USB JTAG interface is the OSBDM.  Unfortunately support for OSBDM  is not completely issue free.  You'll need to add the following to the end of `~/opt/openocd/share/openocd/scripts/target/k60.cfg`:

        $_TARGETNAME configure -event gdb-attach {
            echo "Halting Target"
            reset init
        }


***
## DOWNLOAD OUR SOURCE CODE

        git clone git://github.com/laswick/kinetis.git kinetis.git


***
## VERIFY YOUR TOOLCHAIN

Enter our `kinetis.git` directory, then move into the `getting_started` directory. 

You should see 4 files:

        getting_started.md (this document)
        getting_started.s
        getting_started.ld
        getting_started.mak

sym-link the makefile as "makefile" for convenience:

        ln -s getting_started.mak makefile

Type `make`

`make` should complete without errors and produce `out.axf` and `out.s`.

      out.axf is the executable image you'll use to program and debug.
      out.s is the disassembly listing.

Connect the tower board to your computer via the supplied USB cable.

Type `make openocd` to launch openocd.

In a separate shell (in the same directory) type `make gdb` to launch gdb _(which should automatically connect to openocd)_.

From the (gdb) prompt:

Type `load` to program the tower with your executable image.

Type `layout split` to view the source and assembly listings.

Type `tb first_break` to set a temporary breakpoint in the source.

Type `c` to allow the software to run.  gdb will halt at `first_break`.

From here use the step instruction `si` to step through the rest of the program.


#### A NOTE ABOUT USING gdb's load COMMAND

On some development hosts running `(gdb) load` fails to program the image into the flash memory on the target.  One way to work around this is to override and redefine the `load` command in your `.gdbinit` script.

It's always a good idea to have a `.gdbinit` script when using gdb.  Create a file named `.gdbinit` in the directory you're working in, with the following contents:

        target remote localhost:3333
        b _default_fault_handler
        b assert_
        define reset
        monitor reset init
        end

If you want to override the `load` command, add the following to the end of your `.gdbinit` script:

        define load
        monitor flash write_image erase "out.axf" 0 elf
        reset
        end


***
## NOW WHAT?

Read the `kinetis.git/README.md` file to understand more about this project and how it may be of use to you.

Enjoy, and happy hacking ;)

-- Rob --





