# Pixhawk firmware setup

We (KTH RPL) use an old version of the px4-firmware with useful additions specific for us.
These are the instructions for building this px4 version on Ubuntu 20.04.

**Verify these build install instructions and make any necessary changes to make it easier for building in the future**

## Setup QGround Control

* [Install QGround Control](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)

* Check which pixhawk device you have, first steps of [Loading PX4 Firmware instructions](https://docs.px4.io/master/en/config/firmware.html)
  * Go to settings (the cogs) > Firmware
  * Plug in device and read "Found device: ______"
  * This will determine which version of `px4fmu-v...` you should build
  * These instructions follow the setup for a Pixhawk 4, so we use v5_default in our `make` commands


## RPL PX4 build instructions for Ubuntu 20

Based on the [standard px4 build instructions](https://dev.px4.io/master/en/setup/building_px4.html), with necessary modifications for our version

* Clone [our px4-firmware repo](https://github.com/alsarmie/px4-firmware) into the folder where you want to contain everything

### GCC version you'll need
* Download this arm-gcc version: https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads/7-2018-q2-update
  * Or direct download (https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/7-2018q2/gcc-arm-none-eabi-7-2018-q2-update-linux.tar.bz2)

* Move it to same directory containing `px4-firmware`
  * (next to `px4-firmware`, not inside it)

* Follow commands in the share/doc/gcc-arm-none-eabi/readme.txt to build
  * `$ cd wherever/it/is`
  * `$ tar xjf gcc-arm-none-eabi-7-2018-q2-update-linux.tar.bz2`
  * Instructions also say to arm it, but I don't think you need to if you add line below to .bashrc ...

* Add path `export PATH=$PATH:/YOUR/PATH/TO/gcc-arm-none-eabi-7-2018-q2-update/bin` to `.bashrc`
  * `$ echo 'export PATH=$PATH:/YOUR/PATH/TO/gcc-arm-none-eabi-7-2018-q2-update/bin' >> ~/.bashrc`
  * `$ source ~/.bashrc`

### Back to building our version of PX4-firmware

* `$ cd px4-firmware`

* `$ git submodule update --init --recursive`
  * NOTE: I did this later in the process (after the uavcan stuff), but I think it should've been here. Please confirm where this instruction should be.

* Setup Ubuntu development environment based on [Development Environment on Ubuntu instructions](https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html)
  * `$ ./Tools/setup/ubuntu.sh` OR `$ bash ./Tools/setup/ubuntu.sh`
    * Not sure which is better or if it matters...
  * **Restart computer now**

* Attempt to build (won't actually work)
  * `$ make clean`
  * `$ make px4fmu-v5_default` # Version you use may differ

* Install compatible libuavcan version from Ludvig:
  * `$ cd px4-firmware/` # If not already there
  * `$ cd src/modules/uavcan/libuavcan`
  * `$ git reset --hard aa3650d34d9732af51ced52add24dee044eac79a`
  * `$ git fetch https://git.lericson.se/libuavcan.git`
  * `$ git merge FETCH_HEAD`
  * `$ cd -` # Takes you back to px4-firmware/
  * `$ PYTHONPATH=src/modules/uavcan/libuavcan/libuavcan/dsdl_compiler/pyuavcan`
  * You can try to run this script, but it'll fail. Just make sure the failure wasn't that it couldn't import uavcan
    * `$ python3 src/modules/uavcan/libuavcan/libuavcan/dsdl_compiler/libuavcan_dsdlc .`

* Attempt to build again (won't actually work) <- probably not necessary to try again here, but go for it
  * `$ make clean`
  * `$ make px4fmu-v5_default` # Version you use may differ

* Follow [this comment's solution](https://github.com/PX4/Firmware/issues/9863#issuecomment-411829513) to the math error you'll get
  * `$ cd platforms/nuttx/NuttX/nuttx/include`
  * `$ wget https://raw.githubusercontent.com/esp8266/Arduino/master/tools/sdk/libc/xtensa-lx106-elf/include/_ansi.h`
  * `$ cd -` # Takes you back to px4-firmware/

* Attempt to build again and it should work
  * `$ make clean`
  * `$ make px4fmu-v5_default`

* If pixhawk is connected you can do:
  * `$ make px4fmu-v5_default upload`
  * Otherwise do a make without `upload` and upload manually in QGroundControl per [Loading PX4 Firmware instructions](https://docs.px4.io/master/en/config/firmware.html)

### Likely other dependencies you'll need
* `$ sudo -H pip3 install jinja2`
  * Don't use `sudo apt-get install python-jinja2` as it seems to install for python 2
* `$ sudo apt-get install genromfs`
* `$ sudo apt-get install ninja-build` # Seems to help with building
* `$ pip install pyserial` # For using `upload` command appendix
