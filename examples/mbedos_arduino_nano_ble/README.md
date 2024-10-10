# LSM9DS1 - MBED OS

LSM9DS1 C++ library for MBED OS.

## Overview

This example demonstrates usage of LSM9DS1 for reading accelerometer, gyroscope.

### Hardware Required

To run this example, you should have Arduino Nano 33 BLE Sense. The LSM9DS1 is a system-in-package featuring a 3D digital linear acceleration sensor, a 3D digital angular rate sensor, and a 3D digital magnetic ensor. You can read the datasheet [here](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf).


#### Pin Assignment:

|                  | SDA             | SCL           |
| ---------------- | -------------- | -------------- |
| I2C Master       | I2C_MASTER_SDA | I2C_MASTER_SCL |
| LSM9DS1          | SDA            | SCL            |


For the actual default value of `I2C_MASTER_SDA` and `I2C_MASTER_SCL` see `Example Configuration` in `menuconfig`.

**Note: ** There’s no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.

## How to set up this project:

1. Clone it to your machine.  Don't forget to use `--recursive` to clone the submodules: `git clone --recursive https://github.com/kimsniper/lsm9ds1_cpp.git`
2. You may want to update the mbed-os submodule to the latest version, with `cd lsm9ds1_cpp/examples/mbedos_arduino_nano_ble/mbed-os && git fetch origin && git reset --hard origin/master`
3. Set up the GNU ARM toolchain (and other programs) on your machine using [the toolchain setup guide](https://github.com/mbed-ce/mbed-os/wiki/Toolchain-Setup-Guide).
4. Build the project
    
    1. Open a terminal in the lsm9ds1_cpp\examples\mbedos_arduino_nano_ble.
    2. Make sure you have the needed Python requirements: python3 -m pip install -r mbed-os/tools/requirements.txt (use python instead of python3 on Windows)
        On recent Debian/Ubuntu versions, pip can no longer install packages to the system interpreter. On these systems, you should use xargs sudo apt-get install -y < mbed-os/tools/requirements.apt.txt to install the needed python packages via apt instead.
        This is no longer needed with mbed-os from 3-14-2024 or later
    3. Create and enter a build directory: mkdir build && cd build
    4. Run CMake: `cmake .. -GNinja -DCMAKE_BUILD_TYPE=Develop -DMBED_TARGET=ARDUINO_NANO33BLE -DARDUINO_BOSSAC_SERIAL_PORT:STRING='<your serial port>'`
        
        For ARDUINO_BOSSAC_SERIAL_PORT, it could be 'COM7', '/dev/ttyACM0', etc. Note that in Windows, the com port will change from normal mode to programming mode (See step 5).
        The Develop build type is recommended for normal development work, but there is also the Debug build type which disables optimizations, and the Release build type which disables debug information.
    5. Build the project by running `ninja`
5. Double press the reset button on the Arduino nano BLE sense to put the device in programming mode. 
6. Flash to device
    run `ninja flash-mbedos_implementation` to upload it to a connected device.

## Example Output

```bash
Accelerometer X: 0.69
Accelerometer Y: 0.55
Accelerometer Z: 10.01
Gyro Roll: 1.88
Gyro Pitch: 3.16
Gyro Yaw: 0.72
```
