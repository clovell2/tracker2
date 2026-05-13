# Tracker
This produces track reports based on current ADS-B Mode S messages recieved by a RTL-SDR V4 and prior recieved messages being coasted.

## Overview
The `dump1090` library is capable of displaying received ADS-B messages to either the system terminal or to a webpage via a TCP connection. This is a snapshot of the current airspace, but does not account for ADS-B dropout which can occur frequently. 

This project resolves this issue by creating a track system that coasts prior reported ADS-B messages and propogates those reports in time, with growing position uncertainty, until a user specified timeout or uncertainty threshold is reached.

## Dependencies
Currently, the external library dependencies are submoduled in the [external](/external/) folder.
 - dump1090 - for RTL-SDR hardware connection
 - Eigen - for internal tracker math and estimation
 - spdlog - for logging 
 - Catch2 - for unit test framework
 - nolohmann/Json - for output file construction

### WSL installs
sudo apt-get install libusb-1.0-0-dev
sudo apt install pkg-config
sudo apt-get install librtlsdr-dev

 ### Connecting a USB device to WSL
 Instructions are here:
 https://learn.microsoft.com/en-us/windows/wsl/connect-usb#attach-a-usb-device

 in powershell with admin permissions
 ```
 usbipd list
 usbipd bind --busid <busid>
 usbipd attach --wsl --busid <busid>
```

in linux terminal verify that the usb device is reachable
> lsusb

 ## Build Instructions
 ### On a clean pull
 First we need to build and install the dependencies. We do this seperately so subsequent rebuilds of the Tracker do not have to also rebuild the dependencies.

```
# spdlog Build and Install
cmake -S ./external/spdlog -B ./external/spdlog/build
cmake --build ./external/spdlog/build
cmake --install ./external/spdlog/build --prefix ./external/spdlog/install
```

```
# Catch2 Build and Install
cmake -B ./external/Catch2/build -S ./external/Catch2/ -DBUILD_TESTING=OFF
sudo cmake --build ./external/Catch2/build/ --target install
```

 ### Build the Tracker

 ```
cmake -S . -B ./build -DBUILD_TESTING=ON
cmake --build ./build
 ```

 ### Running Tracker
 ```
 ./build/Tracker_main
 ```
 ### Running Tracker Tests
 ```
 ./build/Tests
```