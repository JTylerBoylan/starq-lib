# STARQ Documentation
*Updated 01/29/2023*

## NVIDIA Jetson AGX Orin 64GB Developer Kit

### NVIDIA Store
https://store.nvidia.com/en-us/jetson/store/

### Developer Guide
https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/index.html

### Login
```
username: nvidia
password: nvidia
```

## Motor Controllers

### ODrive Shop
https://odriverobotics.com/shopfolder

### ODrive Documentation
https://docs.odriverobotics.com/v/latest/guides/getting-started.html

### ODrive Pro Data Sheet
https://docs.odriverobotics.com/v/latest/hardware/pro-datasheet.html

### MJ5208 Moteus Motor Shop
https://mjbots.com/products/mj5208

### MJ5208 ODrive Auto-Configuration Script
1. Plug in USB from the Jetson to the ODrive controller 
2. Open the Terminal on the the Jetson
3. Go to the `docs` folder: $`cd ~/starq-lib/docs`
4. Run the auto-configuration script: $`python3 configure_odrive.py <CAN_ID>`
5. Let the script complete before unplugging

*Note: The thermistor needs to be plugged in before running this script*

## ODrive CAN Communication

### ODrive CAN Guide
https://docs.odriverobotics.com/v/latest/guides/can-guide.html \
Instructions on how to set up a CAN network.

### ODrive CAN Protocol
https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#can-protocol \
CAN frame IDs and required data for the different ODrive commands.

### ODrive Pinout
https://docs.odriverobotics.com/v/latest/hardware/pro-datasheet.html#pro-pinout \
Locations of the CAN, thermistor, and other IO pins.

## Jetson AGX Orin CAN

### Waveshare SN65HVD230 CAN Board
https://www.amazon.com/SN65HVD230-CAN-Board-Communication-Development/dp/B00KM6XMXO \
Converts CAN frames from GPIO to High/Low signals on the CAN bus.

### Jetson GPIO Pinout
https://jetsonhacks.com/nvidia-jetson-agx-orin-gpio-header-pinout/ \
Useful for determining where the 5V, 3.3V, GND, and CAN pins are on the GPIO array.

### Jetson CAN Documentation
https://docs.nvidia.com/jetson/archives/r35.4.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html \
Instructions on how to enable CAN on the Jetson.

### Jetson CAN GPIO Pins

| CAN0 RX | CAN0 TX | CAN1 RX | CAN1 TX | 3.3V |     GND     |
| ------- | ------- | ------- | ------- | ---- | ----------- |
|   29    |   31    |    37   |    33   | 1/17 | 25/30/34/39 |

### Jetson CAN Setup
1. Open the Terminal on the Jetson
2. Go to the `docs` folder: $`cd ~/starq-lib/docs`
3. Run the command: $`sudo ./loadcan_jetson.sh`

## STARQ C++ Library

