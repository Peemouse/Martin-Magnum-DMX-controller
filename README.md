
# Martin Magnum 850/1200 DMX controller
DMX and artnet controller for fog machine Martin Magnum 850/1200

This custom ESP32-based controller adds DMX/ArtNet/sACN connectivity to a Martin Magnum 850 or 1200 (tested only on 850).
In addition, I also made an optional fan control box (which is a simple relay for controlling a fan ON/OFF).

Except few holes needed to attach the enclosure to the back of the machine, everything can be reverted and the use of the original remote is still possible (both DMX and original remote can't be used together).

You may find in this repo :
- Schematics, gerber files and BOM for building the hardware
- Firmware
- 3D files for printing a dedicated enclosure to be attached to the back of the Martin Magnum 850 fog machine.


Note :
Even if the board has been designed to work with Ethernet connectivity, only wired DMX is working as the first step of this project.
Any help is welcome !

## How does the Martin Magnum 850/1200 work ?

Those machine have a RJ45 connector for connecting a remote.
The Magnum 850 was delivered this a single button remote. A timer remote was available as an (expensive) option.
No DMX option.

Thankfully, some internet users could gather details, directly from MARTIN/JEM after sales service, about the RJ45 connector and how the machine reacts to the input signal.
Official Martin schematics are available in the [Official documentation folder](https://github.com/Peemouse/Martin-Magnum-DMX-controller/tree/main/Martin%20official%20docs).

|RJ45 pin|Description  |
|--|--|
| 1 | READY signal (open collector) |
|2 |HEATING signal (open collector)|
| 3 | Ground |
| 4 | Fog intensity signal (0-10V)|
| 5 | +15V |
| 6-7-8 | NC |

Note : on my personal fog machine, +15V is roughly +22V. But no problem !

The fog intensity signal obeys to the following rule :
| Fog signal (pin 4) | Behaviour |
|--|--|
| 0 - 1,5V | Machine OFF (not even heating) |
| 1,5V - 2V | Machine Idle (heating, no output) |
| 2V - 10V | Output fog proportionally to signal|

## Hardware
The controller board has been designed under EasyEDA : https://oshwlab.com/pimousse/led_artnet_copy

Stable version from V1.2.

## Enclosure
You may find a 3D enclosure for the board and connectors on Makerworld `(TODO : add )`
The shape and fixing holes positions have been designed according to the Martin Magnum 850 to be fixed at the back of the machine.

## How does this controller work ?

> At the moment, only wired DMX is working (any help for dev the Ethernet/artnet/sACN features is welcome).

Two modes are available: DMX connected / No connection (= manual mode)
At startup, if no DMX is connectd, the machine is in IDLE mode (heating but not outputing).
If DMX cable is disconnected, the fogger keeps the last DMX values received.

### DMX mode
In DMX mode, the fog machine is driven by 3 DMX channels :
|Channel | Behaviour  | Values  |
|--|--|--|
| 1 | Fog intensity | 0-255 => 0-100% |
| 2 | Timer ON | 0-255 sec |
| 3 | Timer OFF | 0-255 sec |
| 4 | FAN control | 0-127 => OFF; 128-255 => ON |

Notes :
- Continous mode : if either `Timer ON = 0` or `Timer OFF = 0`, the fog machine will be in continous mode.
- Cool down mode : if `Fog intensity = 0` and`Timer ON = 0` and `Timer OFF = 255`, the fog machine will switch into Cool down mode (no more heating, unable to output fog).

For changing the address of the machine on the DMX bus, press Enter button for 2sec.
An arrow will appear next to the channel value. Change it by pressing -/+ buttons.
Apply settings and exit by pressing 2sec on Enter button.

If you stay longer than `TIMEOUT_MENU` (set to 10sec) without activity in the menu, it will automatically exit.

### Manual mode
`No connection` is blinking on the screen if no DMX data is received.
You can fire fog at full intensity by pressing the + button.

You also can use a timer by modifying `Fog intensity`, `Timer ON` and `Timer OFF` values manually by pressing on the Enter button for 2sec.
Use +/- buttons for setting a value and press Enter for changing the next parameter.
Apply settings and exit by pressing 2sec on Enter button.

## Firmware parameters
You will need to tweak some values in `parameters.h` file of the firmware according to your hardware for adjusting the PWM output:

    #define  VOUT_IDLE  37 //1,5Vout
    #define  VOUT_START  48 //2Vout
    #define  VOUT_END  243 //10Vout
With a multimeter, probe the fog signal that goes into the fog machine (the signal can be probed on the OUT RJ45 connector on the machine) and change the parameters above to match the corresponding voltages.

