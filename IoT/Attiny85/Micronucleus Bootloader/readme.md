# Flashing and Updating Micronucleus (no USB) Bootloader for ATtiny85
Those are the steps to flash and update the micronucleus bootloader for the ATtiny85 using an Arduino as ISP.

## Steps

### Step 1: Configure an Arduino as ISP
Follow this tutorial to configure your Arduino as an ISP: [Arduino as ISP Tutorial](https://makersportal.com/blog/2018/5/19/attiny85-arduino-board-how-to-flash-the-arduino-bootloader-and-run-a-simple-sketch)

### Step 2: Flash the Bootloader using the Arduino as ISP
Use the tool in `FlashBootloader_HackEduca_V2.0.1` to flash the bootloader onto your ATtiny85.

### Step 3: Update the Bootloader using the USB Dongle
Use the USB dongle to update the bootloader with the following command:
```
micronucleus --run .\upgrade-t85_entry_on_power_on_no_pullup_fast_exit_on_no_USB.hex
```
You can find micronucleus [here](https://github.com/micronucleus/micronucleus).

### Step 4: Add the URL to Arduino IDE
Go to `File > Settings` and add the following URL:
```
https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json
```

### Step 5: Install ATTinyCore by Spence Konde
Go to `Tools > Boards > Boards Manager` and install ATTinyCore by Spence Konde.

### Step 6: Configure ATTiny85 in Arduino IDE
Use the configuration reported in `Arduino IDE settings configuation.png` to build sketches on the ATTiny85 using the micronucleus bootloader.