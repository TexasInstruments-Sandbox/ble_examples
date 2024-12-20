Smart Remote Control (SRC) - SW Reference Design
================================================

Functionalities
---------------

The following features have been implemented and tested using the ONN Android receiver.

   1. **BLE Connection, Pairing and Bonding:**
      - Implementation of Advertisement with BLE standard UUIDs that are recognized by the Android receiver to start connection/paring/bonding process.
      - Implementation of Directed Adv with peer address saved while bonding (re-connection is faster with Directed Advertisement) - all the re-connections after the bonding will be done with Directed Adv.
   2. **BLE Device Identification Service:**
      - Implementation of Device ID BLE service: device type, name, VID/PID, as well as SW, FW and HW versions are available as GATT characteristics.
      - Verified that ID BLE service is recognized by Android TV receiver and the characteristics values shown.
   3. **BLE Battery Monitoring:**
      - Implementation of battery monitoring using the Battery Monitor driver.
      - Implementation of Battery monitoring BLE service.
      - Verified that Battery Service is detectable by Android TV receiver and percentage value of power consumption updates accordingly.
   4. **Dual-Image BLE OAD support:**
      - Implementation of Dual-Image OAD service for over the air firmware update.
      - Verified that OAD firmware update is successfully done using Simple Link Connect Mobile App.
   5. **BLE Human Interface Device (HID):**
      - Implementation of BLE HID service using Android spec for expected characteristic values.
      - Implementation of database table with Android spec HID key UUID codes.
      - Verified that HID key codes are send when keys are physically pressed at the RC.
      - Verified that HID key command are recognized by Android receiver.
   6. **Audio over BLE:**
      - Implementation of Audio BLE service using Android spec for expected characteristic values.
      - Implementation of audio start/stop notification values.
      - Implementation of DMIC Audio: pdm data fetching (SPI), pdm to pcm, decimation and filtering (16KHz - 16-bit integer), compression (adpcm/msbc support), BLE transmission through notification.
      - Implementation of AMIC Audio: data fetching (ADC/DMA), filtering (16KHz - 16-bit integer), compression (adpcm/msbc support), BLE transmission through notification.
      - Verified that audio quality meets specs: SNR > 30 dB and THD < 5% at 1KHz (measured at receiver side - not using Android receiver).
      - Verified that audio can be transmitted within 10-meter line of sight distance.
      - Verified that audio is properly understood by speech to text assistant Android engine.
   7. **Infrared (IR) support with NEC standard:**
      - Implementation of IR NEC standard driver.
      - Implementation of database table with Android spec IR key codes.
      - Verified that IR key codes are send when keys are physically pressed at the RC using a logic analyzer.
   8. **Infrared (IR) External NVS Database:**
      - Implementation of function to write IR codes into external flash.
      - Implementation of function to read IR codes from external flash.
   9. **LED Behavior:**
      - Implementation of LED color behavior based on spec: Orange for IR (no BLE connection), Red for non-secure BLE connection (no pairing and bonding), Green for secure BLE connection (device is bonded).
   10. **Power Consumption Optimization:**
      - Implementation of Shutdown and key interrupt wake-up: after the RC device has connected to the ONN Android receiver, if no keys are pressed for more than 15 minutes, then the RC device will issue a disconnection and go to shut down. Pressing any key will wake up the device which will connect automatically with the ONN Android receiver.
      - Implementation of Limited Advertising: Once the device has woke up, it will advertise only for 15 minutes before going to shut down. Pressing any key will wake up the device which will start again advertising.
      - Implementation of Connection Latency: when the peripheral has nothing to send to the central, it will skip 200 events * 11,25 ms of conn interval = 2.2 seconds of standby between connection events.

Known Issues and Limitations
----------------------------

   1. If OAD is done using Simple Link Connect mobile app and the device bonds first, the operation will fail (OAD stops suddenly during update).
   2. If *devInfoServGenericAcc* UUID and its characteristics are included in the GATT table, then it is not possible to see all the services using Simple Link Connect mobile app. Therefore, those characteristics are currently commented out from the *devInfoAttrTbl* inside *dev_info_service.c* to test OAD using the mobile app.
   3. All re-connections between RC and ONN Android receiver after the bonding will be done with Directed Adv. However, the device should go back to Undirected Adv when the bond is erased from the central and when the HOME + BACK keys are pressed (this functionality has not been implemented so far).
   4. The functionality of the key that is pressed to wake up the RC is currently lost (as the action of pressing the key is only used to wake up the RC), therefore the user will have to press two times the same button if it wants the functionality of the key to work.
   5. The RC device takes around 4 seconds to be responsive again. This is mainly due to the rebooting process and the MCUBoot being in debug mode.
   6. Some special HID keys do not generate the desired answer from the ONN receiver, this needs to be checked with the provider as we have verified that different Android compliant RCs do have the same limitation which would suggest a special nonstandard protocol is implemented on the Android receiver. The keys are: YouTube, Netflix, App03, App04, Settings, Profile. If the Vol +, Vol - and Mute buttons do not work as expected, please make sure to take a look at the buttons configuration inside the ONN Bluetooth settings. To check this, go to Bluetooth settings and select "Set up remote buttons" and select "onn. 4K Streaming Box" for Volume Control.
   7. There is no standardized OAD service defined for the Android Receiver (Android spec only provides recommendation of what should be provided by the vendor). The ODA Dual Image distributor implementation has to be presented to Google so that they can add support for it.
   8. Missing Infrared (IR) testing on TV receiver.
   9. Missing multiple key-detection implementation.

Quick Demo Guide
----------------

If you are looking for a quick demo test, please do the following:

1. Download/Clone the repository.
2. Go to the *Demo* folder.
3. If you want to test the AMIC implementation, use the *xxx_AMIC_v1.bin* file.
4. If you want to test the DMIC implementation, use the *xxx_DMIC_v1.bin* file.
5. For either of the Audio implementations, use the *mcuboot_dual_image_LP_EM_CC2340R5_nortos_ticlang.hex*.
6. Open *UNIFLASH* and load the *mcuboot_dual_image_LP_EM_CC2340R5_nortos_ticlang.hex* file and load the *smart_remote_control_digital_LP_EM_CC2340R5_v1.bin* selecting the **load address = 0x00006000**.
7. Connect the cc23xx TI-RC board using a XDS110 probe and flash it.
8. Power cycle the board (disconnect and connect the XDS cable) - Important not to forget this step.
9. Power on the ONN Android TV receiver, the RC should connect automatically. If this does not happen, to re-pair press the button for 15 seconds in the back of the ONN receiver until the device starts scanning for RCs.
10. The ONN receiver will connect, pair, and bond to the RC device with default name: ONN-Remote.
11. Browse the ONN user interface using the keys. Just make sure the following buttons work: Center, Left, Up, Right, Down, Home, Back, Power, Mute. In addition, the LED should turn green when pressed.
12. Press and hold the Assistant Key (Audio) while speaking and verify that what you said is prompted in the TV interface.

*Note:* During a connection, if no keys are pressed for more than 15 minutes, then the RC device will issue a disconnection and go to shut down. Pressing any key from the RC will wake up the device which will connect automatically with the ONN Android receiver. Please be aware that the RC device takes around 4 seconds to be responsive again. This is mainly due to the rebooting process which needs to be optimized. During advertising, the device will always advertise only for 90 seconds before going to shut down. Pressing any key will wake up the device which will start again advertising.

Quick Dev Guide
---------------

If you are looking to build the project from CCS, please do the following:

1. Download/Clone the repository.
2. Install the simplelink_lowpower_f3_sdk_8_20_00_119 SDK from ti.com
3. Drag and drop the contents inside the simplelink_lowpower_f3_sdk_8_20_00_119_patch folder to the root of the simplelink_lowpower_f3_sdk_8_20_00_119
   install directory (default installation location is c/ti/simplelink_lowpower_f3_sdk_8_20_00_119). Overwrite any file conflicts with the contents inside
   the simplelink_lowpower_f3_sdk_8_20_00_119_patch folder.
   * Skipping this patch will result in a SysConfig build failure. Ensure you patch your local 8.20 SDK with the patch contents.
4. In order to enable/disable AMIC or DMIC audio solutions, please use the Compiler pre-define symbols inside the project Properties: ``USE_AMIC`` or ``USE_DMIC``. Please be aware that both should not be enabled at the same time.
5. In order to enable/disable the AMIC or DMIC Filters, please use the Compiler pre-define symbols inside the project Properties: ``USE_ANALOG_FILTER`` or ``USE_DIGITAL_FILTER_1`` and ``USE_DIGITAL_FILTER_2``. Please be aware that both should not be enabled at the same time.
6. In order to enable/disable the ADPCM or MSBC codec, please use the Compiler pre-define symbols inside the project Properties: ``USE_ADPCM_CODEC`` or ``USE_MSBC_CODEC``. Please be aware that both should not be enabled at the same time and that Android compliant TV only support ADPCM codec.
7. In order to enable/disable OAD Dual-image, please use the following *.cmd* file: cc23x0_app_freertos_OAD.cmd and make sure you exclude from build the cc23x0_app_freertos_no_OAD.cmd file. Follow the Section below for further details on how to ``Enable OAD``.
8. After development/modifications to the project, please make sure to execute the steps described below in **Workflow**.

Computing Requirements
----------------------

**DMIC Solution:**
   - MCUBoot Flash size: 24.83 KB
   - RC Application Flash size: 200 KB
   - NVS Bonds Flash size: 16.38 KB
   - Total Flash size for Dual-Image OAD: 439.22 KB
   - Total RAM size: 33 KB

**AMIC Solution:**
   - MCUBoot Flash size: 24.83 KB
   - RC Application Flash size: 200 KB
   - NVS Bonds Flash size: 16.38 KB
   - Total Flash size for Dual-Image OAD: 441.22 KB
   - Total RAM: 31 KB

Workflow
--------

If you are working on the RC development, please take a look at the following:

Functional Regression Tests:

To make sure we are not breaking other main functionalities of the SRC project while implementing new features or debugging.
Please make sure to execute the next list of steps. You will need the SRC custom board, an Android TV receptor with internet connection (ONN TV for instance).

   1. Build and flash the project (make sure you are using the cc23x0_app_freertos_no_OAD.cmd for no OAD).
   2. Search for Bluetooth devices in the Android TV receiver using the ONN remote control to go to settings.
   3. Connect, pair, and bond to the device with default name: TI-RemoteControl.
   4. Browse the UI using the keys. Just make sure the following buttons work: Center, Left, Up, Right, Down, Home, Back, Power, Mute. In addition, the LED should turn green when pressed.
   5. Press and hold the Assistant Key (Audio) while speaking and verify that what you said is correctly prompted in the TV interface.
   6. Verify that the device is able to go to Standby using EnergyTrace.

**Once you have completed successfully the steps, you can safely commit your changes.**

Enable OAD
----------

In order to flash and debug the *smart_remote_control_digital_LP_EM_CC2340R5* project with OAD, please use the following *.cmd* file: cc23x0_app_freertos_OAD.cmd and make sure you exclude from build the cc23x0_app_freertos_no_OAD.cmd file.

Functional Regression Tests - OAD included:

To make sure we are not breaking other main functionalities of the SRC project while implementing new features or debugging.
Please make sure to execute the next list of steps. You will need the SRC custom board, an Android TV receptor with internet connection (ONN TV for instance), and a mobile phone with the Simple Link connect app for the OAD process.

   1. Build the project.
   2. Flash the *mcuboot_dual_image_LP_EM_CC2340R5_nortos_ticlang.hex* file and *smart_remote_control_digital_LP_EM_CC2340R5_v1.bin* (load address = 0x00006000) file program into the SRC custom board.
   3. Power cycle the board (disconnect and connect the XDS cable).
   4. Search for Bluetooth devices in the Android TV receiver using the ONN remote control to go to settings.
   5. Connect, pair, and bond to the device with default name: TI-RemoteControl.
   6. Browse the UI using the keys. Just make sure the following buttons work: Center, Left, Up, Right, Down, Home, Back, Power, Mute. In addition, the LED should turn green when pressed.
   7. Press and hold the Assistant Key (Audio) while speaking and verify that what you said is correctly prompted in the TV interface.
   8. Disconnect from Android TV receiver using the ONN remote control to go to settings.
   9. Use the Simple Link App to search for the device and connect to it.
   10. Go to the OAD service, load the *smart_remote_control_digital_LP_EM_CC2340R5_v2.bin* file (modify the Advertisement device name using SysConfig -> BLE -> General Configuration) and start the OAD process. It should take less than 60 seconds to complete.
   11. Once the image has been downloaded and the OAD process ends successfully, disconnect from the device.
   12. Run all the steps from 5 to 8 again. You should verify that the device is now advertising with the new Advertisement Device Name you set for the _v2.bin file.
   13. Verify that the device is able to go to Standby using EnergyTrace.

**Once you have completed successfully the steps, you can safely commit your changes.**
