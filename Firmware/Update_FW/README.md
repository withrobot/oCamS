# How to update the firmware of oCamS
### Step 1. Run *[UpdateFW.exe]*
![ScreenShot](../../images/1_Run_UpdateFW.jpg)
### Step 2. Click *[Erase FW]* to erase the installed firmware of oCamS.
### Step 3. Check *[Device Manager]* to see if oCam appears as a WestBridge device with unknown device driver as shown below.
![ScreenShot](../../images/2_device_change.jpg)
### Step 4. Install the device driver of the WestBridge device using the driver in the *[drv_1.2.3.10]* directory.
### Step 5. Check *[Device Manager]* to see if oCamS appears as a Cypress USB BootLoader device as shown below.
![ScreenShot](../../images/3_driver_setup.jpg)
### Step 6. Run again the *[UpdateFW.exe]*.

![ScreenShot](../../images/4_UpdateFW.jpg)
### Step 7. Click *[Write FW]* to select the firmware image file (*.img). After selecting a new firmware, UpdateFW will write the new firmware to oCamS. On successful writing, a message window will pop up as shown below.
![ScreenShot](../../images/5_Firmware_Update.JPG)
### Step 8. Disconnect and reconnect the USB cable to complete the firmware update procedure.
