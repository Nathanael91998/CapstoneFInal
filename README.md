# Introduction
The goal of this manual is to guide the user throught the steps necessary to take the application from this repository and to download it onto your android device. This application is designed to connect to a BlueFruit NRF52 running the application which is detailed in the readMe below as well.

## Step 1: Downloading Android Studio
In order to download Android Studio you will first need to navigate to https://developer.android.com/studio and click the green button labeled "Download Android Studio".  If your device does not run on Windows-64 bit then you will need to select link below it labeled "Download Options" and select the download link that matches your system.  You will then need to launch the .exe file if you are using Windows, or launch the DMG file if you are using a Mac.  Follow the setup wizard and download whatever it recommends.  If you are having difficulties with this step consult the following link: https://developer.android.com/studio/install.  

## Step 2: Setting up the Environment
After downloading Android Studio, go ahead an open it.  A welcome page will appear with options for starting a new project. Select start a new Android Studio project and select Basic Activity.  Go ahead and name your project and select the language.  It is not important as to what you select as we are only looking to enter the IDE to download the necessary emulators and SDKs, and will download the application from this repository after.  Once you click Finish, you will enter the IDE.  In the top right of the IDE you should see a box icon with an arrow point down titled SDK Manager.  Select this button and then on the left side of the new window that pops up, select Apperance and Behavior > System Settings > Android SDK.  In the window select "SDK Tools" and check the box at the bottom titled "Show Package Details".  You will want to make sure you have the following tools installed: Android Emulator, Android SDK Platform-Tools, Goggle USB Driver, and Intel x86 Emulator Accelerator.  Under the dropdown titled "NDK (Side by side) make sure 21.3.6528147 is downloaded, and under the dropdown titled "Android SDK Build-Tools 31-rc3 you will want to make sure the following are downloaded: 30.0.2, 29.0.2, and 28.0.3.  It is possible that you may need to download more depending on what device you are attempting to develop and run this application on.  To choose which device you will be emulating and loading the application on select the AVD Manager Icon located directly to the left of the SDK Manager Icon. Select the button at the bottom left labeled "Create Virtual Device".  Then select your device (I ran this on a Nexus 5), then select the recommended system image, and then select finish.  Unless your device is running an API level lower than 28, you shouldn't need to download any more SDK build tools than the ones you've already installed.

## Step 3: Downloading the application
Your IDE should be ready to download and run the application.  Click on the green button labelled "Code" at the top of this repository. Then select the option download zip.  Unzip the file and open up Android Studio.  At the top left go ahead and select File > New > Import Project and then select the folder containing the project (You may already have it downloaded if you downloaded the CapstoneFInal-master folder from the flash drive).  The application should load and compile properly.  To test this, select the "Run App" icon at the top of the IDE, where it should run and simulate on the emulator.  You will not be able to test the Bluetooth capabilities until you have downloaded the application to a physical device however.

## Step 4: Importing to your Android device
Follow this guide for setting up your phone to be able to run your Android application: https://guides.codepath.com/android/Running-Apps-on-Your-Device.  It gives an overview of the steps needed to get the device running properly.  It doesn't give much detail regarding setting up the driver for your computer if you're developing on a Windows device, so I included an additional site I used to help me with this: https://developer.android.com/studio/run/oem-usb.  You should be able to use these two sources to get the application onto your phone.

## Potential Issues
* If you're having difficulty getting the code to compile and run, it is likely due to not having the proper SDK tools installed. Make sure you have the proper one installed for whichever device you wish to develop on.
* If your device is too old (8+ years) it may not be able to run the application.

# Arduino
Libraries used

https://github.com/cmaglie/FlashStorage

https://github.com/neu-rah/ArduinoMenu             ArduinoMenu library 4.18.2

https://www.arduino.cc/en/Reference/SD             SD 1.2.4 

https://github.com/adafruit/Adafruit_SSD1306

https://github.com/adafruit/Adafruit-GFX-Library   Adafruit GFX Library 1.0.0

https://github.com/adafruit/Adafruit_FeatherOLED   Adafruit Feather OLED 1.0.0

https://github.com/michd/Arduino-MCP492X

https://playground.arduino.cc/Code/Keypad/

https://github.com/stblassitude/Adafruit_SSD1306_Wemos_OLED

https://github.com/adafruit/Adafruit_nRF52_Arduino Adaruit Bluefruit

https://github.com/Wiznet/WIZ_Ethernet_Library Wiznet ethernet library

Board Manager: Adafruit nrf52 Ver 0.16.0

Adafruit Bluefruit Feather nRF52832 

The circuit:
analog sensors on analog ins 0, 1, 2, 3
SD card attached to SPI bus as follows:
	
	** MOSI - pin 11
	
	** MISO - pin 12
	
	** CLK - pin 13
	
	** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)
	
	CS - pin 28 MCP4922
	
	** OLED_reset pin 7
	
	**
	
	**

Author: Lawrence Kincheloe

add "{build.variant.path}/libarm_cortexM4lf_math.a" to platforms.txt in recipe.c.combine.pattern, before "{build.path/archive_file}"
  
add to boards.txt the below two lines, and replace #feather52832.build.extra_flags=-DNRF52832_XXAA -DNRF52 -DARDUINO_NRF52_FEATHER
feather52832.build.extra_flags=-DNRF52832_XXAA -DNRF52 -DARDUINO_NRF52_FEATHER -D__FPU_PRESENT -DARM_MATH_CM4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 
feather52832.compiler.arm.cmsis.ldflags="-L{build.variant.path}" -larm_cortexM4lf_math -mfloat-abi=hard -mfpu=fpv4-sp-d16


edit operation:

1 - first * -> enter field navigation use +/- to select character position

2 - second * -> enter character edit use +/- to select character value

3 - third * -> return to field navigation (1)

4 - fourth * without changing position -> exit edit mode


