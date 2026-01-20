# Arduino IDE Installation and Usage Tutorial

1.	Enter Arduino's official website：https://arduino.cc
2.	Click  'SOFTWARE' on the home page to enter the download page
3.	Select the corresponding system version to download the installation package
   ![1](https://github.com/user-attachments/assets/33312775-3ca4-4491-ba1a-62c2c4ba9427)

4.	Open the installation package, select the installation path, and install the Arduino software
   
   5.	Download the file [stopwatch.ino](https://github.com/Grovety/CrowBot_GRC_program/blob/main/stopwatch.ino) and open it.
      
 ![2](https://github.com/user-attachments/assets/f710330b-5f63-4de0-ab1e-d5f6fa39423d)

6.	Connect the CrowBot-BOLT with the computer via the USB cable and turn it on.
 
<img width="644" alt="3" src="https://github.com/user-attachments/assets/a9ce9c02-f667-4ff6-aa0d-a42d0d987f8e" />


7.	Click File ->Preferences ->Additional boards manager URLs->Add the following URLs:

https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_dev_index.json

https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 
 ![4](https://github.com/user-attachments/assets/10d065c5-b950-4d0f-9eeb-f34bef9de88e)

![5](https://github.com/user-attachments/assets/5db0681c-96db-4a32-8cc3-d7c962c8d908)


8.	Install the esp32 hardware package. 
Click Tools ->Board ->Board Manager. 
Then search for esp, find esp32 by Espressif Systems, **choose the new version**, and click Install.
 ![6](https://github.com/user-attachments/assets/469785b9-dd04-46ea-8a75-9101cb0095e6)


9. Click "Tools"->"Board" to pop up the motherboard selection window, and select "ESP32-WROOM-DA Module" motherboard from the pull-down menu;
    ![7](https://github.com/user-attachments/assets/52876ff7-7f58-460c-9bd5-6a09496b54da)

 
10.	Click “Tools” -> “Port”, and select the corresponding serial port;
11.	Install needed libraries
    
 ![8](https://github.com/user-attachments/assets/a397c8ab-87d7-4381-8a58-faa7cbd9a090)

![9](https://github.com/user-attachments/assets/df8bc38c-68e7-4a23-bae9-b924d8c7fbe7)

 ![10](https://github.com/user-attachments/assets/370e5a68-7699-41b5-93b6-158c849d5d90)


 

12.	Click the Upload button below the menu bar to upload the program to your Robot
  ![11](https://github.com/user-attachments/assets/60dc89e6-1cea-4502-b13b-4e94310fa899)
  
 
13.	After the program is successfully uploaded, it will be prompted that the download is successful.
    
 <img width="494" alt="12" src="https://github.com/user-attachments/assets/fb3e8dcb-080e-4b8a-9017-ad302a657215" />


14.	Turn off the robot and disconnect the CrowBot-BOLT from the computer. It’s ready to use.
15.	Switch it on, switch on the Development Board, and remember to switch on Bluetooth on the robot. Have fun!
