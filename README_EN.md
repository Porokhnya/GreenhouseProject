<h1>What the firmware can do</h1>
<p>
 <ul>
  <li>Send data to IoT (ThingSpeak and <a href="http://gardenboss.ru" target="_blank">GardenBoss.ru</a>);</li>
  <li>Able to be controlled and configured through configuration software and web-based interface;</li>
  <li>Receive and execute external commands from <a href="http://gardenboss.ru" target="_blank">GardenBoss.ru</a>;</li>
  <li>Publish MQTT-topics;</li>
  <li>Can be controlled via SMS and MQTT;</li>
  <li>Able to use different gates to WAN/LAN: GSM/GPRS (SIM800), Wi-Fi (ESP8266), LAN (W5100);</li>
  <li>Able to work with sensors using different ways - connected directly to microcontroller, through RS-485, 1-Wire, nRF, LoRa;</li>
  <li>Able to be configured by using smart rules - is the key feature of the firmware, where user can provide and manage the needed firmware behaviour;</li>
  <li>Work with different display models - Nextion, LCD 128x64, 7" TFT on Arduino Due;</li>
  <li>Work with different microcontrollers: Arduino Mega, Arduino Due and any PCB, based on it;</li>
  <li>Flexible firmware configuration: work through port extenders, include/exclude software modules in the firmware and so on;</li>
  <li>Work with many sensors of many types (humidity, temperature, water flow and so on);</li>
  <li>Compute deltas within two sensors, even different hardware types (for example, compute temperature delta between DS18B20 and DHT22);</li>
  <li>Work with reservation lists;</li>
  <li>Work with universal modules - is the key feature of the firmware, when the sensors connected to the universal module PCB, and can be queried through RS-485, 1-Wire, nRF, LoRa;</li>
  <li>Gather logs to many locations: local file on SD-card, web-based interface, GardenBoss.ru and so on;</li>
  <li>Alarm support: when the rule is raised by its conditions - you will receive alarm SMS;</li>
  <li>And many, many tasty features...</li>
 </ul>
 
<h1>ATTENTION!</h1>
The firmware needs to be configured through `Globals.h` before uploading to the microcontroller! 
<p>
<h1>License</h1>

For home use only, any commercial use is strictly prohibited. If you want to use this firmware for commercial purposes - please write to <a href="mailto:spywarrior@gmail.com">spywarrior@gmail.com</a> for details.

<h1>Project structure</h1>
<ul>
<li>Arduino IDE version, used for compiling the project - is <b>1.8.5</b>;</li>
<li><b>Main</b> folder - firmware for the microcontroller;</li>
<li><b>ESP_BIN</b> folder - firmware for the ESP8266;</li>
<li><b>SOFT</b> folder - configuration software, can be connected to microcontroller by using the COM-port;</li>
<li><b>Libraries</b> folder - libraries, used in project, need to be installed in Arduino IDE before compiling the firmware;</li>
<li><b>SD</b> folder - these files should be placed on SD card;</li>
<li><b>NewPlan_en.spl7</b> file - SPlan 7.0 project with schematic;</li>
<li><b>Nextion</b> folder - firmware for Nextion 320x240;</li>
<li><b>UniversalSensorsModule</b> folder - firmware for universal module with sensors;</li>
<li><b>UniversalExecutionModule</b> folder - firmware for universal module with execution abilities (translate state of the main controller over supported hardware interfaces);</li>
<li><b>FrequencySoilMoistureSensor</b> folder - firmware for frequency soil moisture sensor support;</li>
<li><b>WEB</b> folder - web-based interface with PHP and sqlite3 support.</li>
</ul>

<h1>Default firmware settings</h1>

Please be attentive - the default firmware settings most likely will not conform your needs! All settings can be (and SHOULD be) changed through the <b>Globals.h</b> file in the firmware source code.

<br/>
<h1>Configuration software screenshots</h1>

<details> 
<summary><b>Click to view...</b><br/><br/></summary>
  
<img src="screen1_en.png" hspace='10'/>
<img src="screen2_en.png" hspace='10'/>
<img src="screen3_en.png" hspace='10'/>
<img src="screen4_en.png" hspace='10'/>
<img src="screen5_en.png" hspace='10'/>
<img src="screen6_en.png" hspace='10'/>
<img src="screen7_en.png" hspace='10'/>
<img src="screen8_en.png" hspace='10'/>
<img src="screen9_en.png" hspace='10'/>


</details>

<p>
<h1>How to use</h1>
<ul>
<li><b>Install the libraries (in Libraries folder) to the Arduino IDE!</b></li>
<li><b>READ ALL INSTRUCTIONS AT THE TOP OF MAIN.INO FILE!!!</b></li>
<li><b>All firmware settings are located in the Globals.h file!</b></li>
</ul>
<p>


<h1>EXAMPLE schematic</h1>
<img src="plan_en.png"/>

<h1>Questions, feedback and so on</h1>

Please write to <a href="mailto:spywarrior@gmail.com">spywarrior@gmail.com</a>
