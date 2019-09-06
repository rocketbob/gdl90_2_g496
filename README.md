# README #

This code allows a NodeMCU to log into any ADS-B out unit that supports GDL90 via. WIFI and output RS232 TIS-A information to a Garmin 396/496.

### How do I get set up? ###

Pin D4 of NodeMCU must be connected to a RS232 level shifter (such as MAX232).  The RS232 output must be connected to the Garmin RS232 input.  The Garmin must be configured to accept TIS input via serial.

The wifi SSID must be configured for your ADS-B unit.

Libraries:
http://arduino.esp8266.com/stable/package_esp8266com_index.json

### Who do I talk to? ###

* skywaycaptain@hotmail.com
* rocketbob@gmail.com# gdl90_2_g496
# gdl90_2_g496
