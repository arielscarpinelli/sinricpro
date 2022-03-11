# ESP8266 Garage door opener controller for Google Home

The code here works with an NodeMCU and a garage opener that has connectors for an external push button and a light that should be turned on while the door motor is running.

To connect with Google Home it uses Sinric.pro

I don't like Google Home not letting the option to open and close in screen when it is mapped as an actual garage door, so I map it as a light.


# Interface

To connect the NodeMCU with the door opener, given it works with 24V DC, I decided to optoisolate the connections using a couple of 4N35s mounted on a small piece of perfboard.

For power I could have used the +24V out of the opener, but that would have required to use a very inefficient linear regulator to drop down to 3.3v; or well a DC-DC converter, but that would be expensive and hard to get here.

Given I was interfacing with optocuplers, I chose to use an old cheap USB phone charger and power it thru the USB connector.

Here is the schematic:


![Schematic](https://raw.githubusercontent.com/arielscarpinelli/esp8266-garage-door-opener-mqtt/master/interface-schematic.svg?sanitize=true)



