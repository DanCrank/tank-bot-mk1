# tank-bot-mk1

A simple obstacle-avoiding vehicle.

Copyright (C) 2020 Dan Crank (danno@danno.org)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

the vehicle is currently based on:
    Adafruit Circuit Playground Bluefruit
        https://www.adafruit.com/product/4333
    Adafruit Crickit
        https://www.adafruit.com/product/3093
    RoboPeak RPLIDAR
        https://www.adafruit.com/product/4010
    DFRobot Devastator tank chassis
        https://www.dfrobot.com/product-1477.html
        (this is the kit version that uses the 6V metal geared motors:
        https://www.dfrobot.com/product-1476.html)

the Crickit is connected at its standard attachment points (see
https://learn.adafruit.com/adafruit-crickit-creative-robotic-interactive-construction-kit)

use of RPLIDAR requires the non-standard library https://github.com/robopeak/rplidar_arduino

the RPLIDAR is connected as generally described in the documentation (see
http://www.robopeak.net/data/doc/rplidar/appnote/RPLDAPPN01-rplidar_appnote_arduinolib-enUS.pdf)

     Pin              RPLIDAR                Circuit Playground
     1                GND                    GND (any)
     2                RX                     TX
     3                TX                     RX
     4                V5.0                   VOUT
     5                GND                    GND (any)
     6                MOTOCTL                AUDIO*
     7                VMOTO                  VOUT
      * NOTE: the circuit playground AUDIO pin is also passed through to
     the speaker terminal on the crickit. do not connect anything to the
     speaker terminal or it will interfere with RPLIDAR motor control.