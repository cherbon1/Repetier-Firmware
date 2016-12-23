# Inofficial modification of the RF firmware for RF1000 and RF2000 devices
Based on Repetier-Firmware - the fast and user friendly firmware.

## Disclaimer

THIS VERSION IS AN UNOFFICIAL MODIFICATION OF THE ORIGINAL FIRMWARE. It is neither endorsed nor
supported by the developers of the RF1000 firmware. USE AT YOUR OWN RISK.

For the official version, have a look at the upstream repository:
https://github.com/RF1000/Repetier-Firmware


## Version RF.01.35mod (2016-12-23)

* This version is based on RF.01.35 and introduces a new command M3900 to scan for the heat bed offset.

For more details, have a look at this forum post (in German):
http://www.rf1000.de/viewtopic.php?f=7&t=1504#p14882


## Version RF.01.35mod forked transcopied to RF2000 by Nibbels

* Version in Testing-Phase!
* This Version is tested on RF2000 only, but should work on RF1000 as well.

* introduceing a new command M3901 to configure M3900
* M116 Temperature tolerance changed to 2°K
* Z_OVERRIDE_MAX = 1.0mm, instead of 0.5mm
* introduceing a new command M3902 to fix a HeatBeds Hole within the HBS Matrix.
* introduceing a new command M3903 to configure a very slow heat bed temperature decrease
* introduceing a new command M355 to configure MOS FET X19 (Wessix-Licht-Mod)
