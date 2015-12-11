Howto: Getting Started With Xadow

Getting Start With Xadow Main Board on Mac is more easier than in Window7. You only need to change some config files on Arduino IDE. Files with path are list below.(Arduino-1.0.1 maybe different depends on your version)
Arduino-1.0.1/hardware/arduino/cores/arduino/USBCore.cpp
Arduino-1.0.1/hardware/arduino/boards.txt
Download our new files and replace:
new USBCore.cpp
new boards.txt
For Mac Lion user, your Arduino folder may not list under Document/Arduino, open your Application folder, find "Arduino", right click , then "show package content".
There will be a path way to "hardware". Find board.txt, replace it(or you can back up original file by rename it to board-old.txt).

Install driver also