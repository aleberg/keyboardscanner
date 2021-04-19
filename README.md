# Keyboard Scanner and Common Controllers to MIDI
MIDI implementation for a 25 key Fatar keybed with force sensitive resistor (ribbon).
This implementation, in addition to the original velocity, includes Aftertouch (Channel Pressure, from the FSR), 
Mod Wheel and Pitch Wheel continuous controls. Velocity is routed to CC71 (Volume) due to my specific usecase.

The Fatar keybed, FSR, pitch and mod wheels are connected to an Arduino Mega acting as keyboard scanner.

A Fatar 25 key keybed has a single ribbon cable. I used a protoboard hat for the Mega and connected the ribbon as so:

![ribbon](https://user-images.githubusercontent.com/10801301/115242664-64617280-a122-11eb-907b-26503e381b23.jpg)

## Diagram of one key
This scheme makes clear how to identify input and output pins. TODO: document Fatar matrix

![key](https://raw.githubusercontent.com/oxesoft/keyboardscanner/master/key_scheme.png)

## How to make your own MIDI controller
1) Disassemble the keyboard to have access to the flat cables (one, two or even three, depending on the number of keys and manufacturer);
2) Using a multimeter with the diode testing function selected, find out and understand the matrix, starting from the first key. Some keyboards have a logical pattern, some doesn't;
3) Connect the ribbon pins **directly** to the Arduino Mega (because it has enough pins to connect any keyboard with velocity). You **dont't** need to change anything in the keyboard circuit board;
4) Change the pins in the code (output_pins + input_pins), uncomment DEBUG_MIDI_MESSAGE and see the console output;
5) If the MIDI messages looks OK, comment DEBUG_MIDI_MESSAGE back and use some Serial<->MIDI Bridge like the excelent [Hairless](https://projectgus.github.io/hairless-midiserial/);
6) If everything goes well, consider turn you Arduino in a MIDI interface using [HIDUINO](https://github.com/ddiakopoulos/hiduino) or similar firmware.
7) Enjoy!
