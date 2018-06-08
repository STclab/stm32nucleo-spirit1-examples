
Instructions (linux or cygwin under windows):

1) make TARGET=stm32nucleo-spirit1 BOARD=ids01a4/ids01a5 serial-sniffer.bin
2) Connect a NUCLEO-L152RE board (with X-NUCLEO-IDS01Ax expansion board) to the PC
3) Flash it wiht serial-sniffer.bin firmware (i.e. copy the file on the relative mass storage device)
4) Run the following command chain:

sudo serialdump-linux  -b115200 /dev/ttyACMx | ./convert-to-binary | wireshark -k -i -

serialdump-windows.exe -b115200 /dev/ttySz   | ./convert-to-binary | wireshark.exe -k -i -

(mind the trailing '-') where ttyACMx or ttySz is the device used by your board.

Prerequisites:
- serialdump-* utilities can be found in contiki/tools/stm32w folder
- To use the convert-to-binary utility you need PERL.
- recent version of wireshark
- serialdump-* utilities and wireshark must be in PATH, or you have to use the full path in the above commands.

Troubleshooting:
- under windows you may need to recompile serialdump.c in order to rebuild serialdump-windows.exe
- under windows you may need to hadrcode the baudrate (115200) in the serialdump.c code
- under windows you may need to use the full path for wireshark.exe if it is in a path with spaces, use '\' as escape char before spaces and parenthesis.


