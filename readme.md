
# SMDT-PC
A minimal "socat like" communication tool for [SMDT](https://github.com/SweMonkey/smdt)<br>
What does it do? It acts as a bridge between a serial tty device or a socket and a remote server.<br>

## Building SMDT-PC
Navigate to the src folder and run this command in your terminal:<br>
`gcc main.c -o smdtpc`<br>
<br>

## Using SMDT-PC
SMDT-PC can be used in three ways;<br><br>
Piping raw traffic between a Mega Drive and a remote server:<br>
`./smdtpc -local=/dev/ttyS4 -remote=127.0.0.1:6969`<br><br>
Use the xport "communication protocol", this allow a Mega Drive to talk to and control network connections on the PC side:<br>
`./smdtpc -xport /dev/ttyS4`<br><br>
Same as above, but connects to a socket instead of a serial tty device.<br>
Use this method when running an emulated Mega Drive on [BlastEm](https://www.retrodev.com/blastem/nightlies/):<br>
`./smdtpc -xportsock ./socketfile.sock`<br><br>
<br>
