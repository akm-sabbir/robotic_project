# robotic_project
automous robotic platform usig seeedruino board and atmega 128
So far I can remember about this project,

Seedruino board atmega128. loaded with FreeRTos operating system. Wrote three tasks. Ran them in Parallel.

Task1: read the input from servo motor. rotation per sec helps to determine the distance travelled.

Task2: IR sensor input. Helped to determine any obstacle close by distance. IR was mounted on the top of robotic platform.

Task3: Inertial Measurement Unit gave the Raw, Pitch and Yaw. Helped to determine the orientation and map generation.

How i programmed all the ports, used analog to digital conversion from IR sensor response, interrupt signals, even the board pin numbers, pin configuration that i can not remember. I also dont posses the board, i can get the schemitic from internet but i really dont have time to further research on it, as i dont work on embedded system anymore.
