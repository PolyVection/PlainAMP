# PlainAMP

Configuration program for PlainAMP to make use of the embedded DSP.

#INSTALLATION
git clone https://github.com/PolyVection/PlainAMP.git

cd PlainAMP

*only for Banana Pi users*: 
edit plainamp.c and replace “/dev/i2c-1” with “/dev/i2c-2”

gcc -o plainamp plainamp.c

#USAGE
./plainamp -u = startup command:    

./plainamp -p = mute command:       

./plainamp -m = volume + 0.5dB:     

./plainamp -l = volume - 0.5dB:     

./plainamp -d = status/debug:       

./plainamp -v 01 (where 01 is the volume in percent from 0 to 124) 

./plainamp -e = load the biamping / active crossover mode.

#BIAMPING / ACTIVE CROSSOVER
The -e argument will activate the embedded DSP of the TAS5756 IC.
Use this if you have one PlainAMP and would like to use it to power a tweeter and a woofer in MONO configuration.
All incomming stereo signals will be mixed to mono internally.

If you have connected two PlainAMPs to your I2C bus (address 0x4c and 0x4d) the program will autodetect this configuration and set one to be the RIGHT and the other to be the LEFT channel.
