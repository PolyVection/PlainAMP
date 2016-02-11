/*
 * plainamp.c  --  Simple cmdline application for controlling TAS5756 from userspace
 *
 * Copyright (C) 2016 Philip Voigt <info@polyvection.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */



#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "plainamp.h"

static bool isPAMP4d=false;
static bool isPAMP4c=false;

/** @brief A simple function to print an interger value in binary format*/
void bin(unsigned n)
{
    unsigned i;
    for (i = 1 << 7; i > 0; i = i / 2)
        (n & i)? printf("1"): printf("0");
    printf("\n");
}

/** @brief The check function to check reachability of PlainAMP
 *  Function reads from a register that returns a non-zero value
 *  @return returns the register value
 */
int checkSlave(int fd, char *fn, int adr){
    
    ioctl(fd, I2C_SLAVE_FORCE, adr);
    
    float data;
    char pg0[2]={0x00, 0x00};
    char regCHECK[1]={0x43};
    char bufferCHECK[1]={0};
    
    write(fd, pg0, 2);
    write(fd, regCHECK, 1);
    read(fd, bufferCHECK,1);
    
    if(!((int)bufferCHECK[0]==0))
        printf("Detected PlainAMP with address 0x%02x at %s!\n", adr,fn);
    else
        printf("No PlainAMP with address 0x%02x at %s!\n", adr,fn);
    
    return (int)bufferCHECK[0];
}

/** @brief The status/debug function
 *  Function reads several registers and prints them out.
 */
void getStatus(int fd, int adr){
    
    ioctl(fd, I2C_SLAVE_FORCE, adr);
    
    char regSTBY[1]={0x02};
    char regMUTE[1]={0x03};
    char regPLL[1]={0x04};
    char regSRC[1]={0x0D};
    char regMISS[1]={0x25};
    char regI2S[1]={0x28};
    char regDSP[1]={0x2B};
    char regSAMR[1]={0x5B};
    
    char bufSTBY[1]={0};
    char bufMUTE[1]={0};
    char bufPLL[1]={0};
    char bufSRC[1]={0};
    char bufMISS[1]={0};
    char bufI2S[1]={0};
    char bufDSP[1]={0};
    char bufSAMR[1]={0};
    
    write(fd, regSTBY, 1);
    read(fd, bufSTBY,1);
    write(fd, regMUTE, 1);
    read(fd, bufMUTE,1);
    write(fd, regPLL, 1);
    read(fd, bufPLL,1);
    write(fd, regSRC, 1);
    read(fd, bufSRC,1);
    write(fd, regMISS, 1);
    read(fd, bufMISS,1);
    write(fd, regI2S, 1);
    read(fd, bufI2S,1);
    write(fd, regDSP, 1);
    read(fd, bufDSP,1);
    write(fd, regSAMR, 1);
    read(fd, bufSAMR,1);
    
    printf("\n---------------------------------------------------------------");
    printf("\nNow printing out some register values of the AMP on 0x%02x.",adr);
    printf("\nPlease refer to the datasheet for how to interpret them.\n");
    printf("---------------------------------------------------------------\n\n");
    printf("STANDBY\t\t(REG 02):\t"); bin((int)bufSTBY[0]);
    printf("MUTE\t\t(REG 03):\t"); bin((int)bufMUTE[0]);
    printf("PLL\t\t(REG 04):\t"); bin((int)bufPLL[0]);
    printf("PLL SRC\t\t(REG 13):\t"); bin((int)bufSRC[0]);
    printf("IGNORE M\t(REG 37):\t"); bin((int)bufMISS[0]);
    printf("FMT + WORD\t(REG 40):\t"); bin((int)bufI2S[0]);
    printf("DSP MODE\t(REG 43):\t"); bin((int)bufDSP[0]);
    printf("S RATE\t\t(REG 91):\t"); bin((int)bufSAMR[0]);
    printf("\n");
}

/** @brief Function for getting the current volume
 *  Reads the values and converts them to dB values.
 *  @return returns the left channel volume as integer.
 */
int getVolume(int fd, int np){
    
    char regVL[1];
    char regVR[1];
    char bufferL[1]={0};
    char bufferR[1]={0};
    
    regVL[0]=0x3d;
    regVR[0]=0x3e;
    
    write(fd, regVL, 1);
    read(fd, bufferL,1);
    write(fd, regVR, 1);
    read(fd, bufferR,1);
    
    int volL = (int)bufferL[0];
    int volR = (int)bufferR[0];
    float dbL = 24;
    float dbR = 24;
    
    if(np==0){
        for (volL; volL>1; volL--){
            dbL=dbL-0.5;
        }
        
        for (volR; volR>1; volR--){
            dbR=dbR-0.5;
        }
        
        printf("\n");
        printf("A CHANNEL:\t%.1f dB\n", dbL);
        printf("B CHANNEL:\t%.1f dB\n", dbR);
        printf("\n");
    }
    
    return volL;
}

/** @brief Setting the volume
 *  Function convertes percent values to suitable register values
 *  and sends them to the device.
 */
void setVolume(int vol100, int vol255, int fd, int adr){
    
    ioctl(fd, I2C_SLAVE_FORCE, adr);
    
    int volume = 0;
    
    if (vol100!=0){
        printf("\n---------------------------------------------------------------\n");
        printf("Setting volume at 0x%02x to %d% / 124%\n",adr,vol100-1);
        printf("---------------------------------------------------------------\n");
        //we want to have a range from 0-124 for volume control
        float multi = 2.06;
        volume =(int)((vol100-1)*multi);
    }
    else volume=vol255;
    
    //stop at 255
    if (volume > 255) {
        volume = 255;
    }
    
    char bufL[10];
    char bufR[10];
    bufL[0] = 0x3d;
    bufR[0] = 0x3e;
    
    if(vol100!=0){
        //toggle all bits and store in buffer
        bufL[1] = ~volume;
        bufR[1] = ~volume;
    }
    else {
        bufL[1] = volume;
        bufR[1] = volume;
    }
    //write buffer to I2C dev
    write(fd, bufL, 2);
    write(fd, bufR, 2);
    getVolume(fd,0);
}

/** @brief Function for rais or decrease the volume by steps
 */
void setVolumeSwitch(int type, int fd, int adr){
    ioctl(fd, I2C_SLAVE_FORCE, adr);
    int currentVol = getVolume(fd,1);
    
    if (type==0){
        printf("\n---------------------------------------------------------------\n");
        printf("Reduced volume at 0x%02x by -0.5dB\n",adr);
        printf("---------------------------------------------------------------\n");
        setVolume(0,currentVol+1,fd,adr);
    }
    else{
        printf("\n---------------------------------------------------------------\n");
        printf("Raised volume at 0x%02x by 0.5dB\n",adr);
        printf("---------------------------------------------------------------\n");
        if(currentVol>0)setVolume(0,currentVol-1,fd,adr);
        else setVolume(0,currentVol,fd,adr);
    }
}

/** @brief Transmitting the sequences from plainamp.h
 */
void transmit_registers(cfg_reg *r, int n, int fd)
{
    int i = 0;
    while (i < n) {
        write(fd,(unsigned char *)&r[i], 2);
        i++;
    }
}

/** @brief Starting up the AMP and setting GPIO4 high
 */
void doUpstart(int fd, int adr){
    ioctl(fd, I2C_SLAVE_FORCE, adr);
    transmit_registers(initSeq, sizeof(initSeq)/sizeof(initSeq[0]),fd);
    printf("\n---------------------------------------------------------------\n");
    printf("PlainAMP at 0x%02x started!\n",adr);
    printf("---------------------------------------------------------------\n\n");
    setVolume(100,0,fd,adr);
}

/** @brief Muting the AMP and set GPIO0 low
 */
void doMute(int fd, int adr){
    ioctl(fd, I2C_SLAVE_FORCE, adr);
    transmit_registers(stopSeq, sizeof(stopSeq)/sizeof(stopSeq[0]),fd);
    printf("\n---------------------------------------------------------------\n");
    printf("PlainAMP at 0x%02x muted!\n", adr);
    printf("---------------------------------------------------------------\n\n");
}

/** @brief Load crossover config
 *
 *   For use in biamped setup / active speaker without passive crossover
 *
 *   SPK-A output is TWEETER
 *   SPK-B output is WOOFER
 *
 *   Crossover at 2800 Hz
 *   Samplerate up to 48 kHz
 */
void doActive(int fd, int adr){
    
    ioctl(fd, I2C_SLAVE_FORCE, adr);
    transmit_registers(BASeq, sizeof(BASeq)/sizeof(BASeq[0]),fd);
    printf("\n---------------------------------------------------------------\n");
    printf("PlainAMP at 0x%02x loading crossover config!\n",adr);
    printf("Crossover at 2800 Hz!\n");
    printf("SPK-A is WOOFER / SPK-B is TWEETER!\n");
    printf("---------------------------------------------------------------\n\n");
    doUpstart(fd, adr);
}

/** @brief Load SW output config
 *
 *   For use in high-res stereo setup with subwoofer out.
 *   Samplerate up to 192 kHz.
 *   Subwoofer output on GPIO1 / Highpass at 120 Hz.
 *
 */
void doSub(int fd, int adr){
    printf("\n---------------------------------------------------------------\n");
    printf("NOT IMPLEMENTED!\n",adr);
    printf("---------------------------------------------------------------\n\n");
}

/** @brief Only LEFT
 *
 *   Only use the left channel of the incomming signal
 *
 */
void doL(int fd, int adr){
    ioctl(fd, I2C_SLAVE_FORCE, adr);
    transmit_registers(setL, sizeof(setL)/sizeof(setL[0]),fd);
    printf("\n---------------------------------------------------------------\n");
    printf("PlainAMP at 0x%02x is LEFT channel!\n",adr);
    printf("---------------------------------------------------------------\n\n");
}

/** @brief Only RIGHT
 *
 *   Only use the right channel of the incomming signal
 *
 */
void doR(int fd, int adr){
    ioctl(fd, I2C_SLAVE_FORCE, adr);
    transmit_registers(setR, sizeof(setR)/sizeof(setR[0]),fd);
    printf("\n---------------------------------------------------------------\n");
    printf("PlainAMP at 0x%02x is RIGHT channel!\n",adr);
    printf("---------------------------------------------------------------\n\n");

}

/** @brief STEREO to MONO
 *
 *   An incomming STEREO signal will be mixed to MONO
 *
 */
void doMono(int fd, int adr){
    ioctl(fd, I2C_SLAVE_FORCE, adr);
    transmit_registers(setM, sizeof(setM)/sizeof(setM[0]),fd);
    printf("\n---------------------------------------------------------------\n");
    printf("PlainAMP at 0x%02x will mix STEREO to MONO!\n",adr);
    printf("---------------------------------------------------------------\n\n");
}

/** @brief RESET
 *
 *   Will reset PlainAMP and load the default settings.
 *
 */
void doReset(int fd, int adr){
    ioctl(fd, I2C_SLAVE_FORCE, adr);
    transmit_registers(resetSeq, sizeof(resetSeq)/sizeof(resetSeq[0]),fd);
    printf("\n---------------------------------------------------------------\n");
    printf("PlainAMP at 0x%02x was resetted!\n",adr);
    printf("---------------------------------------------------------------\n\n");
}

/** @brief Main function
 */
int main(int argc, char *argv[])
{
    extern char *optarg;
    extern int optind;
    
    int c;
    int err = 0;
    int startflag=0;
    int muteflag=0;
    int stopflag=0;
    int volflag=0;
    int mVolFlag=0;
    int lVolFlag=0;
    int biampFlag=0;
    int subFlag=0;
    int debugflag=0;
    
    int volume = 100;
    
    while ((c = getopt(argc, argv, "weupdmlv:")) != -1)
        switch (c) {
                
            case 'u':
                startflag = 1;
                break;
            case 'p':
                muteflag = 1;
                break;
            case 'v':
                volflag = 1;
                volume = (int) strtol(optarg, NULL, 10);
                break;
            case 'd':
                debugflag = 1;
                break;
            case 'm':
                mVolFlag = 1;
                break;
            case 'l':
                lVolFlag = 1;
                break;
            case 'e':
                biampFlag = 1;
                break;
            case 'w':
                subFlag = 1;
                break;
                
                
            case '?':
                printf("\nUSAGE:\n");
                printf("-u -> start\n");
                printf("-p -> mute\n");
                printf("-v [volume number from 0 to 124] -> set volume\n");
                printf("-m -> raise the volume by 0.5dB\n");
                printf("-l -> reduce the volume by 0.5dB\n");
                printf("-d -> print out some register values\n");
                printf("-e -> biamping mode @ 2800Hz autodetect MONO/STEREO\n");
                printf("-w -> subwoofer output mode @ 120Hz\n\n");
                err = 1;
                break;
        }
    
    printf("\n***************************************************************\n");
    printf("**************** PlainAMP CONTROL PROGRAM ****************\n");
    printf("***************************************************************\n\n");
    
    char *fileName = "/dev/i2c-1";
    int file;
    int  address = 0x4d;
    char buf[10];
    
    if ((file = open(fileName, O_RDWR)) < 0) {					// Open port for reading and writing
        printf("Failed to open i2c port\n");
        exit(1);
    }
    
    if (ioctl(file, I2C_SLAVE_FORCE, address) < 0) {		// Set the port options and set the address
        printf("Unable to get bus access to talk to slave\n");  // of the device we wish to speak to
        exit(1);                                                // I2C_SLAVE_FORCE if device is already used by kernel driver
    }
    
    int ret = 0;
    if (checkSlave(file, fileName, 0x4d)>0){ isPAMP4d=true; ret++;};
    if (checkSlave(file, fileName, 0x4c)>0){ isPAMP4c=true; ret++;};
    
    if(ret==0){
        printf("\n---------------------------------------------------------------");
        printf("\nUnable to talk to PlainAMP on 0x4d or 0x4c!\n");
        printf("Please check your connections and your I2C bus!\n");
        printf("---------------------------------------------------------------\n\n");
        printf("***************************V-1.2*******************************\n\n");
        exit(1);
    }
    
    if (startflag) {
        if(isPAMP4c){
            doReset(file,0x4c);
            doUpstart(file,0x4c);
        }
        if(isPAMP4d){
            doReset(file,0x4d);
            doUpstart(file,0x4d);
        }
    }
    
    else if (muteflag) {
        if(isPAMP4c)doMute(file,0x4c);
        if(isPAMP4d)doMute(file,0x4d);
    }
    
    if (volflag) {
        if(isPAMP4c)setVolume(volume+1,0,file,0x4c);
        if(isPAMP4d)setVolume(volume+1,0,file,0x4d);
    }
    
    if (lVolFlag) {
        if(isPAMP4c)setVolumeSwitch(0,file,0x4c);
        if(isPAMP4d)setVolumeSwitch(0,file,0x4d);
    }
    
    if (mVolFlag) {
        if(isPAMP4c)setVolumeSwitch(1,file,0x4c);
        if(isPAMP4d)setVolumeSwitch(1,file,0x4d);
    }
    
    if (debugflag) {
        if(isPAMP4c)getStatus(file,0x4c);
        if(isPAMP4d)getStatus(file,0x4d);
    }
    
    if (biampFlag) {
        if(isPAMP4c && !isPAMP4d){
            printf("\n---------------------------------------------------------------\n");
            printf("Loading MONO biamp config!\n");
            printf("---------------------------------------------------------------\n\n");
            doActive(file,0x4c);
            doMono(file,0x4c);
        }
        if(isPAMP4d && !isPAMP4c){
            printf("\n---------------------------------------------------------------\n");
            printf("Loading MONO biamp config!\n");
            printf("---------------------------------------------------------------\n\n");
            doActive(file,0x4d);
            doMono(file,0x4d);
        }
        if(isPAMP4d && isPAMP4c){
            printf("\n---------------------------------------------------------------\n");
            printf("Loading STEREO biamp config!\n");
            printf("---------------------------------------------------------------\n\n");
            doActive(file,0x4d);
            doActive(file,0x4c);
            doR(file,0x4d);
            doL(file,0x4c);
        }
    }
    
    if (subFlag) {
        if(isPAMP4c)doSub(file,0x4c);
        if(isPAMP4d)doSub(file,0x4d);
    }
    
    printf("***************************V-1.2*******************************\n\n");
    exit(EXIT_SUCCESS);
}
