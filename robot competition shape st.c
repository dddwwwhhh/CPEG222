
/*
 * File:   Project 4.c
 * Author: Cheng Zheng
 *
 * Created on May 5, 2015, 11:06 PM
 */


#include <p32xxxx.h>
#include <plib.h>
#include "dsplib_dsp.h"
#include "fftc.h"

/* SYSCLK = 8MHz Crystal/ FPLLIDIV * FPLLMUL/ FPLLODIV = 80MHz
PBCLK = SYSCLK /FPBDIV = 10MHz*/
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_8
#define Btn1    PORTAbits.RA6
#define Btn2	PORTAbits.RA7
#define SLed1   LATBbits.LATB10
#define SLed2   LATBbits.LATB11
#define SLed3   LATBbits.LATB12
#define SLed4   LATBbits.LATB13
#define C1 PORTAbits.RA9
#define C2 PORTAbits.RA10
#define C3 PORTDbits.RD12
#define C4 PORTCbits.RC4


#define SegA1 LATEbits.LATE0
#define SegB1 LATEbits.LATE1
#define SegC1 LATEbits.LATE2
#define SegD1 LATEbits.LATE3
#define SegE1 LATGbits.LATG9
#define SegF1 LATGbits.LATG8
#define SegG1 LATGbits.LATG7
// Display selection. 0 = right, 1 = left (Cathode)
#define DispSel1 LATGbits.LATG6
//JC and JD
//#define SegA LATGbits.LATG12
//#define SegB LATGbits.LATG13
//#define SegC LATGbits.LATG14
//#define SegD LATGbits.LATG15
//#define SegE LATDbits.LATD7
//#define SegF LATDbits.LATD1
//#define SegG LATDbits.LATD9
//// Display selection. 0 = right, 1 = left (Cathode)
//#define DispSel LATCbits.LATC1


int mode = 1;
int buttonLock1 = 0;
int buttonLock2 = 0;
int i;
int time = 0;
int flash_time = 150;
int A;
int B;
int C;
int D;
int flash = 0b0;
int aaa = 0;
int zADC = 400;
int oc = 0;
int premode;
int premovement;
int l;
int r;
int ml;
int mr;
int black = 1;
int white = 0;
int counter = 0;
int dis = 0;
int discounter = 0;
int movement;
int ccc;

unsigned char SSD_number[] = {
    0b0111111, //0
    0b0000110, //1
    0b1011011, //2
    0b1001111, //3
    0b1100110, //4
    0b1101101, //5
    0b1111101, //6
    0b0000111, //7
    0b1111111, //8
    0b1101111, //9
    0b0000000, //clear 10
    0b1111001, //E 11
    0b0111000, //L 12
    0b1110110, //H 13
    0b0111001, //C 14


};

void displayDigit1(unsigned char value, int a) {

    SegA1 = value & 1;
    SegB1 = (value >> 1) & 1;
    SegC1 = (value >> 2) & 1;
    SegD1 = (value >> 3) & 1;
    SegE1 = (value >> 4) & 1;
    SegF1 = (value >> 5) & 1;
    SegG1 = (value >> 6) & 1;
    DispSel1 = a;
}

void sets(int value) {
    SLed1 = (value & 0b1000) >> 3;
    SLed2 = (value & 0b0100) >> 2;
    SLed3 = (value & 0b0010) >> 1;
    SLed4 = (value & 0b0001);
}

void showNumber1(int digit, int a) {
    displayDigit1(SSD_number[digit], a);
}

int readADC() {
    AD1CON1bits.SAMP = 1; // 1. start sampling
    while (!AD1CON1bits.DONE); // 3. wait conversion complete
    return ADC1BUF0; // 3. read conversion result
}

int main() {
    TRISA = 0b11011000000;
    TRISB = 0b00010; //0x080f;   1000 0000 1111
    TRISC = 0b10000;
    TRISD = 0b1000000000000;
    TRISE = 0;
    TRISF = 0;
    TRISG = 0;

    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;
    PORTF = 0x00;
    PORTG = 0x00;
    PORTB = 0x00;



    AD1PCFG = 0xFFFD; // all PORTB = digital but RB1 = analog (PIN 5 (PIN 2/JJ on MIC)
    AD1CON1 = 0x00E0; // automatic conversion after sampling
    AD1CHS = 0x00010000; // Connect RB1/AN1 as CH0 input
    AD1CSSL = 0; // no scanning required
    AD1CON2 = 0; // use MUXA, AVss/AVdd used as Vref+/-
    AD1CON3 = 0x1F03; // Tad = 512 x Tpb, Sample time = 31 Tad 1FFF
    AD1CON1bits.ADON = 1; // turn on the ADC



    T2CON = 0x000; // Configure Timer3 for a prescaler of 0
    T2CONSET = 0x60; //prescale 64
    TMR2 = 0;
    OC2CON = 0;
    PR2 = 3124; // Set period 3,124 event frq =50HZ =20ms   3905 for 40Hz 25 ms, 2603 for 60 Hz 16.666ms



    IPC2SET = 0x18; //Timer level 6 sub 0
    IFS0CLR = 0x100; // Clear the Timer 3 interrupt flag 800
    IEC0SET = 0x100; // Enable Timer 3 interrupt 800

    OC2CON = 0x0000; // Turn off OC2 while doing setup. bit 3 1 timer 3 0 timer2
    OC2CON = 0b0110; // Compare event toggles OCx pin 0000 1011 timer 3
    OC2R = 231; // Initialize Compare Register 1
    OC2RS = 231;
    IFS0CLR = 0x400; //clear flag OC2
    IEC0SET = 0x400;
    IPC2SET = 0b001000000000000000000; // OC2 level 7 sub3

    OC3CON = 0x0000; // Turn off OC3 while doing setup. bit 3 1 timer 3 0 timer2
    OC3CON = 0b0110; // Compare event toggles OCx pin 0000 1011 timer 3
    OC3R = 235; // Initialize Compare Register 1
    OC3RS = 235;
    IFS0CLR = 0x4000; //clear flag OC3
    IEC0SET = 0x4000;
    IPC3SET = 0b001000000000000000000; // OC3 level 7 sub3 1F0000


    T2CONSET = 0x8000; // Enable Timer 2
    OC2CONSET = 0x8000; // Enable OC2
    OC3CONSET = 0x8000; //Enable OC3
    // Example code for Output Compare 1 ISR:
// 4. Enable vector interrupt
    INTEnableSystemMultiVectoredInt();

    for (i = 0; i < 150000; i++);
    while (1) {
        int ADC = readADC();
        SLed1 = (C1 + 1) & 0b1;
        SLed2 = (C2 + 1) & 0b1;
        SLed3 = (C3 + 1) & 0b1;
        SLed4 = (C4 + 1) & 0b1;
        mr = (C3 + 1) & 0b1;
        ml = (C2 + 1) & 0b1;
        l = (C1 + 1) & 0b1;
        r = (C4 + 1) & 0b1;
        //        OC2RS = (156-10); //right
        //        OC3RS = (312-19); //left 285

        /*
        if (Btn1 == 1 && buttonLock1 == 0) {
            buttonLock1 = 1;
            for (i = 0; i < 100; i++);
            if (OC2R < 340 || OC3R > 120) {

                OC2RS = OC2RS - 1; // front right
                //           OC3RS=OC3RS-1;
                oc++;
            }
        } else if (Btn2 == 1 && buttonLock2 == 0) {
            buttonLock2 = 1;
            for (i = 0; i < 100; i++);
            if (OC2R > 120 || OC3R < 340) {
                //         OC2RS=OC2RS-1;
                //       OC3RS=OC3RS+1;
                OC3RS = OC3RS + 1; //front left
                oc--;
            }
        } else if (!Btn1 && buttonLock1) {
            buttonLock1 = 0;
        } else if (!Btn2 && buttonLock2) {
            buttonLock2 = 0;
        }



         */
        time++;
        if (time > flash_time) {
            time = 0;
            flash++;

        }
        //  aaa=abs(oc);
        flash = (flash & 1);

        if (flash == 1) {

            showNumber1(dis / 10 % 10, flash);
        } else {

            showNumber1(dis % 10, flash);
        }


        if (mode == 1) {




            if (ADC > 450 || (Btn1 == 1 && buttonLock1 == 0)) {
                zADC = ADC;
                buttonLock1 = 1;
                for (i = 0; i < 100; i++);
                mode = 2;
                // AD1CON1bits.ADON = 0; //turn off ADC

            }
        } else if (!Btn1 && buttonLock1) {
            buttonLock1 = 0;
        } else if (!Btn2 && buttonLock2) {
            buttonLock2 = 0;
        } else if (mode == 2) {
            if (l == 1 && ml == 1 && mr == 1 && r == 1){
            OC2RS = 156;
            OC3RS = 293;}
            //for (i = 0; i < 100000; i++);
            else if (l==0 || ml ==0 || mr==0 || r==0){
            mode = 3;}
        } else if (mode == 3) {
            //OC2RS right
            //OC3RS left
            if (l == 1 && ml == 1 && mr == 1 && r == 1) { //xxxx
                movement = 1;
               // counter++;
               // OC2RS = 156;
                //OC3RS = 312;
                //if (counter > 1000) {
                    OC2RS = 231; //stop
                    OC3RS = 235; //stop

                    mode = 1;
                //}
            } else if (l == 1 && ml == 1 && mr == 1 && r == 0) { //xxxo
                movement = 2;
                OC3RS = 235; //left front?
                OC2RS = 156; //right front?

            } else if (l == 1 && ml == 1 && mr == 0 && r == 1) {//xxox
                movement = 3;
                OC3RS = 156; //left front?
                OC2RS = 156; //right front?

            } else if (l == 1 && ml == 1 && mr == 0 && r == 0) { //xxoo
                movement = 4;
                OC3RS = 156; //left front
                OC2RS = 156; //right front?

            } else if (l == 1 && ml == 0 && mr == 1 && r == 1) { //xoxx
                movement = 5;
                OC3RS = 312; //left front?
                OC2RS = 312; //right front

            } else if (l == 1 && ml == 0 && mr == 1 && r == 0) { //xoxo
                movement = 6;
                OC3RS = 293; //left front?
                OC2RS = 156; //right front?

            } else if (l == 1 && ml == 0 && mr == 0 && r == 1) { //xoox
                movement = 7;
                OC3RS = 293; //left front?
                OC2RS = 156; //right front?

            } else if (l == 1 && ml == 0 && mr == 0 && r == 0) {//xooo
                movement = 8;
                OC3RS = 235; //left front
                OC2RS = 156; //right front?

            } else if (l == 0 && ml == 1 && mr == 1 && r == 1) {//oxxx
                movement = 9;
                OC3RS = 312; //left front?
                OC2RS = 231; //right front?

            } else if (l == 0 && ml == 1 && mr == 1 && r == 0) {//oxxo
                movement = 10;
                OC3RS = 293; //left front?
                OC2RS = 156; //right front?

            } else if (l == 0 && ml == 1 && mr == 0 && r == 1) {//oxox
                movement = 11;
                OC3RS = 293; //left front?
                OC2RS = 156; //right front?

            } else if (l == 0 && ml == 1 && mr == 0 && r == 0) {//oxoo
                movement = 12;
                OC3RS = 156; //left front
                OC2RS = 156; //right front?

            } else if (l == 0 && ml == 0 && mr == 1 && r == 1) {//ooxx
                movement = 13;
                OC3RS = 312; //left front?
                OC2RS = 312; //right front

            } else if (l == 0 && ml == 0 && mr == 1 && r == 0) {//ooxo
                movement = 14;
                OC3RS = 312; //left front?
                OC2RS = 312; //right front

            } else if (l == 0 && ml == 0 && mr == 0 && r == 1) {//ooox
                movement = 15;
                OC3RS = 312; //left front?
                OC2RS = 231; //right front

            } else if (l == 0 && ml == 0 && mr == 0 && r == 0) {//oooo
                movement = 16;
                OC3RS = 293; //left front?
                OC2RS = 156; //right front?

            }
        }






    }
}

void __ISR(_TIMER_2_VECTOR, ipl6) Timer2Handler(void) {
    if ((OC3RS>=270&&OC3RS<=320)&&(OC2RS>=140&&OC2RS<=170)){
    discounter++;}
    else if(((OC3RS>=250&&OC3RS<=320)||(OC2RS>=140&&OC2RS<=170))){
        ccc++;
        if (ccc>=2){
        discounter=discounter+1;
        ccc=0;}
    }
    if (discounter >= 10) { //1133364.561
        dis++;
        if (dis >= 100) {
            dis = 0;
        }
        discounter = 0;
    }

    IFS0CLR = 0x00000100; // Be sure to clear the Timer 2 interrupt status

}