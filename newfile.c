
#include <p32xxxx.h>
#include <plib.h>
#include "dsplib_dsp.h"
#include "fftc.h"

/* SYSCLK = 8MHz Crystal/ FPLLIDIV * FPLLMUL/ FPLLODIV = 80MHz
PBCLK = SYSCLK /FPBDIV = 80MHz*/
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1

/* Input array with 16-bit complex fixed-point twiddle factors.
 this is for 16-point FFT. For other configurations, for example 32 point FFT,
 Change it to fft16c32*/
#define fftc fft16c16

/* defines the sample frequency*/
#define SAMPLE_FREQ 2048

/* number of FFT points (must be power of 2) */
#define N 16
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
#define SegA LATGbits.LATG12
#define SegB LATGbits.LATG13
#define SegC LATGbits.LATG14
#define SegD LATGbits.LATG15
#define SegE LATDbits.LATD7
#define SegF LATDbits.LATD1
#define SegG LATDbits.LATD9
// Display selection. 0 = right, 1 = left (Cathode)
#define DispSel LATCbits.LATC1

//key pad
#define C4 PORTBbits.RB0
#define C3 PORTBbits.RB1
#define C2 PORTBbits.RB2
#define C1 PORTBbits.RB3
#define R4 LATBbits.LATB4
#define R3 LATBbits.LATB5
#define R2 LATBbits.LATB8
#define R1 LATBbits.LATB9

int flash = 0b0;
int time = 0;
int flash_time = 30;
int i=0;
int key =-1;
int mode=1;
int password=-1;
int GN=-1;
int counter=0;
int A;
int B;
int C;
int D;


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
    0b0111000, //L 15
    0b1110111, //A 16


};

void displayDigit(unsigned char value, int a) {

    SegA = value & 1;
    SegB = (value >> 1) & 1;
    SegC = (value >> 2) & 1;
    SegD = (value >> 3) & 1;
    SegE = (value >> 4) & 1;
    SegF = (value >> 5) & 1;
    SegG = (value >> 6) & 1;
    DispSel = a;
}

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

void showNumber(int digit, int a) {
    displayDigit(SSD_number[digit], a);
}

void showNumber1(int digit, int a) {
    displayDigit1(SSD_number[digit], a);
}
/* log2(16)=4 */
int log2N = 4;

/* Input array with 16-bit complex fixed-point elements. */
/* int 16c is data struct defined as following:
  typedef struct
{
        int16 re;
        int16 im;
} int16c; */
int16c sampleBuffer[N];

/* intermediate array */
int16c scratch[N];

/* intermediate array holds computed FFT until transmission*/
int16c dout[N];

/* intermediate array holds computed single side FFT */
int singleSidedFFT[N];

/* array that stores the corresponding frequency of each point in frequency domain*/
short freqVector[N];

/* indicates the dominant frequency */
int freq = 0;

// Function definitions
int computeFFT(int16c *sampleBuffer);
void __ISR(_TIMER_5_VECTOR, ipl4) _T5Interrupt(void)
{
	// perform shift


	IFS0CLR = 0x100000;   // Clear Timer5 interrupt status flag (bit 20)
}
void __ISR(_TIMER_4_VECTOR, ipl3) _T4Interrupt(void)
{
	// perform shift


	IFS0CLR = 0x100000;   // Clear Timer5 interrupt status flag (bit 20)
}
void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNoticeHandler(void) {
    //INTDisableInterrupts();
    PORTB;
    IEC1CLR = 0x1;
    PORTB;

    //showNumber(0b1);

    for (i = 0; i < 10000; i++); //deboundce
    R1 = R2 = R3 = R4 = 0;
    if (C1 & C2 & C3 & C4) {
        key = -1;
        //return;
    } else if (key == -1) {
        R1 = 0;
        R2 = 1;
        R3 = 1;
        R4 = 1;
        if (C1 == 0) {
            key = 1;

        } else if (C2 == 0) {
            key = 2;

        } else if (C3 == 0) {
            key = 3;

        } else if (C4 == 0) {
            key = 10;

        } else {

            R2 = 0;
            R1 = 1;
            R3 = 1;
            R4 = 1;
            if (C1 == 0) {
                key = 4;

            } else if (C2 == 0) {
                key = 5;

            } else if (C3 == 0) {
                key = 6;

            } else if (C4 == 0) {
                key = 11;

            } else {

                R3 = 0;
                R2 = 1;
                R1 = 1;
                R4 = 1;
                if (C1 == 0) {
                    key = 7;

                } else if (C2 == 0) {
                    key = 8;

                } else if (C3 == 0) {
                    key = 9;

                } else if (C4 == 0) {
                    key = 12;

                } else {

                    R4 = 0;
                    R2 = 1;
                    R3 = 1;
                    R1 = 1;
                    if (C1 == 0) {
                        key = 0;

                    } else if (C2 == 0) {
                        key = 15;

                    } else if (C3 == 0) {
                        key = 14;


                    } else if (C4 == 0) {
                        key = 13;

                    }
                }
            }
        }
//        check();
    }
    //decode which key was pressed
    PORTB;

    R1 = R2 = R3 = R4 = 0;
    PORTB;
    IFS1CLR = 0x1;


    //for (i = 0; i < 100000; i++); //deboundce
    IEC1SET = 0x1;
    //INTEnableInterrupts();
}

void check (int a ){
    if (mode ==1){//mode 1............................................setting password
        if (key >= 0 && key <= 9) {
            if (password == -1) {
                password = key;


            } else if (password >= 0 && password <= 9) {

                password = password * 10 + key;

            }else if (password >=10 && password <=99){
                 password = password * 10 + key;
            }
            //need to display some thing in somewhere
        } else if (key == 12) { //clear
            password = 0;
        } else if (key == 13) { //delete
            password = password / 10;
        } else if (key == 14) { //enter
            if (password <= 999 && password >= 300) {
                mode = 2;
            } else {
                mode =1;
                password=0;
            }
        }

    }// end of mode 1
    else if (mode ==2){// Unlock mode..........................................
        if (counter >=10000){// loop of time 1 s
            mode =1;
            password=0;
        }
        else {//less 1s
            mode =3;//lock mode
            D=15; //L
            GN=0;
        }
    }// end of Unlock mode
    else if (mode =3){ //Locked mode...........................................
        if (counter>=5000){
            mode =4;
            A=B=C=D=16; //AAAA
            GN=-1;
        }else{// keypad input GN
            if (key >= 0 && key <= 9) {
            if (GN == -1) {
                GN = key;
            } else if (GN >= 0 && GN <= 9) {
                GN = GN * 10 + key;
            }else if (GN >= 10 && GN <= 99) {
                GN = GN * 10 + key;
            }
            //need to display some thing in somewhere
        } else if (key == 12) { //clear
            GN = -1;
        } else if (key == 13) { //delete
            GN = GN / 10;
        } else if (key == 14) { //enter
            if (GN <= 999 && GN >= 300) {
                if (GN==password){
                    GN=-1;
                    mode =2;
                }
            } else {
                GN = -1;
                mode = 4;
                 A=B=C=D=16; //AAAA
            }
        }
        }
    }// end of mode 3
    else if (mode =4){

    }
}

main() {
    AD1PCFG = 0xF7FF; // all PORTB = digital but RB11 = analog
    AD1CON1 = 0; // automatic conversion after sampling
    AD1CHS = 0x000B0000; // Connect RB11/AN11 as CH0 input
    AD1CSSL = 0; // no scanning required
    AD1CON2 = 0; // use MUXA, AVss/AVdd used as Vref+/-
    AD1CON3 = 0x3; // Tad = 128 x Tpb, Sample time = 31 Tad
    AD1CON1bits.ADON = 1; // turn on the ADC
    TRISB = 0xF;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    TRISF = 0;
    TRISG = 0;

    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;
    PORTF = 0x00;
    PORTG = 0x00;
    int i;
    /* assign values to sampleBuffer[] */
    for (i = 0; i < N; i++) {
        sampleBuffer[i].re = i;
        sampleBuffer[i].im = 0;
    }
    /* compute the corresponding frequency of each data point in frequency domain*/
    for (i = 0; i < N / 2; i++) {
        freqVector[i] = i * (SAMPLE_FREQ / 2) / ((N / 2) - 1);
    }


    while (1) {
        /* get the dominant frequency */
        freq = freqVector[computeFFT(sampleBuffer)];
        AD1CON1bits.SAMP = 1; // 1. start sampling
        for (TMR1 = 0; TMR1 < 100; TMR1++); //2. wait for sampling time
        AD1CON1bits.SAMP = 0; // 3. start the conversion
        while (!AD1CON1bits.DONE); // 4. wait conversion complete
        //return ADC1BUF0; // 5. read result
        time++;
        if (time > flash_time) {
            time = 0;
            flash++;

        }
        flash = (flash & 1);
        ;
        if (flash == 1) {
            showNumber(ADC1BUF0 / 1000 % 10, flash);
            showNumber1(ADC1BUF0 / 10 % 10, flash);
        } else {
            showNumber(ADC1BUF0 / 100 % 10, flash);
            showNumber1(ADC1BUF0 / 1 % 10, flash);
        }
    }
}

int computeFFT(int16c *sampleBuffer) {
    int i;
    int dominant_freq = 1;

    /* computer N point FFT, taking sampleBuffer[] as time domain inputs
     * and storing generated frequency domain outputs in dout[] */
    mips_fft16(dout, sampleBuffer, fftc, scratch, log2N);

    /* compute single sided fft */
    for (i = 0; i < N / 2; i++) {
        singleSidedFFT[i] = 2 * ((dout[i].re * dout[i].re) + (dout[i].im * dout[i].im));
    }

    /* find the index of dominant frequency, which is the index of the largest data points */
    for (i = 1; i < N / 2; i++) {
        if (singleSidedFFT[dominant_freq] < singleSidedFFT[i])
            dominant_freq = i;
    }

    return dominant_freq;
}
