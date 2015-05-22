/*
 * MenuStrings.c
 *
 *  Created on: May 22, 2015
 *      Author: jad140230
 */



char clearScreen[] = "\033[2J\033[0;0H";
char menuDivider[] = "-----------------------------------------";
char menuGoBack[] = "\r\n0) Go Back\r\n";
char nope[] = "That's not a number. Press any key to continue.";
int read = 0;


char pwmString1[] = "\nPWM Menu (outputs on GPIO 0-3)";
char pwmString2[] = "1) Toggle PWM:           OFF ";
char pwmString3[] = "2) Duty Cycle:           50% ";
char pwmString4[] = "3) Frequency :      10000 Hz ";
char pwm_is_kill[] = "PWM is not enabled. Please try again.";
char* pwmMenu[6] = {pwmString1, menuDivider, pwmString2, pwmString3, pwmString4, menuGoBack};

char adcString1[] = "\nADC Menu";
char adcString2[] = "1) Begin conversion. Press any key to stop it\r\n   and return to the main menu.";
char adcString3[] = "Current value: ";
char* adcMenu[3] = {adcString1, menuDivider, adcString2};

char gpioString1[] = "\n GPIO Menu";
char gpioString2[] = "Press the indicated keys to toggle the LEDs:\r\n";
char gpioString3[] = "1) LED0: OFF\r\n2) LED1: OFF\r\n3) LED2: OFF\r\n4) LED3: OFF";
char* gpioMenu[5] = {gpioString1, menuDivider, gpioString2, gpioString3, menuGoBack};

char menuString1[] = "\nF28027 Main Menu";
char menuString2[] = "1) PWM";
char menuString3[] = "2) ADC";
char menuString4[] = "3) GPIO";
char* mainMenu[5] = {menuString1, menuDivider, menuString2, menuString3, menuString4};


