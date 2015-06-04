//#############################################################################

#include "DSP28x_Project.h"     // DSP28x Headerfile

#include "f2802x_common/include/adc.h"
#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/wdog.h"

#define CONV_WAIT 1L //Micro-seconds to wait for ADC conversion. Longer than necessary.

int16_t current; //raw current sensor reading

CLK_Handle myClk;
FLASH_Handle myFlash;
GPIO_Handle myGpio;
PIE_Handle myPie;

void main()
{

    ADC_Handle myAdc;
    CPU_Handle myCpu;
    PLL_Handle myPll;
    WDOG_Handle myWDog;

    // Initialize all the handles needed for this application
    myAdc = ADC_init((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));
    myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    myCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
    myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    myPie = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
    myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    myWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));

    // Perform basic system initialization
    WDOG_disable(myWDog);
    CLK_enableAdcClock(myClk);
    (*Device_cal)();

    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc(myClk, CLK_OscSrc_Internal);

    // Setup the PLL for x12 /2 which will yield 60Mhz = 10Mhz * 12 / 2
    PLL_setup(myPll, PLL_Multiplier_12, PLL_DivideSelect_ClkIn_by_2);

    // Disable the PIE and all interrupts
    PIE_disable(myPie);
    PIE_disableAllInts(myPie);
    CPU_disableGlobalInts(myCpu);
    CPU_clearIntFlags(myCpu);
    // If running from flash copy RAM only functions to RAM
    #ifdef _FLASH
        memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
    #endif

        // Initalize GPIO
        // Enable XCLOCKOUT to allow monitoring of oscillator 1
        GPIO_setMode(myGpio, GPIO_Number_18, GPIO_18_Mode_XCLKOUT);
        CLK_setClkOutPreScaler(myClk, CLK_ClkOutPreScaler_SysClkOut_by_1);


        // Setup a debug vector table and enable the PIE
        PIE_setDebugIntVectorTable(myPie);
        PIE_enable(myPie);

        // Initialize the ADC
        ADC_enableBandGap(myAdc);
        ADC_enableRefBuffers(myAdc);
        ADC_powerUp(myAdc);
        ADC_enable(myAdc);
        ADC_setVoltRefSrc(myAdc, ADC_VoltageRefSrc_Int);

        ADC_setSocChanNumber (myAdc, ADC_SocNumber_0, ADC_SocChanNumber_A4);    //Set SOC0 channel select to ADCINA4
        ADC_setSocChanNumber (myAdc, ADC_SocNumber_1, ADC_SocChanNumber_A4);    //Set SOC1 channel select to ADCINA4
        ADC_setSocSampleWindow(myAdc, ADC_SocNumber_0, ADC_SocSampleWindow_7_cycles);   //Set SOC0 acquisition period to 7 ADCCLK
        ADC_setSocSampleWindow(myAdc, ADC_SocNumber_1, ADC_SocSampleWindow_7_cycles);   //Set SOC1 acquisition period to 7 ADCCLK
        ADC_setIntSrc(myAdc, ADC_IntNumber_1, ADC_IntSrc_EOC1);                 //Connect ADCINT1 to EOC1
        ADC_enableInt(myAdc, ADC_IntNumber_1);                                  //Enable ADCINT1
        // Note: two channels have been connected to the ADCinA4
           // so that the first sample can be discarded to avoid the
           // ADC first sample issue.  See the device errata.

           // Set the flash OTP wait-states to minimum. This is important
           // for the performance of the temperature conversion function.
           FLASH_setup(myFlash);


           //Main program loop - continually sample current
           for(;;)
           {

               //Force start of conversion on SOC0 and SOC1
               ADC_forceConversion(myAdc, ADC_SocNumber_0);
               ADC_forceConversion(myAdc, ADC_SocNumber_1);

               //Wait for end of conversion.
               while(ADC_getIntStatus(myAdc, ADC_IntNumber_1) == 0) {
               }

               // Clear ADCINT1
               ADC_clearIntFlag(myAdc, ADC_IntNumber_1);

               // Get current sensor sample result from SOC1
               current = ADC_readResult(myAdc, ADC_ResultNumber_1);

           }
       }

