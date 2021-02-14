# Low_Power_modes

This file will tells you the flow of Program, clock configuration, and low power mode power consuption.

Clock configuration:
clock configuratuion section will explain you the clocks for system bus, Pheripheral buses(APB1, APB2).

RUNMODE clock configuration:
HSI clock is 16MHz.
LSI clock is 37KHz.
PLL block input clock is HSI clock and with multiplier(3) and divider(2). PLL block output is 24MHz.
In this mode system clock is selected from PLL clock.
RTC clock is selected from LSI clock source.
IN this mode HCLK = FCLK = System timer = 24MHZ.

STOPMODE clock configuration:
All clocks are disabled except RTC clock untill the next interrupt from RTC internal block. RTC clock is always clock from LSI clock.
 
RUNMODE Power consumption:
As per the datasheet current consumtion in this mode is 6.3mA. 

STOPMODE power consumption:
As per the datasheet current consumtion in this mode is 410nA.

Avarage Current Consumption:
Avarage Current consumption for the application is caculated as 630uA.


NOTE: 
Measures taken to ensure LOW power consumption:
1. setting unused pins to analog mode that minimize the leakage current from GPIO.
2. setting up the pheripheral clocks and system clocks to disable mode to minimize the power.


problem statement:
Baye's algorithm implementation on core M0+ micro-controller with low power consumption.
1. Problem statement is about the calculating the probability of a given data.
2. Problem training set contains two sets named outlook and temparature.
3. outlook has 3 catagiries namely : SUNNY, OVRCAST, and RAINY.
4. temparature 3 catagories namely : HOT, MILD, and COOL.
5. Probability of Playing yes and playing no will be taken out of the given training samples.




