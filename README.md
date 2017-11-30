# MicromouseRev.3-
CSUChico IEEE MicroMouse Code

IMPORTANT INFO FOR CONVERTING YOUR KIEL TO A C++ COMPILER
1. Open "Options for Target" menu (there is a button on the task bar that looks like a wand with a circuit)
2. Open the "C/C++" tab in that menu
3. On the bottom of that tab there are 3 lines you can fill in. The middle one labeled "Misc Controls" should say
   something like "--C99". Change this to "--cpp". 
4. Your compiler is now compiling C++. Congrats.


PINOUT FOR THE STM32L432KC EVAL BOARD

 1 *   USB   * 30
 2 *         * 29
 3 *         * 28
 4 *         * 27
 5 *         * 26
 6 *         * 25
 7 *         * 24
 8 *         * 23
 9 *         * 22
10 *         * 21
11 *         * 20
12 *         * 19
13 *         * 18
14 *         * 17
15 *         * 16

*******************************************************
# :NAME         :CURRENT USE              :TEST BENCH
*******************************************************
1 :PA9          :PWM motor A1
2 :PA10         :PWM motor A2
3 :RST          :
4 :GND          :GND
5 :PA12         :Button 1 input
6 :PB0          :encoder right A input
7 :PB7          :Switch 2 input
8 :PB6          :switch 1 input
9 :PB1          :encoder right B input
10:PC14         :
11:PC15         :
12:PA8          :PWM motor B1
13:PA11         :PWM motor B2
14:PB5          :encoder left B input
15:PB4          :encoder left A input
16:PB3          :
17:3V3          :
18:AREF         :
19:PA0          :IR_BL
20:PA1          :IR_FL
21:PA3          :IR_M
22:PA4          :IR_FR
23:PA5          :IR_BR
24:PA6          :LED 1
25:PA7          :LED 2
26:PA2          :LED 3
27:5V           :Power input
28:RST          :
29:GND          :GND
30:VIN          :


****************************************************************************************
                                 Testing Checklist
****************************************************************************************

PWM Output Works    :
ADC Input Checks out:
EXTI Interupts Occur:
DIP Switch Works    :
ADC readings from IR:
GPIO init works     :
Button works        :
LEDs work           :


