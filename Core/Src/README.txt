Copies of main were made for different states of the lab.
(just remove "lab02_checkoffX_" part of the file to test)

i.e: 
"lab02_checkoff1_main.c" contains the code necessary to pass checkoff1 (Button causes EXTI IRQ, which toggles LEDs)

"lab02_checkoff2A_main.c" contains the code necessary to pass the first half of checkoff 2 (where EXTI IRQ is priority over SysTick IRQ)

"lab02_checkoff2B_main.c" contains the code necessary to pass the second half of checkoff 2 (where SysTick IRQ is priority over EXTI IRQ)