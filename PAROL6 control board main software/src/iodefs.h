#pragma once

#ifdef PAROL6_BOARD_OCTOPUS_PRO_F446
#include "board_octopus_pro_f446.h"
#else

// STEP пины
#define PUL1  PF13   // Joint 1 → MOTOR0
#define PUL6  PG0    // Joint 2 → MOTOR1
#define PUL5  PF11   // Joint 3 → MOTOR2
#define PUL4  PG4    // Joint 4 → MOTOR3
#define PUL2  PF9    // Joint 5 → MOTOR4
#define PUL3  PC13   // Joint 6 → MOTOR5

// DIR пины
#define DIR1  PF12   // Joint 1 → MOTOR0
#define DIR6  PG1    // Joint 2 → MOTOR1
#define DIR5  PG3    // Joint 3 → MOTOR2
#define DIR4  PC1    // Joint 4 → MOTOR3
#define DIR2  PF10   // Joint 5 → MOTOR4
#define DIR3  PF0    // Joint 6 → MOTOR5

// CS (SELECT) пины для SPI
#define SELECT1  PC4    // Joint 1 → MOTOR0
#define SELECT6  PD11   // Joint 2 → MOTOR1
#define SELECT5  PC6    // Joint 3 → MOTOR2
#define SELECT4  PC7    // Joint 4 → MOTOR3
#define SELECT2  PF2    // Joint 5 → MOTOR4
#define SELECT3  PE4    // Joint 6 → MOTOR5

// EN пины (на Octopus Pro каждый мотор имеет свой EN пин)
#define GLOBAL_ENABLE  PF14   // EN MOTOR0 (Joint 1)
#define ENABLE_M1      PF15   // EN MOTOR1 (Joint 2)
#define ENABLE_M2      PG5    // EN MOTOR2 (Joint 3)
#define ENABLE_M3      PA0    // EN MOTOR3 (Joint 4)
#define ENABLE_M4      PG2    // EN MOTOR4 (Joint 5)
#define ENABLE_M5      PF1    // EN MOTOR5 (Joint 6)

// Концевые выключатели → разъёмы DIAG0-DIAG5
#define LIMIT1  PG6    // → DIAG0
#define LIMIT2  PG9    // → DIAG1
#define LIMIT3  PG10   // → DIAG2
#define LIMIT4  PG11   // → DIAG3
#define LIMIT5  PG12   // → DIAG4
#define LIMIT6  PG13   // → DIAG5

// SPI пины (совпадают с оригиналом)
#define MISO  PA6
#define MOSI  PA7
#define SCK   PA5

// Flash — на Octopus Pro нет внешней flash, оставляем для компиляции
#define FLASH_SELECT  PA4

// Питание
#define SUPPLY_ON_OFF       PE11
#define SUPPLY_BUTTON_STATE PE11

// Дополнительные I/O
#define INPUT1   PC15
#define INPUT2   PC14
#define OUTPUT1  PA2
#define OUTPUT2  PA3

// E-STOP
#define ESTOP  PG15

// LED
#define LED1  PB10
#define LED2  PB11

// USB
#define USB_D_PLUS   PA12
#define USB_D_MINUS  PA11

// CAN
#define CAN1TX  PB9
#define CAN1RX  PB8
#define CAN2TX  PB13
#define CAN2RX  PB12

// Напряжение питания
#define VBUS  PB0

// R_SENSE для TMC5160T Pro
#define R_SENSE  0.075f

#endif
