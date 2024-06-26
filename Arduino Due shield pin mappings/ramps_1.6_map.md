## SAM3X8E Driver

### Pin assignmets for [RAMPS 1.6] board.

Uncomment `#define BOARD_RAMPS16` in [driver.h](../main/driver.h) to use this mapping.

``` plain
                                                          [SCL1] A.17
                                                          [SDA1] A.18
                                      NOT USED [  ]         [  ] AREF
                                         IOREF [  ]         [  ] GND
                                         RESET [  ]    A    [13] B.27
                                           3V3 [  ]    R    [12] D.08
                                            5V [  ]    D    [11] D.07
                                           GND [  ]    U    [10] C.29/A.28
                                           GND [  ]    I    [09] C.21
                                           VIN [  ]    N    [08] C.22 - Spindle dir
                                                       O
                                 X step - A.16 [A0]         [07] C.23
                                  X dir - A.24 [A1]    D    [06] C.24
                               Y enable - A.23 [A2]    U    [05] C.25 - Spindle PWM
                                  Reset - A.22 [A3]    E    [04] C.26/A.29 - Spindle enable
                              Feed hold - A.06 [A4]         [03] C.28 - X limit min
                            Cycle start - A.04 [A5]    S    [02] B.25 - X limit max  (Alternative function: X Auto Square, if compiled)
                                 Y step - A.03 [A6]    A    [01] A.09
                                  Y dir - A.02 [A7]    M    [00] A.08
                                                       3
                               Z enable - B.17 [A8]    X    [14] D.04 - Y limit min
                                Coolant - B.18 [A9]    8    [15] D.05 - Y limit max  (Alternative function: Y Auto Square, if compiled)
                                          B.19 [A10]   E    [16] A.13
                                          B.20 [A11]        [17] A.12
                                          B.15 [DAC0]       [18] A.11 - Z limit min
                                          B.16 [DAC1]       [19] A.10 - Z limit max  (Alternative function: Z Auto Square, if compiled)
                                          A.01 [CANRX]      [20] B.12
                                          A.00 [CANTX]      [21] B.13 - Probe input

C.06 - X enable ------------------------------------+     +-------------------------------------------- B step - C.04 (E2)
C.08 ------------------------------------------+    |     |    +---------------------------------------- B dir - C.02 (E2)
A.19 -------------------------------------+    |    |     |    |    +------------------------------------------- D.10
C.19 --------------------------------+    |    |    |     |    |    |    +--------------------------- B enable - D.09 (E2)
C.17 - Z step ------------------+    |    |    |    |     |    |    |    |    +------------------------- A dir - D.03 (E1)
C.15 - Z dir --------------+    |    |    |    |    |     |    |    |    |    |    +------------------- A step - D.01 (E1)
C.13 ------- ---------+    |    |    |    |    |    |     |    |    |    |    |    |    +------------ A enable - A.15 (E1)
B.21 ------------+    |    |    |    |    |    |    |     |    |    |    |    |    |    |    +------------------ B.26
          [GND] [52] [50] [48] [46] [44] [42] [40] [38] [36] [34] [32] [30] [28] [26] [24] [22] [5V]
          [GND] [53] [51] [49] [47] [45] [43] [42] [39] [37] [35] [33] [31] [29] [27] [25] [23] [5V]
B.14 ------------+    |    |    |    |    |    |    |     |    |    |    |    |    |    |    +------------------ A.14
C.12 -----------------+    |    |    |    |    |    |     |    |    |    |    |    |    +----------------------- D.00
C.14 ----------------------+    |    |    |    |    |     |    |    |    |    |    +---------------------------- D.02
C.16 ---------------------------+    |    |    |    |     |    |    |    |    +--------------------------------- D.06
C.18 --------------------------------+    |    |    |     |    |    |    +-------------------------------------- A.07
A.20 -------------------------------------+    |    |     |    |    +------------------------------------------- C.01
C.09 ------------------------------------------+    |     |    +------------------------------------------------ C.03
C.07 -----------------------------------------------+     + ---------------------------------------------------- C.05

```
---
2024-04-04
