/*

  ramps_smart_map.h - EXPERIMENTAL Mapping for RAMPS SMART by @MrAntza99
  Made from base file ramps_1.6_map.h and redefined all incorrect pins due small changes of these Ramps board variations

  Remade by MrAntza
  Original file from GrblHAL/SAM3X8E for Arduino Due & Ramps 1.6

  Links used
  https://github.com/terjeio/grblHAL/blob/master/drivers/SAM3X8E/Arduino%20Due%20shield%20pin%20mappings/ramps_1.6_map.md
  http://www.robgray.com/temp/Due-pinout-WEB.png
  https://reprap.org/wiki/SMART_RAMPS
  https://reprap.org/wiki/RAMPS_1.6


  Note to myself (inverted safety switch pins)
  https://github.com/grblHAL/core/wiki/First-Run-Grbl-Settings

  Remember to send new map file here
  https://github.com/grblHAL/Controllers/discussions
*/

/* Relevant differences and notes what I've found and redefines between Ramps 1.6 and Ramps Smart

  -M3 and M4 stepper pins were right portpins but commented Due pins was incorrect with question mark at end of number
  -confusion of meaning of these line  SPINDLE_PWM_TIMER   (TC2->TC_CHANNEL[0])  &  SPINDLE_PWM_CCREG   2
  -added Ramps pin names after due pin name for nesessary areas, to easy wiring
  -Noticed weird way to use this as output pin #define COOLANT_FLOOD_PIN   18  // Due A9 & Ramps THERM0 / Connector JP7 (2/6)
  -feed hold & cycle start had wrong pins named in comment
  -Comment //Correcting Ramps 1.6  if has same layout with Ramps 1.6 pin mapping, just my recheck prosess


  Function       Due   Ramps 1.6     Ramps Smart

  COOLANT_FLOOD   A9    AUX-2 (4)     THERM0 JP7 (2) !!!

  RESET           A3      D2 AUX-1 (3)  A3 AUX-2 (3)
  FEED_HOLD       A5    A5 AUX-2 (3)  A5 AUX-2 (5)
  CYCLE_START     A4      D1 AUX-1 (4)  A4 AUX-2 (4)

  AUXOUTPUT0      D53     AUX-3 (6)     AUX-3 (8)
  AUXOUTPUT1      D51     AUX-3 (4)     AUX-3 (6)
  AUXOUTPUT2      D49     AUX-3 (2)     AUX-3 (4)

*/

// Not sure of this meaning, so untouched
#if (N_AUTO_SQUARED && N_AUTO_SQUARED < N_ABC_MOTORS) || N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "Ramps SMART"
#define BOARD_URL "https://reprap.org/wiki/SMART_RAMPS"

// v1.4.1 and v1.4.2 has a 256kbit EEPROM mounted

#if I2C_ENABLE
#undef I2C_ENABLE
#define I2C_ENABLE 2
#endif

// Define step pulse output pins.
#define X_STEP_PORT         PIOA
#define X_STEP_PIN          16  // Due A0           //Correcting Ramps 1.6
#define Y_STEP_PORT         PIOA
#define Y_STEP_PIN          3   // Due A6           //Correcting Ramps 1.6
#define Z_STEP_PORT         PIOC
#define Z_STEP_PIN          17  // Due D46          //Correcting Ramps 1.6

// Define step direction output pins.
#define X_DIRECTION_PORT    PIOA
#define X_DIRECTION_PIN     24  // Due A1           //Correcting Ramps 1.6
#define Y_DIRECTION_PORT    PIOA
#define Y_DIRECTION_PIN     2   // Due A7           //Correcting Ramps 1.6
#define Z_DIRECTION_PORT    PIOC
#define Z_DIRECTION_PIN     15  // Due D48          //Correcting Ramps 1.6

// Define stepper driver enable/disable output pin(s).
#define X_ENABLE_PORT       PIOC
#define X_ENABLE_PIN        6   // Due D38          //Correcting Ramps 1.6
#define Y_ENABLE_PORT       PIOA
#define Y_ENABLE_PIN        23  // Due A2           //Correcting Ramps 1.6
#define Z_ENABLE_PORT       PIOB
#define Z_ENABLE_PIN        17  // Due A8           //Correcting Ramps 1.6

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT        PIOC
#define X_LIMIT_PIN         28  // Due D3 & Ramps X-MIN         //Correcting Ramps 1.6
#define Y_LIMIT_PORT        PIOD
#define Y_LIMIT_PIN         4   // Due D14 & Ramps Y-MIN        //Correcting Ramps 1.6
#define Z_LIMIT_PORT        PIOA
#define Z_LIMIT_PIN         11  // Due D18 & Ramps Z-MIN        //Correcting Ramps 1.6

// Define homing/hard limit switch input pins.
#if X_AUTO_SQUARE
#define M3_LIMIT_PORT       PIOB
#define M3_LIMIT_PIN        25  // Due D2 & Ramps X-MAX
#else
#define X_LIMIT_PORT_MAX    PIOB
#define X_LIMIT_PIN_MAX     25  // Due D2 & Ramps X-MAX         //Correcting Ramps 1.6
#endif
#if Y_AUTO_SQUARE
#define M3_LIMIT_PORT       PIOD
#define M3_LIMIT_PIN        5   // Due D15 & Ramps Y-MAX
#else
#define Y_LIMIT_PORT_MAX    PIOD
#define Y_LIMIT_PIN_MAX     5   // Due D15 & Ramps Y-MAX        //Correcting Ramps 1.6
#endif
#if Z_AUTO_SQUARE
#define M3_LIMIT_PORT       PIOA
#define M3_LIMIT_PIN        10  // Due D19 & Ramps Z-MAX
#else
#define Z_LIMIT_PORT_MAX    PIOA
#define Z_LIMIT_PIN_MAX     10  // Due D19 & Ramps Z-MAX        //Correcting Ramps 1.6
#endif

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PORT        PIOD
#define M3_STEP_PIN         1   // Due D26 & Ramps E0-STEP      //Correcting Ramps 1.6
#define M3_DIRECTION_PORT   PIOD
#define M3_DIRECTION_PIN    3   // Due D28 & Ramps E0-DIR       //Correcting Ramps 1.6
#define M3_ENABLE_PORT      PIOA
#define M3_ENABLE_PIN       15  // Due D24 & Ramps E0-EN        //Correcting Ramps 1.6
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_STEP_PORT        PIOC
#define M4_STEP_PIN         4   // Due D36 & Ramps E1-STEP                   //Correcting Ramps 1.6
#define M4_DIRECTION_PORT   PIOC
#define M4_DIRECTION_PIN    2   // Due D34 & Ramps E1-DIR                    //Correcting Ramps 1.6
#define M4_ENABLE_PORT      PIOC
#define M4_ENABLE_PIN       9   // Due D30 & Ramps E1-EN                     //Correcting Ramps 1.6
#endif

#define AUXOUTPUT0_PORT     PIOB
#define AUXOUTPUT0_PIN      14  // Due D53 & Ramps D53 / AUX-3 Connector U$1 (8/10)         //Ramps 1.6 Has different style AUX-3 port but pin is there (6/8)
#define AUXOUTPUT1_PORT     PIOC
#define AUXOUTPUT1_PIN      12  // Due D51 & Ramps D51 / MOSI / AUX-3 Connector U$1 (6/10)  //Ramps 1.6 Has different style AUX-3 port but pin is there (4/8)
#define AUXOUTPUT2_PORT     PIOC
#define AUXOUTPUT2_PIN      14  // Due D49 & Ramps D49 / AUX-3 Connector U$1 (4/10)         //Ramps 1.6 Has different style AUX-3 port but pin is there (2/8)
#define AUXOUTPUT3_PORT     PIOC // Spindle PWM, Due D5 / TIOA6 / PWM5 & Ramps D5 / SER3              //Correcting Ramps 1.6
#define AUXOUTPUT3_PIN      25
#define AUXOUTPUT4_PORT     PIOC // Spindle direction, Due D8 & Ramps D8 / P$2 (Mosfet Outputs)             //Correcting Ramps 1.6
#define AUXOUTPUT4_PIN      22
#define AUXOUTPUT5_PORT     PIOC // Spindle enable, Due D4 & Ramps D4 / SER4                              //Correcting Ramps 1.6
#define AUXOUTPUT5_PIN      26
#define AUXOUTPUT6_PORT     PIOB // Coolant flood, Due A9 & Ramps THERM0 / Connector JP7 (2/6)              //!!! Ramps 1.6 A9 goes AUX-2 (4/10) !!!
#define AUXOUTPUT6_PIN      15

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT5_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT5_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_TIMER       (TC2->TC_CHANNEL[0])
#define SPINDLE_PWM_CCREG       2
#define SPINDLE_PWM_PORT        AUXOUTPUT3_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT3_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT4_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT4_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT6_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT6_PIN
#if COOLANT_ENABLE & COOLANT_MIST
#undef COOLANT_ENABLE
#define COOLANT_ENABLE COOLANT_FLOOD
#endif
#elif COOLANT_ENABLE & COOLANT_MIST
#undef COOLANT_ENABLE
#define COOLANT_ENABLE 0
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PORT              PIOA
#define RESET_PIN               22  // DUE A3 & Ramps A3 / AUX-2 Connector U$10 (3/10)          //!!! Ramps 1.6 D2 goes AUX-1 A-OUT 3 which Ramps Smart doesnt have !!!
#define FEED_HOLD_PORT          PIOA
#define FEED_HOLD_PIN           4   // DUE A5 & Ramps A5 / AUX-2 Connector U$10 (5/10)          //!!! Ramps 1.6 A5 goes AUX-2 (3/10) !!!
#define CYCLE_START_PORT        PIOA
#define CYCLE_START_PIN         6   // DUE A4 & Ramps A4 / AUX-2 Connector U$10 (4/10)          //!!! Ramps 1.6 D1 goes AUX-1 A-OUT 4 which Ramps Smart doesnt have !!!

#define AUXINPUT0_PORT          PIOA
#define AUXINPUT0_PIN           14  // Due D23 & Ramps D23 / AUX-4 Connector U$9 (16/18)        //Correcting Ramps 1.6
#define AUXINPUT1_PORT          PIOD
#define AUXINPUT1_PIN           0   // Due D25 & Ramps D25 / AUX-4 Connector U$9 (15/18)        //Correcting Ramps 1.6
#define AUXINPUT2_PORT          PIOD
#define AUXINPUT2_PIN           2   // Due D27 & Ramps D27 / AUX-4 Connector U$9 (14/18)        //Correcting Ramps 1.6

#define AUXINTPUT0_ANALOG_PORT  PIOB
#define AUXINTPUT0_ANALOG_PIN   19 // Due A10
#define AUXINTPUT1_ANALOG_PORT  PIOB
#define AUXINTPUT1_ANALOG_PIN   20 // Due A11

/**/
