
#define NUCLEO_F446RE 0
#define NUCLEO_F401RE 1
#define NUCLEO NUCLEO_F401RE //Current nucleo used

//TODO: DEFINE CORRECT PINS FOR F446RE
#if NUCLEO == NUCLEO_F446RE
    //===== NUCLEO F446RE =====
    #define PIN_MOTOR_ENABLE PA_13
    #define PIN_DEVICE_ENABLE PA_14
    #define PIN_MOTOR_SPEEDCTRL PA_15
    #define PIN_RX PA_1
    #define PIN_TX PA_0

#elif NUCLEO == NUCLEO_F401RE
    //===== NUCLEO F401RE =====
    #define PIN_MOTOR_ENABLE PA_13
    #define PIN_DEVICE_ENABLE PA_14
    #define PIN_MOTOR_SPEEDCTRL PA_15
    #define PIN_RX PA_12
    #define PIN_TX PC_6

#endif

































