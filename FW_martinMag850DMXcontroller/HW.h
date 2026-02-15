//Parameters
#ifndef HW_h
#define HW_h


// Define PWM pins
#define PWM_OUT         13

//DMX pins
#define DMX_TX_PIN      16
#define DMX_RX_PIN      17
#define DMX_EN_PIN      26

//I2C Display pins
#define I2C_SDA 	      21
#define I2C_SCL 	      22

//W5500 Ethernet module pins
#define SPI_MOSI        23
#define SPI_MISO        19
#define SPI_CLK         18

#define ETH_PHY_TYPE    ETH_PHY_W5500
#define ETH_PHY_ADDR    1
#define ETH_PHY_CS      5
#define ETH_PHY_IRQ     4
#define ETH_PHY_RST     5

//Switch panel

#define SW_MINUS        33
#define SW_ENTER        25
#define SW_PLUS         32

//FOGGER STATUS INPUT
#define FOGGER_HEATING  14
#define FOGGER_READY    27

//PWM calibration

#define VOUT_IDLE       27 //1,5Vout 
#define VOUT_START      40 //2Vout
#define VOUT_END        243 //10Vout
#define PWM_FREQ        2000 //PWM frequency, set accordingly the RC filter

//OLED display
#define SCREEN_WIDTH    128 // OLED display width, in pixels
#define SCREEN_HEIGHT   32 // OLED display height, in pixels
#define OLED_RESET      -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS  0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32


#endif