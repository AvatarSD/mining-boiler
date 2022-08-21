#ifndef FW_CONFIG_H
#define FW_CONFIG_H

/* GPIO analog inputs */
#define GPIO_V_IN 36
#define GPIO_TOUCH 39 /* not used */

/* GPIO counters IN */
#define GPIO_CNT_CLNT 34
#define GPIO_CNT_BOIL 35
#define GPIO_CNT_WRK0 32
#define GPIO_CNT_WRK1 -1
#define GPIO_CNT_WRK2 -1

/* GPIO enable temp sensors, active high */
#define GPIO_TEMP_WRK0_IN 33
#define GPIO_TEMP_WRK0_OUT 25
#define GPIO_TEMP_WRK1_IN -1
#define GPIO_TEMP_WRK1_OUT -1
#define GPIO_TEMP_WRK2_IN -1
#define GPIO_TEMP_WRK2_OUT -1
#define GPIO_TEMP_CLNT_IN 13
#define GPIO_TEMP_CLNT_OUT 12
#define GPIO_TEMP_BOIL_BOTH 14
#define GPIO_TEMP_COLER 5
#define GPIO_TEMP_AUX0 2
#define GPIO_TEMP_AUX1 -1
#define GPIO_TEMP_AUX2 -1
#define GPIO_TEMP_AUX3 -1
#define GPIO_TEMP_AUX4 -1
#define GPIO_TEMP_AUX5 -1

/* GPIO enable control */
#define GPIO_CTRL_WRK0 21
#define GPIO_CTRL_WRK1 -1
#define GPIO_CTRL_WRK2 -1
#define GPIO_CTRL_PUMP_PRI 19
#define GPIO_CTRL_PUMP_SEC 18
#define GPIO_CTRL_FAN0 23
#define GPIO_CTRL_FAN1 22

/* GPIO interfaces */
#define GPIO_MASTER_SCL 26 /*!< gpio number for I2C master clock */
#define GPIO_MASTER_SDA 27 /*!< gpio number for I2C master data  */
#define GPIO_PM_TX 0
#define GPIO_PM_RX 15
#define GPIO_HW_OW_TX CONFIG_DS2480_UART_TXD
#define GPIO_HW_OW_RX CONFIG_DS2480_UART_RXD
#define GPIO_HW_OW_EN CONFIG_DS2480_ENABLE

/* Periferial config */
#define I2C_MASTER_NUM I2C_NUM_1  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */
#define UART_HW_OW_NUM CONFIG_DS2480_UART_NUM

#endif