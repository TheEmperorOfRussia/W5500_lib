/*
 * w5500.h
 *
 *  Created on: Mar 17, 2025
 *      Author: The emperor Arseni I
 */

#include "inttypes.h"

#ifndef INC_W5500_H_
#define INC_W5500_H_
// 06.05.25
// ======= Определения SPI-команд =======
#define W5500_READ       0x00  // Флаг для чтения
#define W5500_WRITE      0x04  // Флаг для записи
#define W5500_COMMON_REG 0x00  // Общие регистры (Mode, Gateway, Subnet, MAC, IP)

// ======= Common Register Block  W5500 =======
#define W5500_MR       0x0000  // Mode Register			(1 байт)
#define W5500_GAR      0x0001  // Gateway Address Register      (4 байта)
#define W5500_SUBR     0x0005  // Subnet Mask Register          (4 байта)
#define W5500_SHAR     0x0009  // Source MAC Address Register   (6 байт)
#define W5500_SIPR     0x000F  // Source IP Address Register    (4 байта)
#define W5500_INTLEVEL 0x0013  // Interrupt Low Level Timer     (2 байта)
#define W5500_IR       0x0015  // Interrupt                     (1 байт)
#define W5500_IMR      0x0016  // Interrupt Mask                (1 байт)
#define W5500_SIR      0x0017  // Socket interrupt              (1 байт)
#define W5500_SIMR     0x0018  // Socket interrupt mask         (1 байт)
#define W5500_RTR      0x0019  // Retry Time                    (2 байта)
#define W5500_RCR      0x001B  // Retry count                   (1 байт)
#define W5500_PTIMER   0x001C  // PPP LCP Request Timer        	(1 байт)
#define W5500_PMAGIC   0x001D  // PPP LCP magic number          (1 байт)
#define W5500_PHAR     0x001E  // PPP Destination MAC Address   (6 байт)
#define W5500_PSID     0x0024  // PPP Session Identification    (2 байта)
#define W5500_PMRU     0x0026  // PPP Maximum Segment Size      (2 байта)
#define W5500_UIPR     0x0028  // Unreachable IP address        (4 байта)
#define W5500_UPORTR   0x002C  // Unreachable Port              (2 байта)
#define W5500_PHYCFGR  0x002E  // PHY Configuration             (1 байт)
// 0x002F - 0x0038 RESERVED
#define W5500_ChipVers 0x0039  // Chip version                  (1 байт) ИЛИ 0x0038

#define W5500_PHY_standart_init 0xBF  // Стандартная настрйока PHY

// ======= Socket Register Block  W5500 =======
#define W5500_SOCKET_BASE(socket)  (0x1 + 0x4 * socket)  //(1 + 4 * n)
#define W5500_SOCKET_TXBUF(socket) (W5500_SOCKET_BASE(socket) + 0x1)
#define W5500_SOCKET_RXBUF(socket) (W5500_SOCKET_BASE(socket) + 0x2)
#define W5500_Sn_MR                0x0000  // Socket Mode Register 				(1 байт)
#define W5500_Sn_CR                0x0001  // Socket Command Register 				(1 байт)
#define W5500_Sn_IR                0x0002  // Socket Interrupt Register 			(1 байт)
#define W5500_Sn_SR                0x0003  // Socket Status Register 				(1 байт)
#define W5500_Sn_PORT              0x0004  // Socket Source Port Register 			(2 байта)
#define W5500_Sn_DHAR              0x0006  // Socket Destination Hardware Address Register 	(6 байт)
#define W5500_Sn_DIPR              0x000C  // Socket Destination IP Address Register 		(4 байта)
#define W5500_Sn_DPORT             0x0010  // Socket Destination Port Register 			(2 байта)
#define W5500_Sn_MSSR              0x0012  // Socket Maximum Segment Size Register 		(2 байта)
#define W5500_Sn_TOS               0x0015  // Socket IP Type of Service Register 		(1 байт)
#define W5500_Sn_TTL               0x0016  // Socket IP Time to Live Register 			(1 байт)
#define W5500_Sn_RXBUF_SIZE        0x001E  // Socket Receive Buffer Size 			(1 байт)
#define W5500_Sn_TXBUF_SIZE        0x001F  // Socket Transmit Buffer Size 			(1 байт)
#define W5500_Sn_TX_FSR            0x0020  // Socket Transmit Free Size Register 		(2 байта)
#define W5500_Sn_TX_RD             0x0022  // Socket Transmit Read Pointer Register 		(2 байта)
#define W5500_Sn_TX_WR             0x0024  // Socket Transmit Write Pointer Register 		(2 байта)
#define W5500_Sn_RX_RSR            0x0026  // Socket Receive Received Size Register 		(2 байта)
#define W5500_Sn_RX_RD             0x0028  // Socket Receive Read Pointer Register 		(2 байта)
#define W5500_Sn_RX_WR             0x002A  // Socket Receive Write Pointer Register 		(2 байта)
#define W5500_Sn_IMR               0x002C  // *Socket Interrupt Mask 				(1 байт)
#define W5500_Sn_FRAG              0x002D  // Socket Fragment Offset in IP Header Register 	(2 байта)
#define W5500_Sn_KPALVTR           0x002F  // Socket Keep Alive Timer Register 			(1 байт)

// ======= SN_IR bits W5500 =======
#define W5500_SEND_OK 0x10
#define W5500_TIMEOUT 0x8
#define W5500_RECV    0x4
#define W5500_DISCON  0x2
#define W5500_CON     0x1

typedef struct
{
    uint8_t  num;               // Номер сокета (0–7)
    uint8_t  mode;              // Режим работы сокета (Sn_MR)
    uint8_t  status;            // Последний статус сокета (Sn_SR)
    uint8_t  last_interrupt;    // Последнее значение регистра прерываний (Sn_IR)
    uint16_t port;              // Порт источника (Sn_PORT)
    uint8_t  rx_buf_size;       // Размер буфера приема (Sn_RXBUF_SIZE)
    uint8_t  tx_buf_size;       // Размер буфера передачи (Sn_TXBUF_SIZE)
    uint16_t tx_rd_p;           // Указатель чтения из буфера передачи (Sn_TX_RD)
    uint16_t tx_wr_p;           // Указатель записи в буфер передачи (Sn_TX_WR)
    uint16_t rx_rd_p;           // Указатель чтения из буфера приема (Sn_RX_RD)
    uint16_t rx_wr_p;           // Указатель записи в буфер приема (Sn_RX_WR)
    uint8_t  keep_alive_timer;  // Таймер поддержания соединения (Sn_KPALVTR)
    uint8_t  dipr[4];           // IP получателя
    uint16_t dport;             // Порт получателя
} W5500_Socket;

// Тип функции для передачи данных через SPI
typedef uint8_t (*W5500_SPI_Transmit_Func)(const uint8_t* data, uint16_t len);

// Тип функции для приёма данных через SPI
typedef uint8_t (*W5500_SPI_Receive_Func)(uint8_t* buffer, uint16_t len);

// Тип функции для поднятия и опускания CS: 0 - CSs 1 - CSr
typedef void (*W5500_CS_Func)(uint8_t type_operation);

// Тип функции для Задержки
typedef void (*W5500_Delay_Func)(uint32_t delay);

typedef struct
{
    /////// Функции для работы с данными //////
    W5500_SPI_Transmit_Func Transmit;
    W5500_SPI_Receive_Func  Receive;
    W5500_CS_Func           CS;  // 0-CSs 1-CSr
    W5500_Delay_Func        Delay;

    W5500_Socket sockets[8];
} W5500_User_Funcs;

typedef struct W5500_Main_Struct W5500_Main_Struct;

// Тип функции для обработки прерывания результата команды
typedef void (*W5500_Cb_SEND_OK)(W5500_Socket* socket, W5500_Main_Struct* MS);

// Тип функции для обработки прерывания TIMEOUT
typedef void (*W5500_Cb_TIMEOUT)(W5500_Socket* socket, W5500_Main_Struct* MS);

// Тип функции для обработки прерывания получения данных
typedef void (*W5500_Cb_RECV)(uint8_t* buf, uint16_t size, W5500_Socket* socket, W5500_Main_Struct* MS);

// Тип функции для обработки прерывания отключения
typedef void (*W5500_Cb_DISCON)(W5500_Socket* socket, W5500_Main_Struct* MS);

// Тип функции для обработки прерывания подключения
typedef void (*W5500_Cb_CON)(W5500_Socket* socket, W5500_Main_Struct* MS);

typedef struct
{
    /////////////// Callbacks //////
    W5500_Cb_SEND_OK Callback_Send_OK;
    W5500_Cb_TIMEOUT Callback_Timeout;
    W5500_Cb_RECV    Callback_Recv;
    W5500_Cb_DISCON  Callback_Discon;
    W5500_Cb_CON     Callback_Con;

} W5500_User_Callbacks;

// Главная структура
typedef struct W5500_Main_Struct {
    W5500_User_Funcs*     UF;
    W5500_User_Callbacks* UCb;

} W5500_Main_Struct;

// ======= Базовые функции чтения и записи регистров=======
uint8_t W5500_ReadRegister(
    uint16_t          address,
    uint8_t           bsb,
    uint8_t*          buffer,
    uint16_t          len,
    W5500_User_Funcs* UF);

uint8_t W5500_WriteRegister(
    uint16_t          address,
    uint8_t           bsb,
    const uint8_t*    data,
    uint16_t          len,
    W5500_User_Funcs* UF);

// ======= Функции для работы с регистрами =======

// %%%%======= Common Register INIT =======%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Запись Mode Register (MR)
uint8_t W5500_Set_MR(uint8_t mode, W5500_User_Funcs* UF);
// Запись Gateway Address Register (GAR)
uint8_t W5500_Set_Gateway(const uint8_t* gateway_address, W5500_User_Funcs* UF);
// Запись Subnet Mask Register (SUBR)
uint8_t W5500_Set_Mask(const uint8_t* mask_address, W5500_User_Funcs* UF);
// Запись Source MAC Address Register (SHAR)
uint8_t W5500_Set_MAC(const uint8_t* mac_address, W5500_User_Funcs* UF);
// Запись Source IP Address Register (SIPR)
uint8_t W5500_Set_IP(const uint8_t* ip_address, W5500_User_Funcs* UF);
// Запись Interrupt Low Level Timer (INTLEVEL)
uint8_t W5500_Set_INTLEVEL(uint16_t int_level, W5500_User_Funcs* UF);
// Запись Interrupt Mask (IMR)
uint8_t W5500_Set_IMR(uint8_t mask, W5500_User_Funcs* UF);
// Запись Socket Interrupt (SIR)
uint8_t W5500_Set_SIR(uint8_t ir, W5500_User_Funcs* UF);
// Запись Socket Interrupt Mask (SIMR)
uint8_t W5500_Set_SIMR(uint8_t mask, W5500_User_Funcs* UF);
// Запись Retry Time (RTR)
uint8_t W5500_Set_RTR(uint16_t retry_time, W5500_User_Funcs* UF);
// Запись Retry Count (RCR)
uint8_t W5500_Set_RCR(uint8_t retry_count, W5500_User_Funcs* UF);
// Запись PPP LCP Request Timer (PTIMER)
uint8_t W5500_Set_PTIMER(uint8_t timer, W5500_User_Funcs* UF);
// Запись PPP LCP Magic Number (PMAGIC)
uint8_t W5500_Set_PMAGIC(uint8_t magic_number, W5500_User_Funcs* UF);
// Запись PPP Destination MAC Address (PHAR)
uint8_t W5500_Set_PHAR(const uint8_t* ppp_mac_address, W5500_User_Funcs* UF);
// Запись PPP Session Identification (PSID)
uint8_t W5500_Set_PSID(uint16_t session_id, W5500_User_Funcs* UF);
// Запись PPP Maximum Segment Size (PMRU)
uint8_t W5500_Set_PMRU(uint16_t max_segment_size, W5500_User_Funcs* UF);
// Запись PHY Configuration (PHYCFGR)
uint8_t W5500_Set_PHYCFGR(uint8_t phy_config, W5500_User_Funcs* UF);

// ======= Common Register READ ===============================================================
// Чтение Mode Register (MR)
uint8_t W5500_Get_MR(uint8_t* mode, W5500_User_Funcs* UF);
// Чтение Gateway Address Register (GAR)
uint8_t W5500_Get_Gateway(uint8_t* gateway_address, W5500_User_Funcs* UF);
// Чтение Subnet Mask Register (SUBR)
uint8_t W5500_Get_SubnetMask(uint8_t* subnet_mask, W5500_User_Funcs* UF);
// Чтение Source MAC Address Register (SHAR)
uint8_t W5500_Get_MAC(uint8_t* mac_address, W5500_User_Funcs* UF);
// Чтение Source IP Address Register (SIPR)
uint8_t W5500_Get_IP(uint8_t* ip_address, W5500_User_Funcs* UF);
// Чтение Interrupt Low Level Timer (INTLEVEL)
uint8_t W5500_Get_INTLEVEL(uint16_t* INTLEVEL, W5500_User_Funcs* UF);
// Чтение Interrupt (IR)
uint8_t W5500_Get_IR(uint8_t* IR, W5500_User_Funcs* UF);
// Чтение Interrupt Mask (IMR)
uint8_t W5500_Get_IMR(uint8_t* IMR, W5500_User_Funcs* UF);
// Чтение Socket Interrupt (SIR)
uint8_t W5500_Get_SIR(uint8_t* SIR, W5500_User_Funcs* UF);
// Чтение Socket Interrupt Mask (SIMR)
uint8_t W5500_Get_SIMR(uint8_t* SIMR, W5500_User_Funcs* UF);
// Чтение Retry Time (RTR)
uint8_t W5500_Get_RTR(uint16_t* RTR, W5500_User_Funcs* UF);
// Чтение Retry Count (RCR)
uint8_t W5500_Get_RCR(uint8_t* RCR, W5500_User_Funcs* UF);
// Чтение PPP LCP Request Timer (PTIMER)
uint8_t W5500_Get_PTIMER(uint8_t* PTIMER, W5500_User_Funcs* UF);
// Чтение PPP LCP Magic Number (PMAGIC)
uint8_t W5500_Get_PMAGIC(uint8_t* PMAGIC, W5500_User_Funcs* UF);
// Чтение PPP Destination MAC Address (PHAR)
uint8_t W5500_Get_PHAR(uint8_t* ppp_mac_address, W5500_User_Funcs* UF);
// Чтение PPP Session Identification (PSID)
uint8_t W5500_Get_PSID(uint16_t* PSID, W5500_User_Funcs* UF);
// Чтение PPP Maximum Segment Size (PMRU)
uint8_t W5500_Get_PMRU(uint16_t* PMRU, W5500_User_Funcs* UF);
// Чтение Unreachable IP Address (UIPR)
uint8_t W5500_Get_UIPR(uint8_t* unreachable_ip, W5500_User_Funcs* UF);
// Чтение Unreachable Port (UPORTR)
uint8_t W5500_Get_UPORTR(uint16_t* UPORTR, W5500_User_Funcs* UF);
// Чтение PHY Configuration (PHYCFGR)
uint8_t W5500_Get_PHYCFGR(uint8_t* PHYCFGR, W5500_User_Funcs* UF);
// Чтение Chip Version (ChipVers)
uint8_t W5500_Get_ChipVersion(uint8_t* ChipVersion, W5500_User_Funcs* UF);

// %%%%======= Socket Register INIT =======%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Запись Socket Mode Register (Sn_MR)
uint8_t W5500_Set_Sn_MR(uint8_t socket_num, uint8_t mode, W5500_User_Funcs* UF);
// Запись Socket Command Register (Sn_CR)
uint8_t W5500_Set_Sn_CR(uint8_t socket_num, uint8_t command, W5500_User_Funcs* UF);
// Запись Socket Interrupt Register  (Sn_IR)
uint8_t W5500_Set_Sn_IR(uint8_t socket_num, uint8_t interrupt, W5500_User_Funcs* UF);
// Запись Socket Source Port Register (Sn_PORT)
uint8_t W5500_Set_Sn_PORT(uint8_t socket_num, uint16_t port, W5500_User_Funcs* UF);
// Запись Socket Destination Hardware Address Register (Sn_DHAR)
uint8_t W5500_Set_Sn_DHAR(uint8_t socket_num, const uint8_t* dest_mac_address, W5500_User_Funcs* UF);
// Запись Socket Destination IP Address Register (Sn_DIPR)
uint8_t W5500_Set_Sn_DIPR(uint8_t socket_num, const uint8_t* dest_ip_address, W5500_User_Funcs* UF);
// Запись Socket Destination Port Register (Sn_DPORT)
uint8_t W5500_Set_Sn_DPORT(uint8_t socket_num, uint16_t dest_port, W5500_User_Funcs* UF);
// Запись Socket Maximum Segment Size Register (Sn_MSSR)
uint8_t W5500_Set_Sn_MSSR(uint8_t socket_num, uint16_t max_segment_size, W5500_User_Funcs* UF);
// Запись Socket IP Type of Service Register (Sn_TOS)
uint8_t W5500_Set_Sn_TOS(uint8_t socket_num, uint8_t tos, W5500_User_Funcs* UF);
// Запись Socket IP Time to Live Register (Sn_TTL)
uint8_t W5500_Set_Sn_TTL(uint8_t socket_num, uint8_t ttl, W5500_User_Funcs* UF);
// Запись Socket Receive Buffer Size (Sn_RXBUF_SIZE)
uint8_t W5500_Set_Sn_RXBUF_SIZE(uint8_t socket_num, uint8_t rx_buf_size, W5500_User_Funcs* UF);
// Запись Socket Transmit Buffer Size (Sn_TXBUF_SIZE)
uint8_t W5500_Set_Sn_TXBUF_SIZE(uint8_t socket_num, uint8_t tx_buf_size, W5500_User_Funcs* UF);
// Запись Socket Transmit Read Pointer Register (Sn_TX_RD)
uint8_t W5500_Set_Sn_TX_RD(uint8_t socket_num, uint16_t TX_RD, W5500_User_Funcs* UF);
// Запись Socket Transmit Write Pointer Register (Sn_TX_WR)
uint8_t W5500_Set_Sn_TX_WR(uint8_t socket_num, uint16_t TX_WR, W5500_User_Funcs* UF);
// Запись Socket Receive Read Pointer Register (RX_RD)
uint8_t W5500_Set_Sn_RX_RD(uint8_t socket_num, uint16_t RX_RD, W5500_User_Funcs* UF);
// Запись Socket Receive Write Pointer Register (RX_WR)
uint8_t W5500_Set_Sn_RX_WR(uint8_t socket_num, uint16_t RX_WR, W5500_User_Funcs* UF);
// Запись Socket Interrupt Mask (Sn_IMR)
uint8_t W5500_Set_Sn_IMR(uint8_t socket_num, uint8_t interrupt_mask, W5500_User_Funcs* UF);
// Запись Socket Fragment Offset in IP Header Register (Sn_FRAG)
uint8_t W5500_Set_Sn_FRAG(uint8_t socket_num, uint16_t fragment_offset, W5500_User_Funcs* UF);
// Запись Socket Keep Alive Timer Register (Sn_KPALVTR)
uint8_t W5500_Set_Sn_KPALVTR(uint8_t socket_num, uint8_t keep_alive_timer, W5500_User_Funcs* UF);

// ======= Socket Register READ ==============================================================
// Чтение Socket Mode Register (Sn_MR)
uint8_t W5500_Get_Sn_MR(uint8_t* MR, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Command Register (Sn_CR)
uint8_t W5500_Get_Sn_CR(uint8_t* CR, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Status Register (Sn_SR)
uint8_t W5500_Get_Sn_SR(uint8_t* SR, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Interrupt Register (Sn_IR)
uint8_t W5500_Get_Sn_IR(uint8_t* IR, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Source Port Register (Sn_PORT)
uint8_t W5500_Get_Sn_PORT(uint16_t* PORT, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Destination Hardware Address Register (Sn_DHAR)
uint8_t W5500_Get_Sn_DHAR(uint8_t socket_num, uint8_t* dest_mac_address, W5500_User_Funcs* UF);
// Чтение Socket Destination IP Address Register (Sn_DIPR)
uint8_t W5500_Get_Sn_DIPR(uint8_t socket_num, uint8_t* dest_ip_address, W5500_User_Funcs* UF);
// Чтение Socket Destination Port Register (Sn_DPORT)
uint8_t W5500_Get_Sn_DPORT(uint16_t* DPORT, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Maximum Segment Size Register (Sn_MSSR)
uint8_t W5500_Get_Sn_MSSR(uint16_t* MSSR, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Transmit Free Size Register (Sn_TX_FSR)
uint8_t W5500_Get_Sn_TX_FSR(uint16_t* FSR, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket IP Type of Service Register (Sn_TOS)
uint8_t W5500_Get_Sn_TOS(uint8_t* TOS, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket IP Time to Live Register (Sn_TTL)
uint8_t W5500_Get_Sn_TTL(uint8_t* TTL, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Receive Buffer Size (Sn_RXBUF_SIZE)
uint8_t W5500_Get_Sn_RXBUF_SIZE(uint8_t* RX_SIZE, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Transmit Buffer Size (Sn_TXBUF_SIZE)
uint8_t W5500_Get_Sn_TXBUF_SIZE(uint8_t* TX_SIZE, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Transmit Read Pointer Register (Sn_TX_RD)
uint8_t W5500_Get_Sn_TX_RD(uint16_t* TX_RD, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Transmit Write Pointer Register (Sn_TX_WR)
uint8_t W5500_Get_Sn_TX_WR(uint16_t* TX_WR, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Receive Received Size Register (Sn_RX_RSR)
uint8_t W5500_Get_Sn_RX_RSR(uint16_t* RX_RSR, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Receive Read Pointer Register (Sn_RX_RD)  ЛУЧШЕ НЕ ТРОГАТЬ
uint8_t W5500_Get_Sn_RX_RD(uint16_t* RX_RD, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Receive Write Pointer Register (Sn_RX_WR) ЛУЧШЕ НЕ ТРОГАТЬ
uint8_t W5500_Get_Sn_RX_WR(uint16_t* RX_WR, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Interrupt Mask Register (Sn_IMR)
uint8_t W5500_Get_Sn_IMR(uint8_t* IMR, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Fragment Offset in IP Header Register (Sn_FRAG)
uint8_t W5500_Get_Sn_FRAG(uint16_t* FRAG, uint8_t socket_num, W5500_User_Funcs* UF);
// Чтение Socket Keep Alive Timer Register (Sn_KPALVTR)
uint8_t W5500_Get_Sn_KPALVTR(uint8_t* KPALVTR, uint8_t socket_num, W5500_User_Funcs* UF);

// ======= Базовые функции чтения и записи данных ==============================================
// Запись Socket TXBUF
uint8_t W5500_Sn_TXBUF_Write(uint16_t address, uint8_t socket_num, const uint8_t* data, uint16_t len, W5500_User_Funcs* UF);
// Чтение Socket RXBUF
uint8_t W5500_Sn_RXBUF_Read(uint16_t address, uint8_t socket_num, uint8_t* data, uint16_t len, W5500_User_Funcs* UF);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Функции "быстрой" настройки %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// return 0 у всех функций = успешное выполнение
// return 1 - n = шаг неудачного выполнения

// Настройка Common Reg
uint8_t W5500_QuickInit_Common(
    const uint8_t*     mac_address,
    const uint8_t*     ip_address,
    const uint8_t*     subnet_mask,
    const uint8_t*     gateway_address,
    W5500_Main_Struct* MS);

enum W5500_buf_size {
    W5500_empty      = 0,
    W5500_two_kb     = 2,
    W5500_four_kb    = 4,
    W5500_eight_kb   = 8,
    W5500_sixteen_kb = 16
};

// Установка UDP сокета
uint8_t W5500_QuickInit_UDP(
    uint8_t             socket_num,
    uint16_t            src_port,
    enum W5500_buf_size tx_b_s,
    enum W5500_buf_size rx_b_s,
    uint8_t             keep_alive_timer,  // * 5s
    W5500_Main_Struct*  MS);

// Установка TCP сокета
uint8_t W5500_QuickInit_TCP(
    uint8_t             socket_num,
    uint16_t            src_port,
    enum W5500_buf_size tx_b_s,
    enum W5500_buf_size rx_b_s,
    uint8_t             keep_alive_timer,  // * 5s
    W5500_Main_Struct*  MS);

// TCP подключение
uint8_t W5500_TCP_Connect(
    uint8_t            socket_num,
    const uint8_t*     dest_ip,
    uint16_t           dest_port,
    W5500_Main_Struct* MS);

//%%%%%%%%%%%%%%%%%%%% Функции для работы с данными %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Отправка данных без обработчиков прерываний P.S. НЕ РЕКОМЕНДУЕТСЯ
uint8_t W5500_SendData(
    uint8_t            socket_num,
    const uint8_t*     dest_ip,
    uint16_t           dest_port,
    const uint8_t*     data,
    uint16_t           len,
    W5500_Main_Struct* MS);

// Отправка данных с обработчиками прерываний
//* func_mode - аналогичен отправке без прерываний
uint8_t W5500_SendData_IR(
    uint8_t            socket_num,
    const uint8_t*     dest_ip,
    uint16_t           dest_port,
    const uint8_t*     data,
    uint16_t           len,
    W5500_Main_Struct* MS);

// Приём данных
// Если длина буфера < длины данных, то прочитайте
// оставшуюся часть через эту функцию
// Возвращается размер данных в пакете
uint16_t W5500_ReceiveData(
    uint8_t            socket_num,
    uint8_t*           buffer,
    uint16_t           max_len,
    uint16_t*          buffer_len,
    W5500_Main_Struct* MS);

// Сокет переходит в режим TCP Сервера
//* Для выхода из режима закрйоте сокет
uint8_t W5500_TCP_Listen(
    uint8_t            socket_num,
    W5500_Main_Struct* MS);

//%%%%%%%%%%%%%%%%%%%%%%%%%%% Функции работы с соединением %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Открытие сокета
uint8_t W5500_OpenSocket(
    uint8_t            socket_num,
    W5500_Main_Struct* MS);

// Закрытие сокета
uint8_t W5500_CloseSocket(
    uint8_t            socket_num,
    W5500_Main_Struct* MS);

// TCP отключение
uint8_t W5500_TCP_Disconnect(
    uint8_t            socket_num,
    W5500_Main_Struct* MS);

//%%%%%%%%%%%%%%%%%%%%%%% Функции прерывания %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// обрабатываются прерывания всех сокетов
void W5500_IR_processing(
    uint8_t*           buf,
    uint16_t           size,
    W5500_Main_Struct* MS);

#endif /* INC_W5500_H_ */
