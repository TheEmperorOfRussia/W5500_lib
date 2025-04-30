/*
 * w5500.c
 *
 *  Created on: Mar 17, 2025
 *      Author: The emperor Arseni I
 */
#include "w5500.h"
#include "stddef.h"
#include "string.h"
uint8_t W5500_ReadRegister(
    uint16_t          address,
    uint8_t           bsb,
    uint8_t*          buffer,
    uint16_t          len,
    W5500_User_Funcs* UF)
{
    if (len < 1)
        return 2;  // Ошибка длины буфера чтения
    uint8_t control_byte    = (bsb << 3) | W5500_READ | 0b00;
    uint8_t addr_high       = (address >> 8) & 0xFF;                  // Старший байт адреса
    uint8_t addr_low        = address & 0xFF;                         // Младший байт адреса
    uint8_t addr_and_con[3] = { addr_high, addr_low, control_byte };  // Массив первых 3 байт

    UF->CS(1);
    if (UF->Transmit(addr_and_con, 3) != 0) {
        return 1;
    }
    if (UF->Receive(buffer, len) != 0) {
        return 1;
    }
    UF->CS(0);

    return 0;
}

uint8_t W5500_WriteRegister(
    uint16_t          address,
    uint8_t           bsb,
    const uint8_t*    data,
    uint16_t          len,
    W5500_User_Funcs* UF)
{
    if (len < 1)
        return 2;  // Ошибка длины буфера записи
    uint8_t control_byte    = (bsb << 3) | W5500_WRITE | 0b00;
    uint8_t addr_high       = (address >> 8) & 0xFF;                  // Старший байт адреса
    uint8_t addr_low        = address & 0xFF;                         // Младший байт адреса
    uint8_t addr_and_con[3] = { addr_high, addr_low, control_byte };  // Массив первых 3 байт

    UF->CS(1);
    if (UF->Transmit(addr_and_con, 3) != 0) {
        return 1;
    }
    if (UF->Transmit(data, len) != 0) {
        return 1;
    }
    UF->CS(0);

    return 0;
}
// %%%%======= Common Register INIT =======%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Запись Mode Register (MR)
uint8_t W5500_Set_MR(uint8_t mode, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_MR, W5500_COMMON_REG, &mode, 1, UF);
}

// Запись Gateway Address Register (GAR)
uint8_t W5500_Set_Gateway(const uint8_t* gateway_address, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_GAR, W5500_COMMON_REG, gateway_address, 4, UF);
}

// Запись Subnet Mask Register (SUBR)
uint8_t W5500_Set_Mask(const uint8_t* mask_address, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_SUBR, W5500_COMMON_REG, mask_address, 4, UF);
}

// Запись Source MAC Address Register (SHAR)
uint8_t W5500_Set_MAC(const uint8_t* mac_address, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_SHAR, W5500_COMMON_REG, mac_address, 6, UF);
}

// Запись Source IP Address Register (SIPR)
uint8_t W5500_Set_IP(const uint8_t* ip_address, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_SIPR, W5500_COMMON_REG, ip_address, 4, UF);
}

// Запись Interrupt Low Level Timer (INTLEVEL)
uint8_t W5500_Set_INTLEVEL(uint16_t int_level, W5500_User_Funcs* UF)
{
    uint8_t data[2] = { (int_level >> 8) & 0xFF, int_level & 0xFF };
    return W5500_WriteRegister(W5500_INTLEVEL, W5500_COMMON_REG, data, 2, UF);
}

// Запись Interrupt Mask (IMR)
uint8_t W5500_Set_IMR(uint8_t mask, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_IMR, W5500_COMMON_REG, &mask, 1, UF);
}

uint8_t W5500_Set_SIR(uint8_t ir, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_IR, W5500_COMMON_REG, &ir, 1, UF);
}

// Запись Socket Interrupt Mask (SIMR)
uint8_t W5500_Set_SIMR(uint8_t mask, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_SIMR, W5500_COMMON_REG, &mask, 1, UF);
}

// Запись Retry Time (RTR)
uint8_t W5500_Set_RTR(uint16_t retry_time, W5500_User_Funcs* UF)
{
    uint8_t data[2] = { (retry_time >> 8) & 0xFF, retry_time & 0xFF };
    return W5500_WriteRegister(W5500_RTR, W5500_COMMON_REG, data, 2, UF);
}

// Запись Retry Count (RCR)
uint8_t W5500_Set_RCR(uint8_t retry_count, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_RCR, W5500_COMMON_REG, &retry_count, 1, UF);
}

// Запись PPP LCP Request Timer (PTIMER)
uint8_t W5500_Set_PTIMER(uint8_t timer, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_PTIMER, W5500_COMMON_REG, &timer, 1, UF);
}

// Запись PPP LCP Magic Number (PMAGIC)
uint8_t W5500_Set_PMAGIC(uint8_t magic_number, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_PMAGIC, W5500_COMMON_REG, &magic_number, 1, UF);
}

// Запись PPP Destination MAC Address (PHAR)
uint8_t W5500_Set_PHAR(const uint8_t* ppp_mac_address, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_PHAR, W5500_COMMON_REG, ppp_mac_address, 6, UF);
}

// Запись PPP Session Identification (PSID)
uint8_t W5500_Set_PSID(uint16_t session_id, W5500_User_Funcs* UF)
{
    uint8_t data[2] = { (session_id >> 8) & 0xFF, session_id & 0xFF };
    return W5500_WriteRegister(W5500_PSID, W5500_COMMON_REG, data, 2, UF);
}

// Запись PPP Maximum Segment Size (PMRU)
uint8_t W5500_Set_PMRU(uint16_t max_segment_size, W5500_User_Funcs* UF)
{
    uint8_t data[2] = { (max_segment_size >> 8) & 0xFF, max_segment_size & 0xFF };
    return W5500_WriteRegister(W5500_PMRU, W5500_COMMON_REG, data, 2, UF);
}

// Запись PHY Configuration (PHYCFGR)
uint8_t W5500_Set_PHYCFGR(uint8_t phy_config, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_PHYCFGR, W5500_COMMON_REG, &phy_config, 1, UF);
}

// ======= Common Register READ ===============================================================

// Чтение Mode Register (MR)
uint8_t W5500_Get_MR(W5500_User_Funcs* UF)
{
    uint8_t mode;
    uint8_t result = W5500_ReadRegister(W5500_MR, W5500_COMMON_REG, &mode, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return mode;
}

// Чтение Gateway Address Register (GAR)
uint8_t W5500_Get_Gateway(uint8_t* gateway_address, W5500_User_Funcs* UF)
{
    return W5500_ReadRegister(W5500_GAR, W5500_COMMON_REG, gateway_address, 4, UF);
}

// Чтение Subnet Mask Register (SUBR)
uint8_t W5500_Get_SubnetMask(uint8_t* subnet_mask, W5500_User_Funcs* UF)
{
    return W5500_ReadRegister(W5500_SUBR, W5500_COMMON_REG, subnet_mask, 4, UF);
}

// Чтение Source MAC Address Register (SHAR)
uint8_t W5500_Get_MAC(uint8_t* mac_address, W5500_User_Funcs* UF)
{
    return W5500_ReadRegister(W5500_SHAR, W5500_COMMON_REG, mac_address, 6, UF);
}

// Чтение Source IP Address Register (SIPR)
uint8_t W5500_Get_IP(uint8_t* ip_address, W5500_User_Funcs* UF)
{
    return W5500_ReadRegister(W5500_SIPR, W5500_COMMON_REG, ip_address, 4, UF);
}

// Чтение Interrupt Low Level Timer (INTLEVEL)
uint16_t W5500_Get_INTLEVEL(W5500_User_Funcs* UF)
{
    uint8_t data[2];
    uint8_t result = W5500_ReadRegister(W5500_INTLEVEL, W5500_COMMON_REG, data, 2, UF);
    if (result != 0)
        return result;  // Ошибка
    return (data[0] << 8) | data[1];
}

// Чтение Interrupt (IR)
uint8_t W5500_Get_IR(W5500_User_Funcs* UF)
{
    uint8_t interrupt;
    uint8_t result = W5500_ReadRegister(W5500_IR, W5500_COMMON_REG, &interrupt, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return interrupt;
}

// Чтение Interrupt Mask (IMR)
uint8_t W5500_Get_IMR(W5500_User_Funcs* UF)
{
    uint8_t mask;
    uint8_t result = W5500_ReadRegister(W5500_IMR, W5500_COMMON_REG, &mask, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return mask;
}

// Чтение Socket Interrupt (SIR)
uint8_t W5500_Get_SIR(W5500_User_Funcs* UF)
{
    uint8_t socket_interrupt;
    uint8_t result = W5500_ReadRegister(W5500_SIR, W5500_COMMON_REG, &socket_interrupt, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return socket_interrupt;
}

// Чтение Socket Interrupt Mask (SIMR)
uint8_t W5500_Get_SIMR(W5500_User_Funcs* UF)
{
    uint8_t socket_mask;
    uint8_t result = W5500_ReadRegister(W5500_SIMR, W5500_COMMON_REG, &socket_mask, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return socket_mask;
}

// Чтение Retry Time (RTR)
uint16_t W5500_Get_RTR(W5500_User_Funcs* UF)
{
    uint8_t data[2];
    uint8_t result = W5500_ReadRegister(W5500_RTR, W5500_COMMON_REG, data, 2, UF);
    if (result != 0)
        return result;  // Ошибка
    return (data[0] << 8) | data[1];
}

// Чтение Retry Count (RCR)
uint8_t W5500_Get_RCR(W5500_User_Funcs* UF)
{
    uint8_t retry_count;
    uint8_t result = W5500_ReadRegister(W5500_RCR, W5500_COMMON_REG, &retry_count, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return retry_count;
}

// Чтение PPP LCP Request Timer (PTIMER)
uint8_t W5500_Get_PTIMER(W5500_User_Funcs* UF)
{
    uint8_t timer;
    uint8_t result = W5500_ReadRegister(W5500_PTIMER, W5500_COMMON_REG, &timer, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return timer;
}

// Чтение PPP LCP Magic Number (PMAGIC)
uint8_t W5500_Get_PMAGIC(W5500_User_Funcs* UF)
{
    uint8_t magic_number;
    uint8_t result = W5500_ReadRegister(W5500_PMAGIC, W5500_COMMON_REG, &magic_number, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return magic_number;
}

// Чтение PPP Destination MAC Address (PHAR)
uint8_t W5500_Get_PHAR(uint8_t* ppp_mac_address, W5500_User_Funcs* UF)
{
    return W5500_ReadRegister(W5500_PHAR, W5500_COMMON_REG, ppp_mac_address, 6, UF);
}

// Чтение PPP Session Identification (PSID)
uint16_t W5500_Get_PSID(W5500_User_Funcs* UF)
{
    uint8_t data[2];
    uint8_t result = W5500_ReadRegister(W5500_PSID, W5500_COMMON_REG, data, 2, UF);
    if (result != 0)
        return result;  // Ошибка
    return (data[0] << 8) | data[1];
}

// Чтение PPP Maximum Segment Size (PMRU)
uint16_t W5500_Get_PMRU(W5500_User_Funcs* UF)
{
    uint8_t data[2];
    uint8_t result = W5500_ReadRegister(W5500_PMRU, W5500_COMMON_REG, data, 2, UF);
    if (result != 0)
        return result;  // Ошибка
    return (data[0] << 8) | data[1];
}

// Чтение Unreachable IP Address (UIPR)
uint8_t W5500_Get_UIPR(uint8_t* unreachable_ip, W5500_User_Funcs* UF)
{
    return W5500_ReadRegister(W5500_UIPR, W5500_COMMON_REG, unreachable_ip, 4, UF);
}

// Чтение Unreachable Port (UPORTR)
uint16_t W5500_Get_UPORTR(W5500_User_Funcs* UF)
{
    uint8_t data[2];
    uint8_t result = W5500_ReadRegister(W5500_UPORTR, W5500_COMMON_REG, data, 2, UF);
    if (result != 0)
        return result;  // Ошибка
    return (data[0] << 8) | data[1];
}

// Чтение PHY Configuration (PHYCFGR)
uint8_t W5500_Get_PHYCFGR(W5500_User_Funcs* UF)
{
    uint8_t phy_config;
    uint8_t result = W5500_ReadRegister(W5500_PHYCFGR, W5500_COMMON_REG, &phy_config, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return phy_config;
}

// Чтение Chip Version (ChipVers)
uint8_t W5500_Get_ChipVersion(W5500_User_Funcs* UF)
{
    uint8_t chip_version;
    uint8_t result = W5500_ReadRegister(W5500_ChipVers, W5500_COMMON_REG, &chip_version, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return chip_version;
}

// %%%%======= Socket Register INIT =======%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Запись Socket Mode Register (Sn_MR)
uint8_t W5500_Set_Sn_MR(uint8_t socket_num, uint8_t mode, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_Sn_MR, W5500_SOCKET_BASE(socket_num), &mode, 1, UF);
}

// Запись Socket Command Register (Sn_CR)
uint8_t W5500_Set_Sn_CR(uint8_t socket_num, uint8_t command, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_Sn_CR, W5500_SOCKET_BASE(socket_num), &command, 1, UF);
}

// Запись Socket Interrupt Register  (Sn_IR)
uint8_t W5500_Set_Sn_IR(uint8_t socket_num, uint8_t interrupt, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_Sn_IR, W5500_SOCKET_BASE(socket_num), &interrupt, 1, UF);
}

// Запись Socket Source Port Register (Sn_PORT)
uint8_t W5500_Set_Sn_PORT(uint8_t socket_num, uint16_t port, W5500_User_Funcs* UF)
{
    uint8_t data[2] = { (port >> 8) & 0xFF, port & 0xFF };
    return W5500_WriteRegister(W5500_Sn_PORT, W5500_SOCKET_BASE(socket_num), data, 2, UF);
}

// Запись Socket Destination Hardware Address Register (Sn_DHAR)
uint8_t W5500_Set_Sn_DHAR(uint8_t socket_num, const uint8_t* dest_mac_address, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_Sn_DHAR, W5500_SOCKET_BASE(socket_num), dest_mac_address, 6, UF);
}

// Запись Socket Destination IP Address Register (Sn_DIPR)
uint8_t W5500_Set_Sn_DIPR(uint8_t socket_num, const uint8_t* dest_ip_address, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_Sn_DIPR, W5500_SOCKET_BASE(socket_num), dest_ip_address, 4, UF);
}

// Запись Socket Destination Port Register (Sn_DPORT)
uint8_t W5500_Set_Sn_DPORT(uint8_t socket_num, uint16_t dest_port, W5500_User_Funcs* UF)
{
    uint8_t data[2] = { (dest_port >> 8) & 0xFF, dest_port & 0xFF };
    return W5500_WriteRegister(W5500_Sn_DPORT, W5500_SOCKET_BASE(socket_num), data, 2, UF);
}

// Запись Socket Maximum Segment Size Register (Sn_MSSR)
uint8_t W5500_Set_Sn_MSSR(uint8_t socket_num, uint16_t max_segment_size, W5500_User_Funcs* UF)
{
    uint8_t data[2] = { (max_segment_size >> 8) & 0xFF, max_segment_size & 0xFF };
    return W5500_WriteRegister(W5500_Sn_MSSR, W5500_SOCKET_BASE(socket_num), data, 2, UF);
}

// Запись Socket IP Type of Service Register (Sn_TOS)
uint8_t W5500_Set_Sn_TOS(uint8_t socket_num, uint8_t tos, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_Sn_TOS, W5500_SOCKET_BASE(socket_num), &tos, 1, UF);
}

// Запись Socket IP Time to Live Register (Sn_TTL)
uint8_t W5500_Set_Sn_TTL(uint8_t socket_num, uint8_t ttl, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_Sn_TTL, W5500_SOCKET_BASE(socket_num), &ttl, 1, UF);
}

// Запись Socket Receive Buffer Size (Sn_RXBUF_SIZE)
uint8_t W5500_Set_Sn_RXBUF_SIZE(uint8_t socket_num, uint8_t rx_buf_size, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_Sn_RXBUF_SIZE, W5500_SOCKET_BASE(socket_num), &rx_buf_size, 1, UF);
}

// Запись Socket Transmit Buffer Size (Sn_TXBUF_SIZE)
uint8_t W5500_Set_Sn_TXBUF_SIZE(uint8_t socket_num, uint8_t tx_buf_size, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_Sn_TXBUF_SIZE, W5500_SOCKET_BASE(socket_num), &tx_buf_size, 1, UF);
}

// Запись Socket Transmit Read Pointer Register (Sn_TX_RD)
uint8_t W5500_Set_Sn_TX_RD(uint8_t socket_num, uint16_t TX_RD, W5500_User_Funcs* UF)
{
    uint8_t data[2] = { (TX_RD >> 8) & 0xFF, TX_RD & 0xFF };
    return W5500_WriteRegister(W5500_Sn_TX_RD, W5500_SOCKET_BASE(socket_num), data, 2, UF);
}

// Запись Socket Transmit Write Pointer Register (Sn_TX_WR)
uint8_t W5500_Set_Sn_TX_WR(uint8_t socket_num, uint16_t TX_WR, W5500_User_Funcs* UF)
{
    uint8_t data[2] = { (TX_WR >> 8) & 0xFF, TX_WR & 0xFF };
    return W5500_WriteRegister(W5500_Sn_TX_WR, W5500_SOCKET_BASE(socket_num), data, 2, UF);
}

// Запись Socket Receive Read Pointer Register (RX_RD)
uint8_t W5500_Set_Sn_RX_RD(uint8_t socket_num, uint16_t RX_RD, W5500_User_Funcs* UF)
{
    uint8_t data[2] = { (RX_RD >> 8) & 0xFF, RX_RD & 0xFF };
    return W5500_WriteRegister(W5500_Sn_RX_RD, W5500_SOCKET_BASE(socket_num), data, 2, UF);
}

// Запись Socket Receive Write Pointer Register (RX_WR)
uint8_t W5500_Set_Sn_RX_WR(uint8_t socket_num, uint16_t RX_WR, W5500_User_Funcs* UF)
{
    uint8_t data[2] = { (RX_WR >> 8) & 0xFF, RX_WR & 0xFF };
    return W5500_WriteRegister(W5500_Sn_RX_WR, W5500_SOCKET_BASE(socket_num), data, 2, UF);
}

// Запись Socket Interrupt Mask (Sn_IMR)
uint8_t W5500_Set_Sn_IMR(uint8_t socket_num, uint8_t interrupt_mask, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_Sn_IMR, W5500_SOCKET_BASE(socket_num), &interrupt_mask, 1, UF);
}

// Запись Socket Fragment Offset in IP Header Register (Sn_FRAG)
uint8_t W5500_Set_Sn_FRAG(uint8_t socket_num, uint16_t fragment_offset, W5500_User_Funcs* UF)
{
    uint8_t data[2] = { (fragment_offset >> 8) & 0xFF, fragment_offset & 0xFF };
    return W5500_WriteRegister(W5500_Sn_FRAG, W5500_SOCKET_BASE(socket_num), data, 2, UF);
}

// Запись Socket Keep Alive Timer Register (Sn_KPALVTR)
uint8_t W5500_Set_Sn_KPALVTR(uint8_t socket_num, uint8_t keep_alive_timer, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(W5500_Sn_KPALVTR, W5500_SOCKET_BASE(socket_num), &keep_alive_timer, 1, UF);
}

// ======= Socket Register READ ==============================================================

// Чтение Socket Mode Register (Sn_MR)
uint8_t W5500_Get_Sn_MR(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t mode;
    uint8_t result = W5500_ReadRegister(W5500_Sn_MR, W5500_SOCKET_BASE(socket_num), &mode, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return mode;
}

// Чтение Socket Command Register (Sn_CR)
uint8_t W5500_Get_Sn_CR(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t command;
    uint8_t result = W5500_ReadRegister(W5500_Sn_CR, socket_num, &command, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return command;
}

// Чтение Socket Status Register (Sn_SR)
uint8_t W5500_Get_Sn_SR(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t status;
    uint8_t result = W5500_ReadRegister(W5500_Sn_SR, W5500_SOCKET_BASE(socket_num), &status, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return status;
}

// Чтение Socket Interrupt Register (Sn_IR)
uint8_t W5500_Get_Sn_IR(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t interrupt;
    uint8_t result = W5500_ReadRegister(W5500_Sn_IR, W5500_SOCKET_BASE(socket_num), &interrupt, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return interrupt;
}

// Чтение Socket Source Port Register (Sn_PORT)
uint16_t W5500_Get_Sn_PORT(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t data[2];
    W5500_ReadRegister(W5500_Sn_PORT, W5500_SOCKET_BASE(socket_num), data, 2, UF);
    return (data[0] << 8) | data[1];  // Возвращаем значение как 16-битное число
}

// Чтение Socket Destination Hardware Address Register (Sn_DHAR)
void W5500_Get_Sn_DHAR(uint8_t socket_num, uint8_t* dest_mac_address, W5500_User_Funcs* UF)
{
    W5500_ReadRegister(W5500_Sn_DHAR, W5500_SOCKET_BASE(socket_num), dest_mac_address, 6, UF);
}

// Чтение Socket Destination IP Address Register (Sn_DIPR)
void W5500_Get_Sn_DIPR(uint8_t socket_num, uint8_t* dest_ip_address, W5500_User_Funcs* UF)
{
    W5500_ReadRegister(W5500_Sn_DIPR, W5500_SOCKET_BASE(socket_num), dest_ip_address, 4, UF);
}

// Чтение Socket Destination Port Register (Sn_DPORT)
uint16_t W5500_Get_Sn_DPORT(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t data[2];
    W5500_ReadRegister(W5500_Sn_DPORT, W5500_SOCKET_BASE(socket_num), data, 2, UF);
    return (data[0] << 8) | data[1];  // Возвращаем значение как 16-битное число
}

// Чтение Socket Maximum Segment Size Register (Sn_MSSR)
uint16_t W5500_Get_Sn_MSSR(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t data[2];
    W5500_ReadRegister(W5500_Sn_MSSR, W5500_SOCKET_BASE(socket_num), data, 2, UF);
    return (data[0] << 8) | data[1];  // Возвращаем значение как 16-битное число
}

// Чтение Socket Transmit Free Size Register (Sn_TX_FSR)
uint16_t W5500_Get_Sn_TX_FSR(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t data[2];
    uint8_t result = W5500_ReadRegister(W5500_Sn_TX_FSR, W5500_SOCKET_BASE(socket_num), data, 2, UF);
    if (result != 0)
        return result;  // Ошибка
    return (data[0] << 8) | data[1];
}

// Чтение Socket IP Type of Service Register (Sn_TOS)
uint8_t W5500_Get_Sn_TOS(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t tos;
    uint8_t result = W5500_ReadRegister(W5500_Sn_TOS, W5500_SOCKET_BASE(socket_num), &tos, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return tos;
}

// Чтение Socket IP Time to Live Register (Sn_TTL)
uint8_t W5500_Get_Sn_TTL(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t ttl;
    uint8_t result = W5500_ReadRegister(W5500_Sn_TTL, W5500_SOCKET_BASE(socket_num), &ttl, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return ttl;
}

// Чтение Socket Receive Buffer Size (Sn_RXBUF_SIZE)
uint8_t W5500_Get_Sn_RXBUF_SIZE(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t rx_buf_size;
    uint8_t result = W5500_ReadRegister(W5500_Sn_RXBUF_SIZE, W5500_SOCKET_BASE(socket_num), &rx_buf_size, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return rx_buf_size;
}

// Чтение Socket Transmit Buffer Size (Sn_TXBUF_SIZE)
uint8_t W5500_Get_Sn_TXBUF_SIZE(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t tx_buf_size;
    uint8_t result = W5500_ReadRegister(W5500_Sn_TXBUF_SIZE, W5500_SOCKET_BASE(socket_num), &tx_buf_size, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return tx_buf_size;
}

// Чтение Socket Transmit Read Pointer Register (Sn_TX_RD)
uint16_t W5500_Get_Sn_TX_RD(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t data[2];
    uint8_t result = W5500_ReadRegister(W5500_Sn_TX_RD, W5500_SOCKET_BASE(socket_num), data, 2, UF);
    if (result != 0)
        return result;  // Ошибка
    return (data[0] << 8) | data[1];
}

// Чтение Socket Transmit Write Pointer Register (Sn_TX_WR)
uint16_t W5500_Get_Sn_TX_WR(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t data[2];
    uint8_t result = W5500_ReadRegister(W5500_Sn_TX_WR, W5500_SOCKET_BASE(socket_num), data, 2, UF);
    if (result != 0)
        return result;  // Ошибка
    return (data[0] << 8) | data[1];
}

// Чтение Socket Receive Received Size Register (Sn_RX_RSR)
uint16_t W5500_Get_Sn_RX_RSR(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t data[2];
    uint8_t result = W5500_ReadRegister(W5500_Sn_RX_RSR, W5500_SOCKET_BASE(socket_num), data, 2, UF);
    if (result != 0)
        return result;  // Ошибка
    return (data[0] << 8) | data[1];
}

// Чтение Socket Receive Read Pointer Register (Sn_RX_RD)
uint16_t W5500_Get_Sn_RX_RD(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t data[2];
    uint8_t result = W5500_ReadRegister(W5500_Sn_RX_RD, W5500_SOCKET_BASE(socket_num), data, 2, UF);
    if (result != 0)
        return result;  // Ошибка
    return (data[0] << 8) | data[1];
}

// Чтение Socket Receive Write Pointer Register (Sn_RX_WR)
uint16_t W5500_Get_Sn_RX_WR(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t data[2];
    uint8_t result = W5500_ReadRegister(W5500_Sn_RX_WR, W5500_SOCKET_BASE(socket_num), data, 2, UF);
    if (result != 0)
        return result;  // Ошибка
    return (data[0] << 8) | data[1];
}

// Чтение Socket Interrupt Mask Register (Sn_IMR)
uint8_t W5500_Get_Sn_IMR(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t interrupt_mask;  // * 5s
    uint8_t result = W5500_ReadRegister(W5500_Sn_IMR, W5500_SOCKET_BASE(socket_num), &interrupt_mask, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return interrupt_mask;
}

// Чтение Socket Fragment Offset in IP Header Register (Sn_FRAG)
uint16_t W5500_Get_Sn_FRAG(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t data[2];
    uint8_t result = W5500_ReadRegister(W5500_Sn_FRAG, W5500_SOCKET_BASE(socket_num), data, 2, UF);
    if (result != 0)
        return result;  // Ошибка
    return (data[0] << 8) | data[1];
}

// Чтение Socket Keep Alive Timer Register (Sn_KPALVTR)
uint8_t W5500_Get_Sn_KPALVTR(uint8_t socket_num, W5500_User_Funcs* UF)
{
    uint8_t keep_alive_timer;  // * 5s
    uint8_t result = W5500_ReadRegister(W5500_Sn_KPALVTR, W5500_SOCKET_BASE(socket_num), &keep_alive_timer, 1, UF);
    if (result != 0)
        return result;  // Ошибка
    return keep_alive_timer;
}

// ======= Базовые функции чтения и записи данных ==============================================

uint8_t W5500_Sn_TXBUF_Write(uint16_t address, uint8_t socket_num, const uint8_t* data, uint16_t len, W5500_User_Funcs* UF)
{
    return W5500_WriteRegister(address, W5500_SOCKET_TXBUF(socket_num), data, len, UF);
}

uint8_t W5500_Sn_RXBUF_Read(uint16_t address, uint8_t socket_num, uint8_t* data, uint16_t len, W5500_User_Funcs* UF)
{
    return W5500_ReadRegister(address, W5500_SOCKET_RXBUF(socket_num), data, len, UF);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Функции "быстрой" настройки %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// return 0 у всех функций = успешное выполнение
// return 1 - n = шаг неудачного выполнения

// Настройка Common Reg
uint8_t W5500_QuickInit_Common(
    const uint8_t*     mac_address,
    const uint8_t*     ip_address,
    const uint8_t*     subnet_mask,
    const uint8_t*     gateway_address,
    W5500_Main_Struct* MS)
{
    // Настройка Mode Register (MR) - Включение режима сброса
    if (W5500_Set_MR(0x80, MS->UF) != 0)
        return 1;

    MS->UF->Delay(1000);

    // Установка MAC-адреса
    if (W5500_Set_MAC(mac_address, MS->UF) != 0)
        return 2;

    // Установка IP-адреса
    if (W5500_Set_IP(ip_address, MS->UF) != 0)
        return 3;

    // Установка маски подсети
    if (W5500_Set_Mask(subnet_mask, MS->UF) != 0)
        return 4;

    // Установка шлюза
    if (W5500_Set_Gateway(gateway_address, MS->UF) != 0)
        return 5;

    // Настройка PHY Configuration (автоматическое согласование скорости)
    if (W5500_Set_PHYCFGR(0b10111111, MS->UF) != 0)
        return 6;

    return 0;  // Успешная инициализация
}

// Установка UDP сокета
uint8_t W5500_QuickInit_UDP(
    uint8_t            socket_num,
    uint16_t           src_port,
    uint16_t           tx_b_s,            // tx_buf_size
    uint16_t           rx_b_s,            // rx_buf_size
    uint8_t            keep_alive_timer,  // * 5s
    W5500_Main_Struct* MS)
{
    // Проверка: tx_b_s = 0 ИЛИ tx_b_s степень двойки и меньше 16
    if (!(tx_b_s == 0 || (((tx_b_s & (tx_b_s - 1)) == 0) && tx_b_s < 0x10))) {
        return 1;  // неверный размер tx буфера
    }

    // Проверка: rx_b_s = 0 ИЛИ rx_b_s степень двойки и меньше 16
    if (!(rx_b_s == 0 || (((rx_b_s & (rx_b_s - 1)) == 0) && rx_b_s < 0x10))) {
        return 2;  // неверный размер rx буфера
    }

    // Установка режима сокета в UDP
    if (W5500_Set_Sn_MR(socket_num, 0x02, MS->UF) != 0)
        return 3;

    // Установка исходного порта
    if (W5500_Set_Sn_PORT(socket_num, src_port, MS->UF) != 0)
        return 4;

    // Установка размера TX буфера
    if (W5500_Set_Sn_TXBUF_SIZE(socket_num, tx_b_s, MS->UF) != 0)
        return 5;

    // Установка размера RX буфера
    if (W5500_Set_Sn_RXBUF_SIZE(socket_num, rx_b_s, MS->UF) != 0)
        return 6;

    // Установка указателя TX_RD в 0
    if (W5500_Set_Sn_TX_RD(socket_num, 0, MS->UF) != 0)
        return 7;

    // Установка указателя TX_WR в 0
    if (W5500_Set_Sn_TX_WR(socket_num, 0, MS->UF) != 0)
        return 8;

    // Установка указателя RX_RD в 0
    if (W5500_Set_Sn_RX_RD(socket_num, 0, MS->UF) != 0)
        return 9;

    // Установка указателя RX_WR в 0
    if (W5500_Set_Sn_RX_WR(socket_num, 0, MS->UF) != 0)
        return 10;

    // Установка масок прерывания
    W5500_Set_SIMR(0x1 << socket_num, MS->UF);

    W5500_Set_Sn_IMR(socket_num, 0x1F, MS->UF);

    // Открытие сокета
    if (W5500_Set_Sn_CR(socket_num, 0x01, MS->UF) != 0)
        return 11;

    MS->UF->sockets[socket_num].num              = socket_num;
    MS->UF->sockets[socket_num].mode             = 0x2;
    MS->UF->sockets[socket_num].port             = src_port;
    MS->UF->sockets[socket_num].rx_buf_size      = rx_b_s;
    MS->UF->sockets[socket_num].tx_buf_size      = tx_b_s;
    MS->UF->sockets[socket_num].tx_rd_p          = 0;
    MS->UF->sockets[socket_num].tx_wr_p          = 0;
    MS->UF->sockets[socket_num].rx_rd_p          = 0;
    MS->UF->sockets[socket_num].rx_wr_p          = 0;
    MS->UF->sockets[socket_num].keep_alive_timer = keep_alive_timer;

    // Проверка состояния сокета
    uint8_t status;
    for (int i = 0; i < 10; i++) {
        status = W5500_Get_Sn_SR(socket_num, MS->UF);
        if (status == 0x22)  // SOCK_UDP
        {
            MS->UF->sockets[socket_num].status = status;
            return 0;
        }
        MS->UF->Delay(100);
    }

    return 12;  // Ошибка инициализации
}

// Установка TCP сокета
uint8_t W5500_QuickInit_TCP(
    uint8_t            socket_num,
    uint16_t           src_port,
    uint16_t           tx_b_s,            // tx_buf_size
    uint16_t           rx_b_s,            // rx_buf_size
    uint8_t            keep_alive_timer,  // * 5s
    W5500_Main_Struct* MS)
{

    // Проверка: tx_b_s = 0 ИЛИ tx_b_s степень двойки и меньше 16
    if (!(tx_b_s == 0 || (((tx_b_s & (tx_b_s - 1)) == 0) && tx_b_s < 0x10))) {
        return 1;  // неверный размер tx буфера
    }

    // Проверка: rx_b_s = 0 ИЛИ rx_b_s степень двойки и меньше 16
    if (!(rx_b_s == 0 || (((rx_b_s & (rx_b_s - 1)) == 0) && rx_b_s < 0x10))) {
        return 2;  // неверный размер rx буфера
    }

    // Установка режима сокета в TCP
    if (W5500_Set_Sn_MR(socket_num, 0x01, MS->UF) != 0)
        return 3;

    // Установка исходного порта
    if (W5500_Set_Sn_PORT(socket_num, src_port, MS->UF) != 0)
        return 4;

    // Установка времени "пинга"
    if (W5500_Set_Sn_KPALVTR(socket_num, keep_alive_timer, MS->UF) != 0)
        return 5;

    // Установка размера TX буфера
    if (W5500_Set_Sn_TXBUF_SIZE(socket_num, tx_b_s, MS->UF) != 0)
        return 6;

    // Установка размера RX буфера
    if (W5500_Set_Sn_RXBUF_SIZE(socket_num, rx_b_s, MS->UF) != 0)
        return 7;

    // Установка указателя RX_RD в 0
    if (W5500_Set_Sn_RX_RD(socket_num, 0, MS->UF) != 0)
        return 8;

    // Установка указателя RX_WR в 0
    if (W5500_Set_Sn_RX_WR(socket_num, 0, MS->UF) != 0)
        return 9;

    W5500_Set_SIMR(0x1 << socket_num, MS->UF);
    W5500_Set_Sn_IMR(socket_num, 0x1F, MS->UF);

    // Открытие сокета
    if (W5500_Set_Sn_CR(socket_num, 0x01, MS->UF) != 0)
        return 10;

    MS->UF->sockets[socket_num].num              = socket_num;
    MS->UF->sockets[socket_num].mode             = 0x1;
    MS->UF->sockets[socket_num].port             = src_port;
    MS->UF->sockets[socket_num].rx_buf_size      = rx_b_s;
    MS->UF->sockets[socket_num].tx_buf_size      = tx_b_s;
    MS->UF->sockets[socket_num].tx_rd_p          = 0;
    MS->UF->sockets[socket_num].tx_wr_p          = 0;
    MS->UF->sockets[socket_num].rx_rd_p          = 0;
    MS->UF->sockets[socket_num].rx_wr_p          = 0;
    MS->UF->sockets[socket_num].keep_alive_timer = keep_alive_timer;

    // Проверка состояния сокета
    uint8_t status;
    for (int i = 0; i < 10; i++) {
        status = W5500_Get_Sn_SR(socket_num, MS->UF);
        if (status == 0x13)  // SOCK_TCP
        {
            MS->UF->sockets[socket_num].status = status;
            return 0;
        }
        MS->UF->Delay(100);
    }

    return 11;  // Успешная инициализация
}

// TCP подключение
uint8_t W5500_TCP_Connect(
    uint8_t            socket_num,
    const uint8_t*     dest_ip,
    uint16_t           dest_port,
    W5500_Main_Struct* MS)
{

    // Установка IP-адреса назначения
    if (W5500_Set_Sn_DIPR(socket_num, dest_ip, MS->UF) != 0)
        return 1;

    // Установка порта назначения
    if (W5500_Set_Sn_DPORT(socket_num, dest_port, MS->UF) != 0)
        return 2;

    memcpy(MS->UF->sockets[socket_num].dipr, dest_ip, 4);
    MS->UF->sockets[socket_num].dport = dest_port;

    // Команда CONNECT
    if (W5500_Set_Sn_CR(socket_num, 0x04, MS->UF) != 0)
        return 3;

    //  Ожидание установления соединения
    uint8_t status;
    for (int i = 0; i < 10; i++) {
        status = W5500_Get_Sn_SR(socket_num, MS->UF);
        if (status == 0x17)  // SOCK_ESTABLISHED
        {
            return 0;  // Соединение успешно установлено
        }
        MS->UF->Delay(200);
    }
    return 4;
}
//%%%%%%%%%%%%%%%%%%%% Функции для работы с данными "в сокетах" %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Отправка данных без обработчиков прерываний
uint8_t W5500_SendData(
    uint8_t            socket_num,
    uint8_t            func_mode,
    const uint8_t*     dest_ip,
    uint16_t           dest_port,
    const uint8_t*     data,
    uint16_t           len,
    W5500_Main_Struct* MS)
{
    if (func_mode & 0x1) {
        // Устанавливаем IP получателя
        W5500_Set_Sn_DIPR(socket_num, dest_ip, MS->UF);
        memcpy(MS->UF->sockets[socket_num].dipr, dest_ip, 4);
    }

    if (func_mode & 0x2) {
        // Устанавливаем порт получателя
        W5500_Set_Sn_DPORT(socket_num, dest_port, MS->UF);
        MS->UF->sockets[socket_num].dport = dest_port;
    }

    // Перед отправкой закроем и откроем сокет
    if (MS->UF->sockets[socket_num].mode == 0x1 || func_mode & 0x8) {
        W5500_CloseSocket(socket_num, MS);
        W5500_OpenSocket(socket_num, MS);

        // Если TCP пакет, то откроем соединение
        if (MS->UF->sockets[socket_num].mode == 0x1) {
            if (W5500_TCP_Connect(
                    socket_num,
                    MS->UF->sockets[socket_num].dipr,
                    MS->UF->sockets[socket_num].dport,
                    MS)
                != 0) {
                return 1;
            }
        }
    }
    // Получаем указатель записи в буфер передачи !Обязательно при TCP!
    uint16_t tx_wr = W5500_Get_Sn_TX_WR(socket_num, MS->UF);

    // Записываем данные в буфер передачи
    W5500_Sn_TXBUF_Write(tx_wr, socket_num, data, len, MS->UF);

    // Обновляем указатель записи
    tx_wr += len;
    W5500_Set_Sn_TX_WR(socket_num, tx_wr, MS->UF);
    MS->UF->sockets[socket_num].tx_wr_p = tx_wr;

    // Отправляем команду SEND
    W5500_Set_Sn_CR(socket_num, 0x20, MS->UF);  // Команда SEND

    // Проверяем статус выполнения команды
    for (int i = 0; i < 10; i++) {

        if (W5500_Get_Sn_IR(socket_num, MS->UF) & 0x10) {

            // Очищаем флаг прерывания
            W5500_Set_Sn_IR(socket_num, 0x10, MS->UF);

            // Если TCP пакет, то отключаемся от сервера
            if (MS->UF->sockets[socket_num].mode == 0x1) {
                W5500_TCP_Disconnect(socket_num, MS);
            }

            // Закрываем сокет
            if (MS->UF->sockets[socket_num].mode == 0x1 || func_mode & 0x8) {
                W5500_CloseSocket(socket_num, MS);
            }
            return 0;  // Успешная отправка
        }
        MS->UF->Delay(200);
    }
    return 1;  // Неудачная отправка
}

// Отправка данных с обработчиками прерываний
uint8_t W5500_SendData_IR(
    uint8_t            socket_num,
    uint8_t            func_mode,
    const uint8_t*     dest_ip,
    uint16_t           dest_port,
    const uint8_t*     data,
    uint16_t           len,
    W5500_Main_Struct* MS)
{
    if (func_mode & 0x1) {
        // Устанавливаем IP получателя
        W5500_Set_Sn_DIPR(socket_num, dest_ip, MS->UF);
        memcpy(MS->UF->sockets[socket_num].dipr, dest_ip, 4);
    }

    if (func_mode & 0x2) {
        // Устанавливаем порт получателя
        W5500_Set_Sn_DPORT(socket_num, dest_port, MS->UF);
        MS->UF->sockets[socket_num].dport = dest_port;
    }

    // Перед отправкой закроем и откроем сокет
    if (MS->UF->sockets[socket_num].mode == 0x1 || func_mode & 0x8) {
        W5500_CloseSocket(socket_num, MS);
        W5500_OpenSocket(socket_num, MS);

        // Если TCP пакет, то откроем соединение
        if (MS->UF->sockets[socket_num].mode == 0x1) {
            if (W5500_TCP_Connect(
                    socket_num,
                    MS->UF->sockets[socket_num].dipr,
                    MS->UF->sockets[socket_num].dport,
                    MS)
                != 0) {
                return 1;
            }
        }
    }

    // Получаем указатель записи в буфер передачи !Обяхательно при TCP!
    uint16_t tx_wr = W5500_Get_Sn_TX_WR(socket_num, MS->UF);

    // Записываем данные в буфер передачи
    W5500_Sn_TXBUF_Write(tx_wr, socket_num, data, len, MS->UF);

    // Обновляем указатель записи
    tx_wr += len;
    W5500_Set_Sn_TX_WR(socket_num, tx_wr, MS->UF);
    MS->UF->sockets[socket_num].tx_wr_p = tx_wr;

    // Отправляем команду SEND
    W5500_Set_Sn_CR(socket_num, 0x20, MS->UF);  // Команда SEND

    // Если TCP пакет, то отключаемся от сервера
    if (MS->UF->sockets[socket_num].mode == 0x1) {
        W5500_TCP_Disconnect(socket_num, MS);
    }

    if (MS->UF->sockets[socket_num].mode == 0x1 || func_mode & 0x8) {
        // Закрываем сокет
        W5500_CloseSocket(socket_num, MS);
    };

    return 0;  // Конец отправки
}

// Приём данных
uint16_t W5500_ReceiveData(
    uint8_t           socket_num,
    uint8_t*          buffer,
    uint16_t          max_len,
    W5500_User_Funcs* UF)
{
    // Получаем размер доступных данных в буфере приема
    uint16_t rx_rsr = W5500_Get_Sn_RX_RSR(socket_num, UF);
    if (rx_rsr == 0)
        return 0;  // Нет данных для чтения

    // Ограничиваем размер чтения до максимального размера буфера
    uint16_t len = (rx_rsr > max_len) ? max_len : rx_rsr;

    // Получаем указатель чтения из буфера приема
    uint16_t rx_rd = W5500_Get_Sn_RX_RD(socket_num, UF);

    // Читаем данные из буфера приема
    W5500_ReadRegister(rx_rd, W5500_SOCKET_RXBUF(socket_num), buffer, len, UF);

    // Обновляем указатель чтения
    rx_rd += len;
    if (rx_rd > UF->sockets[socket_num].rx_buf_size) {
        rx_rd -= UF->sockets[socket_num].rx_buf_size;
    }
    W5500_Set_Sn_RX_RD(socket_num, rx_rd, UF);
    UF->sockets[socket_num].rx_rd_p = rx_rd;

    // Отправляем команду RECV
    W5500_Set_Sn_CR(socket_num, 0x40, UF);  // Команда RECV

    return rx_rsr;  // Возвращаем количество байт
}

// Сокет переходит в режим TCP Сервера
//* Для выхода из режима закрйоте сокет
void W5500_TCP_Listen(
    uint8_t            socket_num,
    W5500_Main_Struct* MS)
{
    // Закртытие и открытие сокета
    W5500_CloseSocket(socket_num, MS);
    W5500_OpenSocket(socket_num, MS);

    // Переводим сокет в TCP сервер
    W5500_Set_Sn_CR(socket_num, 0x2, MS->UF);
}

////%%%%%%%%%%%%%%%%%%%%%%%%%%% Функции работы с соединением %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Открытие сокета
uint8_t W5500_OpenSocket(
    uint8_t            socket_num,
    W5500_Main_Struct* MS)
{
    return W5500_Set_Sn_CR(socket_num, 0x01, MS->UF);
}

// Закрытие сокета
uint8_t W5500_CloseSocket(
    uint8_t            socket_num,
    W5500_Main_Struct* MS)
{
    W5500_Set_Sn_IR(socket_num, 0xFF, MS->UF);

    // Отправляем команду CLOSE
    W5500_Set_Sn_CR(socket_num, 0x10, MS->UF);  // Команда CLOSE

    // Ожидаем перехода в состояние SOCK_CLOSED
    uint8_t status;
    do {
        status = W5500_Get_Sn_SR(socket_num, MS->UF);
    } while (status != 0x00);  // SOCK_CLOSED

    return 0;  // Успешное закрытие
}

// TCP отключение
uint8_t W5500_TCP_Disconnect(
    uint8_t            socket_num,
    W5500_Main_Struct* MS)
{
    // Отправляем команду DISCON
    W5500_Set_Sn_CR(socket_num, 0x08, MS->UF);  // Команда DISCON

    // Ожидаем перехода в состояние SOCK_CLOSED
    uint8_t status;
    do {
        status = W5500_Get_Sn_CR(socket_num, MS->UF);
    } while (status & 0x20);

    for (int i = 0; i < 10; i++) {
        if (W5500_Get_Sn_SR(socket_num, MS->UF) == 0x00)
            return 0;
        MS->UF->Delay(200);
        W5500_CloseSocket(socket_num, MS);
    }

    return 1;  // Неудачное отключение
}

////%%%%%%%%%%%%%%%%%%%%%%% Функции прерывания %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Обработка прерываний сокетов
void W5500_IR_processing(
    uint8_t*           buf,
    uint8_t            size,
    W5500_Main_Struct* MS)
{
    uint8_t IR_byte = W5500_Get_SIR(MS->UF);
    for (uint8_t i = 0; i < 8; i++) {
        if ((0x1 << i) & IR_byte) {
            uint8_t IR = W5500_Get_Sn_IR(i, MS->UF);
            // sockets[i].last_interrupt = IR;
            if ((IR & CON) && MS->UCb->Callback_Con != NULL) {
                MS->UCb->Callback_Con(&MS->UF->sockets[i], MS->UF);
                W5500_Set_Sn_IR(i, CON, MS->UF);
            }

            if ((IR & DISCON) && MS->UCb->Callback_Discon != NULL) {
                MS->UCb->Callback_Discon(&MS->UF->sockets[i], MS->UF);
                W5500_Set_Sn_IR(i, DISCON, MS->UF);
            }

            if ((IR & RECV) && MS->UCb->Callback_Recv != NULL) {
                MS->UCb->Callback_Recv(buf, size, &MS->UF->sockets[i], MS->UF);
                W5500_Set_Sn_IR(i, RECV, MS->UF);
            }

            if ((IR & TIMEOUT) && MS->UCb->Callback_Timeout != NULL) {
                MS->UCb->Callback_Timeout(&MS->UF->sockets[i], MS->UF);
                W5500_Set_Sn_IR(i, TIMEOUT, MS->UF);
            }

            if ((IR & SEND_OK) && MS->UCb->Callback_Send_OK != NULL) {
                MS->UCb->Callback_Send_OK(&MS->UF->sockets[i], MS->UF);
                W5500_Set_Sn_IR(i, SEND_OK, MS->UF);
            }
            // обнуляем биты прерывания
            W5500_Set_SIR(IR_byte, MS->UF);
            W5500_Set_Sn_IR(i, IR, MS->UF);
        }
    }

    return;
}
