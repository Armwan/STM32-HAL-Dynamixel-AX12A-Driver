#include "AX12.h"
#include <string.h> // For memcpy

// --- Private Macros for Direction Control ---
#define DXL_TX_MODE(h) HAL_GPIO_WritePin(h->DirPort, h->DirPin, GPIO_PIN_SET)
#define DXL_RX_MODE(h) HAL_GPIO_WritePin(h->DirPort, h->DirPin, GPIO_PIN_RESET)
#define DXL_TIMEOUT    20 // Reduced timeout to prevent long blocking

// --- Private Helper: Checksum Calculation ---
// Manual Page 12: Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N) [cite: 167]
static uint8_t AX12_CalcChecksum(uint8_t *packet, uint8_t length) {
    uint8_t checksum = 0;
    // Loop sums ID, Length, Instruction, and all Parameters
    // Packet indices: 0=FF, 1=FF, 2=ID, 3=Len, 4=Instr, 5...=Params
    for (int i = 2; i < length - 1; i++) {
        checksum += packet[i];
    }
    return (~checksum) & 0xFF;
}

// --- Private Helper: Flush RX Buffer (CRITICAL FIX) ---
// Clears the hardware Overrun Error (ORE) flag which causes the "Freeze"
static void AX12_FlushRx(AX12_Handle_TypeDef *motor) {
    // Clear All Error Flags
    __HAL_UART_CLEAR_FLAG(motor->huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);

    // Drain the Data Register
    uint8_t temp;
    while (__HAL_UART_GET_FLAG(motor->huart, UART_FLAG_RXNE)) {
        temp = (uint8_t)(motor->huart->Instance->RDR & 0x00FF);
    }
}

// --- Initialization ---
void AX12_Init(AX12_Handle_TypeDef *motor, UART_HandleTypeDef *huart, GPIO_TypeDef *port, uint16_t pin, uint8_t id) {
    motor->huart = huart;
    motor->DirPort = port;
    motor->DirPin = pin;
    motor->ID = id;
}

// --- Generic Write Function (CORRECTED) ---
static void AX12_WriteRaw(AX12_Handle_TypeDef *motor, uint8_t address, uint8_t *data, uint8_t data_len) {
    uint8_t packet[16];
    uint8_t status_packet[6];

    // Calculation:
    // Instruction(1) + Address(1) + Data(data_len) + Checksum(1)
    // But Packet Length Field = (Instruction + Address + Data + Checksum) - 2?
    // NO, Manual says Length = (Num Parameters) + 2.
    // Parameters = Address(1) + Data(data_len).
    // So Length = 1 + data_len + 2.

    uint8_t length = 1 + data_len + 2; // FIXED CALCULATION

    // 1. Construct Packet
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = motor->ID;
    packet[3] = length;       // Send the calculated length directly
    packet[4] = INST_WRITE;
    packet[5] = address;

    for(int i=0; i<data_len; i++) {
        packet[6+i] = data[i];
    }

    // Checksum is calculated over ID + Length + Instruction + Params
    // Checksum index is at 5 + data_len + 1, but using helper:
    // Packet size is Header(2) + ID(1) + Len(1) + Instr(1) + Addr(1) + Data(len) + CS(1)
    // Total bytes = 7 + data_len
    packet[6+data_len] = AX12_CalcChecksum(packet, 7+data_len);

    // 2. Flush Buffer
    AX12_FlushRx(motor);

    // 3. Transmit
    DXL_TX_MODE(motor);
    HAL_UART_Transmit(motor->huart, packet, 7+data_len, DXL_TIMEOUT);

    // 4. Wait for TC
    while (__HAL_UART_GET_FLAG(motor->huart, UART_FLAG_TC) == RESET);

    // 5. Switch to RX
    DXL_RX_MODE(motor);

    // 6. Read Status Packet
    HAL_UART_Receive(motor->huart, status_packet, 6, DXL_TIMEOUT);
}

// --- Generic Read Function ---
static int AX12_ReadRaw(AX12_Handle_TypeDef *motor, uint8_t address, uint8_t len_to_read, uint8_t *rx_data) {
    uint8_t packet[8];
    uint8_t rx_buffer[16];

    // Construct Packet
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = motor->ID;
    packet[3] = 0x04;
    packet[4] = INST_READ;
    packet[5] = address;
    packet[6] = len_to_read;
    packet[7] = AX12_CalcChecksum(packet, 8);

    // Flush before start
    AX12_FlushRx(motor);

    // Transmit
    DXL_TX_MODE(motor);
    HAL_UART_Transmit(motor->huart, packet, 8, DXL_TIMEOUT);
    while (__HAL_UART_GET_FLAG(motor->huart, UART_FLAG_TC) == RESET);
    DXL_RX_MODE(motor);

    // Receive Response: Header(2)+ID(1)+Len(1)+Err(1)+Data(N)+CS(1)
    uint8_t expected_len = 6 + len_to_read;
    if (HAL_UART_Receive(motor->huart, rx_buffer, expected_len, DXL_TIMEOUT) != HAL_OK) {
        return -1;
    }

    // Validate Packet
    if (rx_buffer[0] == 0xFF && rx_buffer[1] == 0xFF && rx_buffer[2] == motor->ID) {
        if (rx_buffer[4] != 0) return -2; // Motor Error Bit set

        for(int i=0; i<len_to_read; i++) {
            rx_data[i] = rx_buffer[5+i];
        }
        return 0; // Success
    }
    return -3;
}

// --- User API Functions ---

void AX12_TorqueEnable(AX12_Handle_TypeDef *motor, uint8_t enable) {
    AX12_WriteRaw(motor, AX12_TORQUE_ENABLE, &enable, 1);
}

void AX12_SetGoalPosition(AX12_Handle_TypeDef *motor, uint16_t position) {
    if (position > 1023) position = 1023;
    uint8_t data[2];
    data[0] = position & 0xFF;
    data[1] = (position >> 8) & 0xFF;
    AX12_WriteRaw(motor, AX12_GOAL_POSITION_L, data, 2);
}

void AX12_SetMovingSpeed(AX12_Handle_TypeDef *motor, uint16_t speed) {
    if (speed > 1023) speed = 1023;
    uint8_t data[2];
    data[0] = speed & 0xFF;
    data[1] = (speed >> 8) & 0xFF;
    AX12_WriteRaw(motor, AX12_MOVING_SPEED_L, data, 2);
}

void AX12_SetID(AX12_Handle_TypeDef *motor, uint8_t new_id) {
    AX12_WriteRaw(motor, AX12_ID, &new_id, 1);
    motor->ID = new_id;
}

int AX12_GetPresentPosition(AX12_Handle_TypeDef *motor, uint16_t *position) {
    uint8_t data[2];
    if (AX12_ReadRaw(motor, AX12_PRESENT_POSITION_L, 2, data) == 0) {
        *position = (uint16_t)(data[1] << 8) | data[0];
        return 0;
    }
    return -1;
}

int AX12_GetPresentTemperature(AX12_Handle_TypeDef *motor, uint8_t *temp) {
    return AX12_ReadRaw(motor, AX12_PRESENT_TEMPERATURE, 1, temp);
}
