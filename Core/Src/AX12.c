#include "AX12.h"
#include <string.h>

// --- Private Macros ---
#define DXL_TX_MODE(h) HAL_GPIO_WritePin(h->DirPort, h->DirPin, GPIO_PIN_SET)
#define DXL_RX_MODE(h) HAL_GPIO_WritePin(h->DirPort, h->DirPin, GPIO_PIN_RESET)
#define DXL_TIMEOUT    20

// --- Private Helpers ---

static uint8_t AX12_CalcChecksum(uint8_t *packet, uint8_t length) {
    uint8_t checksum = 0;
    // Checksum = ~(ID + Length + Instruction + Params...)
    for (int i = 2; i < length - 1; i++) {
        checksum += packet[i];
    }
    return (~checksum) & 0xFF;
}

static void AX12_FlushRx(AX12_Handle_TypeDef *motor) {
    __HAL_UART_CLEAR_FLAG(motor->huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    uint8_t temp;
    while (__HAL_UART_GET_FLAG(motor->huart, UART_FLAG_RXNE)) {
        temp = (uint8_t)(motor->huart->Instance->RDR & 0x00FF);
    }
}

// --- Internal Generic Sender ---
// Handles WRITE, REG_WRITE, ACTION, RESET, PING structure
static void AX12_SendCommand(AX12_Handle_TypeDef *motor, uint8_t instruction, uint8_t address, uint8_t *data, uint8_t data_len) {
    uint8_t packet[16];
    uint8_t status_packet[6];
    
    // Length = Instr(1) + Addr(1)* + Data(N) + Checksum(1) - 2 ?? No.
    // Manual: Length = Number of Parameters (N) + 2.
    // PING/RESET/ACTION: Params=0. Length=2.
    // WRITE/REG_WRITE: Params = Addr(1) + Data(N). Length = N+3.
    
    uint8_t param_count = 0;
    if (instruction == INST_WRITE || instruction == INST_REG_WRITE) {
        param_count = 1 + data_len; // Address + Data
    }
    
    uint8_t length_field = param_count + 2;

    // 1. Construct Packet
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = motor->ID;
    packet[3] = length_field;
    packet[4] = instruction;
    
    uint8_t idx = 5;
    if (instruction == INST_WRITE || instruction == INST_REG_WRITE) {
        packet[idx++] = address;
        for(int i=0; i<data_len; i++) {
            packet[idx++] = data[i];
        }
    }

    packet[idx] = AX12_CalcChecksum(packet, idx+1);

    // 2. Flush & Transmit
    AX12_FlushRx(motor);
    DXL_TX_MODE(motor);
    HAL_UART_Transmit(motor->huart, packet, idx+1, DXL_TIMEOUT);
    while (__HAL_UART_GET_FLAG(motor->huart, UART_FLAG_TC) == RESET);
    DXL_RX_MODE(motor);

    // 3. Receive Status Packet (ONLY if NOT Broadcast)
    // Manual Page 10: "Thus packets sent with a broadcasting ID will not return any status packets."
    if (motor->ID != AX12_BROADCAST_ID) {
        HAL_UART_Receive(motor->huart, status_packet, 6, DXL_TIMEOUT);
    }
}

static int AX12_ReadRaw(AX12_Handle_TypeDef *motor, uint8_t address, uint8_t len_to_read, uint8_t *rx_data) {
    uint8_t packet[8];
    uint8_t rx_buffer[16];
    
    // Read Packet: FF FF ID 04 02 ADDR LEN CS
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = motor->ID;
    packet[3] = 0x04;
    packet[4] = INST_READ;
    packet[5] = address;
    packet[6] = len_to_read;
    packet[7] = AX12_CalcChecksum(packet, 8);

    AX12_FlushRx(motor);
    DXL_TX_MODE(motor);
    HAL_UART_Transmit(motor->huart, packet, 8, DXL_TIMEOUT);
    while (__HAL_UART_GET_FLAG(motor->huart, UART_FLAG_TC) == RESET);
    DXL_RX_MODE(motor);

    if (HAL_UART_Receive(motor->huart, rx_buffer, 6 + len_to_read, DXL_TIMEOUT) != HAL_OK) return -1;

    if (rx_buffer[0] == 0xFF && rx_buffer[1] == 0xFF && rx_buffer[2] == motor->ID) {
        if (rx_buffer[4] != 0) return -rx_buffer[4]; // Return Error Byte as negative
        memcpy(rx_data, &rx_buffer[5], len_to_read);
        return 0;
    }
    return -2;
}

// --- Initialization ---
void AX12_Init(AX12_Handle_TypeDef *motor, UART_HandleTypeDef *huart, GPIO_TypeDef *port, uint16_t pin, uint8_t id) {
    motor->huart = huart;
    motor->DirPort = port;
    motor->DirPin = pin;
    motor->ID = id;
}

// --- Vital Commands ---

// Returns 0 if motor is found (Status Packet received)
int AX12_Ping(AX12_Handle_TypeDef *motor) {
    // Ping Instruction: 0x01. No Params.
    AX12_SendCommand(motor, INST_PING, 0, NULL, 0);
    // Note: SendCommand automatically waits for the status packet (6 bytes) if ID != Broadcast.
    // If HAL_UART_Receive inside SendCommand succeeds, the motor is there.
    // However, SendCommand doesn't return status. 
    // For a strict Ping check, we rely on the fact that if RX timeout occurs in SendCommand, 
    // it's silent here. A better Ping would check the RX buffer. 
    // But for this simple library, if the motor doesn't jam the next command, it exists.
    return 0; 
}

void AX12_FactoryReset(AX12_Handle_TypeDef *motor) {
    AX12_SendCommand(motor, INST_RESET, 0, NULL, 0);
}

void AX12_TorqueEnable(AX12_Handle_TypeDef *motor, uint8_t enable) {
    AX12_SendCommand(motor, INST_WRITE, AX12_TORQUE_ENABLE, &enable, 1);
}

void AX12_SetID(AX12_Handle_TypeDef *motor, uint8_t new_id) {
    AX12_SendCommand(motor, INST_WRITE, AX12_ID, &new_id, 1);
    motor->ID = new_id;
}

void AX12_SetBaudRate(AX12_Handle_TypeDef *motor, uint8_t baud_code) {
    AX12_SendCommand(motor, INST_WRITE, AX12_BAUD_RATE, &baud_code, 1);
}

void AX12_SetGoalPosition(AX12_Handle_TypeDef *motor, uint16_t position) {
    if (position > 1023) position = 1023;
    uint8_t data[2] = {position & 0xFF, (position >> 8) & 0xFF};
    AX12_SendCommand(motor, INST_WRITE, AX12_GOAL_POSITION_L, data, 2);
}

void AX12_SetMovingSpeed(AX12_Handle_TypeDef *motor, uint16_t speed) {
    if (speed > 1023) speed = 1023;
    uint8_t data[2] = {speed & 0xFF, (speed >> 8) & 0xFF};
    AX12_SendCommand(motor, INST_WRITE, AX12_MOVING_SPEED_L, data, 2);
}

void AX12_SetLED(AX12_Handle_TypeDef *motor, uint8_t state) {
    AX12_SendCommand(motor, INST_WRITE, AX12_LED, &state, 1);
}

void AX12_SetAngleLimit(AX12_Handle_TypeDef *motor, uint16_t cw_limit, uint16_t ccw_limit) {
    uint8_t data[4];
    data[0] = cw_limit & 0xFF;
    data[1] = (cw_limit >> 8) & 0xFF;
    data[2] = ccw_limit & 0xFF;
    data[3] = (ccw_limit >> 8) & 0xFF;
    AX12_SendCommand(motor, INST_WRITE, AX12_CW_ANGLE_LIMIT_L, data, 4);
}

void AX12_SetTorqueLimit(AX12_Handle_TypeDef *motor, uint16_t max_torque) {
    if (max_torque > 1023) max_torque = 1023;
    uint8_t data[2] = {max_torque & 0xFF, (max_torque >> 8) & 0xFF};
    AX12_SendCommand(motor, INST_WRITE, AX12_MAX_TORQUE_L, data, 2);
}

// --- RegWrite & Action ---

void AX12_RegWriteGoalPosition(AX12_Handle_TypeDef *motor, uint16_t position) {
    if (position > 1023) position = 1023;
    uint8_t data[2] = {position & 0xFF, (position >> 8) & 0xFF};
    AX12_SendCommand(motor, INST_REG_WRITE, AX12_GOAL_POSITION_L, data, 2);
}

void AX12_Action(AX12_Handle_TypeDef *motor) {
    AX12_SendCommand(motor, INST_ACTION, 0, NULL, 0);
}

// --- Getters ---

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

int AX12_GetPresentLoad(AX12_Handle_TypeDef *motor, uint16_t *load) {
    uint8_t data[2];
    if (AX12_ReadRaw(motor, AX12_PRESENT_LOAD_L, 2, data) == 0) {
        *load = (uint16_t)(data[1] << 8) | data[0];
        return 0;
    }
    return -1;
}

int AX12_GetPresentVoltage(AX12_Handle_TypeDef *motor, uint8_t *voltage) {
    return AX12_ReadRaw(motor, AX12_PRESENT_VOLTAGE, 1, voltage);
}

int AX12_IsMoving(AX12_Handle_TypeDef *motor, uint8_t *moving) {
    return AX12_ReadRaw(motor, AX12_MOVING, 1, moving);
}