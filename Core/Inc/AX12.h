#ifndef AX12_H
#define AX12_H

#include "main.h" // Includes STM32 HAL and standard types

// --- Control Table Addresses ---
#define AX12_MODEL_NUMBER_L       0x00
#define AX12_VERSION              0x02
#define AX12_ID                   0x03
#define AX12_BAUD_RATE            0x04
#define AX12_RETURN_DELAY_TIME    0x05
#define AX12_CW_ANGLE_LIMIT_L     0x06
#define AX12_CCW_ANGLE_LIMIT_L    0x08
#define AX12_LIMIT_TEMPERATURE    0x0B
#define AX12_DOWN_LIMIT_VOLTAGE   0x0C
#define AX12_UP_LIMIT_VOLTAGE     0x0D
#define AX12_MAX_TORQUE_L         0x0E
#define AX12_RETURN_LEVEL         0x10
#define AX12_ALARM_LED            0x11
#define AX12_ALARM_SHUTDOWN       0x12

#define AX12_TORQUE_ENABLE        0x18
#define AX12_LED                  0x19
#define AX12_CW_COMPLIANCE_MARGIN 0x1A
#define AX12_CCW_COMPLIANCE_MARGIN 0x1B
#define AX12_CW_COMPLIANCE_SLOPE  0x1C
#define AX12_CCW_COMPLIANCE_SLOPE 0x1D
#define AX12_GOAL_POSITION_L      0x1E
#define AX12_MOVING_SPEED_L       0x20
#define AX12_TORQUE_LIMIT_L       0x22
#define AX12_PRESENT_POSITION_L   0x24
#define AX12_PRESENT_SPEED_L      0x26
#define AX12_PRESENT_LOAD_L       0x28
#define AX12_PRESENT_VOLTAGE      0x2A
#define AX12_PRESENT_TEMPERATURE  0x2B
#define AX12_REGISTERED           0x2C
#define AX12_MOVING               0x2E
#define AX12_LOCK                 0x2F
#define AX12_PUNCH_L              0x30

// --- Instructions ---
#define INST_PING       0x01
#define INST_READ       0x02
#define INST_WRITE      0x03
#define INST_REG_WRITE  0x04
#define INST_ACTION     0x05
#define INST_RESET      0x06
#define INST_SYNC_WRITE 0x83

// --- Broadcast ID ---
#define AX12_BROADCAST_ID 0xFE

// --- Struct for Motor Handle ---
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *DirPort;
    uint16_t DirPin;
    uint8_t ID;
} AX12_Handle_TypeDef;

// --- Core Functions ---
void AX12_Init(AX12_Handle_TypeDef *motor, UART_HandleTypeDef *huart, GPIO_TypeDef *port, uint16_t pin, uint8_t id);
int AX12_Ping(AX12_Handle_TypeDef *motor);
void AX12_FactoryReset(AX12_Handle_TypeDef *motor);

// --- Setters (Write) ---
void AX12_TorqueEnable(AX12_Handle_TypeDef *motor, uint8_t enable);
void AX12_SetID(AX12_Handle_TypeDef *motor, uint8_t new_id);
void AX12_SetBaudRate(AX12_Handle_TypeDef *motor, uint8_t baud_code); // 1=1Mbps
void AX12_SetGoalPosition(AX12_Handle_TypeDef *motor, uint16_t position);
void AX12_SetMovingSpeed(AX12_Handle_TypeDef *motor, uint16_t speed);
void AX12_SetLED(AX12_Handle_TypeDef *motor, uint8_t state);
void AX12_SetAngleLimit(AX12_Handle_TypeDef *motor, uint16_t cw_limit, uint16_t ccw_limit);
void AX12_SetTorqueLimit(AX12_Handle_TypeDef *motor, uint16_t max_torque);

// --- Getters (Read) ---
int AX12_GetPresentPosition(AX12_Handle_TypeDef *motor, uint16_t *position);
int AX12_GetPresentTemperature(AX12_Handle_TypeDef *motor, uint8_t *temp);
int AX12_GetPresentLoad(AX12_Handle_TypeDef *motor, uint16_t *load);
int AX12_GetPresentVoltage(AX12_Handle_TypeDef *motor, uint8_t *voltage);
int AX12_IsMoving(AX12_Handle_TypeDef *motor, uint8_t *moving);

// --- Action / RegWrite (Synchronized Moves) ---
void AX12_RegWriteGoalPosition(AX12_Handle_TypeDef *motor, uint16_t position);
void AX12_Action(AX12_Handle_TypeDef *motor); // Triggers the Registered Write

#endif
