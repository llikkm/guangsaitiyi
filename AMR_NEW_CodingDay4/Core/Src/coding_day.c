#include "main.h"
#include "chassis_imu.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart3, huart4;
extern TIM_HandleTypeDef htim1, htim3, htim4, htim7, htim8;

/* 实车标定：用调试器确认轮子前进时的编码器和陀螺仪方向。 */
#define WHEEL_DIAMETER 0.065f
#define WHEEL_TRACK 0.145f
#define COUNTS_PER_REV 1320.0f
#define LEFT_ENCODER_SIGN 1
#define RIGHT_ENCODER_SIGN (-1)
#define LEFT_MOTOR_SIGN (-1)
#define RIGHT_MOTOR_SIGN 1
#define GYRO_SIGN 1
#define PI 3.141592654f
#define HALF 512U
#define MAX_POINTS 64
#define OLED_ADDRESS (0x3CU << 1)

typedef struct { float x, y; } Point;
typedef struct { float x, y, heading; } Pose;
typedef struct { uint16_t offset, length; uint32_t sequence; } DmaBlock;
typedef struct { uint32_t sequence, count, nearest_mm; } LidarResult;
typedef struct { Point goal; uint32_t generation; uint8_t stop; } Command;
typedef struct { Point points[MAX_POINTS]; uint32_t generation; unsigned count; } Path;
typedef struct { uint32_t period_us, execution_us, last, runs; } Timing;
enum { CONTROL, IMU, LIDAR, OUTPUT, OLED, BT_TX, BT_RX, PLANNER, TASK_COUNT };

static osMessageQueueId_t Q_Lidar, Q_Result, Q_Cmd, Q_Path, Q_Byte;
static osSemaphoreId_t imu_tick;
static osMutexId_t i2c_lock, bluetooth_tx_lock;
static osThreadId_t threads[TASK_COUNT];
static Timing timing[TASK_COUNT];
static Pose pose = {0, 0, PI / 2};
static LidarResult latest_lidar;
static volatile uint32_t lidar_sequence, lost_blocks, lost_results;
static volatile uint32_t imu_overrun, lost_commands, command_generation;
static volatile uint32_t rx_errors, imu_errors, oled_errors, lidar_start_error;
static volatile uint8_t imu_ready, stop_requested = 1;
static uint8_t lidar_dma[2 * HALF], bt_byte;
static float gyro_rate;
static uint32_t gyro_at;
static ChassisImuSample latest_imu;
static uint8_t has_imu_sample;
static uint8_t tracking_state;

static uint32_t timing_begin(unsigned id);
static void timing_end(unsigned id, uint32_t start);
static void periodic_wait(uint32_t *next, uint32_t period);
static int block_current(const DmaBlock *block);
static void control_task(void *arg);
static void imu_task(void *arg);
static void lidar_task(void *arg);
static void output_task(void *arg);
static void oled_task(void *arg);
static void telemetry_task(void *arg);
static void bluetooth_task(void *arg);
static void planner_task(void *arg);

//member1 code begin
/* 启动硬件和任务；此函数在内核初始化后、启动调度前调用。 */
void CodingDay_Init(void);

static void pwm_pair(TIM_HandleTypeDef *timer, uint32_t first, uint32_t second, float duty)
{
    uint32_t pulse = (uint32_t)(fabsf(duty) * (float)__HAL_TIM_GET_AUTORELOAD(timer));
    /* 换向时先清零当前反方向的输出。 */
    if (duty >= 0) {
        __HAL_TIM_SET_COMPARE(timer, second, 0);
        __HAL_TIM_SET_COMPARE(timer, first, pulse);
    } else {
        __HAL_TIM_SET_COMPARE(timer, first, 0);
        __HAL_TIM_SET_COMPARE(timer, second, pulse);
    }
}

static void motors(float left, float right)
{
    pwm_pair(&htim4, TIM_CHANNEL_1, TIM_CHANNEL_2, LEFT_MOTOR_SIGN * left);
    pwm_pair(&htim1, TIM_CHANNEL_2, TIM_CHANNEL_1, RIGHT_MOTOR_SIGN * right);
}

/* 同一串口由两项任务输出，保证每行遥测完整。 */
static void send_line(char *line, int length, size_t capacity)
{
    if (length <= 0 || (size_t)length >= capacity) return;
    if (osMutexAcquire(bluetooth_tx_lock, 10) != osOK) return;
    HAL_UART_Transmit(&huart4, (uint8_t *)line, (uint16_t)length, 20);
    osMutexRelease(bluetooth_tx_lock);
}

/* 雷达输出只需要无符号整数，避免在任务栈上调用 printf 格式化器。 */
static char *append_u32(char *out, uint32_t value)
{
    uint32_t place = 1;
    while (value / place >= 10U) place *= 10U;
    do {
        *out++ = (char)('0' + value / place);
        value %= place;
        place /= 10U;
    } while (place != 0U);
    return out;
}


static void output_task(void *arg)
{
    (void)arg;
    LidarResult result;
    for (;;) {
        osMessageQueueGet(Q_Result, &result, NULL, osWaitForever);
        uint32_t start = timing_begin(OUTPUT);
        taskENTER_CRITICAL();
        latest_lidar = result;
        taskEXIT_CRITICAL();
        // 前缀、三个最多 10 位的数、两个逗号和 CRLF。
        char line[6 + 3 * 10 + 2 + 2 + 1] = "LIDAR,";
        char *end = append_u32(line + 6, result.sequence);
        *end++ = ',';
        end = append_u32(end, result.count);
        *end++ = ',';
        end = append_u32(end, result.nearest_mm);
        *end++ = '\r';
        *end++ = '\n';
        send_line(line, (int)(end - line), sizeof line);
        timing_end(OUTPUT, start);
    }
}

static void telemetry_task(void *arg)
{
    (void)arg;
    uint32_t next = osKernelGetTickCount(), frame = 0;
    char line[160];
    for (;;) {
        uint32_t start = timing_begin(BT_TX);
        Pose p;
        LidarResult lidar;
        unsigned state;
        taskENTER_CRITICAL();
        p = pose;
        state = tracking_state;
        lidar = latest_lidar;
        taskEXIT_CRITICAL();
        int n = snprintf(line, sizeof line, "POSE,%lu,%ld,%ld,%ld,%u,%lu,%lu,%lu,%lu,%lu\r\n",
            (unsigned long)osKernelGetTickCount(), (long)(p.x * 1000), (long)(p.y * 1000),
            (long)(p.heading * 1800 / PI), state, (unsigned long)lidar.sequence,
            (unsigned long)lidar.count, (unsigned long)lidar.nearest_mm,
            (unsigned long)lost_blocks, (unsigned long)lost_results);
        send_line(line, n, sizeof line);
        if (++frame % 5 == 0) {
            unsigned i = (frame / 5 - 1) % TASK_COUNT;
            Timing t;
            taskENTER_CRITICAL();
            t = timing[i];
            taskEXIT_CRITICAL();
            n = snprintf(line, sizeof line, "TIME,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n", i,
                (unsigned long)t.period_us, (unsigned long)t.execution_us,
                (unsigned long)t.runs, (unsigned long)imu_overrun, (unsigned long)lost_commands,
                (unsigned long)rx_errors, (unsigned long)imu_errors,
                (unsigned long)oled_errors, (unsigned long)lidar_start_error);
            send_line(line, n, sizeof line);
        }
        timing_end(BT_TX, start);
        periodic_wait(&next, 50);
    }
}
//member1 code end

//member2 code begin
/* DWT 计数器给出任务实际运行间隔及含抢占的执行耗时。 */
static uint32_t timing_begin(unsigned id)
{
    uint32_t now = DWT->CYCCNT;
    Timing *t = &timing[id];
    taskENTER_CRITICAL();
    if (t->runs++) t->period_us = (now - t->last) / (SystemCoreClock / 1000000U);
    t->last = now;
    taskEXIT_CRITICAL();
    return now;
}

static void timing_end(unsigned id, uint32_t start)
{
    timing[id].execution_us = (DWT->CYCCNT - start) / (SystemCoreClock / 1000000U);
}

static void periodic_wait(uint32_t *next, uint32_t period)
{
    *next += period;
    uint32_t now = osKernelGetTickCount();
    if ((int32_t)(*next - now) <= 0) *next = now + period;
    osDelayUntil(*next);
}

static void create_tasks(void)
{
    static const struct {
        const char *name; osPriority_t priority; uint32_t stack;
        void (*run)(void *);
    } cfg[TASK_COUNT] = {
        {"Control", osPriorityHigh, 768, control_task},
        {"IMU", osPriorityHigh1, 640, imu_task},
        {"LiDAR", osPriorityAboveNormal, 768, lidar_task},
        {"Output", osPriorityNormal, 640, output_task},
        {"OLED", osPriorityAboveNormal1, 1024, oled_task},
        {"BluetoothTX", osPriorityBelowNormal, 1024, telemetry_task},
        {"BluetoothRX", osPriorityAboveNormal, 1024, bluetooth_task},
        {"Planner", osPriorityNormal, 1024, planner_task}
    };
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    for (unsigned i = 0; i < TASK_COUNT; ++i) {
        const osThreadAttr_t attr = {
            .name = cfg[i].name, .priority = cfg[i].priority, .stack_size = cfg[i].stack
        };
        threads[i] = osThreadNew(cfg[i].run, NULL, &attr);
        if (!threads[i]) Error_Handler();
    }
}
//member2 code end

//member3 code begin
static void create_queues(void)
{
    const osMessageQueueAttr_t lidar = {.name = "Q_Lidar"}, result = {.name = "Q_Result"};
    const osMessageQueueAttr_t cmd = {.name = "Q_Cmd"}, path = {.name = "Q_Path"};
    Q_Lidar = osMessageQueueNew(4, sizeof(DmaBlock), &lidar);
    Q_Result = osMessageQueueNew(4, sizeof(LidarResult), &result);
    Q_Cmd = osMessageQueueNew(4, sizeof(Command), &cmd);
    Q_Path = osMessageQueueNew(2, sizeof(Path), &path);
    Q_Byte = osMessageQueueNew(64, sizeof(uint8_t), NULL);
    if (!Q_Lidar || !Q_Result || !Q_Cmd || !Q_Path || !Q_Byte) Error_Handler();
}

static void dma_ready(uint16_t offset)
{
    DmaBlock block = {offset, HALF, ++lidar_sequence};
    if (osMessageQueuePut(Q_Lidar, &block, 0, 0) != osOK) ++lost_blocks;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart3) dma_ready(0);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart3) dma_ready(HALF);
    if (uart == &huart4) {
        if (osMessageQueuePut(Q_Byte, &bt_byte, 0, 0) != osOK) {
            ++rx_errors;
            stop_requested = 1;
        }
        HAL_UART_Receive_IT(&huart4, &bt_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart4) {
        ++rx_errors;
        stop_requested = 1;
        if (uart->RxState == HAL_UART_STATE_READY)
            HAL_UART_Receive_IT(uart, &bt_byte, 1);
    }
}

/* SCAN 响应头 7 字节，随后每个测量节点 5 字节。 */
static void lidar_task(void *arg)
{
    (void)arg;
    static const uint8_t stop[] = {0xA5, 0x25}, scan[] = {0xA5, 0x20};
    static const uint8_t descriptor[] = {0xA5, 0x5A, 5, 0, 0, 0x40, 0x81};
    DmaBlock block;
    uint8_t node[5];
    unsigned used = 0, header = 0;
    uint32_t previous = 0;
    HAL_UART_Transmit(&huart3, (uint8_t *)stop, sizeof stop, 20);
    osDelay(50);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    if (HAL_UART_Receive_DMA(&huart3, lidar_dma, sizeof lidar_dma) != HAL_OK ||
        HAL_UART_Transmit(&huart3, (uint8_t *)scan, sizeof scan, 20) != HAL_OK)
        ++lidar_start_error;
    for (;;) {
        osMessageQueueGet(Q_Lidar, &block, NULL, osWaitForever);
        uint32_t start = timing_begin(LIDAR);
        if (block.sequence != previous + 1) used = 0;
        previous = block.sequence;
        /* 序号和 NDTR 一起核对，过期的数据块直接丢弃。 */
        if (!block_current(&block)) {
            used = 0;
            ++lost_blocks;
            timing_end(LIDAR, start);
            continue;
        }
        unsigned old_header = header;
        LidarResult result = {block.sequence, 0, 0};
        for (unsigned i = 0; i < block.length; ++i) {
            uint8_t value = lidar_dma[block.offset + i];
            if (header < sizeof descriptor) {
                header = value == descriptor[header] ? header + 1 : (value == 0xA5);
                continue;
            }
            node[used++] = value;
            if (used != sizeof node) continue;
            unsigned angle = (node[1] >> 1) | ((unsigned)node[2] << 7);
            if (((node[0] ^ (node[0] >> 1)) & 1) && (node[1] & 1) && angle < 23040) {
                unsigned mm = ((unsigned)node[3] | ((unsigned)node[4] << 8)) / 4;
                if (mm && (node[0] >> 2)) {
                    ++result.count;
                    if (!result.nearest_mm || mm < result.nearest_mm) result.nearest_mm = mm;
                }
                used = 0;
            } else {
                memmove(node, node + 1, 4);
                used = 4;
            }
        }
        if (!block_current(&block)) {
            header = old_header;
            used = 0;
            ++lost_blocks;
        } else if (osMessageQueuePut(Q_Result, &result, 0, 0) != osOK) {
            ++lost_results;
        }
        timing_end(LIDAR, start);
    }
}

static int block_current(const DmaBlock *block)
{
    uint32_t remaining = __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    return block->sequence == lidar_sequence && remaining != 0 &&
        (block->offset == 0 ? remaining <= HALF : remaining > HALF);
}
//member3 code end

//member4 code begin
/* TIM7 ISR 发令牌，I2C 读数留给任务。 */
void CodingDay_TimerISR(void)
{
    if (imu_tick && osSemaphoreRelease(imu_tick) != osOK) ++imu_overrun;
}

/* 通过 GDB 读取每个设备最后一次 I2C 操作的结果。 */
typedef struct {
    uint32_t status, error, address, operation, pins;
} I2cDiag;

volatile I2cDiag imu_diag, oled_diag;
volatile uint32_t imu_who_am_i = 0xFFFFFFFFU;
volatile uint32_t i2c_recoveries, i2c_released_pins = 0xFFFFFFFFU;

static void record_i2c(volatile I2cDiag *diag, uint32_t status,
                       uint32_t error, uint16_t address, uint32_t operation)
{
    diag->status = status;
    diag->error = error;
    diag->address = address >> 1;
    diag->operation = operation;
    diag->pins = GPIOB->IDR & (GPIO_PIN_3 | GPIO_PIN_10);
}


/* BUSY 超时后释放总线并重建 I2C2，调用方继续持有互斥锁。 */
static void recover_i2c(uint32_t status, uint32_t error)
{
    if (status != HAL_BUSY || !(error & HAL_I2C_ERROR_TIMEOUT)) return;

    // 释放两个引脚后采样，从而区分外部拉低和 I2C 外设占用。
    if (HAL_I2C_DeInit(&hi2c2) != HAL_OK) return;
    osDelay(2);
    const uint32_t bus_pins = GPIO_PIN_3 | GPIO_PIN_10;
    i2c_released_pins = GPIOB->IDR & bus_pins;
    if (HAL_I2C_Init(&hi2c2) == HAL_OK && i2c_released_pins == bus_pins) {
        ++i2c_recoveries;
    }
}


static int imu_register(uint16_t address, uint8_t reg, uint8_t *data, uint16_t len, int write)
{
    if (osMutexAcquire(i2c_lock, 10) != osOK) {
        record_i2c(&imu_diag, 4, 0, address, reg | (write ? 0x100U : 0U));
        return 0;
    }
    HAL_StatusTypeDef status = write ?
        HAL_I2C_Mem_Write(&hi2c2, address, reg, I2C_MEMADD_SIZE_8BIT, data, len, 4) :
        HAL_I2C_Mem_Read(&hi2c2, address, reg, I2C_MEMADD_SIZE_8BIT, data, len, 4);
    record_i2c(&imu_diag, status, HAL_I2C_GetError(&hi2c2),
               address, reg | (write ? 0x100U : 0U));
    recover_i2c(status, imu_diag.error);
    osMutexRelease(i2c_lock);
    return status == HAL_OK;
}

int ChassisImu_GetLatest(ChassisImuSample *sample)
{
    if (!sample) return 0;
    taskENTER_CRITICAL();
    int valid = has_imu_sample;
    if (valid) *sample = latest_imu;
    taskEXIT_CRITICAL();
    return valid;
}

static int imu_setting(uint16_t address, uint8_t reg, uint8_t value)
{
    uint8_t actual;
    return imu_register(address, reg, &value, 1, 1) &&
           imu_register(address, reg, &actual, 1, 0) && actual == value;
}

#if CHASSIS_IMU == CHASSIS_LSM6DS3TR_C
#define IMU_ADDR_LOW 0x6A
#define IMU_ID_REG 0x0F
#define IMU_EXPECTED_ID 0x6A

static int imu_configure(uint16_t address)
{
    uint8_t reset = 1;
    if (!imu_register(address, 0x12, &reset, 1, 1)) return 0;
    for (unsigned tries = 0; tries < 10; ++tries) {
        osDelay(2);
        if (!imu_register(address, 0x12, &reset, 1, 0)) return 0;
        if (!(reset & 1)) {
            return imu_setting(address, 0x12, 0x44) && /* BDU, auto-increment */
                   imu_setting(address, 0x10, 0x40) && /* XL: 104 Hz, +/-2 g */
                   imu_setting(address, 0x11, 0x40);   /* G: 104 Hz, +/-250 dps */
        }
    }
    return 0;
}

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[1] << 8 | p[0]);
}

/* 1 fresh, 0 waiting for data, -1 I2C fault. */
static int imu_read(uint16_t address, ChassisImuSample *s)
{
    uint8_t status, bytes[14];
    if (!imu_register(address, 0x1E, &status, 1, 0)) return -1;
    if ((status & 3) != 3) return 0;
    if (!imu_register(address, 0x20, bytes, sizeof bytes, 0)) return -1;
    s->temp_raw = le16(bytes);
    s->gx_raw = le16(bytes + 2);
    s->gy_raw = le16(bytes + 4);
    s->gz_raw = le16(bytes + 6);
    s->ax_raw = le16(bytes + 8);
    s->ay_raw = le16(bytes + 10);
    s->az_raw = le16(bytes + 12);
    return 1;
}
#else
#define IMU_ADDR_LOW 0x68
#define IMU_ID_REG 0x75
#define IMU_EXPECTED_ID 0x70

static int imu_configure(uint16_t address)
{
    if (!imu_setting(address, 0x6B, 1)) return 0;
    osDelay(50);
    return imu_setting(address, 0x1B, 0) && imu_setting(address, 0x1C, 0);
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] << 8 | p[1]);
}

static int imu_read(uint16_t address, ChassisImuSample *s)
{
    uint8_t bytes[14];
    if (!imu_register(address, 0x3B, bytes, sizeof bytes, 0)) return -1;
    s->ax_raw = be16(bytes);
    s->ay_raw = be16(bytes + 2);
    s->az_raw = be16(bytes + 4);
    s->temp_raw = be16(bytes + 6);
    s->gx_raw = be16(bytes + 8);
    s->gy_raw = be16(bytes + 10);
    s->gz_raw = be16(bytes + 12);
    return 1;
}
#endif

static void imu_task(void *arg)
{
    (void)arg;
    uint16_t address = 0;
    uint8_t who;
    while (!address) {
        for (uint16_t a = IMU_ADDR_LOW; a <= IMU_ADDR_LOW + 1; ++a) {
            if (!imu_register(a << 1, IMU_ID_REG, &who, 1, 0)) continue;
            imu_who_am_i = who;
            if (who == IMU_EXPECTED_ID && imu_configure(a << 1)) {
                address = a << 1;
                break;
            }
        }
        if (!address) { ++imu_errors; osDelay(250); }
    }
    uint32_t bias_start_ms = osKernelGetTickCount();
#if CHASSIS_IMU == CHASSIS_LSM6DS3TR_C
    /* static10 shows cold-start transients. Keep publishing raw frames during settling. */
    bias_start_ms += 2000U;
#endif
    osDelay(50);
    float sum = 0, bias = 0;
    unsigned samples = 0;
    uint32_t seq = 0;
#if CHASSIS_IMU == CHASSIS_LSM6DS3TR_C
    /* Poll at 200 Hz so the task can consume the sensor's 104 Hz output. */
    __HAL_TIM_SET_AUTORELOAD(&htim7, 49);
#endif
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) Error_Handler();
    for (;;) {
        osSemaphoreAcquire(imu_tick, osWaitForever);
        uint32_t start = timing_begin(IMU);
        ChassisImuSample sample;
        int fresh = imu_read(address, &sample);
        if (fresh > 0) {
            sample.seq = seq++;
            sample.read_tick_ms = osKernelGetTickCount();
            float z = sample.gz_raw;
            if (samples < 100 && (int32_t)(sample.read_tick_ms - bias_start_ms) >= 0) {
                sum += z;
                if (++samples == 100) bias = sum / 100;
            }
            taskENTER_CRITICAL();
            latest_imu = sample;
            has_imu_sample = 1;
            gyro_rate = GYRO_SIGN * (z - bias) * IMU_DPS_PER_COUNT * (PI / 180.0f);
            gyro_at = sample.read_tick_ms;
            imu_ready = samples == 100;
            taskEXIT_CRITICAL();
        } else if (fresh < 0) {
            ++imu_errors;
        }
        timing_end(IMU, start);
    }
}
//member4 code end

//member5 code begin
/* SSD1306 128x64。每次只发送一小段，避免长时间占用 IMU 总线。 */
static int oled_write(const uint8_t *bytes, uint16_t len)
{
    if (osMutexAcquire(i2c_lock, 5) != osOK) {
        record_i2c(&oled_diag, 4, 0, OLED_ADDRESS, len);
        return 0;
    }
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, OLED_ADDRESS,
                                                      (uint8_t *)bytes, len, 5);
    record_i2c(&oled_diag, status, HAL_I2C_GetError(&hi2c2), OLED_ADDRESS, len);
    recover_i2c(status, oled_diag.error);
    osMutexRelease(i2c_lock);
    return status == HAL_OK;
}

static void oled_task(void *arg)
{
    (void)arg;
    static const uint8_t init[] = {0,0xD5,0x80,0xA8,63,0xD3,0,0x40,0x8D,0x14,
        0x20,0x02,0xA1,0xC8,0xDA,0x12,0x81,0x7F,0xD9,0xF1,0xDB,0x40,0xA4,0xA6};
    static const uint8_t glyph[][5] = {
        {0x3E,0x51,0x49,0x45,0x3E},{0,0x42,0x7F,0x40,0},{0x42,0x61,0x51,0x49,0x46},
        {0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30},{1,0x71,9,5,3},{0x36,0x49,0x49,0x49,0x36},
        {6,0x49,0x49,0x29,0x1E},{8,8,8,8,8},{0x63,0x14,8,0x14,0x63},
        {3,4,0x78,4,3},{0x7F,8,8,8,0x7F},{0x26,0x49,0x49,0x49,0x32},{0,0,0,0,0}
    };
    const char *alphabet = "0123456789-XYHS";
    static const uint8_t display_on[] = {0, 0xAF};
    for (;;) {
        // 恢复显示配置时不主动关屏，避免一次通信失败清除可见内容。
        if (!oled_write(init, sizeof init)) {
            ++oled_errors;
            osDelay(250);
            continue;
        }

        int visible = 0;
        for (;;) {
            uint32_t start = timing_begin(OLED);
            Pose p;
            unsigned state;
            taskENTER_CRITICAL();
            p = pose;
            state = tracking_state;
            taskEXIT_CRITICAL();
            int values[] = {(int)(1000 * p.x), (int)(1000 * p.y),
                            (int)(p.heading * 180 / PI), (int)state};

            int ok = 1;
            for (unsigned page = 0; page < 8 && ok; ++page) {
                // 每页先清零，偶数页写字，奇数页和右侧留黑。
                uint8_t pixels[128] = {0};
                if (page % 2 == 0) {
                    unsigned row = page / 2;
                    char line[16];
                    snprintf(line, sizeof line, "%c%6d", "XYHS"[row], values[row]);
                    for (unsigned i = 0; line[i] != '\0'; ++i) {
                        const char *symbol = strchr(alphabet, line[i]);
                        memcpy(pixels + 6 * i, glyph[symbol ? symbol - alphabet : 15], 5);
                    }
                }
                for (unsigned col = 0; col < sizeof pixels && ok; col += 16) {
                    uint8_t cmd[] = {0, (uint8_t)(0xB0 + page),
                                     (uint8_t)(col & 15), (uint8_t)(0x10 | (col >> 4))};
                    uint8_t data[17] = {0x40};
                    memcpy(data + 1, pixels + col, sizeof data - 1);
                    ok = oled_write(cmd, sizeof cmd) && oled_write(data, sizeof data);
                    // 小块传输之间让出总线和 CPU，避免 IMU 连续等待。
                    if (ok) osDelay(4);
                }
            }
            if (ok && !visible) {
                ok = oled_write(display_on, sizeof display_on);
                visible = ok;
            }
            timing_end(OLED, start);
            if (!ok) {
                ++oled_errors;
                osDelay(50);
                break;
            }
        }
    }
}
//member5 code end

//member6 code begin
static float clamp(float x, float limit) { return fminf(limit, fmaxf(-limit, x)); }

static float wrap(float x)
{
    while (x > PI) x -= 2 * PI;
    while (x < -PI) x += 2 * PI;
    return x;
}

static int parse_goal(const char *line, Point *goal)
{
    while (*line == ' ') ++line;
    int bracket = *line == '(';
    if (bracket) ++line;
    char *end;
    goal->x = strtof(line, &end);
    if (end == line) return 0;
    while (*end == ' ') ++end;
    if (*end != ',') return 0;
    line = end + 1;
    goal->y = strtof(line, &end);
    if (end == line) return 0;
    while (*end == ' ') ++end;
    if (bracket && *end++ != ')') return 0;
    while (*end == ' ') ++end;
    return !*end && isfinite(goal->x) && isfinite(goal->y) &&
        fabsf(goal->x) <= 1.5f && fabsf(goal->y) <= 1.5f;
}

static void bluetooth_task(void *arg)
{
    (void)arg;
    uint8_t byte;
    char line[48];
    unsigned used = 0;
    int discard = 0;
    uint32_t seen_errors = 0;
    if (HAL_UART_Receive_IT(&huart4, &bt_byte, 1) != HAL_OK) ++rx_errors;
    for (;;) {
        osMessageQueueGet(Q_Byte, &byte, NULL, osWaitForever);
        uint32_t start = timing_begin(BT_RX);
        if (seen_errors != rx_errors) { seen_errors = rx_errors; discard = 1; }
        if (byte == '\r' || byte == '\n') {
            line[used] = '\0';
            if (used && !discard) {
                Command cmd = {0};
                cmd.stop = strcmp(line, "STOP") == 0;
                if (cmd.stop || parse_goal(line, &cmd.goal)) {
                    /* 新命令使旧路径失效。队满时车辆保持停止。 */
                    stop_requested = 1;
                    cmd.generation = ++command_generation;
                    if (osMessageQueuePut(Q_Cmd, &cmd, 0, 0) != osOK) ++lost_commands;
                }
            }
            used = 0;
            discard = 0;
        } else if (byte < 32 || byte > 126 || used == sizeof line - 1) {
            discard = 1;
        } else if (!discard) {
            line[used++] = (char)byte;
        }
        timing_end(BT_RX, start);
    }
}

static void add_segment(Path *path, Point from, Point to)
{
    unsigned n = (unsigned)ceilf(hypotf(to.x - from.x, to.y - from.y) / 0.25f);
    for (unsigned i = 1; i <= n && path->count < MAX_POINTS; ++i) {
        float f = (float)i / n;
        path->points[path->count++] = (Point){from.x + f * (to.x - from.x),
                                             from.y + f * (to.y - from.y)};
    }
}

static void planner_task(void *arg)
{
    (void)arg;
    Command cmd;
    static Path path;
    for (;;) {
        osMessageQueueGet(Q_Cmd, &cmd, NULL, osWaitForever);
        uint32_t start = timing_begin(PLANNER);
        path.count = 0;
        path.generation = cmd.generation;
        if (!cmd.stop) {
            Point from;
            taskENTER_CRITICAL();
            from = (Point){pose.x, pose.y};
            taskEXIT_CRITICAL();
            /* 假设十字赛道从 (0,0) 出发，先走 y 向走廊，再转入横向支路。 */
            Point corner = {from.x, cmd.goal.y};
            add_segment(&path, from, corner);
            add_segment(&path, corner, cmd.goal);
        }
        if (osMessageQueuePut(Q_Path, &path, 0, 0) != osOK) ++lost_commands;
        timing_end(PLANNER, start);
    }
}

static int16_t encoder_delta(TIM_HandleTypeDef *timer, uint16_t *previous)
{
    uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(timer);
    int16_t change = (int16_t)(uint16_t)(current - *previous);
    *previous = current;
    return change;
}

static void control_task(void *arg)
{
    (void)arg;
    static Path path, incoming;
    unsigned waypoint = 0;
    uint16_t prev_left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint16_t prev_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);
    float integral = 0, previous_error = 0;
    uint32_t next = osKernelGetTickCount(), last = next;
    Pose p = {0, 0, PI / 2};
    unsigned state = 0;
    uint32_t observed_rx_errors = rx_errors;
    uint32_t launch_at = 0;
    int launch_started = 0;
    uint32_t pivot_at = 0;
    uint32_t pivot_unsettled_at = 0;
    int pivoting = 0;
    for (;;) {
        uint32_t start = timing_begin(CONTROL);
        uint32_t now = osKernelGetTickCount();
        float dt = fmaxf((float)(now - last) * 0.001f, 0.001f);
        last = now;
        float metres = PI * WHEEL_DIAMETER / COUNTS_PER_REV;
        float dl = metres * LEFT_ENCODER_SIGN * encoder_delta(&htim3, &prev_left);
        float dr = metres * RIGHT_ENCODER_SIGN * encoder_delta(&htim8, &prev_right);
        float turn = (dr - dl) / WHEEL_TRACK;
        int ready;
        float yaw_rate;
        taskENTER_CRITICAL();
        ready = imu_ready && (osKernelGetTickCount() - gyro_at <= 35);
        // IMU 有效时用角速度积分航向，避免转弯打滑拉偏估计。
        yaw_rate = gyro_rate;
        if (ready) turn = yaw_rate * dt;
        taskEXIT_CRITICAL();
        float middle = p.heading + turn / 2;
        p.x += (dl + dr) / 2 * cosf(middle);
        p.y += (dl + dr) / 2 * sinf(middle);
        p.heading = wrap(p.heading + turn);

        while (osMessageQueueGet(Q_Path, &incoming, NULL, 0) == osOK) {
            taskENTER_CRITICAL();
            if (incoming.generation == command_generation) {
                path = incoming;
                launch_started = 0;
                waypoint = 0;
                pivoting = 0;
                stop_requested = incoming.count == 0 || !ready;
                state = stop_requested ? 0U : 1U;
                integral = previous_error = 0;
            }
            taskEXIT_CRITICAL();
        }
        if (!ready || observed_rx_errors != rx_errors) stop_requested = 1;
        observed_rx_errors = rx_errors;
        float left = 0, right = 0;
        if (!stop_requested && waypoint < path.count) {
            Point target = path.points[waypoint];
            float dx = target.x - p.x, dy = target.y - p.y, distance = hypotf(dx, dy);
            if (distance < 0.035f) {
                if (fabsf(yaw_rate) < 0.20f && ++waypoint == path.count) {
                    stop_requested = 1;
                    state = 2;
                }
                pivoting = 0;
                integral = previous_error = 0;
            } else {
                float error = wrap(atan2f(dy, dx) - p.heading);
                integral = clamp(integral + error * dt, 0.5f);
                float steer = clamp(0.24f * error + 0.05f * integral +
                                    0.001f * wrap(error - previous_error) / dt, 0.34f);
                /* 补偿低速驱动死区，到达阈值内仍按原逻辑停止。 */
                float heading_error = fabsf(error);
                float yaw_speed = fabsf(yaw_rate);
                int aligned = heading_error <= 0.14f && yaw_speed < 0.20f;
                if (!pivoting && heading_error >= 0.45f) {
                    pivoting = 1;
                    pivot_at = pivot_unsettled_at = now;
                }
                if (pivoting && !aligned) pivot_unsettled_at = now;
                if (pivoting && aligned && now - pivot_unsettled_at >= 80U) {
                    pivoting = 0;
                    integral = 0;
                    steer = clamp(0.24f * error, 0.05f);
                }
                float forward = 0;
                if (pivoting) {
                    // 按当前角速度预留 150 ms 的滑行角度，接近目标先撤 PWM。
                    float closing_rate = error > 0.0f ? yaw_rate : -yaw_rate;
                    float predicted_error = heading_error - 0.15f * fmaxf(0.0f, closing_rate);
                    steer = 0;
                    if (predicted_error > 0.14f && yaw_speed < 1.75f) {
                        float duty = now - pivot_at < 120U && heading_error >= 0.70f
                            ? 0.58f : 0.40f;
                        steer = copysignf(duty, error);
                    }
                } else {
                    forward = fminf(0.56f, fmaxf(0.42f, 0.36f + 0.8f * distance));
                    steer = clamp(steer, 0.12f);
                }
                // 每条新路线只助推一次，等车头对准再开始计时。
                if (forward > 0.0f && !launch_started) {
                    launch_started = 1;
                    launch_at = now;
                }
                if (forward > 0.0f && now - launch_at < 200U && distance > 0.07f) {
                    forward = 0.62f;
                    steer = clamp(steer, 0.05f);
                }
                previous_error = error;
                left = clamp(forward - steer, 0.67f);
                right = clamp(forward + steer, 0.67f);
            }
        }
        taskENTER_CRITICAL();
        if (stop_requested && state != 2) state = 0;
        motors(stop_requested ? 0 : left, stop_requested ? 0 : right);
        pose = p;
        tracking_state = state;
        taskEXIT_CRITICAL();
        timing_end(CONTROL, start);
        periodic_wait(&next, 10);
    }
}
//member6 code end

void CodingDay_Init(void)
{
    const osMutexAttr_t attr = {.name = "I2C2", .attr_bits = osMutexPrioInherit};
    i2c_lock = osMutexNew(&attr);
    const osMutexAttr_t tx = {.name = "UART4_TX", .attr_bits = osMutexPrioInherit};
    bluetooth_tx_lock = osMutexNew(&tx);
    imu_tick = osSemaphoreNew(1, 0, NULL);
    if (!i2c_lock || !imu_tick || !bluetooth_tx_lock) Error_Handler();
    create_queues();
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK ||
        HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    create_tasks();
}
