//swing analyzer - main program
//nucleo-f446re + bmi270 on spi1, data to the pc on usart2 at 460800
//every 1 ms: tim2 interrupt pulls cs low and starts a 14 byte spi dma read
//dma done interrupt: cs high, unpack the sample, put it in the ring buffer
//main loop: takes samples out, runs the swing state machine, sends packets by uart dma
//debug pin D7 (PA8) goes high when the timer fires and low when the read is done, for the scope

#include "app.h"
#include "main.h"
#include "bmi270.h"
#include "frame.h"
#include "swing_detect.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

//pins
#define IMU_CS_PORT  GPIOB
#define IMU_CS_PIN   GPIO_PIN_6     //D10
#define DBG_PORT     GPIOA
#define DBG_PIN      GPIO_PIN_8     //D7

#define SAMPLE_RATE_HZ 1000u

//swing settings, starting guesses that still need tuning with real swings
//analyze_log.py --redetect can try new values on a saved recording
#define DPS_TO_RAW(dps) ((uint16_t)((dps) * 32768u / BMI270_GYR_FS_DPS))
static const swing_cfg_t SWING_CFG = {
    .start_raw        = DPS_TO_RAW(300),
    .end_raw          = DPS_TO_RAW(150),
    .start_samples    = 5,                //5 ms
    .end_samples      = 50,               //50 ms
    .min_samples      = 50,               //50 ms
    .max_samples      = 2000,             //2 s
    .cooldown_samples = 300,              //300 ms
    .skip_axis        = 1,                //y runs along the handle on this mount, spinning the racket in
                                          //your hand is almost all y and was counted as swings
};

//ring buffer, the interrupt adds samples and the main loop takes them out
#define RING_SIZE 256u                    //has to be a power of 2
static imu_sample_t s_ring[RING_SIZE];
static volatile uint32_t s_ring_head;     //only the interrupt writes this
static volatile uint32_t s_ring_tail;     //only the main loop writes this

//shared with the interrupts
static bmi270_t s_imu = { &hspi1, IMU_CS_PORT, IMU_CS_PIN, 0, 0 };
static bmi270_status_t s_imu_status = BMI270_ERR_SPI;
static uint8_t s_spi_tx[BMI270_BURST_XFER_LEN];
static uint8_t s_spi_rx[BMI270_BURST_XFER_LEN];
static volatile uint8_t  s_spi_busy;
static volatile uint32_t s_tick_seq;      //goes up every tick even if the read gets skipped
static volatile uint32_t s_pending_seq, s_pending_t;
static volatile uint32_t s_missed_reads, s_ring_overflows, s_spi_errors;
static uint32_t s_tx_dropped;
static uint32_t s_rate_mhz;

//two uart buffers, the main loop fills one while dma sends the other
#define TXBUF_SIZE 1024u
static uint8_t  s_txbuf[2][TXBUF_SIZE];
static uint16_t s_txlen[2];
static uint8_t  s_txfill;                 //which buffer is being filled
static volatile uint8_t s_uart_busy;

static swing_detector_t s_swing;

//cycle counter for timestamps, 5.6 ns per count at 180 mhz
//it wraps every ~24 s but subtracting two readings still works
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

//apb1 timers run at 2x pclk1 when the apb1 divider isn't 1
static uint32_t tim2_clock_hz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    return (ppre1 < 4u) ? pclk1 : 2u * pclk1;
}

//tim2 counts at 1 mhz and overflows every 1000 counts = 1 khz, whatever clock cubemx set
static void timer_setup_1khz(void)
{
    uint32_t clk = tim2_clock_hz();
    uint32_t psc = clk / 1000000u - 1u;
    uint32_t arr = 1000000u / SAMPLE_RATE_HZ - 1u;
    __HAL_TIM_SET_PRESCALER(&htim2, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    htim2.Instance->EGR = TIM_EGR_UG;              //load the new prescaler now
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    //configured rate in millihertz for the status packet, not a measurement
    s_rate_mhz = (uint32_t)((uint64_t)clk * 1000u / ((uint64_t)(psc + 1u) * (arr + 1u)));
}

static void tx_queue(const uint8_t *frame, size_t n)
{
    if ((size_t)s_txlen[s_txfill] + n > TXBUF_SIZE) { s_tx_dropped++; return; }
    memcpy(&s_txbuf[s_txfill][s_txlen[s_txfill]], frame, n);
    s_txlen[s_txfill] = (uint16_t)(s_txlen[s_txfill] + n);
}

//start sending the filled buffer if the uart is free
static void tx_kick(void)
{
    if (s_uart_busy || s_txlen[s_txfill] == 0) return;
    uint8_t b = s_txfill;
    s_txfill ^= 1u;
    s_txlen[s_txfill] = 0;
    s_uart_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart2, s_txbuf[b], s_txlen[b]) != HAL_OK) {
        s_uart_busy = 0;
        s_tx_dropped++;
    }
}

static void send_status(void)
{
    uint8_t f[FRAME_MAX_BYTES];
    status_info_t st = {0};
    st.core_hz        = SystemCoreClock;
    st.uptime_ms      = HAL_GetTick();
    st.ticks          = s_tick_seq;
    st.missed_reads   = s_missed_reads;
    st.ring_overflows = s_ring_overflows;
    st.tx_dropped     = s_tx_dropped;
    st.spi_errors     = s_spi_errors;
    st.swing_count    = s_swing.count;
    st.rate_mhz       = s_rate_mhz;
    st.gyro_fs_dps    = BMI270_GYR_FS_DPS;
    st.acc_fs_g       = BMI270_ACC_FS_G;
    st.bmi_status     = (uint8_t)s_imu_status;
    st.chip_id        = s_imu.chip_id;
    st.bmi_diag       = s_imu.diag;
    tx_queue(f, frame_encode_status(f, &st));
}

void app_init(void)
{
    dwt_init();
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DBG_PORT, DBG_PIN, GPIO_PIN_RESET);

    swing_init(&s_swing, &SWING_CFG);
    s_imu_status = bmi270_init(&s_imu);
    bmi270_prepare_burst(s_spi_tx);
    timer_setup_1khz();

    send_status();     //first status goes out even if the imu failed
    tx_kick();

    if (s_imu_status == BMI270_OK) {
        HAL_TIM_Base_Start_IT(&htim2);   //sampling starts here
    }
}

void app_loop(void)
{
    static uint32_t last_status_ms;
    uint8_t f[FRAME_MAX_BYTES];

    //empty the ring buffer
    while (s_ring_tail != s_ring_head) {
        imu_sample_t s = s_ring[s_ring_tail & (RING_SIZE - 1u)];
        __DMB();                         //finish reading before freeing the slot
        s_ring_tail = s_ring_tail + 1u;

        tx_queue(f, frame_encode_sample(f, &s));

        swing_event_t ev;
        if (swing_update(&s_swing, s.seq, s.gyr, s.flags, &ev)) {
            tx_queue(f, frame_encode_swing(f, &ev));
        }
    }

    uint32_t now = HAL_GetTick();
    if (now - last_status_ms >= 1000u) {
        last_status_ms = now;
        send_status();
    }

    tx_kick();
}

//interrupt callbacks, hal calls these by name

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;

    uint32_t t = DWT->CYCCNT;            //timestamp first, for jitter
    DBG_PORT->BSRR = DBG_PIN;            //debug pin high
    uint32_t seq = s_tick_seq++;

    if (s_spi_busy) {                    //last read still going, skip this one
        s_missed_reads++;
        return;                          //shows up as a seq gap on the pc
    }
    s_spi_busy = 1;
    s_pending_seq = seq;
    s_pending_t = t;
    IMU_CS_PORT->BSRR = (uint32_t)IMU_CS_PIN << 16;   //cs low
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, s_spi_tx, s_spi_rx, BMI270_BURST_XFER_LEN) != HAL_OK) {
        IMU_CS_PORT->BSRR = IMU_CS_PIN;               //cs high
        DBG_PORT->BSRR = (uint32_t)DBG_PIN << 16;
        s_spi_errors++;
        s_spi_busy = 0;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;
    IMU_CS_PORT->BSRR = IMU_CS_PIN;                   //cs high

    imu_sample_t s;
    s.seq = s_pending_seq;
    s.t_cyc = s_pending_t;
    bmi270_parse_burst(s_spi_rx, s.acc, s.gyr);
    s.flags = 0;
    for (int i = 0; i < 3; i++) {
        if (s.gyr[i] == INT16_MAX || s.gyr[i] == INT16_MIN) s.flags |= FLAG_GYRO_CLIPPED;
        if (s.acc[i] == INT16_MAX || s.acc[i] == INT16_MIN) s.flags |= FLAG_ACC_CLIPPED;
    }

    uint32_t head = s_ring_head;
    if (head - s_ring_tail >= RING_SIZE) {
        s_ring_overflows++;                           //main loop fell behind
    } else {
        s_ring[head & (RING_SIZE - 1u)] = s;
        __DMB();                                      //write the data before moving head
        s_ring_head = head + 1u;
    }
    s_spi_busy = 0;
    DBG_PORT->BSRR = (uint32_t)DBG_PIN << 16;         //debug pin low
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;
    IMU_CS_PORT->BSRR = IMU_CS_PIN;
    DBG_PORT->BSRR = (uint32_t)DBG_PIN << 16;
    s_spi_errors++;
    s_spi_busy = 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) s_uart_busy = 0;
}
