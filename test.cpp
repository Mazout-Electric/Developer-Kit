#include <Arduino.h>
#include <stm32g4xx_hal.h>
#include <HardwareSerial.h>

HardwareSerial Serial1(PA10, PA9); // RX, TX

/* ── FDCAN globals ── */
FDCAN_HandleTypeDef   hfdcan1;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04};
uint8_t RxData[8];

void MX_FDCAN1_Init(void);
void FDCAN_Config(void);

/* ─────────────────────────────────────────────────────── */
void setup()
{
    Serial1.begin(115200);
    delay(500); // give serial time to come up

    Serial1.println("Boot OK");

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Serial1.println("ERROR: FDCAN clock source config failed");
        while(1);
    }

    MX_FDCAN1_Init();
    FDCAN_Config();

    Serial1.println("FDCAN init done");
}

/* ─────────────────────────────────────────────────────── */
void loop()
{
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {
        uint32_t err = HAL_FDCAN_GetError(&hfdcan1);
        uint32_t state = hfdcan1.State;
        uint32_t txFree = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);

        Serial1.print("FDCAN state: ");
        Serial1.println(state);
        Serial1.print("FDCAN error reg: 0x");
        Serial1.println(err, HEX);
        Serial1.print("TX fifo free slots: ");
        Serial1.println(txFree);
    } else {
        Serial1.print("TX sent: ID=0x");
        Serial1.print(TxHeader.Identifier, HEX);
        Serial1.print(" data=");
        for (int i = 0; i < 4; i++) {
            Serial1.print(TxData[i], HEX);
            Serial1.print(" ");
        }
        Serial1.println();
    }

    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,
                                   &RxHeader, RxData) == HAL_OK)
        {
            Serial1.print("RX got: ID=0x");
            Serial1.print(RxHeader.Identifier, HEX);
            Serial1.print(" data=");
            for (int i = 0; i < 8; i++) {
                Serial1.print(RxData[i], HEX);
                Serial1.print(" ");
            }
            Serial1.println();
        }
    }

    // Print bus error state every loop so we can see what's happening
    Serial1.print("PSR: 0x");
    Serial1.println(hfdcan1.Instance->PSR, HEX);

    delay(500);
}

/* ─────────────────────────────────────────────────────── */
void MX_FDCAN1_Init(void)
{
    /* GPIO: PA11=RX, PA12=TX, AF9 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_FDCAN_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hfdcan1.Instance                  = FDCAN1;
    hfdcan1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission   = ENABLE;
    hfdcan1.Init.TransmitPause        = ENABLE;
    hfdcan1.Init.ProtocolException    = DISABLE;

    /* 500 kbit/s — tune these if your baud rate differs */
    hfdcan1.Init.NominalPrescaler     = 15;
    hfdcan1.Init.NominalSyncJumpWidth = 3;
    hfdcan1.Init.NominalTimeSeg1      = 16;
    hfdcan1.Init.NominalTimeSeg2      = 3;

    // Data timing same (unused in classic CAN but must be valid)
    hfdcan1.Init.DataPrescaler        = 15;
    hfdcan1.Init.DataSyncJumpWidth    = 3;
    hfdcan1.Init.DataTimeSeg1         = 16;
    hfdcan1.Init.DataTimeSeg2         = 3;

    hfdcan1.Init.StdFiltersNbr        = 1;
    hfdcan1.Init.ExtFiltersNbr        = 0;
    hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        Serial1.println("ERROR: FDCAN init failed");
        while(1);
    }

    hfdcan1.Instance->CCCR &= ~FDCAN_CCCR_MON;
}

/* ─────────────────────────────────────────────────────── */
void FDCAN_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0x000;
    sFilterConfig.FilterID2    = 0x000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Serial1.println("ERROR: filter config failed");
        while(1);
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
            FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Serial1.println("ERROR: global filter failed");
        while(1);
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Serial1.println("ERROR: FDCAN start failed");
        while(1);
    }

    TxHeader.Identifier          = 0x123;
    TxHeader.IdType              = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    TxHeader.FDFormat             = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker        = 0;
}
