/**
 * @file    drv_spiflash.c
 * @brief   EN25Q128 driver over SPI1 (PB3 SCK / PB4 MISO / PB5 MOSI, CS = PB14).
 *
 * Notes for STM32F4:
 *   PB3/PB4 default to JTDO/NJTRST. Unlike F1 there is no AFIO remap needed:
 *   setting the pins to AF5 is enough, SWD (PA13/PA14) stays usable.
 */

#include "stm32f4xx_hal.h"
#include "bsp_pinmap.h"
#include "drv_spiflash.h"

/* command set */
#define CMD_WRITE_ENABLE     0x06u
#define CMD_READ_STATUS      0x05u
#define CMD_READ_DATA        0x03u
#define CMD_PAGE_PROGRAM     0x02u
#define CMD_SECTOR_ERASE     0x20u   /* 4KB  */
#define CMD_BLOCK_ERASE      0xD8u   /* 64KB */
#define CMD_JEDEC_ID         0x9Fu

#define STATUS_BUSY          0x01u

#define SPI_TIMEOUT_MS       1000u
#define ERASE_SECTOR_MS      500u
#define ERASE_BLOCK_MS       2000u
#define PROGRAM_PAGE_MS      100u

static SPI_HandleTypeDef s_spi;
static uint8_t           s_dummy[64];   /* 0xFF filler for read phases */

/* ---------------------------------------------------------------- helpers */
static void cs_low(void)
{
    HAL_GPIO_WritePin(PIN_FLASH_CS_PORT, PIN_FLASH_CS_PIN, GPIO_PIN_RESET);
}

static void cs_high(void)
{
    HAL_GPIO_WritePin(PIN_FLASH_CS_PORT, PIN_FLASH_CS_PIN, GPIO_PIN_SET);
}

/* Send only. */
static void spi_tx(const uint8_t *buf, uint16_t len)
{
    HAL_SPI_Transmit(&s_spi, (uint8_t *)buf, len, SPI_TIMEOUT_MS);
}

/* Clock in len bytes while shifting out 0xFF. */
static void spi_rx(uint8_t *dst, uint32_t len)
{
    uint16_t chunk;

    while (len > 0u) {
        chunk = (len > sizeof(s_dummy)) ? (uint16_t)sizeof(s_dummy) : (uint16_t)len;
        HAL_SPI_TransmitReceive(&s_spi, s_dummy, dst, chunk, SPI_TIMEOUT_MS);
        dst += chunk;
        len -= chunk;
    }
}

static void flash_wren(void)
{
    uint8_t cmd = CMD_WRITE_ENABLE;

    cs_low();
    spi_tx(&cmd, 1);
    cs_high();
}

/* Poll the BUSY bit until it clears. 0 = idle, -1 = timeout. */
static int flash_wait_busy(uint32_t timeout_ms)
{
    uint8_t  cmd = CMD_READ_STATUS;
    uint8_t  st;
    uint32_t t0 = HAL_GetTick();

    for (;;) {
        cs_low();
        spi_tx(&cmd, 1);
        spi_rx(&st, 1);
        cs_high();

        if ((st & STATUS_BUSY) == 0u) return 0;
        if ((HAL_GetTick() - t0) > timeout_ms) return -1;
    }
}

static void addr_to_cmd(uint8_t *cmd, uint8_t op, uint32_t addr)
{
    cmd[0] = op;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)(addr);
}

/* ------------------------------------------------------------------- api */
int drv_flash_init(void)
{
    GPIO_InitTypeDef g;
    uint32_t id;

    for (uint16_t i = 0; i < sizeof(s_dummy); i++) s_dummy[i] = 0xFFu;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* CS first: keep the chip deselected while the bus is being set up */
    HAL_GPIO_WritePin(PIN_FLASH_CS_PORT, PIN_FLASH_CS_PIN, GPIO_PIN_SET);
    g.Pin   = PIN_FLASH_CS_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(PIN_FLASH_CS_PORT, &g);

    /* SCK / MISO / MOSI as AF5. On F4 this also takes PB3/PB4 away from JTAG. */
    g.Pin       = PIN_SPI1_SCK_PIN | PIN_SPI1_MISO_PIN | PIN_SPI1_MOSI_PIN;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FAST;
    g.Alternate = PIN_SPI1_AF;
    HAL_GPIO_Init(PIN_SPI1_SCK_PORT, &g);

    s_spi.Instance               = SPI1;
    s_spi.Init.Mode              = SPI_MODE_MASTER;
    s_spi.Init.Direction         = SPI_DIRECTION_2LINES;
    s_spi.Init.DataSize          = SPI_DATASIZE_8BIT;
    s_spi.Init.CLKPolarity       = SPI_POLARITY_LOW;     /* mode 0 */
    s_spi.Init.CLKPhase          = SPI_PHASE_1EDGE;      /* mode 0 */
    s_spi.Init.NSS               = SPI_NSS_SOFT;
    s_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;   /* 84MHz/4 = 21MHz */
    s_spi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    s_spi.Init.TIMode            = SPI_TIMODE_DISABLE;
    s_spi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    s_spi.Init.CRCPolynomial     = 10;

    if (HAL_SPI_Init(&s_spi) != HAL_OK) return -1;

    id = drv_flash_read_id();
    if (id == 0x000000u || id == 0xFFFFFFu) return -2;   /* bus dead / no power */
    if ((id >> 16) != FLASH_EXPECTED_MFR)   return -3;   /* not a Winbond/EON NOR */

    return 0;
}

uint32_t drv_flash_read_id(void)
{
    uint8_t cmd = CMD_JEDEC_ID;
    uint8_t id[3] = { 0, 0, 0 };

    cs_low();
    spi_tx(&cmd, 1);
    spi_rx(id, 3);
    cs_high();

    return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | (uint32_t)id[2];
}

int drv_flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t cmd[4];

    if (buf == 0 || len == 0u) return -1;
    if (addr + len > FLASH_TOTAL_SIZE) return -1;

    addr_to_cmd(cmd, CMD_READ_DATA, addr);

    cs_low();
    spi_tx(cmd, 4);
    spi_rx(buf, len);
    cs_high();

    return 0;
}

int drv_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint8_t  cmd[4];
    uint32_t page_left;
    uint16_t chunk;

    if (buf == 0 || len == 0u) return -1;
    if (addr + len > FLASH_TOTAL_SIZE) return -1;

    while (len > 0u) {
        /* never let a single program command cross a page boundary */
        page_left = FLASH_PAGE_SIZE - (addr & (FLASH_PAGE_SIZE - 1u));
        chunk     = (len > page_left) ? (uint16_t)page_left : (uint16_t)len;

        addr_to_cmd(cmd, CMD_PAGE_PROGRAM, addr);

        flash_wren();
        cs_low();
        spi_tx(cmd, 4);
        spi_tx(buf, chunk);
        cs_high();

        if (flash_wait_busy(PROGRAM_PAGE_MS) != 0) return -2;

        addr += chunk;
        buf  += chunk;
        len  -= chunk;
    }

    return 0;
}

int drv_flash_erase_sector(uint32_t addr)
{
    uint8_t cmd[4];

    if (addr >= FLASH_TOTAL_SIZE) return -1;

    addr_to_cmd(cmd, CMD_SECTOR_ERASE, addr);

    flash_wren();
    cs_low();
    spi_tx(cmd, 4);
    cs_high();

    return (flash_wait_busy(ERASE_SECTOR_MS) == 0) ? 0 : -2;
}

int drv_flash_erase_block(uint32_t addr)
{
    uint8_t cmd[4];

    if (addr >= FLASH_TOTAL_SIZE) return -1;

    addr_to_cmd(cmd, CMD_BLOCK_ERASE, addr);

    flash_wren();
    cs_low();
    spi_tx(cmd, 4);
    cs_high();

    return (flash_wait_busy(ERASE_BLOCK_MS) == 0) ? 0 : -2;
}
