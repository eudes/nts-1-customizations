#ifndef __nts1_common_h
#define __nts1_common_h

#include "nts1_iface.h"


// ----------------------------

#define PANEL_ID_MASK    0x38  // Bits 3-5 // 00111000 
#define PANEL_CMD_EMARK  0x40  // Bit  6   // 01000000
#define PANEL_START_BIT  0x80  // Bit  7   // 10000000

#define SPI_TX_BUF_SIZE (0x200)
#define SPI_TX_BUF_MASK (SPI_TX_BUF_SIZE - 1)

#define SPI_RX_BUF_SIZE (0x200 )
#define SPI_RX_BUF_MASK (SPI_RX_BUF_SIZE - 1)


// ----------------------------------------------------


static uint8_t  s_panel_id = PANEL_ID_MASK; // Bits 3-5 "ppp"="111"
static uint8_t  s_dummy_tx_cmd = (PANEL_ID_MASK + 0xC7); // B'11ppp111;

static uint8_t  s_started;

static uint8_t  s_spi_tx_buf[SPI_TX_BUF_SIZE];
static uint16_t s_spi_tx_ridx;  // Read  Index (from s_spi_tx_buf)
static uint16_t s_spi_tx_widx;  // Write Index (to s_spi_tx_buf)

static uint8_t  s_spi_rx_buf[SPI_RX_BUF_SIZE];
static uint16_t s_spi_rx_ridx;  // Read  Index (from s_spi_rx_buf)
static uint16_t s_spi_rx_widx;  // Write Index (to s_spi_rx_buf)

static uint8_t  s_panel_rx_status;
static uint8_t  s_panel_rx_data_cnt;
static uint8_t  s_panel_rx_data[127];


// ----------------------------------------------------

#define SPI_TX_BUF_RESET() (s_spi_tx_ridx = s_spi_tx_widx = 0)
#define SPI_TX_BUF_EMPTY() ((SPI_TX_BUF_MASK & s_spi_tx_ridx) == (SPI_TX_BUF_MASK & s_spi_tx_widx))
#define SPI_RX_BUF_RESET() (s_spi_rx_ridx = s_spi_rx_widx = 0)
#define SPI_RX_BUF_EMPTY() ((SPI_RX_BUF_MASK & s_spi_rx_ridx) == (SPI_RX_BUF_MASK & s_spi_rx_widx))

#define SPI_BUF_INC(idx, bufSize) (((idx+1) == bufSize) ? 0 : idx + 1)

// --------------------------------------


#ifndef true 
#define true 1
#endif

#ifndef false
#define false 0
#endif

// --------------------------------------

static uint8_t s_spi_chk_rx_buf_space(uint16_t size)
{
  uint16_t count;

  if (s_spi_rx_ridx <= s_spi_rx_widx) {
    count = (SPI_RX_BUF_SIZE + s_spi_rx_ridx) - s_spi_rx_widx;
  } else {
    count = s_spi_rx_ridx - s_spi_rx_widx;
  }
  return (count > size);
}

static uint8_t s_spi_rx_buf_write(uint8_t data) 
{
  uint16_t bufdatacount;
  if (s_spi_rx_ridx <= s_spi_rx_widx) {
    bufdatacount = s_spi_rx_widx - s_spi_rx_ridx;
  } else {
    bufdatacount = SPI_RX_BUF_SIZE + s_spi_rx_widx - s_spi_rx_ridx;
  }
  if (bufdatacount < (SPI_RX_BUF_SIZE - 2)) {
    s_spi_rx_buf[SPI_RX_BUF_MASK & s_spi_rx_widx] = data;
    s_spi_rx_widx = SPI_BUF_INC(s_spi_rx_widx, SPI_RX_BUF_SIZE);
    return true;
  }
  return false;
}

static uint8_t s_spi_rx_buf_read(void)
{
  const uint8_t data = s_spi_rx_buf[SPI_RX_BUF_MASK & s_spi_rx_ridx];
  s_spi_rx_ridx = SPI_BUF_INC(s_spi_rx_ridx, SPI_RX_BUF_SIZE);
  return data;
}

static uint8_t s_spi_chk_tx_buf_space(uint16_t size)
{
  uint16_t count;
  if (s_spi_tx_ridx <= s_spi_tx_widx) {
    count = SPI_TX_BUF_SIZE + s_spi_tx_ridx - s_spi_tx_widx;
  } else {
    count = s_spi_tx_ridx - s_spi_tx_widx;
  }
  return (count > size);
}

// writes the byte to the buffer at the appropiate position
// and moves the buffer pointers
static void s_spi_tx_buf_write(uint8_t data)
{
  s_spi_tx_buf[SPI_TX_BUF_MASK & s_spi_tx_widx] = data;
  s_spi_tx_widx = SPI_BUF_INC(s_spi_tx_widx, SPI_TX_BUF_SIZE);
}

static uint8_t s_spi_tx_buf_read(void)
{
  const uint8_t data = s_spi_tx_buf[SPI_TX_BUF_MASK & s_spi_tx_ridx];
  s_spi_tx_ridx = SPI_BUF_INC(s_spi_tx_ridx, SPI_TX_BUF_SIZE);
  return data;
}


#endif // __nts1_common_h