#if defined(ESP_PLATFORM)

#include "../nts1_impl.h"

#include "assert.h"
#include "esp_types.h"
#include "stdio.h"

#include "driver/spi_slave.h"
#include "driver/gpio.h"

// -----------------------------
// pins and ports
#define SPI_MISO_PIN 19 // TX_PANEL
#define SPI_MOSI_PIN 23 // RX_PANEL
#define SPI_SCK_PIN 18  // SCK
#define ACK_PIN 21      // ACK

#define SPI_CS_PIN 5 // - None, controlled by software, allways on

#define SPI_MODE 3
#define SPI_BITORDER SPI_SLAVE_BIT_LSBFIRST

#define S_SPI_HOST VSPI_HOST // Use the SPI3 device
#define DMA_CHANNEL 0        // disable dma, use direct spi buffer

// #define SPI_QUEUE_TTW 0
#define SPI_QUEUE_TTW portMAX_DELAY

// ----------------------------------------

extern inline void s_port_startup_ack(void)
{
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << ACK_PIN));
}

extern inline void s_port_wait_ack(void)
{
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1 << ACK_PIN));
}

// ----------------------------------------------------


// Called after a transaction is queued and ready for pickup by master. We use this to set the ACK line high.
void s_spi_irq_handler_post_setup(spi_slave_transaction_t *trans)
{
    if (!s_spi_chk_rx_buf_space(32))
    {
        // the ACK pin is set to 0
        s_port_wait_ack();
        // I assume this means the NTS1 will stop sending data
    }
    else
    { //Remaining buffer
        // otherwise the ACK pin is set to 1
        s_port_startup_ack();
        // which will allow the NTS1 to send data
    }
}

// Called after transaction is sent/received. We use this to set the ACK line low.
void s_spi_irq_handler_post_transaction(spi_slave_transaction_t *trans)
{
    s_port_wait_ack();
    if (!s_spi_chk_rx_buf_space((SPI_RX_BUF_SIZE - 2)))
    {
        // If RxBuf is full, reset it.
        SPI_RX_BUF_RESET();
    }
}

// ----------------------------------------------------

nts1_status_t s_spi_init()
{
    // DMA interrupts:
    // SPI_IN_DONE_INT
    // SPI_OUT_DONE_INT

    //Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_SCK_PIN,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = SPI_MODE,
        .spics_io_num = SPI_CS_PIN,
        .queue_size = 3,
        .flags = SPI_BITORDER,
        .post_setup_cb = s_spi_irq_handler_post_setup,
        .post_trans_cb = s_spi_irq_handler_post_transaction,
    };

    // Pull down on the Chip Select pin to make it always on
    gpio_set_pull_mode(SPI_CS_PIN, GPIO_PULLDOWN_ONLY);

    // TODO check that this works, the line wasn't like this in the main.c program
    // Pull the SCK line up (CPOL = 1, normally high)
    //gpio_set_pull_mode(SPI_SCK_PIN, GPIO_PULLUP_ONLY);

    //Initialize SPI slave interface
    if (!spi_slave_initialize(S_SPI_HOST, &buscfg, &slvcfg, DMA_CHANNEL))
    {

        return k_nts1_status_error;
    }


    return k_nts1_status_ok;
}

nts1_status_t s_spi_teardown()
{
    spi_slave_free(S_SPI_HOST);
    return k_nts1_status_ok;
}

void s_ack_init()
{
    // Configuration for the handshake line
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1 << ACK_PIN)};

    //Configure handshake line as output
    gpio_config(&io_conf);
}

// ----------------------------------------------------
// Interrupt

// ----------------------------------------------------
#define SPI_TRANSACTION_BYTES 4 // 4 bytes is the minimum the master supports

nts1_status_t nts1_idle()
{
    uint8_t txdata;
    spi_slave_transaction_t transaction;
    transaction.length = 8*SPI_TRANSACTION_BYTES; 
    uint8_t *tx_buf_ptr_first_byte = s_spi_tx_buf+s_spi_tx_ridx;

    bool has_data = false;
    // HOST <- PANEL
    for (uint8_t i = 0; i < SPI_TRANSACTION_BYTES; i++)
    {
        if (SPI_TX_BUF_EMPTY()){
            break;
        } else { // Send buffer has data
            has_data = true;
            uint8_t *tx_buf_ptr = s_spi_tx_buf+s_spi_tx_ridx;
            // If there's data to be sent
            txdata = s_spi_tx_buf_read(); // read it and advance the read idx pointer
            if (txdata & PANEL_START_BIT)
            { // Check if the data contains 0x80 (is a Status message)
                // if (!SPI_TX_BUF_EMPTY())
                // { // There is (more) data (after the status) to be sent next in the send buffer
                    // an EMARK is set on the status
                    txdata |= PANEL_CMD_EMARK;
                    // Note: this will set endmark on almost any status, especially those who have pending data,
                    //       which seems to contradict the endmark common usage of marking only the last command of a group
                // }
            }

            // Save changes to the buffer
            *tx_buf_ptr = txdata;
        }
    }

    if(has_data){
        transaction.tx_buffer = (void *) tx_buf_ptr_first_byte;
        // Point the rx buffer of the transaction to the current write idx
        uint8_t *rx_buf_ptr = s_spi_rx_buf + s_spi_rx_widx;
        // Increase the write idx to avoid ovewriting the reseved memory
        s_spi_rx_widx = SPI_BUF_INC(s_spi_rx_widx, SPI_RX_BUF_SIZE);
        transaction.rx_buffer = (void *) rx_buf_ptr;
        
        spi_slave_queue_trans(S_SPI_HOST, &transaction, SPI_QUEUE_TTW);
    }

    // write one byte of data from the Tx buffer to the Tx FIFO.
    // if the byte is a Status message, and it has more data after it,
    // add the ENDMARK (byte 7 set to 1)
    // If there's no data to be sent, write the DUMMY message
    // HOST I/F Give priority to Idle processing of received data
    // As long as the reception buffer is not empty
    while (!SPI_RX_BUF_EMPTY())
    {
        // Reads from the buffer and executes the handler
        s_rx_msg_handler(s_spi_rx_buf_read());
    }

    return k_nts1_status_ok;
}

#endif //ESP_PLATFORM