/**
 * @file panel_nx100i.c
 * @brief YardForce NX100i hood panel ("RM EC3", LCD + 18 keys) driver.
 *
 * Protocol reverse-engineered from the stock firmware (see NX100I-HOOD.md):
 * USART1 115200 8N1, frames `55 AA LEN ID_HI ID_LO PAYLOAD[LEN-2] SUM`
 * (SUM = byte sum of everything before it).
 *
 *   - Handshake, each step repeated until the panel answers:
 *       FFFF (empty) -> echo; FFFE (empty) -> echo; FFFD `0A 99 21` -> reply
 *       whose first payload byte is 0x0A; FFFB (empty) -> reply carrying the
 *       panel's 26-byte ID starting "RM".
 *   - Keepalive every 200 ms: 5084 `<state> FF 01`, answered by 5084 `04`.
 *     2.5 s without any frame drops the link and restarts the handshake.
 *   - The panel runs its own menus; the mainboard only receives semantic
 *     commands: 5005 = START, 5006 = HOME, plus requests it must answer
 *     (5001 version, 5002 serial, 5003 clock, 5018 settings, 5030 status).
 *   - The LCD shows a message CODE sent as 5081 `<code>`; the code -> text
 *     table is read off the display on the bench (stock: 0,1,4..9,11,12,13,
 *     0x34, error numbers). Whether the panel accepts an unsolicited 5081
 *     is unverified; the stock sends it as the answer to 5030.
 *
 * This file provides the common panel.h interface. START/HOME become the
 * same UI events the 500B keypad produces (button ids 4/5), and START also
 * acts as a virtual held play button so a latched emergency can be released
 * from the hood exactly as on a 500B. All USART work is DMA/interrupt driven;
 * frames are parsed in the RX-idle callback and only flags are set there,
 * everything that transmits runs from PANEL_Tick() in the main loop.
 */
#include "board.h"

#if PANEL_TYPE == PANEL_TYPE_YARDFORCE_NX100I

#include <string.h>
#include "stm32f_board_hal.h"
#include "panel.h"
#include "main.h"

/* ---- shared symbols of the panel interface -------------------------------- */
UART_HandleTypeDef PANEL_USART_Handler;
DMA_HandleTypeDef hdma_uart1_rx;
DMA_HandleTypeDef hdma_uart1_tx;
uint8_t buttonstate[PANEL_BUTTON_BYTES + 2];
uint8_t buttonupdated = 0;
uint8_t buttoncleared = 1;
uint8_t Led_States[LED_STATE_SIZE];

/* ---- protocol constants --------------------------------------------------- */
#define NX_ID_HELLO1 0xFFFFu
#define NX_ID_HELLO2 0xFFFEu
#define NX_ID_CONFIG 0xFFFDu
#define NX_ID_IDENT 0xFFFBu
#define NX_ID_RESET 0xFFFAu
#define NX_ID_KEEPALIVE 0x5084u
#define NX_ID_VERSION 0x5001u
#define NX_ID_SERIAL 0x5002u
#define NX_ID_CLOCK 0x5003u
#define NX_ID_CLOCK_ACK 0x5004u
#define NX_ID_START 0x5005u
#define NX_ID_HOME 0x5006u
#define NX_ID_SETTINGS 0x5018u
#define NX_ID_STATUS_REQ 0x5030u
#define NX_ID_MESSAGE 0x5081u

#define NX_KEEPALIVE_MS 200u
#define NX_LINK_TIMEOUT_MS 2500u
#define NX_PANEL_ID_LEN 26u
#define NX_VIRTUAL_PLAY_MS (PLAY_BUTTON_CLEAR_EMERGENCY_MILLIS + 300u)
#define NX_RX_BUF_LEN 64u
#define NX_TX_BUF_LEN 48u
#define NX_REPLY_QUEUE_LEN 4u

static const uint8_t nx_config_payload[3] = {0x0A, 0x99, 0x21};
static const uint8_t nx_version_reply[21] = "Mowgli-Next NX100i   ";
static const uint8_t nx_serial_reply[10] = "0000000000";

typedef enum {
  NX_LINK_HELLO1 = 0,
  NX_LINK_HELLO2,
  NX_LINK_CONFIG,
  NX_LINK_IDENT,
  NX_LINK_CONNECTED
} nx_link_state_t;

/* ---- state ---------------------------------------------------------------- */
static uint8_t nx_rx_buf[NX_RX_BUF_LEN];
static uint8_t nx_tx_buf[NX_TX_BUF_LEN];
static nx_link_state_t nx_link = NX_LINK_HELLO1;
static volatile uint8_t nx_ack_mask = 0; /* bit0 FFFF, bit1 FFFE, bit2 FFFD, bit3 FFFB */
static volatile uint32_t nx_last_rx_tick = 0;
static uint32_t nx_last_keepalive_tick = 0;
static char nx_panel_id[NX_PANEL_ID_LEN + 1] = {0};
static volatile uint8_t nx_pending_start = 0;
static volatile uint8_t nx_pending_home = 0;
static volatile uint16_t nx_reply_queue[NX_REPLY_QUEUE_LEN];
static volatile uint8_t nx_reply_count = 0;
static uint8_t nx_message_code = 0;
static uint8_t nx_message_dirty = 0;
static uint32_t nx_virtual_play_until = 0;
static uint8_t nx_buttons_reported = 0;

/* ---- helpers -------------------------------------------------------------- */
static uint8_t nx_checksum(const uint8_t *buf, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++) sum = (uint8_t)(sum + buf[i]);
  return sum;
}

static uint8_t nx_wait_tx_complete(uint32_t timeout_ms) {
  const uint32_t start = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(&PANEL_USART_Handler, USART_FLAG_TC) != 1) {
    if ((HAL_GetTick() - start) >= timeout_ms) return 0;
  }
  return 1;
}

static void nx_send(uint16_t id, const uint8_t *payload, uint8_t len) {
  if ((uint32_t)len + 6u > NX_TX_BUF_LEN) return;
  if (!nx_wait_tx_complete(3)) return;
  uint8_t p = 0;
  nx_tx_buf[p++] = 0x55;
  nx_tx_buf[p++] = 0xAA;
  nx_tx_buf[p++] = (uint8_t)(len + 2u);
  nx_tx_buf[p++] = (uint8_t)(id >> 8);
  nx_tx_buf[p++] = (uint8_t)(id & 0xFFu);
  for (uint8_t i = 0; i < len; i++) nx_tx_buf[p++] = payload[i];
  nx_tx_buf[p] = nx_checksum(nx_tx_buf, p);
  p++;
  HAL_UART_Transmit_DMA(&PANEL_USART_Handler, nx_tx_buf, p);
}

static void nx_queue_reply(uint16_t id) {
  if (nx_reply_count < NX_REPLY_QUEUE_LEN) {
    nx_reply_queue[nx_reply_count++] = id;
  }
}

/* Runs in the RX-idle callback: validate one frame, set flags only. */
static void nx_handle_frame(uint16_t id, const uint8_t *payload, uint8_t plen) {
  nx_last_rx_tick = HAL_GetTick();
  switch (id) {
    case NX_ID_HELLO1: nx_ack_mask |= 0x01u; break;
    case NX_ID_HELLO2: nx_ack_mask |= 0x02u; break;
    case NX_ID_CONFIG:
      if (plen >= 1u && payload[0] == 0x0Au) nx_ack_mask |= 0x04u;
      break;
    case NX_ID_IDENT:
      if (plen >= 2u && payload[0] == 'R' && payload[1] == 'M') {
        const uint8_t n = plen < NX_PANEL_ID_LEN ? plen : NX_PANEL_ID_LEN;
        memcpy(nx_panel_id, payload, n);
        nx_panel_id[n] = '\0';
        nx_ack_mask |= 0x08u;
      }
      break;
    case NX_ID_RESET:
      nx_ack_mask = 0;
      break;
    case NX_ID_KEEPALIVE: break; /* ack: nx_last_rx_tick already refreshed */
    case NX_ID_START: nx_pending_start = 1; break;
    case NX_ID_HOME: nx_pending_home = 1; break;
    case NX_ID_VERSION:
    case NX_ID_SERIAL:
    case NX_ID_CLOCK:
    case NX_ID_SETTINGS:
    case NX_ID_STATUS_REQ:
      nx_queue_reply(id);
      break;
    default: break;
  }
}

static void nx_parse(const uint8_t *buf, uint16_t size) {
  uint16_t i = 0;
  while (i + 6u <= size) {
    if (buf[i] != 0x55 || buf[i + 1] != 0xAA) { i++; continue; }
    const uint8_t len = buf[i + 2];
    const uint16_t total = (uint16_t)len + 4u;
    if (len < 2u || i + total > size) { i++; continue; }
    if (nx_checksum(&buf[i], (uint8_t)(total - 1u)) != buf[i + total - 1u]) { i++; continue; }
    const uint16_t id = (uint16_t)((buf[i + 3] << 8) | buf[i + 4]);
    nx_handle_frame(id, &buf[i + 5], (uint8_t)(len - 2u));
    i = (uint16_t)(i + total);
  }
}

static void nx_arm_rx(void) {
  HAL_UARTEx_ReceiveToIdle_DMA(&PANEL_USART_Handler, nx_rx_buf, NX_RX_BUF_LEN);
  __HAL_DMA_DISABLE_IT(&hdma_uart1_rx, DMA_IT_HT);
}

/* ---- public interface ----------------------------------------------------- */
void PANEL_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Direct GPIO play/home inputs exist on the board; keep reading them. */
  PLAY_BUTTON_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = PLAY_BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PLAY_BUTTON_PORT, &GPIO_InitStruct);
  HOME_BUTTON_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = HOME_BUTTON_PIN;
  HAL_GPIO_Init(HOME_BUTTON_PORT, &GPIO_InitStruct);

  /* USART1 on PA9/PA10, 115200 8N1, DMA on both directions. */
  PANEL_USART_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = PANEL_USART_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(PANEL_USART_RX_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = PANEL_USART_TX_PIN;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PANEL_USART_TX_PORT, &GPIO_InitStruct);

  PANEL_USART_USART_CLK_ENABLE();
  PANEL_USART_Handler.Instance = PANEL_USART_INSTANCE;
  PANEL_USART_Handler.Init.BaudRate = 115200;
  PANEL_USART_Handler.Init.WordLength = UART_WORDLENGTH_8B;
  PANEL_USART_Handler.Init.StopBits = USART_STOPBITS_1;
  PANEL_USART_Handler.Init.Parity = UART_PARITY_NONE;
  PANEL_USART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  PANEL_USART_Handler.Init.Mode = USART_MODE_TX_RX;
  HAL_UART_Init(&PANEL_USART_Handler);

  hdma_uart1_rx.Instance = DMA2_Stream5;
  hdma_uart1_rx.Init.Channel = DMA_CHANNEL_4;
  hdma_uart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_uart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_uart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_uart1_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_uart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_uart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_uart1_rx.Init.Mode = DMA_NORMAL;
  hdma_uart1_rx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_uart1_rx) != HAL_OK) Error_Handler();
  __HAL_LINKDMA(&PANEL_USART_Handler, hdmarx, hdma_uart1_rx);

  hdma_uart1_tx.Instance = DMA2_Stream7;
  hdma_uart1_tx.Init.Channel = DMA_CHANNEL_4;
  hdma_uart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_uart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_uart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_uart1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_uart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_uart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_uart1_tx.Init.Mode = DMA_NORMAL;
  hdma_uart1_tx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_uart1_tx) != HAL_OK) Error_Handler();
  __HAL_LINKDMA(&PANEL_USART_Handler, hdmatx, hdma_uart1_tx);

  HAL_NVIC_SetPriority(PANEL_USART_IRQ, 0, 0);
  HAL_NVIC_EnableIRQ(PANEL_USART_IRQ);
  __HAL_UART_ENABLE_IT(&PANEL_USART_Handler, UART_IT_TC);

  memset(Led_States, 0, LED_STATE_SIZE);
  memset(buttonstate, 0, sizeof(buttonstate));
  buttonupdated = 0;
  buttoncleared = 1;
  nx_link = NX_LINK_HELLO1;
  nx_ack_mask = 0;
  nx_reply_count = 0;
  nx_pending_start = 0;
  nx_pending_home = 0;
  nx_last_rx_tick = HAL_GetTick();
  nx_last_keepalive_tick = 0;
  nx_arm_rx();
  DB_TRACE(" * NX100i panel: USART1 up, starting handshake\r\n");
}

/* Called every PANEL_NBT_TIME_MS (100 ms) from panel_handler(). */
void PANEL_Tick(void) {
  const uint32_t now = HAL_GetTick();

  /* --- link state machine: each step repeats until the panel answers --- */
  switch (nx_link) {
    case NX_LINK_HELLO1:
      if (nx_ack_mask & 0x01u) { nx_link = NX_LINK_HELLO2; }
      else { nx_send(NX_ID_HELLO1, NULL, 0); }
      break;
    case NX_LINK_HELLO2:
      if (nx_ack_mask & 0x02u) { nx_link = NX_LINK_CONFIG; }
      else { nx_send(NX_ID_HELLO2, NULL, 0); }
      break;
    case NX_LINK_CONFIG:
      if (nx_ack_mask & 0x04u) { nx_link = NX_LINK_IDENT; }
      else { nx_send(NX_ID_CONFIG, nx_config_payload, sizeof(nx_config_payload)); }
      break;
    case NX_LINK_IDENT:
      if (nx_ack_mask & 0x08u) {
        nx_link = NX_LINK_CONNECTED;
        nx_last_keepalive_tick = 0;
        nx_last_rx_tick = now;
        DB_TRACE(" * NX100i panel connected: %s\r\n", nx_panel_id);
      } else {
        nx_send(NX_ID_IDENT, NULL, 0);
      }
      break;
    case NX_LINK_CONNECTED:
    default:
      if ((uint32_t)(now - nx_last_rx_tick) > NX_LINK_TIMEOUT_MS) {
        nx_link = NX_LINK_HELLO1;
        nx_ack_mask = 0;
        DB_TRACE(" * NX100i panel link lost, re-handshaking\r\n");
        break;
      }
      if ((uint32_t)(now - nx_last_keepalive_tick) >= NX_KEEPALIVE_MS) {
        /* Payload semantics of bytes 1..2 are not decoded; the stock sends
         * <mower state> FF 01 and the 500B keypad driver sends 00 FF 01. */
        const uint8_t keepalive[3] = {0x00, 0xFF, 0x01};
        nx_send(NX_ID_KEEPALIVE, keepalive, sizeof(keepalive));
        nx_last_keepalive_tick = now;
      } else if (nx_reply_count > 0) {
        /* one queued reply per tick keeps the TX path simple */
        uint16_t id;
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        id = nx_reply_queue[0];
        for (uint8_t i = 1; i < nx_reply_count; i++) nx_reply_queue[i - 1] = nx_reply_queue[i];
        nx_reply_count--;
        __set_PRIMASK(primask);
        switch (id) {
          case NX_ID_VERSION: nx_send(NX_ID_VERSION, nx_version_reply, sizeof(nx_version_reply)); break;
          case NX_ID_SERIAL: nx_send(NX_ID_SERIAL, nx_serial_reply, sizeof(nx_serial_reply)); break;
          case NX_ID_CLOCK: { const uint8_t ack = 0; nx_send(NX_ID_CLOCK_ACK, &ack, 1); break; }
          case NX_ID_SETTINGS: { const uint8_t z[5] = {0}; nx_send(NX_ID_SETTINGS, z, sizeof(z)); break; }
          case NX_ID_STATUS_REQ: nx_send(NX_ID_MESSAGE, &nx_message_code, 1); nx_message_dirty = 0; break;
          default: break;
        }
      } else if (nx_message_dirty) {
        nx_send(NX_ID_MESSAGE, &nx_message_code, 1);
        nx_message_dirty = 0;
      }
      break;
  }

  /* --- buttons: panel commands plus the direct GPIO inputs --- */
  uint8_t start = 0, home = 0;
  if (nx_pending_start) { nx_pending_start = 0; start = 1; nx_virtual_play_until = now + NX_VIRTUAL_PLAY_MS; }
  if (nx_pending_home) { nx_pending_home = 0; home = 1; }
  if (!HAL_GPIO_ReadPin(PLAY_BUTTON_PORT, PLAY_BUTTON_PIN)) start = 1; /* pull-up, active low */
  if (!HAL_GPIO_ReadPin(HOME_BUTTON_PORT, HOME_BUTTON_PIN)) home = 1;

  if ((start || home) && !nx_buttons_reported) {
    buttonstate[PANEL_BUTTON_DEF_START] = start;
    buttonstate[PANEL_BUTTON_DEF_HOME] = home;
    buttonupdated = 1;   /* panel_handler emits UI events 4 (START) / 5 (HOME) */
    buttoncleared = 0;
    nx_buttons_reported = 1;
  } else if (!start && !home) {
    buttonstate[PANEL_BUTTON_DEF_START] = 0;
    buttonstate[PANEL_BUTTON_DEF_HOME] = 0;
    buttoncleared = 1;
    nx_buttons_reported = 0;
  }
}

void PANEL_Set_LED(uint8_t led, PANEL_LED_STATE state) {
  /* No discrete LEDs on the EC3; keep the states for a future code mapping. */
  if (led < LED_STATE_SIZE) Led_States[led] = (uint8_t)state;
}

void PANEL_Send_Message(const uint8_t *data, uint8_t dataLength, uint16_t command) {
  nx_send(command, data, dataLength);
}

uint8_t PANEL_NX100I_Connected(void) { return nx_link == NX_LINK_CONNECTED; }
const char *PANEL_NX100I_Id(void) { return nx_panel_id; }

void PANEL_NX100I_ShowMessageCode(uint8_t code) {
  nx_message_code = code;
  nx_message_dirty = 1;
}

uint8_t PANEL_NX100I_VirtualPlayHeld(void) {
  return (int32_t)(nx_virtual_play_until - HAL_GetTick()) > 0 ? 1u : 0u;
}

/* RX-idle DMA callback (USART1): parse whatever arrived, re-arm. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == PANEL_USART_INSTANCE) {
    if (Size > 0u && Size <= NX_RX_BUF_LEN) nx_parse(nx_rx_buf, Size);
    nx_arm_rx();
  }
}

#endif /* PANEL_TYPE == PANEL_TYPE_YARDFORCE_NX100I */
