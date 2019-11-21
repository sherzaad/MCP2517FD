#include "mcp2517fd.h"

#define MAX_TXQUEUE_ATTEMPTS 50

mcp2517fd CAN_FD_Ch1(10, 2);

uint32_t can_msg_count = 0;

uint8_t txd[MAX_DATA_BYTES]; //tramitted data frame array
uint8_t rxd[MAX_DATA_BYTES]; //receiveing data frame array
uint8_t attempts = MAX_TXQUEUE_ATTEMPTS;

CAN_TX_MSGOBJ txObj; //transmitted message FD frame
CAN_RX_MSGOBJ rxObj; //received message FD frame

void setup() {

  Serial.begin(115200);

  Serial.println(F("CAN Bus Tx test"));

  CAN_FD_Ch1.Init(CAN_500K_2M);       // 500k arbitration rate and 2Mbps data rate
  // APP_CANFDSPI_Init(CAN_500K_4M);       // 500k arbitration rate and 2Mbps data rate
  // APP_CANFDSPI_Init(CAN_500K_8M);       // 500k arbitration rate and 2Mbps data rate


  txObj.bF.ctrl.IDE = 0;      // Extended CAN ID false
  txObj.bF.id.SID = 0x7df;    // CAN ID
  txObj.bF.ctrl.BRS = 1;      // Switch Bitrate true (switch to 2Mbps)
  txObj.bF.ctrl.FDF = 1;      // CAN FD true
  txObj.bF.ctrl.DLC = 15;      // Data length, 64 bytes
  txd[4] = can_msg_count >> 24;
  txd[5] = can_msg_count >> 16;
  txd[6] = can_msg_count >> 8;
  txd[7] = can_msg_count;
  delay(100);
}

void loop() {
  CAN_TX_FIFO_EVENT txFlags;
  CAN_RX_FIFO_EVENT rxFlags;
  CAN_ERROR_STATE errorFlags;
  uint8_t tec, rec;


  // Check if FIFO is not full
  txFlags = CAN_FD_Ch1.TransmitChannelEventGet();
  if (!(txFlags & CAN_TX_FIFO_NOT_FULL_EVENT) && attempts > 0) {
    attempts--;
  }
  else if (attempts == 0) {
    CAN_FD_Ch1.ErrorCountStateGet(&tec, &rec, &errorFlags);
    attempts = MAX_TXQUEUE_ATTEMPTS;
  }
  else {
    // Load message and transmit CAN FD frame
    uint8_t n = CAN_FD_Ch1.DLCtoDataLength(txObj.bF.ctrl.DLC);
    CAN_FD_Ch1.TransmitChannelLoad(&txObj, txd, n);

    can_msg_count++;
    
    txd[4] = can_msg_count >> 24;
    txd[5] = can_msg_count >> 16;
    txd[6] = can_msg_count >> 8;
    txd[7] = can_msg_count;
    
    attempts = MAX_TXQUEUE_ATTEMPTS;
  }

  delay(100);

  if (CAN_FD_Ch1.available())
  {
    //read out the data
    uint8_t dlc, i, rxstep;

    rxFlags = CAN_FD_Ch1.ReceiveChannelEventGet();
    if (rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) {
      // Read the message
      CAN_FD_Ch1.ReceiveMessageGet(&rxObj, rxd, MAX_DATA_BYTES);
      dlc = CAN_FD_Ch1.DLCtoDataLength(rxObj.bF.ctrl.DLC);


      Serial.print(' ');

      for (i = 0; i < 8; i++) {
        Serial.print(rxd[i], HEX);
      }

      Serial.println(' ');
    }
  }

}
