/*
init()
setChannel()    //RF_CH register
setPower()      //RF_PWR bits in RF_SETUP
setBitRate()    //RF_DR_HIGH, RF_DR_LOW
powerDown()     //PWR_UP bit in CONFIG
wake()
listen()
isChannelClear()
getRxData()
send()
bleBeacon()
*/

// Commands
#define READ_REGSITER             0x00   //000A AAAA
#define WRITE_REGISTER            0x20   //001A AAAA
#define READ_RX_PAYLOAD           0x61   //0110 0001
#define WRITE_TX_PAYLOAD          0xA0   //1010 0000
#define FLUSH_TX                  0xE1   //1110 0001
#define FLUSH_RX                  0xE2   //1110 0010
#define REUSE_TX_PAYLOAD          0xE3   //1110 0011
#define ACTIVATE                  0x50   //0101 0000
#define READ_RX_PAYLOAD_WID       0x60   //0110 0000
#define WRITE_ACK_PAYLOAD         0xA8   //1010 1PPP
#define WRITE_TX_PAYLOAD_NO_ACK   0xB0   //1011 0000
#define NOP                       0xFF   //1111 1111


#define ADDR_MASK                 0x1F
     
#define PIPE_0                    0x00
#define PIPE_1                    0x01
#define PIPE_2                    0x02
#define PIPE_3                    0x03
#define PIPE_4                    0x04
#define PIPE_5                    0x05

// Registers
#define CONFIG_REG 0x00
#define   DISABLE_RX_DR           BIT6  // 0 = interrupt enabled
#define   DISABLE_TX_DS           BIT5  // 0 = interrupt enabled
#define   DISABLE_MAX_RT          BIT4  // 0 = interrupt enabled
#define   EN_CRC                  BIT3  // 1 = CRC Enabled
#define   CRC_2BYTES              BIT2  // 0 = 1 byte, 1 = 2bytes
#define   PWR_UP                  BIT1  // 1 = Power On
#define   PRIM_RX                 BIT0  // 1 = RX Mode, 0 = TX mode

#define EN_AA_REG 0x01  //Enable Auto ACK
#define   ENAA_P5                 BIT5
#define   ENAA_P4                 BIT4
#define   ENAA_P3                 BIT3
#define   ENAA_P2                 BIT2
#define   ENAA_P1                 BIT1
#define   ENAA_P0                 BIT0

#define EN_RXADDR_REG 0x02  //Enable RX Addresses
#define   ERX_P5                  BIT5
#define   ERX_P4                  BIT4
#define   ERX_P3                  BIT3
#define   ERX_P2                  BIT2
#define   ERX_P1                  BIT1
#define   ERX_P0                  BIT0

#define SETUP_ADDRESS_WIDTH_REG 0x03 //Setup address width
#define   W_3BYTES                0x01
#define   W_4BYTES                0x02
#define   W_5BYTES                0x03

#define SETUP_AUTO_RETRANSMISSION_REG 0x04
#define   DELAY_250US             0x00
#define   DELAY_500US             0x10
#define   DELAY_750US             0x20
#define   DELAY_1000US            0x30
#define   DELAY_1250US            0x40
#define   DELAY_1500US            0x50
#define   DELAY_1750US            0x60
#define   DELAY_2000US            0x70
#define   DELAY_2250US            0x80
#define   DELAY_2500US            0x90
#define   DELAY_2750US            0xA0
#define   DELAY_3000US            0xB0
#define   DELAY_3250US            0xC0
#define   DELAY_3500US            0xD0
#define   DELAY_3750US            0xE0
#define   DELAY_4000US            0xF0
     
#define   N_RETRANSMIT_0          0x00
#define   N_RETRANSMIT_1          0x01
#define   N_RETRANSMIT_2          0x02
#define   N_RETRANSMIT_3          0x03
#define   N_RETRANSMIT_4          0x04
#define   N_RETRANSMIT_5          0x05
#define   N_RETRANSMIT_6          0x06
#define   N_RETRANSMIT_7          0x07
#define   N_RETRANSMIT_8          0x08
#define   N_RETRANSMIT_9          0x09
#define   N_RETRANSMIT_10         0x0A
#define   N_RETRANSMIT_11         0x0B
#define   N_RETRANSMIT_12         0x0C
#define   N_RETRANSMIT_13         0x0D
#define   N_RETRANSMIT_14         0x0E
#define   N_RETRANSMIT_15         0x0F

#define RF_CHANNEL_REG 0x05
//2400 + regVal = channel in MHz

#define RF_SETUP_REG 0x06
#define   FORCE_PLL_LOCK          BIT4
  
#define   DATA_RATE_1MBPS         0x00
#define   DATA_RATE_2MBPS         0x04
#define   DATA_RATE_250KBPS       0x10
#define   DATA_RATE_2MBPS         0x14
  
#define   RF_PWR_0                0x00
#define   RF_PWR_1                0x02
#define   RF_PWR_2                0x04
#define   RF_PWR_3                0x06

#define   LNA_LOW_GAIN            0x00
#define   LNA_HIGH_GAIN           0x01

#define STATUS_REG 0x07
#define   RBANK                   BIT7
#define   RX_DATA_READY           BIT6
#define   TX_DATA_SENT            BIT5
#define   MAX_RETRANS             BIT4
#define   RX_FIFO_EMPTY           0x0E
#define   TX_FIFO_FULL            BIT0

#define OBSERVE_TX_REG            0x08
#define   PACKET_LOSS_CNT         0xF0
#define   RETRANS_CNT             0x0F

#define CARRIER_DECTECT_REG       0x09

#define RX_ADDR_P0                0x0A
#define RX_ADDR_P1                0x0B
#define RX_ADDR_P2                0x0C
#define RX_ADDR_P3                0x0D
#define RX_ADDR_P4                0x0E
#define RX_ADDR_P5                0x0F

#define TX_ADDR                   0x10

#define RX_DATA_IN_P0             0x11
#define RX_DATA_IN_P1             0x12
#define RX_DATA_IN_P2             0x13
#define RX_DATA_IN_P3             0x14
#define RX_DATA_IN_P4             0x15
#define RX_DATA_IN_P5             0x16
     
#define FIFO_STATUS               0x17
#define   TX_REUSE                BIT6
#define   TX_FULL                 BIT5
#define   TX_EMPTY                BIT4
#define   RX_FULL                 BIT1
#define   RX_EMPTY                BIT0
     
#define DYNAMIC_PAYLOAD_REG       0x1C
#define   DYNP_EN_P5              BIT5
#define   DYNP_EN_P4              BIT4
#define   DYNP_EN_P3              BIT3
#define   DYNP_EN_P2              BIT2
#define   DYNP_EN_P1              BIT1
#define   DYNP_EN_P0              BIT0
     
#define FEATURE_REG  0x1D     
#define   EN_DYNAMIC_PAYLOAD      BIT2
#define   EN_ACK_PAYLOAD          BIT1
#define   EN_DYNAMIC_ACK          BIT0
