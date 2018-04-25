#include <string.h>
#include <math.h>
#include "board.h"
#include "radio.h"
#include "gpio.h"

/*!
 * User application data buffer size
 */
//#define LORAWAN_APP_DATA_MAX_SIZE                           242

#define DATA_SIZE                                             16 // 16 bytes

/*!
 * User application data
 */
static uint8_t AppData[DATA_SIZE];

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

static void OnRadioTxDone( void )
{
    Radio.Sleep( );
}

static void OnRadioRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
}

static void OnRadioRxError( void )
{
    Radio.Sleep( );
}

static void OnRadioTxTimeout( void )
{
    Radio.Sleep( );
}

static void OnRadioRxTimeout( void )
{
    Radio.Sleep( );
}

/**
 * Main application entry point.
 */
int main( void )
{
    BoardInitMcu( );
    BoardInitPeriph( );
	
	  // Initialize Radio driver
    RadioEvents.TxDone = OnRadioTxDone;
    RadioEvents.RxDone = OnRadioRxDone;
    RadioEvents.RxError = OnRadioRxError;
    RadioEvents.TxTimeout = OnRadioTxTimeout;
    RadioEvents.RxTimeout = OnRadioRxTimeout;
    Radio.Init( &RadioEvents );

    // Random seed initialization
    srand1( Radio.Random( ) );

    bool PublicNetwork = true;
    Radio.SetPublicNetwork( PublicNetwork );
    Radio.Sleep( );

		float tx_dutycycle = 2; // s
		uint32_t node_id = 0x7777;
		int8_t power = 20; // [20, 16, 14, 12, 10, 7, 5, 2] dBm
		uint32_t freq = 487500000; // Hz
		uint32_t datarate = 8; // Spreading Factor
		uint32_t bandwidth = 0; // 0: 125 kHz, 1: 250 kHz, 2: 500 kHz
		uint8_t coderate = 1; // 1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8
		uint8_t data_size = 16; // App Data Size, 16 bytes
		uint32_t delimiter = 0x1234; // Delimiter in the payload

    // log_info("scanf NODE_ID, POWER, SF, BW, CR, TX_DUTYCYCLE, APPDATASIZE, FREQUENCY\n");
		
//		scanf("%d", &node_id);
//		scanf("%d", (uint32_t*)&power);
//		scanf("%d", &datarate);
//		scanf("%d", &bandwidth);
//		scanf("%d", (uint32_t*)&coderate);
//		scanf("%d", &tx_dutycycle);
//		scanf("%d", (uint32_t*)&data_size);
//		scanf("%d", &freq);

		memcpy( AppData, &delimiter, 4 ); // store delimiter in AppData
		memcpy( AppData+4, &node_id, 4 ); // store node_id in AppData
		memcpy( AppData+12, &delimiter, 4 ); // store delimiter in AppData
		
//		for( int i = 0; i < data_size; i++ )
//		{
//				printf("%x", AppData[i]);
//		}
//		printf("\n");
	
    while( 1 )
    {
				static uint32_t sentCnt = 0;
        
			  // see SX1276SetChannel
				Radio.SetChannel( freq );

				/*!
				 * \brief Sets the transmission parameters
				 *
				 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa] 
				 * \param [IN] power        Sets the output power [dBm] [0: 20, 16, 14, 12, 10, 7, 5, 2]
				 * \param [IN] fdev         Sets the frequency deviation (FSK only)
				 *                          FSK : [Hz]
				 *                          LoRa: 0
				 * \param [IN] bandwidth    Sets the bandwidth (LoRa only)
				 *                          FSK : 0
				 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
				 *                                 2: 500 kHz, 3: Reserved] 
				 * \param [IN] datarate     Sets the Datarate
				 *                          FSK : 600..300000 bits/s
				 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
				 *                                10: 1024, 11: 2048, 12: 4096  chips]
				 * \param [IN] coderate     Sets the coding rate (LoRa only)
				 *                          FSK : N/A ( set to 0 )
				 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8] 
				 * \param [IN] preambleLen  Sets the preamble length
				 *                          FSK : Number of bytes 
				 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
				 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
				 * \param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
				 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
				 *                          FSK : N/A ( set to 0 )
				 *                          LoRa: [0: OFF, 1: ON]
				 * \param [IN] HopPeriod    Number of symbols bewteen each hop
				 *                          FSK : N/A ( set to 0 )
				 *                          LoRa: Number of symbols
				 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
				 *                          FSK : N/A ( set to 0 )
				 *                          LoRa: [0: not inverted, 1: inverted]
				 * \param [IN] timeout      Transmission timeout [ms]
				 *
				void    ( *SetTxConfig )( RadioModems_t modem, int8_t power, uint32_t fdev, 
																	uint32_t bandwidth, uint32_t datarate,
																	uint8_t coderate, uint16_t preambleLen,
																	bool fixLen, bool crcOn, bool FreqHopOn,
																	uint8_t HopPeriod, bool iqInverted, uint32_t timeout );*/
        
				// see SX1276SetTxConfig
				Radio.SetTxConfig( MODEM_LORA, power, 0,
													 bandwidth, datarate,
													 coderate, 8,
													 false, true, 0,
													 0, false, 3000 );
				/*!
				 * \brief Sets the maximum payload length.
				 *
				 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
				 * \param [IN] max        Maximum payload length in bytes
				 *
				void SX1276SetMaxPayloadLength( RadioModems_t modem, uint8_t max );*/
				// Setup maximum payload lenght of the radio driver
				// Radio.SetMaxPayloadLength( MODEM_LORA, data_size );
				
				log_info("sentCnt:%d, power:%d, bw:%d, dr:%d, cr:%d, freq:%d\n",
					sentCnt++, power, bandwidth, datarate, coderate, freq);
        
				memcpy( AppData+8, &sentCnt, 4 ); // store count-number in AppData
				
				// Send now
				Radio.Send( AppData, data_size );
				
				// LED toggling
				GpioWrite( &Led2, GpioRead( &Led2 ) ^ 1 );

				Delay( tx_dutycycle ); // delay 10s
    }
}
