#include <wixel.h>
#include <usb.h>
#include <usb_com.h>
#include <radio_com.h>
#include <radio_link.h>
#include <stdio.h>
#include <gpio.h>
#include <spi1_master.h>
#include "ADXL362.h"

// http://www.ntu.edu.sg/home/ehchua/programming/cpp/cp4_PointerReference.html
//trying...
//unsigned long *deviceIdLocation = 0x3E0; //http://www.pololu.com/docs/0J46/11

#define spiXMasterTransfer			spi1MasterTransfer
#define spiXMasterBusy				spi1MasterBusy
#define spiXMasterInit              spi1MasterInit
#define spiXMasterSetFrequency      spi1MasterSetFrequency
#define spiXMasterSetClockPolarity  spi1MasterSetClockPolarity
#define spiXMasterSetClockPhase     spi1MasterSetClockPhase
#define spiXMasterSetBitOrder       spi1MasterSetBitOrder
#define spiXMasterSendByte			spi1MasterSendByte
#define spiXMasterReceiveByte       spi1MasterReceiveByte
#define CSlow						setDigitalOutput(13,0)	// The Slave select
#define CShigh						setDigitalOutput(13,1)

/** Global Constants & Variables **********************************************/

//SPI transfer byte array
uint8 XDATA spi_txdata[14] = {0};
uint8 XDATA txdata[14] = {0x40,0x22,0x20,0x00,0x00,0x08,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
uint8 XDATA spi_txdata_nulls[4] = {0x00, 0x00, 0x00, 0x00};


//SPI receive byte array
uint8 XDATA spi_rxdata[14] = {0};
uint8 XDATA spi_rxdata_x[2] = {0};
uint8 XDATA spi_rxdata_y[2] = {0};
uint8 XDATA spi_rxdata_z[2] = {0};


/** Parameters ****************************************************************/

#define BRIDGE_MODE_RADIO_SPI 0
#define BRIDGE_MODE_USB_SPI   1

//*
int32 CODE param_bridge_mode = BRIDGE_MODE_RADIO_SPI;
/*/
int32 CODE param_bridge_mode = BRIDGE_MODE_USB_SPI;
//*/

static uint8 response = 0;
static BIT returnResponse = 0;

enum spiState {IDLE, GET_ADDR, GET_LEN, GET_DATA};
enum spiState state = IDLE;

static BIT started = 0;
static BIT dataDirIsRead = 0;
static uint8 dataLength = 0;

// function pointers to selected serial interface
uint8 (*rxAvailableFunction)(void)   = NULL;
uint8 (*rxReceiveByteFunction)(void) = NULL;
uint8 (*txAvailableFunction)(void)   = NULL;
void  (*txSendByteFunction)(uint8)   = NULL;

/** Functions *****************************************************************/

void updateLeds(void)
{
	usbShowStatusWithGreenLed();
	//LED_YELLOW(0);
	//LED_RED(0);
}


// http://svn.modlabupenn.org/mod-ros-pkg/wixel/sdk/apps/Can_timing/Can_timing.c
void spi_Init(){

	// http://pololu.github.io/wixel-sdk/spi0__master_8h.html#a4853b06748e26925b1ad9081dee071f3
	spiXMasterInit();

	// SPI Clock
	// ADXL362 recommended 1-5MHz
	// Wixel supports 23Hz-3MHz
	// thus use range 1-3MHz
	spiXMasterSetFrequency(2000000);

	spiXMasterSetClockPolarity(SPI_POLARITY_IDLE_HIGH); // idle when high
	spiXMasterSetClockPhase(SPI_PHASE_EDGE_TRAILING);  // data sampled when clock goes from active to idle

	// p19 for FIFO
	// "Data is presented least significant byte first, followed by the most significant byte."
	// always the case?
	spiXMasterSetBitOrder(SPI_BIT_ORDER_MSB_FIRST);
	//spiXMasterSetBitOrder(SPI_BIT_ORDER_LSB_FIRST);
}

void spiWrite(uint8 registerAddress, uint8 value){
	spi_txdata[0] = ADXL362_COMMAND_WRITE_REGISTER;
	spi_txdata[1] = registerAddress;
	spi_txdata[2] = value;

	CSlow;
	spiXMasterTransfer(spi_txdata,spi_rxdata,3);
	while(spiXMasterBusy()){ }
	CShigh;
}

uint8 spiReadRegister(uint8 registerAddress){
	spi_txdata[0] = ADXL362_COMMAND_READ_REGISTER;
	spi_txdata[1] = registerAddress;
	spi_txdata[2] = 0x00;

	CSlow;
	spiXMasterTransfer(spi_txdata,spi_rxdata,3);
	while(spiXMasterBusy()){ }
	CShigh;

	return spi_rxdata[2];
}

void spiReadMeasurement(){
	CSlow;

	spiXMasterSendByte(ADXL362_COMMAND_READ_FIFO);

	/*
	spiXMasterTransfer(spi_txdata_nulls,spi_rxdata_x,2);
	while(spiXMasterBusy()){ }

	spiXMasterTransfer(spi_txdata_nulls,spi_rxdata_y,2);
	while(spiXMasterBusy()){ }

	spiXMasterTransfer(spi_txdata_nulls,spi_rxdata_z,2);
	while(spiXMasterBusy()){ }
	//*/

	/*
	// works for y & z
	spi_rxdata_x[0] = spiReadRegister(ADXL362_REGISTER_XDATA);
	spi_rxdata_y[0] = spiReadRegister(ADXL362_REGISTER_YDATA);
	spi_rxdata_z[0] = spiReadRegister(ADXL362_REGISTER_ZDATA);
	//*/

	spi_rxdata_x[0] = spiReadRegister(ADXL362_REGISTER_XDATA_L);
	spi_rxdata_x[1] = spiReadRegister(ADXL362_REGISTER_XDATA_H);// & ADXL362_REGISTER_MASK_XDATA_H;
	spi_rxdata_y[0] = spiReadRegister(ADXL362_REGISTER_YDATA_L);
	spi_rxdata_y[1] = spiReadRegister(ADXL362_REGISTER_YDATA_H);// & ADXL362_REGISTER_MASK_YDATA_H;
	spi_rxdata_z[0] = spiReadRegister(ADXL362_REGISTER_XDATA_L);
	spi_rxdata_z[1] = spiReadRegister(ADXL362_REGISTER_ZDATA_H);// & ADXL362_REGISTER_MASK_ZDATA_H;



	/*
	x = spiReadRegister(ADXL362_REGISTER_XDATA_L);
	x += (spiReadRegister(ADXL362_REGISTER_XDATA_H) & ADXL362_REGISTER_MASK_XDATA_H) << 8;
	y] = spiReadRegister(ADXL362_REGISTER_YDATA_L);
	y += (spiReadRegister(ADXL362_REGISTER_YDATA_H) & ADXL362_REGISTER_MASK_YDATA_H) << 8;
	z = spiReadRegister(ADXL362_REGISTER_XDATA_L);
	z += (spiReadRegister(ADXL362_REGISTER_ZDATA_H) & ADXL362_REGISTER_MASK_ZDATA_H) << 8;
	//*/

	CShigh;
}

void initADXL362(){
	// soft reset
	spiWrite(ADXL362_REGISTER_SOFT_RESET, ADXL362_REGISTER_SOFT_RESET_COMMAND_RESET);
	// power control the ADXL to ultra-low noise, measurement mode
	spiWrite(ADXL362_REGISTER_POWER_CTL, 0x22); // 0x22 == 00100010_b, page 34
	// FIFO control, no temp, triggered
	// Triggered mode (triggered by interrupt, not read)
	//spiWrite(ADXL362_REGISTER_POWER_CTL, 0x03); // 0x03 == 00000011_b, page 30
	// Oldest saved mode
	//spiWrite(ADXL362_REGISTER_POWER_CTL, 0x01); // 0x01 == 00000001_b, page 30
	// Stream Mode
	//spiWrite(ADXL362_REGISTER_POWER_CTL, 0x02);
}

// http://stackoverflow.com/a/3212885
char hexDigit(unsigned n)
{
    if (n < 10) {
        return n + '0';
    } else {
        return (n - 10) + 'A';
    }
}

void spiService(void)
{
	char hex[2] = {' ',' '};
    // Only try to process I2C if there isn't a response still waiting to be returned on serial.
    if (!returnResponse)
    {
    	/*
    	//response = 0xAD;


		// We are doing an I2C read, so handle that.

    	//response = spiReadRegister(ADXL362_REGISTER_DEVID_AD);
    	//response = spiReadRegister(ADXL362_REGISTER_DEVID_AD) & 0x0F;
    	//response = 0xAD & 0x0F;

    	//response = spiReadRegister(ADXL362_REGISTER_PARTID);
    	//response = spiReadRegister(ADXL362_REGISTER_PARTID) & 0x0F;

    	response = spiReadRegister(ADXL362_REGISTER_ZDATA);
    	//response = spiReadRegister(ADXL362_REGISTER_ZDATA) & 0x0F;

    	//spiReadMeasurement();
    	//response = spi_rxdata_x[0];

    	returnResponse = 1;

    	/*/
    	spiReadMeasurement();

    	while(!txAvailableFunction()){}

    	txSendByteFunction('^');

    	// USB format, so omit index 0
    	txSendByteFunction(serialNumberStringDescriptor[1]);
    	txSendByteFunction(serialNumberStringDescriptor[2]);
    	txSendByteFunction(serialNumberStringDescriptor[3]);
    	txSendByteFunction(serialNumberStringDescriptor[4]);
    	txSendByteFunction(serialNumberStringDescriptor[5]);
    	txSendByteFunction(serialNumberStringDescriptor[6]);
    	txSendByteFunction(serialNumberStringDescriptor[7]);
    	txSendByteFunction(serialNumberStringDescriptor[8]);
    	txSendByteFunction(serialNumberStringDescriptor[9]);
    	txSendByteFunction(serialNumberStringDescriptor[10]);
    	txSendByteFunction(serialNumberStringDescriptor[11]);

		txSendByteFunction('@');
    	hex[0] = hexDigit(spi_rxdata_x[1] & 0x0F);
		hex[1] = hexDigit((spi_rxdata_x[1] & 0xF0) >> 4);
		txSendByteFunction(hex[1]);
		txSendByteFunction(hex[0]);
		txSendByteFunction(':');
		hex[0] = hexDigit(spi_rxdata_x[0] & 0x0F);
		hex[1] = hexDigit((spi_rxdata_x[0] & 0xF0) >> 4);
		txSendByteFunction(hex[1]);
		txSendByteFunction(hex[0]);
		txSendByteFunction(',');
		hex[0] = hexDigit(spi_rxdata_y[1] & 0x0F);
		hex[1] = hexDigit((spi_rxdata_y[1] & 0xF0) >> 4);
		txSendByteFunction(hex[1]);
		txSendByteFunction(hex[0]);
		txSendByteFunction(':');
		hex[0] = hexDigit(spi_rxdata_y[0] & 0x0F);
		hex[1] = hexDigit((spi_rxdata_y[0] & 0xF0) >> 4);
		txSendByteFunction(hex[1]);
		txSendByteFunction(hex[0]);
		txSendByteFunction(',');
		hex[0] = hexDigit(spi_rxdata_z[1] & 0x0F);
		hex[1] = hexDigit((spi_rxdata_z[1] & 0xF0) >> 4);
		txSendByteFunction(hex[1]);
		txSendByteFunction(hex[0]);
		txSendByteFunction(':');
		hex[0] = hexDigit(spi_rxdata_z[0] & 0x0F);
		hex[1] = hexDigit((spi_rxdata_z[0] & 0xF0) >> 4);
		txSendByteFunction(hex[1]);
		txSendByteFunction(hex[0]);
		txSendByteFunction('&');
    	txSendByteFunction(0x0D);
    	txSendByteFunction(0x0A);
    	//*/
    }

    if (returnResponse && txAvailableFunction())
    {
    	hex[0] = hexDigit(response & 0x0F);
    	hex[1] = hexDigit((response & 0xF0) >> 4);

        txSendByteFunction(hex[1]);
        txSendByteFunction(hex[0]);

        txSendByteFunction(0x0D);
        txSendByteFunction(0x0A);
        returnResponse = 0;
    }
}

void main(void)
{
    systemInit();
    spi_Init();
    usbInit();

    initADXL362();

    switch (param_bridge_mode)
    {
    case BRIDGE_MODE_RADIO_SPI:
        radioComInit();
        rxAvailableFunction   = radioComRxAvailable;
        rxReceiveByteFunction = radioComRxReceiveByte;
        txAvailableFunction   = radioComTxAvailable;
        txSendByteFunction    = radioComTxSendByte;
        break;
    case BRIDGE_MODE_USB_SPI:
        rxAvailableFunction   = usbComRxAvailable;
        rxReceiveByteFunction = usbComRxReceiveByte;
        txAvailableFunction   = usbComTxAvailable;
        txSendByteFunction    = usbComTxSendByte;
        break;
    }

    while (1)
    {
        boardService();
        updateLeds();

        switch (param_bridge_mode)
        {
        case BRIDGE_MODE_RADIO_SPI:
            radioComTxService();
            break;
        }

        usbComService();

        spiService();
    }
}

// Local Variables: **
// mode: C **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: nil **
// end: **
