// TRANSMITTER CODE (Sensor Nodes) - Sensor 1 with RTS packet of sensor 1


// Standard includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "interrupt.h"
#include "utils.h"
#include "uart.h"
#include "hw_memmap.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "tmp006drv.h"
#include"i2c_if.h"
//Common interface includes
#include "common.h"
#ifndef NOTERM
#include "uart_if.h"
#endif
//#include "tmp006.h"
#include "pinmux.h"


#define APPLICATION_NAME        "TRANSCEIVER_MODE"
#define APPLICATION_VERSION     "1.4.0"

#define PREAMBLE            1        /* Preamble value 0- short, 1- long */
#define CPU_CYCLES_1MSEC (80*1000)

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    TX_CONTINUOUS_FAILED = -0x7D0,
    RX_STATISTICS_FAILED = TX_CONTINUOUS_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = RX_STATISTICS_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;


typedef struct
{
    int choice;
    int channel;
    int packets;
    SlRateIndex_e rate;
    int Txpower;
}UserIn;

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID

char RawData_Ping[] = {
       /*---- wlan header start -----*/
       0x88,                                /* version , type sub type */
       0x02,                                /* Frame control flag */
       0x2C, 0x00,
       0x00, 0x23, 0x75, 0x55,0x55, 0x55,   /* destination */
       0x00, 0x22, 0x75, 0x55,0x55, 0x55,   /* bssid */
       0x08, 0x00, 0x28, 0x19,0x02, 0x85,   /* source */
       0x80, 0x42, 0x00, 0x00,
       0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00, /* LLC */
       /*---- ip header start -----*/
       0x45, 0x00, 0x00, 0x54, 0x96, 0xA1, 0x00, 0x00, 0x40, 0x01,
       0x57, 0xFA,                          /* checksum */
       0xc0, 0xa8, 0x00, 0x01,              /* src ip */
       0xc0, 0xa8, 0x00, 0x00,              /* dest ip  */
       /* payload - ping/icmp */
       0x08, 0x00, 0xA5, 0x51,
       0x5E, 0x18, 0x00, 0x00, 0x41, 0x08, 0xBB, 0x8D, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00};
char SENDING_HI_Ping[] = {
                          /*---- wlan header start -----*/
                          0x88,                                /* version , type sub type */
                          0x02,                                /* Frame control flag */
                          0x2C, 0x00,
                          0x00, 0x23, 0x75, 0x55,0x55, 0x55,   /* destination */
                          0x00, 0x22, 0x75, 0x55,0x55, 0x55,   /* bssid */
                          0x08, 0x00, 0x28, 0x19,0x02, 0x85,   /* source */
                          0x80, 0x42, 0x00, 0x00,
                          0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00, /* LLC */
                          /*---- ip header start -----*/
                          0x45, 0x00, 0x00, 0x54, 0x96, 0xA1, 0x00, 0x00, 0x40, 0x01,
                          0x57, 0xFA,                          /* checksum */
                          0xc0, 0xa8, 0x00, 0x01,              /* src ip */
                          0xc0, 0xa8, 0x00, 0x00,              /* dest ip  */
                          /* payload - ping/icmp */
                          0x08, 0x00, 0xA5, 0x51,
                          0x5E, 0x18, 0x00, 0x00, 0x41, 0x08, 0xBB, 0x8D, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00};
char SENDING_RTS_Ping[] = {
                           /*---- wlan header start -----*/
                           0x88,                                /* version , type sub type */
                           0x02,                                /* Frame control flag */
                           0x2C, 0x00,
                           0x00, 0x23, 0x75, 0x55,0x55, 0x55,   /* destination */
                           0x00, 0x22, 0x75, 0x55,0x55, 0x55,   /* bssid */
                           0x08, 0x00, 0x28, 0x19,0x02, 0x85,   /* source */
                           0x80, 0x42, 0x00, 0x00,
                           0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00, /* LLC */
                           /*---- ip header start -----*/
                           0x45, 0x00, 0x00, 0x54, 0x96, 0xA1, 0x00, 0x00, 0x40, 0x01,
                           0x57, 0xFA,                          /* checksum */
                           0xc0, 0xa8, 0x00, 0x01,              /* src ip */
                           0xc0, 0xa8, 0xAA, 0xAA,              /* dest ip  */
                           /* payload - ping/icmp */
                           0x08, 0x00, 0xA5, 0x51,
                           0x5E, 0x18, 0x00, 0x00, 0x41, 0x08, 0xBB, 0x8D, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00};

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
//static UserIn UserInput();
static int Tx_continuous_HI(int iChannel,SlRateIndex_e rate,int iNumberOfPackets,
                                    int iTxPowerLevel,long dIntervalMiliSec);
static int Tx_continuous_RTS(int iChannel,SlRateIndex_e rate,int iNumberOfPackets,
                                    int iTxPowerLevel,long dIntervalMiliSec);
static int Tx_continuous_DATA(int iChannel,SlRateIndex_e rate,int iNumberOfPackets,
                                    int iTxPowerLevel,long dIntervalMiliSec);
static int TransceiverModeRx_HI (int channel_number);
static int TransceiverModeRx_CTS (int channel_number);




void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t' - Applications
            // can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                      "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION
            if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                           " BSSID: %x:%x:%x:%x:%x:%x on application's"
                           " request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else
            {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s,"
                           " BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************



void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                     "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
        }
        break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    // Unused in this application
}


void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}



void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    //
    // This application doesn't work w/ socket - Events are not expected
    //

}

static void InitializeAppVariables()
{
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
}

static long ConfigureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode
    if (ROLE_STA != lMode)
    {
        if (ROLE_AP == lMode)
        {
            // If the device is in AP mode, we need to wait for this event
            // before doing anything
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
            }
        }

        // Switch to STA role and restart
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again
        if (ROLE_STA != lRetVal)
        {
            // We don't want to proceed if the device is not coming up in STA-mode
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }

    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal)
    {
        // Wait
        while(IS_CONNECTED(g_ulStatus))
        {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();

    return lRetVal; // Success
}


static void DisplayBanner(char * AppName)
{
    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\t\t\t CC3200 %s Application       \n\r", AppName);
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\n\n\n\r");
}

static void BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}




static int Tx_continuous_HI(int iChannel,SlRateIndex_e rate,int iNumberOfPackets,
                                    int iTxPowerLevel,long dIntervalMiliSec)
// sending HI to SINK //

{
    int iSoc;
    long lRetVal = -1;
    int i = 0;

    iSoc = sl_Socket(SL_AF_RF,SL_SOCK_RAW,iChannel);
    ASSERT_ON_ERROR(iSoc);

    UART_PRINT("SENDING HI...\r\n");
    for(i = 0; i < 7; i++)
    {
        lRetVal = sl_Send(iSoc,SENDING_HI_Ping,sizeof(SENDING_HI_Ping),\
                   SL_RAW_RF_TX_PARAMS(iChannel,  rate, iTxPowerLevel, PREAMBLE));
        UART_PRINT("Sent message: %02x, %02x, %02x, %02x, %02x  \n\r",SENDING_HI_Ping[54],SENDING_HI_Ping[55],SENDING_HI_Ping[56],SENDING_HI_Ping[57],SENDING_HI_Ping[58],SENDING_HI_Ping[59]);
        if(lRetVal < 0)
        {
            sl_Close(iSoc);
            ASSERT_ON_ERROR(lRetVal);
        }
        //Sleep(dIntervalMiliSec);
        MAP_UtilsDelay(dIntervalMiliSec);
    }

    lRetVal = sl_Close(iSoc);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("Transmission complete.\r\n");
    return SUCCESS;
}

static int Tx_continuous_RTS(int iChannel,SlRateIndex_e rate,int iNumberOfPackets,
                                    int iTxPowerLevel,long dIntervalMiliSec)
// sending RTS to SINK //
{
    int iSoc;
    long lRetVal = -1;
    int i = 0;

    iSoc = sl_Socket(SL_AF_RF,SL_SOCK_RAW,iChannel);
    ASSERT_ON_ERROR(iSoc);

    UART_PRINT("SENDING RTS...\r\n");
    for(i = 0; i < 7; i++)
    {
        lRetVal = sl_Send(iSoc,SENDING_RTS_Ping,sizeof(SENDING_RTS_Ping),\
                   SL_RAW_RF_TX_PARAMS(iChannel,  rate, iTxPowerLevel, PREAMBLE));
        UART_PRINT("Sent message: %02x, %02x, %02x, %02x, %02x  \n\r",SENDING_RTS_Ping[54],SENDING_RTS_Ping[55],SENDING_RTS_Ping[56],SENDING_RTS_Ping[57],SENDING_RTS_Ping[58],SENDING_RTS_Ping[59]);
        if(lRetVal < 0)
        {
            sl_Close(iSoc);
            ASSERT_ON_ERROR(lRetVal);
        }
        //Sleep(dIntervalMiliSec);
        MAP_UtilsDelay(dIntervalMiliSec);
    }

    lRetVal = sl_Close(iSoc);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("Transmission complete.\r\n");
    return SUCCESS;
}

static int Tx_continuous_DATA(int iChannel,SlRateIndex_e rate,int iNumberOfPackets,
                                    int iTxPowerLevel,long dIntervalMiliSec)
// sending DATA to SINK //
{
    int iSoc;
    long lRetVal = -1;
    int i = 0;

    iSoc = sl_Socket(SL_AF_RF,SL_SOCK_RAW,iChannel);
    ASSERT_ON_ERROR(iSoc);

    UART_PRINT("SENDING DATA...\r\n");
    for(i = 0; i < 7; i++)
    {
        lRetVal = sl_Send(iSoc,RawData_Ping,sizeof(RawData_Ping),\
                   SL_RAW_RF_TX_PARAMS(iChannel,  rate, iTxPowerLevel, PREAMBLE));
        UART_PRINT("Sent message: %02x, %02x, %02x, %02x, %02x  \n\r",RawData_Ping[54],RawData_Ping[55],RawData_Ping[56],RawData_Ping[57],RawData_Ping[58],RawData_Ping[59]);
        if(lRetVal < 0)
        {
            sl_Close(iSoc);
            ASSERT_ON_ERROR(lRetVal);
        }
        //Sleep(dIntervalMiliSec);
        MAP_UtilsDelay(dIntervalMiliSec);
    }

    lRetVal = sl_Close(iSoc);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("Transmission Complete!.\r\n");
    return SUCCESS;
}

/********************************************************/
/*                    RX                               */

static int TransceiverModeRx_HI (int channel_number) { // (to listen to both HELLO BACK and CTS from sink node)
// HELLO BACK MESSAGE FROM SINK //
    SlTransceiverRxOverHead_t *frameRadioHeader = NULL;

    unsigned char buffer[1500] = {'\0'};

    int socket_hanlde = -1;


    int recievedBytes = 0;
    int i = 0;

    int iRetVal = -1;
    iRetVal= sl_Socket(SL_AF_RF, SL_SOCK_RAW, channel_number);
    ASSERT_ON_ERROR(iRetVal);

    time_t current_time;
    struct tm *local_time;
    int Message;
    time_t last_str_time = time(NULL);  // initialize the last "Hello 1" time to now
    for(i = 0; i < 7; i++)
    {

//int i = 0;
            memset(&buffer[0], 0, sizeof(buffer));

            recievedBytes = sl_Recv(socket_hanlde, buffer, 4000, 0);
            frameRadioHeader = (SlTransceiverRxOverHead_t *)buffer;
          //  char* str = array_to_string(SRCMAC, 6);

            // SINK IP: 192.168.0.0
            // SENSOR IP: 192.168.0.1

            //BUFFER INDEX
            //SRC IP: 54, 55, 56, 57
            //DEST IP: 58, 59, 60, 61

            if(!(buffer[58] == 0xC0 && buffer[59] == 0xA8 && buffer[60] == 0x00 && buffer[61] == 0x01))
            {
                UART_PRINT("IP: %d.%d.%d.%d \r\n", buffer[58], buffer[59], buffer[60], buffer[61]);
                return FAILURE;
            }

    }

    UART_PRINT("RECEIVED HELLO BACK FROM SINK  \n\r");

    iRetVal = sl_Close(socket_hanlde);
    ASSERT_ON_ERROR(iRetVal);

    return SUCCESS;
}

static int TransceiverModeRx_CTS (int channel_number) {
// CTS MESSAGE FROM SINK //
    SlTransceiverRxOverHead_t *frameRadioHeader = NULL;

    unsigned char buffer[1500] = {'\0'};

    int socket_hanlde = -1;

    int recievedBytes = 0;

    int i = 0;

    int iRetVal = -1;
    iRetVal= sl_Socket(SL_AF_RF, SL_SOCK_RAW, channel_number);
    ASSERT_ON_ERROR(iRetVal);

    time_t current_time;
    struct tm *local_time;
    int Message;
    time_t last_str_time = time(NULL);  // initialize the last "Hello 1" time to now
    for(i = 0; i < 7; i++)
    {

//int i = 0;
            memset(&buffer[0], 0, sizeof(buffer));

            recievedBytes = sl_Recv(socket_hanlde, buffer, 4000, 0);
            frameRadioHeader = (SlTransceiverRxOverHead_t *)buffer;
          //  char* str = array_to_string(SRCMAC, 6);

            // SINK IP: 192.168.0.0
            // SENSOR IP: 192.168.0.1

            //BUFFER INDEX
            //SRC IP: 54, 55, 56, 57
            //DEST IP: 58, 59, 60, 61

            if(!(buffer[58] == 0xC0 && buffer[59] == 0xA8 && buffer[60] == 0x00 && buffer[61] == 0x01))
            {
                UART_PRINT("IP: %d.%d.%d.%d\r\n", buffer[58], buffer[59], buffer[60], buffer[61]);
                return FAILURE;
            }
    UART_PRINT("RECEIVED CTS FROM THE SINK \n\r");

    sl_Close(socket_hanlde);
    ASSERT_ON_ERROR(iRetVal);
    return SUCCESS;

}
}

int main() {
   // UserIn User;
    int iFlag = 1;
    long lRetVal = -1;
    //char cChar;
    unsigned char policyVal;

    //
    // Initialize Board configuration
    //
    BoardInit();

    //Pin muxing
    //
    PinMuxConfig();

    // Configuring UART
    //
    InitTerm();
    DisplayBanner(APPLICATION_NAME);

    InitializeAppVariables();



    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
          UART_PRINT("Failed to configure the device in its default state \n\r");
          LOOP_FOREVER();
    }

    UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    //


    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal)
    {
        UART_PRINT("Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    UART_PRINT("Device started as STATION \n\r");

    //
    // reset all network policies
    //
    lRetVal = sl_WlanPolicySet(  SL_POLICY_CONNECTION,
                  SL_CONNECTION_POLICY(0,0,0,0,0),
                  &policyVal,
                  1 /*PolicyValLen*/);
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to set policy \n\r");
        LOOP_FOREVER();
    }
    int tx_status = -1;
    while (1)
    {

        lRetVal = sl_Start(0, 0, 0);

        UART_PRINT("Main started \n \r");


        do{
        tx_status = Tx_continuous_HI(3,30,100,50,4000000); // sending HI to SINK //
        } while(tx_status != 0);
        tx_status = -1;

        float temperature;
            int T = 0;
            I2C_IF_Open (I2C_MASTER_MODE_FST);
            TMP006DrvOpen();     //Init Temprature Sensor
            TMP006DrvGetTemp(&temperature);    // reads temperature from sensor
            T = (int)temperature;
              RawData_Ping[54] = T;

            UART_PRINT("Temperature is:= %d \n \r", T);
        do{
            tx_status = TransceiverModeRx_HI(3); // listening to HELLO BACK FROM SINK //

        MAP_UtilsDelay(4000000); // delay //
        } while(tx_status != 0);
        tx_status = -1;
        do{
            tx_status = Tx_continuous_RTS(3,30,100,50,4000000); // sending RTS to SINK //
        } while(tx_status != 0);
        tx_status = -1;
        do{

            tx_status = TransceiverModeRx_CTS(3);// listening to CTS FROM SINK //

        MAP_UtilsDelay(4000000); // delay //
        } while(tx_status != 0);
        tx_status = -1;
        do{
                    tx_status = Tx_continuous_DATA(3,30,100,50,4000000); // sending DATA to SINK //
                } while(tx_status != 0);

        lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    }
//}

   // UART_PRINT("\n\rEnter \"1\" to restart or \"0\" to quit: ");



    LOOP_FOREVER();

}
