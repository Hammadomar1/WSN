// SINK NODE CODE //

/*
 * includes.h
 *
 */

#ifndef INCLUDES_H_
#define INCLUDES_H_


// Standard includes
#include <time.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>


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


//Common interface includes
#include "common.h"
#ifndef NOTERM
#include "uart_if.h"
#endif
#include "pinmux.h"


#define APPLICATION_NAME        " Hello   "
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



#endif /* INCLUDES_H_ */

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif


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
                       0xc0, 0xa8, 0x01, 0x64,              /* src ip */
                       0xc0, 0xa8, 0x01, 0x02,              /* dest ip  */
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
char HI_PING[] = {
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
                  0xc0, 0xa8, 0x00, 0x00,              /* src ip */
                  0xc0, 0xa8, 0x00, 0x01,              /* dest ip  */
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
char CTS_PING[] = {
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
                   0xc0, 0xa8, 0x00, 0x00,              /* src ip */
                   0xc0, 0xa8, 0x00, 0x01,              /* dest ip  */
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

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************



//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static void DisplayBanner(char * AppName);
static void BoardInit(void);



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

   // UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    /*
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);
*/
    // Set connection policy to Auto + SmartConfig
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);



    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore
    // other return-codes
    //
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
    UART_PRINT("\t\t\t  %s Application       \n\r", AppName);
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



typedef struct

{

    _u8 rate; /* Received Rate */

    _u8 channel; /* The received channel*/

    _i8 rssi; /* The computed RSSI value in
    db of current frame */

    _u8 padding; /* pad to align to 32 bits */

    _u32 timestamp; /* Timestamp in microseconds*/

}TransceiverRxOverHead_t;


char* array_to_string(unsigned char* arr, size_t arr_len) {
    char* str = malloc(arr_len + 1);
    if (!str) return NULL;
    str[0] = '\0'; // start with an empty string
    int i;
    for ( i = 0; i < arr_len; i++) {
        sprintf(str + strlen(str), "%c", arr[i]);
    }
    return str;
}


#define STR_TIMEOUT_SEC 20

static int Tx_continuous_HI(int iChannel,SlRateIndex_e rate,int iNumberOfPackets,
                                    int iTxPowerLevel,long dIntervalMiliSec)
// sending HI to SENSOR //

{
    int iSoc;
    long lRetVal = -1;
    int i = 0;

    iSoc = sl_Socket(SL_AF_RF,SL_SOCK_RAW,iChannel);
    ASSERT_ON_ERROR(iSoc);

    UART_PRINT("SENDING HI Back eh yaba...\r\n");
    for(i = 0; i < 7; i++)
    {
        lRetVal = sl_Send(iSoc,HI_PING,sizeof(HI_PING),\
                   SL_RAW_RF_TX_PARAMS(iChannel,  rate, iTxPowerLevel, PREAMBLE));
        UART_PRINT("Sent message: %02x, %02x, %02x, %02x, %02x  \n\r",HI_PING[54],HI_PING[55],HI_PING[56],HI_PING[57],HI_PING[58],HI_PING[59]);
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

static int Tx_continuous_CTS(int iChannel,SlRateIndex_e rate,int iNumberOfPackets,
                                    int iTxPowerLevel,long dIntervalMiliSec)
// sending CTS to SENSOR //
{
    int iSoc;
    long lRetVal = -1;
    int i = 0;

    iSoc = sl_Socket(SL_AF_RF,SL_SOCK_RAW,iChannel);
    ASSERT_ON_ERROR(iSoc);

    UART_PRINT("SENDING CTS...\r\n");
    for(i = 0; i < 7; i++)
    {
        lRetVal = sl_Send(iSoc,CTS_PING,sizeof(CTS_PING),\
                   SL_RAW_RF_TX_PARAMS(iChannel,  rate, iTxPowerLevel, PREAMBLE));
        UART_PRINT("Sent message: %02x, %02x, %02x, %02x, %02x  \n\r",CTS_PING[54],CTS_PING[55],CTS_PING[56],CTS_PING[57],CTS_PING[58],CTS_PING[59]);
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

//static int Tx_continuous_DATA(int iChannel,SlRateIndex_e rate,int iNumberOfPackets,
//                                    int iTxPowerLevel,long dIntervalMiliSec)
//// sending DATA to SINK //
//{
//    int iSoc;
//    long lRetVal = -1;
//    int i = 0;
//
//    iSoc = sl_Socket(SL_AF_RF,SL_SOCK_RAW,iChannel);
//    ASSERT_ON_ERROR(iSoc);
//
//    UART_PRINT("SENDING DATA...\r\n");
//    for(i = 0; i < 5; i++)
//    {
//        lRetVal = sl_Send(iSoc,RawData_Ping,sizeof(RawData_Ping),\
//                   SL_RAW_RF_TX_PARAMS(iChannel,  rate, iTxPowerLevel, PREAMBLE));
//        UART_PRINT("Sent message: %02x, %02x, %02x, %02x, %02x  \n\r",RawData_Ping[54],RawData_Ping[55],RawData_Ping[56],RawData_Ping[57],RawData_Ping[58],RawData_Ping[59]);
//        if(lRetVal < 0)
//        {
//            sl_Close(iSoc);
//            ASSERT_ON_ERROR(lRetVal);
//        }
//        //Sleep(dIntervalMiliSec);
//        MAP_UtilsDelay(dIntervalMiliSec);
//    }
//
//    lRetVal = sl_Close(iSoc);
//    ASSERT_ON_ERROR(lRetVal);
//
//    UART_PRINT("Transmission complete.\r\n");
//    return SUCCESS;
//}

/********************************************************/
/*                    RX                               */

static int TransceiverModeRx_HI (int channel_number) {
    // Listen to Hi MESSAGE FROM SENSOR //

    SlTransceiverRxOverHead_t *frameRadioHeader = NULL;

    unsigned char buffer[1500] = {'\0'};

    int socket_hanlde = -1;

    int recievedBytes = 0;

    socket_hanlde= sl_Socket(SL_AF_RF, SL_SOCK_RAW, channel_number);

    time_t current_time;
    struct tm *local_time;
    int Message;
    time_t last_str_time = time(NULL);  // initialize the last "Hello 1" time to now
    int i = 0;
    for(i = 0; i < 7; i++)
//    while(1)
    {

            memset(&buffer[0], 0, sizeof(buffer));

            recievedBytes = sl_Recv(socket_hanlde, buffer, 4000, 0);
            frameRadioHeader = (SlTransceiverRxOverHead_t *)buffer;
          //  char* str = array_to_string(SRCMAC, 6);

//                if(buffer[62] == 0x48 && buffer[63] == 0x65 && buffer[64] == 0x6C)
//                {
//            UART_PRINT("Received message: %02x, %02x, %02x, %02x, %02x  \n\r", buffer[62],buffer[63],buffer[64],buffer[65],buffer[66],buffer[67]);
//}
            // SINK MAC: 0x00, 0x23, 0x75, 0x55, 0x55, 0x00
            if(!(buffer[58] == 0xC0 && buffer[59] == 0xA8 && buffer[60] == 0xAA && buffer[61] == 0xAA))
            {
                UART_PRINT("IP: %d.%d.%d.%d", buffer[58], buffer[59], buffer[60], buffer[61]);
                return FAILURE;
            }

    }
    UART_PRINT("RECEIVED HELLO FROM SENSOR %02x \n\r", buffer[57]);
    HI_PING[53] = buffer[57];


    sl_Close(socket_hanlde);
    return SUCCESS;
}

static int TransceiverModeRx_RTS (int channel_number) {
// Listen TO RTS MESSAGE FROM SENSOR //
    SlTransceiverRxOverHead_t *frameRadioHeader = NULL;

    unsigned char buffer[1500] = {'\0'};

    int socket_hanlde = -1;

    int recievedBytes = 0;

    socket_hanlde= sl_Socket(SL_AF_RF, SL_SOCK_RAW, channel_number);

    time_t current_time;
    struct tm *local_time;
    int Message;
    time_t last_str_time = time(NULL);  // initialize the last "Hello 1" time to now

    int i = 0;
    for(i = 0; i < 7; i++)
    {

            memset(&buffer[0], 0, sizeof(buffer));

            recievedBytes = sl_Recv(socket_hanlde, buffer, 4000, 0);
            frameRadioHeader = (SlTransceiverRxOverHead_t *)buffer;
          //  char* str = array_to_string(SRCMAC, 6);

//                if(buffer[24] == 0x00 && buffer[25] == 0x23 && buffer[26] == 0x75 && buffer[27] == 0x55 && buffer[28] == 0x33 /* && buffer[29] == 0x00 */ ){
//                    UART_PRINT("Received from Sensor %02x \r\n", buffer[29]);
//                }


            if(!(buffer[58] == 0xC0 && buffer[59] == 0xA8 && buffer[60] == 0xAA && buffer[61] == 0xAA))
            {
                UART_PRINT("IP: %d.%d.%d.%d \r\n", buffer[58], buffer[59], buffer[60], buffer[61]);
                return FAILURE;
            }

    }
    UART_PRINT("RECEIVED HELLO FROM SENSOR %02x \n\r", buffer[57]);
    HI_PING[53] = buffer[57];


    sl_Close(socket_hanlde);
    return SUCCESS;
}
static int TransceiverModeRx_DATA (int channel_number) {
// LISTEN to DATA MESSAGE FROM SENSOR //
    SlTransceiverRxOverHead_t *frameRadioHeader = NULL;

    unsigned char buffer[1500] = {'\0'};

    int socket_hanlde = -1;

    int recievedBytes = 0;

    socket_hanlde= sl_Socket(SL_AF_RF, SL_SOCK_RAW, channel_number);

    time_t current_time;
    struct tm *local_time;
    int Message;
    time_t last_str_time = time(NULL);  // initialize the last "Hello 1" time to now
    int i = 0;

    for(i = 0; i < 7; i++)
    {

            memset(&buffer[0], 0, sizeof(buffer));

            recievedBytes = sl_Recv(socket_hanlde, buffer, 4000, 0);
            frameRadioHeader = (SlTransceiverRxOverHead_t *)buffer;
          //  char* str = array_to_string(SRCMAC, 6);

//                if(buffer[62] == 0x08 && buffer[63] == 0x00 && buffer[64] == 0xA5)
//                {
//            UART_PRINT("Received message: %02x, %02x, %02x, %02x, %02x  \n\r",buffer[62],buffer[63],buffer[64],buffer[65],buffer[66],buffer[67]);
//
//                }
            // SINK IP: 192.168.255.255
            // SENSOR IP: 192.168.1.X

            //SRC IP: 54, 55, 56, 57
            //DEST IP: 58, 59, 60, 61

            if(!(buffer[58] == 0xC0 && buffer[59] == 0xA8 && buffer[60] == 0xAA && buffer[61] == 0xAA))
            {
                UART_PRINT("IP: %d.%d.%d.%d\r\n", buffer[58], buffer[59], buffer[60], buffer[61]);
                return FAILURE;
            }

    }
    UART_PRINT("RECEIVED HELLO FROM SENSOR %d \n\r", buffer[57]);
    HI_PING[53] = buffer[57];


    sl_Close(socket_hanlde);
    return SUCCESS;
}
int main()
{


    long lRetVal = -1;
    unsigned char policyVal;
    BoardInit();
    PinMuxConfig();
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

   // UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal)
    {
        UART_PRINT("Failed to start the device \n\r");
        LOOP_FOREVER();
    }

   // UART_PRINT("Device started as STATION \n\r");
    lRetVal = sl_WlanPolicySet(  SL_POLICY_CONNECTION,
                  SL_CONNECTION_POLICY(0,0,0,0,0),
                  &policyVal,
                  1 );
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to set policy \n\r");
        LOOP_FOREVER();
    }
    UART_PRINT("SRC MAC | Temperature | Time \n\r");

    int status = -1;

    // MAC Address template for sink is: 0x00, 0x23, 0x75, 0x55, 0x55, 0xXX
    // MAC Address template for sensor is: 0x00, 0x23, 0x75, 0x55, 0x33, 0xXX


while(1)
{
    UART_PRINT("MAIN STARTED \n\r");
//            char isJoined = 0, isRTS = 0;

            //*** listen for new nodes ***//

            do{
            status = TransceiverModeRx_HI(3); // listening to HELLO FROM SENSOR //
            } while (status != 0);
            MAP_UtilsDelay(4000000); // delay //
            status = -1;
            do{
            status = Tx_continuous_HI(3,30,100,50,4000000); // sending HI BACK to SENSOR //
            } while(status != 0);
            status = -1;

            //*** END ***//

//            if(isRTS == 1){
                do {
                    status = TransceiverModeRx_RTS(3);// listening to RTS FROM SENSOR //
                } while (status != 0);
                MAP_UtilsDelay(4000000); // delay //
                status = -1;
                do {
                    status = Tx_continuous_CTS(3,30,100,50,4000000); // sending CTS to SENSOR //
                } while (status != 0);
                status = -1;
                do {
                    status = TransceiverModeRx_DATA(3); // Receiving DATA from SENSOR //
                } while(status != 0);
           }

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    UART_PRINT("Sequence Complete! \n\r");
    LOOP_FOREVER();
}