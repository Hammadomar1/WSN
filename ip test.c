  // SINK IP: 192.168.255.255
            // SENSOR IP: 192.168.1.X

            //SRC IP: 54, 55, 56, 57
            //DEST IP: 58, 59, 60, 61

            if(!(buffer[58] == 0xC0 && buffer[59] == 0xA8 && buffer[60] == 0x00 && buffer[61] == 0x00))
            {
                UART_PRINT("IP: %d.%d.%d.%d", buffer[58], buffer[59], buffer[60], buffer[61]);
                return FAILURE;
            }

    }
    UART_PRINT("RECEIVED HELLO FROM SENSOR %d \n\r", buffer[57]);
    HI_PING[53] = buffer[57];
