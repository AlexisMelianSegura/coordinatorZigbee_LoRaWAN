/**
 * \file Zigbee.cpp
 * \author Alexis Melian Segura
 * \date 20/08/19
 * \brief Program that defines the operation of a Zigbee module.
 */

/****************************************************************************
*                               Includes                                    *
****************************************************************************/

#include "Zigbee.h"

/****************************************************************************
*                             Objects                                       *
****************************************************************************/

Zigbee  zigbee_object = Zigbee();

/****************************************************************************
*                               Functions                                   *
****************************************************************************/

Zigbee::Zigbee() {
}


Zigbee::~Zigbee() {
}

void Zigbee::init_Zigbee(){
  // Zigbee module activation.
  xbee802.ON(SOCKET_ZIGBEE);

  // In the case that the module does not know the coordinator's MAC address, it will send a broadcast message for the coordinator to send its address.
  #if ZIGBEE_MODE == 1
    if(status_coordinatorAddress == false){
      coordinatorMACRequest();
    }   
  #endif
}

int8_t Zigbee::receiveZigbee(uint8_t* data){
  int8_t receive_status = xbee802.receivePacketTimeout(RECEPTION_TIME_XBEE);
  if(receive_status == 0){
    memset(data, 0x00, DATA_MAX);
    memcpy(data, xbee802._payload, DATA_MAX);
  }
  return receive_status;
}

uint8_t Zigbee::sendZigbee(struct ZigbeePacket* packet){  

  // Local variables
  uint8_t TX[100];
  uint8_t counter=0; 
  unsigned long previous=0;
  uint8_t protection=0;
  uint8_t type=0;
  uint8_t status=1;
  uint8_t old_netAddress[2];
  uint8_t net_Address_changed = 0;   
  int8_t status_value=2;

  // The Socket associated with the Zigbee module is activated.
  if(SOCKET_ZIGBEE == SOCKET0){
    Utils.setMuxSocket0();
  }else{
    Utils.setMuxSocket1();
  }

  // If the maximum payload has been exceeded, a truncation is performed.
  if(packet->data_size > DATA_MAX){
    packet->data_size=DATA_MAX;
  } 
    
  // The transmission frame buffer is started.
  memset(TX,0x00,sizeof(TX));
    
  // Zigbee frame is created.
  // The header is added.
  TX[0] = 0x7E;
  TX[1] = 0x00;
  
  //Frame ID as 0x01.
  TX[4] = 0x01;

  xbee802.error_AT = 2;

  // The variables where the old address is saved are initialized.
  old_netAddress[0] = 0x00;
  old_netAddress[1] = 0x00;

  if( packet->communicationMode == BROADCAST ){ // Communication mode: Broadcast.
    type = 15;
    
    // Packet size.
    TX[2] = 11+packet->data_size; 
    
    // Frame type.
    TX[3] = 0x00;
      
    // The own module address is backed up.
    previous = millis();
    xbee802.error_AT = 2;
    while( ((xbee802.error_AT == 1) || (xbee802.error_AT == 2)) && (millis()-previous<500) ){
      status = xbee802.getOwnNetAddress();
      if( millis() < previous ){
        previous=millis();
      }
    }
      
    old_netAddress[0] = xbee802.sourceNA[0];
    old_netAddress[1] = xbee802.sourceNA[1];
      
    previous = millis(); 
    xbee802.error_AT = 2;      
    while( ((xbee802.error_AT == 1) || (xbee802.error_AT == 2)) && (millis()-previous<500) )
    {
      status=xbee802.setOwnNetAddress(0xFF,0xFF);
      net_Address_changed = 1;
      if( millis() < previous ){
        previous=millis();
      }
     }
      
    status_value = 2;
    
    // The destination MAC address (broadcast address) is entered.
    TX[5] = 0x00;
    TX[6] = 0x00;
    TX[7] = 0x00;
    TX[8] = 0x00;
    TX[9] = 0x00;
    TX[10] = 0x00;
    TX[11] = 0xFF;
    TX[12] = 0xFF;
    
    // ACK. 
    TX[13] = 0x00;
    
    // It is generate the payload by an API header and the corresponding data.
    payloadGenerator(packet,TX,14);
        
    // It is introduced the Checksum.
    TX[packet->data_size+14] = checksumGenerator(TX); 
    
  }else if(packet->communicationMode == UNICAST){ // Communication mode: Broadcast.
    
    type = 15;
    
    // Packet size.    
    TX[2] = 11+packet->data_size;
    
    // Frame type.
    TX[3] = 0x00;
    
    // The destination MAC address is entered.
    TX[5] = packet->destinationMAC[0];
    TX[6] = packet->destinationMAC[1];
    TX[7] = packet->destinationMAC[2];
    TX[8] = packet->destinationMAC[3];
    TX[9] = packet->destinationMAC[4];
    TX[10] = packet->destinationMAC[5];
    TX[11] = packet->destinationMAC[6];
    TX[12] = packet->destinationMAC[7];  
      
    // The own module address is backed up.
    previous = millis();
    xbee802.error_AT = 2;
    while( ((xbee802.error_AT == 1) || (xbee802.error_AT == 2)) && (millis()-previous<500) ){
      status = xbee802.getOwnNetAddress();
      if( millis() < previous ){
        previous=millis(); 
      }
    }     
    
    old_netAddress[0] = xbee802.sourceNA[0];
    old_netAddress[1] = xbee802.sourceNA[1];
      
    previous=millis();
    xbee802.error_AT = 2;
    while( ((xbee802.error_AT == 1) || (xbee802.error_AT == 2)) && (millis()-previous<500) ){
      status=xbee802.setOwnNetAddress(0xFF,0xFF);
      net_Address_changed = 1;
      if( millis() < previous ){
        previous=millis(); 
      }
     }
      
      status_value = 2;
      
      TX[13] = 0x00;

      // It is generate the payload by an API header and the corresponding data.
      payloadGenerator(packet,TX,14);
          
      // It is introduced the Checksum.
      TX[packet->data_size+14] = checksumGenerator(TX);
  }else{
    // If the mode has not been determined as Unicast or Broadcast, the value 2 will be returned, indicating that there has been an error in the sending procedure.
    return 2;
  }
   
  //The frame to be sent is adapted to the required mode in AP = 2.
  frameGeneratorAP2(packet,TX,protection,type); 
   
  counter = 0;
  
  // The frame is sent by the corresponding UART.
  while( counter < (packet->data_size+type+protection) ){
    // The byte is printed in associated UART.
    printByte(TX[counter], SOCKET_ZIGBEE); 
    counter++;
  }
  counter = 0;
  
  // The transmitter response is collected to see if the send has been correct.
  xbee802.error_TX = transmissionStatusResponse();
  status_value = xbee802.error_TX;
    
  // Enter again the MAC address of the module.
  if( net_Address_changed == 1 ){
    xbee802.error_AT = 2;
    previous = millis();
    while( ((xbee802.error_AT == 1) || (xbee802.error_AT == 2)) && (millis()-previous<500) ){
      status = xbee802.setOwnNetAddress(old_netAddress[0],old_netAddress[1]);
      if( millis() < previous ){
        previous=millis(); 
      }
    }
  }    
    return status_value;
}

void Zigbee::payloadGenerator(struct ZigbeePacket* paquete, uint8_t* TX_array,uint8_t start_pos ){
  // It is stored in the array that represents the frame to transmit the associated data, from the appropriate position.
  for( uint16_t j=0 ; j<paquete->data_size ; j++){
    TX_array[start_pos+j]=uint8_t(paquete->data[j]);
  }
}

uint8_t Zigbee::checksumGenerator(uint8_t* TX){
  uint8_t checksum = 0;
  uint16_t length_field = (uint16_t)((TX[1]<<8)&0xFF00) + (uint16_t)(TX[2]&0x00FF);

  //Checksum is calculated.
  for( uint16_t i=0 ; i < length_field;i++){
    checksum = (checksum + TX[i+3])&0xFF;
  }

  while( checksum > 255 ){
    checksum = checksum - 256;
  }
  checksum = 255 - checksum;
  return checksum;
}

void Zigbee::frameGeneratorAP2(struct ZigbeePacket* packet,uint8_t* TX_array,uint8_t &protect,uint8_t type){
    uint16_t index = 1;
    bool final = false;
    uint16_t aux = 0;
    uint16_t aux2 = 0;

    // The matrix representing the data frame is traversed to adapt it to AP2 mode.
    while( index < ( packet->data_size + type + protect) ){
        if( ( TX_array[index] == 0x11)
    ||  ( TX_array[index] == 0x13)
    ||  ( TX_array[index] == 0x7E)
    ||  ( TX_array[index] == 0x7D) )
        {
            TX_array[index] = TX_array[index] xor 0x20;
            protect++;
            aux = TX_array[index];
            TX_array[index] = 0x7D;
            uint16_t k = index - 1;
            
            // After including the escpaing indicator: 0x7D the escape value is included and the rest of the buffer is copied
            while( final == false )
            {
                aux2 = TX_array[k+2];
                TX_array[k+2] = aux;
                if( ( k + 3 ) >= ( packet->data_size + type + protect ) )
                {
                    final = true;
                    break;
                }
                aux = TX_array[k+3];
                TX_array[k+3] = aux2;
                if( ((k+4)>=(packet->data_size+type+protect)) )
                {
                    final = true;
                    break;
                }
                k++;
                k++;
            }
            final = false;
        }
        index++;
    }
}


uint8_t Zigbee::sendZigbeePackets( uint8_t* macAddress, uint8_t* data, uint16_t size_data ){
  // Counter for data forwarding.
  uint8_t forwarding_counter = 0;

  // Zigbee packet structure
  ZigbeePacket packet; 

  // The buffer associated with the packet is initialized.
  memset( &packet, 0x00, sizeof(packet) );

  // The transmission mode is chosen. It can be UNICAST or BROADCAST
  packet.communicationMode = UNICAST;

  //The parameters associated with the structure of the Zigbee packet are added.
  parametersDestinationPacket( &packet, macAddress, data, size_data);

  // The Zigbee packet associated with the data collected by parameters will be sent, taking into account that in case of error in the send there will be a maximum number of tryings.
  while( forwarding_counter <= MAX_SENDS )
  {
    // The packet sending function is called.
    sendZigbee(&packet);

    // The error flag is verified, to determine if the send has been correct.
    if( xbee802.error_TX == 0 ){
      // In this case the value 0 is returned, indicating that the send has been correct.
      return 0;
    }else{
      // In the case of an error, 300ms are expected before the next send.
      delay(300);
    }
    // The forwarding counter is increased.
    forwarding_counter++;
  }
  return 1;
}

uint8_t Zigbee::parametersDestinationPacket(ZigbeePacket* paq, uint8_t* address,uint8_t* data,uint8_t size_data){

  // MAC address is assigned.
  for(uint8_t i=0; i<8; i++){
    paq->destinationMAC[i] = address[i];
  }

  // Variable used to the assign of data to the corresponding field of the created packet.
  uint16_t data_counter = 0; 
    
    while( data_counter < size_data ){
        paq->data[data_counter] = data[data_counter];
        data_counter++;

        // In the case that the maximum data is reached, the frame is truncated.
        if( data_counter>=DATA_MAX ) break;
    }
    // The size of the data is saved in the Zigbee packet structure.
    paq->data_size=data_counter;
    return 1;
}


uint8_t Zigbee::transmissionStatusResponse(){
  // create reception buffer
  uint8_t ByteIN[MAX_PARSE];
    unsigned long previous=millis();

     // set number of bytes that TX Status frame (0x89) has
    uint16_t numberBytes=7;
    uint8_t end=0;
    uint16_t counter3=0;
    uint8_t undesired=0;
    uint8_t status=0;
    uint16_t num_TX=0;
    uint8_t num_esc=0;
    uint16_t interval=5000;
    uint8_t num_mes=0;
    uint16_t i=1;
    uint16_t length_mes=0;
    uint16_t length_prev=0;
    uint8_t maxFrame=110;

    xbee802.error_TX=2;

  // If a frame was truncated before, we set the first byte as 0x7E
  // and we add a new packet to 'num_mes' counter
    if( xbee802.frameNext ){
        ByteIN[0]=0x7E;
        counter3=1;
        num_mes=1;
        xbee802.frameNext=0;
    }

  // Read data from XBee while data is available
    while( end==0 && !xbee802.frameNext ){
    // check available data
    if(serialAvailable(xbee802.uart)>0){
      // read byte from correspondent uart
            ByteIN[counter3]=serialRead(xbee802.uart);
            counter3++;
            previous=millis();

            // check if a new frame is received
            if(ByteIN[counter3-1]==0x7E){
        // if there is no memory available for a whole new packet
        // then we escape and select frameNext=1 in order to get it
        // the next time we read from XBee
              if( (MAX_PARSE-counter3) < maxFrame ){
          xbee802.frameNext=1;
        }else num_mes++;
            }

            // If some corrupted frame has appeared, it is discarded,
            // counter3 is set to zero again
            if( (counter3==1) && (ByteIN[counter3-1]!=0x7E) )
            {
        counter3=0;
      }

      // if counter3 reaches the maximum data to parse, then finish
            if( counter3>=MAX_PARSE )
            {
        end=1;
      }

      // Discard any non-TX status frame which are determined by a frame
      // type which may be 0x89(TX Status) or 0x8A(Modem Status)
            if( (counter3 == (4 + (uint16_t)status*6 + undesired))
        && (undesired != 1)  )
            {
                if( (ByteIN[counter3-1]!= 0x89) && (ByteIN[counter3-1]!=0x8A) ){
          // increment undesired counter
                  undesired=1;

                  // sum 3 new bytes corresponding to
                  // start delimiter (1Byte) + length (2Bytes)
                    numberBytes+=3;
                }
            }

            // if undesired counter is active, increment 'numberBytes'
            if( undesired == 1 ){
        numberBytes++;
      }

            // If a escape character (0x7D) is found increment 'numberBytes'
          if( (ByteIN[counter3-1]==0x7D) && (!undesired) ){
              numberBytes++;
          }

          /* If a modem status frame (0x8A)
             *  ____________________________________________________
             * |      |     |     |            |         |          |
             * | 0x7E | MSB | LSB | Frame Type | cmdData | checksum |
             * |______|_____|_____|____________|_________|__________|
             *    0      1     2        3           4         5
             */
          if( (ByteIN[counter3-1] == 0x8A) && (counter3 == (4+(uint16_t)status*6)) ){
        // increment in 6Bytes 'numberBytes'
              numberBytes+=6;

              // increment 'status' in order to add a new 'modem status frame'
              status++;
          }

          // If a new frame is read after reading any undesired frame,
          // decrement 'numberBytes' and set undesired
          if( (ByteIN[counter3-1] == 0x7E) && (undesired == 1) ){
              numberBytes--;
              undesired=numberBytes-7;
          }

          // if 'counter3' is the same as 'numberBytes' we finish
          // This means that TX status has been found
          if(counter3 == numberBytes){
        end=1;
          }
        }

        // avoid millis overflow problem
        if( millis() < previous ) previous=millis();

        // check if time is out
        if( (millis()-previous) > interval ){
            end=1;
            serialFlush(xbee802.uart);
        }
    }

    // Store number of read bytes
    num_TX=counter3;
    counter3=0;

  #if DEBUG_XBEE > 0
  PRINT_XBEE(F("RX:"));
    for(uint16_t i = 0; i < num_TX ; i++){
    USB.printHex(ByteIN[i]);
  }
  USB.println();
  #endif

    // If some corrupted frame has appeared we jump it
    if( ByteIN[0]!=0x7E ){
    // jump until a new frame start delimiter is found
    while( ByteIN[i]!=0x7E && i<num_TX )
        {
      i++;
    }
  }

    // Parse the received messages from the XBee
    while( num_mes>0 ){
    // get length of the packet until another start delimiter
    // is found or the end of read bytes is reached
        while( ByteIN[i]!=0x7E && i<num_TX ){
      i++;
    }
        length_mes=i-length_prev;

    // If any byte has been escaped, it must be converted before parsing it
    // So first count number of escaped bytes and then make conversion
    for( xbee802.it=0; xbee802.it < length_mes ; xbee802.it++){
            if( ByteIN[xbee802.it+length_prev]==0x7D ) num_esc++;
        }

        // if there are escaped bytes, make conversion
        if( num_esc ){
      xbee802.des_esc(ByteIN,length_mes,i-length_mes);
    }

    /* Call parsing function depending on the Frame Type
     *  _______________________________________________
     * |      |     |     |          |         |
     * | 0x7E | MSB | LSB | Frame Type |    ......     |
     * |______|_____|_____|____________|_______________|
     *    0      1     2        3          variable
     */
        switch( ByteIN[(i-length_mes)+3] ){
            case 0x8A : //Modem Status
            xbee802.modemStatusResponse(ByteIN,length_mes-num_esc+length_prev,i-length_mes);
            break;

            case 0x80 : // XBee_802 - RX (Receive) Packet: 64-bit Address
            case 0x81 : // XBee_802 - RX (Receive) Packet: 16-bit Address
            xbee802.error_RX=xbee802.rxData(ByteIN,length_mes-num_esc+length_prev,i-length_mes);
            break;

            case 0x89 : // TX (Transmit) Status
            xbee802.delivery_status=ByteIN[i-length_mes+5];
            if( xbee802.delivery_status == 0 ){
              xbee802.error_TX=0;
            }else{
              xbee802.error_TX=1;
            }
            break;

            default   : break;
        }

    // decrement number of pending packets to be treated
        num_mes--;

        // update previous index in input buffer in order to carry on with the
        // following message stored in 'memory'
        length_prev=i;
        i++;
        num_esc=0;

    }

    return xbee802.error_TX;
}


void Zigbee::coordinatorMACRequest(){
  // Construction of the frame and send of this.
  uint8_t requestMACFrame [2];
  // The frame is initialized
  memset(requestMACFrame,0x00,sizeof(requestMACFrame));
  // The parameters of the destination network and the purpose of the frame are established.
  requestMACFrame[0] = ZIGBEE_NETWORK; //Network: Frame directed to the Zigbee network.
  requestMACFrame[1] = MAC_REQUEST; //Purpouse: MAC request.
  send_status = sendZigbeePackets( boradcast_address, requestMACFrame, sizeof(requestMACFrame) );
  if(send_status == 0){
    #if DEBUG_ZIGBEE == 1
      USB.println(F("Send MAC request correct."));
    #endif
  }else{
    #if DEBUG_ZIGBEE == 1
      USB.println(F("Send MAC request incorrect."));
    #endif
  }
  
  // Once the request has been sent, the module will remain waiting to receive a response.
  do{
    reception_status = xbee802.receivePacketTimeout(RECEPTION_TIME_XBEE);
    #if DEBUG_ZIGBEE == 1
      USB.print(F("Reception error: "));USB.println(reception_status,HEX);
    #endif
  }while(reception_status==2);
    
  if(reception_status == 0){
    #if DEBUG_ZIGBEE == 1
      USB.println(F("Receiving data correct."));
    #endif
    
    // The byte that determines the destination network is obtained.
    uint8_t destination_network = xbee802._payload[0];
  
    // It is verified that the packet is directed to a Zigbee network
    if(destination_network & ZIGBEE_NETWORK){ // Zigbee network.
      // After receiving a response, it must verify that the packet is destined for the node that sent the request, since the coordinator sends the message on Broadcast mode.
      
      // The first step is to obtain the MAC of the node that is running the code.
      uint8_t ownMAC[8];
      memset(ownMAC,0x00,sizeof(ownMAC));
      getOwnMAC(ownMAC);
      
      // Then the MAC that comes in the payload is checked. And if it coincides with the MAC of the device (which means that the message was addressed to it), the coordinator's MAC is saved.
      uint8_t receivedMAC[8];
      memset(receivedMAC,0x00,sizeof(receivedMAC));
      memcpy(receivedMAC, xbee802._payload+3, 8);
      
      /** Boolean variable to see if the MAC matches. */
      bool coincidenciaMAC = true;

      for(uint8_t i=0; i<8; i++){
        if(ownMAC[i] != receivedMAC[i]){
          coincidenciaMAC = false;
        }
      }
      
      // If the MAC matches, it means that the packet received is for the module that executes this code. Otherwise the packet is directed to another module.
      if(coincidenciaMAC){
        memset(coordinatorAddress,0x00,sizeof(coordinatorAddress));
        //memcpy(coordinatorAddress, xbee802._srcMAC, 8);
        getDestinationMAC(coordinatorAddress);
        
        #if DEBUG_ZIGBEE == 1
          USB.print(F("Coordinator MAC Address "));
          for(int i=0; i<8; i++){
            USB.print(coordinatorAddress[i],HEX); 
          }
        #endif

        status_coordinatorAddress = true;
        #if DEBUG_ZIGBEE == 1
          USB.print("Id node: ");
          USB.println(xbee802._payload[2],HEX);
        #endif;
        
        // The node identifier is saved in EEPROM memory.
        setIdNodo(xbee802._payload[2]);
      }else{
        send_counter++; // The send counter is increased.
        status_coordinatorAddress = false;
      }
    }
    
  }else{
    #if DEBUG_ZIGBEE == 1
      USB.println(F("Receiving data incorrect."));
    #endif
  }  
}

void Zigbee::getOwnMAC(uint8_t *MACAddress){
  xbee802.ON();
  xbee802.getOwnMac(); 
  memcpy(MACAddress, xbee802.sourceMacHigh, sizeof(xbee802.sourceMacHigh));
  memcpy(MACAddress+sizeof(xbee802.sourceMacHigh), xbee802.sourceMacLow, sizeof(xbee802.sourceMacLow));
}

void Zigbee::getDestinationMAC(uint8_t *MACAddress){
  memcpy(MACAddress, xbee802._srcMAC, sizeof(xbee802._srcMAC));
}


uint8_t Zigbee::getIdNodo(){
  // Auxiliary variable to write the node identifier in EEPROM memory.
  uint8_t aux_eeprom = 0;
  aux_eeprom = Utils.readEEPROM(ADDRESS_ID_EEPROM);
  return aux_eeprom;  
}

void Zigbee::setIdNodo(uint8_t node_identifier){
  Utils.writeEEPROM(ADDRESS_ID_EEPROM,node_identifier);
}

void Zigbee::addBoardSensor(uint8_t type, int value, uint8_t size_value){
  uint8_t data[4];

  // In the case that the value is less than zero, the sign of the data sent will be entered in the associated field in the frame. (0-> Positive, 1-> Negative). 
  // In addition, the sign of value is reversed for its treatment and inclusion in the plot.
  if(value < 0){
     data_send_Zigbee[initial_position_data_send_Zigbee+2] |= 0x80;
     value = -value;
  }
  
  conversions_object.int_to_uint8t(value,data);
  addBoardSensor(type, data, size_value);
}


void Zigbee::addBoardSensor(uint8_t type, int value1, int value2, uint8_t size_value){
  uint8_t data1[4];
  uint8_t data2[4];

  // In the case that the value is less than zero, the sign of the data sent will be entered in the associated field in the frame. (0-> Positive, 1-> Negative). 
  // In addition, the sign of value is reversed for its treatment and inclusion in the plot.
  if(value1 < 0){
     data_send_Zigbee[initial_position_data_send_Zigbee+2] |= 0x80;
     value1 = -value1;
  }
  if(value2 < 0){
     data_send_Zigbee[initial_position_data_send_Zigbee+2] |= 0x40;
     value2 = -value2;
  }
  
  conversions_object.int_to_uint8t(value1,data1);
  conversions_object.int_to_uint8t(value2,data2);
  
  addBoardSensor(type, data1, data2, size_value);
}


void Zigbee::addBoardSensor(uint8_t type, int value1, int value2, int value3 ,uint8_t size_value){
  uint8_t data1[4];
  uint8_t data2[4];
  uint8_t data3[4];

  // In the case that the value is less than zero, the sign of the data sent will be entered in the associated field in the frame. (0-> Positive, 1-> Negative). 
  // In addition, the sign of value is reversed for its treatment and inclusion in the plot.
  if(value1 < 0){
     data_send_Zigbee[initial_position_data_send_Zigbee+2] |= 0x80; 
     value1 = -value1;
  }
  if(value2 < 0){
     data_send_Zigbee[initial_position_data_send_Zigbee+2] |= 0x40;
     value2 = -value2;
  }
  if(value3 < 0){
     data_send_Zigbee[initial_position_data_send_Zigbee+2] |= 0x20;
     value3 = -value3;
  }

  conversions_object.int_to_uint8t(value1,data1);
  conversions_object.int_to_uint8t(value2,data2);
  conversions_object.int_to_uint8t(value3,data3);
  
  addBoardSensor(type, data1, data2, data3, size_value);
}



void Zigbee::addBoardSensor(uint8_t type, uint8_t* value, uint8_t size_value){
  uint8_t size_frame; // Frame size that will be stored in the buffer, to verify that it does not exceed the sending limit.
  size_frame = initial_position_data_send_Zigbee + size_value + 3; //+3-> Type, number of values and size of the value.

  // In the case that the data to be entered in the sending buffer excees the maximum of the buffer, it will be saved
  // in the second buffer and the first will be sent, and then pass the data from the second buffer to the first.
  if(size_frame > sizeof(data_send_Zigbee)){
    packet_fragmentation = true; 
    sendDataBoardSensor();
    memset(data_send_Zigbee, 0x00, sizeof(data_send_Zigbee)); // The buffer is initialize
    initial_position_data_send_Zigbee = 0;                    // The initial position is initialize.
  }else{
    packet_fragmentation = false;
  }
    data_send_Zigbee[initial_position_data_send_Zigbee] = type;                               // Sensor identifier.
    data_send_Zigbee[initial_position_data_send_Zigbee+1] = 0x01;                            // Number of sensor values.
    data_send_Zigbee[initial_position_data_send_Zigbee+2] |= size_value;                    // Size of the each value.
    memcpy(data_send_Zigbee + initial_position_data_send_Zigbee + 3, value, size_value);    // Sensor data.
    initial_position_data_send_Zigbee = initial_position_data_send_Zigbee + 3 + size_value; // The value for the next insertion in the buffer is updated.
    number_of_sensors++; // The variable associated with the number of sensors is increased.
}


void Zigbee::addBoardSensor(uint8_t type, uint8_t* value1, uint8_t* value2, uint8_t size_value){
  uint8_t size_frame; // Frame size that will be stored in the buffer, to verify that it does not exceed the sending limit.
  size_frame = initial_position_data_send_Zigbee + (size_value*2) + 3; //+3-> Type, number of values and size of the value.

  // In the case that the data to be entered in the sending buffer excees the maximum of the buffer, it will be saved
  // in the second buffer and the first will be sent, and then pass the data from the second buffer to the first.
  if(size_frame > sizeof(data_send_Zigbee)){
    packet_fragmentation = true; 
    sendDataBoardSensor();
    memset(data_send_Zigbee, 0x00, sizeof(data_send_Zigbee)); // The buffer is initialize
    initial_position_data_send_Zigbee = 0;                    // The initial position is initialize.
  }else{
    packet_fragmentation = false;
  }
    data_send_Zigbee[initial_position_data_send_Zigbee] = type;                            // Sensor identifier.
    data_send_Zigbee[initial_position_data_send_Zigbee+1] = 0x02;                         // Number of sensor values.
    data_send_Zigbee[initial_position_data_send_Zigbee+2] |= size_value;                  // Size of the each value.
    memcpy(data_send_Zigbee + initial_position_data_send_Zigbee + 3, value1, size_value); // Sensor data.
    memcpy(data_send_Zigbee + initial_position_data_send_Zigbee + 3 + size_value, value2, size_value); // Sensor data.
    initial_position_data_send_Zigbee = initial_position_data_send_Zigbee + 3 + (size_value*2); // The value for the next insertion in the buffer is updated.
    number_of_sensors++; // The variable associated with the number of sensors is increased.
}


void Zigbee::addBoardSensor(uint8_t type, uint8_t* value1, uint8_t* value2, uint8_t* value3, uint8_t size_value){
  uint8_t size_frame; // Frame size that will be stored in the buffer, to verify that it does not exceed the sending limit.
  size_frame = initial_position_data_send_Zigbee + (size_value*3) + 3; //+3-> Type, number of values and size of the value.

  // In the case that the data to be entered in the sending buffer excees the maximum of the buffer, it will be saved
  // in the second buffer and the first will be sent, and then pass the data from the second buffer to the first.
  if(size_frame > sizeof(data_send_Zigbee)){
    packet_fragmentation = true;
    sendDataBoardSensor();
    memset(data_send_Zigbee, 0x00, sizeof(data_send_Zigbee)); // The buffer is initialize
    initial_position_data_send_Zigbee = 0;                    // The initial position is initialize.
  }else{
    packet_fragmentation = false; 
  }
    data_send_Zigbee[initial_position_data_send_Zigbee] = type;                              // Sensor identifier.
    data_send_Zigbee[initial_position_data_send_Zigbee+1] = 0x03;                           // Number of sensor values.
    data_send_Zigbee[initial_position_data_send_Zigbee+2] |= size_value;                    // Size of the each value.
    memcpy(data_send_Zigbee + initial_position_data_send_Zigbee + 3, value1, size_value);   // Sensor data.
    memcpy(data_send_Zigbee + initial_position_data_send_Zigbee + 3 + size_value, value2, size_value); // Sensor data.
    memcpy(data_send_Zigbee + initial_position_data_send_Zigbee + 3 + (size_value*2), value3, size_value); // Sensor data.
    initial_position_data_send_Zigbee += initial_position_data_send_Zigbee + 3 + (size_value*3);  // The value for the next insertion in the buffer is updated.
    number_of_sensors++; // The variable associated with the number of sensors is increased.
}

void Zigbee::sendDataBoardSensor(){

  // In case the coordinator's MAC has not been established, it is requested again.
  while(send_counter < MAX_SENDS && status_coordinatorAddress == false){
    coordinatorMACRequest();
  }
 
  memset(frameDataSensor,0x00,sizeof(frameDataSensor));

  // If the data has been fragmented the associated flag is put to high level.
  if(packet_fragmentation){
     frameDataSensor[1] = 0x80;
  }else{
     frameDataSensor[1] = 0x00;
  }
  
  frameDataSensor[0] = LORAWAN_NETWORK;       // Network: LoRaWAN.
  frameDataSensor[1] |= BOARD_SENSOR;        // Type: board sensor.
  frameDataSensor[2] = getIdNodo();         // Node identifier.
  frameDataSensor[3] = number_of_sensors;  // Nomber of sensors.
  memcpy(frameDataSensor+4, data_send_Zigbee, sizeof(data_send_Zigbee));
  send_status = sendZigbeePackets( coordinatorAddress, frameDataSensor, sizeof(frameDataSensor) );
  if(send_status == 0){
    #if DEBUG_ZIGBEE == 1
      USB.println(F("Correct sending of data."));
    #endif
    memset(data_send_Zigbee, 0x00, sizeof(data_send_Zigbee)); // The buffer is initialize.
    initial_position_data_send_Zigbee = 0;                    // The initial position is initialize.
    number_of_sensors = 0;                                    // The counter of the number of sensors is initialize.
  }else{
    #if DEBUG_ZIGBEE == 1
      USB.print("Error send: ");USB.println(send_status,HEX);
    #endif
  }
}


void Zigbee::sendRTCData(){
    
    // In case the coordinator's MAC has not been established, it is requested again.
    while(send_counter < MAX_SENDS && status_coordinatorAddress == false){
      coordinatorMACRequest();
    }
   
    memset(frameDataSensor,0x00,sizeof(frameDataSensor));
  
    char timeRTC[100];
    memcpy(timeRTC, RTC.getTime(), sizeof(timeRTC));
    USB.println(timeRTC);
    
    frameDataSensor[0] = LORAWAN_NETWORK;       // Network: LoRaWAN.
    frameDataSensor[1] = RTC_TIME;        // Type: board sensor.
    frameDataSensor[2] = getIdNodo();         // Node identifier.
    frameDataSensor[3] = RTC.hour;
    frameDataSensor[4] = RTC.minute;
    frameDataSensor[5] = RTC.second;
    #if DEBUG_ZIGBEE == 1
      USB.print("Hora Enviada: ");USB.print((int)frameDataSensor[3]);USB.print(":");USB.print((int)frameDataSensor[4]);USB.print(":");USB.println((int)frameDataSensor[5]);
      USB.println(RTC.getTime());
    #endif
    send_status = sendZigbeePackets( coordinatorAddress, frameDataSensor, sizeof(frameDataSensor) );
    if(send_status == 0){
      #if DEBUG_ZIGBEE == 1
        USB.println(F("Correct sending of data."));
      #endif
    }else{
      #if DEBUG_ZIGBEE == 1
        USB.print("Error send: ");USB.println(send_status,HEX);
      #endif
    }
}
