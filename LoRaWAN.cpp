/**
 * \file LoRaWAN.cpp
 * \author Alexis Melian Segura
 * \date 14/08/19
 * \brief Program that define the LoRaWAN emitter.
 */

/****************************************************************************
*                             Includes                                     *
****************************************************************************/
#include "LoRaWAN.h"

/****************************************************************************
*                               Objects                                     *
****************************************************************************/

Lorawan lorawan_object = Lorawan();

/****************************************************************************
*                              Variables                                    *
****************************************************************************/

/** 64-bit identifier of the LoRaWAN device. */
char DEVICE_EUI[] = "0102030405060708";

/** Unique application identifier used to group LoRaWAN devices. This 64-bit address is used to classify devices by application. */
char APP_EUI[] = "70B3D57ED001D6BF";

/** 128-bit AES secret key shared between the peripheral device and the network. It is used to determine session keys. */
char APP_KEY[] = "87628C495F56C598BA76B9EF767E3418";

/****************************************************************************
*                             Functions                                     *
****************************************************************************/

Lorawan::Lorawan(){
  
}

Lorawan::~Lorawan(){
  
}

void Lorawan::init_lorawan(){
  ///////////////////////////////
  // 1. Activation
  ///////////////////////////////

  // Module LoRaWAN is activated.
  lorawan_status = LoRaWAN.ON(SOCKET_LORAWAN);
  
  #if DEBUG_LORAWAN == 1
    if( lorawan_status == 0 ){
      USB.println(F("1. LoRaWAN activated."));     
    }else{
      USB.print(F("1. Error in the activation of the LoRaWAN module = ")); 
      USB.println(lorawan_status, DEC);
    }
  
    USB.println(F("LoRaWAN module configuration."));
  #endif

  ///////////////////////////////
  // 2. Data Rate
  ///////////////////////////////

  // The value of the data rate is set.
  lorawan_status = LoRaWAN.setDataRate(DATA_RATE_LORAWAN);

  #if DEBUG_LORAWAN == 1
    if( lorawan_status == 0 ){
      USB.println(F("2. Data Rate OK"));     
    }else{
      USB.print(F("2. Configuration Error - Data Rate= ")); 
      USB.println(lorawan_status, DEC);
    }
  #endif

  ///////////////////////////////
  // 3. Device EUI
  ///////////////////////////////

  // The LoRaWAN device identifier is assigned.
  lorawan_status = LoRaWAN.setDeviceEUI(DEVICE_EUI);

  #if DEBUG_LORAWAN == 1
    if( lorawan_status == 0 ){
      USB.println(F("3. Device EUI OK"));     
    }else{
      USB.print(F("3. Configuration Error - Device EUI = ")); 
      USB.println(lorawan_status, DEC);
    }
  #endif

  ///////////////////////////////
  // 4. Application EUI
  ///////////////////////////////

  // The application identifier is assigned.
  lorawan_status = LoRaWAN.setAppEUI(APP_EUI);

  #if DEBUG_LORAWAN == 1
    if( lorawan_status == 0 ){
      USB.println(F("4. Application EUI OK"));     
    }else{
      USB.print(F("4. Configuration Error - Application EUI = ")); 
      USB.println(lorawan_status, DEC);
    }
  #endif

  ///////////////////////////////
  // 5. Set Application Session Key
  ///////////////////////////////

  // The application key is assigned.
  lorawan_status = LoRaWAN.setAppKey(APP_KEY);

  #if DEBUG_LORAWAN == 1
    if( lorawan_status == 0 ){
      USB.println(F("5. Application Key OK"));     
    }else{
      USB.print(F("5. Configuration Error - Application Key = ")); 
      USB.println(lorawan_status, DEC);
    }
  #endif

  /////////////////////////////////
  // 6. Join the LoRaWAN network by OTAA to negotiate the keys with the server
  /////////////////////////////////

  // The procedure for joining the LoRaWAN module through OTAA is called.
  lorawan_status = LoRaWAN.joinOTAA();

  if( lorawan_status == 0 ){
    Utils.setLED(LED1, LED_ON);  
  }else{
    Utils.setLED(LED0, LED_ON);  
  }

  #if DEBUG_LORAWAN == 1
    if( lorawan_status == 0 ){
      USB.println(F("6. Connection to the LoRaWAN network correct."));         
    }else{
      USB.print(F("6. Error in joining the LoRaWAN network = ")); 
      USB.println(lorawan_status, DEC);
    }
  #endif

  ///////////////////////////////
  // 7. Save settings
  ///////////////////////////////

  // The procedure for saving the previously set configuration is called.
  lorawan_status = LoRaWAN.saveConfig();

  #if DEBUG_LORAWAN == 1
    if( lorawan_status == 0 ){
      USB.println(F("7. Saved configuration."));     
    }else{
      USB.print(F("7. Error saving settings = ")); 
      USB.println(lorawan_status, DEC);
    }
  #endif

  ///////////////////////////////
  // 8. LoRaWAN module off
  ///////////////////////////////

  // The LoRaWAN module is turned off once its configuration is complete.
  lorawan_status = LoRaWAN.OFF(SOCKET_LORAWAN);

  #if DEBUG_LORAWAN == 1
    if( lorawan_status == 0 ){
      USB.println(F("8. Module turned off."));     
    }else{
      USB.print(F("8. The module could not be turned off = ")); 
      USB.println(lorawan_status, DEC);
    }
  #endif
}


void Lorawan::lorawan_send(uint8_t port, uint8_t* data_send, uint16_t size_data){
  ///////////////////////////////
  // 1. LoRaWAN module activation.
  ///////////////////////////////

  // The LoRaWAN module is activated.
  lorawan_status = LoRaWAN.ON(SOCKET_LORAWAN);

  #if DEBUG_LORAWAN == 1
    if( lorawan_status == 0 ){
      USB.println(F("1. LoRaWAN module activated."));     
    }else{
      USB.print(F("1. Error activating LoRaWAN module = ")); 
      USB.println(lorawan_status, DEC);
    }
  #endif

  ///////////////////////////////
  // 2. LoRaWAN network connection
  ///////////////////////////////

  lorawan_status = LoRaWAN.joinABP();

  if( lorawan_status == 0 ) {
    #if DEBUG_LORAWAN == 1
      USB.println(F("2. Connection to the LoRaWAN network correct."));   
    #endif  

    ///////////////////////////////
    // 3. Sending confirmed data through LoRaWAN
    ///////////////////////////////
  
    lorawan_status = LoRaWAN.sendConfirmed( PORT, data_send, size_data);

    // Error messages:
    /*
     * '6' : Module hasn't joined a network
     * '5' : Sending error
     * '4' : Error with data length   
     * '2' : Module didn't response
     * '1' : Module communication error   
     */

    #if DEBUG_LORAWAN == 1
      if( lorawan_status == 0 ){
        USB.println(F("3. Send confirmed through LoRaWAN correct.")); 
        /*USB.print(F(" Data sent: "));
        USB.println(data_send); */
      }else{
        USB.print(F("3. LoRaWAN sending error = ")); 
        USB.println(lorawan_status, DEC);
      } 
    #endif
  }
  else 
  {
    #if DEBUG_LORAWAN == 1
      USB.print(F("2. Failed joining to the LoRaWAN network = ")); 
      USB.println(lorawan_status, DEC);
    #endif
  }

  ///////////////////////////////
  // 4. LoRaWAN module turn off
  ///////////////////////////////

  // The LoRaWAN module is turned off.
  lorawan_status = LoRaWAN.OFF(SOCKET_LORAWAN);

  #if DEBUG_LORAWAN == 1
    if( lorawan_status == 0 ){
      USB.println(F("4. LoRaWAN module turned off."));     
    }else{
      USB.print(F("4. Error turned off LoRaWAN module = ")); 
      USB.println(lorawan_status, DEC);
      USB.println();
    }
  #endif
}

bool Lorawan::receive_lorawan(uint8_t* port_received, char* data_received, int* data_size_received, int* fragment_size){
  // Variable Inicialization.
  lorawan_receiving = true;
  *data_size_received = 0;
  *fragment_size = 0;
  status_lorawan_reception = false;

  // The LoRaWAN data reception function is called as long as it has not finished receiving all the data.
  while(lorawan_receiving){
    status_lorawan_reception = lorawan_reception();
    // It is checked if the data has been fragmented through the flag defined in the frame. If so, it will be indicated that the reception is continued. Otherwise, the end of the reception will be indicated.
    if((conversions_object.char_to_uint8t(LoRaWAN._data[2],LoRaWAN._data[3]) & 0x80 ) && status_lorawan_reception == true){
      lorawan_receiving = true;
      *fragment_size = strlen(LoRaWAN._data);
    }else{
      lorawan_receiving = false;
    }
    // The data received through LoRaWAN is updated.
    memset((data_received + ((*data_size_received)*sizeof(char))), '0', (strlen(LoRaWAN._data)*sizeof(char)));
    memcpy((data_received + ((*data_size_received)*sizeof(char))), LoRaWAN._data, (strlen(LoRaWAN._data)*sizeof(char)));
    (*port_received) = LoRaWAN._port;
    (*data_size_received) += strlen(LoRaWAN._data);

    // A timeout is established between fragmented data frames.
    if((conversions_object.char_to_uint8t(LoRaWAN._data[2],LoRaWAN._data[3]) & 0x80 )){
      delay(FRAGMENTATION_WAITING_TIME);   
    }

    // The receive array is initialized to zero.
    memset(LoRaWAN._data, '0', (strlen(LoRaWAN._data)*sizeof(char)));  
  }
  return status_lorawan_reception;
}


bool Lorawan::lorawan_reception(){
  ///////////////////////////////
  // 1. LoRaWAN module activation.
  ///////////////////////////////

  // The LoRaWAN module is activated.
  lorawan_status = LoRaWAN.ON(SOCKET_LORAWAN);

  #if DEBUG_LORAWAN == 1
    if( lorawan_status == 0 ){
      USB.println(F("1. LoRaWAN module activated."));     
    }else{
      USB.print(F("1. Error activating LoRaWAN module = ")); 
      USB.println(lorawan_status, DEC);
    }
  #endif

  ///////////////////////////////
  // 2. LoRaWAN network connection
  ///////////////////////////////

  lorawan_status = LoRaWAN.joinABP();

  if( lorawan_status == 0 ) {
    #if DEBUG_LORAWAN == 1
      USB.println(F("2. Connection to the LoRaWAN network correct."));   
    #endif  

    ///////////////////////////////
    // 3. Sending confirmed data through LoRaWAN
    ///////////////////////////////

    // Data sent to the LoRaWAN gateway for data reception.
    uint8_t data[] = {0x00};
  
    lorawan_status = LoRaWAN.sendConfirmed( PORT, data, sizeof(data));

    // Error messages:
    /*
     * '6' : Module hasn't joined a network
     * '5' : Sending error
     * '4' : Error with data length   
     * '2' : Module didn't response
     * '1' : Module communication error   
     */

      if( lorawan_status == 0 ){
        #if DEBUG_LORAWAN == 1
          USB.println(F("3. Send confirmed through LoRaWAN correct.")); 
        #endif
        
        if (LoRaWAN._dataReceived == true){
          #if DEBUG_LORAWAN == 1
          USB.println(F("Data received by LoRaWAN: ")); 
            USB.print(F(" Port: "));
            USB.print(LoRaWAN._port,DEC);
            USB.print(F("\r\n Data: "));
            USB.print(LoRaWAN._data);
            USB.print(F("\r\n Data Size: "));
            USB.println(strlen(LoRaWAN._data));
          #endif
          
        }  
      }else{
        #if DEBUG_LORAWAN == 1
          USB.print(F("3. LoRaWAN sending error = ")); 
          USB.println(lorawan_status, DEC);
        #endif
      } 
  }
  else 
  {
    #if DEBUG_LORAWAN == 1
      USB.print(F("2. Failed joining to the LoRaWAN network = ")); 
      USB.println(lorawan_status, DEC);
    #endif
  }

  ///////////////////////////////
  // 4. LoRaWAN module turn off
  ///////////////////////////////

  // The LoRaWAN module is turned off.
  lorawan_status = LoRaWAN.OFF(SOCKET_LORAWAN);

  #if DEBUG_LORAWAN == 1
    if( lorawan_status == 0 ){
      USB.println(F("4. LoRaWAN module turned off."));     
    }else{
      USB.print(F("4. Error turned off LoRaWAN module = ")); 
      USB.println(lorawan_status, DEC);
      USB.println();
    }
  #endif

   ///////////////////////////////
  // 5. Message Information Received
  ///////////////////////////////
  if (LoRaWAN._dataReceived == true){
    #if DEBUG_LORAWAN == 1
      USB.println(F("5. LoRaWAN frame received."));
    #endif
    return true;
  }else{
    #if DEBUG_LORAWAN == 1
      USB.println(F("5. No LoRaWAN frames received."));
    #endif
    return false;
  }
}
