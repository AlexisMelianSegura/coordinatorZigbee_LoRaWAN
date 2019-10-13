/**
 * \file coordinadorZigbee.pde
 * \author Alexis Melian Segura
 * \date 21/08/19
 * \brief Porgram that define the coordinator Zigbee-LoRaWAN functionality.
 */

/****************************************************************************
*                             Includes                                     *
****************************************************************************/

#include "Zigbee.h"
#include "Conversions.h"
#include "Frame.h"
#include "SDCard.h"
#include "LoRaWAN.h"

/****************************************************************************
*                              Defines                                      *
****************************************************************************/

#define DEBUG 0

/****************************************************************************
*                                  Variables                                *
****************************************************************************/
/** Status of the Zigbee data reception. */
int8_t reception_status;

/** Buffer to store the data that will be seen via LoRaWAN. */
uint8_t lorawan_data[50];

/** Broadcast Address */
uint8_t broadcast_address[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF};

int int_idNodo;


/****************************************************************************
*                             Función Setup                                 *
****************************************************************************/

void setup(){  
  // Serial port is initializate.
  USB.ON();
  #if DEBUG == 1
    USB.println("Coordinator module");
  #endif
  
  // Module Zigbee is initialize.
  zigbee_object.init_Zigbee();  
  
  // SD Card is initialize.
  sdcard_object.init_SD();

  // Module LoRaWAN is initialize.
  lorawan_object.init_lorawan();
}

/****************************************************************************
*                             Función Loop                                  *
****************************************************************************/

void loop(){           
  
  reception_status = zigbee_object.receiveZigbee(lorawan_data);

  // In the case that a packet has been received.
  if( reception_status == 0 ){

  // The identifier of the destination network is obtained of the receive frame.
  uint8_t destination_network = lorawan_data[0];

  uint8_t purspouse_frame = lorawan_data[1];
   

  if(destination_network & LORAWAN_NETWORK){  // LoRaWAN network.
    switch(purspouse_frame & 0x7F){ // The purpouse of the frame is identifier.      
      case BOARD_SENSOR: // Sending the associated data to a board sensors.
        #if DEBUG == 1
          USB.println("Board Sensor");
        #endif

        #if DEBUG == 1
          USB.print("Sensor type: ");USB.println(lorawan_data[3],HEX);
          USB.print("Value: ");USB.println(lorawan_data[5],HEX);
        #endif

        lorawan_object.lorawan_send(PORT, lorawan_data, sizeof(lorawan_data));
        break;
      case EXTERNAL_SENSOR: // Sending the associated data to a external sensors.
        #if DEBUG == 1
          USB.println("External Sensor");
        #endif
        break;
      case MAC_REQUEST: // MAC request made by a Zigbee node.
        #if DEBUG == 1
          USB.println("MAC Request");
        #endif
        break;
      case MAC_ANSWER: // Answer to the MAC request made by a Zigbee node.
        #if DEBUG == 1
          USB.println("MAC Answer");
        #endif
        break;  
      case ACTUATOR: // Data associated with the interaction with actuators.
        #if DEBUG == 1
          USB.println("Actuator");
        #endif
        break;  
      case ZIGBEE_DATA: // Data directed to a Zigbee node.
        #if DEBUG == 1
          USB.println("Zigbee data");
        #endif
        break; 
      case VLC_DATA: // Data directed to a VLC network.
        #if DEBUG == 1
          USB.println("VLC data");
        #endif
        break;  
      default:
        break;  
      }
    }

    if(destination_network & VLC_NETWORK){  // VLC network.
      switch(purspouse_frame & 0x7F){ // The purpouse of the frame is identifier.
        case BOARD_SENSOR: // Sending the associated data to a board sensors.
        #if DEBUG == 1
          USB.println("Board Sensor");
        #endif
        break;
      case EXTERNAL_SENSOR: // Sending the associated data to a external sensors.
        #if DEBUG == 1
          USB.println("External Sensor");
        #endif
        break;
      case MAC_REQUEST: // MAC request made by a Zigbee node.
        #if DEBUG == 1
          USB.println("MAC Request");
        #endif
        break;
      case MAC_ANSWER: // Answer to the MAC request made by a Zigbee node.
        #if DEBUG == 1
          USB.println("MAC Answer");
        #endif
        break;  
      case ACTUATOR: // Data associated with the interaction with actuators.
        #if DEBUG == 1
          USB.println("Actuator");
        #endif
        break;  
      case ZIGBEE_DATA: // Data directed to a Zigbee node.
        #if DEBUG == 1
          USB.println("Zigbee data");
        #endif
        break; 
      case VLC_DATA: // Data directed to a VLC network.
        #if DEBUG == 1
          USB.println("VLC data");
        #endif
        break;  
      default:
        break;     
        }
    }

    if(destination_network & ZIGBEE_NETWORK){  // Zigbee network.
      switch(purspouse_frame & 0x7F){ // The purpouse of the frame is identifier.
        case BOARD_SENSOR: // Sending the associated data to a board sensors.
        #if DEBUG == 1
          USB.println("Board Sensor");
        #endif
        break;
      case EXTERNAL_SENSOR: // Sending the associated data to a external sensors.
        #if DEBUG == 1
          USB.println("External Sensor");
        #endif
        break;
      case MAC_REQUEST: // MAC request made by a Zigbee node.
        #if DEBUG == 1
          USB.println("MAC Request");
        #endif

        // It is made the answer to the node that send the request.
        uint8_t frameRequestMAC[11];
        memset(frameRequestMAC,0x00,sizeof(frameRequestMAC));
        
        // The MAC address of the node that made the request is obtained.
        uint8_t nodeRequestMAC[8];
        memset(nodeRequestMAC,0x00,sizeof(nodeRequestMAC));
        zigbee_object.getDestinationMAC(nodeRequestMAC);
        
        // It is checked if the MAC is registered in the coordinator. If yes, its identifier is returned. If it is not registered, it is inserted in the address table and assigned an identifier.
        uint8_t idNodo;
        int_idNodo = sdcard_object.chkMACNode(nodeRequestMAC);
        if(int_idNodo == 0){
          USB.println(F("It could not save the node MAC Address."));
        }else{
          idNodo = (uint8_t) int_idNodo;
          frameRequestMAC[0] = 0x04; // Netwoek: Zigbee network.
          frameRequestMAC[1] = 0x04; // Type: MAC_ANSWER
          frameRequestMAC[2] = idNodo; // Node identifier.
          // The MAC is copied from the third byte of the payload.
          memcpy(frameRequestMAC+3,nodeRequestMAC,sizeof(nodeRequestMAC));
          uint8_t send_status = zigbee_object.sendZigbeePackets( broadcast_address, frameRequestMAC, sizeof(frameRequestMAC) );
          #if DEBUG == 1
            if(send_status == 0){
              USB.println(F("Data send correct."));
            }else{
              USB.println(F("Data send incorrect."));
            }
          #endif
        }
        break;
      case MAC_ANSWER: // Answer to the MAC request made by a Zigbee node.
        #if DEBUG == 1
          USB.println("MAC Answer");
        #endif
        break;  
      case ACTUATOR: // Data associated with the interaction with actuators.
        #if DEBUG == 1
          USB.println("Actuator");
        #endif
        break;  
      case ZIGBEE_DATA: // Data directed to a Zigbee node.
        #if DEBUG == 1
          USB.println("Zigbee data");
        #endif
        break; 
      case VLC_DATA: // Data directed to a VLC network.
        #if DEBUG == 1
          USB.println("VLC data");
        #endif
        break;  
      default:
        break;
      }
      
    }
   
  }else{
    // Error message:
    /*
     * '7' : Buffer full. Not enough memory space
     * '6' : Error escaping character within payload bytes
     * '5' : Error escaping character in checksum byte
     * '4' : Checksum is not correct	  
     * '3' : Checksum byte is not available	
     * '2' : Frame Type is not valid
     * '1' : Timeout when receiving answer   
    */
    #if DEBUG == 1
      USB.print("Error in the Zigbee data reception:");
      USB.println(reception_status,DEC);     
    #endif
  } 
}
