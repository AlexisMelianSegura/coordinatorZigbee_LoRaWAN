/**
 * \file Zigbee.h
 * \author Alexis Melian Segura
 * \date 31/05/19
 * \brief Program that defines the operation of a Zigbee module.
 */

#ifndef ZIGBEE_H
#define ZIGBEE_H

/****************************************************************************
*                               Includes                                    *
****************************************************************************/

#ifndef __WPROGRAM_H__
  #include "WaspClasses.h"
#endif


#include <WaspXBee802.h>
#include <WaspFrame.h>

#include "Conversions.h"
#include "Frame.h"

/****************************************************************************
*                               Defines                                     *
****************************************************************************/

#define DEBUG_ZIGBEE 0

/** Defines if the module will be used as node (1) or as coordinator (0). */
#define ZIGBEE_MODE 1

/** Socket of the Waspmote board to which the Zigbee module will be connected. */
#define SOCKET_ZIGBEE SOCKET0

/** Unicast mode idenfier. */
//#define UNICAST   0

/** Broadcast mode idenfier. */
//#define BROADCAST   1

/** Maximum payload size. */
#define DATA_MAX  50

/** Maximum number of sends. */
#define MAX_SENDS    5

/** Time the module is waiting for the reception of a Zigbee frame. This will be expressed in milliseconds. */
#define RECEPTION_TIME_XBEE 10000

/** Memory address in the EEPROM where the node identifier is located. EEPROM addresses available from 1024 to 4095. */
#define ADDRESS_ID_EEPROM 1025

/*! \struct ZigbeePacket
    \brief  Structure that will compouse the Zigbee packet.
 */ 
struct ZigbeePacket {
  uint8_t communicationMode; /** Communication mode. */
  uint8_t destinationMAC[8]; /** Zigbee destination MAC address. */
  uint8_t data[DATA_MAX];    /** Buffer used for the data. */
  uint16_t data_size;        /** Size of the data. */
};

/****************************************************************************
*                             Class                                         *
****************************************************************************/

class Zigbee{

  public:

    /**
    * \fn Zigbee()
    * 
    * Class constructor.
    */
    Zigbee();
  
    /**
    * \fn ~Zigbee()
    * 
    * Class destructor.
    */
    ~Zigbee();

    /**
    * \fn void init_Zigbee()
    * 
    * Initialization of Zigbee module.
    */
    void init_Zigbee();

    /**
    * \fn void coordinatorMACRequest()
    * 
    * Function responsible for sending to the Zigbee coordinator a request from the MAC of this to send a packet to obtain his MAC.
    */
    void coordinatorMACRequest();

    /**
    * \fn int8_t receiveZigbee(uint8_t* data)
    * 
    * Function that exceute the procedure asociated with the reception of Zigbee data.
    */
    int8_t receiveZigbee(uint8_t* data);

    /**
    * \fn uint8_t sendZigbeePackets( uint8_t* macAddress, uint8_t* data, uint16_t size_data)
    * 
    * Function responsible for managing the sending of data through the Zigbee network.
    */
    uint8_t sendZigbeePackets( uint8_t* macAddress, uint8_t* data, uint16_t size_data);
  
    /**
     * \fn uint8_t parametersDestinationPacket (ZigbeePacket * paq, uint8_t * address, uint8_t * data, uint8_t size_data)
     * \param Pointer associated to the structure created related to the packet to be sent.
     * \param Pointer related to the MAC address of the receiver.
     * \param Pointer related to the data to send.
     * \param Size of the data to be sent.
     * \retval Numeric indicator that will indicate if the assignment has been successfully completed (1).
     *
     * Function responsible for saving in the structure associated with the packet, the data that is will send, such as the MAC address of the receiver, 
     * the data to be transmitted and the size of these.
     */
    uint8_t parametersDestinationPacket(ZigbeePacket* paq, uint8_t* address, uint8_t* data, uint8_t size_data);
    
    /**
    * \fn uint8_t sendZigbee(struct ZigbeePacket* packet)
    * \param Pointer linked to the structure created for sending a Zigbee packet.
    * \retval Value returned by the sendZigbeePriv function, which will indicate whether the send was successful (0) or not.
    *
    * Function that first verifies the maximum payload, that is, the maximum size of the field associated with the data to be sent.
    * Finally deals with build the Zigbee frame to send, adapting it to the mode.
    */
    uint8_t sendZigbee(struct ZigbeePacket* packet);

    /**
    * \fn void payloadGenerator(struct ZigbeePacket* packet, uint8_t* TX_array, uint8_t start_position)
    * \param Pointer linked to the structure created for sending a Zigbee data.
    * \param Pointer associated to the buffer that represents the frame to be transmitted.
    * \param Start position, from which the data in the array representing the frame to be sent will be copied.
    *
    * Function responsible for generating the payload associated with the data frame that will be sent.
    */
    void payloadGenerator(struct ZigbeePacket* packet, uint8_t* TX_array, uint8_t start_position);
  
    /**
    * \fn uint8_t checksumGenerator(uint8_t* TX)
    * \param Pointer associated to the array related to the frame of data to send.
    * \retval Value assigned to the Checksum of the frame.
    *
    * Function whose purpose is to add the value of Checksum to the frame of data to be sent.
    */
    uint8_t checksumGenerator(uint8_t* TX);
  
    /**
    * \fn void frameGeneratorAP2(struct ZigbeePacket* packet, uint8_t* TX_array, uint8_t &protect, uint8_t type)
    * \param Pointer linked to the structure created for data to send via Zigbee.
    * \param Pointer associated to the array related to the frame of data to send.
    * \param Value associated with the identification of whether the frame is protected or not.
    * \param Value associated with the type of the frame.
    *
    * Function that adapts the array associated to the data frame, to the mode required in AP = 2.
    */
    void frameGeneratorAP2(struct ZigbeePacket* packet, uint8_t* TX_array, uint8_t &protect, uint8_t type);

    /**
    * \fn uint8_t transmissionStatusResponse()
    * \retval Size of the value associated with the sensor.
    *
    * Function that manages the status of the reception after sending data through Zigbee, to verify the correct sending of the data.
    */
    uint8_t transmissionStatusResponse();
    
    /**
    * \fn void getOwnMAC(uint8_t *MACAddress)
    * \param Pointer to the memory address where the device's own MAC address will be saved.
    *
    * Function that returns the MAC address of the Zigbee device that executes the code.
    */
    void getOwnMAC(uint8_t *MACAddress);
  
    /**
    * \fn void getDestinatioMAC(uint8_t *MACAddress)
    * \param Pointer to the memory address where the MAC address will be saved.
    *
    * Function that set the MAC address of the device that has sent sent packet.
    */
    void getDestinationMAC(uint8_t *MACAddress);

    /**
    * \fn uint8_t getIdNodo()
    * \retval Node identifier.
    * 
    * Function responsible for returning the value of the node identifier stored in EEPROM memory.
    */
    uint8_t getIdNodo();
  
    /**
    * \fn void setIdNodo(uint8_t node_identifier)
    * \param Node identifier.
    * 
    * Function responsible for writing the node identifier in EEPROM memory.
    */
    void setIdNodo(uint8_t node_identifier);

    /**
    * \fn void addBoardSensor(uint8_t type, int value, uint8_t size_value)
    * \param Identifier of the sensor from which the data is to be sent.
    * \param Value associated with the sensor.
    * \param Size of the value associated with the sensor.
    *
    * Function that adds to the sending data buffer the data sensor indicated through the parameters, where the type, value and size of the value provided by the sensor will be introduced.
    */
    void addBoardSensor(uint8_t type, int value, uint8_t size_value);

    /**
    * \fn void addBoardSensor(uint8_t type, uint8_t* value1, uint8_t* value2, uint8_t size_value)
    * \param Identifier of the sensor from which the data is to be sent.
    * \param Value1 associated with the sensor.
    * \param Value2 associated with the sensor.
    * \param Size of the value associated with the sensor.
    *
    * Function that adds to the sending data buffer the data sensor indicated through the parameters, where the type, value and size of the value provided by the sensor will be introduced.
    */
    void addBoardSensor(uint8_t type, int value1, int value2, uint8_t size_value);

    /**
    * \fn void addBoardSensor(uint8_t type, int value1, int value2, int value3 ,uint8_t size_value)
    * \param Identifier of the sensor from which the data is to be sent.
    * \param Value1 associated with the sensor.
    * \param Value2 associated with the sensor.
    * \param Value3 associated with the sensor.
    * \param Size of the value associated with the sensor.
    *
    * Function that adds to the sending data buffer the data sensor indicated through the parameters, where the type, value and size of the value provided by the sensor will be introduced.
    */
    void addBoardSensor(uint8_t type, int value1, int value2, int value3 ,uint8_t size_value);
    
    /**
    * \fn void addBoardSensor(uint8_t type, uint8_t value, uint8_t size_value)
    * \param Identifier of the sensor from which the data is to be sent.
    * \param Value associated with the sensor.
    * \param Size of the value associated with the sensor.
    *
    * Function that adds to the sending data buffer the data sensor indicated through the parameters, where the type, value and size of the value provided by the sensor will be introduced.
    */
    void addBoardSensor(uint8_t type, uint8_t* value, uint8_t size_value);
  
    /**
    * \fn void addBoardSensor(uint8_t type, uint8_t* value1, uint8_t* value2, uint8_t size_value)
    * \param Identifier of the sensor from which the data is to be sent.
    * \param Value1 associated with the sensor.
    * \param Value2 associated with the sensor.
    * \param Size of the value associated with the sensor.
    *
    * Function that adds to the sending data buffer the data sensor indicated through the parameters, where the type, value and size of the value provided by the sensor will be introduced.
    */
    void addBoardSensor(uint8_t type, uint8_t* value1, uint8_t* value2, uint8_t size_value);
  
    /**
    * \fn void addBoardSensor(uint8_t type, uint8_t* value1, uint8_t* value2, uint8_t* value3, uint8_t size_value)
    * \param Identifier of the sensor from which the data is to be sent.
    * \param Value1 associated with the sensor.
    * \param Value2 associated with the sensor.
    * \param Value3 associated with the sensor.
    * \param Size of the value associated with the sensor.
    *
    * Function that adds to the sending data buffer the data sensor indicated through the parameters, where the type, value and size of the value provided by the sensor will be introduced.
    */
    void addBoardSensor(uint8_t type, uint8_t* value1, uint8_t* value2, uint8_t* value3, uint8_t size_value);

    /**
    * \fn void sendDataBoardSensor()
    * 
    * Function that send the sensor data that has been set by parameters to the Zigbee coordinator.
    */
    void sendDataBoardSensor();

    /**
    * \fn void sendRTCData(
    * 
    * Function that send the time get for the RTC of the board.
    */
    void sendRTCData();

  private:

    /** Variable to control the number of resends. */
    uint8_t send_counter = 0;
    
    /** Buffer where the MAC address of the coordinator is saved. */
    uint8_t coordinatorAddress[8];
    
    /** Buffer where the data to be sent will be saved. */
    uint8_t data_send_Zigbee[45];
    
    /** Initial position from which the data stored in the data_send_Zigbee buffer is saved. */
    uint8_t initial_position_data_send_Zigbee = 0;
    
    /** Number of sensors sent in the frame. */
    uint8_t number_of_sensors = 0;
    
    /** Variable that determines if the coordinator's address has been established and is correct. */
    bool status_coordinatorAddress = false;
    
    /** Zigbee Broadcat address. */
    uint8_t boradcast_address[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF};
    
    /** Variable where the send status identifier is saved. */
    uint8_t send_status;

    /** Variable where the reception status identifier is saved. */
    uint8_t reception_status;

    /** Buffer to send data of the sensors of the board. */
    uint8_t frameDataSensor[DATA_MAX];

    /** Variable to determinate if the data are fragmented (True->Fragmented, False->No Fragmented) */
    bool packet_fragmentation;
  
  protected:
};

/****************************************************************************
*                             Objects                                       *
****************************************************************************/
extern Zigbee zigbee_object;


#endif
