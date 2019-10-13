/**
 * \file LoRaWAN.h
 * \author Alexis Melian Segura
 * \date 14/08/19
 * \brief Program that define the LoRaWAN receiver.
 */

#ifndef _LORAWAN_H
#define lorawan

/****************************************************************************
*                             Includes                                     *
****************************************************************************/

#ifndef __WPROGRAM_H__
  #include "WaspClasses.h"
#endif

#include <WaspLoRaWAN.h>

#include "Conversions.h"

/****************************************************************************
*                             Define                                        *
****************************************************************************/

/** Message debug. */
#define DEBUG_LORAWAN 0

/** Socket of the Waspmote board to which the LoRaWAN module will be connected. */
#define SOCKET_LORAWAN SOCKET1

/** Data rate used in communication by LoRaWAN. */
/*
 * Data Rate    Configuration      Indicative physical bit rate (bits/s)  Maximum payload
 *    0       LoRa: SF12/125 kHz                    250                         51
 *    1       LoRa: SF11/125 kHz                    440                         51
 *    2       LoRa: SF10/125 kHz                    980                         51
 *    3       LoRa: SF9/125 kHz                    1760                        115
 *    4       LoRa: SF8/125 kHz                    3125                        222   
 *    5       LoRa: SF7/125 kHz                    5470                        222
*/
#define DATA_RATE_LORAWAN 3

/** LoRaWAN communication port. */
#define PORT 3

/** Waiting time between receiving fragmented data frames. */
#define FRAGMENTATION_WAITING_TIME 5000

class Lorawan{
  public:

    /**
    * \fn Lorawan()
    * 
    * Class constructor.
    */
    Lorawan();

    /**
    * \fn ~Lorawan()
    * 
    * Class destructor.
    */
    ~Lorawan();

    /**
    * \fn ~init_lorawan()
    * 
    * Function responsible for setting the parameters of the LoRaWAN module and establishing communication with the Gateway.
    */
    void init_lorawan();

    /**
    * \fn void lorawan_send(uint8_t port, uint8_t* data_send, uint16_t size_data)
    * \param Sending port used in the LoRaWAN communication.
    * \param Data to be sent through LoRaWAN.
    * \param Size of the data to send.
    * 
    * Function responsible for sending data through LoRaWAN.
    */
    void lorawan_send(uint8_t port, uint8_t* data_send, uint16_t size_data);

    /**
    * \fn bool lorawan_reception()
    * \retval In the case that data has been received, the function will return true. Otherwise it will return false.
    * 
    * Function responsible for the call to the LoRaWAN reception function (bool receive_lorawan (uint8_t * port_recived, char * data_received, int * data_size_received, int * fragment_size)), and which will handle the data processing, as is the case of fragmentation.
    */
    bool lorawan_reception();

    /**
    * \fn bool receive_lorawan(uint8_t* port_recived, char* data_received, int* data_size_received)
    * \param Data reception port.
    * \param Pointer associated with the data received through LoRaWAN
    * \param Pointer associated with the size of the data received through LoRaWAN
    * \param Pointer associated with the size of the data fragment received in case there is fragmentation. In case there is no fragmentation this value will be zero.
    * \retval In the case that data has been received, the function will return true. Otherwise it will return false.
    * 
    * Function used to receive data through LoRaWAN, where initially a message will be sent so that a downlink link can be established, so that it will be possible to know if data has been received.
    */
    bool receive_lorawan(uint8_t* port_recived, char* data_received, int* data_size_received, int* fragment_size);
  
  private:

    /** Status variable used for verification on the LoRaWAN connection. */
    uint8_t lorawan_status; 

    /** Boolean variable that determines if is stop of receiving LoRaWAN data or no. True -> Start/Continue with data reception. False -> Total data received. It does not continue to receive.*/
    bool lorawan_receiving;

    /** Numbers of LoRaWAN frames received. */
    int num_lorawan_frames_received;

    /** Status LoRWAN reception. */
    bool status_lorawan_reception;
    
  protected:
  
};

/****************************************************************************
*                             Objects                                       *
****************************************************************************/

extern Lorawan lorawan_object;

#endif
