/**
 * \file SDCard.h
 * \author Alexis Melian Segura
 * \date 21/08/19
 * \brief Program that define the interactuation with the SD Card.
 */

#ifndef _SDCARD_H
#define _SDCARD_H

/****************************************************************************
*                             Includes                                      *
****************************************************************************/

#ifndef __WPROGRAM_H__
  #include "WaspClasses.h"
#endif

/****************************************************************************
*                             Defines                                       *
****************************************************************************/

#define DEBUG_SDCARD 1

/****************************************************************************
*                             Clase                                         *
****************************************************************************/

class SDCard{

public:

  /**
  * \fn SDCard()
  * 
  * Class constructor.
  */
  SDCard();

  /**
  * \fn ~SDCard()
  * 
  * Class destructor.
  */
  ~SDCard();

  /**
  * \fn void init_SD()
  * 
  * Function that initialize the SD Card and creating the directories and files that will be used.
  */
  void init_SD();

  /**
  * \fn int chkMACNode(uint8_t *MACAddress)
  * \param Pointer to the memory address where the MAC address of the device that sent a packet is saved.
  * \retval Value associated to the node identifier.
  *
  * Function that checks if the MAC is registered in the coordinator. If yes, its identifier is returned. If it is not registered, it is inserted in the associated file on the SD card
  * and an identifier is assigned associated to the line number in which it is in the file.
  */
  int chkMACNode(uint8_t *MACAddress);

private:

  /** Indicator if the operation related to the SD card was successful or no. */
  bool statusSD;

  
  /** Buffer where the data relationship whit the SD Card will be saved. */
  char bufferFileSD[100];

protected:

};

/****************************************************************************
*                             Objects                                       *
****************************************************************************/
extern SDCard sdcard_object;


#endif


