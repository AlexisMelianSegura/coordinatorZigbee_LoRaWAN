/**
 * \file SDCard.cpp
 * \author Alexis Melian Segura
 * \date 16/08/19
 * \brief Program that define the interactuation with the SD Card.
 */

/****************************************************************************
*                             Includes                                     *
****************************************************************************/

#include "SDCard.h"


/****************************************************************************
*                             Objects                                       *
****************************************************************************/

SDCard  sdcard_object = SDCard();

/****************************************************************************
*                            Variables                                      *
****************************************************************************/

/** Name of the directory where is saved the file that containing the addresses of the nodes. */
char nameDirectory[] = "/dir"; 

/** Name of the file where the node addresses and their identifier are stored. */
char nameFile[] = "/dir/dirtable";

/****************************************************************************
*                             Functions                                     *
****************************************************************************/

SDCard::SDCard() {
}


SDCard::~SDCard() {
}


void SDCard::init_SD(){
  SD.ON();

  statusSD=SD.isSD(); 
  #if DEBUG_SDCARD == 1 
    if(!statusSD){
      USB.println(F("No SD card inserted."));
    }
  #endif

  // The directory where the file associated with the address table is saved is created. In the case of error it could be because it is already created.
  // (statusSD = 0-> Directory created successfully statusSD = 1-> Could not create directory)
  statusSD = SD.mkdir(nameDirectory);
  #if DEBUG_SDCARD == 1
    if(!statusSD){
      USB.println(F("Directory already created."));
    }
  #endif

  // The file where the addresses are saved is created. In case of error it may be that the file has been previously created.
  statusSD = SD.create(nameFile);
  #if DEBUG_SDCARD == 1
    if(!statusSD){
      USB.println(F("Error in file creation."));
    }
  #endif
}


int SDCard::chkMACNode(uint8_t *MACAddress){

  bool matchMAC = true; // Variable to check the MAC match.
  int MACIndex = 0; // Index associated to the position in the file that is the MAC address.
  // IMPORTANT: Each MAC will be related to an identifier of each node relationship to the line on which the MAC is located.
  
  // The SD buffer is initialize.
  memset(bufferFileSD,0x00,sizeof(bufferFileSD));

  // The MAC is converted to char string.
  Utils.hex2str( MACAddress, bufferFileSD, 8); 

  // If it does not have anything written on the first line, it will put the broadcast address.
  if(SD.numln(nameFile)==0){
    SD.appendln(nameFile, "000000000000FFFF");
  }  

  // It is checked if the MAC is registered in the file associated with the address table found in the SD.
  for(int i=0; i<SD.numln(nameFile); i++){
    SD.catln(nameFile, i, 1); 
    matchMAC = true;
    for(uint8_t j=0; j<16; j++){
      if(SD.buffer[j] != bufferFileSD[j]){
        matchMAC = false;
      }
      if(matchMAC == true && j==15){
        MACIndex = i;
      }
    }
  }

  // In the case that the MAC is in the file, the line number, related to the node identifier, is returned.
  if(MACIndex>0){
    return MACIndex;
  }else{  // Otherwise, the address is written to the file and the identifier associated to the line number on which it is located is returned.
    bool statusSD = SD.appendln(nameFile, bufferFileSD);
    if(!statusSD){
      #if DEBUG_SDCARD == 1
        USB.println(F("Error writing the address in the directions file."));
      #endif
      return -1;
    }else{
      return (SD.numln(nameFile));
    }
  }
}
