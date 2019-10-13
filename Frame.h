/**
 * \file Frame.h
 * \author Alexis Melian Segura
 * \date 18/08/19
 * \brief Program that define the Frame characteristics.
 */

#ifndef _FRAME_H
#define _FRAME_H

/****************************************************************************
*                             Defines                                      *
****************************************************************************/

/** LoRaWAN network identifier. */
#define LORAWAN_NETWORK 0X01

/** VLC network identifier. */
#define VLC_NETWORK 0X02

/** Zigbee network identifier. */
#define ZIGBEE_NETWORK 0X04


/****************************************************************************
*                            Enumerations                                   *
****************************************************************************/

/** Enumeration of the different plot purposes. */
enum FramePurpouse{
  BOARD_SENSOR = 1,     /** Board Sensor */
  EXTERNAL_SENSOR,  /** External Sensor */
  MAC_REQUEST,      /** MAC Request */
  MAC_ANSWER,       /** MAC Answer */
  ACTUATOR,         /** Actuator Data */
  ZIGBEE_DATA,      /** Zigbee Data */
  VLC_DATA,          /** VLC Data */
  RTC_TIME          /** RTC */
};

#endif
