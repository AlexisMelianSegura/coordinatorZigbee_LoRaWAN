/**
 * \file conversions.h
 * \author Alexis Melian Segura
 * \date 16/08/19
 * \brief Program that define the differents data conversions.
 */

#ifndef _CONVERSIONS_H
#define _CONVERSIONS_H

/****************************************************************************
*                             Includes                                      *
****************************************************************************/

#ifndef __WPROGRAM_H__
  #include "WaspClasses.h"
#endif

/****************************************************************************
*                             Clase                                         *
****************************************************************************/

class Conversions{

public:

  /**
  * \fn Conversions()
  * 
  * Class constructor.
  */
  Conversions();

  /**
  * \fn ~Conversions()
  * 
  * Class destructor.
  */
  ~Conversions();

  /**
   * \fn void unit8t_to_binary (uint8_t a, char * b)
   *
   * \param Hexadecimal number of type uint8_t that has to become to its equivalent in binary.
   * \param Pointer associated to the place in memory where the binary equivalent to the hexadecimal number will be stored.
   *
   * Function that converts a hexadecimal uint8_t to an array to the equivalent in binary (of type char).
   */
  void unit8t_to_binary(uint8_t a, char* b);

  /**
   * \fn void uint16t_to_uint8t (uint16_t a, uint8_t * b)
   * \param Hexadecimal number of type uint16_t.
   * \param Pointer associated to the place in memory where the equivalent of type uint16_t passed by parameters will be stored as a uint8_t value.
   *
   * Function that converts a uint16_t to a pointer of type uint8_t.
   */
  void uint16t_to_uint8t(uint16_t a, uint8_t* b);

  /**
   * \fn void uint32t_to_uint8t (uint32_t a, uint8_t * b)
   * \param Hexadecimal number of type uint32_t.
   * \param Pointer associated to the place in memory where the equivalent of type uint32_t passed by parameters will be stored as a uint8_t value.
   *
   * Function that converts a uint32_t to a pointer of type uint8_t.
   */
  void uint32t_to_uint8t(uint32_t a, uint8_t* b);


  /**
   * \fn void int_to_uint8t (int a, uint8_t * b)
   * \param Integer number of type int.
   * \param Pointer associated to the place in memory where the equivalent of type int passed by parameters will be stored as a uint8_t value.
   *
   * Function that converts an int to a pointer of type uint8_t.
   */
  void int_to_uint8t(int a, uint8_t* b);

  /**
   * \fn char uint8t_to_char (uint8_t a)
   * \param Hexadecimal number of type uint8_t.
   * \retval Hexadecimal equivalent of char type of the number passed by parameters.
   *
   * Function that converts uint8_t to its same value in char.
   */
  char uint8t_to_char(uint8_t a);

  /**
   * \fn uint8_t char_to_uint8t (char * str)
   * \param Hexadecimal value of type char.
   * \retval Hexadecimal equivalent of type uint8_t.
   *
   * Function that converts char to its same value in uint8_t.
   */
  uint8_t char_to_uint8t(char* a);

  /**
   * \fn uint8_t char_to_uint8t (char str1, char str2)
   * \param Most significant hexadecimal value of type char.
   * \param Less significant hexadecimal value of type char.
   * \retval Hexadecimal equivalent of type uint8_t.
   *
   * Function that converts char to its same value in uint8_t.
   */
  uint8_t char_to_uint8t(char str1, char str2);

private:

protected:
 
};

/****************************************************************************
*                             Objects                                       *
****************************************************************************/
extern Conversions conversions_object;


#endif
