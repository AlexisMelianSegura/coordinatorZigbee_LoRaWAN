/**
 * \file conversions.cpp
 * \author Alexis Melian Segura
 * \date 16/08/19
 * \brief Program that define the differents data conversions.
 */

/****************************************************************************
*                             Includes                                     *
****************************************************************************/

#include "Conversions.h"


/****************************************************************************
*                             Objects                                       *
****************************************************************************/

Conversions  conversions_object = Conversions();

/****************************************************************************
*                             Functions                                     *
****************************************************************************/

Conversions::Conversions() {
}


Conversions::~Conversions() {
}


void Conversions::unit8t_to_binary(uint8_t a, char* b ){
    uint8_t i = 0;
    uint8_t j = 0;
    
    for(i=0x80;i!=0;i>>=1){
      b[j] = (a&i)?'1':'0';
      j++; 
     }
}


void Conversions::uint16t_to_uint8t(uint16_t a, uint8_t* b){
  *b = (uint8_t)((a & 0xFF00) >> 8);
  *(b+1) = (uint8_t)(a & 0x00FF);
}


void Conversions::uint32t_to_uint8t(uint32_t a, uint8_t* b){
  *b = (uint8_t)((a & 0xFF000000) >> 24);
  *(b+1) = (uint8_t)((a & 0x00FF0000) >> 16);
  *(b+2) = (uint8_t)((a & 0x0000FF00) >> 8);
  *(b+3) = (uint8_t)(a & 0x000000FF);
}


void Conversions::int_to_uint8t(int a, uint8_t* b){
  char aux[2];
  int i=1;
  int j=3;
  memset(aux,'0',sizeof(aux));
  memset(b,0x00,4);
  // If it is negative, it becomes positive. The negative value will be indicated in the frame to be sent.
  if(a<0){
    a = -a;
  }
  do{
     aux[i] = uint8t_to_char(a%16);
     a/=16;
     i--;
     if(i<0){ // The index of the char array is reset.
      b[j]=char_to_uint8t(aux);
      memset(aux,'0',sizeof(aux));
      j--;
      i=1;
     }
  }while((a)>16);
  aux[i] = uint8t_to_char(a);
   b[j]=char_to_uint8t(aux);
}


uint8_t Conversions::char_to_uint8t(char* str){
    int aux=0, aux2=0;

    if( (*str>='0') && (*str<='9') ){
        aux=*str++-'0';
    }else if( (*str>='A') && (*str<='F') ){
        aux=*str++-'A'+10;
    }else if( (*str>='a') && (*str<='f') ){
        aux=*str++-'a'+10;
    }
    
    if( (*str>='0') && (*str<='9') ){
        aux2=*str-'0';
    }else if( (*str>='A') && (*str<='F') ){
        aux2=*str-'A'+10;
    }else if( (*str>='a') && (*str<='f') ){
        aux2=*str-'a'+10;
    }

    return aux*16+aux2;
}

uint8_t Conversions::char_to_uint8t(char str1, char str2){
    int aux=0, aux2=0;

    if( (str1>='0') && (str1<='9') ){
        aux=str1-'0';
    }else if( (str1>='A') && (str1<='F') ){
        aux=str1-'A'+10;
    }else if( (str1>='a') && (str1<='f') ){
        aux=str1-'a'+10;
    }
    
    if( (str2>='0') && (str2<='9') ){
        aux2=str2-'0';
    }else if( (str2>='A') && (str2<='F') ){
        aux2=str2-'A'+10;
    }else if( (str2>='a') && (str2<='f') ){
        aux2=str2-'a'+10;
    }
    return aux*16+aux2;
}


char Conversions::uint8t_to_char(uint8_t a){
  switch(a){
    case 0:
      return '0';
    break;
    case 1:
      return '1';
    break;
    case 2:
      return '2';
    break;
    case 3:
      return '3';
    break;
    case 4:
      return '4';
    break;
    case 5:
      return '5';
    break;
    case 6:
      return '6';
    break;
    case 7:
      return '7';
    break;
    case 8:
      return'8';
    break;
    case 9:
      return '9';
    break;
    case 10:
      return 'A';
    break;
    case 11:
      return 'B';
    break;
    case 12:
      return 'C';
    break;
    case 13:
      return 'D';
    break;
    case 14:
      return 'E';
    break;
    case 15:
      return 'F';
    break;  
  }
}
