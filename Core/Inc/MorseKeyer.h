/*
 * MorseKeyer.h
 *
 *  Created on: Oct 10, 2021
 *      Author: w3dmj
 */

#ifndef INC_MORSEKEYER_H_
#define INC_MORSEKEYER_H_


const struct {
  char     c;
  uint32_t pattern;
  uint8_t  length;
} SYMBOL_LIST[] = {
  {'A', 0xB8000000U, 8U},
  {'B', 0xEA800000U, 12U},
  {'C', 0xEBA00000U, 14U},
  {'D', 0xEA000000U, 10U},
  {'E', 0x80000000U, 4U},
  {'F', 0xAE800000U, 12U},
  {'G', 0xEE800000U, 12U},
  {'H', 0xAA000000U, 10U},
  {'I', 0xA0000000U, 6U},
  {'J', 0xBBB80000U, 16U},
  {'K', 0xEB800000U, 12U},
  {'L', 0xBA800000U, 12U},
  {'M', 0xEE000000U, 10U},
  {'N', 0xE8000000U, 8U},
  {'O', 0xEEE00000U, 14U},
  {'P', 0xBBA00000U, 14U},
  {'Q', 0xEEB80000U, 16U},
  {'R', 0xBA000000U, 10U},
  {'S', 0xA8000000U, 8U},
  {'T', 0xE0000000U, 6U},
  {'U', 0xAE000000U, 10U},
  {'V', 0xAB800000U, 12U},
  {'W', 0xBB800000U, 12U},
  {'X', 0xEAE00000U, 14U},
  {'Y', 0xEBB80000U, 16U},
  {'Z', 0xEEA00000U, 14U},
  {'1', 0xBBBB8000U, 20U},
  {'2', 0xAEEE0000U, 18U},
  {'3', 0xABB80000U, 16U},
  {'4', 0xAAE00000U, 14U},
  {'5', 0xAA800000U, 12U},
  {'6', 0xEAA00000U, 14U},
  {'7', 0xEEA80000U, 16U},
  {'8', 0xEEEA0000U, 18U},
  {'9', 0xEEEE8000U, 20U},
  {'0', 0xEEEEE000U, 22U},
  {'/', 0xEAE80000U, 16U},
  {'?', 0xAEEA0000U, 18U},
  {',', 0xEEAEE000U, 22U},
  {'-', 0xEAAE0000U, 18U},
  {'=', 0xEAB80000U, 16U},
  {'.', 0xBAEB8000U, 20U},
  {' ', 0x00000000U, 4U},
  {0U,  0x00000000U, 0U}
};

const uint8_t BIT_MASK_TABLE[] = {0x80U, 0x40U, 0x20U, 0x10U, 0x08U, 0x04U, 0x02U, 0x01U};

#define WRITE_BIT_FM(p,i,b) p[(i)>>3] = (b) ? (p[(i)>>3] | BIT_MASK_TABLE[(i)&7]) : (p[(i)>>3] & ~BIT_MASK_TABLE[(i)&7])
#define READ_BIT_FM(p,i)    (p[(i)>>3] & BIT_MASK_TABLE[(i)&7])


#endif /* INC_MORSEKEYER_H_ */
