/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Kenta Yonekura (a.k.a. yoneken)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_STM32_HARDWARE_H_
#define ROS_STM32_HARDWARE_H_

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_uart.h"
#include "stm32f3xx_hal_tim.h"

class STM32Hardware {
  protected:
	TIM_HandleTypeDef *htim;
    UART_HandleTypeDef *huart;

    const uint16_t rbuflen = 128;
    uint8_t rbuf[rbuflen];
    uint32_t rind;
    inline uint32_t getRdmaInd(void){ return (rbuflen - huart->hdmarx->Instance->CNDTR) & (rbuflen - 1); }

    const uint16_t tbuflen = 128;
    uint8_t tbuf[tbuflen];
    uint32_t tind;
    inline uint32_t getTdmaInd(void){ return (tbuflen - huart->hdmatx->Instance->CNDTR) & (tbuflen - 1); }

  public:
    STM32Hardware(TIM_HandleTypeDef *htim_, UART_HandleTypeDef *huart_):
      htim(htim_), huart(huart_), rind(0), tind(0){
    }
  
    void init(){
      HAL_UART_Receive_DMA(huart, rbuf, rbuflen);
      HAL_UART_Transmit_DMA(huart, tbuf, tbuflen);

      HAL_TIM_Base_Start_IT(htim);
    }

    int read(){
      uint8_t c = -1;
      if(rind != getRdmaInd()){
        c = rbuf[rind++];
        rind &= rbuflen - 1;
      }
      return c;
    }

    void write(uint8_t* data, int length){
      int n = length;
      n = n <= tbuflen ? n : tbuflen;

      int n_tail = n <= tbuflen - tind ? n : tbuflen - tind;
      memcpy(&(tbuf[tind]), data, n_tail);
      tind = (tind + n) & (tbuflen - 1);

      if(n != n_tail){
    	memcpy(tbuf, &(data[n_tail]), n - n_tail);
      }
    }

    unsigned long time(){
      return __HAL_TIM_GET_COUNTER(htim);
    }

  protected:
};

#endif

