/*
 * Copyright 2010 HUAWEI Tech. Co., Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * IRDA remote control driver
 *
 */

#ifndef _IRDA_CTRL_GPIO_H_
#define _IRDA_CTRL_GPIO_H_
#define RMT_CTRL_GPIO_IRQ             			(40)

#define IRDA_GPIO_INVERTED 1  
struct rmt_ctrl_gpio_platform_data {
	int irq_gpio_pin;
	int (*gpio_setup)(void);
	void (*gpio_release)(void);
};

#define RMT_CTRL_GPIO_DEVICE_NAME                    "irda_rmt_ctrl_gpio"

//remote control dug msg control
#define RMT_CTRL_GPIO_ERR(format,arg...)   printk(KERN_ALERT format, ## arg); 

#define _RMT_CTRL_GPIO_DEBUG_    0
#if(_RMT_CTRL_GPIO_DEBUG_)
//#define RMT_CTRL_GPIO_DEG(format,arg...)   printk(KERN_ALERT format, ## arg); 
#define RMT_CTRL_GPIO_DEG(format,arg...)   do { (void)(format); } while (0)
#else
#define RMT_CTRL_GPIO_DEG(format,arg...)   do { (void)(format); } while (0)
#endif

#endif




