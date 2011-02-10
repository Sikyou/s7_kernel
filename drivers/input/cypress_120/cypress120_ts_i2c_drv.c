/* Driver for Cypress 120 touch panel device 

 *  Copyright 2010 HUAWEI Tech. Co., Ltd.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  Cypress 120 i2c touch panle driver.
 *
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/system.h>
#include <mach/gpio.h>
#include <linux/pm.h>			/* pm_message_t */
#include <mach/cypress120_ts_dev.h>
#include "ISSP_Routines.h"
#include <mach/msm_rpcrouter.h>

#define CYPRESS120_DEVICE_DUG	0
#if (CYPRESS120_DEVICE_DUG)
#define CYPRESS120_DEV_DUG(format,arg...)		printk(KERN_ALERT format, ## arg); 
#define CYPRESS120_DEV_INFO(format,arg...)		printk(KERN_ALERT format, ## arg); 
#define CYPRESS120_DEV_ERR(format,arg...)		printk(KERN_ALERT format, ## arg); 
#else
#define CYPRESS120_DEV_DUG(format,arg...)		do { (void)(format); } while (0)
#define CYPRESS120_DEV_INFO(format,arg...)		do { (void)(format); } while (0)
#define CYPRESS120_DEV_ERR(format,arg...)		do { (void)(format); } while (0)
#endif

#define   REG_FW_VERSION     		0x03
#define   REG_CON_STATUS	 		0x04
#define   REG_DEEPSLEEP_TIMEOUT	 	0x06
#define   REG_C_FLAG				0x07

#define   REG_POS_X1_H	 	 		0x09
#define   REG_POS_X1_L	 	 		0x0a
#define   REG_POS_Y1_H	 	 		0x0b
#define   REG_POS_Y1_L	 	 		0x0c

#define   REG_POS_X2_H	 	 		0x0d
#define   REG_POS_X2_L	 	 		0x0e
#define   REG_POS_Y2_H	 	 		0x0f
#define   REG_POS_Y2_L	 	 		0x10

#define   REG_FINGER	 	 		0x11
#define   REG_GESTURE	 	 		0x12

#define   TOUCH_SCREEN_WIDTH		800
#define   TOUCH_SCREEN_HIGHT		480
#define BSP_X_SIZE  800
#define BSP_Y_SIZE  480
#define BSP_X_COMPENSATE  16
#define BSP_Y_COMPENSATE  16

#define CYPRESS_PENUP_TIMEOUT_MS 60
static int gfig_num = 0; //touch finger num
static struct cypress120_ts_read_info gcypress_read_info;
static struct timer_list gCypress_Timer;

static void Cypress_TimerHandler(unsigned long data);

struct  i2c_client* cypress120_i2c_client;
static unsigned char g_cypress_dect_flag = 0;

struct cypress120_ts_drive_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int dect_irq;
	struct early_suspend early_suspend;
};

static ssize_t cap_touchscreen_attr_show(struct kobject *kobj, struct kobj_attribute *attr,
        char *buf)
 {
     	  return sprintf(buf, "%d", g_cypress_dect_flag);		  
 }

 static struct kobj_attribute cap_touchscreen_attribute =
         __ATTR(state, 0666, cap_touchscreen_attr_show, NULL);

 static struct attribute* cap_touchscreen_attributes[] =
 {
         &cap_touchscreen_attribute.attr,
         NULL
 };

 static struct attribute_group cap_touchscreen_defattr_group =
 {
         .attrs = cap_touchscreen_attributes,
 };
static struct workqueue_struct *cypress120_wq;

static int cypress120_ts_suspend(struct i2c_client *client, pm_message_t mesg);
static int cypress120_ts_resume(struct i2c_client *client);

static int cypress120_i2c_write(struct i2c_client *client, unsigned char reg, unsigned char data)
{
	int rc;
	
	rc = i2c_smbus_write_byte_data(client, reg, data);
	return rc;
}

static int cypress120_i2c_read(struct i2c_client *client, unsigned char reg)
{
	int rc;	

	rc = i2c_smbus_read_byte_data(client, reg);
	return rc;
}

#define TS_FW_STATE_RPC_PROG	0x30000089
#define TS_FW_STATE_RPC_VERS	0x00010001
#define TS_FW_STATE_READ_ONEPROC  24
#define TS_FW_STATE_WRITE_ONEPROC  25
#define TS_FW_STATE_RPC_TIMEOUT    1000	/* 10 sec 10000*/// 
static int g_ts_fw_state = 0;
static struct msm_rpc_endpoint *ts_fw_state_ep;

static int init_ts_fw_rpc(void)
{
	int rc = 0;
	ts_fw_state_ep =
	    msm_rpc_connect_compatible(TS_FW_STATE_RPC_PROG, TS_FW_STATE_RPC_VERS, 0);

	if (ts_fw_state_ep == NULL)
	{
		printk(KERN_ERR "%s: rpc connect failed .\n", __func__);
		return -ENODEV;
	}
	else if (IS_ERR(ts_fw_state_ep)) 
	{
		rc = PTR_ERR(ts_fw_state_ep);
		printk(KERN_ERR
		       "%s: rpc connect failed ."
		       " rc = %d\n ", __func__, rc);
		ts_fw_state_ep = NULL;
		return rc;
	}
	return 0;
}

static int get_ts_fw_state(void)
{
	struct rpc_request_hdr request_ts_fw_state;
	struct rpc_rep_ts_fw_state
	{
		struct rpc_reply_hdr hdr;
		u32 ts_fw_state;
	} reply_ts_fw_state;

	int rc = 0;
	
	rc = msm_rpc_call_reply(ts_fw_state_ep,
				TS_FW_STATE_READ_ONEPROC,
				&request_ts_fw_state, sizeof(request_ts_fw_state),
				&reply_ts_fw_state, sizeof(reply_ts_fw_state),
				msecs_to_jiffies(TS_FW_STATE_RPC_TIMEOUT));
	if (rc < 0) 
	{
		printk(KERN_ERR "%s(): msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, TS_FW_STATE_READ_ONEPROC, rc);
		return rc;
	} 

	g_ts_fw_state = be32_to_cpu(reply_ts_fw_state.ts_fw_state);  
	printk(KERN_ERR "%s(): msm_rpc_call_reply succeed! proc=%d, ts_fw_state=0x%x, g_ts_fw_state = %d\n",
		       __func__, TS_FW_STATE_READ_ONEPROC,reply_ts_fw_state.ts_fw_state, g_ts_fw_state);
	
	return g_ts_fw_state;
}

static int set_ts_fw_state(void)
{
	struct rpc_request_hdr request_ts_fw_state;
	struct rpc_rep_ts_fw_state
	{
		struct rpc_reply_hdr hdr;
		u32 ts_fw_state;
	} reply_ts_fw_state;
	int rc = 0;
	rc = msm_rpc_call_reply(ts_fw_state_ep,
				TS_FW_STATE_WRITE_ONEPROC,
				&request_ts_fw_state, sizeof(request_ts_fw_state),
				&reply_ts_fw_state, sizeof(reply_ts_fw_state),
				msecs_to_jiffies(TS_FW_STATE_RPC_TIMEOUT));
	if (rc < 0) 
	{
		printk(KERN_ERR "%s(): msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, TS_FW_STATE_READ_ONEPROC, rc);
		return rc;
	} 
	printk(KERN_ERR "%s(): msm_rpc_call_reply succeed! proc=%d, emmc_state=0x%x, g_emmc_state = %d\n",
		       __func__, TS_FW_STATE_READ_ONEPROC,reply_ts_fw_state.ts_fw_state, g_ts_fw_state);
	return 0;
}


static int download_ts_firmware(void)
{
	int ret = 0;
	ret = init_ts_fw_rpc();
	if( ret!= 0)
	{
		CYPRESS120_DEV_ERR("Upgrade touchscreen firmware [init_ts_fw_rpc]error!\n");
	    return 0;
	}
	ret = get_ts_fw_state();
	if(1 == ret )
	{
		CYPRESS120_DEV_ERR("------cypress  ret = get_ts_fw_state() = 1\n");
		if ( IsspProgram() != 0 )
		{
			CYPRESS120_DEV_ERR("Upgrade touchscreen firmware [IsspProgram] error!\n");
		    return -1;
		}
		else
		{
			if ( set_ts_fw_state()!= 0 )
			{
				CYPRESS120_DEV_ERR("Upgrade touchscreen firmware [set_ts_fw_state] error!\n");			    
			}
		}
	}
	else if (0 == ret)
	{
	    CYPRESS120_DEV_ERR("Upgrade touchscreen firmware [get_ts_fw_state] ret = 0!\n");
	}
	else
	{
		CYPRESS120_DEV_ERR("Upgrade touchscreen firmware [get_ts_fw_state] error!\n");
	}
	return 0;	
}

struct cypress120_ts_read_info {
	unsigned char crl_status;
	unsigned char intl_flag;
	unsigned char x1_pos_h;
	unsigned char x1_pos_l;
	unsigned char y1_pos_h;
	unsigned char y1_pos_l;

	unsigned char x2_pos_h;
	unsigned char x2_pos_l;
	unsigned char y2_pos_h;
	unsigned char y2_pos_l;

	unsigned char finger;
	unsigned char gesture;
	
	unsigned short x1_pos;
	unsigned short y1_pos;
	unsigned short x2_pos;
	unsigned short y2_pos;
};

static int cypress120_get_touchinfo(struct cypress120_ts_read_info *pts_info)
{
	int rc = 0;
	CYPRESS120_DEV_DUG("------------enter cypress120_get_touchinfo func !\n");
	rc = i2c_smbus_read_word_data(cypress120_i2c_client,REG_POS_X1_H);
	if(rc < 0)
		goto read_ts_err;
	else
		pts_info->x1_pos = ( (unsigned short)(rc & 0xff )<<8 | (unsigned short)(rc & 0xff00)>>8 );
	CYPRESS120_DEV_DUG("cypress120_get_touchinfo:x1_pos = 0x%04x\n",pts_info->x1_pos);
	rc = i2c_smbus_read_word_data(cypress120_i2c_client,REG_POS_Y1_H);
	if(rc < 0)
		goto read_ts_err;
	else
		pts_info->y1_pos = ( (unsigned short)(rc & 0xff )<<8 | (unsigned short)(rc & 0xff00)>>8 );
	CYPRESS120_DEV_DUG("cypress120_get_touchinfo:y1_pos = 0x%04x\n",pts_info->y1_pos);
	rc = i2c_smbus_read_word_data(cypress120_i2c_client,REG_POS_X2_H);
	if(rc < 0)
		goto read_ts_err;
	else
		pts_info->x2_pos = ( (unsigned short)(rc & 0xff )<<8 | (unsigned short)(rc & 0xff00)>>8 );
	CYPRESS120_DEV_DUG("cypress120_get_touchinfo:x2_pos = 0x%04x\n",pts_info->x2_pos);
	rc = i2c_smbus_read_word_data(cypress120_i2c_client,REG_POS_Y2_H);
	if(rc < 0)
		goto read_ts_err;
	else
		pts_info->y2_pos = ( (unsigned short)(rc & 0xff )<<8 | (unsigned short)(rc & 0xff00)>>8 );
	CYPRESS120_DEV_DUG("cypress120_get_touchinfo:y2_pos = 0x%04x\n",pts_info->y2_pos);
	rc = cypress120_i2c_read(cypress120_i2c_client,REG_FINGER);
	if(rc < 0)
		goto read_ts_err;
	else
		pts_info->finger = (unsigned char)(rc & 0xff);
	CYPRESS120_DEV_DUG("cypress120_get_touchinfo:finger = 0x%02x\n",pts_info->finger);
	rc = cypress120_i2c_read(cypress120_i2c_client,REG_GESTURE);
	if(rc < 0)
		goto read_ts_err;
	else
		pts_info->gesture = (unsigned char)(rc & 0xff);
	CYPRESS120_DEV_DUG("cypress120_get_touchinfo:gesture = 0x%02x\n",pts_info->gesture);
	return rc;
read_ts_err:
	CYPRESS120_DEV_ERR("cypress120_get_touchinfo: read data err! errcode=%d\n",rc);
	memset(pts_info,0,sizeof(struct cypress120_ts_read_info));
	return rc;
}

#define  MUTI_TOUCH_SUPPORT 1
static void cypress120_report_event(struct cypress120_ts_drive_data *ddata, 
	struct cypress120_ts_read_info* pTSEventInfo)
{

	static int x1_last = 0, y1_last = 0, x2_last = 0, y2_last = 0; 
	unsigned short position[2][2]={0,0,0,0}; 

#if(MUTI_TOUCH_SUPPORT)
	if(pTSEventInfo->finger == 1) //one finger touch 
	{

#if 0
		position[0][0] = pTSEventInfo->x1_pos;
		position[0][1] = pTSEventInfo->y1_pos;
#else
        position[0][0] = pTSEventInfo->x1_pos + (BSP_X_SIZE - 2*(pTSEventInfo->x1_pos))*BSP_X_COMPENSATE/BSP_X_SIZE;
        position[0][1] = pTSEventInfo->y1_pos + (BSP_Y_SIZE - 2*(pTSEventInfo->y1_pos))*BSP_Y_COMPENSATE/BSP_Y_SIZE;
#endif

        
        input_report_abs(ddata->input_dev, ABS_MT_TOUCH_MAJOR, 255);
		input_report_abs(ddata->input_dev, ABS_MT_POSITION_X, position[0][0]);
		input_report_abs(ddata->input_dev, ABS_MT_POSITION_Y, position[0][1]);
		input_mt_sync(ddata->input_dev);
		input_sync(ddata->input_dev); 

		CYPRESS120_DEV_ERR("----------Cypress report event: One Finger Press! x1=%d,y1=%d \n", position[0][0],position[0][1]);
		if(gfig_num == 0) //first scan 
		{
			gfig_num = 1;
			x1_last = pTSEventInfo->x1_pos;
			y1_last = pTSEventInfo->y1_pos;
			CYPRESS120_DEV_ERR("----------Cypress report event: One Finger Start Press scan! x1_last=%d,y1_last=%d \n", x1_last,y1_last);
			CYPRESS120_DEV_DUG("-----------------------------------------------------------\n\n");
			return;
		}
		CYPRESS120_DEV_ERR("-----------------------------------------------------------\n\n");

	}
	else if(pTSEventInfo->finger == 2)//two finger touch
	{
		position[0][0] = pTSEventInfo->x1_pos;
		position[0][1] = pTSEventInfo->y1_pos;
		position[1][0] = pTSEventInfo->x2_pos;
		position[1][1] = pTSEventInfo->y2_pos;


        input_report_abs(ddata->input_dev, ABS_MT_TOUCH_MAJOR, 255);
		input_report_abs(ddata->input_dev, ABS_MT_POSITION_X, position[0][0]);
		input_report_abs(ddata->input_dev, ABS_MT_POSITION_Y, position[0][1]);
		input_mt_sync(ddata->input_dev);
		
		input_report_abs(ddata->input_dev, ABS_MT_TOUCH_MAJOR, 255);
		input_report_abs(ddata->input_dev, ABS_MT_POSITION_X, position[1][0]);
		input_report_abs(ddata->input_dev, ABS_MT_POSITION_Y, position[1][1]);
		input_mt_sync(ddata->input_dev);
		input_sync(ddata->input_dev);
		
		CYPRESS120_DEV_ERR("----------Cypress report event: Two Finger Press! x1=%d,y1=%d \n", 
			position[0][0],position[0][1]);
		CYPRESS120_DEV_ERR("----------Cypress report event: Two Finger Press! x2=%d,y2=%d \n",
			position[1][0],position[1][1]);

		if(gfig_num == 0) //first scan
		{
			gfig_num = 2;
			x1_last = pTSEventInfo->x1_pos;
			y1_last = pTSEventInfo->y1_pos;
			x2_last = pTSEventInfo->x2_pos;
			y2_last = pTSEventInfo->y2_pos;

			CYPRESS120_DEV_DUG("----------Cypress report event: Two Finger Start Press scan! x1_last=%d,y1_last=%d,x2_last=%d,y2_last=%d \n", x1_last,y1_last,x2_last,y2_last);
			CYPRESS120_DEV_DUG("-----------------------------------------------------------\n\n");
			return;
		}
		CYPRESS120_DEV_ERR("-----------------------------------------------------------\n\n");
	}
#else
	else if((pTSEventInfo->finger == 1) || (pTSEventInfo->finger == 2)) //one finger touch 
	{
		position[0][0] = pTSEventInfo->x1_pos;
		position[0][1] = pTSEventInfo->y1_pos;
		position[1][0] = pTSEventInfo->x2_pos;
		position[1][1] = pTSEventInfo->y2_pos;

		if(gfig_num == 0) //first scan
		{
			CYPRESS120_DEV_DUG("-----------cypress scan: first scan\n");
			gfig_num = pTSEventInfo->finger;
			input_report_abs(ddata->input_dev, ABS_PRESSURE, 255);
			input_report_abs(ddata->input_dev, ABS_X, position[0][0]);
			input_report_abs(ddata->input_dev, ABS_Y, position[0][1]);					 	 
			input_report_abs(ddata->input_dev, ABS_TOOL_WIDTH, 1);
			input_report_key(ddata->input_dev, BTN_TOUCH, 1);
			input_sync(ddata->input_dev);
			x1_last = position[0][0];
			y1_last = position[0][1];
		}
		else 
		{
			CYPRESS120_DEV_ERR("-----------cypress scan: curx1 = %d, cury1 = %d, x1_last = %d, y1_last = %d \n",position[0][0],position[0][1],x1_last,y1_last);
			if (((position[0][0]-x1_last) >= 5) || ((x1_last-position[0][0]) >= 5) \			
				|| ((position[0][1]-y1_last) >= 5) || ((y1_last-position[0][1]) >= 5)) //valid touch motion
			{
					input_report_abs(ddata->input_dev, ABS_PRESSURE, 255);
					input_report_abs(ddata->input_dev, ABS_X, position[0][0]);
					input_report_abs(ddata->input_dev, ABS_Y, position[0][1]);					 	 
					input_report_abs(ddata->input_dev, ABS_TOOL_WIDTH, 1);
					input_report_key(ddata->input_dev, BTN_TOUCH, 1);
					input_sync(ddata->input_dev);
					x1_last = position[0][0];
					y1_last = position[0][1];
			}
		}
	}
#endif
	if(pTSEventInfo->gesture > GESTURE_NO_GESTURE)
	{
		input_report_gesture(ddata->input_dev, pTSEventInfo->gesture ,0);
		input_sync(ddata->input_dev);
		CYPRESS120_DEV_ERR("---------cypress report 0x%02x gesture!\n",pTSEventInfo->gesture);
	}
	x1_last = position[0][0];
	y1_last = position[0][1];
	x2_last = position[1][0];
	y2_last = position[1][1];
}


static void Cypress_TimerHandler(unsigned long data)
{
    struct cypress120_ts_drive_data *ddata = (struct cypress120_ts_drive_data *)data;
    
	if((gcypress_read_info.finger == 0) && (gfig_num > 0))
	{
		CYPRESS120_DEV_ERR("----------Cypress report event: Press Release! \n");
		input_report_abs(ddata->input_dev, ABS_MT_TOUCH_MAJOR,255 );
		input_sync(ddata->input_dev);
		gfig_num = 0;
		CYPRESS120_DEV_DUG("-----------------------------------------------------------\n\n");
	}
}

static void cypress120_ts_work_func(struct work_struct *work)
{
	struct cypress120_ts_drive_data *ddata = container_of(work, struct cypress120_ts_drive_data, work);
	int rc = 0;

	CYPRESS120_DEV_DUG("------------enter cypress120_ts_work_func func !\n");

	rc = cypress120_get_touchinfo(&gcypress_read_info);
	if(rc < 0)
		CYPRESS120_DEV_ERR("cypress120_ts_work_func:cypress120_get_touchinfo return err code %d!\n",rc);
	//clear reg07 c_flag
	rc = cypress120_i2c_write(cypress120_i2c_client,REG_C_FLAG,0x01);
	CYPRESS120_DEV_DUG("cypress120_ts_work_func: cypress120_i2c_client=0x%0x \n",cypress120_i2c_client);
	CYPRESS120_DEV_DUG("cypress120_ts_work_func:cypress120_i2c_txdata write Reg07 return %d!\n",rc);
	enable_irq(ddata->client->irq);
	
	cypress120_report_event(ddata,&gcypress_read_info);	

    mod_timer(&gCypress_Timer, jiffies + msecs_to_jiffies(CYPRESS_PENUP_TIMEOUT_MS));
	
}

static irqreturn_t cypress120_ts_irq_handler(int irq, void *dev_id)
{
	struct cypress120_ts_drive_data *ddata = dev_id;
	CYPRESS120_DEV_ERR("---------------Enter cypress120_ts_irq_handler: \n");
	disable_irq(ddata->client->irq);
	queue_work(cypress120_wq, &ddata->work);

	return IRQ_HANDLED;
}


static int cypress120_ts_dev_open(struct input_dev *dev)
{
	struct cypress120_ts_drive_data *ddata = input_get_drvdata(dev);
	int rc;
	CYPRESS120_DEV_DUG("---------------Enter cypress120_ts_dev_open: \n");
	if (!ddata->client) {
		CYPRESS120_DEV_ERR("cypress120_ts_dev_open: no i2c adapter present\n");
		rc = -ENODEV;
		return rc;
	}

	CYPRESS120_DEV_DUG("---------------cypress120_ts_dev_open: request cypress int gpio succeed! \n");
	
	ddata->dect_irq = MSM_GPIO_TO_INT(CYPRESS_DECT_INT_GPIO);

	rc = request_irq(ddata->client->irq,
		       &cypress120_ts_irq_handler,
		       IRQF_TRIGGER_FALLING,
		       "Cypress120 dect int ",
		       ddata);
	if (rc) {
		CYPRESS120_DEV_ERR("cypress120_ts_dev_open: FAILED: request_irq rc=%d\n", rc);
		return rc;
	}

	CYPRESS120_DEV_DUG("---------------cypress120_ts_dev_open: request chg irq succeed\n");
	return 0;
}

static void cypress120_ts_dev_close(struct input_dev *dev)
{
	struct cypress120_ts_drive_data *ddata = input_get_drvdata(dev);

	CYPRESS120_DEV_DUG("---------------Enter cypress120_ts_dev_close: \n");
	if (ddata->dect_irq)
	{
		free_irq(ddata->dect_irq, ddata);
		ddata->dect_irq = 0;
	}
	cancel_work_sync(&ddata->work);
}

static int cypress120_ts_gpio_reset(void)
{
	gpio_set_value(CYPRESS_RST_GPIO, 0);
	udelay(10);
	gpio_set_value(CYPRESS_RST_GPIO, 1);
	mdelay(5);
	gpio_set_value(CYPRESS_RST_GPIO, 0);
   	mdelay(15);     
	CYPRESS120_DEV_DUG("---------------Enter cypress120_ts_gpio_reset success--: \n");
	return 0;
}

static int cypress_gpio_request(void)
{
	int rc = 0;
	//--------------1. request interrupt gpio,alloc and enable interrupt gpio-------------
	struct msm_gpio cypress_dect_gpio = { GPIO_CFG(CYPRESS_DECT_INT_GPIO, 0, GPIO_INPUT, 
			GPIO_PULL_UP, GPIO_2MA),"cypress_dect_irq_test" };
	rc = msm_gpios_request_enable(&cypress_dect_gpio,1);
	//---------------2. request reset gpio---------------------
	rc = gpio_request(CYPRESS_RST_GPIO, " CYPRESS_RST_GPIO RESET Enable");
	if (rc) 
	{
		msm_gpios_disable_free(&cypress_dect_gpio,1);
		CYPRESS120_DEV_ERR("-----------cypress120_ts_gpio_reset: CYPRESS_TS RESET request failed,err code %d\n",rc);
		return rc;
	}
	//--------------3.config reset gpio-------------------
	rc = gpio_tlmm_config(GPIO_CFG(CYPRESS_RST_GPIO, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
	if(rc < 0)
	{
		CYPRESS120_DEV_ERR("-----------cypress120_ts_gpio_reset: gpio_tlmm_config failed,err code %d\n",rc);
		gpio_free(CYPRESS_RST_GPIO);
		msm_gpios_disable_free(&cypress_dect_gpio,1);
		return rc;
	}
	return rc;
}

static void cypress_gpio_release(void)
{
	struct msm_gpio cypress_dect_gpio = { GPIO_CFG(CYPRESS_DECT_INT_GPIO, 0, GPIO_INPUT, 
			GPIO_PULL_UP, GPIO_2MA),"cypress_dect_irq_test" };
	msm_gpios_disable_free(&cypress_dect_gpio,1);
	gpio_free(CYPRESS_RST_GPIO);   
}


static int cypress120_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cypress120_ts_drive_data *ddata = NULL;
	int ret = 0;
	int icount = 0;
	unsigned char ts_status = 0;
	struct kobject *kobj = NULL;
	CYPRESS120_DEV_DUG("---------------Enter cypress120_ts_probe: \n");
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CYPRESS120_DEV_ERR("cypress120_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	download_ts_firmware();
	kobj = kobject_create_and_add("cap_touchscreen", NULL);
  	if (kobj == NULL) {	
		CYPRESS120_DEV_ERR("-------kobject_create_and_add error  kobj==NULL----------- \n" );
		return -1;
	}
  	if (sysfs_create_group(kobj, &cap_touchscreen_defattr_group)) {
		kobject_put(kobj);
		CYPRESS120_DEV_ERR("-------sysfs_create_group---kobject_put(kobj)-------- \n" );
		return -1;
	}
	CYPRESS120_DEV_ERR("------create sys kobject SUCCESS!-------- \n" );

	ret = cypress_gpio_request();
	if (ret) {
		CYPRESS120_DEV_ERR("------------------------cypress120_ts_probe:Failed to request GPIO pin %d (ret=%d)\n",CYPRESS_DECT_INT_GPIO, ret);
		goto err_gpio_request;
	}
	else
	{
		CYPRESS120_DEV_ERR("-----------------------cypress120_ts_probe: succeed request GPIO pin %d \n",
		       CYPRESS_DECT_INT_GPIO);
	}
	CYPRESS120_DEV_DUG("---------------cypress120_ts_gpio_reset: request RST gpio succeed \n");

	cypress120_ts_gpio_reset();
	cypress120_i2c_client = client;	
	ts_status = (1<<3);
	ret = cypress120_i2c_write(cypress120_i2c_client,REG_CON_STATUS,ts_status);
	cypress120_ts_gpio_reset();

	if(!g_cypress_dect_flag)
	{
		unsigned char hw_ver = 0;
		ret = cypress120_i2c_read(cypress120_i2c_client,REG_FW_VERSION);
		if(ret < 0)
			goto err_read_vendor;
		else
			hw_ver = (unsigned char)(ret & 0xff);
		g_cypress_dect_flag = 1;
	}
	

	cypress120_wq = create_singlethread_workqueue("cypress120_wq");
	if (!cypress120_wq) {
		CYPRESS120_DEV_ERR("create cypress120_wq error\n");
		goto err_creat_workqueue_fail;
	}
	ddata = kzalloc(sizeof(struct cypress120_ts_drive_data), GFP_KERNEL);
	if (ddata == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ddata->client = client;
	i2c_set_clientdata(client, ddata);
	INIT_WORK(&ddata->work, cypress120_ts_work_func);

	ddata->input_dev = input_allocate_device();
	if (ddata->input_dev == NULL) {
		ret = -ENOMEM;
		CYPRESS120_DEV_ERR("cypress120_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	CYPRESS120_DEV_ERR("---------------cypress120_ts_probe: input_allocate_device succeed\n");
	input_set_drvdata(ddata->input_dev, ddata);
	ddata->input_dev->open       	= cypress120_ts_dev_open;
	ddata->input_dev->close      	= cypress120_ts_dev_close;
	ddata->input_dev->name       	= "cypress120-i2c-touchscreen";
	ddata->input_dev->phys       	= "dev/cypress120_ts";
	ddata->input_dev->id.bustype 	= BUS_I2C;
	ddata->input_dev->id.product 	= 1;
	ddata->input_dev->id.version 	= 1;

	set_bit(EV_SYN, ddata->input_dev->evbit);
	set_bit(EV_KEY, ddata->input_dev->evbit);
	set_bit(BTN_TOUCH, ddata->input_dev->keybit);	
	set_bit(EV_ABS, ddata->input_dev->evbit);	
	
	input_set_abs_params(ddata->input_dev, ABS_X, 0, TOUCH_SCREEN_WIDTH, 0, 0);
	input_set_abs_params(ddata->input_dev, ABS_Y, 0, TOUCH_SCREEN_HIGHT, 0, 0); 
	input_set_abs_params(ddata->input_dev, ABS_MT_TOUCH_MAJOR, 0, 64, 0, 0);
	input_set_abs_params(ddata->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	input_set_abs_params(ddata->input_dev, ABS_MT_POSITION_X, 0, TOUCH_SCREEN_WIDTH, 0, 0);
	input_set_abs_params(ddata->input_dev, ABS_MT_POSITION_Y, 0, TOUCH_SCREEN_HIGHT, 0, 0);
	
	set_bit(EV_GESTURE, ddata->input_dev->evbit);
	for (icount = GESTURE_NO_GESTURE; icount < GESTURE_MAX; icount++)
		set_bit(icount, ddata->input_dev->gesturebit);	
	
	ret = input_register_device(ddata->input_dev);
	if (ret) {
		CYPRESS120_DEV_ERR("cypress120_ts_probe: Unable to register %s input device\n", ddata->input_dev->name);
		goto err_input_register_device_failed;
	}   

	ddata->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	ddata->early_suspend.suspend = cypress120_ts_suspend;
	ddata->early_suspend.resume = cypress120_ts_resume;
	register_early_suspend(&ddata->early_suspend);

    memset(&gcypress_read_info,0,sizeof(struct cypress120_ts_read_info));
    setup_timer(&gCypress_Timer, Cypress_TimerHandler, (unsigned long)ddata);
	CYPRESS120_DEV_ERR("---------------cypress120_ts_probe: input_register_device succeed\n");	
	return 0;
err_input_register_device_failed:                //step 8
	input_free_device(ddata->input_dev);
err_input_dev_alloc_failed:                      //step 7
	kfree(ddata);
err_alloc_data_failed:                           //step 6
	destroy_workqueue(cypress120_wq);
err_creat_workqueue_fail:                            //step 5
err_read_vendor:                                     //step 4
	CYPRESS120_DEV_ERR("---------------read vendor error \n" );
err_gpio_request:                                   //step 2
	cypress_gpio_release();
err_check_functionality_failed:
	CYPRESS120_DEV_ERR("---------------cypress120_ts_probe FAILED\n");
	return ret;
}

static int cypress120_ts_remove(struct i2c_client *client)
{
	struct cypress120_ts_drive_data *ddata = i2c_get_clientdata(client);

	CYPRESS120_DEV_DUG("---------------Enter cypress120_ts_remove: \n");
	g_cypress_dect_flag = 0;
	if (ddata->dect_irq)
	{
		free_irq(ddata->dect_irq, ddata);
		ddata->dect_irq = 0;
	}

    del_timer_sync(&gCypress_Timer);
    
	cypress_gpio_release();
	input_unregister_device(ddata->input_dev);
	input_free_device(ddata->input_dev);
	kfree(ddata);
	return 0;
}

#if 1//ifndef CONFIG_PM
static int cypress120_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int rc = 0;
	unsigned char ts_status = 0;

	struct cypress120_ts_drive_data *ddata = i2c_get_clientdata(cypress120_i2c_client);
	

	// disable irq
	disable_irq( cypress120_i2c_client->irq );
	cancel_work_sync(&ddata->work);
//	return rc;
 	rc = cypress120_ts_gpio_reset();
	if(rc < 0)
	{
		CYPRESS120_DEV_ERR("cypress120_ts_suspend: rst gpio reset failed! err is %d! \n",rc );
		return rc;
	}
	rc = cypress120_i2c_write(cypress120_i2c_client,REG_DEEPSLEEP_TIMEOUT,0x00);
	if(rc < 0)
	{
		CYPRESS120_DEV_ERR("cypress120_ts_suspend: cypress120_i2c_write failed! err = %d! \n",rc);
		return rc;		
	}
	rc = cypress120_i2c_read(cypress120_i2c_client,REG_CON_STATUS);
	if(rc < 0)
		return rc;
	else
		ts_status = rc | (1<<4) | (1<<3);

	rc = cypress120_i2c_write(cypress120_i2c_client,REG_CON_STATUS,ts_status);
	if(rc < 0)
		return rc;
	CYPRESS120_DEV_ERR("---------------Exit cypress120_ts_suspend \n" );
	return 0;
}

static int cypress120_ts_resume(struct i2c_client *client)
{
	int rc = 0;
	CYPRESS120_DEV_ERR("---------------Enter cypress120_ts_resume: \n");
	rc = cypress120_ts_gpio_reset();
	if(rc < 0)
	{
		CYPRESS120_DEV_ERR("cypress120_ts_suspend: rst gpio reset failed! err code = %d! \n",rc);
		return rc;
	}
	enable_irq( cypress120_i2c_client->irq );
	return 0;
}
#endif

static const struct i2c_device_id  cypress120_ts_id[] = {
	{ CYPRESS120_I2C_TS_NAME, 0 },
	{ }
};

static struct i2c_driver cypress120_ts_driver = {
	.probe		= cypress120_ts_probe,
	.remove		= cypress120_ts_remove,
	.id_table	= cypress120_ts_id,
	.driver = {
		.name	= CYPRESS120_I2C_TS_NAME,
	},
};

static int __devinit cypress120_ts_init(void)
{
    CYPRESS120_DEV_DUG("---------------Enter cypress120_ts_init: \n");
    return i2c_add_driver(&cypress120_ts_driver);
}

static void __exit cypress120_ts_exit(void)
{
    CYPRESS120_DEV_DUG("---------------Enter cypress120_ts_exit: \n");
   
    i2c_del_driver(&cypress120_ts_driver);
    if (cypress120_wq)
    {
        destroy_workqueue(cypress120_wq);
    }
}

module_init(cypress120_ts_init);
module_exit(cypress120_ts_exit);


MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("0.1");
MODULE_AUTHOR("HUAWEI Tech. Co., Ltd . wufan 00163571");
MODULE_DESCRIPTION("Cypress120 I2c Touchscreen Driver");
MODULE_ALIAS("platform:cypress120-ts");

