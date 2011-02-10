#ifndef __USB_SWITCH_HUAWEI_H__
#define __USB_SWITCH_HUAWEI_H__


#define USB_AUTO_DEBUG

#define ADB_FLOW_QUERY_TIME                 5
#define ADB_INACTIVE_THRESHHOLD             2

#define ADB_REACTIVATE_EXPIRE_TIME          120

#define CHG_SUSP_WORK_DELAY            msecs_to_jiffies(100)

typedef  struct _usbsdms_read_toc_cmd_type
{
  u8  op_code;  
  u8  msf;             /* bit1 is MSF, 0: address format is LBA form
			  1: address format is MSF form */
  u8  format;          
  u8  reserved1;  
  u8  reserved2;  
  u8  reserved3;  
  u8  session_num;     /* a specific session or a track */
  u8  allocation_length_msb;
  u8  allocation_length_lsb;
  u8  control;
} usbsdms_read_toc_cmd_type;

typedef struct _scsi_rewind_cmd_type
{
  u8 cmd;
  u8 ind;
  u8 os_type;
  u8 time_to_delay;
  u8 pidh;
  u8 pidl;
} scsi_rewind_cmd_type;

#define SC_REWIND               0x01
#define SC_REWIND_11            0x11

#define OS_TYPE_MASK            0xf0
#define OS_TYPE_WINDOWS         0x00
#define OS_TYPE_WIN98           0x01
#define OS_TYPE_WIN2K           0x02
#define OS_TYPE_WINXP           0x03
#define OS_TYPE_VISTA32         0x04
#define OS_TYPE_VISTA64         0x05
#define OS_TYPE_MAC             0x10
#define OS_TYPE_LINUX           0x20
#define OS_TYPE_GATEWAY         0x30

#define USB_SERIAL_KEY_SIZE 16


typedef struct _app_usb_para
{
  unsigned usb_pid_index;
  unsigned usb_pid;
}app_usb_para;

typedef struct _usb_pid_stru
{
  u16     cdrom_pid;
  u16     norm_pid;
  u16     udisk_pid;
}usb_pid_stru;

extern usb_pid_stru *curr_usb_pid_ptr;

typedef struct _usb_switch_stru
{
  u16     dest_pid;       /* destination usb pid */
  u16     inprogress;     /* 1: inprogress, 0: finished */
}usb_switch_stru;

typedef struct _adb_io_stru
{
  int     read_num;
  int     write_num;
  u8      active;     /* 1: active, 0: inactive*/
  u8      query_num;
}adb_io_stru;


#ifdef USB_AUTO_DEBUG
#define USB_PR(fmt, ...) \
  printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
#else
#define USB_PR(fmt, ...) 
#endif


u16 pid_index_to_pid(u32 pid_index);

#endif  /* __USB_SWITCH_HUAWEI_H__ */
