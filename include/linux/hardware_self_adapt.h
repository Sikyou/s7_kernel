#ifndef HARDWARE_SELF_ADAPT_H
#define HARDWARE_SELF_ADAPT_H

typedef enum
{
    LCD_S6D74A0_SAMSUNG_HVGA,
    LCD_ILI9325_INNOLUX_QVGA,
    LCD_ILI9325_BYD_QVGA,
    LCD_ILI9325_WINTEK_QVGA,
    LCD_SPFD5408B_KGM_QVGA,

    LCD_HX8357A_BYD_QVGA,
    LCD_HX8368A_SEIKO_QVGA,

    LCD_HX8347D_TRULY_QVGA,
    LCD_HX8347D_INNOLUX_QVGA,
    LCD_ILI9325C_WINTEK_QVGA,    
    
    LCD_HX8368A_TRULY_QVGA,
    LCD_HX8357A_TRULY_HVGA,
    LCD_HX8357A_WINTEK_HVGA,
	
    LCD_MAX_NUM,
    LCD_NONE =0xFF
}lcd_panel_type;

typedef enum
{
    HW_VER_SUB_VA            = 0x0,
    HW_VER_SUB_VB            = 0x1,
    HW_VER_SUB_VC            = 0x2,
    HW_VER_SUB_VD            = 0x3,
    HW_VER_SUB_VE            = 0x4,
    HW_VER_SUB_SURF          = 0xF,
    HW_VER_SUB_MAX           = 0xF
}hw_ver_sub_type;

typedef enum
{
	GS_ADIX345 	= 0x01,
	GS_ST35DE	= 0x02,
	GS_ST303DLH = 0X03
}hw_gs_type;
	
#define HW_VER_MAIN_MASK (0xFFF0)
#define HW_VER_SUB_MASK  (0x000F)

lcd_panel_type lcd_panel_probe(void);
int board_use_tssc_touch(bool * use_touch_key);

int board_support_ofn(bool * ofn_support);

bool st303_gs_is_supported(void);
void set_st303_gs_support(bool status);

unsigned int board_support_bcm_wifi(unsigned *p_gpio);

char *get_lcd_panel_name(void);

hw_ver_sub_type get_hw_sub_board_id(void);

#endif

