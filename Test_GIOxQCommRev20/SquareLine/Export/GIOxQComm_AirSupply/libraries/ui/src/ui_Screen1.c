// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 9.1.0
// Project name: GIOxQComm_AirSupply

#include "ui.h"

void ui_Screen1_screen_init(void)
{
ui_Screen1 = lv_obj_create(NULL);
lv_obj_remove_flag( ui_Screen1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Image2 = lv_image_create(ui_Screen1);
lv_image_set_src(ui_Image2, &ui_img_ancestry_6927683_png);
lv_obj_set_width( ui_Image2, LV_SIZE_CONTENT);  /// 512
lv_obj_set_height( ui_Image2, LV_SIZE_CONTENT);   /// 512
lv_obj_set_x( ui_Image2, 120 );
lv_obj_set_y( ui_Image2, -28 );
lv_obj_set_align( ui_Image2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Image2, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_remove_flag( ui_Image2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_image_set_scale(ui_Image2,50);

ui_Image3 = lv_image_create(ui_Screen1);
lv_image_set_src(ui_Image3, &ui_img_wifi_2803371_png);
lv_obj_set_width( ui_Image3, LV_SIZE_CONTENT);  /// 512
lv_obj_set_height( ui_Image3, LV_SIZE_CONTENT);   /// 512
lv_obj_set_x( ui_Image3, 150 );
lv_obj_set_y( ui_Image3, -140 );
lv_obj_set_align( ui_Image3, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Image3, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_remove_flag( ui_Image3, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_image_set_scale(ui_Image3,10);

ui_lblPM25 = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_lblPM25, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lblPM25, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lblPM25, -81 );
lv_obj_set_y( ui_lblPM25, 29 );
lv_obj_set_align( ui_lblPM25, LV_ALIGN_CENTER );
lv_label_set_text(ui_lblPM25,"PM2.5");
lv_obj_set_style_text_font(ui_lblPM25, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_lblPM1 = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_lblPM1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lblPM1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lblPM1, -191 );
lv_obj_set_y( ui_lblPM1, 100 );
lv_obj_set_align( ui_lblPM1, LV_ALIGN_CENTER );
lv_label_set_text(ui_lblPM1,"PM1");

ui_vPM1 = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_vPM1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_vPM1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_vPM1, -191 );
lv_obj_set_y( ui_vPM1, 130 );
lv_obj_set_align( ui_vPM1, LV_ALIGN_CENTER );
lv_label_set_text(ui_vPM1,"5");

ui_lblPM10 = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_lblPM10, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lblPM10, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lblPM10, -131 );
lv_obj_set_y( ui_lblPM10, 100 );
lv_obj_set_align( ui_lblPM10, LV_ALIGN_CENTER );
lv_label_set_text(ui_lblPM10,"PM10");

ui_vPM10 = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_vPM10, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_vPM10, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_vPM10, -131 );
lv_obj_set_y( ui_vPM10, 130 );
lv_obj_set_align( ui_vPM10, LV_ALIGN_CENTER );
lv_label_set_text(ui_vPM10,"6");

ui_lblCO2 = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_lblCO2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lblCO2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lblCO2, -74 );
lv_obj_set_y( ui_lblCO2, 100 );
lv_obj_set_align( ui_lblCO2, LV_ALIGN_CENTER );
lv_label_set_text(ui_lblCO2,"CO2");

ui_vCO2 = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_vCO2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_vCO2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_vCO2, -74 );
lv_obj_set_y( ui_vCO2, 130 );
lv_obj_set_align( ui_vCO2, LV_ALIGN_CENTER );
lv_label_set_text(ui_vCO2,"530");

ui_vVOC = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_vVOC, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_vVOC, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_vVOC, -22 );
lv_obj_set_y( ui_vVOC, 130 );
lv_obj_set_align( ui_vVOC, LV_ALIGN_CENTER );
lv_label_set_text(ui_vVOC,"24");

ui_lblVOC = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_lblVOC, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lblVOC, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lblVOC, -22 );
lv_obj_set_y( ui_lblVOC, 100 );
lv_obj_set_align( ui_lblVOC, LV_ALIGN_CENTER );
lv_label_set_text(ui_lblVOC,"VOC");

ui_lblTEMP = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_lblTEMP, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lblTEMP, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lblTEMP, 102 );
lv_obj_set_y( ui_lblTEMP, 100 );
lv_obj_set_align( ui_lblTEMP, LV_ALIGN_CENTER );
lv_label_set_text(ui_lblTEMP,"TEMP :");

ui_vTEMP = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_vTEMP, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_vTEMP, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_vTEMP, 158 );
lv_obj_set_y( ui_vTEMP, 100 );
lv_obj_set_align( ui_vTEMP, LV_ALIGN_CENTER );
lv_label_set_text(ui_vTEMP,"28.2");

ui_lblTEMPunit = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_lblTEMPunit, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lblTEMPunit, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lblTEMPunit, 200 );
lv_obj_set_y( ui_lblTEMPunit, 100 );
lv_obj_set_align( ui_lblTEMPunit, LV_ALIGN_CENTER );
lv_label_set_text(ui_lblTEMPunit,"C");

ui_lblHUMI = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_lblHUMI, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lblHUMI, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lblHUMI, 102 );
lv_obj_set_y( ui_lblHUMI, 130 );
lv_obj_set_align( ui_lblHUMI, LV_ALIGN_CENTER );
lv_label_set_text(ui_lblHUMI,"HUMI :");

ui_vHUMI = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_vHUMI, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_vHUMI, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_vHUMI, 158 );
lv_obj_set_y( ui_vHUMI, 130 );
lv_obj_set_align( ui_vHUMI, LV_ALIGN_CENTER );
lv_label_set_text(ui_vHUMI,"68.9");

ui_lblHUMIunit = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_lblHUMIunit, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lblHUMIunit, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lblHUMIunit, 200 );
lv_obj_set_y( ui_lblHUMIunit, 130 );
lv_obj_set_align( ui_lblHUMIunit, LV_ALIGN_CENTER );
lv_label_set_text(ui_lblHUMIunit,"%");

ui_dateTime = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_dateTime, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_dateTime, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_dateTime, -165 );
lv_obj_set_y( ui_dateTime, -140 );
lv_obj_set_align( ui_dateTime, LV_ALIGN_CENTER );
lv_label_set_text(ui_dateTime,"12/08/2024  -  23:24");
lv_obj_set_style_text_font(ui_dateTime, &lv_font_montserrat_12, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_lblsignal = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_lblsignal, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lblsignal, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lblsignal, 184 );
lv_obj_set_y( ui_lblsignal, -140 );
lv_obj_set_align( ui_lblsignal, LV_ALIGN_CENTER );
lv_label_set_text(ui_lblsignal,"99");
lv_obj_set_style_text_font(ui_lblsignal, &lv_font_montserrat_12, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_lblsignalUnit = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_lblsignalUnit, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lblsignalUnit, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lblsignalUnit, 210 );
lv_obj_set_y( ui_lblsignalUnit, -140 );
lv_obj_set_align( ui_lblsignalUnit, LV_ALIGN_CENTER );
lv_label_set_text(ui_lblsignalUnit,"%");
lv_obj_set_style_text_font(ui_lblsignalUnit, &lv_font_montserrat_12, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_lblPM25unit = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_lblPM25unit, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lblPM25unit, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lblPM25unit, -81 );
lv_obj_set_y( ui_lblPM25unit, 0 );
lv_obj_set_align( ui_lblPM25unit, LV_ALIGN_CENTER );
lv_label_set_text(ui_lblPM25unit,"ug/m3");
lv_obj_set_style_text_font(ui_lblPM25unit, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_vPM25unit1 = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_vPM25unit1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_vPM25unit1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_vPM25unit1, -81 );
lv_obj_set_y( ui_vPM25unit1, -52 );
lv_obj_set_align( ui_vPM25unit1, LV_ALIGN_CENTER );
lv_label_set_text(ui_vPM25unit1,"15");
lv_obj_set_style_text_font(ui_vPM25unit1, &lv_font_montserrat_48, LV_PART_MAIN| LV_STATE_DEFAULT);

}