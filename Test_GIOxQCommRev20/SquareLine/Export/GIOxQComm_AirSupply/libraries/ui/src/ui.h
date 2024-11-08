// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 9.1.0
// Project name: GIOxQComm_AirSupply

#ifndef _GIOXQCOMM_AIRSUPPLY_UI_H
#define _GIOXQCOMM_AIRSUPPLY_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
  #if __has_include("lvgl.h")
    #include "lvgl.h"
  #elif __has_include("lvgl/lvgl.h")
    #include "lvgl/lvgl.h"
  #else
    #include "lvgl.h"
  #endif
#else
  #include "lvgl.h"
#endif

#include "ui_helpers.h"
#include "ui_events.h"

// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
extern lv_obj_t *ui_Screen1;
extern lv_obj_t *ui_Image2;
extern lv_obj_t *ui_Image3;
extern lv_obj_t *ui_lblPM25;
extern lv_obj_t *ui_lblPM1;
extern lv_obj_t *ui_vPM1;
extern lv_obj_t *ui_lblPM10;
extern lv_obj_t *ui_vPM10;
extern lv_obj_t *ui_lblCO2;
extern lv_obj_t *ui_vCO2;
extern lv_obj_t *ui_vVOC;
extern lv_obj_t *ui_lblVOC;
extern lv_obj_t *ui_lblTEMP;
extern lv_obj_t *ui_vTEMP;
extern lv_obj_t *ui_lblTEMPunit;
extern lv_obj_t *ui_lblHUMI;
extern lv_obj_t *ui_vHUMI;
extern lv_obj_t *ui_lblHUMIunit;
extern lv_obj_t *ui_dateTime;
extern lv_obj_t *ui_lblsignal;
extern lv_obj_t *ui_lblsignalUnit;
extern lv_obj_t *ui_lblPM25unit;
extern lv_obj_t *ui_vPM25unit1;
extern lv_obj_t *ui____initial_actions0;

LV_IMG_DECLARE( ui_img_ancestry_6927683_png);   // assets/ancestry_6927683.png
LV_IMG_DECLARE( ui_img_wifi_2803371_png);   // assets/wifi_2803371.png




void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
