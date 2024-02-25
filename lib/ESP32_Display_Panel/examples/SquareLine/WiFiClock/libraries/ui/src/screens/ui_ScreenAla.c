// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: ArduinoDemo

#include "../ui.h"

void ui_ScreenAla_screen_init(void)
{
ui_ScreenAla = lv_obj_create(NULL);
lv_obj_clear_flag( ui_ScreenAla, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_ScreenAla, lv_color_hex(0x112D4E), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ScreenAla, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_ScreenAla, lv_color_hex(0x112D4E), LV_PART_SCROLLBAR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ScreenAla, 255, LV_PART_SCROLLBAR| LV_STATE_DEFAULT);

ui_Rollerhour = lv_roller_create(ui_ScreenAla);
lv_roller_set_options( ui_Rollerhour, "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23", LV_ROLLER_MODE_INFINITE );
lv_obj_set_width( ui_Rollerhour, 60);
lv_obj_set_height( ui_Rollerhour, 100);
lv_obj_set_x( ui_Rollerhour, -107 );
lv_obj_set_y( ui_Rollerhour, -8 );
lv_obj_set_align( ui_Rollerhour, LV_ALIGN_CENTER );
lv_obj_set_style_text_color(ui_Rollerhour, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Rollerhour, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Rollerhour, lv_color_hex(0x010408), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Rollerhour, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_Rollerhour, lv_color_hex(0x112D4E), LV_PART_SELECTED | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Rollerhour, 255, LV_PART_SELECTED| LV_STATE_DEFAULT);

ui_Rollerminute = lv_roller_create(ui_ScreenAla);
lv_roller_set_options( ui_Rollerminute, "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23\n24\n25\n26\n27\n28\n29\n30\n31\n32\n33\n34\n35\n36\n37\n38\n39\n40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n50\n51\n52\n53\n54\n55\n56\n57\n58\n59", LV_ROLLER_MODE_INFINITE );
lv_obj_set_width( ui_Rollerminute, 60);
lv_obj_set_height( ui_Rollerminute, 100);
lv_obj_set_x( ui_Rollerminute, 1 );
lv_obj_set_y( ui_Rollerminute, -8 );
lv_obj_set_align( ui_Rollerminute, LV_ALIGN_CENTER );
lv_obj_set_style_text_color(ui_Rollerminute, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Rollerminute, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Rollerminute, lv_color_hex(0x010408), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Rollerminute, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_Rollerminute, lv_color_hex(0x112D4E), LV_PART_SELECTED | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Rollerminute, 255, LV_PART_SELECTED| LV_STATE_DEFAULT);

ui_Rollersecond = lv_roller_create(ui_ScreenAla);
lv_roller_set_options( ui_Rollersecond, "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23\n24\n25\n26\n27\n28\n29\n30\n31\n32\n33\n34\n35\n36\n37\n38\n39\n40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n50\n51\n52\n53\n54\n55\n56\n57\n58\n59", LV_ROLLER_MODE_INFINITE );
lv_obj_set_width( ui_Rollersecond, 60);
lv_obj_set_height( ui_Rollersecond, 100);
lv_obj_set_x( ui_Rollersecond, 109 );
lv_obj_set_y( ui_Rollersecond, -8 );
lv_obj_set_align( ui_Rollersecond, LV_ALIGN_CENTER );
lv_obj_set_style_text_color(ui_Rollersecond, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Rollersecond, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Rollersecond, lv_color_hex(0x010408), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Rollersecond, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_Rollersecond, lv_color_hex(0x112D4E), LV_PART_SELECTED | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Rollersecond, 255, LV_PART_SELECTED| LV_STATE_DEFAULT);

ui_Labelcolon1 = lv_label_create(ui_ScreenAla);
lv_obj_set_width( ui_Labelcolon1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Labelcolon1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Labelcolon1, -55 );
lv_obj_set_y( ui_Labelcolon1, -6 );
lv_obj_set_align( ui_Labelcolon1, LV_ALIGN_CENTER );
lv_label_set_text(ui_Labelcolon1,":");
lv_obj_set_style_text_font(ui_Labelcolon1, &ui_font_FontNumber48bp4, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Labelcolon2 = lv_label_create(ui_ScreenAla);
lv_obj_set_width( ui_Labelcolon2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Labelcolon2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Labelcolon2, 55 );
lv_obj_set_y( ui_Labelcolon2, -7 );
lv_obj_set_align( ui_Labelcolon2, LV_ALIGN_CENTER );
lv_label_set_text(ui_Labelcolon2,":");
lv_obj_set_style_text_font(ui_Labelcolon2, &ui_font_FontNumber48bp4, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ButtonConfirm = lv_btn_create(ui_ScreenAla);
lv_obj_set_width( ui_ButtonConfirm, 93);
lv_obj_set_height( ui_ButtonConfirm, 38);
lv_obj_set_x( ui_ButtonConfirm, -1 );
lv_obj_set_y( ui_ButtonConfirm, 78 );
lv_obj_set_align( ui_ButtonConfirm, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_ButtonConfirm, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_ButtonConfirm, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_ButtonConfirm, lv_color_hex(0x3F72AF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ButtonConfirm, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_LabelConfirm = lv_label_create(ui_ButtonConfirm);
lv_obj_set_width( ui_LabelConfirm, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_LabelConfirm, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_LabelConfirm, 0 );
lv_obj_set_y( ui_LabelConfirm, 2 );
lv_obj_set_align( ui_LabelConfirm, LV_ALIGN_CENTER );
lv_label_set_text(ui_LabelConfirm,"Confirm");
lv_obj_set_style_text_font(ui_LabelConfirm, &ui_font_FontPuHui20bp4, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ButtonRetSet2 = lv_btn_create(ui_ScreenAla);
lv_obj_set_width( ui_ButtonRetSet2, 30);
lv_obj_set_height( ui_ButtonRetSet2, 26);
lv_obj_set_x( ui_ButtonRetSet2, -141 );
lv_obj_set_y( ui_ButtonRetSet2, -105 );
lv_obj_set_align( ui_ButtonRetSet2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_ButtonRetSet2, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_ButtonRetSet2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_ButtonRetSet2, lv_color_hex(0x3F72AF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ButtonRetSet2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ImageRetSet2 = lv_img_create(ui_ButtonRetSet2);
lv_img_set_src(ui_ImageRetSet2, &ui_img_return_png);
lv_obj_set_width( ui_ImageRetSet2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_ImageRetSet2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_ImageRetSet2, -2 );
lv_obj_set_y( ui_ImageRetSet2, 1 );
lv_obj_set_align( ui_ImageRetSet2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_ImageRetSet2, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_ImageRetSet2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_img_set_zoom(ui_ImageRetSet2,30);
lv_obj_set_style_img_recolor(ui_ImageRetSet2, lv_color_hex(0xDBE2EF), LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_img_recolor_opa(ui_ImageRetSet2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_LabelHour = lv_label_create(ui_ScreenAla);
lv_obj_set_width( ui_LabelHour, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_LabelHour, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_LabelHour, -107 );
lv_obj_set_y( ui_LabelHour, -77 );
lv_obj_set_align( ui_LabelHour, LV_ALIGN_CENTER );
lv_label_set_text(ui_LabelHour,"Hour");
lv_obj_set_style_text_color(ui_LabelHour, lv_color_hex(0xDBE2EF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_LabelHour, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_LabelHour, &ui_font_FontPuHui20bp4, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_LabelHour1 = lv_label_create(ui_ScreenAla);
lv_obj_set_width( ui_LabelHour1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_LabelHour1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_LabelHour1, 108 );
lv_obj_set_y( ui_LabelHour1, -77 );
lv_obj_set_align( ui_LabelHour1, LV_ALIGN_CENTER );
lv_label_set_text(ui_LabelHour1,"Sec");
lv_obj_set_style_text_color(ui_LabelHour1, lv_color_hex(0xDBE2EF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_LabelHour1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_LabelHour1, &ui_font_FontPuHui20bp4, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_LabelHour2 = lv_label_create(ui_ScreenAla);
lv_obj_set_width( ui_LabelHour2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_LabelHour2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_LabelHour2, -1 );
lv_obj_set_y( ui_LabelHour2, -77 );
lv_obj_set_align( ui_LabelHour2, LV_ALIGN_CENTER );
lv_label_set_text(ui_LabelHour2,"Min");
lv_obj_set_style_text_color(ui_LabelHour2, lv_color_hex(0xDBE2EF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_LabelHour2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_LabelHour2, &ui_font_FontPuHui20bp4, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_ButtonConfirm, ui_event_ButtonConfirm, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_ButtonRetSet2, ui_event_ButtonRetSet2, LV_EVENT_ALL, NULL);

}