#include "lvgl.h"
#include "lis3dh.h"  // Include your accelerometer header

lv_obj_t *accel_label;  // Make this global so other code can access it

void example_lvgl_demo_ui(lv_disp_t *disp)
{
    lv_obj_t *scr = lv_scr_act();
    accel_label = lv_label_create(scr);

    lv_label_set_text(accel_label, "Initializing...");
    lv_obj_set_style_text_font(accel_label, &lv_font_montserrat_14, 0);
    lv_obj_align(accel_label, LV_ALIGN_TOP_MID, 0, 0);
}

