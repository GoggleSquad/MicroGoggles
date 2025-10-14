/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "lvgl.h"

// void example_lvgl_demo_ui(lv_display_t *disp)
// {
//     lv_obj_t *scr = lv_display_get_screen_active(disp);
//     lv_obj_t *label = lv_label_create(scr);
//     lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
//     lv_label_set_text(label, "Swim Session Starting in 3. . . 2 . . . 1 . . . Go! ");
//     /* Size of the screen (if you use rotation 90 or 270, please use lv_display_get_vertical_resolution) */
//     lv_obj_set_width(label, lv_display_get_horizontal_resolution(disp));
//     lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
// }

void example_lvgl_demo_ui(lv_disp_t *disp)
{
    // Create label on active screen
    lv_obj_t *scr = lv_scr_act();
    lv_obj_t *label = lv_label_create(scr);

    // Set scrolling mode
    // lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);

    // Set the text
    lv_label_set_text(label, "Swim Session Starting in 3... 2... 1... Go!");

    // Set font (use a font that exists in your build)
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);

    // Set label width to screen width so it scrolls horizontally
    lv_obj_set_width(label, lv_disp_get_hor_res(disp));

    // Align the label to the top-middle
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
}


/*
#1
lv_disp_t *disp = lv_disp_get_default();
lv_disp_set_rotation(disp, LV_DISP_ROT_MIRROR_X);

#2 
lv_disp_t *disp = lv_disp_get_default();
lv_disp_set_rotation(disp, LV_DISP_ROT_NONE | LV_DISP_ROT_MIRROR_X);

#3
void my_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
    int width = area->x2 - area->x1 + 1;
    int height = area->y2 - area->y1 + 1;

    // Mirror horizontally
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width / 2; x++) {
            lv_color_t tmp = color_p[y * width + x];
            color_p[y * width + x] = color_p[y * width + (width - 1 - x)];
            color_p[y * width + (width - 1 - x)] = tmp;
        }
    }

    // Then send the (mirrored) buffer to your OLED
    oled_send_pixels(area, color_p);

    lv_disp_flush_ready(drv);
}

*/