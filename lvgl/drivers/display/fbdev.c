/**
 * @file fbdev.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "fbdev.h"
#if USE_FBDEV || USE_BSD_FBDEV

#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include "../../../common/device.h"

#if USE_BSD_FBDEV
#include <sys/fcntl.h>
#include <sys/time.h>
#include <sys/consio.h>
#include <sys/fbio.h>
#else  /* USE_BSD_FBDEV */
#include <linux/fb.h>
#endif /* USE_BSD_FBDEV */

/*********************
 *      DEFINES
 *********************/
#ifndef FBDEV_PATH
#define FBDEV_PATH  "/dev/fb0"
#endif

#ifndef DIV_ROUND_UP
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#endif

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *      STRUCTURES
 **********************/

struct bsd_fb_var_info{
    uint32_t xoffset;
    uint32_t yoffset;
    uint32_t xres;
    uint32_t yres;
    int bits_per_pixel;
 };

struct bsd_fb_fix_info{
    long int line_length;
    long int smem_len;
};

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
#if USE_BSD_FBDEV
static struct bsd_fb_var_info vinfo;
static struct bsd_fb_fix_info finfo;
#else
static struct fb_var_screeninfo vinfo;
static struct fb_fix_screeninfo finfo;
#endif /* USE_BSD_FBDEV */
static char *fbp = 0;
static long int screensize = 0;
static int fbfd = 0;

/**********************
 *      MACROS
 **********************/

#if USE_BSD_FBDEV
#define FBIOBLANK FBIO_BLANK
#endif /* USE_BSD_FBDEV */

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void fbdev_init(const char *fbdev_path)
{
    // Open the file for reading and writing
    fbfd = open(fbdev_path, O_RDWR);
    if(fbfd == -1) {
        perror("Error: cannot open framebuffer device");
        return;
    }
    LV_LOG_INFO("The framebuffer device was opened successfully");

#if FBDEV_DISPLAY_POWER_ON
    // Make sure that the display is on.
    if (ioctl(fbfd, FBIOBLANK, FB_BLANK_UNBLANK) != 0) {
        perror("ioctl(FBIOBLANK)");
        return;
    }
#endif /* FBDEV_DISPLAY_POWER_ON */

#if USE_BSD_FBDEV
    struct fbtype fb;
    unsigned line_length;

    //Get fb type
    if (ioctl(fbfd, FBIOGTYPE, &fb) != 0) {
        perror("ioctl(FBIOGTYPE)");
        return;
    }

    //Get screen width
    if (ioctl(fbfd, FBIO_GETLINEWIDTH, &line_length) != 0) {
        perror("ioctl(FBIO_GETLINEWIDTH)");
        return;
    }

    vinfo.xres = (unsigned) fb.fb_width;
    vinfo.yres = (unsigned) fb.fb_height;
    vinfo.bits_per_pixel = fb.fb_depth;
    vinfo.xoffset = 0;
    vinfo.yoffset = 0;
    finfo.line_length = line_length;
    finfo.smem_len = finfo.line_length * vinfo.yres;
#else /* USE_BSD_FBDEV */

    // Get fixed screen information
    if(ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
        perror("Error reading fixed information");
        return;
    }

    // Get variable screen information
    if(ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        perror("Error reading variable information");
        return;
    }
#endif /* USE_BSD_FBDEV */

    LV_LOG_INFO("%dx%d, %dbpp", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);

    // Figure out the size of the screen in bytes
    screensize =  finfo.smem_len; //finfo.line_length * vinfo.yres;    

    // Map the device to memory
    fbp = (char *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
    if((intptr_t)fbp == -1) {
        perror("Error: failed to map framebuffer device to memory");
        return;
    }

    // Don't initialise the memory to retain what's currently displayed / avoid clearing the screen.
    // This is important for applications that only draw to a subsection of the full framebuffer.

    LV_LOG_INFO("The framebuffer device was mapped to memory successfully");

}

void fbdev_exit(void)
{
    close(fbfd);
}

/**
 * Check for hdmi state and adapt according to the current device
 * @param disp driver pointer to display driver which active screen should be get.
 */
void fbdev_hdmi_rotate(lv_disp_drv_t * old_driver)
{
    lv_disp_drv_t * driver;
    char * hdmi_state_file="/sys/devices/platform/soc/6000000.hdmi/extcon/hdmi/state";
    FILE * f = fopen(hdmi_state_file, "r");
    char state[7];
    fgets(state,7,f);
    fclose(f);
    printf("state HDMI is %s \n",state);
    printf("hdmi compare: %d \n",strcmp(state,"HDMI=1"));
    printf("name compare %d \n",strcmp(device.DEVICE.NAME,"RG28XX"));

    if (strstr(state,"HDMI=1") && strstr(device.DEVICE.NAME,"RG28XX")){
        printf("HDMI is ON \n");
        driver->hor_res = device.SCREEN.HEIGHT;
        driver->ver_res = device.SCREEN.WIDTH;
        driver->sw_rotate = 0;
        driver->rotated = 0;
        vinfo.xres = driver->hor_res;
        vinfo.yres = driver->ver_res;
        vinfo.xres_virtual = vinfo.xres;
        vinfo.yres_virtual = vinfo.yres *2;
    }

}

/**
 * Flush a buffer to the marked area
 * @param drv pointer to driver where this function belongs
 * @param area an area where to copy `color_p`
 * @param color_p an array of pixels to copy to the `area` part of the screen
 */
void fbdev_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(fbp == NULL ||
            area->x2 < 0 ||
            area->y2 < 0 ||
            area->x1 > (int32_t)vinfo.xres - 1 ||
            area->y1 > (int32_t)vinfo.yres - 1) {
        lv_disp_flush_ready(drv);
        return;
    }

    /*Truncate the area to the screen*/
    int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t act_x2 = area->x2 > (int32_t)vinfo.xres - 1 ? (int32_t)vinfo.xres - 1 : area->x2;
    int32_t act_y2 = area->y2 > (int32_t)vinfo.yres - 1 ? (int32_t)vinfo.yres - 1 : area->y2;

    printf("area x2 %d area y2 %d \n",act_x2,act_y2);
    printf("x offset %d y offset %d \n",vinfo.xoffset, vinfo.yoffset);
    printf("virtual x %d y %d \n",vinfo.xres_virtual, vinfo.yres_virtual);
    lv_coord_t w = (act_x2 - act_x1 + 1);
    long int location = 0;
    long int byte_location = 0;
    unsigned char bit_location = 0;

    /*32 or 24 bit per pixel*/
    if(vinfo.bits_per_pixel == 32 || vinfo.bits_per_pixel == 24) {
        uint32_t * fbp32 = (uint32_t *)fbp;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length / 4;
            memcpy(&fbp32[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1) * 4);
            color_p += w;
        }
    }
    /*16 bit per pixel*/
    else if(vinfo.bits_per_pixel == 16) {
        uint16_t * fbp16 = (uint16_t *)fbp;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length / 2;
            memcpy(&fbp16[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1) * 2);
            color_p += w;
        }
    }
    /*8 bit per pixel*/
    else if(vinfo.bits_per_pixel == 8) {
        uint8_t * fbp8 = (uint8_t *)fbp;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            location = (act_x1 + vinfo.xoffset) + (y + vinfo.yoffset) * finfo.line_length;
            memcpy(&fbp8[location], (uint32_t *)color_p, (act_x2 - act_x1 + 1));
            color_p += w;
        }
    }
    /*1 bit per pixel*/
    else if(vinfo.bits_per_pixel == 1) {
        uint8_t * fbp8 = (uint8_t *)fbp;
        int32_t x;
        int32_t y;
        for(y = act_y1; y <= act_y2; y++) {
            for(x = act_x1; x <= act_x2; x++) {
                location = (x + vinfo.xoffset) + (y + vinfo.yoffset) * vinfo.xres;
                byte_location = location / 8; /* find the byte we need to change */
                bit_location = location % 8; /* inside the byte found, find the bit we need to change */
                fbp8[byte_location] &= ~(((uint8_t)(1)) << bit_location);
                fbp8[byte_location] |= ((uint8_t)(color_p->full)) << bit_location;
                color_p++;
            }

            color_p += area->x2 - act_x2;
        }
    } else {
        /*Not supported bit per pixel*/
    }

    //May be some direct update command is required
    //ret = ioctl(state->fd, FBIO_UPDATE, (unsigned long)((uintptr_t)rect));

    lv_disp_flush_ready(drv);
}

void fbdev_get_sizes(uint32_t *width, uint32_t *height, uint32_t *dpi) {
    if (width)
        *width = vinfo.xres;

    if (height)
        *height = vinfo.yres;

    if (dpi && vinfo.height)
        *dpi = DIV_ROUND_UP(vinfo.xres * 254, vinfo.width * 10);
}

void fbdev_set_offset(uint32_t xoffset, uint32_t yoffset) {
    vinfo.xoffset = xoffset;
    vinfo.yoffset = yoffset;
}

static void flush_cb(lv_disp_drv_t * dsc, const lv_area_t * area, uint8_t * color_p)
{

    if(fbp == NULL ||
            area->x2 < 0 ||
            area->y2 < 0 ||
            area->x1 > (int32_t)vinfo.xres - 1 ||
            area->y1 > (int32_t)vinfo.yres - 1) {
        lv_disp_flush_ready(drv);
        return;
    }


    int32_t w = lv_area_get_width(area);
    int32_t h = lv_area_get_height(area);
    lv_color_format_t cf = lv_display_get_color_format(disp);
    uint32_t px_size = lv_color_format_get_size(cf);

    lv_area_t rotated_area;
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);

    /* Not all framebuffer kernel drivers support hardware rotation, so we need to handle it in software here */
    if(rotation != LV_DISPLAY_ROTATION_0 && LV_LINUX_FBDEV_RENDER_MODE == LV_DISPLAY_RENDER_MODE_PARTIAL) {
        /* (Re)allocate temporary buffer if needed */
        size_t buf_size = w * h * px_size;
        if(!dsc->rotated_buf || dsc->rotated_buf_size != buf_size) {
            dsc->rotated_buf = realloc(dsc->rotated_buf, buf_size);
            dsc->rotated_buf_size = buf_size;
        }

        /* Rotate the pixel buffer */
        uint32_t w_stride = lv_draw_buf_width_to_stride(w, cf);
        uint32_t h_stride = lv_draw_buf_width_to_stride(h, cf);

        switch(rotation) {
            case LV_DISPLAY_ROTATION_0:
                break;
            case LV_DISPLAY_ROTATION_90:
                lv_draw_sw_rotate(color_p, dsc->rotated_buf, w, h, w_stride, h_stride, rotation, cf);
                break;
            case LV_DISPLAY_ROTATION_180:
                lv_draw_sw_rotate(color_p, dsc->rotated_buf, w, h, w_stride, w_stride, rotation, cf);
                break;
            case LV_DISPLAY_ROTATION_270:
                lv_draw_sw_rotate(color_p, dsc->rotated_buf, w, h, w_stride, h_stride, rotation, cf);
                break;
        }
        color_p = dsc->rotated_buf;

        /* Rotate the area */
        rotated_area = *area;
        lv_display_rotate_area(disp, &rotated_area);
        area = &rotated_area;

        if(rotation != LV_DISPLAY_ROTATION_180) {
            w = lv_area_get_width(area);
            h = lv_area_get_height(area);
        }
    }

    /* Ensure that we're within the framebuffer's bounds */
    if(area->x2 < 0 || area->y2 < 0 || area->x1 > (int32_t)dsc->vinfo.xres - 1 || area->y1 > (int32_t)dsc->vinfo.yres - 1) {
        lv_display_flush_ready(disp);
        return;
    }

    uint32_t fb_pos =
        (area->x1 + dsc->vinfo.xoffset) * px_size +
        (area->y1 + dsc->vinfo.yoffset) * dsc->finfo.line_length;

    uint8_t * fbp = (uint8_t *)dsc->fbp;
    int32_t y;
    if(LV_LINUX_FBDEV_RENDER_MODE == LV_DISPLAY_RENDER_MODE_DIRECT) {
        uint32_t color_pos =
            area->x1 * px_size +
            area->y1 * disp->hor_res * px_size;

        for(y = area->y1; y <= area->y2; y++) {
            lv_memcpy(&fbp[fb_pos], &color_p[color_pos], w * px_size);
            fb_pos += dsc->finfo.line_length;
            color_pos += disp->hor_res * px_size;
        }
    }
    else {
        w = lv_area_get_width(area);
        for(y = area->y1; y <= area->y2; y++) {
            lv_memcpy(&fbp[fb_pos], color_p, w * px_size);
            fb_pos += dsc->finfo.line_length;
            color_p += w * px_size;
        }
    }

    if(dsc->force_refresh) {
        dsc->vinfo.activate |= FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE;
        if(ioctl(dsc->fbfd, FBIOPUT_VSCREENINFO, &(dsc->vinfo)) == -1) {
            perror("Error setting var screen info");
        }
    }

    lv_display_flush_ready(disp);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif
