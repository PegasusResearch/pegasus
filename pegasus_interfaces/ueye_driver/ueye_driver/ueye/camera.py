#!/usr/bin/env python

from pyueye import ueye
from .utils import (uEyeException, Rect, get_bits_per_pixel, ImageBuffer, check)

import ctypes
import numpy as np

class Camera:
    def __init__(self, device_id=0):
        self.h_cam = ueye.HIDS(device_id)
        self.img_buffers = []

    def __enter__(self):
        self.init()
        return self

    def __exit__(self, _type, value, traceback):
        self.exit()

    def handle(self):
        return self.h_cam

    def alloc(self, buffer_count=3):
        rect = self.get_aoi()
        bpp = get_bits_per_pixel(self.get_colormode())

        for buff in self.img_buffers:
            check(ueye.is_FreeImageMem(self.h_cam, buff.mem_ptr, buff.mem_id))

        for i in range(buffer_count):
            buff = ImageBuffer()
            ueye.is_AllocImageMem(self.h_cam,
                                  rect.width, rect.height, bpp,
                                  buff.mem_ptr, buff.mem_id)
            
            check(ueye.is_AddToSequence(self.h_cam, buff.mem_ptr, buff.mem_id))

            self.img_buffers.append(buff)

        ueye.is_InitImageQueue(self.h_cam, 0)

    def init(self):
        ret = ueye.is_InitCamera(self.h_cam, None)
        if ret != ueye.IS_SUCCESS:
            self.h_cam = None
            raise uEyeException(ret)
            
        return ret

    def exit(self):
        ret = None
        if self.h_cam is not None:
            ret = ueye.is_ExitCamera(self.h_cam)
        if ret == ueye.IS_SUCCESS:
            self.h_cam = None

    def get_aoi(self):
        rect_aoi = ueye.IS_RECT()
        ueye.is_AOI(self.h_cam, ueye.IS_AOI_IMAGE_GET_AOI, rect_aoi, ueye.sizeof(rect_aoi))

        return Rect(rect_aoi.s32X.value,
                    rect_aoi.s32Y.value,
                    rect_aoi.s32Width.value,
                    rect_aoi.s32Height.value)

    def set_aoi(self, x, y, width, height):
        rect_aoi = ueye.IS_RECT()
        rect_aoi.s32X = ueye.int(x)
        rect_aoi.s32Y = ueye.int(y)
        rect_aoi.s32Width = ueye.int(width)
        rect_aoi.s32Height = ueye.int(height)

        return ueye.is_AOI(self.h_cam, ueye.IS_AOI_IMAGE_SET_AOI, rect_aoi, ueye.sizeof(rect_aoi))

    def capture_video(self, wait=False):
        wait_param = ueye.IS_WAIT if wait else ueye.IS_DONT_WAIT
        return ueye.is_CaptureVideo(self.h_cam, wait_param)

    def stop_video(self):
        return ueye.is_StopLiveVideo(self.h_cam, ueye.IS_FORCE_VIDEO_STOP)
    
    def freeze_video(self, wait=False):
        wait_param = ueye.IS_WAIT if wait else ueye.IS_DONT_WAIT
        return ueye.is_FreezeVideo(self.h_cam, wait_param)

    def set_colormode(self, colormode):
        check(ueye.is_SetColorMode(self.h_cam, colormode))
        
    def get_colormode(self):
        ret = ueye.is_SetColorMode(self.h_cam, ueye.IS_GET_COLOR_MODE)
        return ret

    def get_format_list(self):
        count = ueye.UINT()
        check(ueye.is_ImageFormat(self.h_cam, ueye.IMGFRMT_CMD_GET_NUM_ENTRIES, count, ueye.sizeof(count)))
        format_list = ueye.IMAGE_FORMAT_LIST(ueye.IMAGE_FORMAT_INFO * count.value)
        format_list.nSizeOfListEntry = ueye.sizeof(ueye.IMAGE_FORMAT_INFO)
        format_list.nNumListElements = count.value
        check(ueye.is_ImageFormat(self.h_cam, ueye.IMGFRMT_CMD_GET_LIST,
                                  format_list, ueye.sizeof(format_list)))
        return format_list

    def set_full_auto(self):
        print("full auto")
        disable = ueye.DOUBLE(0)
        enable = ueye.DOUBLE(1)
        zero = ueye.DOUBLE(0)
        ms = ueye.DOUBLE(20)
        rate = ueye.DOUBLE(50)
        newrate = ueye.DOUBLE()
        number = ueye.UINT()

        ret = ueye.is_SetAutoParameter(self.h_cam, ueye.IS_SET_ENABLE_AUTO_GAIN, enable, zero);
        print('AG:',ret)
        ret = ueye.is_SetAutoParameter(self.h_cam, ueye.IS_SET_ENABLE_AUTO_SHUTTER, enable, zero);
        print('A_SHUTTER:',ret)
        ret = ueye.is_SetFrameRate(self.h_cam, rate, newrate);
        print('FR:',ret,newrate)
        ret = ueye.is_Exposure(self.h_cam, ueye.IS_EXPOSURE_CMD_GET_EXPOSURE, ms, ueye.sizeof(ms));
        print('EXP:',ret,ms)
        ret = ueye.is_PixelClock(self.h_cam, ueye.IS_PIXELCLOCK_CMD_GET_NUMBER, number, ueye.sizeof(number))
        print('PxCLK #:',ret, number)
        PCrange = (ctypes.c_uint * 3)()
        ret = ueye.is_PixelClock(self.h_cam, ueye.IS_PIXELCLOCK_CMD_GET_RANGE, PCrange, 3*ueye.sizeof(number))
        print('PxCLK range:', ret, PCrange[0], PCrange[1], PCrange[2])
        list_pixel_clocks = (ctypes.c_uint * 150)()
        ret = ueye.is_PixelClock(self.h_cam, ueye.IS_PIXELCLOCK_CMD_GET_LIST,  list_pixel_clocks, number*ueye.sizeof(number))
        list_np = np.frombuffer(list_pixel_clocks, int)
        print('PxCLK list:', ret, list_np[0:number.value])
        ret = ueye.is_PixelClock(self.h_cam, ueye.IS_PIXELCLOCK_CMD_GET, number, ueye.sizeof(number))
        print('PxCLK current:',ret, number)
        ret = ueye.is_PixelClock(self.h_cam, ueye.IS_PIXELCLOCK_CMD_GET_DEFAULT, number, ueye.sizeof(number))
        print('PxCLK default:',ret, number)
        ret = ueye.is_PixelClock(self.h_cam, ueye.IS_PIXELCLOCK_CMD_SET, ueye.UINT(20), ueye.sizeof(number))
        print('PxCLK set:',ret, number)
        ret = ueye.is_PixelClock(self.h_cam, ueye.IS_PIXELCLOCK_CMD_GET, number, ueye.sizeof(number))
        print('PxCLK current:',ret, number)