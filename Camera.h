#ifndef MINIBOT_CAMERA_H
#define MINIBOT_CAMERA_H

#include <cstdio>
#include <cstdlib>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/v4l2-common.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fstream>
#include "iostream"

#define CAMERA_DEV "/dev/video0"
#define VIDEO_WIDTH 1920
#define VIDEO_HEIGHT 1080

class Camera {
    public:
        Camera();
        void capture_frame();
    private:
        int fd;
        char *img_buffer;
};


#endif //MINIBOT_CAMERA_H
