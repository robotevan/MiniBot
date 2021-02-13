//
// Created by Evan on 2021-02-12.
//

#include "Camera.h"

Camera::Camera() {
    // create file descriptor
    fd = open(CAMERA_DEV, O_RDWR);
    if (fd < 0){
        std::cout << "Failed to open camera" << std::endl;
        return;
    }
    // get device capabilities
    v4l2_capability capability{};
    if (ioctl(fd, VIDIOC_QUERYCAP, &capability) < 0){
        std::cout << "failed to retrieve camera capabilities" << std::endl;
        return;
    }
    // Set camera format
    v4l2_format img_format{};
    img_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    img_format.fmt.pix.width = VIDEO_WIDTH;
    img_format.fmt.pix.height = VIDEO_HEIGHT;
    img_format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    img_format.fmt.pix.field = V4L2_FIELD_NONE;
    // update the cameras format
    if(ioctl(fd, VIDIOC_S_FMT, &img_format)){
        std::cout << "Failed to set camera format" << std::endl;
        return;
    }
}

void Camera::capture_frame() {
    return 0;
}
