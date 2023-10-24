
#ifndef __TIS_TEGRACAM_V4L2_H__
#define __TIS_TEGRACAM_V4L2_H__

int begin_tegracam_v4l2subdev_register(struct tegracam_device *tc_dev, bool is_sensor);
int end_tegracam_v4l2subdev_register(struct tegracam_device *tc_dev);

#endif // __TIS_TEGRACAM_V4L2_H__