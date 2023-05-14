# magmed_ws
ROS workspace for remote magnetic medical robotics

## magmed_camera

HIK VISION camera

|-demo01_camera_connect (connect test)

|

|-demo02_grabimagebuffer (unstable, unkonw reason)

|

|-demo03_grabimageBGR (stable)


test log: 2023/5/12

    demo02_grabimagebuffer creates a thread to grab image, using camera's memory.

    demo02_grabimagebuffer dies when nFrame reaches 1900+ for unkonw reason. Probably insufficient memory.

    demo03_grabimageBGR uses callback function to grab image.

    demo03_grabimageBGR runs fast and fine. 

    demo02_grabimagebuffer is now changed to demo02_grabimage_activestream.

    demo03_grabimageBGR is now changed to demo03_grabimage_callback.

    free pDataBGR solves the insufficient memory issue.

    不建议在demo03_grabimage_callback的callback函数中添加额外的耗时图像处理

