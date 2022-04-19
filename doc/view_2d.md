# Viewing the RGB image

The RGB image is published on the `/ifm3d/camera*/rgb` topic in a compressed JPEG format.
This is useful to save bandwidth, but cannot be visualized as-is with RViz.

The image can be uncompressed using the `image_transport republish` node:
```bash
$ sudo apt install ros-galactic-compressed-image-transport
$ ros2 run image_transport republish compressed raw --ros-args --remap /in/compressed:=/ifm3d/camera/rgb --remap out:=/uncompressed_rgb
```

In RViz, you can now subscribe to the `/uncompressed_rgb` topic to visualize the RGB image.