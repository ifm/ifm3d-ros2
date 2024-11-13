
# Advertised Services

| Name    | Service Definition                          | Description                                                                                                               |
| ------- | ------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| `Dump`    | <a href="srv/Dump.srv">ifm3d_ros2/srv/Dump</a>       | Dumps the state of the camera parameters to JSON                                                                          |
| `Config`  | <a href="srv/Config.srv">ifm3d_ros2/srv/Config</a>   | Provides a means to configure the camera and imager settings, declaratively from a JSON encoding of the desired settings. |
| `GetDiag` | <a href="srv/GetDiag.srv">ifm3d_ros2/srv/GetDiag</a> | Get all the diagnostic messages corresponding to a JSON filter. <br>The filter can be left blank to get all active and dormant error messages: `ros2 service call /ifm3d/camera/GetDiag ifm3d_ros2/srv/GetDiag "{filter: '{}'}"`.|
| `Softon`  | <a href="srv/Softon.srv">ifm3d_ros2/srv/Softon</a> | Provides a means to quickly change the camera state from IDLE to RUN.                                                     |
| `Softoff` | <a href="srv/Softoff.srv">ifm3d_ros2/srv/Softoff</a> | Provides a means to quickly change the camera state from RUN to IDLE.                                                     |

## Dump and Config

The ifm3d_ros2 package allows the user to configure the O3R camera platform using ROS native service calls.

### Dump
Calling the native ROS service `/ifm3d_ros2/camera/Dump` for a certain `camera` will return the camera configuration as a JSON string. Please notice the use of backslashes (`\` before each `"`) to escape each upper quotation mark. This is necessary to allow us to keep the JSON syntax native to the underlying API (ifm3d).  

Call this service via, e.g. for camera:
```bash
$ ros2 service call /ifm3d/camera/Dump ifm3d_ros2/srv/Dump

requester: making request: ifm3d_ros2.srv.Dump_Request()

response:
ifm3d_ros2.srv.Dump_Response(status=0, config='{"device":{"clock":{"currentTime":1581110268783361824},"diagnostic":{"temperatures":[],"upTime":19624000000000},"info":{"device":"0301","deviceTreeBinaryBlob":"tegra186-quill-p3310-1000-c03-00-base.dtb","features":{},"name":"","partNumber":"M03975","productionState":"AA","serialNumber":"000201234159","vendor":"0001"},"network":{"authorized_keys":"","ipAddressConfig":0,"macEth0":"00:04:4B:EA:9F:35","macEth1":"00:02:01:23:41:59","networkSpeed":1000,"staticIPv4Address":"192.168.0.69","staticIPv4Gateway":"192.168.0.201","staticIPv4SubNetMask":"255.255.255.0","useDHCP":false},"state":{"errorMessage":"","errorNumber":""},"swVersion":{"kernel":"4.9.140-l4t-r32.4+gc35f5eb9d1d9","l4t":"r32.4.3","os":"0.13.13-221","schema":"v0.1.0","swu":"0.15.12"}},"ports":{"port0":{"acquisition":{"framerate":10.0,"version":{"major":0,"minor":0,"patch":0}},"data":{"algoDebugConfig":{},"availablePCICOutput":[],"pcicTCPPort":50010},"info":{"device":"2301","deviceTreeBinaryBlobOverlay":"001-ov9782.dtbo","features":{"fov":{"horizontal":127,"vertical":80},"resolution":{"height":800,"width":1280},"type":"2D"},"name":"","partNumber":"M03969","productionState":"AA","sensor":"OV9782","sensorID":"OV9782_127x80_noIllu_Csample","serialNumber":"000000000395","vendor":"0001"},"mode":"experimental_autoexposure2D","processing":{"extrinsicHeadToUser":{"rotX":0.0,"rotY":0.0,"rotZ":0.0,"transX":0.0,"transY":0.0,"transZ":0.0},"version":{"major":0,"minor":0,"patch":0}},"state":"RUN"}}')
```


### Config
Below you can see an example on how to configure your camera via a ROS service call. The JSON string can be a partial JSON string. It only needs to follow basic JSON syntax. Please wrap the JSON string in a YAML syntax and use the field `"json"`.

```bash
$ ros2 service call /ifm3d/camera/Config ifm3d_ros2/srv/Config "{json: '{\"ports\":{\"port0\":{\"mode\":\"standard_range4m\"}}}'}"
requester: making request: ifm3d_ros2.srv.Config_Request(json='{"ports":{"port2":{"mode":"standard_range4m"}}}')

response:
ifm3d_ros2.srv.Config_Response(status=0, msg='OK')
```

## Diagnostic

The `ifm3d_ros2` package provides a service to poll diagnostic data from the device. A filter can be provided to retrieve specific diagnostic data. 

For example, to retrieve all active diagnostic corresponding to port 0:
```
$ ros2 service call /ifm3d/camera/GetDiag ifm3d_ros2/srv/GetDiag "{filter: '{\"source\":\"/ports/port0\", \"state\":\"active\"}'}"

requester: making request: ifm3d_ros2.srv.GetDiag_Request(filter='{"source":"/ports/port0", "state":"active"}')

response:
ifm3d_ros2.srv.GetDiag_Response(status=0, msg='{"bootid":"3202350a-a620-40a0-9114-5221bb55b1ff","events":[],"timestamp":1651187568817246656,"version": {"diagnostics":"0.0.11","euphrates":"1.34.32","firmware":"1.4.30.4443"}}')
```