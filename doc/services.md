
# Advertised Services
| Name    | Service Definition                          | Description                                                                                                               |
| ------- | ------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| Dump    | <a href="srv/Dump.srv">ifm3d/Dump</a>       | Dumps the state of the camera parameters to JSON                                                                          |
| Config  | <a href="srv/Config.srv">ifm3d/Config</a>   | Provides a means to configure the camera and imager settings, declaratively from a JSON encoding of the desired settings. |
| Softon  | <a href="srv/Softon.srv">ifm3d/Softon</a>   | Provides a means to quickly change the camera state from IDLE to RUN.                                                     |
| Softoff | <a href="srv/Softoff.srv">ifm3d/Softoff</a> | Provides a means to quickly change the camera state from RUN to IDLE.                                                     |

## Dump and Config

The ifm3d_ros2 package allows the user to configure the O3R camera platform via two separate ways:  
1. Use ROS native service calls
2. Use the dump and config service proxies

### ROS native service calls

#### Dump
Calling the native ROS service `/ifm3d_ros2/camera/Dump` for a certain `camera` will return the camera configuration as a JSON string. Please notice the use of backslashes (`\` before each `"`) to escape each upper quotation mark. This is necessary to allow us to keep the JSON syntax native to the underlying API (ifm3d).  

Call this service via, e.g. for camera:
```bash
$ ros2 service call /ifm3d/camera/Dump ifm3d_ros2/srv/Dump

1637355550.468025 [0]       ros2: using network interface enp0s31f6 (udp/192.168.0.10) selected arbitrarily from: enp0s31f6, wlp0s20f3, docker0, enx98fc84eebfc8
requester: making request: ifm3d_ros2.srv.Dump_Request()

response:
ifm3d_ros2.srv.Dump_Response(status=0, config='{"device":{"clock":{"currentTime":1581110268783361824},"diagnostic":{"temperatures":[],"upTime":19624000000000},"info":{"device":"0301","deviceTreeBinaryBlob":"tegra186-quill-p3310-1000-c03-00-base.dtb","features":{},"name":"","partNumber":"M03975","productionState":"AA","serialNumber":"000201234159","vendor":"0001"},"network":{"authorized_keys":"","ipAddressConfig":0,"macEth0":"00:04:4B:EA:9F:35","macEth1":"00:02:01:23:41:59","networkSpeed":1000,"staticIPv4Address":"192.168.0.69","staticIPv4Gateway":"192.168.0.201","staticIPv4SubNetMask":"255.255.255.0","useDHCP":false},"state":{"errorMessage":"","errorNumber":""},"swVersion":{"kernel":"4.9.140-l4t-r32.4+gc35f5eb9d1d9","l4t":"r32.4.3","os":"0.13.13-221","schema":"v0.1.0","swu":"0.15.12"}},"ports":{"port0":{"acquisition":{"framerate":10.0,"version":{"major":0,"minor":0,"patch":0}},"data":{"algoDebugConfig":{},"availablePCICOutput":[],"pcicTCPPort":50010},"info":{"device":"2301","deviceTreeBinaryBlobOverlay":"001-ov9782.dtbo","features":{"fov":{"horizontal":127,"vertical":80},"resolution":{"height":800,"width":1280},"type":"2D"},"name":"","partNumber":"M03969","productionState":"AA","sensor":"OV9782","sensorID":"OV9782_127x80_noIllu_Csample","serialNumber":"000000000395","vendor":"0001"},"mode":"experimental_autoexposure2D","processing":{"extrinsicHeadToUser":{"rotX":0.0,"rotY":0.0,"rotZ":0.0,"transX":0.0,"transY":0.0,"transZ":0.0},"version":{"major":0,"minor":0,"patch":0}},"state":"RUN"}}')
```


#### Config
Below you can see an example on how to configure your camera via a ROS service call. The JSON string can be a partial JSON string. It only needs to follow basic JSON syntax. Please wrap the JSON string in a YAML syntax and use the field `"json"`.

```bash
$ ros2 service call /ifm3d/camera/Config ifm3d_ros2/srv/Config "{json: '{\"ports\":{\"port0\":{\"mode\":\"standard_range4m\"}}}'}"
1637355711.266033 [0]       ros2: using network interface enp0s31f6 (udp/192.168.0.10) selected arbitrarily from: enp0s31f6, wlp0s20f3, docker0, enx98fc84eebfc8
requester: making request: ifm3d_ros2.srv.Config_Request(json='{"ports":{"port2":{"mode":"standard_range4m"}}}')

response:
ifm3d_ros2.srv.Config_Response(status=0, msg='OK')
```



### Dump and config service proxies
`ifm3d-ros2` provides access to each camera parameter via the `Dump` and `Config` services exposed by the `camera_node`. 

Additionally, command-line scripts called `dump` and `config` are provided as wrapper interfaces to the native API `ifm3d`. This gives a feel similar to using the underlying C++ API's command-line tool, from the ROS-independent driver except proxying the calls through the ROS network.

For example, to dump the state of the camera:
(exemplary output from an O3R perception platform with one camera head connected is shown)

```bash
$ ros2 run ifm3d_ros2 dump
{
  "device": {
    "clock": {
      "currentTime": 1581193835185485800
    },
    "diagnostic": {
      "temperatures": [],
      "upTime": 103190000000000
    },
    "info": {
      "device": "0301",
      "deviceTreeBinaryBlob": "tegra186-quill-p3310-1000-c03-00-base.dtb",
      "features": {},
      "name": "",
      "  partNumber": "M03975",
      "productionState": "AA",
      "serialNumber": "000201234160",
      "vendor": "0001"
    },
    "network": {
      "authorized_keys": "",
      "ipAddressConfig": 0,
      "macEth0": "00:04:4B:EA:9F:0E",
      "macEth1": "00:02:01:23:41:60",
      "networkSpeed": 1000,
      "staticIPv4Address": "192.168.0.69",
      "staticIPv4Gateway": "192.168.0.201",
      "staticIPv4SubNetMask": "255.255.255.0",
      "useDHCP": false
    },
    "state": {
      "errorMessage": "",
      "errorNumber": ""
    },
    "swVersion": {
      "kernel": "4.9.140-l4t-r32.4+gc35f5eb9d1d9",
      "l4t": "r32.4.3",
      "os": "0.13.13-221",
      "schema": "v0.1.0",
      "swu": "0.15.12"
    }
  },
  "ports": {
    "port0": {
      "acquisition": {
        "framerate": 10,
        "version": {
          "major": 0,
          "minor": 0,
          "patch": 0
        }
      },
      "data": {
        "algoDebugConfig": {},
        "availablePCICOutput": [],
        "pcicTCPPort": 50010
      },
      "info": {
        "device": "2301",
        "deviceTreeBinaryBlobOverlay": "001-ov9782.dtbo",
        "features": {
          "fov": {
            "horizontal": 127,
            "vertical": 80
          },
          "  resolution": {
            "height": 800,
            "width": 1280
          },
          "type": "2D"
        },
        "name": "",
        "partNumber": "M03969",
        "productionState": "AA",
        "sensor": "OV9782",
        "sensorID": "OV9782_127x80_noIllu_Csample",
        "serialNumber": "000000000382",
        "vendor": "0001"
      },
      "mode": "experimental_autoexposure2D",
      "processing": {
        "extrinsicHeadToUser": {
          "rotX": 0,
          "rotY": 0,
          "rotZ": 0,
          "  transX": 0,
          "transY": 0,
          "transZ": 0
        },
        "version": {
          "major": 0,
          "minor": 0,
          "  patch": 0
        }
      },
      "state": "RUN"
    },
    "port2": {
      "acquisition": {
        "exposureLong": 5000,
        "  exposureShort": 400,
        "framerate": 10,
        "offset": 0,
        "version": {
          "major": 0,
          "  minor": 0,
          "patch": 0
        }
      },
      "data": {
        "algoDebugConfig": {},
        "availablePCICOutput": [],
        "pcicTCPPort": 50012
      },
      "info": {
        "device": "3101",
        "deviceTreeBinaryBlobOverlay": "001-irs2381c.dtbo",
        "features": {
          "fov": {
            "horizontal": 105,
            "vertical": 78
          },
          "  resolution": {
            "height": 172,
            "width": 224
          },
          "type": "3D"
        },
        "name": "",
        "partNumber": "M03969",
        "productionState": "AA",
        "sensor": "IRS2381C",
        "sensorID": "IRS2381C_105x78_4x2W_110x90_C7",
        "serialNumber": "000000000382",
        "vendor": "0001"
      },
      "mode": "standard_range4m",
      "processing": {
        "diParam": {
          "anfFilterSizeDiv2": 2,
          "enableDynamicSymmetry": true,
          "enableStraylight": true,
          "enableTemporalFilter": true,
          "excessiveCorrectionThreshAmp": 0.3,
          "excessiveCorrectionThreshDist": 0.08,
          "maxDistNoise": 0.02,
          "maxSymmetry": 0.4,
          "medianSizeDiv2": 0,
          "minAmplitude": 20,
          "minReflectivity": 0,
          "mixedPixelFilterMode": 1,
          "mixedPixelThresholdRad": 0.15
        },
        "extrinsicHeadToUser": {
          "rotX": 1,
          "rotY": 0,
          "rotZ": 0,
          "transX": 100,
          "transY": 0,
          "transZ": 0
        },
        "version": {
          "  major": 0,
          "minor": 0,
          "patch": 0
        }
      },
      "state": "RUN"
    }
  }
}
```

Chaining together Linux pipelines works just as it does in `ifm3d`. For example, using a combination of `dump` and `config` to change the framerate
from 5Hz to 10Hz of the single application on this particular camera would look like:

```bash
$ ros2 run ifm3d_ros2 dump | jq '.ports.port0.mode="standard_range2m"' | ros2 run ifm3d_ros2 config
$ ros2 run ifm3d_ros2 dump | jq .ports.port0.mode
"standard_range2m"
```

:::{note}
If you do not have `jq` on your system, it can be installed with: `$ sudo apt install jq`.
:::

For the `config` command, one difference between our ROS implementation and the `ifm3d` implementation is that we only accept input on `stdin`. So, if you had a file with JSON you wish to configure your camera with, you would simply use the file I/O redirection facilities of your shell (or something like `cat`) to feed the data to `stdin`. For example, in `bash`:

```bash
$ ros2 run ifm3d_ros2 config < camera.json
```