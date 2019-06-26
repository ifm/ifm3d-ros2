
ifm3d0-ros2: Dump and Config
============================

`ifm3d-ros2` provides access to the camera/imager parameters via the `Dump` and
`Config` services exposed by the `camera_node`. Additionally, command-line
scripts called `dump` and `config` are provided as wrapper interfaces to those
services. This gives a feel similar to using the underlying `ifm3d`
command-line tool from the ROS-independent driver except proxying the calls
through the ros network.

For example, to dump the state of the camera:

(exemplary output from an O3D303 is shown)

```
$ ros2 run ifm3d_ros2 dump
{
    "ifm3d": {
        "Apps": [
            {
                "Description": "Full-layer Depalletization",
                "Id": "1999576749",
                "Imager": {
                    "AutoExposureMaxExposureTime": "10000",
                    "AutoExposureReferencePointX": "88",
                    "AutoExposureReferencePointY": "66",
                    "AutoExposureReferenceROI": "{\"ROIs\":[{\"id\":0,\"group\":0,\"type\":\"Rect\",\"width\":130,\"height\":100,\"angle\":0,\"center_x\":88,\"center_y\":66}]}",
                    "AutoExposureReferenceType": "0",
                    "Channel": "0",
                    "ClippingBottom": "131",
                    "ClippingCuboid": "{\"XMin\": -3.402823e+38, \"XMax\": 3.402823e+38, \"YMin\": -3.402823e+38, \"YMax\": 3.402823e+38, \"ZMin\": -3.402823e+38, \"ZMax\": 3.402823e+38}",
                    "ClippingLeft": "0",
                    "ClippingRight": "175",
                    "ClippingTop": "0",
                    "ContinuousAutoExposure": "false",
                    "ContinuousUserFrameCalibration": "false",
                    "EnableAmplitudeCorrection": "true",
                    "EnableFastFrequency": "false",
                    "EnableFilterAmplitudeImage": "true",
                    "EnableFilterDistanceImage": "true",
                    "EnableRectificationAmplitudeImage": "false",
                    "EnableRectificationDistanceImage": "false",
                    "ExposureTime": "5000",
                    "ExposureTimeList": "125;5000",
                    "ExposureTimeRatio": "40",
                    "FrameRate": "5",
                    "MaxAllowedLEDFrameRate": "11.9",
                    "MinimumAmplitude": "42",
                    "Resolution": "0",
                    "SpatialFilter": {},
                    "SpatialFilterType": "0",
                    "SymmetryThreshold": "0.4",
                    "TemporalFilter": {},
                    "TemporalFilterType": "0",
                    "ThreeFreqMax2FLineDistPercentage": "80",
                    "ThreeFreqMax3FLineDistPercentage": "80",
                    "TwoFreqMaxLineDistPercentage": "80",
                    "Type": "upto30m_moderate",
                    "UseSimpleBinning": "false"
                },
                "Index": "1",
                "LogicGraph": "{\"IOMap\": {\"OUT1\": \"RFT\",\"OUT2\": \"AQUFIN\"},\"blocks\": {\"B00001\": {\"pos\": {\"x\": 200,\"y\": 200},\"properties\": {},\"type\": \"PIN_EVENT_IMAGE_ACQUISITION_FINISHED\"},\"B00002\": {\"pos\": {\"x\": 200,\"y\": 75},\"properties\": {},\"type\": \"PIN_EVENT_READY_FOR_TRIGGER\"},\"B00003\": {\"pos\": {\"x\": 600,\"y\": 75},\"properties\": {\"pulse_duration\": 0},\"type\": \"DIGITAL_OUT1\"},\"B00005\": {\"pos\": {\"x\": 600,\"y\": 200},\"properties\": {\"pulse_duration\": 0},\"type\": \"DIGITAL_OUT2\"}},\"connectors\": {\"C00000\": {\"dst\": \"B00003\",\"dstEP\": 0,\"src\": \"B00002\",\"srcEP\": 0},\"C00001\": {\"dst\": \"B00005\",\"dstEP\": 0,\"src\": \"B00001\",\"srcEP\": 0}}}",
                "Name": "dpal",
                "PcicEipResultSchema": "{ \"layouter\": \"flexible\", \"format\": { \"dataencoding\": \"binary\", \"order\": \"big\" }, \"elements\" : [ { \"type\": \"string\", \"value\": \"star\", \"id\": \"start_string\" }, { \"type\": \"records\", \"id\": \"models\", \"elements\": [ { \"type\": \"int16\", \"id\": \"boxFound\" }, { \"type\": \"int16\", \"id\": \"width\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"height\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"length\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"xMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"yMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"zMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"yawAngle\" }, { \"type\": \"int16\", \"id\": \"qualityLength\" }, { \"type\": \"int16\", \"id\": \"qualityWidth\" }, { \"type\": \"int16\", \"id\": \"qualityHeight\" } ] }, { \"type\": \"string\", \"value\": \"stop\", \"id\": \"end_string\" } ] }",
                "PcicPnioResultSchema": "{\"layouter\" : \"flexible\", \"format\": { \"dataencoding\": \"binary\", \"order\": \"big\" }, \"elements\" : [ { \"type\": \"string\", \"value\": \"star\", \"id\": \"start_string\" }, { \"type\": \"records\", \"id\": \"models\", \"elements\": [ { \"type\": \"int16\", \"id\": \"boxFound\" }, { \"type\": \"int16\", \"id\": \"width\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"height\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"length\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"xMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"yMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"zMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"yawAngle\" }, { \"type\": \"int16\", \"id\": \"qualityLength\" }, { \"type\": \"int16\", \"id\": \"qualityWidth\" }, { \"type\": \"int16\", \"id\": \"qualityHeight\" } ] }, { \"type\": \"string\", \"value\": \"stop\", \"id\": \"end_string\" } ] }",
                "PcicTcpResultSchema": "{ \"layouter\": \"flexible\", \"format\": { \"dataencoding\": \"ascii\" }, \"elements\": [ { \"type\": \"string\", \"value\": \"star\", \"id\": \"start_string\" }, { \"type\": \"blob\", \"id\": \"normalized_amplitude_image\" }, { \"type\": \"blob\", \"id\": \"distance_image\" }, { \"type\": \"blob\", \"id\": \"x_image\" }, { \"type\": \"blob\", \"id\": \"y_image\" }, { \"type\": \"blob\", \"id\": \"z_image\" }, { \"type\": \"blob\", \"id\": \"confidence_image\" }, { \"type\": \"blob\", \"id\": \"diagnostic_data\" }, { \"type\": \"string\", \"value\": \"stop\", \"id\": \"end_string\" } ] }",
                "RtspOverlayStyle": "{\"ROI\": {\"default\": {\"visible\": true, \"filled\": false, \"use_symbol\": false, \"label_alignment\": \"top\", \"label_content\": \"\", \"label_background\": \"black\", \"label_fontsize\": 8, \"label_failonly\": false}, \"model_defaults\": {}, \"specific\": {} } }",
                "TemplateInfo": "",
                "TriggerMode": "1",
                "Type": "Camera"
            }
        ],
        "Device": {
            "ActiveApplication": "1",
            "ArticleNumber": "O3D303",
            "ArticleStatus": "AD",
            "Description": "Goldeneye_1.5.371-patched.swu",
            "DeviceType": "1:2",
            "EIPConsumingSize": "8",
            "EIPProducingSize": "450",
            "EnableAcquisitionFinishedPCIC": "false",
            "EthernetFieldBus": "1",
            "EthernetFieldBusEndianness": "0",
            "EvaluationFinishedMinHoldTime": "10",
            "ExtrinsicCalibRotX": "0",
            "ExtrinsicCalibRotY": "0",
            "ExtrinsicCalibRotZ": "0",
            "ExtrinsicCalibTransX": "0",
            "ExtrinsicCalibTransY": "0",
            "ExtrinsicCalibTransZ": "0",
            "IODebouncing": "true",
            "IOExternApplicationSwitch": "0",
            "IOLogicType": "1",
            "IPAddressConfig": "0",
            "ImageTimestampReference": "1538395793",
            "Name": "My Camera",
            "OperatingMode": "0",
            "PNIODeviceName": "",
            "PasswordActivated": "false",
            "PcicProtocolVersion": "3",
            "PcicTcpPort": "50010",
            "PcicTcpSchemaAutoUpdate": "false",
            "SaveRestoreStatsOnApplSwitch": "true",
            "ServiceReportFailedBuffer": "15",
            "ServiceReportPassedBuffer": "15",
            "SessionTimeout": "30",
            "TemperatureFront1": "3276.7",
            "TemperatureFront2": "3276.7",
            "TemperatureIMX6": "62.2599983215332",
            "TemperatureIllu": "66.9",
            "UpTime": "2.76944444444444"
        },
        "Net": {
            "MACAddress": "00:02:01:40:97:AB",
            "NetworkSpeed": "0",
            "StaticIPv4Address": "192.168.0.69",
            "StaticIPv4Gateway": "192.168.0.201",
            "StaticIPv4SubNetMask": "255.255.255.0",
            "UseDHCP": "false"
        },
        "Time": {
            "CurrentTime": "1538395791",
            "NTPServers": "",
            "StartingSynchronization": "false",
            "Stats": "",
            "SynchronizationActivated": "false",
            "Syncing": "false",
            "WaitSyncTries": "2"
        },
        "_": {
            "Date": "Wed Jun 26 15:00:03 2019",
            "HWInfo": {
                "Connector": "#!02_A300_C40_03408592_008023176",
                "Diagnose": "#!02_D322_C34_03370362_008026824",
                "Frontend": "#!02_F342_C34_18_00017_008023607",
                "Illumination": "#!02_I300_001_03423860_008001175",
                "MACAddress": "00:02:01:40:97:AB",
                "Mainboard": "#!02_M381_C41_03144276_008023690",
                "MiraSerial": "0030-608d-0170-064c"
            },
            "SWVersion": {
                "Algorithm_Version": "2.1.9",
                "Calibration_Device": "00:02:01:40:97:ab",
                "Calibration_Version": "0.9.0",
                "Diagnostic_Controller": "v1.0.77-d1383f8f56-dirty",
                "IFM_Recovery": "unversioned",
                "IFM_Software": "1.23.2848",
                "Linux": "Linux version 3.14.34-rt31-yocto-standard-00010-g82eeb38179d5-dirty (jenkins@dettlx190) (gcc version 4.9.2 (GCC) ) #1 SMP PREEMPT RT Wed Sep 12 09:50:11 CEST 2018",
                "Main_Application": "1.0.77"
            },
            "ifm3d_version": 1300
        }
    }
}
```

Chaining together Linux pipelines works just as it does in `ifm3d`. For
example, using a combination of `dump` and `config` to change the frame rate
from 5Hz to 10Hz of the single application on this particular camera would look
like:

```
$ ros2 run ifm3d_ros2 dump | jq '.ifm3d.Apps[0].Imager.FrameRate="10"' | ros2 run ifm3d_ros2 config
```

You can check that it worked with:

```
$ ros2 run ifm3d_ros2 dump | jq .ifm3d.Apps[0].Imager.FrameRate
"10"
```

**NOTE:** If you do not have `jq` on your system, it can be installed with: `$ sudo apt install jq`

For the `config` command, one difference between our ROS implementation and the
`ifm3d` implementation is that we only accept input on `stdin`. So, if you had
a file with JSON you wish to configure your camera with, you would simply use the
file I/O redirection facilities of your shell (or something like `cat`) to feed
the data to `stdin`. For example, in `bash`:

```
$ ros2 run ifm3d_ros2 config < camera.json
```

Beyond the requirement of prefacing your command-line with `ros2 run ...` to invoke
the ROS version of these tools, they operate the same. To learn more about the
functionality and concepts, you can read the docs
[here](https://github.com/lovepark/ifm3d/blob/master/doc/configuring.md).
