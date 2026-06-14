# handy2

## pps
```
sudo apt install pps-tools
sudo setcap cap_sys_time+ep $(which ppstest)
ppstest /dev/pps0
```

## chrony
```
sudo service chrony start
chronyc sources -v
```

## ホワイトバランス調整
```
ros2 param set /camera_node AwbEnable true
ros2 topic echo /diagnostics | grep "ColourGains" -A1

'[ 2.560000, 2.080000 ]' だったら

ros2 param set /camera_node AwbEnable false
ros2 param set /camera_node ColourGains [ 2.560000, 2.080000 ]

```

## 露出調整
```
ros2 param set /camera_node AnalogueGain 20.0
ros2 param get /camera_node AnalogueGain
```

## カメラキャリブレーション
```
ros2 run camera_calibration cameracalibrator --size 6x9 --square 0.0233 --no-service-check --ros-args -r image:=/camera_node/image_raw
tar -xvf /tmp/calibrationdata.tar.gz
cat ost.yaml
```