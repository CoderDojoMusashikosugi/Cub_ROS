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
```
