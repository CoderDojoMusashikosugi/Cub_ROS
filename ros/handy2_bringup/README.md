# handy2

## pps
```
sudo apt install pps-tools
sudo setcap cap_sys_time+ep $(which ppstest)
ppstest /dev/pps0
```

# chrony
```
sudo service chrony start
chronyc sources -v
```