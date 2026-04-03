## LeafFC service monitoring

For responsive journalctl monitoring of leafFC.service use:

```conf
[Unit]
Description=DroneLeaf FC launch service.
Requires=roscore.service
After=network-online.target roscore.service

[Service]
User=pi
ExecStartPre=/bin/sleep 4
Environment=ROS_MASTER_URI=http://localhost:11311
Environment=ROS_HOME=/home/pi/.ros
Environment=PYTHONUNBUFFERED=1
Environment=ROSCONSOLE_STDOUT_LINE_BUFFERED=1
ExecStart=/usr/bin/stdbuf -oL -eL /bin/bash -lc 'source /opt/ros/noetic/setup.bash && exec roslaunch --screen flight_controller betaflight_racer.launch'
RemainAfterExit=no
Restart=on-failure
RestartSec=2s
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

Then use the command

```bash
sudo journalctl -u leafFC.service -f -o cat
```
## LeafFC monitoring over network socket

* Create the socket unit 

```bash
sudo tee /etc/systemd/system/leaffc-log-stream.socket >/dev/null <<'EOF'
[Unit]
Description=TCP socket for live leafFC journal stream

[Socket]
ListenStream=0.0.0.0:5555
Accept=yes
NoDelay=true

[Install]
WantedBy=sockets.target
EOF
```

* Create the per-connection service

```bash
sudo tee /etc/systemd/system/leaffc-log-stream@.service >/dev/null <<'EOF'
[Unit]
Description=Live leafFC journal stream to TCP client

[Service]
User=pi
StandardInput=socket
StandardOutput=socket
StandardError=journal
ExecStart=/bin/bash -lc 'exec journalctl -u leafFC.service -f -o cat -n 50'
EOF
```

* What this does:
	* sends the last 50 lines
	* then continues following live output
* Enable it

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now leaffc-log-stream.socket
sudo systemctl status leaffc-log-stream.socket
```

* Make sure the firewall allows it

```bash
sudo ufw allow 5555/tcp
```

On any roblab computer we can use the following:

```bash
nc 10.0.0.107 5555
```

or

```bash
telnet 10.0.0.107 5555
```

to stream the logs