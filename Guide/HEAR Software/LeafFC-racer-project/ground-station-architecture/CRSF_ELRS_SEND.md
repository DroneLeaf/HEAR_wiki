## How to run

On the aircraft-side machine that injects CRSF telemetry:

```bash
python3 send_bbox.py --port /dev/ttyUSB0 --baud 416666 --rate 10 --x 320 --y 180 --w 64 --h 48
```
On the laptop connected to the TX16s USB VCP mirror:

```bash
python3 decode_bbox.py --port /dev/ttyACM0
```

You should see:

```bash
BBOX x=320 y=180 w=64 h=48 (10.0 Hz)
BBOX x=320 y=180 w=64 h=48 (10.0 Hz)
...
```