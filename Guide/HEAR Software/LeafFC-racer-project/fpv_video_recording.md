# FPV Record

Record video from analogue (V4L2) or digital (RTSP) camera sources on DroneLeaf target machines. Recordings run inside a `tmux` session for easy monitoring and use fragmented MP4 to survive crashes and power loss.

## Installation

```bash
hear-cli target copy_run_program_term --p fpv_record_install
```

> [!NOTE]  
> To test the recording feature on your local machine run `hear-cli local_machine run_program --p fpv_record_install`


This installs `ffmpeg`, `tmux`, and places two commands in `~/.local/bin/`:

- `fpv-record-digital` — record from an RTSP camera
- `fpv-record-analog` — record from a V4L2 capture device

After installation, open a new terminal or run `source ~/.bashrc`.

## Usage

### Digital camera (RTSP)

```bash
# High quality (stream=0, default)
fpv-record-digital

# Low quality (stream=1)
fpv-record-digital --quality low

# Custom RTSP source
fpv-record-digital --source "rtsp://user:pass@10.0.0.5/stream=0"

# Stop recording
fpv-record-digital --stop
```

| Option | Values | Default |
|--------|--------|---------|
| `--quality` | `high`, `low` | `high` |
| `--source` | Any RTSP URL | `rtsp://root:12345@192.168.1.10/stream=0` (high) or `stream=1` (low) |
| `--stop` | — | — |

### Analogue camera (V4L2)

```bash
# Default device /dev/video0
fpv-record-analog

# Custom device
fpv-record-analog --source /dev/video2

# Stop recording
fpv-record-analog --stop
```

| Option | Values | Default |
|--------|--------|---------|
| `--source` | V4L2 device path | `/dev/video0` |
| `--stop` | — | — |

## Output

Recordings are saved to:

```
~/videos/recording-YYYYMMDD-HHMMSS-digital.mp4
~/videos/recording-YYYYMMDD-HHMMSS-analog.mp4
```

## How it works

- Both commands launch `ffmpeg` inside a shared tmux session named `fpv-record`.
- Only **one recording at a time** is allowed. Starting a new recording automatically stops any existing one.
- `--stop` sends `q` to ffmpeg for a clean shutdown, then kills the tmux session.
- You can inspect the live ffmpeg output at any time:
  ```bash
  tmux attach -t fpv-record
  ```
  Detach with `Ctrl+B` then `D`.

### Debugging with tmux

The recording runs inside a tmux session. Use these commands to inspect and manage it:

```bash
# List all tmux sessions (check if a recording is running)
tmux ls

# Attach to the recording session to see live ffmpeg output
tmux attach -t fpv-record

# Detach from the session (keeps recording running)
# Press: Ctrl+B then D

# Kill the recording session manually
tmux kill-session -t fpv-record

# If tmux reports "no server running", there is no active session
```

> [!NOTE]  
> **tmux key strokes cheat sheet**
>
> | Action | Keys | Description |
> |--------|------|-------------|
> | Attach | Type `tmux attach -t fpv-record` + `Enter` | Connects your terminal to the recording session |
> | Detach | `Ctrl+B` then `D` | Disconnects you from the session — recording keeps running in background |
> | Scroll up | `Ctrl+B` then `[` | Enters scroll mode. Use `↑`/`↓` or `PgUp`/`PgDn` to navigate. Press `Q` to exit scroll mode |
> | Stop ffmpeg | `Q` | While attached, press `Q` to tell ffmpeg to stop gracefully and finalise the file |
> | Kill session | `Ctrl+B` then `X`, then `Y` | Force-kills the current pane (and the session if it's the only one) |
>
> **Important:** `Ctrl+B` is the tmux **prefix key** — press it first, release, *then* press the next key. Do not hold them together.

**Common scenarios:**

| Symptom | Action |
|---------|--------|
| Recording started but no output file growing | `tmux attach -t fpv-record` — check ffmpeg error messages |
| `fpv-record-*` says "Stopping existing recording..." every time | `tmux ls` to see stale sessions, then `tmux kill-session -t fpv-record` |
| ffmpeg shows `Connection refused` or `timeout` | Camera is unreachable — check network/cable and `--source` URL |
| ffmpeg shows `No such file or directory` for `/dev/videoX` | Device not connected — run `ls /dev/video*` to find available devices |
| Session exists but ffmpeg already exited | Attach and press Enter to close, or `tmux kill-session -t fpv-record` |

### Crash recovery

Both commands write **fragmented MP4** (`-movflags +frag_keyframe+empty_moov+default_base_moof`). The file header (moov atom) is written at the start and video data is flushed in keyframe-aligned fragments. If the process dies, the computer reboots, or power is lost, the recording up to the last flushed fragment is intact and playable.

## Technical details

**Digital (RTSP):**
```
ffmpeg -rtsp_transport tcp -max_delay 0 -stimeout 5000000 -buffer_size 2097152
       -fflags nobuffer+discardcorrupt+genpts
       -i <SOURCE>
       -c copy -map 0 -copyts -start_at_zero -avoid_negative_ts make_zero
       -movflags +frag_keyframe+empty_moov+default_base_moof
       <OUTPUT>
```

**Analogue (V4L2):**
```
ffmpeg -f v4l2 -input_format mjpeg -video_size 640x480
       -i <SOURCE>
       -c:v libx264 -preset ultrafast -pix_fmt yuv420p
       -movflags +frag_keyframe+empty_moov+default_base_moof
       <OUTPUT>
```

## Dependencies

Installed automatically by the installer:

- `ffmpeg` — video recording/encoding
- `tmux` — persistent terminal session

## Files

| File | Description |
|------|-------------|
| `main.sh` | HEAR_CLI installer program |
| `parameters.json` | Program parameters |
| `fpv-record-digital` | Digital camera recording script |
| `fpv-record-analog` | Analogue camera recording script |
