# RealSense D435i/D405 Stereo IR + Color Recorder

Records synchronized stereo infrared and color frames from Intel RealSense cameras on Raspberry Pi 5, using native kernel modules (V4L2/HID) instead of the RSUSB backend.

## Quick Start

```bash
# Install dependencies
pip install pyrealsense2 opencv-python numpy
pip install av           # optional: for --encode-video MP4 encoding
pip install pyyaml       # optional: for ORB-SLAM YAML generation

# Record (headless SSH)
python record_realsense.py -o /path/to/output --camera realsense_d405 --headless

# Record (with GUI preview)
python record_realsense.py -o /path/to/output --camera realsense_d435i

# Record to MP4 (smallest files)
python record_realsense.py -o /path/to/output --encode-video --headless

# Record to JPEG (smaller than PNG, no encoding step)
python record_realsense.py -o /path/to/output --image-format jpeg --headless
```

Controls: `r` = start recording, `s` = stop, `q` = quit

## Output Structure

```
recordings/
└── 20260604_143052-png/          # session: timestamp + format
    ├── episode_001/
    │   ├── left_000000.png        # left IR
    │   ├── right_000000.png       # right IR
    │   ├── color_000000.png       # color
    │   ├── times.txt              # hardware timestamps
    │   ├── timestamps.json        # timestamps + metadata
    │   └── camera_info_*.json     # intrinsics (ROS camera_info format)
    └── episode_002/
```

With `--encode-video`, images are encoded to `left.mp4`, `right.mp4`, `color.mp4` after each episode and originals are deleted.

---

## Raspberry Pi 5 Setup: Kernel Modules + SDK

The RPi OS kernel disables HID sensor modules needed for IMU, and older kernels lack RealSense video formats. This section covers building the required kernel modules and SDK from source.

Tested on: **Raspberry Pi 5, Kernel 6.12.75+rpt-rpi-2712, Raspberry Pi OS (Debian 13)**

### 1. Install Dependencies

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git pkg-config \
    libusb-1.0-0-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    libssl-dev libelf-dev bison flex bc \
    python3-dev python3-pip python3-venv \
    pybind11-dev python3-pybind11 \
    raspberrypi-kernel-headers
```

### 2. Configure Swap

```bash
sudo apt install -y dphys-swapfile
sudo sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
sudo dphys-swapfile setup && sudo dphys-swapfile swapon
```

### 3. Get Kernel Source

```bash
cd ~/codes
git clone --depth=1 --branch rpi-6.12.y https://github.com/raspberrypi/linux.git rpi-linux
```

If the source version doesn't match `uname -r`, adjust `SUBLEVEL` in the Makefile:

```bash
# Check running kernel
uname -r   # e.g. 6.12.75+rpt-rpi-2712

# Check source version
head -5 ~/codes/rpi-linux/Makefile

# Adjust if needed (e.g. source says 47 but kernel is 75)
sed -i 's/^SUBLEVEL = .*/SUBLEVEL = 75/' ~/codes/rpi-linux/Makefile
```

### 4. Prepare Kernel Source

```bash
cd ~/codes/rpi-linux
make mrproper
cp /lib/modules/$(uname -r)/build/.config .
cp /lib/modules/$(uname -r)/build/Module.symvers .
scripts/config --set-str LOCALVERSION "+rpt-rpi-2712"
make olddefconfig
make modules_prepare

# Verify version matches
cat include/generated/utsrelease.h
# Should show: #define UTS_RELEASE "6.12.75+rpt-rpi-2712"
```

### 5. Build UVC Modules

Kernel 6.12 already includes RealSense video formats (Y16I, RW16, W10) — no patch needed.

```bash
cd ~/codes/rpi-linux
make -j4 M=drivers/media/common
make -j4 M=drivers/media/usb/uvc
```

### 6. Build HID Modules (IMU Support)

RPi OS disables `CONFIG_HID_SENSOR_HUB` by default. Enable and build:

```bash
cd ~/codes/rpi-linux
scripts/config --module HID_SENSOR_HUB
scripts/config --module HID_SENSOR_IIO_COMMON
scripts/config --module HID_SENSOR_IIO_TRIGGER
scripts/config --module HID_SENSOR_ACCEL_3D
scripts/config --module HID_SENSOR_GYRO_3D
make olddefconfig
make modules_prepare

# Build in dependency order
make -j4 M=drivers/hid
grep "hid-sensor-hub" drivers/hid/Module.symvers > /tmp/hid-sensor-hub.symvers
make -j4 M=drivers/iio/common/hid-sensors KBUILD_EXTRA_SYMBOLS=/tmp/hid-sensor-hub.symvers
cat /tmp/hid-sensor-hub.symvers drivers/iio/common/hid-sensors/Module.symvers > /tmp/combined.symvers
make -j4 M=drivers/iio/accel KBUILD_EXTRA_SYMBOLS=/tmp/combined.symvers
make -j4 M=drivers/iio/gyro KBUILD_EXTRA_SYMBOLS=/tmp/combined.symvers
```

### 7. Install Kernel Modules

```bash
KVER=$(uname -r)
SRC=~/codes/rpi-linux
KMOD=/lib/modules/$KVER/kernel

# Remove old compressed modules
sudo rm -f $KMOD/drivers/media/common/uvc.ko.xz
sudo rm -f $KMOD/drivers/media/usb/uvc/uvcvideo.ko.xz

# UVC
sudo cp $SRC/drivers/media/common/uvc.ko $KMOD/drivers/media/common/
sudo cp $SRC/drivers/media/usb/uvc/uvcvideo.ko $KMOD/drivers/media/usb/uvc/

# HID
sudo cp $SRC/drivers/hid/hid-sensor-hub.ko $KMOD/drivers/hid/
sudo mkdir -p $KMOD/drivers/iio/common/hid-sensors
sudo cp $SRC/drivers/iio/common/hid-sensors/hid-sensor-iio-common.ko \
        $SRC/drivers/iio/common/hid-sensors/hid-sensor-trigger.ko \
        $KMOD/drivers/iio/common/hid-sensors/
sudo mkdir -p $KMOD/drivers/iio/accel
sudo cp $SRC/drivers/iio/accel/hid-sensor-accel-3d.ko $KMOD/drivers/iio/accel/
sudo mkdir -p $KMOD/drivers/iio/gyro
sudo cp $SRC/drivers/iio/gyro/hid-sensor-gyro-3d.ko $KMOD/drivers/iio/gyro/

sudo depmod -a
```

### 8. Udev Rules & Permissions

```bash
# From librealsense
cd ~/codes/rasp_realsense/librealsense
./scripts/setup_udev_rules.sh
sudo usermod -aG video $USER

# Disable USB autosuspend for D435i (critical for IMU)
cat << 'EOF' | sudo tee /etc/udev/rules.d/99-realsense-power.rules
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b3a", ATTR{power/control}="on", ATTR{power/autosuspend}="-1"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 9. Build librealsense SDK

```bash
cd ~/codes/rasp_realsense/librealsense
mkdir -p build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_PYTHON_BINDINGS=ON \
    -DPYTHON_EXECUTABLE=$(which python3) \
    -DFORCE_RSUSB_BACKEND=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TOOLS=ON
make -j4
sudo make install
sudo ldconfig
```

`FORCE_RSUSB_BACKEND=OFF` uses native V4L2/HID instead of the unstable RSUSB backend.

### 10. Verify

```bash
sudo modprobe uvcvideo hid-sensor-hub hid-sensor-accel-3d hid-sensor-gyro-3d
lsmod | grep -E "uvc|hid_sensor"
```

```python
import pyrealsense2 as rs
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth)
cfg.enable_stream(rs.stream.accel)
cfg.enable_stream(rs.stream.gyro)
pipe.start(cfg)
for _ in range(10):
    f = pipe.wait_for_frames()
    print("Depth+IMU OK" if f.first_or_default(rs.stream.accel) else "FAIL")
pipe.stop()
print("System ready.")
```

---

## Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| "Disagrees about version of symbol" | Kernel source version mismatch | Adjust `SUBLEVEL` in Makefile to match `uname -r` |
| "loading out-of-tree module taints kernel" | Normal for custom modules | Harmless, ignore |
| "exported twice" during HID build | Passing full `drivers/hid/Module.symvers` to KBUILD_EXTRA_SYMBOLS | Filter with `grep "hid-sensor-hub"` |
| IMU not found | HID modules not loaded | `sudo modprobe hid-sensor-hub hid-sensor-accel-3d hid-sensor-gyro-3d` |
| Video/IMU drops | USB autosuspend | Apply the power management udev rule (step 8) |
| `pyrealsense2` not found | Installed to wrong Python path | Check `sudo make install` output, symlink or add to PYTHONPATH |
| GitHub clone failures (GnuTLS) | Network issues on RPi | Retry, or install deps via apt (`pybind11-dev`) |
| sudo scripts fail with "No such file" | `~` expands to `/root` under sudo | Use absolute paths in scripts |
