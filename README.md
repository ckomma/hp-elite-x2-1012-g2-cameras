# HP Elite x2 1012 G2 - Linux Camera & Windows Hello (Howdy) Suite

[![License: GPL v2](https://img.shields.io/badge/License-GPL%20v2-blue.svg)](LICENSE)
[![Kernel: 6.x / 7.x](https://img.shields.io/badge/Kernel-6.x%20%2F%207.x-orange.svg)]()
[![DKMS: Supported](https://img.shields.io/badge/DKMS-Automated%20Build-green.svg)]()
[![Hardware: HP Elite x2 1012 G2](https://img.shields.io/badge/Hardware-Board%2082CA-lightgrey.svg)]()

Complete kernel driver suite, on-demand bridge daemon, and configuration toolkit enabling **all three physical cameras** and **Windows Hello facial authentication (Howdy)** on the **HP Elite x2 1012 G2** (Motherboard `82CA`) under modern Linux (Ubuntu 24.04+, Debian, Arch).

---

## 📸 Hardware Architecture

The HP Elite x2 1012 G2 incorporates three image sensors and an active Near-Infrared (NIR) illuminator connected via MIPI CSI-2 to an **Intel IPU3 (CIO2)** imaging processor and managed by a **Texas Instruments TPS68470** Power Management IC (PMIC):

| Sensor / Component | Type | Resolution | CSI-2 Port | Function |
| :--- | :--- | :--- | :--- | :--- |
| **OmniVision OV5670** | RGB | 5.0 MP | Port 1 | Front-facing standard webcam |
| **OmniVision OV8858** | RGB | 8.0 MP | Port 0 | Rear-facing high-resolution camera |
| **OmniVision OV7251** | NIR | 640x480 | Port 2 | Front-facing Near-Infrared camera |
| **TPS68470 WLED2** | IR LED | ~850/940 nm | I2C `INT3472:05` | Active IR illuminator for Windows Hello |

### Why out-of-the-box Linux failed:
1. **Proprietary ACPI INT3472 Tables:** Sensor clocks, GPIOs, and power rails are coupled to the TPS68470 PMIC; uninitialized sensors produce I2C NAK (`-121`) errors.
2. **IPU3 CIO2 Port Restrictions:** Mainline `ipu3-cio2` was limited in endpoint parsing, skipping Port 0 (Rear) and Port 2 (NIR).
3. **Missing IR Emitter (WLED2) Driver:** The TPS68470 WLED2 driver defaulted to a 2 ms hardware strobe pulse (`MODE=00`) with 0 mA torch current (`0x2D = 0`), rendering facial recognition impossible in dark environments.
4. **IPU3 DMA Receiver Conflict:** The Intel IPU3 hardware has a single active CSI-2 DMA receiver. If a desktop app inadvertently opens the NIR camera, the RGB cameras become unavailable (`EBUSY`).

---

## 🚀 Key Features of this Suite

* **DKMS Kernel Driver (`hp-camera-unified`):**
  - Fully automated compilation and signing during system kernel updates (`apt upgrade`).
  - Implements unified PMIC sequencing for both OV8858 (Rear) and OV7251 (IR).
  - Software standby (< 10 µW) between capture sessions to eliminate I2C bus lockups.
  - Safe, continuous **hardware Torch mode** (65–97.5 mA) on TPS68470 WLED2 with a kernel safety auto-off watchdog timer.
* **On-Demand V4L2 Bridge (`hp-ir-feeder`):**
  - Lightweight C daemon monitoring `/dev/video20` via `inotify`.
  - Instantly activates WLED2 and streams unpacked IPU3 Raw-10 to BGR24 at 30 FPS when authentication starts.
  - Shuts down hardware within 10 ms as soon as Howdy closes the device.
* **PipeWire / WirePlumber Desktop Isolation:**
  - Hides the NIR loopback device from user-facing apps (GNOME Snapshot, Cheese, Zoom, Chrome).
  - Front RGB (`OV5670`) is the default webcam; Rear RGB (`OV8858`) is selectable via the camera toggle button.
  - Eliminates IPU3 DMA receiver conflicts.
* **Full Windows Hello Facial Authentication (Howdy):**
  - Unlocks in pitch-black darkness using pure active NIR illumination.
  - Average authentication speed: **~1.5 seconds**.
  - Dual PAM integration: `sudo` in terminal and GDM lock screen / login.

---

## 📦 Installation

### 1. Prerequisites
Install kernel headers, DKMS, build tools, and V4L2 loopback:
```bash
# Ubuntu / Debian
sudo apt update
sudo apt install build-essential dkms v4l2loopback-dkms v4l-utils git
```

### 2. Clone and Install
```bash
git clone https://github.com/ckomma/hp-elite-x2-1012-g2-cameras.git
cd hp-elite-x2-1012-g2-cameras
sudo ./install.sh
```

The installer will:
1. Register and build the `hp-camera-unified` DKMS module for your running kernel.
2. Install modprobe and WirePlumber configuration rules.
3. Compile and install `hp-ir-feeder` to `/usr/local/bin/`.
4. Enable and start `hp-camera-unified.service` and `hp-ir-feeder.service`.

---

## 👤 Setting Up Windows Hello Facial Recognition (Howdy)

### 1. Install Howdy
Follow the official Howdy installation instructions for your distribution (e.g. from PPA `boltgolt/howdy` on Ubuntu).

### 2. Configure Howdy
Open Howdy's configuration:
```bash
sudo howdy config
```
Set the following critical parameters:
```ini
[video]
# Point to our dedicated loopback bridge
device_path = /dev/video20

# Set dark threshold to 95 to prevent false rejections in pitch-black rooms
dark_threshold = 95

# Optional: use CNN for maximum accuracy on NIR monochrome images
use_cnn = true

[debug]
end_report = false
```

### 3. Enroll Your Face
Position yourself in front of the tablet and run:
```bash
sudo howdy add
```
*(Tip: Add a second model in low light or with/without glasses using `sudo howdy add`)*

### 4. Enable PAM Integration
#### Sudo in Terminal (`/etc/pam.d/sudo`):
Add the following line right below `@include common-auth`:
```pam
auth sufficient pam_python.so /lib/security/howdy/pam.py
```

#### GDM Login & Screen Unlock (`/etc/pam.d/gdm-password`):
Add the line right above `@include common-auth`:
```pam
auth sufficient pam_python.so /lib/security/howdy/pam.py
```

---

## 🔍 Verification

### Desktop Webcams (Snapshot / Zoom / Browser)
Open **GNOME Snapshot** or **Cheese**. You will see:
- Primary: **Front-facing 5MP RGB camera** (`OV5670`).
- Toggle button: Switch to **Rear-facing 8MP RGB camera** (`OV8858`).

### Infrared Face Authentication
Lock your screen (`Super + L`) or run:
```bash
sudo ls
```
You will see the red IR emitter pulse on the top bezel next to the front camera, and the system will unlock automatically upon recognizing your face.

---

## 🗑️ Uninstallation

To revert all changes and clean up:
```bash
sudo ./uninstall.sh
```

---

## 📄 License
This project is licensed under the **GNU General Public License v2.0** ([GPL-2.0](LICENSE)).
