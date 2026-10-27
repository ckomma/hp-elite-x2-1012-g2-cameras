#!/bin/bash
set -euo pipefail
STATE=/var/lib/hp-camera-unified

manage_user_pipewire() {
  local action="$1"
  for u in $(loginctl list-users --no-legend 2>/dev/null | awk '{print $1}'); do
    [ "$u" -ge 1000 ] 2>/dev/null || continue
    systemctl --user -M "${u}@" "$action" pipewire.socket pipewire.service wireplumber.service 2>/dev/null || true
  done
}

case "${1:-}" in
  supported)
    test "$(cat /sys/class/dmi/id/board_name 2>/dev/null)" = 82CA
    ;;
  start)
    mkdir -p "$STATE"; chmod 700 "$STATE"
    
    # 1. Load fixture if not loaded
    if [ ! -d /sys/module/hp_camera_fixture ]; then
      if [ ! -d /sys/module/ipu3_cio2 ]; then modprobe ipu3-cio2 || true; fi
      modprobe hp_camera_fixture run=1
      for ((i=0;i<40;i++)); do
        result=$(cat /sys/module/hp_camera_fixture/parameters/result 2>/dev/null || echo -115)
        [ "$result" != -115 ] && break
        sleep 0.2
      done
      test "$result" = 0
    fi

    # 2. Load sensor drivers
    if [ ! -d /sys/module/hp_ov8858 ]; then modprobe hp_ov8858; fi
    if [ ! -d /sys/module/hp_ov7251 ]; then modprobe hp_ov7251; fi

    # 3. Load 3-camera CIO2 driver (ensure hp_elite_x2_front_only is not Y)
    if [ ! -d /sys/module/ipu3_cio2 ] || [ "$(cat /sys/module/ipu3_cio2/parameters/hp_elite_x2_all_cameras 2>/dev/null)" != "Y" ] || [ "$(cat /sys/module/ipu3_cio2/parameters/hp_elite_x2_front_only 2>/dev/null)" = "Y" ]; then
      manage_user_pipewire stop
      rmmod ipu3_cio2 || true
      modprobe ipu3-cio2 hp_elite_x2_all_cameras=1 hp_elite_x2_front_only=0 hp_elite_x2_front_ir=0
      udevadm settle --timeout=10
      manage_user_pipewire start
    fi

    # 4. Configure media pipeline and subdevs
    SUB=''
    for n in /sys/class/video4linux/v4l-subdev*; do
      [[ $(readlink -f "$n/device" 2>/dev/null) == $(readlink -f /sys/bus/i2c/devices/2-0060 2>/dev/null) ]] || continue
      SUB=/dev/${n##*/}
      SENSOR=$(cat "$n/name")
      break
    done

    if [ -n "$SUB" ]; then
      for m in /dev/media*; do
        if media-ctl -d "$m" -p | grep -Fq "$SENSOR"; then
          MEDIA=$m
          break
        fi
      done
      if [ -n "$MEDIA" ]; then
        media-ctl -d "$MEDIA" -l "\"$SENSOR\":0 -> \"ipu3-csi2 2\":0[1]" 2>/dev/null || true
        media-ctl -d "$MEDIA" -V "\"$SENSOR\":0 [fmt:Y10_1X10/640x480 field:none]" 2>/dev/null || true
        media-ctl -d "$MEDIA" -V '"ipu3-csi2 2":0 [fmt:Y10_1X10/640x480 field:none]' 2>/dev/null || true
        v4l2-ctl -d "$SUB" --set-subdev-fps pad=0,stream=0,fps=30 2>/dev/null || true
        v4l2-ctl -d "$SUB" --set-ctrl 'test_pattern=0,exposure=1500,gain=64' 2>/dev/null || true
      fi
    fi

    # 5. Make sure v4l2loopback is loaded
    if [ ! -d /sys/module/v4l2loopback ]; then
      modprobe v4l2loopback video_nr=20 card_label="HP-IR-Camera" exclusive_caps=0
    fi

    cam -l >"$STATE/cameras-start.txt" 2>&1 || true
    echo "HP_CAMERA_UNIFIED started successfully."
    ;;
  stop)
    manage_user_pipewire stop
    if [ -d /sys/module/ipu3_cio2 ]; then rmmod ipu3_cio2 || true; fi
    if [ -d /sys/module/hp_ov7251 ]; then rmmod hp_ov7251 || true; fi
    if [ -d /sys/module/hp_ov8858 ]; then rmmod hp_ov8858 || true; fi
    if [ -d /sys/module/hp_camera_fixture ]; then rmmod hp_camera_fixture || true; fi
    manage_user_pipewire start
    echo "HP_CAMERA_UNIFIED stopped successfully."
    ;;
  *)
    exit 2
    ;;
esac
