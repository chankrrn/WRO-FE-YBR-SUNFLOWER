#!/bin/bash
# Started by systemd (wro-robot.service) every time the Pi boots.
# The round to run is in /boot/firmware/wro_mode.txt (one line):
#   qualification | final | final --surprise | off
# That file is on the SD card's boot partition, so you can also edit it
# from any laptop with an SD card reader.
MODE_FILE=/boot/firmware/wro_mode.txt
MODE=$(head -n 1 "$MODE_FILE" 2>/dev/null | tr -d '\r')

if [ -z "$MODE" ] || [ "$MODE" = "off" ]; then
    echo "wro_mode.txt says off - robot program not started"
    rfkill unblock wifi
    rfkill unblock bluetooth
    exit 0
fi

# WRO rule 11.10: no Wi-Fi or Bluetooth during a round.
rfkill block wifi
rfkill block bluetooth

cd "$(dirname "$0")/.."      # = src/Raspberrypi
# $MODE is unquoted on purpose, so "final --surprise" becomes two words.
exec .venv/bin/python main.py $MODE
