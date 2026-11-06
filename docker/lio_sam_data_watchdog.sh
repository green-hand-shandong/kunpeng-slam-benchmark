#!/bin/bash
# LIO-SAM 官方 sample 数据下载守护：每小时探测一次 Google Drive，通了自动下载 walking 数据集
FOLDER_ID="1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq"
DEST="/data0/slam-datasets/lio_sam_samples"
LOG="/data0/slam-datasets/lio_sam_watchdog.log"
mkdir -p "$DEST"
echo "$(date) watchdog start" >> "$LOG"
while true; do
    if timeout 30 curl -sI "https://drive.google.com" > /dev/null 2>&1; then
        echo "$(date) drive reachable, downloading..." >> "$LOG"
        gdown --folder "https://drive.google.com/drive/folders/$FOLDER_ID" -O "$DEST" >> "$LOG" 2>&1 && \
            echo "$(date) download success" >> "$LOG" && break
        echo "$(date) download failed, retry next hour" >> "$LOG"
    else
        echo "$(date) drive unreachable" >> "$LOG"
    fi
    sleep 3600
done
