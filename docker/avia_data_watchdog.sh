#!/bin/bash
# avia (R3LIVE dataset) 自动下载守护：等 pan.baidu.com 恢复，转存+下载
BDUSS="X5aMHRrbWdOb2xNc084WVMzMkZ3VEFiTGFMdUh0T3hnQjhrfllCWH5xVDZFaGxyRVFBQUFBJCQAAAAAAAAAAAEAAADsWMnoAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAPqF8Wr6hfFqS"
STOKEN="5b5143dd58b781a22788b37e008ca9f0c80e713b485e2c47d573eba02558064c"
SURL="1zmVxkcwOSul8oTBwaHfuFg"
LOG="/data0/slam-datasets/avia_watchdog.log"
DEST="/data0/slam-datasets/modality_D"
mkdir -p "$DEST"
echo "$(date) avia watchdog start" >> "$LOG"

while true; do
    # 探测 API
    R=$(curl -s --max-time 25 -b "BDUSS=$BDUSS" -H "User-Agent: netdisk" "https://pan.baidu.com/api/list?dir=%2F&web=1" 2>/dev/null)
    E=$(echo "$R" | python3 -c "import json,sys; print(json.load(sys.stdin).get('errno','?'))" 2>/dev/null)
    if [ "$E" = "0" ]; then
        echo "$(date) API reachable, processing share..." >> "$LOG"
        # 获取分享的 shareid/uk
        PAGE=$(curl -s --max-time 30 -b "BDUSS=$BDUSS" -H "User-Agent: Mozilla/5.0" "https://pan.baidu.com/s/$SURL" 2>/dev/null)
        SHAREID=$(echo "$PAGE" | grep -oE '"shareid":[0-9]+' | head -1 | grep -oE '[0-9]+')
        UK=$(echo "$PAGE" | grep -oE '"uk":[0-9]+' | head -1 | grep -oE '[0-9]+')
        if [ -n "$SHAREID" ] && [ -n "$UK" ]; then
            echo "$(date) shareid=$SHAREID uk=$UK" >> "$LOG"
            # 列出分享文件
            LIST=$(curl -s --max-time 30 -b "BDUSS=$BDUSS" -H "User-Agent: Mozilla/5.0" \
                "https://pan.baidu.com/share/list?shareid=$SHAREID&uk=$UK&dir=%2F" 2>/dev/null)
            echo "$LIST" | python3 -c "
import json, sys
d = json.load(sys.stdin)
for f in d.get('list', []):
    print(f.get('server_filename', f.get('path','?')), f.get('fs_id', ''), f.get('size', 0))
" >> "$LOG" 2>/dev/null
            echo "$(date) share listing done, manual transfer may be needed" >> "$LOG"
        else
            echo "$(date) cannot extract shareid/uk" >> "$LOG"
        fi
        break
    fi
    echo "$(date) API unreachable (errno=$E)" >> "$LOG"
    sleep 1800
done
