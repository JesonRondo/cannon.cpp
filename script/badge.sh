#!/bin/bash
file_path=./build/cannon_lib
file_size_bytes=$(stat -c %s "$file_path")

file_size_kb=$((file_size_bytes / 1024))

json_content='{
    "lib_size": "'"$file_size_kb"'KB"
}'
echo "$json_content" > badge.json
