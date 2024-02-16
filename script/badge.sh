#!/bin/bash

# build
cmake -S . -B build
cmake --build build

# calc
source_file_path=./build/libcannon.a
release_file_path=./build/libcannon-release.a

cp $source_file_path $release_file_path
strip -s "$release_file_path"
gzip -c "$release_file_path" > "$release_file_path.gz"

release_file_size_bytes=$(stat -c %s "$release_file_path")
release_file_size_kb=$((release_file_size_bytes / 1024))

gzip_file_size_bytes=$(stat -c %s "$release_file_path.gz")
gzip_file_size_kb=$((gzip_file_size_bytes / 1024))

json_content='{
    "lib_size": "'"$release_file_size_kb"'KB (Gzipped '"$gzip_file_size_kb"'KB)"
}'
echo "$json_content" > badge.json
