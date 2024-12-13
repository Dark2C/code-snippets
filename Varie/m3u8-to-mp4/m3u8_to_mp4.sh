#!/bin/bash

SOURCE_DIR=$1
DEST_DIR=$2

if [ "$(ls -A "$SOURCE_DIR")" ]; then
    file=$(ls -1 "$SOURCE_DIR" | head -n 1)
    mv "$SOURCE_DIR/$file" "$DEST_DIR/processing/"
    if [[ $file == *.m3u8 ]]; then
        docker run --rm -it -v "$DEST_DIR":/config linuxserver/ffmpeg -protocol_whitelist file,http,https,tcp,tls,crypto -i "/config/processing/$file" -bsf:a aac_adtstoasc -c copy "/config/processing/${file%.*}.mp4"
        mv "$DEST_DIR/processing/${file%.*}.mp4" "$DEST_DIR/processed/"
        rm "$DEST_DIR/processing/$file"
    else
        exit
    fi
else
    exit
fi