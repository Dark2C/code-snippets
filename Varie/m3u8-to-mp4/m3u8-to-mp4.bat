@echo off
set m3u8="%~1%"
rem se m3u8 non è stato specificato (ovvero se è uguale alla stringa composta da due doppi apici), esco dallo script e stampo un messaggio di errore
if %m3u8%=="" (
    echo m3u8 non specificato
    pause
    exit /b 1
)
set mp4=%m3u8:.m3u8=.mp4%
ffmpeg -i %m3u8% -bsf:a aac_adtstoasc -c copy %mp4%