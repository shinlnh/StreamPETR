#!/bin/sh
# THIS FILE WILL BE RAN AUTOMATICALLY BY SYSTEM INIT (SystemV or systemd)
# DO NOT RUN THIS FILE


case "$1" in
    start)
        modprobe ssd1307fb
        ;;
    stop)
        echo "Not support yet!"
        ;;
    status)
        echo "Not support yet!"
        ;;

esac

exit 0
