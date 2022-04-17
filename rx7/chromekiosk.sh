#/bin/bash

if pidof chromium-browser > /dev/null
then
	echo Chrome already running
else
DISPLAY=:0 chromium-browser --kiosk --start-fullscreen --disable-pinch --incognito http://localhost:1880/ui
fi
