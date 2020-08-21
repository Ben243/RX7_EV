#! /bin/sh

set -e


echo "=> Stopping shutdown listener...\n"
sudo update-rc.d execute_gpio.sh remove
sudo /etc/init.d/execute_gpio.sh stop

echo "=> Removing shutdown listener...\n"
sudo rm -rf /usr/local/bin/gpio.py 
sudo rm -rf /etc/init.d/execute_gpio.sh 

echo "Shutdown listener uninstalled.\n"