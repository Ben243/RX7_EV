#! /bin/sh

set -e

echo "=> Installing gpio executer...\n"
sudo cp gpio.py /usr/local/bin/
sudo chmod +x /usr/local/bin/gpio.py

echo "=> Starting gpio executer...\n"
sudo cp execute_gpio.sh /etc/init.d/
sudo chmod +x /etc/init.d/execute_gpio.sh

sudo update-rc.d execute_gpio.sh defaults
sudo /etc/init.d/execute_gpio.sh start

echo "Shutdown listener installed.