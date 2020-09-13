
#! /bin/sh

### BEGIN INIT INFO
# Provides:          app.py
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
### END INIT INFO

# If you want a command to always run, put it here

# Carry out specific functions when asked to by the system
# todo flask run this 
case "$1" in
  start)
    echo "Starting gpio.py"
    python3 /home/pi/RX7_EV/app.py &
    ;;
  stop)
    echo "Stopping gpio.py"
    pkill -f /home/pi/RX7_EV/app.py
    ;;
  *)
    echo "Usage: /etc/init.d/execute_gpio.sh {start|stop}"
    exit 1
    ;;
esac

exit 0