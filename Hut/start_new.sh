var="$(ip addr show wlan0|grep inet)"
while [ -z "$var" ]
do
  sleep 1s
  var="$(ip addr show wlan0|grep inet)"
done

sleep 15s

pyro4-ns -n 0.0.0.0 &
python /home/pi/Hut/test_new.py
