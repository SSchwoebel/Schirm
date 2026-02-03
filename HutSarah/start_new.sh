var="$(ip addr show wlan0|grep inet)"
while [ -z "$var" ]
do
  sleep 1s
  var="$(ip addr show wlan0|grep inet)"
done

sleep 1s

pyro4-ns -n 0.0.0.0 &
python ./test_new.py
