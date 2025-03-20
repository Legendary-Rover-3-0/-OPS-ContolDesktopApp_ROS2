# Przygotowanie środowiska

Zainstalowany ROS2

```
pip install -r requirements.txt
sudo apt-get install sshpass
```
# Uruchomienie
```
source /opt/ros/humble/setup.bash
python3 main.py
```


# Ważne uwagi
- sprawdź czy w na dole pliku science_tab.py i gps_tab.py, w które uruchamiają osobne aplikacje w kodzie znajdue się "python" czasami trzeba zmienić to na "python3"
- w pliku config zmień adres IP na swój (domyślnie jest 192.168.2.10, dla Stacji Operatorskiej)
- w przypdaku gdy nie masz komunikacji między bulletami sprawdź jaki masz adres IP, a także czy adres ip Jetsona jest 192.168.2.100 (przy komunikcji przez bullety)
- można zmiejszyć rozdzelczość kamerek w wizji w pliku config.py
- można zmienić domyślną wartość dla wartości początkowej manipulatora w config.py
