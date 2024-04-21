# Teleop_twist_keyboard

```
cd /opt/ros/humble/lib/python3.10/site-packages
```

```
sudo cp teleop_twist_keyboard.py teleop_twist_keyboard_copy.py
```

```
sudo gedit teleop_twist_keyboard.py
```

Usuń całą zawartość przez ctrl + A -> Del, następnie wklej zawartość z pliku 
Piper_Gui/ModyfikacajTopica/teleop_twist_keyboard.py i kliknij save w prawym górnym rogu okna Gedit i wyłącz terminal.


# Download

Na początku należy zainstalować wymagane programy (Ros2 humble itd.).

Następnie należy pobrać repozytorium:
```
cd Desktop/

git clone https://github.com/Dezinter8/Piper_Gui.git
```

Aby zainstalować aplikację, najlepiej stworzyć środowisko wirtualne (venv) przy użyciu programów takich jak pycharm lub bezpośrednio z konsoli powershell przy pomocy komendy:

```
cd Piper_Gui/

virtualenv venv

source venv/bin/activate
```

Aby Pobrać wymagane biblioteki należy użyć komendy:

```
pip install PyQt5 PyQt5-tools vtk opencv-python-headless
```

Aby uruchomić program należy użyć komendy:

```
python main.py
```
