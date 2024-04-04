# Download

Na początku należy zainstalować wymagane programy.

Aby zainstalować aplikację, najlepiej stworzyć środowisko wirtualne (venv) przy użyciu programów takich jak pycharm lub bezpośrednio z konsoli powershell przy pomocy komendy:

```
virtualenv venv
```

Aby wejść do środowiska wirtualnego używajać terminalu należy użyć komendy:

```
source venv/bin/activate
```

Aby Pobrać wymagane biblioteki najszybciej użyć komendy z wbudowanej biblioteki freeze należy użyć komendy:

```
python -m pip install -r requirements.txt
```

Aby uruchomić program należy użyć komendy:

```
python main.py
```

# requirements.txt

Aby wytworzyć plik zawierający potrzebne biblioteki najszybciej użyć komendy z wbudowanej biblioteki freeze

```
python -m pip freeze > requirements.txt
```
