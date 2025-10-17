
idf.py create-project -p . <project name>
code .
start idf.py menuconfig
idf.py fullclean or idf.py set-target
idf.py build
idf.py flash
idf.py monitor

More command-lines in: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/idf-py.html

LOGS: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/log.html

Documentation: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html

Adafruit Huzzah32 esp32 official documentation: https://cdn-learn.adafruit.com/downloads/pdf/adafruit-huzzah32-esp32-feather.pdf

GIT command line:
    git status
    git add .
    git commit -m "write text"
    git log //then press "q" to quit
    git push