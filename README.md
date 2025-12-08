
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

NETXION:
    Instruction Set: https://nextion.tech/instruction-set/#s3
                     https://nextion.tech/instruction-set/
    Editor Guide: https://nextion.tech/editor_guide/

Notas para fazer mais tarde:
1. Ver chatgpt, conversa "Escolher exemplo UART" sugestao de separaçao de erros e outros return datas do Nextion de forma estrutura. Isto existe em principio restruturar a parte que está criada no rxFromNextion para o Touch Event
2. Driver L298N pode ser inapropriado para o projeto, ver alternativas como: TOP 1: DRV8825 ou TOP 2: TMC2208 / TMC2209 / TMC2225 ou TOP 3: TB6600/TB67S109 
3. Trocar PINOUTS do codigo stepper_basic_test - OK!!
4. ver video: https://www.youtube.com/watch?v=Bb0Qfj1jdPQ
5. Testes feito na funçao "testing_led" do GPIO_config.c manualmente e com funçao "step_motor" do stepper_basic_test.c
6. Ler documentacao sobre RMT para entender o conceito: https://docs.espressif.com/projects/esp-idf/en/stable/esp32h2/api-reference/peripherals/rmt.html
(nota: o proprio readme.md de como usar o codigo stepper_motor_main_example.c tem explicacao de como usar)
7. Motores de pesquisa academicos: https://scholar.google.com/ and https://ieeexplore.ieee.org/Xplore/guesthome.jsp;jsessionid=D5B2BFACF2DA399E2D5B67D7B38060B3
8. Videos diferença entre Stepper motor NEMA 17, 23 e 34: https://www.youtube.com/watch?v=8wff-IcBTVo
9. Qual é velocidade máxima stepper motor? https://www.youtube.com/watch?v=E7gTkXXCiaQ





