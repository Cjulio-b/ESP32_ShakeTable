# Plano de Execução: Abordagem Multi-Microcontrolador

Respondendo às tuas preocupações:
1. **Pinos GPIO:** Não podemos usar os pinos do MCP23017 para o UART (porta série), pois a UART necessita de pinos físicos reais do ESP32 ligados aos periféricos internos de hardware. Além disso, o sinal de Sincronismo (SYNC) deve ser instantâneo, e o MCP23017 introduz atrasos de I2C.
   **Solução:** Como sugeriste, vamos aproveitar a tua ideia de agrupar os acelerómetros num só barramento no futuro. Para já, vamos usar os pinos físicos **GPIO 25 (UART TX)** e **GPIO 26 (SYNC)** do MCU1 para ligar ao MCU2.
2. **Build / Compilação:** Não tens de te preocupar! Um projeto ESP-IDF é definido pelo ficheiro `CMakeLists.txt` na raiz. Ao criarmos uma pasta totalmente nova (`C:\Users\cjulio\Documents\cjulio\ESP32_DAQ_Controller`) ao lado da tua pasta atual, os compiladores não se vão misturar. Terás de abrir essa nova pasta no VS Code para compilar o código do MCU2, mantendo tudo 100% separado do MCU1.

---

## Proposed Changes

### Fase 1: Limpeza do MCU1 (O teu projeto atual)
Vamos focar o MCU1 exclusivamente nos motores e HMI.

#### [MODIFY] `main/ESP32_ShakeTable.c`
- **Remover** as funções `csv_writer_task` e `accelerometer_task` e a declaração da Fila (`csv_queue`).
- **Remover** a criação destas tasks no `app_main`.
- **Adicionar Inicialização UART1:** Configurar a UART1 (TX no GPIO 25) com baudrate 115200.
- **Adicionar Inicialização SYNC:** Configurar o GPIO 26 como `GPIO_MODE_OUTPUT`.
- **Lógica de Sincronismo:** Nas funções que iniciam o movimento (ex: dentro da task dos motores ou no parser do Nextion), colocar `gpio_set_level(26, 1);` e enviar a string de configuração pela UART1. Quando o movimento terminar, fazer `gpio_set_level(26, 0);`.

#### [MODIFY] `main/CMakeLists.txt`
- (Opcional) Podemos remover a compilação do `adxl345.c`, pois já não será usado aqui.

---

### Fase 2: Criação do MCU2 (DAQ Controller)
Vou criar uma nova estrutura de pastas e ficheiros de raiz para o MCU2.

#### [NEW] `../ESP32_DAQ_Controller/CMakeLists.txt`
- Configuração do projeto base ESP-IDF.

#### [NEW] `../ESP32_DAQ_Controller/main/main.c`
- **Lógica Principal:**
  - Inicializa o LittleFS, a UART1 (para Receber do MCU1) e o Barramento I2C.
  - Implementa a leitura da UART e o parsing dos parâmetros de teste.
  - O loop principal aguarda que o pino SYNC (ligado do MCU1) vá a HIGH. Quando for a HIGH, abre o ficheiro CSV e arranca um timer para ler os acelerómetros a 100Hz e gravar no disco, calculando o `Target_X` localmente. Quando for a LOW, fecha o ficheiro.

#### [NEW] `../ESP32_DAQ_Controller/main/adxl345.c` e `.h`
- Cópia dos drivers do acelerómetro que usas atualmente.

## Verification Plan

1. Vou alterar o código do **MCU1**. Tu irás compilar e confirmar que o movimento do motor funciona. (Não haverá acelerómetros nem gravação de ficheiros, apenas movimento puro).
2. De seguida, criarei o código do **MCU2**. Podes carregar esse código noutro ESP32 na tua bancada.
3. Faremos as ligações físicas (GND-GND, GPIO25-RX2, GPIO26-SYNC).
4. Correremos um ensaio completo e verificaremos se o MCU2 deteta o arranque e cria o ficheiro CSV com sucesso.
