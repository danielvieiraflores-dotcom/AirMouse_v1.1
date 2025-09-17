# AirMouse ESP32 + MPU-6050

Este projeto transforma um ESP32 com o módulo MPU-6050 em um mouse aéreo com filtros avançados e detecção automática de orientação. O firmware lê dados do giroscópio/acelerômetro, aplica uma cadeia de filtros para suavizar o movimento e envia os valores normalizados pela porta serial. Um script Python no computador (`gyro_mouseCOMbotao.py`) traduz essas mensagens em comandos reais de mouse.

## Requisitos

### Hardware
- ESP32 com comunicação I2C nos pinos GPIO 21 (SDA) e GPIO 22 (SCL).
- Módulo MPU-6050 (GY-521).
- Botão tátil ligado ao GPIO 15 para clique (utiliza o resistor de pull-up interno).
- Cabos jumper e alimentação estável de 3,3 V para o sensor.

### Software
- [PlatformIO](https://platformio.org/) ou Arduino IDE para compilar e gravar o firmware.
- Python 3 com as dependências do script `gyro_mouseCOMbotao.py` para mapear os dados seriais em eventos de mouse.

## Ligações de Hardware

O projeto utiliza I2C para conversar com o MPU-6050. Utilize a pinagem padrão do ESP32.

| MPU6050 Pin | ESP32 Pin | Finalidade         |
|-------------|-----------|--------------------|
| VCC         | 3.3V      | Alimentação        |
| GND         | GND       | Referência comum   |
| SCL         | GPIO 22   | Clock do barramento|
| SDA         | GPIO 21   | Dados do barramento|

### Botão

- Conecte um terminal ao GND e o outro ao GPIO 15.
- O pino é configurado como `INPUT_PULLUP` pelo firmware, dispensando resistores externos.

## Como o firmware funciona

- Frequência alvo de amostragem: 100 Hz para leituras consistentes do sensor.
- Calibração automática do giroscópio em dois momentos:
  - Na inicialização (com o dispositivo parado).
  - Automaticamente sempre que detectar repouso por mais de 2 segundos.
- Recuperação automática da comunicação I2C em caso de falhas.
- Cadeia de filtros para suavizar o movimento:
  - Dead zone inteligente com transição suave.
  - Sensibilidade adaptativa com três zonas (precisão, normal e rápida).
  - Limitação de “jerk” para evitar saltos bruscos.
  - Filtro de suavização exponencial com fator ajustado (`SMOOTHING_ALPHA`).
- Detecção automática de orientação (horizontal, vertical, invertida, esquerda e direita) com temporização para evitar falsos positivos.
- Supressão de inclinação involuntária do punho para manter o movimento horizontal estável.
- Leitura do estado do botão no GPIO 15 para enviar cliques ao computador.

## Saídas seriais

O firmware publica mensagens no formato:

```
X:<valor> Y:<valor> B:<0 ou 1>
```

- `X` e `Y` representam os deslocamentos processados do mouse.
- `B` indica o estado do botão (1 = pressionado, 0 = solto).
- Mensagens informativas adicionais iniciam com `I:` (por exemplo, `I:ORIENTACAO_HORIZONTAL` ou `I:RECUPERANDO_I2C`) e auxiliam no debug.

## Fluxo de uso

1. Monte o hardware e conecte o ESP32 ao computador.
2. Carregue o firmware com PlatformIO ou Arduino IDE.
3. Abra um monitor serial a 115200 bps para confirmar as mensagens de inicialização e garantir que a calibração foi concluída com o dispositivo parado.
4. Execute `gyro_mouseCOMbotao.py` para transformar os dados seriais em movimentos reais do mouse.
5. Teste os diferentes modos de orientação segurando o dispositivo em posições distintas.

## Dicas de operação

- Mantenha o sensor imóvel durante a calibração inicial e sempre que desejar forçar uma nova calibração (deixe-o parado por alguns segundos).
- Caso a comunicação I2C falhe, o firmware tentará reinicializar automaticamente; verifique cabeamento e alimentação.
- Ajustes finos (sensibilidade, limiares de dead zone e intensidade dos filtros) podem ser feitos diretamente em `src/main.cpp` caso precise adequar o comportamento ao seu uso.
