ESP32 Air Mouse v1.1 Horizontal Correction
=========================================

Firmware para ESP32 que transforma o conjunto MPU-6050 (GY-521) em um mouse/ponteiro aereo com deteccao automatica de orientacao e filtros avancados.

Principais Recursos
-------------------
- Leitura do giroscopio e acelerometro via I2C para gerar movimentos X/Y com suavizacao e zonas adaptativas.
- Auto-calibracao do giroscopio quando o dispositivo esta parado.
- Deteccao automatica de orientacao (horizontal, vertical, invertido, esquerda, direita) com ajuste dinamico do mapa de controles.
- Botao no GPIO 15 para acionar cliques via script Python `gyro_mouseCOMbotao.py`.
- **Novo:** camada reforcada de resiliencia I2C com verificacao de escrita, reconfiguracao automatica do barramento e checagem `WHO_AM_I` apos falhas.

Novidades desta atualizacao
---------------------------
- `mpuWrite` agora verifica o status do `Wire.endTransmission`, impedindo que configuracoes continuem apos erros de barramento.
- `recoverFromI2CFailure` reinicializa o driver I2C, reaplica a configuracao do MPU-6050 e confirma o retorno do sensor lendo o registrador `WHO_AM_I` (0x68).
- Laco de inicializacao substituido por `waitForMPU(timeout_ms)`, evitando travamentos durante o boot quando o dispositivo nao responde.
- Calibracao e loop principal agora registram falhas de leitura (`E2`) e de recuperacao (`E5`), ignorando amostras invalidas em vez de propagar dados corrompidos.

Compilacao e Upload
-------------------
1. Instale PlatformIO (CLI ou extensao VS Code).
2. Conecte o ESP32 (DOIT DEVKIT V1) e ajuste a porta serial se necessario em `platformio.ini`.
3. Compile com `pio run`.
4. Faca upload com `pio run -t upload` e monitore com `pio device monitor` a 115200 bps.

Recuperacao de Falhas I2C
-------------------------
- O firmware registra `E2:LEITURA_FALHOU` quando uma leitura retorna incompleta.
- `recoverFromI2CFailure` tenta ate tres reinicializacoes; se todas falharem, aparece `E4:RECUPERACAO_I2C_FALHOU` ou `E5:RECUPERACAO_FALHOU` indicando necessidade de intervencao manual.
- Durante o boot, `waitForMPU` exibe `E1:MPU_NAO_RESPONDE` caso o sensor esteja desconectado ou sem alimentacao.
- Se a configuracao inicial nao for recuperada, o firmware entra em loop seguro e orienta a reinicializar o dispositivo.

Script de Integracao com PC
---------------------------
- Utilize `gyro_mouseCOMbotao.py` para converter os dados seriais em eventos de mouse/teclado no computador.
- Ajuste o script de acordo com o sistema operacional e mapeamentos desejados.

Dicas de Solucao de Problemas
-----------------------------
- Verifique cabos SDA/SCL e GND compartilhados quando mensagens `E1`, `E4` ou `E5` aparecerem continuamente.
- Certifique-se de que nenhuma outra placa esteja ocupando o mesmo barramento I2C ou alterando a tensao de pull-up.
- Em casos extremos, considere adicionar um watchdog externo ou `ESP.restart()` apos multiplas falhas consecutivas.

