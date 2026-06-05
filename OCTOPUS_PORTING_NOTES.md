# OCTOPUS_PORTING_NOTES.md

## 1. Цель проекта

Оригинальная прошивка PAROL6 рассчитана на custom control board проекта PAROL6.

Сейчас мы переносим прошивку на BIGTREETECH Octopus Pro V1.0 с микроконтроллером STM32F446ZET6. На этом этапе цель не в том, чтобы сразу запустить робота целиком, а в том, чтобы сначала получить стабильную базу: корректный старт платы, правильный clock, рабочий USB CDC, безопасный старт прошивки и понятный pin mapping.

Моторы, датчики, драйверы, CAN и gripper нужно подключать только после того, как базовая прошивка и питание платы проверены отдельно.

## 2. Текущая подтвержденная рабочая база

Уже подтверждено:

- Плата входит в STM32 ROM DFU через BOOT0.
- macOS видит STM32 BOOTLOADER в DFU.
- Прошивка через `dfu-util` работает.
- Тестовая USB CDC прошивка `octopus_usb_test_0000` работает.
- macOS видит устройство как `/dev/cu.usbmodem...`.
- Serial monitor показывает `millis`.
- Основная PAROL6 firmware в env `octopus_parol6_direct_0000` запускается.
- `setup()` проходит до конца.
- `loop()` работает.
- Serial monitor показывает `LOOP alive`.

## 3. Важный вывод по проблеме USB

Плата не была сгоревшей. Проблема была в софте: неправильная комбинация clock setup и flash offset.

`nucleo_f446ze` подходит как временная база только потому, что у него тот же класс MCU, но он опасен как готовая board-конфигурация для Octopus Pro F446. У Nucleo другие clock assumptions, включая HSE bypass / 8 MHz-style assumptions.

Для Octopus Pro F446 нужен внешний HSE crystal 12 MHz. Рабочее решение сейчас использует кастомный `SystemClock_Config` в:

```text
PAROL6 control board main software/src/octopus_clock.cpp
```

## 4. Flash Layout / Boot Mode

Текущее решение:

- Используем direct DFU через BOOT0.
- Прошивка стартует с `0x08000000`.
- НЕ используем BTT bootloader offset `0x08008000`.
- НЕ используем `VECT_TAB_OFFSET=0x8000`.
- `.isr_vector = 0x08000000`.
- `LD_FLASH_OFFSET = 0x0`.

Если позже нужно будет вернуться к BTT SD bootloader, для этого нужен отдельный env. В нем потребуется:

- upload address `0x08008000`;
- linker flash origin `0x08008000`;
- `VECT_TAB_OFFSET=0x8000`;
- уменьшенный flash size.

Не смешивать direct DFU env и bootloader env.

## 5. PlatformIO Environments

### `octopus_usb_test_0000`

Минимальный USB CDC тест.

Он:

- не использует PAROL6 runtime;
- не использует CAN;
- не использует TMC;
- не использует motor logic;
- нужен как контрольная прошивка, что USB и clock на Octopus работают.

### `octopus_parol6_direct_0000`

Основная PAROL6 firmware для Octopus.

Свойства:

- direct DFU;
- старт с `0x08000000`;
- CAN временно отключен;
- включает debug flags:
  - `OCTOPUS_BOOT_DEBUG`;
  - `OCTOPUS_LOOP_HEARTBEAT`;
- печатает BOOT-сообщения и `LOOP alive`.

### `octopus_parol6_direct_quiet_0000`

Та же PAROL6 firmware для Octopus, но без постоянных debug-сообщений.

Свойства:

- direct DFU;
- старт с `0x08000000`;
- без BOOT debug;
- без LOOP heartbeat;
- CAN все еще отключен;
- нужна как более чистая сборка.

### `octopus_parol6_limit_diag_0000`

Безопасная diagnostic firmware для чтения LIMIT/Home входов.

Свойства:

- direct DFU;
- старт с `0x08000000`;
- CAN все еще отключен через `DISABLE_CAN_INIT_FOR_USB_TEST`;
- обычная логика PAROL6 `loop()` не выполняется;
- после diagnostic print в `loop()` стоит `return`;
- моторы не должны двигаться;
- homing не запускается;
- step pulses не отправляются;
- раз в секунду печатает состояние входов:
  - `LIMIT1` / Stop0 / Joint1;
  - `LIMIT2` / Stop1 / Joint2;
  - `LIMIT3` / Stop2 / Joint3;
  - `LIMIT4` / Stop3 / Joint4;
  - `LIMIT5` / Stop4 / Joint5;
  - `LIMIT6` / Stop5 / Joint6.

Этот env нужен для будущей проверки Stop0-Stop5 без моторов, без homing и без движения.

## 6. Clock Configuration

Файл:

```text
PAROL6 control board main software/src/octopus_clock.cpp
```

Текущие параметры clock:

- HSE = 12 MHz;
- `RCC_HSE_ON`;
- PLL source = HSE;
- PLLM = 6;
- PLLN = 168;
- PLLP = DIV2;
- PLLQ = 7;
- SYSCLK = 168 MHz;
- USB clock = 48 MHz;
- APB1 = HCLK / 4;
- APB2 = HCLK / 2.

Расчет:

```text
12 MHz / 6 = 2 MHz
2 MHz * 168 = 336 MHz VCO
336 MHz / 2 = 168 MHz SYSCLK
336 MHz / 7 = 48 MHz USB clock
```

USB требует точного 48 MHz clock. Если clock настроен неправильно, macOS может не увидеть плату как USB device, даже если прошивка формально прошилась.

## 7. CAN Status

CAN сейчас временно отключен через:

```text
DISABLE_CAN_INIT_FOR_USB_TEST
```

Причина: в CAN-коде есть потенциально бесконечные ожидания.

Известные места:

- `CAN.cpp`: ожидание CAN1 init mode.
- `CAN.cpp`: ожидание CAN2 init mode.
- `coms_CAN.cpp`: `while(true)`, если `CANInit()` вернул false.

CAN нельзя возвращать сразу. Сначала нужно заменить бесконечные ожидания на timeout и error-reporting, чтобы ошибка CAN не могла повесить весь firmware startup.

## 8. HAL_Init

Повторный `HAL_Init()` в main code защищен и не вызывается в direct test env.

Причина: Arduino core уже делает HAL init. Повторный HAL init может быть нежелателен для USB, timing и уже настроенных peripheral state.

## 9. Motor Enable Safety

Enable-пины драйверов выставляются `HIGH` на старте.

Для TMC stepstick-style драйверов обычно:

- `HIGH` = disabled;
- `LOW` = enabled.

В direct test env motor enable `LOW` пропущен. Значит драйверы должны оставаться выключенными при старте.

Enable mapping:

| Joint | Octopus motor slot | ENABLE macro | ENABLE pin |
| ----- | ------------------ | ------------ | ---------- |
| Joint 1 | MOTOR0 | `GLOBAL_ENABLE` | `PF14` |
| Joint 2 | MOTOR1 | `ENABLE_M1` | `PF15` |
| Joint 3 | MOTOR2 | `ENABLE_M2` | `PG5` |
| Joint 4 | MOTOR3 | `ENABLE_M3` | `PA0` |
| Joint 5 | MOTOR4 | `ENABLE_M4` | `PG2` |
| Joint 6 | MOTOR5 | `ENABLE_M5` | `PF1` |

Позже можно добавить:

```cpp
#define ENABLE_M0 GLOBAL_ENABLE
```

Это улучшит читаемость, но сейчас не обязательно.

## 10. Current Joint -> Octopus Mapping

| Joint | Octopus motor slot | STEP macro | STEP pin | DIR macro | DIR pin | CS macro | CS pin | LIMIT macro | LIMIT pin | Stop/DIAG | ENABLE macro | ENABLE pin |
| ----- | ------------------ | ---------- | -------- | --------- | ------- | -------- | ------ | ----------- | --------- | --------- | ------------ | ---------- |
| Joint 1 | MOTOR0 | `PUL1` | `PF13` | `DIR1` | `PF12` | `SELECT1` | `PC4` | `LIMIT1` | `PG6` | Stop0 / DIAG0 | `GLOBAL_ENABLE` | `PF14` |
| Joint 2 | MOTOR1 | `PUL6` | `PG0` | `DIR6` | `PG1` | `SELECT6` | `PD11` | `LIMIT2` | `PG9` | Stop1 / DIAG1 | `ENABLE_M1` | `PF15` |
| Joint 3 | MOTOR2 | `PUL5` | `PF11` | `DIR5` | `PG3` | `SELECT5` | `PC6` | `LIMIT3` | `PG10` | Stop2 / DIAG2 | `ENABLE_M2` | `PG5` |
| Joint 4 | MOTOR3 | `PUL4` | `PG4` | `DIR4` | `PC1` | `SELECT4` | `PC7` | `LIMIT4` | `PG11` | Stop3 / DIAG3 | `ENABLE_M3` | `PA0` |
| Joint 5 | MOTOR4 | `PUL2` | `PF9` | `DIR2` | `PF10` | `SELECT2` | `PF2` | `LIMIT5` | `PG12` | Stop4 / DIAG4 | `ENABLE_M4` | `PG2` |
| Joint 6 | MOTOR5 | `PUL3` | `PC13` | `DIR3` | `PF0` | `SELECT3` | `PE4` | `LIMIT6` | `PG13` | Stop5 / DIAG5 | `ENABLE_M5` | `PF1` |

## 11. Important Fix Already Applied

Раньше limit mapping был перепутан:

- Joint 1 использовал `LIMIT6`;
- Joint 6 использовал `LIMIT1`.

Это приводило к физической схеме:

- Joint 1 -> Stop5;
- Joint 6 -> Stop0.

Исправлено в:

```text
PAROL6 control board main software/src/motor_init.cpp
```

Теперь:

- Joint 1 использует `LIMIT1`;
- Joint 6 использует `LIMIT6`.

Физическая схема стала понятной:

- Joint 1 -> Stop0;
- Joint 2 -> Stop1;
- Joint 3 -> Stop2;
- Joint 4 -> Stop3;
- Joint 5 -> Stop4;
- Joint 6 -> Stop5.

## 12. Что Сейчас НЕ Проверено

Пока НЕ проверено:

- TMC5160 hardware drivers не подключались.
- Моторы не подключались.
- 24V питание пока не проверено мультиметром.
- Limit/home датчики физически не проверены.
- `octopus_parol6_limit_diag_0000` уже создан и собирается, но физически Stop0-Stop5 еще не проверялись.
- Optocoupler board физически не проверена.
- E-stop физически не проверен.
- CAN не включен.
- Gripper не проверен.
- Commander protocol не проверен.

## 13. Hardware Safety Notes

Перед продолжением:

- До рабочей батарейки для мультиметра не подавать 24V.
- Не вставлять TMC5160 под питанием.
- Не подключать и не отключать моторы под питанием.
- Не подавать 24V на Stop/DIAG входы.
- Не запускать homing до проверки датчиков.
- Сначала проверять питание, потом входы, потом драйверы, потом один мотор.

## 14. Следующие Рекомендуемые Шаги

1. Проверить мультиметр, заменить батарейку 9V.
2. Проверить лабораторный блок питания мультиметром.
3. Проверить правильность +24V/GND на POWER_IN.
4. Не подключая TMC и моторы, проверить питание платы.
5. Diagnostic mode уже добавлен: `octopus_parol6_limit_diag_0000`.
6. После появления рабочей батарейки 9V для мультиметра прошить `octopus_parol6_limit_diag_0000` и проверить Stop0-Stop5 вручную.
7. Только потом подключать TMC5160 по одному.
8. Проверить SPI communication с одним TMC.
9. Потом проверить один мотор без механической нагрузки.
10. Потом запускать homing только на минимальной скорости и с физическим выключением питания рядом.

## 15. Текущее состояние на момент остановки

- USB/clock проблема решена.
- Плата подтвержденно живая.
- PAROL6 firmware стартует на Octopus.
- Debug и quiet env сохранены.
- Limit diagnostic env создан.
- Pin mapping Joint 1-6 приведен к физическому порядку Stop0-Stop5.
- Дальше не продолжать hardware-тесты до рабочей батарейки 9V для мультиметра.
- 24V пока не подавать.
