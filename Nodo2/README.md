# Nodo2
Firmware del nodo de pierna/tobillo basado en ESP32.

## Estado actual del backend de almacenamiento

Este proyecto **ya no usa tarjeta SD**.

Se dejo el modulo `hal_sdcard` por compatibilidad con la arquitectura original,
pero ahora actua como un **backend serial**: el firmware imprime la metadata,
la cabecera y cada fila CSV por UART/USB, y un script en Python reconstruye el
archivo final en el computador.

## Logger CSV hacia el host

- El firmware emite la cabecera y cada fila CSV por el puerto serie con prefijos `CSV_META|`, `CSV_HEADER|` y `CSV_ROW|`.
- El script `scripts/logger.py` escucha el puerto y crea el archivo dentro de `./logs/`.
- Esta ruta reemplaza por ahora toda la parte de microSD, que queda documentada pero fuera de uso.

### Uso rapido en Windows

1. Instala la dependencia:
   `pip install pyserial`
2. Flashea el ESP32.
3. En otra terminal, desde la carpeta del proyecto, ejecuta:
   `python scripts/logger.py --port COM6 --baud 115200 --project-dir .`

El archivo quedara en `logs/adl_session_XXXX.csv`.
