#!/usr/bin/env python3
## @file logger.py
#  @brief Captura el protocolo CSV emitido por el Nodo 2 y lo guarda en disco.
#
#  Este script reemplaza la etapa de escritura en tarjeta SD. El firmware del
#  Nodo 2 imprime lineas por el puerto serie con prefijos fijos y este logger
#  las reconstruye en un archivo CSV dentro de la carpeta ``logs`` del proyecto.

"""Captura el protocolo CSV emitido por el ESP32 y lo guarda en ./logs/."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import serial
    from serial import SerialException
except ImportError as exc:  # pragma: no cover
    raise SystemExit("Falta pyserial. Instalalo con: pip install pyserial") from exc

META_PREFIX = "CSV_META|"
HEADER_PREFIX = "CSV_HEADER|"
ROW_PREFIX = "CSV_ROW|"


## @brief Procesa los argumentos de linea de comandos del logger serial.
#  @return Namespace con puerto, baudrate, codificacion y carpeta del proyecto.
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Logger serial a CSV para Nodo2")
    parser.add_argument("--port", required=True, help="Puerto serie, por ejemplo COM6 o /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate del puerto serie")
    parser.add_argument(
        "--project-dir",
        default=".",
        help="Ruta del proyecto donde se creara la carpeta logs",
    )
    parser.add_argument(
        "--encoding",
        default="utf-8",
        help="Codificacion usada para mostrar texto recibido",
    )
    return parser.parse_args()


## @brief Limpia el nombre recibido desde el ESP32 para usarlo como archivo.
#  @param name Nombre crudo recibido en la linea ``CSV_META|``.
#  @return Nombre filtrado con caracteres seguros para el sistema de archivos.
def sanitize_filename(name: str) -> str:
    cleaned = "".join(ch for ch in name.strip() if ch.isalnum() or ch in ("-", "_", "."))
    return cleaned or "adl_session_unknown.csv"


## @brief Crea un CSV nuevo y escribe su cabecera.
#  @param path Ruta completa del archivo a crear.
#  @param header Cabecera CSV emitida por el firmware.
def write_new_file(path: Path, header: str) -> None:
    with path.open("w", encoding="utf-8", newline="") as fh:
        fh.write(header + "\n")


## @brief Agrega una fila CSV al archivo actualmente activo.
#  @param path Ruta del archivo destino.
#  @param row Fila serializada emitida con el prefijo ``CSV_ROW|``.
def append_row(path: Path, row: str) -> None:
    with path.open("a", encoding="utf-8", newline="") as fh:
        fh.write(row + "\n")


## @brief Bucle principal del logger serial.
#  @return Codigo de salida de proceso.
def main() -> int:
    args = parse_args()
    project_dir = Path(args.project_dir).resolve()
    logs_dir = project_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)

    current_path: Path | None = None
    current_header: str | None = None

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except SerialException as exc:
        print(f"No se pudo abrir {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"Escuchando {args.port} a {args.baud} baudios...")
    print(f"Guardando CSV en: {logs_dir}")

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            text = raw.decode(args.encoding, errors="replace").rstrip("\r\n")
            if text:
                print(text)

            idx = text.find(META_PREFIX)
            if idx >= 0:
                filename = sanitize_filename(text[idx + len(META_PREFIX):])
                current_path = logs_dir / filename
                continue

            idx = text.find(HEADER_PREFIX)
            if idx >= 0:
                current_header = text[idx + len(HEADER_PREFIX):]
                if current_path is None:
                    current_path = logs_dir / "adl_session_unknown.csv"
                if not current_path.exists() or current_path.stat().st_size == 0:
                    write_new_file(current_path, current_header)
                continue

            idx = text.find(ROW_PREFIX)
            if idx >= 0:
                if current_path is None:
                    current_path = logs_dir / "adl_session_unknown.csv"
                if current_header is not None and (not current_path.exists() or current_path.stat().st_size == 0):
                    write_new_file(current_path, current_header)
                append_row(current_path, text[idx + len(ROW_PREFIX):])
    except KeyboardInterrupt:
        print("\nCaptura finalizada por el usuario.")
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
