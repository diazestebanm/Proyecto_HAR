# README - Puesta en marcha del proyecto Nodo1 + Nodo2

Este documento explica **cómo poner a funcionar el proyecto cada vez que se vaya a usar**.

La idea actual del sistema es:

- **Nodo 1** adquiere sus datos, los procesa y los envía por **ESP-NOW**.
- **Nodo 2** adquiere sus datos locales, recibe los datos de Nodo 1, hace el emparejamiento por ventana y:
  - muestra el **resumen por ventana** en el serial
  - emite las filas `CSV_ROW|...` por serial
- Un script de **Python** en el PC escucha el puerto serial de **Nodo 2** y guarda el archivo `.csv` dentro de la carpeta del proyecto.

---

## 1. Requisitos previos

Antes de iniciar:

- Tener instalado **ESP-IDF** y su terminal funcionando.
- Tener instalado **Python** en Windows.
- Tener instalado `pyserial`.
- Tener conectados por USB:
  - **Nodo 1**
  - **Nodo 2**
- Tener identificado qué puerto COM corresponde a cada placa.

Para instalar `pyserial` una sola vez:

```powershell
pip install pyserial
```

Si `python` no está en el PATH, usar el ejecutable completo. Ejemplo:

```powershell
C:\Users\esteb\scoop\apps\python313\current\python.exe -m pip install pyserial
```

---

## 2. Identificar los puertos COM

En una terminal de Windows:

```powershell
mode
```

Ahí verificas qué puertos están disponibles, por ejemplo:

- `COM5` -> Nodo 1
- `COM6` -> Nodo 2

**Importante:** los puertos pueden cambiar al desconectar y volver a conectar las placas.

---

## 3. Orden correcto de arranque

Cada vez que vayas a usar el sistema, sigue este orden:

1. Conectar ambas placas.
2. Flashear **Nodo 1** si hiciste cambios.
3. Flashear **Nodo 2** si hiciste cambios.
4. Cerrar cualquier monitor serial que use el puerto de **Nodo 2**.
5. Ejecutar el **logger de Python** sobre el puerto de **Nodo 2**.
6. Reiniciar o energizar las placas si hace falta.

---

## 4. Ejecutar Nodo 1

Abrir una terminal en la carpeta del proyecto de **Nodo 1**.

### Si solo quieres compilar:

```powershell
idf.py build
```

### Si quieres flashear:

```powershell
idf.py -p COM5 flash
```

Cambia `COM5` por el puerto real de **Nodo 1**.

### Si también quieres ver el serial de Nodo 1:

```powershell
idf.py -p COM5 monitor
```

**Nota:** esto solo aplica para **Nodo 1**. No uses el monitor en **Nodo 2** si vas a usar el logger de Python, porque ambos pelean por el mismo puerto.

---

## 5. Ejecutar Nodo 2

Abrir una terminal en la carpeta del proyecto de **Nodo 2**.

### Si solo quieres compilar:

```powershell
idf.py build
```

### Si quieres flashear:

```powershell
idf.py -p COM6 flash
```

Cambia `COM6` por el puerto real de **Nodo 2**.

### Muy importante

Después de flashear **Nodo 2**, **no** abras:

- `idf.py monitor`
- monitor serial de VS Code
- Arduino Serial Monitor
- PuTTY
- Tera Term
- otro programa que tome `COM6`

porque el puerto debe quedar libre para el logger de Python.

---

## 6. Ejecutar el logger de Python para guardar el CSV

Este paso se hace desde la carpeta del proyecto de **Nodo 2**.

### Opción normal

```powershell
python scripts\logger.py --port COM6 --baud 115200 --project-dir .
```

### Opción recomendada si `python` no está en PATH

```powershell
C:\Users\esteb\scoop\apps\python313\current\python.exe scripts\logger.py --port COM6 --baud 115200 --project-dir .
```

Cambia `COM6` por el puerto real de **Nodo 2**.

---

## 7. Dónde queda guardado el archivo

El archivo se guarda dentro de la carpeta del proyecto de **Nodo 2**, en:

```text
.\logs\
```

Ejemplo:

```text
C:\Users\esteb\Downloads\Nodo2\logs\adl_session_0001.csv
```

El nombre exacto depende de la línea `CSV_META|...` que envíe el firmware.

---

## 8. Qué hace el logger

El logger **no guarda todo el serial**.

Solo guarda las líneas que empiezan por:

- `CSV_META|`
- `CSV_HEADER|`
- `CSV_ROW|`

Por eso:

- los logs `I (...)`, `W (...)`, `E (...)` se ven en pantalla
- pero **no se meten al CSV**

---

## 9. Flujo recomendado de uso cada vez

### Caso A: no cambiaste código, solo quieres correr el sistema

1. Conecta Nodo 1 y Nodo 2.
2. Verifica los puertos COM con:

```powershell
mode
```

3. En la carpeta de **Nodo 2**, ejecuta el logger:

```powershell
C:\Users\esteb\scoop\apps\python313\current\python.exe scripts\logger.py --port COM6 --baud 115200 --project-dir .
```

4. Energiza o reinicia las placas.
5. Espera a que Nodo 2 empiece a mostrar:
   - `RADIO: N1 seq=...`
   - `SUMMARY: ...`
   - `CSV_ROW|...`

Con eso el sistema ya está funcionando y el CSV se va guardando en `logs`.

---

### Caso B: hiciste cambios de código

#### Nodo 1

En la carpeta de Nodo 1:

```powershell
idf.py build
idf.py -p COM5 flash
```

#### Nodo 2

En la carpeta de Nodo 2:

```powershell
idf.py build
idf.py -p COM6 flash
```

Luego ejecuta el logger:

```powershell
C:\Users\esteb\scoop\apps\python313\current\python.exe scripts\logger.py --port COM6 --baud 115200 --project-dir .
```

---

## 10. Si el logger dice que no puede abrir COM6

Ejemplo de error:

```text
PermissionError(13, 'Access is denied.')
```

Eso significa que **otro programa ya está usando el puerto**.

Haz esto:

1. Cierra cualquier monitor serial.
2. Cierra VS Code si dejó el puerto ocupado.
3. Desconecta y vuelve a conectar la placa.
4. Verifica otra vez con:

```powershell
mode
```

5. Ejecuta de nuevo el logger con el COM correcto.

---

## 11. Si quieres ver el serial de Nodo 1 y guardar el CSV de Nodo 2 al mismo tiempo

Sí se puede, siempre que cada uno use su propio puerto:

- **Nodo 1** -> `idf.py monitor` en su COM
- **Nodo 2** -> `logger.py` en su COM

Ejemplo:

### Terminal 1 - Nodo 1

```powershell
idf.py -p COM5 monitor
```

### Terminal 2 - Nodo 2

```powershell
C:\Users\esteb\scoop\apps\python313\current\python.exe scripts\logger.py --port COM6 --baud 115200 --project-dir .
```

---

## 12. Comando rápido de uso normal

Si todo ya está compilado y flasheado, normalmente solo necesitas esto en **Nodo 2**:

```powershell
C:\Users\esteb\scoop\apps\python313\current\python.exe scripts\logger.py --port COM6 --baud 115200 --project-dir .
```

Y luego energizar/reiniciar las placas.

---

## 13. Resumen corto

Cada vez que quieras usar el proyecto:

1. Conecta las placas.
2. Revisa los puertos COM.
3. Flashea si hubo cambios.
4. En **Nodo 2**, ejecuta el logger de Python.
5. No abras monitor serial sobre el mismo puerto de Nodo 2.
6. Revisa el archivo generado en `logs`.

