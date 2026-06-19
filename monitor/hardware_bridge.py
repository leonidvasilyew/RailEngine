"""Мост: serial базы ESP32-S3 (AutoLego) → монитор RailEngine.

База пишет в serial машинный канал (см. MONITOR_OUT в Base_WS.ino):
  @SNS <sensor_id> <1|0>            — проезд датчика: 1=въезд, 0=выезд
  @SPD <train_id> <speed> <smooth> — заданная поезду скорость (знак=направление)
  @SIG <sensor_id> <c1> <c2> <ang> — светофоры + угол серво (положение стрелки)
  @CON <T|S> <id>                  — узел вышел на связь

Мост переводит их в команды монитора и шлёт по WebSocket в наш сервер:
  @SNS …1   → fire_sensor (коррекция позиции по въезду)
  @SPD      → set_speed (знаковая скорость; направление как в схеме монитора)
  @SIG      → set_switch (угол серво → STRAIGHT/BRANCH)

Монитор — пассивное ЗЕРКАЛО: поезда нужно заранее поставить в мониторе, задав им
тот же физический id (поле "id" при постановке), и датчикам в редакторе задать их
физический id — тогда маппинг не нужен. Сервер друга не трогается (данные из serial).

Запуск:
  python -m monitor.hardware_bridge --port COM5
Тест без железа (подать строки вручную):
  echo "@SNS 10 1" | python -m monitor.hardware_bridge --stdin

Зависимости: websockets (есть), pyserial (для --port).
"""

import argparse
import asyncio
import json
import sys
from datetime import datetime

import websockets

# ─────────────────────────── КОНФИГ (подгони под макет) ───────────────────────────
MONITOR_WS = 'ws://127.0.0.1:8077/ws'
SERIAL_BAUD = 115200

# БЕЗ МАППИНГА. В мониторе/редакторе задаются те же физические uint16 id:
#   датчик-детектор  → id датчика   (поле ID у датчика в редакторе)  → @SNS
#   поезд            → id поезда    (поле id при постановке)         → @SPD
#   стрелка-узел     → id управляющего ДАТЧИКА (поле ID у точки)     → @SIG
# Серво приходит в @SIG с id датчика; стрелка-узел имеет тот же id → set_switch(node=id).
SPEED_SCALE = 0.5         # int8-скорость базы → м/с монитора
STRAIGHT_MAX_ANGLE = 97   # угол серво <= порога → STRAIGHT(1), иначе BRANCH(2); 255 = не трогать
# ──────────────────────────────────────────────────────────────────────────────────


def parse_line(line):
    """@-строку базы → команду монитора (dict) или None."""
    line = line.strip()
    if not line.startswith('@'):
        print(datetime.now().time(), line)
        return None
    p = line.split()
    try:
        op = p[0]
        if op == '@SNS':
            sid, stat = int(p[1]), int(p[2])
            if stat == 1:                       # въезд → коррекция позиции
                return {'cmd': 'fire_sensor', 'sensor': sid}    # id напрямую, без маппинга
            return None                         # выезд пока игнорируем
        if op == '@SPD':
            tid, sp = int(p[1]), int(p[2])
            return {'cmd': 'set_speed', 'train': tid,           # id напрямую, без маппинга
                    'v_target': round(sp * SPEED_SCALE, 3)}
        if op == '@SIG':
            sid, ang = int(p[1]), int(p[4])
            if ang != 255:                          # стрелка-узел имеет id датчика
                return {'cmd': 'set_switch', 'node': sid,
                        'position': 1 if ang <= STRAIGHT_MAX_ANGLE else 2}
            return None
        # @CON — узел на связи; для зеркала пока не используем
        return None
    except (IndexError, ValueError):
        return None


# ─────────────────────────── Источники строк ───────────────────────────
async def _read_serial(port, baud, queue):
    import serial
    ser = serial.Serial(port, baud, timeout=0.3)
    print(f'[bridge] serial {port}@{baud}', flush=True)
    while True:
        raw = await asyncio.to_thread(ser.readline)
        if raw:
            await queue.put(raw.decode('utf-8', 'replace'))


async def _read_stdin(queue):
    print('[bridge] stdin', flush=True)
    while True:
        line = await asyncio.to_thread(sys.stdin.readline)
        if not line:
            await queue.put(None)               # EOF
            return
        await queue.put(line)


# ─────────────────────────── Основной цикл ───────────────────────────
async def run(args):
    queue = asyncio.Queue()
    if args.stdin:
        asyncio.create_task(_read_stdin(queue))
    else:
        asyncio.create_task(_read_serial(args.port, args.baud, queue))

    while True:
        try:
            async with websockets.connect(args.ws) as ws:
                print(f'[bridge] monitor {args.ws}', flush=True)

                async def _drain():                # входящие от монитора игнорируем
                    try:
                        async for _ in ws:
                            pass
                    except Exception:
                        pass
                asyncio.create_task(_drain())

                while True:
                    line = await queue.get()
                    if line is None:               # EOF stdin → выходим
                        return
                    cmd = parse_line(line)
                    if cmd is not None:
                        await ws.send(json.dumps(cmd))
                        print('→', cmd, flush=True)
        except (OSError, websockets.exceptions.WebSocketException) as e:
            print(f'[bridge] монитор недоступен, повтор через 2с: {e}', flush=True)
            await asyncio.sleep(2)


def main():
    ap = argparse.ArgumentParser(description='Serial базы ESP32-S3 → монитор RailEngine')
    ap.add_argument('--port', default='COM5', help='serial-порт базы (Windows: COMx)')
    ap.add_argument('--baud', type=int, default=SERIAL_BAUD)
    ap.add_argument('--ws', default=MONITOR_WS, help='WebSocket монитора')
    ap.add_argument('--stdin', action='store_true', help='читать строки из stdin (тест без железа)')
    args = ap.parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
