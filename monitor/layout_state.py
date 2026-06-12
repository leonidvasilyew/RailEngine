"""Ядро live-мониторинга: оценка положения поездов в реальном времени.

Модель замкнутого контура (closed-loop):
  * положение каждого поезда интегрируется из заданных скорости/ускорения
    (dead reckoning) на каждом тике;
  * маршрутизация в узлах определяется СОСТОЯНИЕМ СТРЕЛОК
    (Switch.position ∈ {STRAIGHT, BRANCH});
  * при срабатывании датчика положение поезда подтягивается к известной
    координате датчика (коррекция накопленного дрейфа).

Существующие файлы не изменяются: topology.py и pathfinder.Train импортируются.
Запуск автономного прогона:  python -m monitor.layout_state
"""

import math
import os
import sys

# Делаем корень проекта импортируемым независимо от способа запуска,
# чтобы работали `import topology` / `from pathfinder import Train`.
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from topology import Topology, Switch, Port      # noqa: E402
from pathfinder import Train                      # noqa: E402


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class Sensor:
    """Точечный датчик на ребре. position — нормализованная координата (0..1).
    radius — радиус коррекции в МЕТРАХ вдоль путей: при срабатывании
    корректируется ближайший поезд, голова которого в этом радиусе."""
    __slots__ = ('id', 'edge_id', 'position', 'name', 'radius')

    def __init__(self, sensor_id, edge_id, position, name='', radius=20.0):
        self.id = sensor_id
        self.edge_id = edge_id
        self.position = position
        self.name = name or str(sensor_id)
        self.radius = radius

    def to_dict(self):
        return {'id': self.id, 'edge_id': self.edge_id,
                'position': self.position, 'name': self.name,
                'radius': self.radius}


class LiveTrain:
    """Оценка состояния одного поезда на макете.

    edge_id, s   — текущее ребро и нормализованная позиция ГОЛОВЫ (0=node_a, 1=node_b)
    direction    — направление прохода по ребру: +1 (s растёт) / -1 (s убывает),
                   в которую движется НОС при ходе вперёд
    v            — модуль текущей скорости (>=0, м/с)
    v_target     — ЗНАКОВАЯ целевая скорость относительно СХЕМЫ (джойстик):
                   знак задаёт желаемое направление по схеме, не зависит от поезда
    face         — куда «смотрит» нос в системе отсчёта схемы: +1 = направление
                   постановки, -1 = противоположное. Меняется только при развороте.
    accel/decel  — берутся из train, но команда может задать иное ускорение
    history      — список [edge_id, entry_s, exit_s] для отрисовки тела поезда;
                   exit_s = None у текущего (головного) ребра.
    """
    __slots__ = ('id', 'train', 'edge_id', 's', 'direction', 'face',
                 'v', 'v_target', 'accel', 'decel',
                 'history', 'color', 'name', 'stopped_reason')

    def __init__(self, train_id, train, edge_id, s, direction=1,
                 color='#e74c3c', name=''):
        self.id = train_id
        self.train = train
        self.edge_id = edge_id
        self.s = s
        self.direction = direction
        self.face = 1            # +1 = направление постановки (схемный «плюс»)
        self.v = 0.0             # модуль скорости
        self.v_target = 0.0      # знаковая цель (относительно схемы)
        self.accel = train.accel
        self.decel = train.decel
        self.color = color
        self.name = name or f'T{train_id}'
        self.history = []
        self.stopped_reason = None


class LayoutState:
    """Состояние всего макета: поезда, датчики, положения стрелок."""

    def __init__(self, topo):
        self.topo = topo
        self.trains = {}        # id -> LiveTrain
        self.sensors = {}       # id -> Sensor
        self.switches = {}      # node_id -> Port.STRAIGHT | Port.BRANCH
        self.time = 0.0
        # Положения стрелок по умолчанию — прямо (STRAIGHT)
        for nid, node in topo._nodes.items():
            if isinstance(node, Switch):
                self.switches[nid] = Port.STRAIGHT

    # ── Команды ──────────────────────────────────────────────────────────
    def place_train(self, train_id, train, edge_id, position,
                    direction=1, color='#e74c3c', name=''):
        edge = self.topo.get_edge(edge_id)
        s = position / edge.length if position > 1.0 else position
        s = _clamp(s, 0.0, 1.0)
        lt = LiveTrain(train_id, train, edge_id, s, direction, color, name)
        # Начальная история: хвост вытянут назад против direction на длину поезда.
        entry_s = _clamp(s - direction * train.length / edge.length, 0.0, 1.0)
        lt.history.append([edge_id, entry_s, None])
        self.trains[train_id] = lt
        return lt

    def remove_train(self, train_id):
        self.trains.pop(train_id, None)

    def set_speed(self, train_id, v_target, accel=None, decel=None):
        lt = self.trains.get(train_id)
        if lt is None:
            return
        # Знаковая цель: <0 — команда «назад» (вызовет разворот при переходе
        # скорости через ноль). Модуль ограничен max_speed.
        mx = lt.train.max_speed
        lt.v_target = _clamp(v_target, -mx, mx)
        if accel is not None:
            lt.accel = accel
        if decel is not None:
            lt.decel = decel
        lt.stopped_reason = None

    def add_sensor(self, sensor_id, edge_id, position, name='', radius=1.0):
        self.sensors[sensor_id] = Sensor(sensor_id, edge_id, position, name, radius)
        return self.sensors[sensor_id]

    def set_switch(self, node_id, position):
        """position: Port.STRAIGHT или Port.BRANCH."""
        if node_id in self.switches:
            self.switches[node_id] = position

    def toggle_switch(self, node_id):
        if node_id in self.switches:
            self.switches[node_id] = (
                Port.BRANCH if self.switches[node_id] == Port.STRAIGHT
                else Port.STRAIGHT)
        return self.switches.get(node_id)

    # ── Маршрутизация в узле по состоянию стрелки ────────────────────────
    def _next_port(self, node, from_port):
        """Куда уходит поезд, въехавший в узел через from_port. None = блок/тупик."""
        if isinstance(node, Switch):
            P, S, B = Port.POINT, Port.STRAIGHT, Port.BRANCH
            pos = self.switches.get(node.id, S)
            if from_port == P:
                return pos                       # facing move: на выбранную ветку
            if from_port == pos:
                return P                         # trailing move согласован со стрелкой
            return None                          # against the switch — блокировка
        # Обычный узел / тупик: единственный другой подключённый порт.
        cands = [p for p, e in enumerate(node.edges)
                 if e is not None and p != from_port]
        return cands[0] if len(cands) == 1 else None

    # ── Тик симуляции/оценки ─────────────────────────────────────────────
    def tick(self, dt):
        """Продвигает все поезда на dt секунд. Возвращает список событий."""
        self.time += dt
        events = []
        for lt in self.trains.values():
            events.extend(self._advance_train(lt, dt))
        return events

    def _advance_train(self, lt, dt):
        events = []
        # 1. Джойстик относительно СХЕМЫ: знак v_target — желаемое направление по
        #    схеме, lt.face — куда смотрит нос. Знаки совпадают → просто разгон/
        #    торможение до |v_target|. Различаются → тормозим до нуля (затем
        #    разворот). Повторная команда того же знака разворот НЕ вызывает.
        vt = lt.v_target
        if vt == 0.0:
            mag_target = 0.0
        elif (vt > 0) == (lt.face > 0):
            mag_target = abs(vt)
        else:
            mag_target = 0.0     # требуется остановка перед разворотом

        # lt.v — модуль скорости (>=0)
        if lt.v < mag_target:
            lt.v = min(mag_target, lt.v + lt.accel * dt)
        elif lt.v > mag_target:
            lt.v = max(mag_target, lt.v - lt.decel * dt)
        lt.v = _clamp(lt.v, 0.0, lt.train.max_speed)

        # 2. Разворот: остановились, а ехать нужно в другую сторону по схеме.
        #    Нос меняется с хвостом, face инвертируется. v_target НЕ трогаем —
        #    он остаётся знаковым относительно схемы.
        need_reverse = (vt != 0.0 and (vt > 0) != (lt.face > 0))
        if need_reverse and lt.v <= 1e-9:
            self._reverse_train(lt)
            lt.face = -lt.face
            events.append({'type': 'reversed', 'train': lt.id,
                           'edge': lt.edge_id, 's': lt.s,
                           'direction': lt.direction, 'face': lt.face})
            return events

        if lt.v <= 1e-9:
            return events

        # 3. Перемещение (нос ведёт; ds по lt.direction, с переходами узлов).
        dist = lt.v * dt
        guard = 0
        while dist > 1e-9 and guard < 64:
            guard += 1
            edge = self.topo.get_edge(lt.edge_id)
            s_prev = lt.s
            ds_norm = lt.direction * dist / edge.length
            s_new = lt.s + ds_norm

            # Доехали ли до конца ребра в этом шаге?
            if 0.0 <= s_new <= 1.0:
                lt.s = s_new
                events += self._check_sensors(lt, s_prev, s_new)
                dist = 0.0
                break

            # Дошли до границы — частичное перемещение до узла.
            boundary = 1.0 if lt.direction > 0 else 0.0
            events += self._check_sensors(lt, s_prev, boundary)
            used = abs(boundary - lt.s) * edge.length
            dist -= used
            lt.s = boundary

            moved = self._cross_node(lt)
            if not moved:
                # Тупик/стрелка против — остановка на границе.
                lt.v = 0.0
                lt.v_target = 0.0
                events.append({'type': 'stopped', 'train': lt.id,
                               'edge': lt.edge_id, 'reason': lt.stopped_reason})
                dist = 0.0
                break
            events.append({'type': 'transition', 'train': lt.id,
                           'edge': lt.edge_id, 's': lt.s})
        return events

    def _cross_node(self, lt):
        """Переводит поезд на следующее ребро по состоянию стрелки.
        Возвращает True при успешном переходе."""
        edge = self.topo.get_edge(lt.edge_id)
        # Узел, через который выезжаем.
        exit_node = edge.node_b if lt.direction > 0 else edge.node_a
        node = self.topo.get_node(exit_node)
        from_port = self.topo.get_port(lt.edge_id, exit_node)

        to_port = self._next_port(node, from_port)
        if to_port is None or node.edges[to_port] is None:
            lt.stopped_reason = ('dead_end' if to_port is None
                                 and not isinstance(node, Switch)
                                 else 'against_switch')
            return False

        next_eid = node.edges[to_port]
        nedge = self.topo.get_edge(next_eid)

        # Фиксируем выходную координату на покидаемом ребре (для отрисовки тела).
        if lt.history and lt.history[-1][0] == lt.edge_id:
            lt.history[-1][2] = 1.0 if lt.direction > 0 else 0.0

        # Входим на новое ребро со стороны общего узла.
        if nedge.node_a == exit_node:
            lt.direction = 1
            lt.s = 0.0
            entry_s = 0.0
        else:
            lt.direction = -1
            lt.s = 1.0
            entry_s = 1.0
        lt.edge_id = next_eid
        lt.history.append([next_eid, entry_s, None])
        self._prune_history(lt)
        return True

    def _prune_history(self, lt):
        """Убирает рёбра, полностью оставшиеся позади хвоста."""
        # Совокупная длина истории от хвоста; держим запас.
        keep = []
        acc = 0.0
        need = lt.train.length + 1.0
        for item in reversed(lt.history):
            keep.append(item)
            eid, en, ex = item
            edge = self.topo.get_edge(eid)
            head_end = ex if ex is not None else lt.s
            acc += abs(head_end - en) * edge.length
            if acc >= need:
                break
        lt.history = list(reversed(keep))

    # ── Разворот поезда ──────────────────────────────────────────────────
    def _occupied_segments(self, lt):
        """Занятые рёбра от ГОЛОВЫ к хвосту: список (edge_id, s_head, s_tail).

        Тот же проход, что и в body_points, но с координатами обоих концов
        каждого занятого куска (нужно для разворота)."""
        n = len(lt.history)
        if n == 0:
            return [(lt.edge_id, lt.s, lt.s)]
        segs = []
        rem = lt.train.length
        for i in range(n - 1, -1, -1):
            eid, en, ex = lt.history[i]
            edge = self.topo.get_edge(eid)
            head_end = lt.s if i == n - 1 else (ex if ex is not None else 1 - en)
            span_m = abs(head_end - en) * edge.length
            if rem <= 1e-9:
                segs.append((eid, head_end, head_end))
                break
            if span_m >= rem and span_m > 1e-9:
                ratio = rem / span_m
                tail_s = head_end + (en - head_end) * ratio
                segs.append((eid, head_end, tail_s))
                break
            segs.append((eid, head_end, en))
            rem -= span_m
        return segs

    def _reverse_train(self, lt):
        """Меняет голову и хвост местами: новая голова = бывший хвост.

        История пересобирается так, что тело занимает те же рёбра, но движение
        теперь идёт в обратную сторону. Скорость/направление правит вызывающий.
        """
        segs = self._occupied_segments(lt)        # голова→хвост
        if not segs:
            lt.direction = -lt.direction
            return
        # Новая ориентация: для каждого куска голово-сторонний конец становится
        # хвостовым и наоборот. Порядок истории (старое→новое): бывшая голова
        # как самый старый, бывший хвост как новый головной сегмент.
        new_hist = []
        for i, (eid, hs, ts) in enumerate(segs):
            if i == len(segs) - 1:
                new_hist.append([eid, hs, None])  # новый головной сегмент
            else:
                new_hist.append([eid, hs, ts])
        head_eid, head_hs, head_ts = segs[-1]
        lt.edge_id = head_eid
        lt.s = head_ts
        if abs(head_ts - head_hs) > 1e-9:
            lt.direction = 1 if head_ts > head_hs else -1
        else:
            lt.direction = -lt.direction
        lt.history = new_hist
        self._prune_history(lt)

    def reverse(self, train_id):
        """Явный разворот на месте (нос меняет конец). Инвертирует и face, и знак
        цели, чтобы команда осталась согласованной со схемой."""
        lt = self.trains.get(train_id)
        if lt is None:
            return
        self._reverse_train(lt)
        lt.face = -lt.face
        lt.v_target = -lt.v_target

    def _check_sensors(self, lt, s_from, s_to):
        """Фиксирует пересечение датчиков на текущем ребре (только лог в sim).

        Пересечение строгое со стороны s_from: датчик ровно в точке старта
        шага считается уже пройденным (иначе после снапа к датчику он бы
        срабатывал каждый тик)."""
        if s_from == s_to:
            return []
        out = []
        for sensor in self.sensors.values():
            if sensor.edge_id != lt.edge_id:
                continue
            p = sensor.position
            crossed = (s_from < p <= s_to) or (s_to <= p < s_from)
            if crossed:
                out.append({'type': 'sensor', 'sensor': sensor.id,
                            'train': lt.id, 'edge': lt.edge_id,
                            'position': p})
        return out

    # ── Коррекция по датчику ─────────────────────────────────────────────
    def fire_sensor(self, sensor_id, train_id=None):
        """Срабатывание датчика: голова ближайшего поезда (в радиусе, по путям)
        подтягивается точно на координату датчика. Поезд может быть и НЕ на ребре
        датчика — голова переезжает на ребро датчика вдоль реального пути.

        Возвращает знаковую величину коррекции (м, + если поезд продвинулся
        вперёд) или None, если в радиусе никого нет.
        В реальном режиме вызывается мостом по сигналу от ESP32. Если train_id
        задан явно — корректируется именно он (радиус не ограничивает).
        """
        sensor = self.sensors.get(sensor_id)
        if sensor is None:
            return None

        if train_id is not None:
            lt = self.trains.get(train_id)
            if lt is None:
                return None
            match = self._head_offset(lt, sensor, math.inf)
            if match is None:
                return None
            dist, mode = match
        else:
            lt, dist, mode = self._nearest_train(sensor)
            if lt is None:
                return None

        if mode == 'fwd':
            self._apply_forward(lt, sensor)
            return dist
        else:
            self._apply_backward(lt, sensor)
            return -dist

    def _nearest_train(self, sensor):
        """Ближайший к датчику поезд (по голове, по путям, в радиусе).
        Возвращает (train, distance, mode) или (None, None, None)."""
        best = (None, None, None)
        best_d = None
        for lt in self.trains.values():
            m = self._head_offset(lt, sensor, sensor.radius)
            if m is None:
                continue
            dist, mode = m
            if best_d is None or dist < best_d:
                best_d = dist
                best = (lt, dist, mode)
        return best

    def _head_offset(self, lt, sensor, max_dist):
        """Расстояние от ГОЛОВЫ поезда до датчика вдоль путей, не больше max_dist.
        Ищем и вперёд (по маршруту стрелок), и назад (по истории); берём меньшее.
        Возвращает (distance, mode) где mode='fwd'|'bwd', либо None."""
        fwd = self._scan_forward(lt, sensor, max_dist)
        bwd = self._scan_backward(lt, sensor, max_dist)
        if fwd is None and bwd is None:
            return None
        if bwd is None or (fwd is not None and fwd <= bwd):
            return (fwd, 'fwd')
        return (bwd, 'bwd')

    def _peek_cross(self, edge_id, direction):
        """Чистый расчёт следующего ребра по состоянию стрелки (без мутаций).
        Возвращает (next_edge, next_dir, next_s) или None (тупик/против стрелки)."""
        edge = self.topo.get_edge(edge_id)
        exit_node = edge.node_b if direction > 0 else edge.node_a
        node = self.topo.get_node(exit_node)
        from_port = self.topo.get_port(edge_id, exit_node)
        to_port = self._next_port(node, from_port)
        if to_port is None or node.edges[to_port] is None:
            return None
        next_eid = node.edges[to_port]
        nedge = self.topo.get_edge(next_eid)
        if nedge.node_a == exit_node:
            return (next_eid, 1, 0.0)
        return (next_eid, -1, 1.0)

    def _scan_forward(self, lt, sensor, max_dist):
        """Дистанция от головы ВПЕРЁД по маршруту до датчика, либо None."""
        eid, s, d = lt.edge_id, lt.s, lt.direction
        dist = 0.0
        for _ in range(256):
            ln = self.topo.get_edge(eid).length
            if eid == sensor.edge_id and (sensor.position - s) * d >= -1e-9:
                add = abs(sensor.position - s) * ln
                return (dist + add) if dist + add <= max_dist + 1e-9 else None
            dist += ((1 - s) if d > 0 else s) * ln
            if dist > max_dist + 1e-9:
                return None
            nx = self._peek_cross(eid, d)
            if nx is None:
                return None
            eid, d, s = nx
        return None

    def _scan_backward(self, lt, sensor, max_dist):
        """Дистанция от головы НАЗАД по истории до датчика, либо None."""
        n = len(lt.history)
        dist = 0.0
        for i in range(n - 1, -1, -1):
            eid, en, ex = lt.history[i]
            ln = self.topo.get_edge(eid).length
            head_end = lt.s if i == n - 1 else (ex if ex is not None else 1 - en)
            if eid == sensor.edge_id:
                lo, hi = (en, head_end) if en <= head_end else (head_end, en)
                if lo - 1e-9 <= sensor.position <= hi + 1e-9:
                    total = dist + abs(head_end - sensor.position) * ln
                    return total if total <= max_dist + 1e-9 else None
            dist += abs(head_end - en) * ln
            if dist > max_dist + 1e-9:
                return None
        return None

    def _apply_forward(self, lt, sensor):
        """Двигает голову ВПЕРЁД по маршруту до датчика (история обновляется)."""
        for _ in range(256):
            if lt.edge_id == sensor.edge_id and \
                    (sensor.position - lt.s) * lt.direction >= -1e-9:
                lt.s = sensor.position
                self._prune_history(lt)
                return
            lt.s = 1.0 if lt.direction > 0 else 0.0
            if not self._cross_node(lt):
                return

    def _apply_backward(self, lt, sensor):
        """Отводит голову НАЗАД по истории до датчика (срезает пройденные рёбра)."""
        for _ in range(256):
            head_entry = lt.history[-1]
            if lt.edge_id == sensor.edge_id:
                en = head_entry[1]
                lo, hi = (en, lt.s) if en <= lt.s else (lt.s, en)
                if lo - 1e-9 <= sensor.position <= hi + 1e-9:
                    lt.s = sensor.position
                    return
            if len(lt.history) < 2:
                lt.s = head_entry[1]      # дальше истории нет — упираемся в хвостовой конец
                return
            lt.history.pop()
            prev = lt.history[-1]          # [eid, entry_s, exit_s]
            exit_s = prev[2] if prev[2] is not None else prev[1]
            lt.edge_id = prev[0]
            lt.s = exit_s
            lt.direction = 1 if exit_s >= prev[1] else -1   # сохраняем ход вперёд
            prev[2] = None                # снова головное ребро

    # ── Геометрия тела поезда ────────────────────────────────────────────
    def body_points(self, lt):
        """Полилиния тела от ГОЛОВЫ к хвосту: список (edge_id, s)."""
        pts = [(lt.edge_id, lt.s)]
        rem = lt.train.length
        if rem <= 1e-9:
            return pts
        n = len(lt.history)
        for i in range(n - 1, -1, -1):
            eid, en, ex = lt.history[i]
            edge = self.topo.get_edge(eid)
            head_end = lt.s if i == n - 1 else (ex if ex is not None else 1 - en)
            span_m = abs(head_end - en) * edge.length
            if span_m >= rem and span_m > 1e-9:
                ratio = rem / span_m
                tail_s = head_end + (en - head_end) * ratio
                pts.append((eid, tail_s))
                return pts
            pts.append((eid, en))
            rem -= span_m
        return pts

    # ── Сериализация для фронта/моста ────────────────────────────────────
    def to_dict(self):
        trains = []
        for lt in self.trains.values():
            trains.append({
                'id': lt.id, 'name': lt.name, 'color': lt.color,
                'edge_id': lt.edge_id, 's': lt.s, 'direction': lt.direction,
                'face': lt.face,
                'v': lt.v * lt.face,        # знаковая скорость по схеме (джойстик)
                'v_target': lt.v_target,    # знаковая цель по схеме
                'length': lt.train.length,
                'max_speed': (lt.train.max_speed
                              if math.isfinite(lt.train.max_speed) else None),
                'stopped_reason': lt.stopped_reason,
                'body': [{'edge_id': e, 's': s} for e, s in self.body_points(lt)],
            })
        return {
            'time': self.time,
            'trains': trains,
            'sensors': [s.to_dict() for s in self.sensors.values()],
            'switches': {str(k): int(v) for k, v in self.switches.items()},
        }


# ── Автономный прогон: проверка физики, переходов и коррекции ────────────
def _demo_topology():
    """Небольшая ветка с одной стрелкой (без правки main.py)."""
    topo = Topology()
    a = topo.add_node(1, name='A')          # тупик-старт
    c1 = topo.add_node(1, name='C1')        # тупик-конец прямого
    c2 = topo.add_node(1, name='C2')        # тупик-конец ветки
    sw = topo.add_switch(name='SW', max_speed_straight=10, max_speed_branch=6)
    e_in = topo.add_edge(a, sw, 100, 10, port_b=Port.POINT, name='e_in')
    e_st = topo.add_edge(sw, c1, 120, 10, port_a=Port.STRAIGHT, name='e_straight')
    e_br = topo.add_edge(sw, c2, 80, 6, port_a=Port.BRANCH, name='e_branch')
    return topo, dict(a=a, sw=sw, e_in=e_in, e_st=e_st, e_br=e_br)


def _demo():
    topo, ref = _demo_topology()
    print(topo.dump(), '\n')

    state = LayoutState(topo)
    train = Train(max_speed=12, accel=2, decel=2, length=20)
    lt = state.place_train(1, train, ref['e_in'], position=10, direction=1,
                           color='#3498db', name='T1')

    # Датчик в конце входного ребра — для коррекции дрейфа.
    SENSOR_S = 0.9
    state.add_sensor(10, ref['e_in'], position=SENSOR_S, name='S_in')

    # Ставим стрелку на ветку и командуем скорость.
    state.set_switch(ref['sw'], Port.BRANCH)
    state.set_speed(1, 8.0)
    print("Стрелка SW = BRANCH, цель скорости = 8 м/с")
    print("Оценка занижает скорость на 4% (имитация дрейфа dead reckoning)\n")

    # «Физический» поезд (ground truth) на входном ребре — отдельно от оценки.
    # В реальной системе это даёт ESP32; здесь — для проверки коррекции.
    e_in_len = topo.get_edge(ref['e_in']).length
    true_pos = 10.0      # метры от node_a по e_in
    true_v = 0.0
    sensor_fired = False
    DRIFT = 0.96         # оценка движется на 4% медленнее истины

    dt = 0.1
    for step in range(220):
        # Оценка (то, что «знает» backend): движется медленнее истины.
        lt.v_target = 8.0 * DRIFT
        events = state.tick(dt)

        # Ground truth на e_in (точная физика), пока не уехал на стрелку.
        if not sensor_fired:
            true_v = min(8.0, true_v + train.accel * dt)
            true_pos += true_v * dt
            if true_pos >= SENSOR_S * e_in_len and lt.edge_id == ref['e_in']:
                sensor_fired = True
                before = lt.s
                delta = state.fire_sensor(10, 1)   # сигнал «физика пересекла датчик»
                lt.v_target = 8.0
                print(f"  [t={state.time:4.1f}] ФИЗИКА пересекла датчик S_in "
                      f"(истина={true_pos:.1f} м). Оценка была s={before:.3f} "
                      f"({before*e_in_len:.1f} м) → снап к {SENSOR_S:.2f}; "
                      f"коррекция {delta:+.2f} м")

        for ev in events:
            if ev['type'] == 'transition':
                print(f"  [t={state.time:4.1f}] переход на e{ev['edge']} "
                      f"(s={ev['s']:.2f}, dir={lt.direction:+d})")
            elif ev['type'] == 'stopped':
                print(f"  [t={state.time:4.1f}] ОСТАНОВКА на e{ev['edge']} "
                      f"({ev['reason']})")
        if step % 20 == 0:
            body = state.body_points(lt)
            print(f"  t={state.time:4.1f}  edge=e{lt.edge_id}  s={lt.s:.3f}  "
                  f"v={lt.v:.2f}  тело={[(e, round(s,2)) for e,s in body]}")

    print(f"\nИтог: поезд на e{lt.edge_id} (ожидалось e{ref['e_br']} — ветка), "
          f"s={lt.s:.3f}, v={lt.v:.2f}")
    assert lt.edge_id == ref['e_br'], "поезд должен был уйти на ветку BRANCH"
    assert sensor_fired, "датчик должен был сработать"
    print("OK: маршрутизация по стрелке и коррекция по датчику работают.")


if __name__ == '__main__':
    _demo()
