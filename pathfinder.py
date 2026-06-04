"""Поиск кратчайшего по времени маршрута

Архитектура:
    Segment - единая структура: профиль + расписание (одного участка), цепочка через .prev
    Train - параметры поезда
    PathResult - итоговый маршрут (список Segment, разрешённый из цепочки)

CBS:
    find_path принимает constraints - занятость рёбер по времени
    При входе на ребро вычисляется wait_time, прибавляется к расписанию
"""

import heapq
import math
from topology import Topology, INF


# Параметры поезда
class Train:
    """Параметры поезда."""
    __slots__ = ('max_speed', 'accel', 'decel', 'length', 'clearance')

    def __init__(self, max_speed=INF, accel=INF, decel=INF, length=0.0,
                 clearance=5.0):
        self.max_speed = max_speed
        self.accel = accel
        self.decel = decel
        self.length = length
        # Запас: используется и для pocket-разворота (расстояние между хвостом
        # и стрелкой при остановке), и для виртуального светофора (расстояние
        # до узла где поезд останавливается перед занятым ребром)
        self.clearance = clearance


# Сегмент: один участок пути с временем и профилем скорости
class Segment:
    """Один участок маршрута: проезд по части ребра
    s_enter, s_exit - нормализованные позиции на ребре (0 = node_a, 1 = node_b)
    t_enter, t_exit - абсолютные времена
    v_enter, v_peak, v_exit - профиль скорости (трапеция или треугольник)
    direction - направление поезда (+1/-1)
    v_max - макс. скорость на этом участке (для повторного расчёта при коррекции)
    prev - предыдущий сегмент в цепочке (None для первого)
    """
    __slots__ = ('edge_id', 's_enter', 's_exit', 't_enter', 't_exit',
                 'v_enter', 'v_peak', 'v_exit',
                 'direction', 'v_max', 'dist',
                 't_tail_exit',
                 'prev')

    def __init__(self, edge_id, s_enter, s_exit, t_enter, t_exit,
                 v_enter, v_peak, v_exit, direction, v_max,
                 dist=0.0, prev=None):
        self.edge_id = edge_id
        self.s_enter = s_enter
        self.s_exit = s_exit
        self.t_enter = t_enter
        self.t_exit = t_exit
        self.v_enter = v_enter
        self.v_peak = v_peak
        self.v_exit = v_exit
        self.direction = direction
        self.v_max = v_max
        self.dist = dist
        self.t_tail_exit = t_exit  # значение по умолчанию, уточнится в _annotate_tail_times
        self.prev = prev

    def __repr__(self):
        sign = '+' if self.direction > 0 else '-'
        return (f'  e{self.edge_id}[{self.s_enter:.2f}→{self.s_exit:.2f}] '
                f't={self.t_enter:.3f}..{self.t_exit:.3f} '
                f'v={self.v_enter:.1f}→{self.v_peak:.1f}→{self.v_exit:.1f} '
                f'dir={sign}')


class PathResult:
    __slots__ = ('total_time', 'edges', 'schedule', 'found')

    def __init__(self, total_time=0.0, edges=None, schedule=None, found=False):
        self.total_time = total_time
        self.edges = edges or []
        self.schedule = schedule or []  # list[Segment]
        self.found = found

    def __repr__(self):
        if not self.found:
            return 'PathResult(not found)'
        lines = [f'PathResult(time={self.total_time:.3f}, edges={self.edges})']
        for s in self.schedule:
            lines.append(repr(s))
        return '\n'.join(lines)


# Физика
def _peak_speed(dist, v_enter, v_exit, v_cruise, accel, decel):
    """Реальная пиковая скорость на участке"""
    if dist <= 0: return v_enter
    if accel >= INF and decel >= INF: return v_cruise
    d_a = (v_cruise**2 - v_enter**2)/(2*accel) if accel < INF and v_cruise > v_enter else 0
    d_b = (v_cruise**2 - v_exit**2)/(2*decel) if decel < INF and v_cruise > v_exit else 0
    if d_a + d_b <= dist: return v_cruise
    if decel >= INF or d_b == 0:
        return min(v_cruise, math.sqrt(max(0, v_enter**2 + 2*accel*dist)))
    if accel >= INF or d_a == 0:
        return min(v_cruise, math.sqrt(max(0, v_exit**2 + 2*decel*dist)))
    vp2 = (2*accel*decel*dist + decel*v_enter**2 + accel*v_exit**2)/(accel+decel)
    return min(math.sqrt(max(0, vp2)), v_cruise)


def _seg_time(dist, v_enter, v_exit, v_cruise, train):
    """Время проезда участка и пиковая скорость."""
    if dist <= 0: return 0.0, v_enter
    vp = _peak_speed(dist, v_enter, v_exit, v_cruise, train.accel, train.decel)
    a, d = train.accel, train.decel
    ta = (vp-v_enter)/a if a < INF and vp > v_enter else 0
    da = (v_enter+vp)/2*ta
    tb = (vp-v_exit)/d if d < INF and vp > v_exit else 0
    db = (vp+v_exit)/2*tb
    dc = max(0, dist-da-db)
    tc = dc/vp if vp > 0 else 0
    return ta+tc+tb, vp


def _max_entry(v_exit, dist, decel):
    """Макс. скорость входа, чтобы успеть затормозить до v_exit на dist"""
    if decel >= INF: return INF
    if dist <= 0: return v_exit
    return math.sqrt(v_exit**2 + 2*decel*dist)


def _max_speed_after(dist, v_enter, v_cruise, accel):
    """Макс. скорость после участка длины dist (при разгоне)"""
    if accel >= INF or dist <= 0:
        return v_cruise if dist > 0 else v_enter
    return min(v_cruise, math.sqrt(v_enter**2 + 2*accel*dist))


# Построение сегментов для одного ребра
def _build_segments(edge_id, s_from, s_to, length, v_max_edge,
                    v_enter, v_exit_target, v_max_node_in,
                    direction, t_start, train, prev_segment):
    """Создаёт цепочку Segment для проезда по ребру (или его части)
    Учитывает длину поезда: если max_v_node_in < v_max_edge, поезд может разогнаться только после того как полностью покинет узел
    s_from, s_to - нормализованные позиции на ребре (0..1)
    length - физическое расстояние
    v_max_edge - макс. скорость на ребре (с учётом train.max_speed)
    v_enter - скорость входа
    v_exit_target - целевая скорость выхода
    v_max_node_in - ограничение узла на входе
    direction - направление поезда
    t_start - абсолютное время начала
    prev_segment - последний сегмент предыдущего ребра
    Возвращает (last_segment, total_time)
    """
    if length <= 0:
        return prev_segment, 0.0

    tl = train.length
    split = (tl > 0 and 0 < v_max_node_in < v_max_edge and length > tl)

    if split:
        # пока хвост в узле
        d1 = tl
        d2 = length - d1
        s_mid = s_from + (s_to - s_from) * d1 / length
        v_max_1 = min(v_max_node_in, v_max_edge)

        # Скорость на стыке: ограничена v_max_1 и тем что дальше
        if v_exit_target is not None:
            # Нужно успеть затормозить до v_exit_target на d2
            v_at_mid = min(_max_entry(v_exit_target, d2, train.decel), v_max_1)
        else:
            v_at_mid = _max_speed_after(d1, v_enter, v_max_1, train.accel)

        # Сегмент 1
        t1, vp1 = _seg_time(d1, v_enter, v_at_mid, v_max_1, train)
        seg1 = Segment(edge_id, s_from, s_mid, t_start, t_start + t1,
                       v_enter, vp1, v_at_mid, direction, v_max_1, d1, prev_segment)

        # Сегмент 2
        v_exit2 = v_exit_target if v_exit_target is not None \
            else _max_speed_after(d2, v_at_mid, v_max_edge, train.accel)
        t2, vp2 = _seg_time(d2, v_at_mid, v_exit2, v_max_edge, train)
        seg2 = Segment(edge_id, s_mid, s_to, t_start + t1, t_start + t1 + t2,
                       v_at_mid, vp2, v_exit2, direction, v_max_edge, d2, seg1)

        return seg2, t1 + t2
    else:
        # Один сегмент
        v_max = min(v_max_node_in, v_max_edge) if (tl > 0 and length <= tl) else v_max_edge
        v_exit = v_exit_target if v_exit_target is not None \
            else _max_speed_after(length, v_enter, v_max, train.accel)
        t, vp = _seg_time(length, v_enter, v_exit, v_max, train)
        seg = Segment(edge_id, s_from, s_to, t_start, t_start + t,
                      v_enter, vp, v_exit, direction, v_max, length, prev_segment)
        return seg, t


# Разворот на стрелке
def _build_pocket_segments(pocket_eid, pocket_edge, switch_node_id,
                           v_arrive, v_max_in, v_max_out,
                           t_arrive, switch_time,
                           direction_in, train, prev_chain):
    """Манёвр S<->B через pocket-ребро
    1. Заезд: от стрелки вглубь pocket
    2. Пауза switch_time
    3. Разворот: новая голова = бывший хвост
    prev_chain: цепочка до заезда
    Возвращает (последний_pocket_сегмент, общее_время)
    """
    pocket_dist = train.length + train.clearance

    if pocket_edge.node_a == switch_node_id:
        s_from, s_to_dir = 0.0, 1.0
    else:
        s_from, s_to_dir = 1.0, -1.0

    s_to = s_from + s_to_dir * pocket_dist / pocket_edge.length

    # Скорости:
    #   vc_pocket - макс. на самом pocket-ребре
    #   v_switch_in  - ограничение стрелки при заезде
    #   v_switch_out - ограничение стрелки при выезде
    vc_pocket = min(pocket_edge.speed, train.max_speed)
    v_switch_in = min(v_max_in, vc_pocket)
    v_switch_out = min(v_max_out, vc_pocket)

    # 1. Заезд
    tl = train.length
    if tl > 0 and 0 < v_switch_in < vc_pocket and pocket_dist > tl:
        d1 = tl
        d2 = pocket_dist - d1
        s_mid = s_from + s_to_dir * d1 / pocket_edge.length
        
        v_at_mid = min(v_switch_in, _max_entry(0, d2, train.decel))
        t1, vp1 = _seg_time(d1, v_arrive, v_at_mid, v_switch_in, train)
        seg_in_part1 = Segment(pocket_eid, s_from, s_mid,
                               t_arrive, t_arrive + t1,
                               v_arrive, vp1, v_at_mid, direction_in, v_switch_in,
                               d1, prev_chain)
        t2, vp2 = _seg_time(d2, v_at_mid, 0, vc_pocket, train)
        seg_in = Segment(pocket_eid, s_mid, s_to,
                         seg_in_part1.t_exit, seg_in_part1.t_exit + t2,
                         v_at_mid, vp2, 0, direction_in, vc_pocket,
                         d2, seg_in_part1)
        t_in = t1 + t2
    else:
        t_in, vp_in = _seg_time(pocket_dist, v_arrive, 0, vc_pocket, train)
        seg_in = Segment(pocket_eid, s_from, s_to, t_arrive, t_arrive + t_in,
                         v_arrive, vp_in, 0, direction_in, vc_pocket, pocket_dist,
                         prev_chain)

    # 2. Пауза

    # 3. Разворот
    t_after_pause = t_arrive + t_in + switch_time
    new_dir = -direction_in
    outgoing_from = s_to + (train.length / pocket_edge.length) * (-s_to_dir)
    outgoing_dist = train.clearance

    # Если выезд "заблокирован" поезд остаётся на месте
    if v_max_out <= 0:
        # Стоящий сегмент
        stand = Segment(pocket_eid, outgoing_from, outgoing_from,
                        t_arrive + t_in, t_after_pause,
                        0, 0, 0, new_dir, vc_pocket, 0.0, seg_in)
        return stand, t_in + switch_time

    # Реальная максимальная скорость которую можно достичь
    v_reach = _max_speed_after(outgoing_dist, 0, vc_pocket, train.accel)
    v_actual_exit = min(v_switch_out, v_reach)
    t_out, vp_out = _seg_time(outgoing_dist, 0, v_actual_exit, vc_pocket, train)
    seg_out = Segment(pocket_eid, outgoing_from, s_from,
                      t_after_pause, t_after_pause + t_out,
                      0, vp_out, v_actual_exit, new_dir, vc_pocket,
                      outgoing_dist, seg_in)

    return seg_out, t_in + switch_time + t_out


# Обратное торможение
def _propagate_brake(chain, v_required, train):
    """Идёт назад по цепочке, занижая скорости. Возвращает (new_chain, dt_extra)."""
    if chain is None or v_required >= chain.v_exit:
        return chain, 0.0

    corrections = []
    current = chain
    req = v_required

    while current is not None:
        mv = _max_entry(req, current.dist, train.decel)
        corrections.append((current, req))
        if current.v_enter <= mv: break
        req = mv
        current = current.prev

    deepest = corrections[-1][0]
    tail = deepest.prev
    t_start = deepest.t_enter

    new_chain = tail
    cur_t = t_start
    penalty = 0.0

    for old, new_v_exit in reversed(corrections):
        if old is deepest:
            nve = old.v_enter
        else:
            nve = min(old.v_enter, _max_entry(new_v_exit, old.dist, train.decel))

        nt, nvp = _seg_time(old.dist, nve, new_v_exit, old.v_max, train)
        old_t = old.t_exit - old.t_enter
        penalty += nt - old_t

        new_chain = Segment(
            old.edge_id, old.s_enter, old.s_exit,
            cur_t, cur_t + nt,
            nve, nvp, new_v_exit,
            old.direction, old.v_max, old.dist, new_chain)
        cur_t += nt

    return new_chain, penalty


def _chain_to_list(chain):
    """Разворачивает цепочку Segment в список (от старта к концу)."""
    result = []
    c = chain
    while c is not None:
        result.append(c)
        c = c.prev
    result.reverse()
    return result


# CBS
def _annotate_tail_times(schedule, train):
    """Заполняет seg.t_tail_exit для всех сегментов"""
    if train.length <= 0:
        for seg in schedule:
            seg.t_tail_exit = seg.t_exit
        return

    n = len(schedule)
    for i, seg in enumerate(schedule):
        j = i
        while j + 1 < n and schedule[j + 1].edge_id == seg.edge_id:
            j += 1
        remaining = train.length
        t_tail = schedule[j].t_exit
        found = False
        for k in range(j + 1, n):
            nxt = schedule[k]
            if nxt.dist >= remaining:
                t_in_seg = _time_to_distance(nxt, remaining, train)
                t_tail = nxt.t_enter + t_in_seg
                found = True
                break
            remaining -= nxt.dist
            t_tail = nxt.t_exit
        if not found:
            t_tail = INF
        seg.t_tail_exit = t_tail


def _time_to_distance(seg, dist_target, train):
    """Время прохождения dist_target от начала сегмента seg"""
    if dist_target <= 0:
        return 0.0
    if dist_target >= seg.dist:
        return seg.t_exit - seg.t_enter

    ve, vp, vx = seg.v_enter, seg.v_peak, seg.v_exit
    ac, dc = train.accel, train.decel

    d1 = 0.0
    if vp > ve and ac < INF:
        t1 = (vp - ve) / ac
        d1 = (ve + vp) / 2 * t1
    d3 = 0.0
    if vp > vx and dc < INF:
        t3 = (vp - vx) / dc
        d3 = (vp + vx) / 2 * t3
    d2 = max(0.0, seg.dist - d1 - d3)

    # разгон
    if dist_target <= d1 and d1 > 0:
        # ve*t + 0.5*ac*t² = dist_target → t = (-ve + sqrt(ve² + 2*ac*dist_target)) / ac
        return (-ve + (ve * ve + 2 * ac * dist_target) ** 0.5) / ac
    rem = dist_target - d1
    t = (vp - ve) / ac if (vp > ve and ac < INF) else 0.0
    # крейс
    if rem <= d2 and vp > 0:
        return t + rem / vp
    t += d2 / vp if vp > 0 else 0.0
    rem -= d2
    # торможение
    disc = max(0.0, vp * vp - 2 * dc * rem)
    return t + (vp - disc ** 0.5) / dc


def schedule_to_constraints(schedule, cap_inf=True):
    """Преобразует расписание поезда в ограничение для других поездов
    Возвращает {edge_id: [(t_head_enter, t_tail_exit), ...]}
    """
    if not schedule:
        return {}
    total = schedule[-1].t_exit
    final_edge = schedule[-1].edge_id
    by_edge = {}
    for seg in schedule:
        eid = seg.edge_id
        te = seg.t_tail_exit
        if cap_inf and te == INF and eid != final_edge:
            te = total
        by_edge.setdefault(eid, []).append((seg.t_enter, te))

    # Объединение пересекающихся интервалов
    result = {}
    for eid, intervals in by_edge.items():
        intervals.sort()
        merged = [intervals[0]]
        for t_from, t_to in intervals[1:]:
            last_from, last_to = merged[-1]
            if t_from <= last_to:
                merged[-1] = (last_from, max(last_to, t_to))
            else:
                merged.append((t_from, t_to))
        result[eid] = merged
    return result


def _edge_free_for(edge_id, t_arrive, duration, constraints):
    """Когда можно занять ребро без пересечений
    Проверяет пересечение данного [t_try, t_try+duration] с другим
    Если пересекается - сдвиг t_try
    """
    if not constraints or edge_id not in constraints:
        return t_arrive
    busy = constraints[edge_id]
    t_try = t_arrive
    while True:
        t_end = t_try + duration
        conflict_end = None
        for tf, tt in busy:
            # Пересечение [t_try, t_end] с [tf, tt]
            if t_try < tt and t_end > tf:
                conflict_end = max(conflict_end or 0, tt)
        if conflict_end is None or conflict_end == INF:
            return t_try if conflict_end is None else INF
        t_try = conflict_end


def _wait_in_place(c, t_free, train):
    """Ожидание на текущей позиции"""
    if c is None or t_free <= c.t_exit:
        return c
    cur = c
    if cur.v_exit > 0:
        brake_time = cur.v_exit / train.decel if train.decel > 0 else 0
        brake_seg = Segment(cur.edge_id, cur.s_exit, cur.s_exit,
                            cur.t_exit, cur.t_exit + brake_time,
                            cur.v_exit, cur.v_exit, 0, cur.direction, cur.v_max, 0.0, cur)
        cur = brake_seg
    # Сегмент стоянки
    if t_free > cur.t_exit:
        cur = Segment(cur.edge_id, cur.s_exit, cur.s_exit,
                      cur.t_exit, t_free,
                      0, 0, 0, cur.direction, cur.v_max, 0.0, cur)
    return cur


def _wait_at_signal(c, switch_node_id, train, t_free):
    """Останавливает поезд до узла на ребре, ждёт t_free
    Возвращает (new_chain, total_extra_time)
    """
    if c is None or t_free <= c.t_exit:
        return c, 0.0

    last = c

    # для коротких рёбер
    if last.dist <= train.clearance:
        stand = Segment(last.edge_id, last.s_exit, last.s_exit,
                        last.t_exit, t_free,
                        0, 0, 0, last.direction, last.v_max, 0.0, last)
        return stand, t_free - last.t_exit

    # Замена последннго сегмента
    s_dir = 1 if last.s_exit > last.s_enter else -1
    new_s_exit = last.s_exit - s_dir * train.clearance / last.dist
    new_dist = last.dist - train.clearance
    t_brake, vp_brake = _seg_time(new_dist, last.v_enter, 0, last.v_max, train)
    new_last = Segment(last.edge_id, last.s_enter, new_s_exit,
                       last.t_enter, last.t_enter + t_brake,
                       last.v_enter, vp_brake, 0, last.direction, last.v_max,
                       new_dist, last.prev)

    # Сегмент стоянки
    stand_end = new_last
    if t_free > new_last.t_exit:
        stand_end = Segment(last.edge_id, new_s_exit, new_s_exit,
                            new_last.t_exit, t_free,
                            0, 0, 0, last.direction, last.v_max, 0.0, new_last)

    # Разгон
    v_after = _max_speed_after(train.clearance, 0, last.v_max, train.accel)
    t_move, vp_move = _seg_time(train.clearance, 0, v_after, last.v_max, train)
    advance = Segment(last.edge_id, new_s_exit, last.s_exit,
                      stand_end.t_exit, stand_end.t_exit + t_move,
                      0, vp_move, v_after, last.direction, last.v_max,
                      train.clearance, stand_end)

    return advance, advance.t_exit - last.t_exit


# Поиск пути
def find_path(topo, start_edge, start_offset,
              goal_edge, goal_offset, train=None,
              constraints=None, t_start=0.0):
    """Кратчайший маршрут с точной физикой
    constraints: {edge_id: [(t_from, t_to), ...]} - занятость рёбер для CBS
    """
    if train is None:
        train = Train()
    if constraints is None:
        constraints = {}

    if t_start != 0:
        adj_constraints = {}
        for eid, intervals in constraints.items():
            adj = []
            for tf, tt in intervals:
                new_tf = tf - t_start
                new_tt = tt - t_start
                if new_tt > 0:
                    adj.append((max(0.0, new_tf), new_tt))
            if adj:
                adj_constraints[eid] = adj
        constraints = adj_constraints

    s_edge = topo.get_edge(start_edge)
    g_edge = topo.get_edge(goal_edge)

    best_time = INF
    best_chain = None

    if start_edge == goal_edge:
        dist = abs(goal_offset - start_offset)
        vc = min(s_edge.speed, train.max_speed)
        se = start_offset / s_edge.length
        sx = goal_offset / s_edge.length
        t, vp = _seg_time(dist, 0, 0, vc, train)
        seg = Segment(start_edge, se, sx, 0, t, 0, vp, 0, 1, vc, dist, None)
        best_time = t
        best_chain = seg

    # Дейкстра
    best = {} # state - лучшее время
    chain_at = {} # state - Segment (последний в цепочке)
    pq = []

    vc0 = min(s_edge.speed, train.max_speed)
    for node_id in (s_edge.node_a, s_edge.node_b):
        if node_id == s_edge.node_a:
            dist = start_offset
            se, sx = start_offset / s_edge.length, 0.0
        else:
            dist = s_edge.length - start_offset
            se, sx = start_offset / s_edge.length, 1.0

        # Для стартового ребра: ограничения нет, выход на оптимистичной скорости
        vx = _max_speed_after(dist, 0, vc0, train.accel)
        t, vp = _seg_time(dist, 0, vx, vc0, train)

        seg = Segment(start_edge, se, sx, 0, t, 0, vp, vx, 1, vc0, dist, None)
        port = topo.get_port(start_edge, node_id)
        state = (node_id, port)

        if t < best.get(state, INF):
            best[state] = t
            chain_at[state] = seg
            heapq.heappush(pq, (t, node_id, port))

    while pq:
        cur_t, nid, fport = heapq.heappop(pq)
        state = (nid, fport)

        if cur_t > best.get(state, INF): continue
        if cur_t >= best_time: break

        chain = chain_at[state]
        v_arrive = chain.v_exit
        cur_dir = chain.direction

        for nb_id, arr_port, eid, length, espeed, max_v_in, v_out, delay, \
                reverses, pocket_eid, v_pocket_out in topo.get_transitions(state):

            edge = topo.get_edge(eid)
            vc = min(espeed, train.max_speed)

            # Скорость через узел
            if reverses and pocket_eid is not None and train.length > 0:
                vc_pocket = min(topo.get_edge(pocket_eid).speed, train.max_speed)
                v_in = min(v_arrive, max_v_in, vc_pocket)
            else:
                v_in = min(v_arrive, max_v_in, vc)
            v_after_node = v_out if v_out is not None else v_in
            v_after_node = min(v_after_node, vc)

            new_dir = -cur_dir if reverses else cur_dir

            # Обратное торможение для ограничения узла
            c, brake_pen = (chain, 0.0)
            if v_in < v_arrive:
                c, brake_pen = _propagate_brake(chain, v_in, train)

            # S<->B разворот
            if pocket_eid is not None and train.length > 0:
                pocket_edge = topo.get_edge(pocket_eid)

                # pocket-ребро занимается поездом
                vc_pocket = min(pocket_edge.speed, train.max_speed)
                pocket_duration = (
                    (train.length + train.clearance) * 2 / vc_pocket
                    if vc_pocket > 0 else INF)
                t_pocket_free = _edge_free_for(
                    pocket_eid, c.t_exit, pocket_duration, constraints)
                if t_pocket_free == INF:
                    continue
                if t_pocket_free > c.t_exit:
                    c, _ = _wait_at_signal(c, nid, train, t_pocket_free)
                    v_in = min(c.v_exit, max_v_in)
                    v_arrive_pocket = v_in
                else:
                    v_arrive_pocket = v_in

                t_pocket_estimate = c.t_exit + pocket_duration / 2 + delay
                my_duration_next = (length + train.length) / vc if vc > 0 else INF
                t_next_free = _edge_free_for(eid, t_pocket_estimate,
                                              my_duration_next, constraints)
                blocked_after_pocket = t_next_free > t_pocket_estimate

                pocket_chain, pocket_dt = _build_pocket_segments(
                    pocket_eid, pocket_edge, nid,
                    v_arrive_pocket, max_v_in,
                    0.0 if blocked_after_pocket else v_pocket_out,
                    c.t_exit, delay,
                    cur_dir, train, c)
                c = pocket_chain
                brake_pen = 0.0

                # Если выезд был заблокирован - стоим на месте бывшего хвоста до t_free
                if blocked_after_pocket:
                    t_next_free = _edge_free_for(eid, c.t_exit,
                                                  my_duration_next, constraints)
                    if t_next_free == INF:
                        continue
                    if t_next_free > c.t_exit:
                        c = _wait_in_place(c, t_next_free, train)
                    out_s_from = c.s_exit
                    if pocket_edge.node_a == nid:
                        out_s_to, out_dir_sign = 0.0, -1
                    else:
                        out_s_to, out_dir_sign = 1.0, 1
                    out_dist = train.clearance
                    vc_out = min(pocket_edge.speed, train.max_speed)
                    v_reach = _max_speed_after(out_dist, 0, vc_out, train.accel)
                    v_actual = min(v_pocket_out, v_reach)
                    t_out_seg, vp_out = _seg_time(out_dist, 0, v_actual, vc_out, train)
                    delta_seg = Segment(pocket_eid, out_s_from, out_s_to,
                                        c.t_exit, c.t_exit + t_out_seg,
                                        0, vp_out, v_actual, c.direction, vc_out,
                                        out_dist, c)
                    c = delta_seg

                v_after_node = c.v_exit
                v_in = c.v_exit
                max_v_in = v_pocket_out
                delay = 0.0

            # Время прибытия на новое ребро
            t_at_edge = c.t_exit + delay

            # Ограничение для хвоста
            v_tail_limit = min(max_v_in, c.v_max)

            # Направление на новом ребре
            if edge.node_a == nid:
                rse, rsx = 0.0, 1.0
            else:
                rse, rsx = 1.0, 0.0

            # Если цель на ребре
            if eid == goal_edge:
                if edge.node_a == nid:
                    gdist = goal_offset
                    gse, gsx = 0.0, goal_offset / edge.length
                else:
                    gdist = edge.length - goal_offset
                    gse, gsx = 1.0, goal_offset / edge.length

                # Ограничение торможения до 0 на цели
                max_v_for_goal = min(_max_entry(0, gdist, train.decel), vc)
                if v_out is not None and v_out > max_v_for_goal:
                    continue  # не успеваем затормозить

                my_duration = (gdist + train.length) / vc if vc > 0 else INF
                t_free = _edge_free_for(eid, t_at_edge, my_duration, constraints)
                if t_free == INF:
                    continue
                if t_free > t_at_edge:
                    if pocket_eid is not None and train.length > 0 and c.edge_id == pocket_eid:
                        c = _wait_in_place(c, t_free, train)
                        t_at_edge = c.t_exit + delay
                        v_after_node = 0.0
                    else:
                        c, _ = _wait_at_signal(c, nid, train, t_free)
                        t_at_edge = c.t_exit + delay
                        v_after_node = min(c.v_exit, v_in if v_out is None else v_out)
                        if v_out is not None:
                            v_after_node = v_out

                # Доп. торможение для goal_speed=0
                gc = c
                if v_after_node > max_v_for_goal and v_out is None:
                    target = max_v_for_goal
                    gc, _ = _propagate_brake(chain, target, train)
                    v_after_node = target
                    t_at_edge = gc.t_exit + delay

                goal_seg, gt = _build_segments(
                    eid, gse, gsx, gdist, vc,
                    v_after_node, 0, v_tail_limit,
                    new_dir, t_at_edge, train, gc)

                t_total = t_at_edge + gt
                if t_total < best_time:
                    best_time = t_total
                    best_chain = goal_seg

            # Обычный переход
            my_duration = (length + train.length) / vc if vc > 0 else INF
            t_free = _edge_free_for(eid, t_at_edge, my_duration, constraints)
            if t_free == INF:
                continue
            if t_free > t_at_edge:
                # Если был pocket-разворот - стоянка у стрелки
                if pocket_eid is not None and train.length > 0 and c.edge_id == pocket_eid:
                    c = _wait_in_place(c, t_free, train)
                    t_at_edge = c.t_exit + delay
                    v_after_node = 0.0
                    v_in = 0.0
                else:
                    c, _ = _wait_at_signal(c, nid, train, t_free)
                    t_at_edge = c.t_exit + delay
                    v_after_node = min(c.v_exit, max_v_in, vc)
                    v_in = v_after_node

            seg, et = _build_segments(
                eid, rse, rsx, length, vc,
                v_after_node, None, v_tail_limit,
                new_dir, t_at_edge, train, c)

            new_t = t_at_edge + et
            new_state = (nb_id, arr_port)

            if new_t < best.get(new_state, INF):
                best[new_state] = new_t
                chain_at[new_state] = seg
                heapq.heappush(pq, (new_t, nb_id, arr_port))

    if best_chain is None:
        return PathResult()

    schedule = _chain_to_list(best_chain)
    _annotate_tail_times(schedule, train)

    # Сдвиг всё на t_start
    if t_start != 0:
        for seg in schedule:
            seg.t_enter += t_start
            seg.t_exit += t_start
            if seg.t_tail_exit != INF:
                seg.t_tail_exit += t_start
        best_time += t_start

    edges = list(dict.fromkeys(s.edge_id for s in schedule))
    return PathResult(best_time, edges, schedule, True)


# CBS
class CBSResult:
    """Результат CBS поиска
    success: True если найдено бесконфликтное решение
    results: list[PathResult] по одному на поезд. PathResult(found=False) если не найдено
    log: список строк описания принятых решений
    iterations: количество итераций CBS
    """
    __slots__ = ('success', 'results', 'log', 'iterations')

    def __init__(self, success=False, results=None, log=None, iterations=0):
        self.success = success
        self.results = results or []
        self.log = log or []
        self.iterations = iterations

    def __repr__(self):
        status = 'OK' if self.success else 'FAILED'
        lines = [f'CBSResult({status}, iters={self.iterations})']
        for i, r in enumerate(self.results):
            if r and r.found:
                lines.append(f'  Train {i}: time={r.total_time:.2f}, edges={r.edges}')
            else:
                lines.append(f'  Train {i}: NOT FOUND')
        return '\n'.join(lines)


def _detect_conflict(schedules):
    """Находит первый конфликт между расписаниями"""
    n = len(schedules)
    intervals = [{} for _ in range(n)]
    for ti, sched in enumerate(schedules):
        if sched is None: continue
        if not sched: continue
        total = sched[-1].t_exit
        final_edge = sched[-1].edge_id
        for seg in sched:
            te = seg.t_tail_exit
            if te == INF and seg.edge_id != final_edge:
                te = total
            intervals[ti].setdefault(seg.edge_id, []).append(
                (seg.t_enter, te))

    earliest = None
    for a in range(n):
        for b in range(a + 1, n):
            common = set(intervals[a].keys()) & set(intervals[b].keys())
            for eid in common:
                for a_from, a_to in intervals[a][eid]:
                    for b_from, b_to in intervals[b][eid]:
                        if a_from < b_to and b_from < a_to:
                            t_start = max(a_from, b_from)
                            if earliest is None or t_start < earliest[0]:
                                earliest = (t_start, a, b, eid,
                                            (a_from, a_to), (b_from, b_to))
    if earliest is None:
        return None
    return earliest[1:]


def _merge_intervals(intervals):
    """Объединяет пересекающиеся интервалы."""
    if not intervals:
        return []
    sorted_iv = sorted(intervals)
    merged = [sorted_iv[0]]
    for t_from, t_to in sorted_iv[1:]:
        last_from, last_to = merged[-1]
        if t_from <= last_to:
            merged[-1] = (last_from, max(last_to, t_to))
        else:
            merged.append((t_from, t_to))
    return merged


def _add_constraint(constraints, edge_id, interval):
    """Добавляет интервал в constraints"""
    new_c = {k: list(v) for k, v in constraints.items()}
    new_c.setdefault(edge_id, []).append(interval)
    new_c[edge_id] = _merge_intervals(new_c[edge_id])
    return new_c


def _replan_train(topo, task, constraints, t_start=0.0):
    """Перепланирует один поезд с заданными constraints"""
    return find_path(
        topo,
        task['start_edge'], task['start_offset'],
        task['goal_edge'], task['goal_offset'],
        train=task['train'],
        constraints=constraints,
        t_start=t_start)


def _total_cost(results):
    """Суммарное время всех поездов"""
    total = 0.0
    for r in results:
        if r is None or not r.found:
            return INF
        total += r.total_time
    return total


def solve_cbs(topo, tasks, max_iterations=300):
    """Conflict-Based Search для нескольких поездов
    tasks: [{'train', 'start_edge', 'start_offset', 'goal_edge', 'goal_offset'}, ...]
    Возвращает CBSResult с success/results/log/iterations
    """
    log = []
    n = len(tasks)

    root_results = []
    root_constraints = [{} for _ in range(n)]
    for i, task in enumerate(tasks):
        r = _replan_train(topo, task, root_constraints[i])
        root_results.append(r)
        if not r.found:
            log.append(f"Train {i}: no path from start (root)")

    if any(not r.found for r in root_results):
        log.append("Some trains have no path even without constraints - CBS aborted")
        return CBSResult(False, root_results, log, 0)

    cost = _total_cost(root_results)
    log.append(f"Root: total cost = {cost:.2f}")

    counter = 0
    heap = [(cost, counter, root_results, root_constraints, list(log))]
    seen = set()

    def _hash_constraints(constraints_list):
        return tuple(
            tuple(sorted((eid, tuple(intervals))
                         for eid, intervals in c.items()))
            for c in constraints_list
        )

    best_partial = root_results
    best_log = list(log)
    iterations = 0

    attempt_count = {}

    while heap and iterations < max_iterations:
        iterations += 1
        cost, _, results, constraints, branch_log = heapq.heappop(heap)

        # Пропускаем дубликаты
        h = _hash_constraints(constraints)
        if h in seen:
            continue
        seen.add(h)

        # Обнаружение конфликта
        schedules = [r.schedule if r and r.found else None for r in results]
        conflict = _detect_conflict(schedules)

        if conflict is None:
            branch_log.append(f"Бесконфликтное решение найдено, cost={cost:.2f}")
            return CBSResult(True, results, branch_log, iterations)

        a, b, eid, a_int, b_int = conflict
        conflict_msg = (f"Конфликт: train {a} ({a_int[0]:.2f}..{a_int[1]:.2f}) "
                        f"vs train {b} ({b_int[0]:.2f}..{b_int[1]:.2f}) "
                        f"на edge {eid}")
        branch_log.append(conflict_msg)
        best_log = branch_log

        start_edge_a = tasks[a]['start_edge']
        start_edge_b = tasks[b]['start_edge']

        def try_branch(ti, eid, avoid_interval, t_start):
            new_c = list(constraints)
            new_c[ti] = _add_constraint(constraints[ti], eid, avoid_interval)
            new_r = _replan_train(topo, tasks[ti], new_c[ti], t_start=t_start)
            if not new_r.found:
                return None
            new_results = list(results)
            new_results[ti] = new_r
            new_cost = _total_cost(new_results)
            msg = (f"  → Train {ti} избегает edge {eid} в "
                   f"[{avoid_interval[0]:.2f}..{avoid_interval[1]:.2f}]"
                   f"{f', старт с t={t_start:.2f}' if t_start else ''}: "
                   f"новое время {new_r.total_time:.2f}")
            return (new_cost, new_results, new_c, msg)

        def try_full_yield(yielder, other, t_start):
            """Полный пропуск (резервный алоритм)"""
            other_constraints = schedule_to_constraints(results[other].schedule)
            new_c = list(constraints)
            merged = {k: list(v) for k, v in constraints[yielder].items()}
            for e_id, intervals in other_constraints.items():
                merged[e_id] = _merge_intervals(merged.get(e_id, []) + intervals)
            new_c[yielder] = merged
            new_r = _replan_train(topo, tasks[yielder], new_c[yielder], t_start=t_start)
            if not new_r.found:
                return None
            new_results = list(results)
            new_results[yielder] = new_r
            new_cost = _total_cost(new_results)
            msg = (f"  ⇒ Train {yielder} полностью пропускает Train {other}"
                   f"{f', старт с t={t_start:.2f}' if t_start else ''}: "
                   f"время {new_r.total_time:.2f}")
            return (new_cost, new_results, new_c, msg)

        def grow_interval(interval, attempts):
            if attempts == 0:
                return interval
            t_from, t_to = interval
            mid = (t_from + t_to) / 2
            grow = (t_to - t_from) / 2 * (2 ** attempts)
            return (max(0.0, mid - grow), mid + grow)

        def add_branches_for(ti, eid, avoid, start_eid):
            avoid_grown = grow_interval(avoid, attempt_count.get((ti, eid), 0))
            if eid == start_eid:
                yield try_branch(ti, eid, avoid_grown, avoid_grown[1])
            else:
                yield try_branch(ti, eid, avoid_grown, 0.0)
                yield try_branch(ti, eid, avoid_grown, avoid_grown[1])

        def yield_branches_for(yielder, other):
            """Ветки полного пропуска с разными t_start."""
            other_milestones = sorted({
                tt for ints in schedule_to_constraints(results[other].schedule).values()
                for _, tt in ints if tt < INF})
            ts = [0.0]
            if other_milestones:
                ts.append(other_milestones[len(other_milestones) // 2])
                ts.append(other_milestones[-1])
            for t in ts:
                yield try_full_yield(yielder, other, t)

        # Собираем ветки
        branches = list(add_branches_for(a, eid, b_int, start_edge_a))
        branches += list(add_branches_for(b, eid, a_int, start_edge_b))
        branches += list(yield_branches_for(a, b))
        branches += list(yield_branches_for(b, a))

        # Регистрируем попытки
        attempt_count[(a, eid)] = attempt_count.get((a, eid), 0) + 1
        attempt_count[(b, eid)] = attempt_count.get((b, eid), 0) + 1

        for r in branches:
            if r is None: continue
            new_cost, new_results, new_c, msg = r
            counter += 1
            heapq.heappush(heap, (new_cost, counter, new_results, new_c,
                                  branch_log + [msg]))

        if all(r is None for r in branches):
            branch_log.append("  Ни одна из веток не нашла решения")

        best_partial = results

    best_log.append(f"CBS исчерпан после {iterations} итераций")

    best_log.append("Fallback: Prioritized Planning с перебором порядков")
    import itertools

    def try_pp_order(order):
        """Возвращает (results, conflict_or_None) или None если не все нашли."""
        results = [None] * len(tasks)
        cs = {}
        for i in order:
            r = _replan_train(topo, tasks[i], cs)
            if not r.found:
                return None
            results[i] = r
            for eid, ints in schedule_to_constraints(r.schedule).items():
                cs[eid] = _merge_intervals(cs.get(eid, []) + ints)
        return results, _detect_conflict([r.schedule for r in results])

    n = len(tasks)
    orders = [list(range(n)), list(range(n - 1, -1, -1))]
    if n <= 5:
        for p in itertools.permutations(range(n)):
            if list(p) not in orders:
                orders.append(list(p))
                if len(orders) >= 24: break

    best_pp = None
    for order in orders:
        res = try_pp_order(order)
        if res is None:
            best_log.append(f"  PP-порядок {order}: не все нашли путь")
            continue
        pp_results, conflict = res
        if conflict is None:
            best_log.append(f"  PP-порядок {order}: УСПЕХ")
            return CBSResult(True, pp_results, best_log, iterations)
        a, b, eid, _, _ = conflict
        best_log.append(f"  PP-порядок {order}: конфликт {a} vs {b} на e{eid}")
        if best_pp is None:
            best_pp = (pp_results, order)

    if best_pp is not None:
        pp_results, order = best_pp
        best_log.append(f" Бесконфликтное не найдено. "
                        f"Возврат PP-порядка {order} с конфликтами.")
        return CBSResult(False, pp_results, best_log, iterations)

    return CBSResult(False, best_partial, best_log, iterations)

