from enum import IntEnum

class Port(IntEnum):
    POINT = 0
    STRAIGHT = 1
    BRANCH = 2

INF = float('inf')


class Node:
    """Базовый узел. Проезд с ограничением скорости max_v.
    edges[port] = edge_id или None.
    """
    __slots__ = ('id', 'name', 'edges', 'max_v')

    def __init__(self, node_id, port_count=0, name='', max_v=INF, **_):
        self.id = node_id
        self.name = name or str(node_id)
        self.edges = [None] * port_count
        self.max_v = max_v

    @property
    def port_count(self):
        return len(self.edges)

    def _grow_ports(self):
        n = len(self.edges)
        self.edges.append(None)
        return n

    @classmethod
    def from_existing(cls, old, **params):
        new = cls(old.id, name=old.name, **params)
        for i, eid in enumerate(old.edges):
            if i < len(new.edges):
                new.edges[i] = eid
            else:
                new.edges.append(eid)
        return new

    def get_transitions(self, from_port, edges, nodes):
        """Кортеж: (nb_id, arr_port, edge_id, length, edge_speed,
                    max_v_in, v_out, time_delay, reverses, pocket_edge_id)"""
        result = []
        for to_port, eid in enumerate(self.edges):
            if to_port == from_port or eid is None or eid not in edges:
                continue
            edge = edges[eid]
            nb_id = edge.other_node(self.id)
            if nb_id not in nodes:
                continue
            arr_port = nodes[nb_id].edges.index(eid)
            result.append((nb_id, arr_port, eid, edge.length, edge.speed,
                           self.max_v, None, 0.0, False, None, None))
        return result

    def __repr__(self):
        name = f' "{self.name}"' if self.name else ''
        return f'Node({self.id}, {self.port_count}p{name})'


class Switch(Node):
    """Стрелка. P↔S: ограничение. P↔B: ограничение. S↔B: остановка + разворот."""
    __slots__ = ('max_speed_straight', 'max_speed_branch', 'switch_time')

    def __init__(self, node_id, name='', max_speed_straight=INF,
                 max_speed_branch=INF, switch_time=0.0, **_):
        super().__init__(node_id, port_count=3, name=name)
        self.max_speed_straight = max_speed_straight
        self.max_speed_branch = max_speed_branch
        self.switch_time = switch_time

    def get_transitions(self, from_port, edges, nodes):
        P, S, B = Port.POINT, Port.STRAIGHT, Port.BRANCH
        # Ребро на порту POINT (для S↔B заезда)
        point_edge_id = self.edges[P]
        result = []
        for to_port, eid in enumerate(self.edges):
            if to_port == from_port or eid is None or eid not in edges:
                continue
            edge = edges[eid]
            nb_id = edge.other_node(self.id)
            if nb_id not in nodes:
                continue
            arr_port = nodes[nb_id].edges.index(eid)
            ports = frozenset((from_port, to_port))
            pocket = None
            v_pocket_out = None  # только для S↔B
            if ports == frozenset((P, S)):
                mv, vo, d, rev = self.max_speed_straight, None, 0.0, False
            elif ports == frozenset((P, B)):
                mv, vo, d, rev = self.max_speed_branch, None, 0.0, False
            else:  # S↔B: разворот через POINT-ребро
                # Заезд: from_port → POINT → ограничение по from_port
                # Выезд: POINT → to_port → ограничение по to_port
                if from_port == S:
                    mv = self.max_speed_straight       # заезд S→P
                    v_pocket_out = self.max_speed_branch  # выезд P→B
                else:
                    mv = self.max_speed_branch         # заезд B→P
                    v_pocket_out = self.max_speed_straight  # выезд P→S
                vo, d, rev = 0.0, self.switch_time, True
                pocket = point_edge_id
            result.append((nb_id, arr_port, eid, edge.length, edge.speed,
                           mv, vo, d, rev, pocket, v_pocket_out))
        return result

    def __repr__(self):
        return f'Switch({self.id} "{self.name}")'


class Turntable(Node):
    """Поворотный круг. Остановка + поворот + разворот."""
    __slots__ = ('rotation_time_per_port',)

    def __init__(self, node_id, port_count=0, name='',
                 rotation_time_per_port=1.0, **_):
        super().__init__(node_id, port_count=port_count, name=name)
        self.rotation_time_per_port = rotation_time_per_port

    def get_transitions(self, from_port, edges, nodes):
        result = []
        for to_port, eid in enumerate(self.edges):
            if to_port == from_port or eid is None or eid not in edges:
                continue
            edge = edges[eid]
            nb_id = edge.other_node(self.id)
            if nb_id not in nodes:
                continue
            arr_port = nodes[nb_id].edges.index(eid)
            steps = min(abs(to_port - from_port),
                        self.port_count - abs(to_port - from_port))
            result.append((nb_id, arr_port, eid, edge.length, edge.speed,
                           0.0, 0.0, steps * self.rotation_time_per_port, True, None, None))
        return result

    def __repr__(self):
        return f'Turntable({self.id}, {self.port_count}p "{self.name}")'


class Edge:
    __slots__ = ('id', 'name', 'node_a', 'node_b', 'length', 'speed')

    def __init__(self, edge_id, node_a, node_b, length, speed, name=''):
        self.id = edge_id
        self.name = name or str(edge_id)
        self.node_a = node_a
        self.node_b = node_b
        self.length = length
        self.speed = speed

    def other_node(self, node_id):
        return self.node_b if self.node_a == node_id else self.node_a

    def __repr__(self):
        return f'Edge({self.id}, {self.node_a}↔{self.node_b}, L={self.length} "{self.name}")'


AUTO_TYPE = {3: Switch}


class Topology:
    def __init__(self):
        self._nodes = {}
        self._edges = {}
        self._transitions = {}
        self._next_node_id = 0
        self._next_edge_id = 0

    @property
    def node_count(self):
        return len(self._nodes)

    @property
    def edge_count(self):
        return len(self._edges)

    def _rebuild_node(self, nid):
        node = self._nodes[nid]
        for fp in range(node.port_count):
            self._transitions[(nid, fp)] = node.get_transitions(
                fp, self._edges, self._nodes)

    def _auto_reclassify(self, nid):
        node = self._nodes[nid]
        cls = AUTO_TYPE.get(node.port_count)
        if cls and type(node) is Node:
            self._nodes[nid] = cls.from_existing(node)
            self._rebuild_node(nid)

    def add_node(self, port_count=0, max_v=INF, name='', **kw):
        nid = self._next_node_id; self._next_node_id += 1
        self._nodes[nid] = Node(nid, port_count, name, max_v=max_v, **kw)
        self._rebuild_node(nid)
        return nid

    def add_existing_node(self, node):
        nid = node.id
        if nid >= self._next_node_id:
            self._next_node_id = nid + 1
        self._nodes[nid] = node
        self._rebuild_node(nid)
        return nid

    def add_switch(self, name='', **kw):
        nid = self._next_node_id
        return self.add_existing_node(Switch(nid, name=name, **kw))

    def add_turntable(self, port_count, name='', **kw):
        nid = self._next_node_id
        return self.add_existing_node(
            Turntable(nid, port_count=port_count, name=name, **kw))

    def add_edge(self, node_a, node_b, length, speed,
                 port_a=None, port_b=None, name=''):
        if node_a is None: node_a = self.add_node()
        if node_b is None: node_b = self.add_node()
        port_a = self._resolve_port(node_a, port_a)
        port_b = self._resolve_port(node_b, port_b)
        eid = self._next_edge_id; self._next_edge_id += 1
        self._edges[eid] = Edge(eid, node_a, node_b, length, speed, name)
        self._nodes[node_a].edges[port_a] = eid
        self._nodes[node_b].edges[port_b] = eid
        self._rebuild_node(node_a)
        self._rebuild_node(node_b)
        return eid

    def _resolve_port(self, nid, port):
        node = self._nodes[nid]
        if port is not None:
            if port >= len(node.edges):
                raise ValueError(f"Node {nid}: port {port} вне диапазона")
            if node.edges[port] is not None:
                raise ValueError(f"Node {nid}: port {port} уже занят")
            return port
        for i, e in enumerate(node.edges):
            if e is None: return i
        p = node._grow_ports()
        self._auto_reclassify(nid)
        return p

    def convert_node(self, nid, cls, **params):
        self._nodes[nid] = cls.from_existing(self._nodes[nid], **params)
        self._rebuild_node(nid)

    def remove_node(self, nid):
        node = self._nodes[nid]
        removed = []
        for eid in node.edges:
            if eid is not None and eid in self._edges:
                self.remove_edge(eid); removed.append(eid)
        for fp in range(node.port_count):
            self._transitions.pop((nid, fp), None)
        del self._nodes[nid]
        return removed

    def remove_edge(self, eid):
        edge = self._edges[eid]
        for nid in (edge.node_a, edge.node_b):
            if nid in self._nodes:
                node = self._nodes[nid]
                for i, e in enumerate(node.edges):
                    if e == eid: node.edges[i] = None; break
                self._rebuild_node(nid)
        del self._edges[eid]

    def get_transitions(self, state):
        return self._transitions.get(state, [])

    def get_node(self, nid): return self._nodes[nid]
    def get_edge(self, eid): return self._edges[eid]
    def get_port(self, eid, nid): return self._nodes[nid].edges.index(eid)

    def __repr__(self):
        return f'Topology(nodes={self.node_count}, edges={self.edge_count})'

    def dump(self):
        lines = [repr(self), '', 'Nodes:']
        for nid, node in self._nodes.items():
            lines.append(f'  {node}')
            for p, eid in enumerate(node.edges):
                if eid is None: lines.append(f'    p{p}: ---')
                else:
                    nb = self._edges[eid].other_node(nid)
                    lines.append(f'    p{p}: →e{eid}→n{nb}')
        lines.append('Edges:')
        for eid, edge in self._edges.items():
            lines.append(f'  {edge}')
        return '\n'.join(lines)
