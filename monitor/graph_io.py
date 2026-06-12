"""Мост между графом редактора (JSON) и topology.Topology.

Формат графа редактора (canonical):
{
  "version": 1,
  "nodes": [
    {"id": 1, "name": "n1", "x": 0.0, "y": 0.0, "kind": "node"},
    {"id": 3, "name": "s0", "x": .., "y": .., "kind": "switch",
     "ports": {"point": <edgeId>, "straight": <edgeId>, "branch": <edgeId>}}
  ],
  "edges": [
    {"id": 10, "a": 1, "b": 3, "length": 100, "speed": 10, "name": "e10"}
  ]
}

x/y — координаты точки (опциональны; нужны, чтобы при загрузке не сбилось
положение). kind="switch" появляется, когда у точки 3 ребра; ports задаёт, какое
ребро POINT/STRAIGHT/BRANCH.

Существующие файлы не изменяются: topology.py импортируется.
"""

import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from topology import Topology, Port      # noqa: E402

DEFAULT_SPEED = 10.0
DEFAULT_LENGTH = 100.0


def graph_to_topology(graph):
    """Строит (Topology, coords, edge_map) из графа редактора.

    coords:   {topo_node_id: [x, y]} — для точек, у которых заданы координаты.
    edge_map: {editor_edge_id: topo_edge_id} — для привязки датчиков к рёбрам.
    Бросает ValueError при некорректной стрелке (не 3 ребра / роли не заданы).
    """
    nodes = graph.get('nodes', [])
    edges = graph.get('edges', [])
    node_by_id = {n['id']: n for n in nodes}

    # Степень каждой точки + список инцидентных рёбер.
    incident = {n['id']: [] for n in nodes}
    for e in edges:
        for end in ('a', 'b'):
            nid = e[end]
            if nid not in incident:
                raise ValueError(f"Ребро {e.get('id')} ссылается на несуществующую точку {nid}")
            incident[nid].append(e['id'])

    topo = Topology()
    id_map = {}      # editor id -> topo id
    coords = {}

    for n in nodes:
        eid = n['id']
        deg = len(incident[eid])
        name = n.get('name') or str(eid)
        kind = n.get('kind', 'node')
        if kind == 'switch':
            if deg != 3:
                raise ValueError(f"Стрелка '{name}' должна иметь ровно 3 ребра (сейчас {deg})")
            _validate_switch_ports(n, incident[eid], name)
            tid = topo.add_switch(name=name)
        else:
            tid = topo.add_node(port_count=deg, name=name)
        id_map[eid] = tid
        if n.get('x') is not None and n.get('y') is not None:
            coords[tid] = [float(n['x']), float(n['y'])]

    edge_map = {}
    for e in edges:
        na, nb = node_by_id[e['a']], node_by_id[e['b']]
        pa = _role_port(na, e['id']) if na.get('kind') == 'switch' else None
        pb = _role_port(nb, e['id']) if nb.get('kind') == 'switch' else None
        length = float(e.get('length', DEFAULT_LENGTH))
        speed = float(e.get('speed', DEFAULT_SPEED))
        name = e.get('name') or str(e['id'])
        teid = topo.add_edge(id_map[e['a']], id_map[e['b']], length, speed,
                             port_a=pa, port_b=pb, name=name)
        edge_map[e['id']] = teid

    return topo, coords, edge_map


def _validate_switch_ports(node, incident_edges, name):
    ports = node.get('ports') or {}
    assigned = [ports.get('point'), ports.get('straight'), ports.get('branch')]
    if any(p is None for p in assigned):
        raise ValueError(f"Стрелка '{name}': не назначены роли POINT/STRAIGHT/BRANCH")
    if set(assigned) != set(incident_edges):
        raise ValueError(f"Стрелка '{name}': роли портов не совпадают с её рёбрами")


def _role_port(node, edge_id):
    ports = node.get('ports') or {}
    if ports.get('point') == edge_id:
        return Port.POINT
    if ports.get('straight') == edge_id:
        return Port.STRAIGHT
    if ports.get('branch') == edge_id:
        return Port.BRANCH
    return None


def topology_to_graph(topo, coords=None, sensors=None):
    """Сериализует Topology (+ координаты + датчики) в граф редактора.
    sensors: список Sensor (из LayoutState) либо None."""
    coords = coords or {}
    nodes = []
    for nid, n in topo._nodes.items():
        item = {'id': nid, 'name': n.name, 'kind': 'node'}
        if type(n).__name__ == 'Switch':
            item['kind'] = 'switch'
            item['ports'] = {
                'point': _port_edge(n, Port.POINT),
                'straight': _port_edge(n, Port.STRAIGHT),
                'branch': _port_edge(n, Port.BRANCH),
            }
        if nid in coords and coords[nid] is not None:
            item['x'], item['y'] = float(coords[nid][0]), float(coords[nid][1])
        nodes.append(item)
    edges = []
    for eid, e in topo._edges.items():
        edges.append({'id': eid, 'a': e.node_a, 'b': e.node_b,
                      'length': e.length, 'speed': e.speed, 'name': e.name})
    out = {'version': 1, 'nodes': nodes, 'edges': edges}
    if sensors is not None:
        out['sensors'] = [{'id': s.id, 'edge': s.edge_id, 'position': s.position,
                           'name': s.name, 'radius': s.radius} for s in sensors]
    return out


def _port_edge(node, port):
    eid = node.edges[port] if port < len(node.edges) else None
    return eid if eid is not None else None


# ── Самопроверка ─────────────────────────────────────────────────────────
if __name__ == '__main__':
    # Граф, эквивалентный build_demo: A→SW(point), SW(straight)→C1, SW(branch)→C2
    graph = {
        'version': 1,
        'nodes': [
            {'id': 1, 'name': 'A', 'x': 0, 'y': 0, 'kind': 'node'},
            {'id': 2, 'name': 'C1', 'x': 300, 'y': -50, 'kind': 'node'},
            {'id': 3, 'name': 'C2', 'x': 300, 'y': 50, 'kind': 'node'},
            {'id': 4, 'name': 'SW', 'x': 150, 'y': 0, 'kind': 'switch',
             'ports': {'point': 10, 'straight': 11, 'branch': 12}},
        ],
        'edges': [
            {'id': 10, 'a': 1, 'b': 4, 'length': 100, 'name': 'e_in'},
            {'id': 11, 'a': 4, 'b': 2, 'length': 120, 'name': 'e_straight'},
            {'id': 12, 'a': 4, 'b': 3, 'length': 80, 'name': 'e_branch'},
        ],
    }
    topo, coords, edge_map = graph_to_topology(graph)
    print(topo.dump())
    print('coords:', coords, '| edge_map:', edge_map)

    # Проверим маршрутизацию через стрелку (POINT→BRANCH).
    sw = [nid for nid, n in topo._nodes.items() if type(n).__name__ == 'Switch'][0]
    print('\nswitch topo id:', sw, '| ports:',
          [(p, topo._edges[e].name if e is not None else None)
           for p, e in enumerate(topo._nodes[sw].edges)])

    # Round-trip обратно в граф.
    back = topology_to_graph(topo, coords)
    print('\nround-trip nodes:', len(back['nodes']), 'edges:', len(back['edges']))
    sw_back = [n for n in back['nodes'] if n['kind'] == 'switch'][0]
    print('switch ports round-trip:', sw_back['ports'])
    assert sw_back['ports']['point'] is not None
    print('OK')
