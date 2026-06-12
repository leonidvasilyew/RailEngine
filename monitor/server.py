"""WebSocket-сервер мониторинга (FastAPI + uvicorn).

Держит единый LayoutState, крутит реальный тик-цикл в фоне (asyncio), рассылает
состояние всем клиентам и принимает команды. Всё в одном event loop — гонок нет,
блокировки не нужны.

Контракт сообщений:
  сервер→клиент:  {type:"init",  topology, state}
                  {type:"state", state, events}
  клиент→сервер:  {cmd:"set_speed",   train, v_target, accel?}  v_target<0 → разворот
                  {cmd:"set_switch",   node, position}        position: 1=STRAIGHT,2=BRANCH
                  {cmd:"toggle_switch",node}
                  {cmd:"place_train",  train?, edge, position, direction?, length?,
                                       max_speed?, accel?, decel?, color?, name?}
                  {cmd:"remove_train", train}
                  {cmd:"reverse",      train}                 (явный разворот на месте)
                  {cmd:"fire_sensor",  sensor, train?}        (отладка/мост)

Запуск демо:  python -m monitor.server
"""

import asyncio
import json
import os
from contextlib import asynccontextmanager

# layout_state при импорте добавляет корень проекта в sys.path — делаем это первым,
# чтобы затем сработали `from topology import ...` и `from visualize import ...`.
from monitor.layout_state import LayoutState

from topology import Topology, Port          # noqa: E402
from pathfinder import Train                  # noqa: E402
from visualize import _ser_topo               # noqa: E402  (переиспользуем сериализатор)

from fastapi import FastAPI, WebSocket, WebSocketDisconnect   # noqa: E402
from fastapi.responses import HTMLResponse, JSONResponse      # noqa: E402

_WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')
_HTML_FILE = os.path.join(_WEB_DIR, 'monitor.html')
_EDITOR_FILE = os.path.join(_WEB_DIR, 'editor.html')


class MonitorServer:
    """Состояние сервера: LayoutState + подключённые клиенты + тик-цикл."""

    def __init__(self, state, topo, frame_hz=30, coords=None):
        self.state = state
        self.topo = topo
        self.topo_json = _ser_topo(topo)
        self.coords = coords            # {node_id: [x,y]} или None (тогда auto-layout)
        self.clients = set()
        self.frame_dt = 1.0 / frame_hz
        self._next_train_id = (max(state.trains, default=0) + 1)
        # Сюда будущий hardware_bridge подпишется, чтобы транслировать команды
        # на ESP32 (set_speed/set_switch). None — чистая симуляция.
        self.on_command = None

    def init_message(self):
        return {'type': 'init', 'topology': self.topo_json,
                'coords': self.coords, 'state': self.state.to_dict()}

    def apply_graph(self, graph):
        """Заменяет живой граф на отредактированный (кнопка «Применить»).
        Пересобирает Topology + LayoutState; датчики переносятся из графа.
        Поезда стараемся сохранить: если ребро под ГОЛОВОЙ поезда уцелело
        (id ребра редактора совпадает со старым), переносим поезд на новое ребро
        с той же позицией головы, скоростью и направлением."""
        from monitor.graph_io import graph_to_topology
        old_trains = list(self.state.trains.values())
        topo, coords, edge_map = graph_to_topology(graph)
        state = LayoutState(topo)
        for s in graph.get('sensors', []):
            teid = edge_map.get(s['edge'])
            if teid is None:
                continue
            state.add_sensor(s['id'], teid, float(s['position']),
                             name=s.get('name', ''),
                             radius=float(s.get('radius', 20.0)))
        # Перенос поездов по голове (edge_map ключуется id рёбер редактора,
        # которые для уцелевших рёбер совпадают со старыми id топологии).
        for lt in old_trains:
            new_eid = edge_map.get(lt.edge_id)
            if new_eid is None:
                continue            # ребро под головой удалено — поезд не переносим
            nt = state.place_train(lt.id, lt.train, new_eid, lt.s,
                                   direction=lt.direction, color=lt.color, name=lt.name)
            nt.face = lt.face
            nt.v = lt.v
            nt.v_target = lt.v_target
            nt.accel = lt.accel
            nt.decel = lt.decel
        self.state = state
        self.topo = topo
        self.topo_json = _ser_topo(topo)
        self.coords = coords or None
        self._next_train_id = max(state.trains, default=0) + 1

    # ── Тик-цикл ──────────────────────────────────────────────────────────
    async def run_loop(self):
        loop = asyncio.get_event_loop()
        last = loop.time()
        while True:
            await asyncio.sleep(self.frame_dt)
            now = loop.time()
            dt = now - last
            last = now
            if dt <= 0:
                continue
            events = self.state.tick(dt)
            await self.broadcast({'type': 'state',
                                  'state': self.state.to_dict(),
                                  'events': events})

    async def broadcast(self, message):
        if not self.clients:
            return
        text = json.dumps(message)
        dead = []
        # Снимок: во время await ниже другой клиент может подключиться/отключиться
        # (clients.add/discard). Без копии это «Set changed size during iteration»,
        # что убивало бы тик-цикл навсегда — и state перестал бы рассылаться.
        for ws in list(self.clients):
            try:
                await ws.send_text(text)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    # ── Команды ───────────────────────────────────────────────────────────
    def handle_command(self, msg):
        cmd = msg.get('cmd')
        st = self.state
        if cmd == 'set_speed':
            st.set_speed(msg['train'], float(msg['v_target']),
                         accel=msg.get('accel'), decel=msg.get('decel'))
        elif cmd == 'set_switch':
            st.set_switch(int(msg['node']), Port(int(msg['position'])))
        elif cmd == 'toggle_switch':
            st.toggle_switch(int(msg['node']))
        elif cmd == 'place_train':
            self._place_train(msg)
        elif cmd == 'remove_train':
            st.remove_train(msg['train'])
        elif cmd == 'reverse':
            st.reverse(msg['train'])
        elif cmd == 'fire_sensor':
            st.fire_sensor(msg['sensor'], msg.get('train'))
        # Транслируем команду наружу (на макет), если подписан мост.
        if self.on_command is not None:
            try:
                self.on_command(msg)
            except Exception:
                pass

    def _place_train(self, msg):
        tid = msg.get('train')
        if tid is None:
            tid = self._next_train_id
            self._next_train_id += 1
        else:
            self._next_train_id = max(self._next_train_id, int(tid) + 1)
        train = Train(
            max_speed=float(msg.get('max_speed', 12.0)),
            accel=float(msg.get('accel', 2.0)),
            decel=float(msg.get('decel', 2.0)),
            length=float(msg.get('length', 20.0)))
        self.state.place_train(
            tid, train, int(msg['edge']), float(msg['position']),
            direction=int(msg.get('direction', 1)),
            color=msg.get('color', '#e74c3c'),
            name=msg.get('name', f'T{tid}'))


def create_app(state, topo, frame_hz=30, coords=None):
    server = MonitorServer(state, topo, frame_hz, coords=coords)

    @asynccontextmanager
    async def lifespan(app):
        task = asyncio.create_task(server.run_loop())
        try:
            yield
        finally:
            task.cancel()

    app = FastAPI(lifespan=lifespan)
    app.state.monitor = server

    @app.get('/topology')
    async def topology():
        return JSONResponse(server.topo_json)

    @app.get('/graph')
    async def graph():
        # Текущий живой граф в формате редактора (структура + координаты + датчики).
        from monitor.graph_io import topology_to_graph
        return JSONResponse(topology_to_graph(
            server.topo, server.coords, server.state.sensors.values()))

    @app.get('/', response_class=HTMLResponse)
    async def index():
        if os.path.exists(_HTML_FILE):
            with open(_HTML_FILE, 'r', encoding='utf-8') as f:
                return HTMLResponse(f.read())
        return HTMLResponse(_TEST_PAGE)

    @app.get('/editor', response_class=HTMLResponse)
    async def editor():
        if os.path.exists(_EDITOR_FILE):
            with open(_EDITOR_FILE, 'r', encoding='utf-8') as f:
                return HTMLResponse(f.read())
        return HTMLResponse('<h3>editor.html не найден</h3>')

    @app.websocket('/ws')
    async def ws(websocket: WebSocket):
        await websocket.accept()
        # init ДО добавления в clients — иначе фоновый тик мог бы отправить
        # state раньше init (рассинхрон topo/state у клиента).
        await websocket.send_text(json.dumps(server.init_message()))
        server.clients.add(websocket)
        try:
            while True:
                raw = await websocket.receive_text()
                try:
                    msg = json.loads(raw)
                    if msg.get('cmd') == 'apply_graph':
                        server.apply_graph(msg['graph'])
                        # Все клиенты (мониторы) перечитывают новый граф.
                        await server.broadcast(server.init_message())
                    else:
                        server.handle_command(msg)
                except Exception as e:
                    await websocket.send_text(json.dumps(
                        {'type': 'error', 'message': str(e)}))
        except WebSocketDisconnect:
            pass
        finally:
            server.clients.discard(websocket)

    return app


# ── Минимальная тест-страница (до Phase 3) ───────────────────────────────
_TEST_PAGE = r"""<!DOCTYPE html><html><head><meta charset="utf-8">
<title>Monitor test</title>
<style>body{background:#0a0e17;color:#c8d6e5;font:13px system-ui;padding:16px}
button{background:#22304a;color:#c8d6e5;border:1px solid #3a4d72;border-radius:6px;
 padding:4px 10px;margin:2px;cursor:pointer}
input{width:70px;background:#0f1626;color:#c8d6e5;border:1px solid #3a4d72;border-radius:4px;padding:3px}
.row{margin:6px 0;padding:6px;border:1px solid #1e2a44;border-radius:8px}
pre{background:#0f1626;padding:8px;border-radius:8px;max-height:40vh;overflow:auto}</style>
</head><body>
<h3>Monitor — тест-страница (Phase 2)</h3>
<div id="trains"></div>
<div id="switches"></div>
<h4>Состояние</h4><pre id="dump"></pre>
<script>
const ws=new WebSocket(`ws://${location.host}/ws`);
let topo=null;
function send(o){ws.send(JSON.stringify(o))}
ws.onmessage=e=>{
  const m=JSON.parse(e.data);
  if(m.type==='init'){topo=m.topology; render(m.state)}
  if(m.type==='state'){render(m.state)}
  if(m.type==='error'){console.warn('err',m.message)}
};
function render(s){
  document.getElementById('dump').textContent=JSON.stringify(s,null,1);
  // trains
  const td=document.getElementById('trains');
  td.innerHTML='<h4>Поезда</h4>';
  s.trains.forEach(t=>{
    const d=document.createElement('div');d.className='row';
    d.innerHTML=`<b style="color:${t.color}">${t.name}</b> e${t.edge_id} s=${t.s.toFixed(3)} v=${t.v.toFixed(2)} `+
      `<input id="v${t.id}" value="${t.v_target}"> `+
      `<button onclick="send({cmd:'set_speed',train:${t.id},v_target:+document.getElementById('v${t.id}').value})">speed</button> `+
      `<button onclick="send({cmd:'set_speed',train:${t.id},v_target:0})">stop</button> `+
      `<button onclick="send({cmd:'remove_train',train:${t.id}})">remove</button>`;
    td.appendChild(d);
  });
  // switches
  const sd=document.getElementById('switches');
  sd.innerHTML='<h4>Стрелки</h4>';
  Object.entries(s.switches).forEach(([nid,pos])=>{
    const d=document.createElement('div');d.className='row';
    d.innerHTML=`узел ${nid}: ${pos===1?'STRAIGHT':'BRANCH'} `+
      `<button onclick="send({cmd:'toggle_switch',node:${nid}})">toggle</button>`;
    sd.appendChild(d);
  });
}
</script></body></html>"""


# ── Сборка демо-состояния и запуск ───────────────────────────────────────
def build_demo():
    """Демо-макет: разветвление с одной стрелкой и одним датчиком."""
    topo = Topology()
    a = topo.add_node(1, name='A')
    c1 = topo.add_node(1, name='C1')
    c2 = topo.add_node(1, name='C2')
    sw = topo.add_switch(name='SW', max_speed_straight=10, max_speed_branch=6)
    e_in = topo.add_edge(a, sw, 100, 10, port_b=Port.POINT, name='e_in')
    e_st = topo.add_edge(sw, c1, 120, 10, port_a=Port.STRAIGHT, name='e_straight')
    e_br = topo.add_edge(sw, c2, 80, 6, port_a=Port.BRANCH, name='e_branch')

    state = LayoutState(topo)
    state.add_sensor(10, e_in, 0.9, name='S_in', radius=100)
    state.place_train(1, Train(max_speed=12, accel=2, decel=2, length=20),
                      e_in, 10, direction=1, color='#3498db', name='T1')
    return state, topo


def main(host='127.0.0.1', port=8077):
    import uvicorn
    state, topo = build_demo()
    app = create_app(state, topo)
    print(f"Monitor server: http://{host}:{port}")
    uvicorn.run(app, host=host, port=port, log_level='info')


if __name__ == '__main__':
    main()
