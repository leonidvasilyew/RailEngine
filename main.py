"""Главный исполняемый файл"""

from topology import Topology, Port
from pathfinder import Train, solve_cbs
from visualize import visualize


def main():
    topo = Topology()

    # Ноды
    n0 = topo.add_node(1, name="n0")
    n1 = topo.add_node(1, name="n1")
    n2 = topo.add_node(1, name="n2")
    n3 = topo.add_node(1, name="n3")
    n4 = topo.add_node(1, name="n4")

    # Стрелки
    s0 = topo.add_switch(name="s0", max_speed_straight=10, max_speed_branch=6, switch_time=2.0)
    s1 = topo.add_switch(name="s1", max_speed_straight=10, max_speed_branch=6, switch_time=2.0)
    s2 = topo.add_switch(name="s2", max_speed_straight=10, max_speed_branch=6, switch_time=2.0)
    s3 = topo.add_switch(name="s3", max_speed_straight=8, max_speed_branch=5, switch_time=2.0)
    s4 = topo.add_switch(name="s4", max_speed_straight=8, max_speed_branch=5, switch_time=2.0)
    s5 = topo.add_switch(name="s5", max_speed_straight=8, max_speed_branch=4, switch_time=2.0)
    s6 = topo.add_switch(name="s6", max_speed_straight=10, max_speed_branch=5, switch_time=2.0)

    # Рёбра
    e0 = topo.add_edge(n0, s0, 100, 10, port_b=Port.POINT, name="e0 (s0:POINT)")
    e1 = topo.add_edge(s0, s2, 150, 10, port_a=Port.STRAIGHT, port_b=Port.POINT, name="e1 (s0:STRAIGHT, s2:POINT)")
    e2 = topo.add_edge(s6, s1, 90, 8, port_a=Port.POINT, port_b=Port.STRAIGHT, name="e2 (s6:POINT, s1:STRAIGHT)")
    e3 = topo.add_edge(s1, n1, 100, 10, port_a=Port.POINT, name="e3 (s1:POINT)")
    e4 = topo.add_edge(s2, n2, 120, 8, port_a=Port.BRANCH, name="e4 (s2:BRANCH)")

    e5 = topo.add_edge(s0, s5, 80, 9, port_a=Port.BRANCH, port_b=Port.POINT, name="e5 (s0:BRANCH, s5:POINT)")
    e6 = topo.add_edge(s3, s4, 200, 8, port_a=Port.STRAIGHT, port_b=Port.STRAIGHT, name="e6 (s3:STRAIGHT, s4:STRAIGHT)")
    e7 = topo.add_edge(s4, s1, 130, 6, port_a=Port.POINT, port_b=Port.BRANCH, name="e7 (s4:POINT, s1:BRANCH)")
    e8 = topo.add_edge(s3, n3, 80, 5, port_a=Port.BRANCH, name="e8 (s3:BRANCH)")
    e9 = topo.add_edge(s4, n4, 80, 5, port_a=Port.BRANCH, name="e9 (s4:BRANCH)")

    e10 = topo.add_edge(s5, s6, 200, 12, port_a=Port.BRANCH, port_b=Port.BRANCH, name="e10 (s5:BRANCH, s6:BRANCH)")
    e11 = topo.add_edge(s6, s2, 80, 9, port_a=Port.STRAIGHT, port_b=Port.STRAIGHT, name="e11 (s6:STRAIGHT, s2:STRAIGHT)")
    e12 = topo.add_edge(s5, s3, 80, 9, port_a=Port.STRAIGHT, port_b=Port.POINT, name="e12 (s5:STRAIGHT, s3:POINT)")

    print(topo.dump(), '\n')

    train1 = Train(max_speed=100, accel=2, decel=2, length=30)
    train2 = Train(max_speed=100, accel=2, decel=2, length=20)
    train3 = Train(max_speed=100, accel=2, decel=2, length=25)
    train4 = Train(max_speed=100, accel=2, decel=2, length=40)

    tasks = [
        {'train': train1,
         'start_edge': e0, 'start_offset': 40,
         'goal_edge': e3, 'goal_offset': 90},
        {'train': train2,
         'start_edge': e4, 'start_offset': 90,
         'goal_edge': e8, 'goal_offset': 70},
        {'train': train3,
         'start_edge': e3, 'start_offset': 65,
         'goal_edge': e9, 'goal_offset': 70},
        {'train': train4,
         'start_edge': e8, 'start_offset': 30,
         'goal_edge': e4, 'goal_offset': 100},
    ]

    print("Решаем CBS для 4-х поездов...\n")
    result = solve_cbs(topo, tasks, max_iterations=500)

    print(f"Результат: {'УСПЕХ' if result.success else 'ОТКАЗ'}")
    print(f"Итераций: {result.iterations}\n")
    print("ЛОГ CBS:")
    for line in result.log:
        print(f"  {line}")

    if result.success:
        print("\nРАСПИСАНИЯ:")
        for i, r in enumerate(result.results):
            print(f"\nПоезд {i+1}: time={r.total_time:.2f}")
            for s in r.schedule:
                print(s)

    trains_list = [t['train'] for t in tasks]

    base_colors = ['#e74c3c', '#3498db', '#2ecc71', '#f1c40f', '#9b59b6']
    colors = [base_colors[i % len(base_colors)] for i in range(len(trains_list))]

    viz = [(r, trains_list[i], colors[i]) for i, r in enumerate(result.results)
           if r and r.found]

    if viz:
        print("\nОткрытие визуализации...")
        visualize(topo, viz)


if __name__ == '__main__':
    main()
