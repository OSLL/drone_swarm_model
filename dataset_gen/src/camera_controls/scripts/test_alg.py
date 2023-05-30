import numpy as np

x = (-15, 15, 5)
y = (-15, 15, 5)
z = (1, 6, 3)

m_x = np.linspace(*x).tolist()
m_y = np.linspace(*y).tolist()
m_z = np.linspace(*z).tolist()

list()
d = ([], [], [], [])

y_for_1 = m_y[:]
y_for_2 = m_y[:]
x_for_3 = m_x[:]
x_for_4 = m_x[:]

for _z in m_z:
    # 1: x0 + y
    x0 = x[0]
    for _y in y_for_1:
        d[0].append((x0, _y, _z, 0, 0, 0))
    y_for_1.reverse()

    # 2: x1 + y
    x1 = x[1]
    for _y in y_for_2:
        d[1].append((x1, _y, _z, 0, 0, -np.pi))
    y_for_2.reverse()

    # 3: x + y0
    y0 = y[0]
    for _x in x_for_3:
        d[2].append((_x, y0, _z, 0, 0, np.pi / 2))
    x_for_3.reverse()

    # 4: x + y1
    y1 = y[1]
    for _x in x_for_4:
        d[3].append((_x, y1, _z, 0, 0, -np.pi / 2))
    x_for_4.reverse()

for drones in zip(*d):
    for i, dr in enumerate(drones):
        print(f"drone{i+1}", *dr)
