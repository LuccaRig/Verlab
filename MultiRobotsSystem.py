import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

WORLDX, WORLDY = 10, 6

def att_force(q, goal, katt=.01):
    return katt*(goal - q)

def rep_force(q, obs, R=3, krep=.1):
    v = q - obs[0:2]
    d = np.linalg.norm(v, axis=1) - obs[2]
    d = d.reshape((len(v), 1))

    rep = (1/d**2)*((1/d)-(1/R))*(v/d)

    invalid = np.squeeze(d > R)
    rep[invalid, :] = 0

    return krep*rep

def rob_position(posx, posy):
    rob = np.array([posx, posy, .1])
    #ax.add_patch(patches.Circle((rob[0], rob[1]), rob[2], color='y'))
    return rob

def rob_movement(posx, posy):
    robp = np.dstack([posx, posy]).reshape(-1, 2)
    return robp

def calc_forcas_x(robpos, robpos2):
    Fatt = att_force(robpos, goal)
    Fatt_x = Fatt[:,0]

    Frep1 = rep_force(robpos, obs1)
    Frep1_x = np.copy(Frep1[:,0])

    Frep2 = rep_force(robpos, obs2)
    Frep2_x = np.copy(Frep2[:,0])

    Frepr = rep_force(robpos, robpos2)/10000
    Frepr_x = np.copy(Frepr[:,0])

    Ft = Fatt + Frep1 + Frep2 + Frepr

    Ft = Ft.astype(float)
    Ft_x = Ft[:,0]

    fmax = .15
    Fm = np.linalg.norm(Ft, axis=1)
    Ft_x[Fm > fmax] = 0

    return Ft_x[0]

def calc_forcas_y(robpos, robpos2):
    Fatt = att_force(robpos, goal)
    Fatt_y = Fatt[:,1]

    Frep1 = rep_force(robpos, obs1)
    Frep1_y = np.copy(Frep1[:,1])

    Frep2 = rep_force(robpos, obs2)
    Frep2_y = np.copy(Frep2[:,1])

    Frepr = rep_force(robpos, robpos2)/10000
    Frepr_y = np.copy(Frepr[:,1])

    Ft = Fatt + Frep1 + Frep2 + Frepr

    Ft = Ft.astype(float)
    Ft_y = Ft[:,1]

    fmax = .15
    Fm = np.linalg.norm(Ft, axis=1)
    Ft_y[Fm > fmax] = 0

    return Ft_y[0]    

fig = plt.figure(figsize=(8,5), dpi=100)
ax = fig.add_subplot(111, aspect='equal')

XX, YY = np.meshgrid(np.arange(0, WORLDX+.4, .4), np.arange(0, WORLDY+.4, .4))
XY = np.dstack([XX, YY]).reshape(-1, 2)

goal = np.array([8, 2])
obs1 = np.array([3, 4, .5])
obs2 = np.array([7, 5, .5])

positions = []
last_patch = None  # Variável para armazenar a última bolinha plotada
positions2 = []
last_patch2 = None

def update(frame):
    global posix, posiy, last_patch
    global posix2, posiy2, last_patch2

    rob = rob_position(posix, posiy)
    robp = rob_movement(posix, posiy)
    rob2 = rob_position(posix2, posiy2)
    robp2 = rob_movement(posix2, posiy2)

    F1x = calc_forcas_x(robp, rob2)
    F1y = calc_forcas_y(robp, rob2)
    F2x = calc_forcas_x(robp2, rob)
    F2y = calc_forcas_y(robp2, rob)

    posix += F1x
    posiy += F1y
    posix2 += F2x
    posiy2 += F2y

    if last_patch is not None:
        last_patch.remove()  # Remove a bolinha anteriormente plotada

    if last_patch2 is not None:
        last_patch2.remove()

    #ax.quiver(rob[0], rob[1], Ft_x, Ft_y, color='b')
    last_patch = patches.Circle((rob[0], rob[1]), rob[2], color='y')
    last_patch2 = patches.Circle((rob2[0], rob2[1]), rob2[2], color='r')
    ax.add_patch(last_patch)  # Adiciona a nova bolinha à tela
    ax.add_patch(last_patch2)
    positions.append((rob[0], rob[1]))  # Adiciona a posição à lista de posições
    positions2.append((rob2[0], rob2[1]))

    return ax,

# Define a posição inicial
posix = 1
posiy = 6
posix2 = 2
posiy2 = 2

# Cria a animação
ani = FuncAnimation(fig, update, frames=200, interval=100, blit=True)

plt.plot(goal[0], goal[1], 'og', markersize=10)
ax.add_patch(patches.Circle((obs1[0], obs1[1]), obs1[2], color='k'))
ax.add_patch(patches.Circle((obs2[0], obs2[1]), obs2[2], color='k'))

ax.set_xlim(0, WORLDX)
ax.set_ylim(0, WORLDY)

plt.show()