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

# Cria os robos, suas posições e seus last_patches
def quantity_robs(posrobs):
    robs = []
    robspos = []
    for x in range(len(posrobs)):
        newrob = rob_position(posrobs[x][0], posrobs[x][1])
        newrobp = rob_movement(posrobs[x][0], posrobs[x][1])
        robs.append(newrob)
        robspos.append(newrobp)
    robt = []
    robt.append(robs)
    robt.append(robspos)
    return robt

def calc_forcas_x(robpos, robpos2, robpos3):
    Fatt = att_force(robpos, goal)

    Frep1 = rep_force(robpos, obs1)

    Frep2 = rep_force(robpos, obs2)

    Frepr = rep_force(robpos, robpos2)/15000
    Frepr2 = rep_force(robpos, robpos3)/15000

    Ft = Fatt + Frep1 + Frep2 + Frepr + Frepr2

    Ft = Ft.astype(float)
    Ft_x = Ft[:,0]

    fmax = .15
    Fm = np.linalg.norm(Ft, axis=1)
    Ft_x[Fm > fmax] = 0

    return Ft_x[0]

def calc_forcas_y(robpos, robpos2, robpos3):
    Fatt = att_force(robpos, goal)

    Frep1 = rep_force(robpos, obs1)

    Frep2 = rep_force(robpos, obs2)

    Frepr = rep_force(robpos, robpos2)/15000
    Frepr2 = rep_force(robpos, robpos3)/15000

    Ft = Fatt + Frep1 + Frep2 + Frepr + Frepr2

    Ft = Ft.astype(float)
    Ft_y = Ft[:,1]

    fmax = .15
    Fm = np.linalg.norm(Ft, axis=1)
    Ft_y[Fm > fmax] = 0

    return Ft_y[0]    

# Atualiza as posições dos robos e as imprime
def update_anim(robs):
    global colors, last_patches
    for x in range(len(robs[0])):
        if last_patches[x] is not None:
            last_patches[x].remove()
        last_patches[x] = patches.Circle((robs[0][x][0], robs[0][x][1]), robs[0][x][2], color=colors[x])
        ax.add_patch(last_patches[x])

fig = plt.figure(figsize=(8,5), dpi=100)
ax = fig.add_subplot(111, aspect='equal')

XX, YY = np.meshgrid(np.arange(0, WORLDX+.4, .4), np.arange(0, WORLDY+.4, .4))
XY = np.dstack([XX, YY]).reshape(-1, 2)

goal = np.array([8, 2])
obs1 = np.array([3, 4, .5])
obs2 = np.array([7, 5, .5])

def update(frame):
    global pos, robs

    robs = quantity_robs(pos)

    F1x = calc_forcas_x(robs[1][0], robs[0][1], robs[0][2])
    F1y = calc_forcas_y(robs[1][0], robs[0][1], robs[0][2])
    F2x = calc_forcas_x(robs[1][1], robs[0][0], robs[0][2])
    F2y = calc_forcas_y(robs[1][1], robs[0][0], robs[0][2])
    F3x = calc_forcas_x(robs[1][2], robs[0][0], robs[0][1])
    F3y = calc_forcas_y(robs[1][2], robs[0][0], robs[0][1])

    pos[0][0] += F1x
    pos[0][1] += F1y
    pos[1][0] += F2x
    pos[1][1] += F2y
    pos[2][0] += F3x
    pos[2][1] += F3y

    update_anim(robs)

    return ax

# Define a posição inicial dos robos
pos = [[2, 2], [2.5, 2], [3, 2], [1, 1], [1.5, 1.5], [0.5, 0.5]]
last_patches = []
for x in range(len(pos)):
    last_patches.append(None)

# Define a cor dos robos
colors = ['y', 'r', 'm', 'k', 'k', 'k']

# Cria a animação
ani = FuncAnimation(fig, update, frames=250, interval=30, blit=True)

plt.plot(goal[0], goal[1], 'og', markersize=10)
ax.add_patch(patches.Circle((obs1[0], obs1[1]), obs1[2], color='k'))
ax.add_patch(patches.Circle((obs2[0], obs2[1]), obs2[2], color='k'))

ax.set_xlim(0, WORLDX)
ax.set_ylim(0, WORLDY)

plt.show()