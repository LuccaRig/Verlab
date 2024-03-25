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

# create a robot matrix with 2 vectors (positions_list, movement_list)
def create_robot_matrix(robot_positions_list):
    robots_new_positions_list = []
    robots_new_movement_list = []
    for x in range(len(robot_positions_list)):
        new_robot_position = rob_position(robot_positions_list[x][0], robot_positions_list[x][1])
        new_robot_movement = rob_movement(robot_positions_list[x][0], robot_positions_list[x][1])
        robots_new_positions_list.append(new_robot_position)
        robots_new_movement_list.append(new_robot_movement)
    robots_positions_and_movement_matrix = []
    robots_positions_and_movement_matrix.append(robots_new_positions_list)
    robots_positions_and_movement_matrix.append(robots_new_movement_list)
    return robots_positions_and_movement_matrix

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
def update_anim(robot_matrix):
    global robot_colors_list, last_patches
    for x in range(len(robot_matrix[0])):
        if last_patches[x] is not None:
            last_patches[x].remove()
        last_patches[x] = patches.Circle((robot_matrix[0][x][0], robot_matrix[0][x][1]), robot_matrix[0][x][2], color=robot_colors_list[x])
        ax.add_patch(last_patches[x])

fig = plt.figure(figsize=(8,5), dpi=100)
ax = fig.add_subplot(111, aspect='equal')

XX, YY = np.meshgrid(np.arange(0, WORLDX+.4, .4), np.arange(0, WORLDY+.4, .4))
XY = np.dstack([XX, YY]).reshape(-1, 2)


def update(frame):
    global robot_positions_list

    robot_matrix = create_robot_matrix(robot_positions_list)

    F1x = calc_forcas_x(robot_matrix[1][0], robot_matrix[0][1], robot_matrix[0][2])
    F1y = calc_forcas_y(robot_matrix[1][0], robot_matrix[0][1], robot_matrix[0][2])
    F2x = calc_forcas_x(robot_matrix[1][1], robot_matrix[0][0], robot_matrix[0][2])
    F2y = calc_forcas_y(robot_matrix[1][1], robot_matrix[0][0], robot_matrix[0][2])
    F3x = calc_forcas_x(robot_matrix[1][2], robot_matrix[0][0], robot_matrix[0][1])
    F3y = calc_forcas_y(robot_matrix[1][2], robot_matrix[0][0], robot_matrix[0][1])

    robot_positions_list[0][0] += F1x
    robot_positions_list[0][1] += F1y
    robot_positions_list[1][0] += F2x
    robot_positions_list[1][1] += F2y
    robot_positions_list[2][0] += F3x
    robot_positions_list[2][1] += F3y

    update_anim(robot_matrix)

    return last_patches


# define as posicoes do goal e dos obstaculos
goal = np.array([8, 2])
obs1 = np.array([3, 4, .5])
obs2 = np.array([5, 2, .5])

# Define a posição inicial dos robos
robot_positions_list = [[2, 2], [2.5, 2], [3, 2], [1, 1], [1.5, 1.5], [0.5, 0.5]]
last_patches = []
for x in range(len(robot_positions_list)):
    last_patches.append(None)

# Define a cor dos robos
robot_colors_list = ['y', 'r', 'm', 'k', 'k', 'k']

# Cria a animação
ani = FuncAnimation(fig, update, frames=250, interval=30, blit=True)

plt.plot(goal[0], goal[1], 'og', markersize=10)
ax.add_patch(patches.Circle((obs1[0], obs1[1]), obs1[2], color='k'))
ax.add_patch(patches.Circle((obs2[0], obs2[1]), obs2[2], color='k'))

ax.set_xlim(0, WORLDX)
ax.set_ylim(0, WORLDY)

print(len(robot_positions_list))
plt.show()