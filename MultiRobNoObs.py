import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

WORLDX, WORLDY = 10, 6

def att_force(q, goal, katt=.1):
    return katt*(goal - q)

def rep_force(q1, q2, color1, color2, alpha=.0001):
    if color1 == color2: d = 60
    elif color1  != color2: d = 120

    #else if robot2 color = b and robot2 color = b: d = 8
    #else if robot1 color = y and robot2 color = y: d = 3
    v = q1 - q2[0:2]
    q = np.linalg.norm(v, axis=1) - q2[2]
    q = q.reshape((len(v), 1))

    rep = -(q - d + (1/q) - (d/q**2))*(v/q)

    invalid = np.squeeze(q > d)
    rep[invalid, :] = 0

    return alpha*rep

def rob_position(posx, posy):
    rob = np.array([posx, posy, .1])
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

def calc_forcas_x(selected_robot, robot_positions_list, robot_matrix, robot_selected_color, robot_colors_list):
    Fatt = att_force(selected_robot, goal)
    Ft = Fatt 

    for i in range(len(robot_positions_list)):
        Frepr = rep_force(selected_robot, robot_matrix[0][i], robot_selected_color, robot_colors_list[i])
        Ft += Frepr 

    Ft = Ft.astype(float)
    Ft_x = Ft[:,0]

    return Ft_x[0]

def calc_forcas_y(selected_robot, robot_positions_list, robot_matrix, robot_selected_color, robot_colors_list):
    Fatt = att_force(selected_robot, goal)
    Ft = Fatt

    for i in range(len(robot_positions_list)):
        Frepr = rep_force(selected_robot, robot_matrix[0][i], robot_selected_color, robot_colors_list[i])
        Ft += Frepr

    Ft = Ft.astype(float)
    Ft_y = Ft[:,1]

    return Ft_y[0]    

# Atualiza as posições dos robos e as imprime
def update_anim(robot_matrix):
    global robot_colors_list, last_patches
    for x in range(len(robot_matrix[0])):
        if last_patches[x] is not None:
            last_patches[x].remove()
        last_patches[x] = patches.Circle((robot_matrix[0][x][0], robot_matrix[0][x][1]), robot_matrix[0][x][2], color=robot_colors_list[x])
        ax.add_patch(last_patches[x])


def update(frame):
    global robot_positions_list, robot_colors_list

    robot_matrix = create_robot_matrix(robot_positions_list)

    for i in range(len(robot_positions_list)):
        Fx = calc_forcas_x(robot_matrix[1][i], robot_positions_list, robot_matrix, robot_colors_list[i], robot_colors_list)
        Fy = calc_forcas_y(robot_matrix[1][i], robot_positions_list, robot_matrix, robot_colors_list[i], robot_colors_list)
        for j in range(len(robot_positions_list[i])):
            if j%2 == 0:
                robot_positions_list[i][j] += Fx
            else:
                robot_positions_list[i][j] += Fy

    update_anim(robot_matrix)

    return last_patches

def main():
    # define as posicoes do goal e dos obstaculos
    global goal, ax, robot_positions_list, robot_colors_list, last_patches
    
    goal = np.array([5, 3])

    # Define a posição inicial dos robos
    robot_positions_list = [[2, 2], [2.5, 2], [3, 2], [1, 2], [1.5, 1.5],
                            [0.5, 0.5], [1, 6], [7, 2], [8, 3], [9, 5], [7, 5], [6, 1]]
    last_patches = []
    for x in robot_positions_list:
        last_patches.append(None)

    # Define a cor dos robos
    robot_colors_list = ['b', 'r', 'b', 'b', 'r', 'y', 'y', 'y', 'r', 'r', 'b', 'y']

    fig = plt.figure(figsize=(8,5), dpi=100)
    ax = fig.add_subplot(111, aspect='equal')

    # Cria a animação
    ani = FuncAnimation(fig, update, frames=250, interval=30, blit=True)

    plt.plot(goal[0], goal[1], 'og', markersize=10)

    ax.set_xlim(0, WORLDX)
    ax.set_ylim(0, WORLDY)

    plt.show()

main()