import os
import time

from tkinter import *
from tkinter.filedialog import askopenfilename

import sys
import pickle
import pygame

import refPointBot as rpb
import measuringBot as mb
from room import *
import swarmExplorationUWBSLAM as seUWBSLAM
import time


def LoadFile():
    
    window = Tk()
    window.withdraw()
        
    availableFormats = [("Pickle dump", "*.pickle")]
    
    filename = askopenfilename(title="Open File", filetypes=availableFormats)
    filePath = filename[:]

    if filePath:
        file = open(filePath, "rb")
        colors = pickle.load(file)
        file.close()

        # convertir tableau numpy en simulateur
        
        filePathList = filePath.split("/")
        fileName = filePathList[-1]
        pygame.display.set_caption("Simulation - " + fileName)

        return colors, fileName[:-7]
    
    return None, None


def find_walls_corners(table):
    # trouver les murs verticaux
    vertical_walls_corners = []

    for c in range(len(table[0])-1):
        column = table[:,c]
        next_column = table[:,c+1]

        if 0 in column and 0 in next_column:
            potential_walls = table[:,c:c+2]
            
            r = 0
            in_a_wall = False
            for r in range(len(table)):
                
                if potential_walls[r,:][0] == 0 and potential_walls[r,:][1] == 0 and not in_a_wall:
                    in_a_wall = True
                    new_wall = [[r,c],[r,c+1]]
                    length_wall = 1
                elif potential_walls[r,:][0] == 0 and potential_walls[r,:][1] == 0 and in_a_wall:
                    length_wall += 1
                elif (potential_walls[r,:][0] != 0 or potential_walls[r,:][1] != 0) and in_a_wall:
                    in_a_wall = False
                    new_wall.append([r-1,c])
                    new_wall.append([r-1,c+1])
                    new_wall.append('v')
                    if length_wall > 2:
                        vertical_walls_corners.append(new_wall)

    # trouver les murs horizontaux
    horizontal_walls_corners = []

    for r in range(len(table)-1):
        row = table[r,:]
        next_row = table[r+1,:]

        if 0 in row and 0 in next_row:
            potential_walls = table[r:r+2,:]
            
            c = 0
            in_a_wall = False
            for c in range(len(table[0])):
                if potential_walls[:,c][0] == 0 and potential_walls[:,c][1] == 0 and not in_a_wall:
                    in_a_wall = True
                    new_wall = [[r,c],[r+1,c]]
                    length_wall = 1
                elif potential_walls[:,c][0] == 0 and potential_walls[:,c][1] == 0 and in_a_wall:
                    length_wall += 1
                elif (potential_walls[:,c][0] != 0 or potential_walls[:,c][1] != 0) and in_a_wall:
                    in_a_wall = False
                    new_wall.append([r,c-1])
                    new_wall.append([r+1,c-1])
                    new_wall.append('h')
                    if length_wall > 2:
                        horizontal_walls_corners.append(new_wall)

    walls_corners = vertical_walls_corners + horizontal_walls_corners

    return walls_corners


def drawing_to_simulation(table,surface1,surface2,surface3,surface4,surface5,mode, parameters):

    robots_centers = []
    for row in range(len(table)):
        for col in range(len(table[0])):
            if table[row,col] in [1,2,3]:
                # juste par souci de compr??hension
                y = row
                x = col
                robots_centers.append([[x,y],table[row,col]])

    # point de d??part des trac??s, pour ne pas ??tre trop pr??t du bord de l'??cran
    offset = 10
    
    # facteur multiplicatif pour avoir des distances de l'ordre de la taille de l'??cran (on part de 220x128 et on va vers 1600*900)
    scale = 7

    walls_corners = find_walls_corners(table)
    for i in range(len(walls_corners)):
        for j in range(4):
            walls_corners[i][j][0] = walls_corners[i][j][0]*scale + offset
            walls_corners[i][j][1] = walls_corners[i][j][1]*scale + offset

    for i in range(len(robots_centers)):
        robots_centers[i][0][0] = robots_centers[i][0][0]*scale  + offset
        robots_centers[i][0][1] = robots_centers[i][0][1]*scale  + offset

    # cr??ation de la salle
    room = Room(walls_corners,surface1,surface2)

    measuringBots = []
    refPointBots = []
    
    for bot in robots_centers:
        botType = bot.pop()
        if botType == 1:
            measuringBots.append(mb.MeasuringBot(bot[0][0], bot[0][1], 10, room, objective = None, haveObjective = False, showDetails=True))
        elif botType == 2:
            refPointBots.append(rpb.RefPointBot(bot[0][0], bot[0][1], 6, room, objective = None, haveObjective = False, showDetails = True))

            
    bots = measuringBots + refPointBots

    room.addBots(bots)

    SEUWBSLAM = seUWBSLAM.SwarmExploratorUWBSLAM(surface3, surface4, surface5, room, measuringBots[0], refPointBots, mode, parameters)

    return room, SEUWBSLAM


def redrawGameWindow(room, background, control):
    
    ### Composition de la sc??ne
    # on choisit et on applique la couleur de l'arri??re plan de la simulation
    background.fill((100,100,100))

    # ajout d'une surcouche transparente pour les zones d??j?? explor??es et sombre dans les zones non explor??es
    background.blit(room.surface2, (0,0))

    # ajout des murs et robots au dessus de l'arri??re plan
    room.surface1.fill((0,0,0,0)) # (noir) transparent
    # mise ?? jour des robots
    for bot in room.bots:
        bot.draw()

    # affichage optionel des obtsacles :
    # for obstacle in room.obstacles:
    #     obstacle.draw()

    # mise ?? jour des murs vus
    room.draw_walls()
    background.blit(room.surface1, (0,0))

    # on ajoute ?? l'arri??re plan tous les affichages sp??cifiques ?? la m??thode de contr??le de l'essaim choisie
    control.draw()
    background.blit(control.surfaceUWB, (0,0))
    background.blit(control.surfaceGrid, (0,0))
    background.blit(control.surfaceReferenceBot,(0,0))

    ### mise ?? jour de l'affichage complet
    pygame.display.flip()


def load_and_launch_exact_simulation(parameters):

    table, fileName = LoadFile()

    initStart = time.time()

    if table is not None : # ??vite un crash si on ne s??lectionne pas de fichier

        sw, sh = 1600, 900
        background = pygame.display.set_mode((sw, sh))
        # les murs et les robots
        surface1 = pygame.Surface((sw,sh),  pygame.SRCALPHA)
        # la vision des robots
        surface2 = pygame.Surface((sw,sh),  pygame.SRCALPHA)
        # la surface couverte par les balises UWB
        surface3 = pygame.Surface((sw,sh),  pygame.SRCALPHA)
        # la grille qui sert aux d??placements
        surface4 = pygame.Surface((sw,sh),  pygame.SRCALPHA)
        # une surface suppl??mentaire pour des affichages annexes
        surface5 = pygame.Surface((sw,sh),  pygame.SRCALPHA)

        ################# vision et zone UWB exactes ('exact') ou discr??tis??es ('discrete')
        mode = 'exact'
        #################

        room, SEUWBSLAM = drawing_to_simulation(table,surface1,surface2,surface3,surface4,surface5,mode, parameters)
        initDuration = (time.time()-initStart)
        simulationStart = time.time()

        clock = pygame.time.Clock()
        hz = 144

        run = True 
        ## Choix du type de d??placement
        control = SEUWBSLAM

        while run:
            start_iteration = time.time()
            clock.tick(hz)
            
            # utilisateur ferme la fenetre
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    run = False  

            # fin de la simulation (les robtos ont arr??t?? de bouger)
            if control.end_simulation == True:
                run = False

            # t = time.time()
            control.move()
            # print("duration of control.move() : ", time.time() - t)
            
            ## It??ration sur l'ensemble des robots pour les faire se d??placer
            # t = time.time()
            for bot in room.bots:
                bot.move()
            # print("duration of bot.move() : ", time.time() - t)

            if mode == "exact": # si mode == 'discrete', alors la vision est inclue dans le control.move()
                ## Prise en compte des nouvelles zones vues par les robots
                # t = time.time()
                bots = None
                movingRefPointBots = control.checkMovingRefPointBots()

                if control.status == "movingMeasuringBot":
                    bots = [control.measurerBot]
                if movingRefPointBots[0]:
                    bots = [control.refPointBots[movingRefPointBots[1]]]
                elif control.checkMovingMeasurerBot():
                    bots = [control.measurerBot]
                room.updateExploration(debug = False, bots=bots)
                # print("duration of updateExploration : ", time.time() - t)

            start_draw = time.time()
            redrawGameWindow(room, background, control)      
            end_draw = time.time()

            end_iteration = time.time()

            # print((end_draw-start_draw)/(end_iteration-start_iteration))

        
        # si la simulation s'est achev??e, on affiche les m??triques et on attend que l'utilisateur ferme la fen??tre
        simulationDuration = time.time() - simulationStart
        metrics = control.print_metrics()
        print("Dur??e de l'initialisation : %3.2f s" %initDuration)
        print('Dur??e de la simulation : %3.2f s' %simulationDuration)
        print('Dur??e totale : %3.2f s' %(initDuration + simulationDuration))
        print('\n')
        

        while not run:
            
            # utilisateur ferme la fenetre
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    run = True 

        dirname = os.path.dirname(__file__)
        file = open(os.path.join(dirname, "./results/",str(fileName)+"-exact-results.pickle"), "wb")
        pickle.dump(metrics, file)
        file.close()


def load_and_launch_discrete_simulation(parameters):

    table, fileName = LoadFile()

    initStart = time.time()
    print("parameters load launch : ", parameters)
    if table is not None : # ??vite un crash si on ne s??lectionne pas de fichier

        sw, sh = 1600, 900
        background = pygame.display.set_mode((sw, sh))
        # les murs et les robots
        surface1 = pygame.Surface((sw,sh),  pygame.SRCALPHA)
        # la vision des robots
        surface2 = pygame.Surface((sw,sh),  pygame.SRCALPHA)
        # la surface couverte par les balises UWB
        surface3 = pygame.Surface((sw,sh),  pygame.SRCALPHA)
        # la grille qui sert aux d??placements
        surface4 = pygame.Surface((sw,sh),  pygame.SRCALPHA)
        # une surface suppl??mentaire pour des affichages annexes
        surface5 = pygame.Surface((sw,sh),  pygame.SRCALPHA)

        ################# vision et zone UWB exactes ('exact') ou discr??tis??es ('discrete')
        mode = 'discrete'
        #################

        room, SEUWBSLAM = drawing_to_simulation(table,surface1,surface2,surface3,surface4,surface5,mode, parameters)
        initDuration = (time.time()-initStart)
        simulationStart = time.time()

        clock = pygame.time.Clock()
        hz = 144

        run = True 
        ## Choix du type de d??placement
        control = SEUWBSLAM

        while run:
            start_iteration = time.time()
            clock.tick(hz)
            
            # utilisateur ferme la fenetre
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    run = False  

            # fin de la simulation (les robtos ont arr??t?? de bouger)
            if control.end_simulation == True:
                run = False

            # t = time.time()
            control.move()
            # print("duration of control.move() : ", time.time() - t)
            
            ## It??ration sur l'ensemble des robots pour les faire se d??placer
            # t = time.time()
            for bot in control.room.bots:
                bot.move()
            # print("duration of bot.move() : ", time.time() - t)

            if mode == "exact": # si mode == 'discrete', alors la vision est inclue dans le control.move()
                ## Prise en compte des nouvelles zones vues par les robots
                # t = time.time()
                bots = None
                movingRefPointBots = control.checkMovingRefPointBots()

                if control.status == "movingMeasuringBot":
                    bots = [control.measurerBot]
                if movingRefPointBots[0]:
                    bots = [control.refPointBots[movingRefPointBots[1]]]
                elif control.checkMovingMeasurerBot():
                    bots = [control.measurerBot]
                room.updateExploration(debug = False, bots=bots)
                # print("duration of updateExploration : ", time.time() - t)

            start_draw = time.time()
            redrawGameWindow(room, background, control)      
            end_draw = time.time()

            end_iteration = time.time()

            # print((end_draw-start_draw)/(end_iteration-start_iteration))

        
        # si la simulation s'est achev??e, on affiche les m??triques et on attend que l'utilisateur ferme la fen??tre
        simulationDuration = time.time() - simulationStart
        metrics = control.print_metrics()
        print("Dur??e de l'initialisation : %3.2f s" %initDuration)
        print('Dur??e de la simulation : %3.2f s' %simulationDuration)
        print('Dur??e totale : %3.2f s' %(initDuration + simulationDuration))
        print('\n')
        

        while not run:
            
            # utilisateur ferme la fenetre
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    run = True 
        
        dirname = os.path.dirname(__file__)
        file = open(os.path.join(dirname, "./results/",str(fileName)+"-discrete-results.pickle"), "wb")
        pickle.dump(metrics, file)
        file.close()

