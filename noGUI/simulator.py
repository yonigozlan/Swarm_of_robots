import random as rd
from click.types import DateTime
import numpy as np
import time
import copy
import os
import sys
import pickle
from mpi4py import MPI
import csv
from datetime import datetime

# Interface en ligne de commande pour le lancement de la simulation
import click

from drawing_to_simulation import load_and_launch_single_simulation, initialize_simulation, launch_parametered_simulation

@click.command()
@click.argument('path', type=click.Path(exists=True))
@click.option('-w','--width', default=50, help='Specify the width of the tiles in the simulation.')
@click.option('-r','--recursive', is_flag=True, default=False, help='To launch simulations on each room of a folder.')
@click.option('-m','--multithread', is_flag=True, default=False, help='To launch a multithreaded simulation')
@click.option('-p','--progressive',is_flag=True, default=False, help='To resume a simulation. in which not all starting positions were tested. Use it with setup.')
@click.option('--setup', is_flag=True, default=False, help='Use this option to trigger a series of prompts to setup a statistical test.')   


def sim(path,width,recursive, multithread, progressive, setup):

    if not setup: # Lancement d'une seule simulation, emplacement initial du robot mesureur déterminé par le dessin de départ.
            if recursive:
                print("You cannot run a single simulation on a folder !")
                return None

            duration = load_and_launch_single_simulation(path,width)
            print("Done in %3.2f seconds" %duration)

    else : # Lancement de nombreuses simulations pour récolter des données statistiques.

        if not multithread: # Single-threaded simulation
                
            answers = get_answers()
        
            if recursive: # Simulations sur un dossier
                list_control = []
                list_params = []
                list_filename = []

                print("There are %s rooms in the selected folder." %len(os.listdir(path)))
                print("\n")
                print("Initializing the simulations...")

                for filename in os.listdir(path):
                    print(filename+"\n")
                    control = init_sim(os.path.join(path,filename),width,recursive)
                    control, params = setup_sim(control,answers,recursive,progressive,filename)
                    list_control.append(control)
                    list_params.append(params)
                    list_filename.append(filename)

                n_sim = 0
                for params in list_params:
                    n_sim += len(params)

                print("Done")

                print("\n")
                print("The simulator will now attempt to run %s simulations." %n_sim)
                input("Press Enter to start. ")

                for k in range(len(list_control)):
                    multi_sim(list_control[k],list_params[k],list_filename[k],multithread)
                
                
                combineAllResultsAfterSim(1, list_filename, delete=False)   
                
            else: # Simulations sur un seul fichier
                filename = path.split('\\')[-1]
                control = init_sim(path,width,recursive)
                control, params = setup_sim(control,answers,recursive,progressive,filename)

                multi_sim(control,params,filename,multithread)
                combineAllResultsAfterSim(1, [filename], delete=False)  


        else : # Multi-threaded simulation
               # For multithreading : mpiexec -n [nb_threads] python ./simulator.py [...]
            global comm
            comm = MPI.COMM_WORLD
            global size
            size = comm.Get_size()
            global rank
            rank = comm.Get_rank()

            if rank == 0:
                print("nb of threads : ", size,flush=True)

            if rank == 0:
                rd.seed(10)
           
            if rank == 0:
                answers = get_answers()
            else :
                answers = None
            answers = comm.bcast(answers, root = 0)

            if recursive: # simulations sur un dossier
                list_control = []
                list_params = []
                list_filename = []

                if rank == 0:
                    print("There are %s rooms in the selected folder." %len(os.listdir(path)),flush=True)
                    print("\n",flush=True)
                    print("Initializing the simulations...",flush=True)

                # parallel initialization
                for i, filename in enumerate(os.listdir(path)):
                    if i%size == rank:
                        print(filename+"\n",flush=True)
                        control = init_sim(os.path.join(path,filename),width,recursive)
                        control, params = setup_sim(control,answers,recursive,progressive,filename)
                        list_control.append(control)
                        list_params.append(params)
                        list_filename.append(filename)

                list_control = comm.allreduce(list_control)
                list_params = comm.allreduce(list_params)
                list_filename = comm.allreduce(list_filename)
                
                n_sim = 0
                for params in list_params:
                    n_sim += len(params)

                if rank == 0:
                    print("Done",flush=True)

                if rank == 0:
                    print("\n",flush=True)
                    print("The simulator will now attempt to run %s simulations." %n_sim,flush=True)
                    input("Press Enter to start. ")

                #------------------------------------------
                comm.Barrier() 
                #------------------------------------------
                nb_room = 1
                for k in range(len(list_control)):
                    if rank == 0:
                        print('\n', flush=True)
                        print('Room',nb_room, 'out of',len(os.listdir(path)),flush=True)
                    multi_sim(list_control[k],list_params[k],list_filename[k],multithread)    
                    nb_room += 1
                combineAllResultsAfterSim(size, list_filename, multithreaded=True, delete=False) 
                
            else: # Simulations sur un seul fichier
                filename = path.split('\\')[-1]
                if rank == 0:
                    control = init_sim(path,width,recursive)
                    control, params = setup_sim(control,answers,recursive,progressive, filename)
                else:
                    control=None
                    params = None
                control = comm.bcast(control, root=0)
                params = comm.bcast(params, root=0)
                #------------------------------------------
                comm.Barrier() 
                #------------------------------------------
                multi_sim(control,params,filename,multithread)
                combineAllResultsAfterSim(size, [filename], multithreaded=True, delete=False) 


def init_sim(filename,width,recursive):
        # Initialisation, pour créer la grille une seule fois.
        if not recursive :
            print("\n",flush=True)
            print("Initializing the simulation...",flush=True)
            control = initialize_simulation(filename,width)
            print("Done\n",flush=True)
        else :
            control = initialize_simulation(filename,width)

        return control


def get_answers():
    try:
        print("\n")
        answer_h = str(input("Do you want to save a detailed historic of the state of each tile ? [y/n] "))
    except ValueError:
        print("Invalid entry - Aborting")
        return None
    print("\n")
    
    if answer_h not in ['y','n']:
        print('Please answer with yes [y] or no [n] - Aborting')
        return None
        
    # Nombre de positions de départ
    try:
        n_pos_percent = int(input("What percentage of starting positions would you like to test ? "))
    except ValueError:
        print("Not an integer - Aborting")
        return None

    if  n_pos_percent < 0 or n_pos_percent > 100:
        print("Invalid number - Aborting")
        return None
    
    # Nombres d'angles de départ par postion de départ
    try:
        n_angle = int(input("How many starting angles would you like to test per starting position ? "))
    except ValueError:
        print("Not an integer - Aborting")
        return None

    if n_angle < 1:
        print("Invalid number - Aborting")
        return None

    # Nombre de robots points de repère
    try:
        print("\nHow many RefPointBots ?")
        n_rp_bots_min = int(input("From : "))
        n_rp_bots_max = int(input("To : "))
    except ValueError:
        print("Not an integer - Aborting")
        return None

    if n_rp_bots_min < 1 or n_rp_bots_max < n_rp_bots_min:
        print("Invalid number - Aborting")
        return None

    # Types de méthodes :
    try:
        print("\nYou will now select the different methods that need to be tested.\r")
        print("If you want to select multiple methods in each category, simply type more than one number.\n")
        globalMethodRPB = str(input("Global Methods : 1='progressive' 2='reset' "))
        target_method = str(input("Target Methods : 1='findTargetV1' 2='findTargetV2' "))
        cluster_exploration_method = str(input("Cluster Exploartion Methods : 1='findClosestClusterToOrigin' 2='findClosestClusterToMeasurerBot' "))
        visited_cluster_exploration_method = str(input("Visited Cluster Exploartion Methods: same choices "))
        RPB_selection_method = str(input("RPB Selection Methods : 1='self.findLeastUsefulBotsEuclidian', 2='self.findLeastUsefulBotsDijkstra', 3='self.findLeastUsefulBotsV2Euclidian', 4='self.findLeastUsefulBotsV2Dijkstra', 5='self.findFurthestBotEuclidian', 6='self.findFurthestBotDijkstra' "))
        change_first = str(input("Change First : 1='cluster' 2='RPB' "))
        anti_loop_method = str(input("Anti Loop Method : 1='aggressive' 2='patient' "))

    except ValueError:
        print("Value Error - Aborting")
        return None

    # conversion des entrées de l'utilisateur

    tmp = []
    for char in globalMethodRPB:
            if char in ["1","2"]:
                tmp.append(int(char))
            elif char != " ":
                print("bad char globalMethodRPB: ", char)
                print("Not a valid number - Aborting")
                return None
    globalMethodRPB = tmp

    tmp = []
    for char in target_method:
            if char in ["1","2","3"]:
                tmp.append(int(char))
            elif char != " ":
                print("bad char target_method: ", char)
                print("Not a valid number - Aborting")
                return None
    target_method = tmp

    tmp = []
    for char in cluster_exploration_method:
            if char in ["1","2"]:
                tmp.append(int(char))
            elif char != " ":
                print("bad char cluster_exploration_method: ", char)
                print("Not a valid number - Aborting")
                return None
    cluster_exploration_method = tmp

    tmp = []
    for char in visited_cluster_exploration_method:
            if char in ["1","2"]:
                tmp.append(int(char))
            elif char != " ":
                print("bad char visited_cluster_exploration_method: ", char)
                print("Not a valid number - Aborting")
                return None
    visited_cluster_exploration_method = tmp

    tmp = []
    for char in RPB_selection_method:
            if char in ["1","2","3","4","5","6"]:
                tmp.append(int(char))
            elif char != " ":
                print("bad char RPB_selection_method: ", char)
                print("Not a valid number - Aborting")
                return None
    RPB_selection_method = tmp

    tmp = []
    for char in change_first:
            if char in ["1","2"]:
                tmp.append(int(char))
            elif char != " ":
                print("bad char change_first: ", char)
                print("Not a valid number - Aborting")
                return None
    change_first = tmp

    tmp = []
    for char in anti_loop_method:
            if char in ["1","2"]:
                tmp.append(int(char))
            elif char != " ":
                print("bad char anti_loop_method: ", char)
                print("Not a valid number - Aborting")
                return None
    anti_loop_method = tmp

    answers = { "history": answer_h,
                "pos_percentage": n_pos_percent,
                "nb_starting_angles": n_angle,
                "nb_rpb_min": n_rp_bots_min,
                "nb_rpb_max": n_rp_bots_max,
                "globalMethodRPB" : globalMethodRPB,
                "target_method": target_method,
                "cluster_exploration_method": cluster_exploration_method,
                "visited_cluster_exploration_method": visited_cluster_exploration_method,
                "RPB_selection_method": RPB_selection_method,
                "change_first": change_first,
                "anti_loop_method": anti_loop_method}
    
    print(answers)

    return answers


def setup_sim(control,answers,recursive, progressive, fileName):

    history = answers["history"]
    if history == 'y':
        control.grid.no_history = False
    else:
        control.grid.no_history = True

    pos_percentage = answers["pos_percentage"]
    positions = control.grid.inside
    n_pos = len(positions)
    n_pos_sim = int(np.ceil(pos_percentage/100 * n_pos))

    n_angle = answers["nb_starting_angles"]

    n_rp_bots_max = answers["nb_rpb_max"]
    n_rp_bots_min = answers["nb_rpb_min"]

    globalMethodRPB = answers["globalMethodRPB"]
    target_method = answers["target_method"]
    cluster_exploration_method = answers["cluster_exploration_method"]
    visited_cluster_exploration_method = answers["visited_cluster_exploration_method"]
    RPB_selection_method = answers["RPB_selection_method"]
    change_first = answers["change_first"]
    anti_loop_method = answers["anti_loop_method"]

    
    # Calcul des différentes valeurs de positions initiales et angles initiaux

    rd.shuffle(positions) # pour éviter d'avoir des positions voisines quand on ne teste pas toutes les cases

    if progressive:
        fileName = fileName.split('.')[-2]
        positions_sim = []
        tested_positions = []
        if os.path.exists('./rooms_tested_positions/'+fileName+'.csv'):
            with open('./rooms_tested_positions/'+fileName+'.csv','r') as tested_positions_csv:
                reader = csv.reader(tested_positions_csv, delimiter=' ')
                for row in reader:
                    tested_positions.append((int(row[0]),int(row[1])))
            tested_positions_csv.close()
            
            index = 0
            while len(positions_sim) < n_pos_sim and index < len(positions):
                pos = positions[index]
                if not pos in tested_positions:
                   positions_sim.append(pos)
                index += 1

            with open('./rooms_tested_positions/'+fileName+'.csv','a',newline='') as tested_positions_csv:
                writer = csv.writer(tested_positions_csv, delimiter=' ')
                for pos in positions_sim:
                    writer.writerow([pos[0],pos[1]])
            tested_positions_csv.close()

        else:
            for pos in positions[:n_pos_sim]:
                positions_sim.append(pos)

            with open('./rooms_tested_positions/'+fileName+'.csv','w',newline='') as tested_positions_csv:
                writer = csv.writer(tested_positions_csv, delimiter=' ')
                for pos in positions_sim:
                    writer.writerow([pos[0],pos[1]])
            tested_positions_csv.close()
            
    else:
        if n_pos_sim < n_pos :
            positions_sim = positions[:n_pos_sim]
        else:
            positions_sim = positions

    
    angles_sim = np.linspace(0, 2*np.pi/control.nbRefPointBots, n_angle+1)
    angles_sim = angles_sim[:-1]

    # Création de toutes les combinaisons uniques de paramètres
    method_params = []
    for g in globalMethodRPB:
        for t in target_method:
            for c in cluster_exploration_method:
                for v in visited_cluster_exploration_method:
                    for R in RPB_selection_method:
                        for ch in change_first:
                            for a in anti_loop_method:
                                method_params.append((g,t,c,v,R,ch,a))

    parameters = []
    for pos in positions_sim:
        for angle in angles_sim:
            for nbRefPointBots in range(n_rp_bots_min,n_rp_bots_max + 1):
                for methods in method_params:
                    parameters.append((pos,angle,nbRefPointBots,methods))

    # On mélange les paramètres, de manière à ne pas confier à un thread des simulations toujours plus simples (cas particulier possible)
    rd.shuffle(parameters)

    if not recursive:
        print("\n")
        print("The simulator will now attempt to run %s simulations." %len(parameters))
        input("Press Enter to start. ")

    return control, parameters


def multi_sim(control,parameters,filename, multithread):

    if not multithread:
        dirname = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(dirname, "./results/",str(filename)+"LOG.txt"), "w") as log:

            simulation_number = 1
            file_number = 1
            multiple_metrics = {'sim_number'           : [],
                                'start_pos'            : [],
                                'start_angle'          : [],
                                'nbRefPointBots'       : [],
                                'nbMeasurerBots'       : [],
                                'globalMethodRPB'      : [],
                                'mb_exp_method'        : [],
                                'rpb_exp_method'       : [],
                                'rpb_sel_method'       : [],
                                'first_loop'           : [],
                                'measuredTiles'        : [],
                                'surface'              : [],
                                'pathLength'           : [],
                                'visitsPerTile'        : [],
                                'history'              : [],
                                'sim_duration'         : [],
                                'avg_iteration_duration':[],
                                'totalPathLengthRPB'   : [],
                                'nbMovesRPB'           : [],
                                'averageMoveLengthRPB' : [],
                                'maxLengthMoveRPB'     : []}

            ### Multiples simulations
            start = time.time()
            metrics = None

            durations = []
            nb_simu = len(parameters)
            
            for params in parameters:
                
                control_param = copy.deepcopy(control)
                metrics = launch_parametered_simulation(control_param,params)

                multiple_metrics['sim_number'].append(simulation_number)
                multiple_metrics['start_pos'].append(params[0])
                multiple_metrics['start_angle'].append(params[1])
                for key in metrics.keys():
                    if (key != 'history' and control.grid.no_history == True) or control.grid.no_history == False:
                        multiple_metrics[key].append(metrics[key])
                
                log.write("Simulation number %s done in %3.2f s\n" %(simulation_number,metrics["sim_duration"]))
                nbRPB = multiple_metrics["nbRefPointBots"][-1]
                log.write(f"Nb RefPointBots : {nbRPB}\n")
                nbTiles = multiple_metrics["measuredTiles"][-1]
                log.write(f"Measured Tiles : {nbTiles}\n")

                # gestion de la barre de chargement
                durations.append(metrics["sim_duration"])
                avg_duration = np.mean(durations)
                done = simulation_number/nb_simu
                nb = int(done*40)

                bar = '   ['+'#'*nb +'-'*(40-nb)+']' + '  ' +' '*(3-len(str(int(done*100))))+str(int(done*100))+'%' + '  ' + time.strftime('%H:%M:%S', time.gmtime(int(max(0,nb_simu*avg_duration*(1-done)))))
                
                sys.stdout.write("\033[F") # efface la barre précédente
                print(bar,flush=True)

                simulation_number += 1
                if simulation_number % 100 == 0: # Toutes les 100 simulations, on sauvegarde les résultats dans un gros fichier.
                    file = open("./results/" +str(filename)+"-noGUI-results-"+str(file_number)+".pickle", "wb")
                    pickle.dump(multiple_metrics, file)
                    file.close()

                    multiple_metrics = {'sim_number'           : [],
                                        'start_pos'            : [],
                                        'start_angle'          : [],
                                        'nbRefPointBots'       : [],
                                        'nbMeasurerBots'       : [],
                                        'globalMethodRPB'      : [],
                                        'mb_exp_method'        : [],
                                        'rpb_exp_method'       : [],
                                        'rpb_sel_method'       : [],
                                        'first_loop'           : [],
                                        'measuredTiles'        : [],
                                        'surface'              : [],
                                        'pathLength'           : [],
                                        'visitsPerTile'        : [],
                                        'history'              : [],
                                        'sim_duration'         : [],
                                        'avg_iteration_duration':[],
                                        'totalPathLengthRPB'   : [],
                                        'nbMovesRPB'           : [],
                                        'averageMoveLengthRPB' : [],
                                        'maxLengthMoveRPB'     : []}
                    file_number += 1

            if multiple_metrics is not None:
                file = open("./results/"+str(filename)+"-noGUI-results-"+str(file_number)+".pickle", "wb")
                pickle.dump(multiple_metrics, file)
                file.close()
           
            print("Done in %3.2f seconds" %(time.time()-start))
            
            log.write("Simulations were all successfull")

        log.close()


    else: # multithread
        
        dirname = os.path.dirname(os.path.abspath(__file__))
        if rank == 0: 
            print("dirname : " ,dirname,'\n', flush=True)
            done = 0
        else:
            done = None
        
        simulation_number = 1
        file_number = 1
        multiple_metrics = {'sim_number'           : [],
                            'start_pos'            : [],
                            'start_angle'          : [],
                            'nbRefPointBots'       : [],
                            'nbMeasurerBots'       : [],
                            'globalMethodRPB'      : [],
                            'mb_exp_method'        : [],
                            'rpb_exp_method'       : [],
                            'rpb_sel_method'       : [],
                            'first_loop'           : [],
                            'measuredTiles'        : [],
                            'surface'              : [],
                            'pathLength'           : [],
                            'visitsPerTile'        : [],
                            'history'              : [],
                            'sim_duration'         : [],
                            'avg_iteration_duration':[],
                            'totalPathLengthRPB'   : [],
                            'nbMovesRPB'           : [],
                            'averageMoveLengthRPB' : [],
                            'maxLengthMoveRPB'     : []}

        ### Multiples simulations
        start = time.time()
        metrics = None

        durations = []
        
        for i, params in enumerate(parameters):
            if i%size == rank: 
                control_param = copy.deepcopy(control)
                metrics = launch_parametered_simulation(control_param,params)
                multiple_metrics['sim_number'].append(simulation_number)
                multiple_metrics['start_pos'].append(params[0])
                multiple_metrics['start_angle'].append(params[1])
                for key in metrics.keys():
                    if (key != 'history' and control.grid.no_history == True) or control.grid.no_history == False:
                        multiple_metrics[key].append(metrics[key])
                with open("./results/" +str(filename)+"LOG-nproc"+str(rank)+".txt", "a") as log:
                    log.write("Simulation number %s done in %3.2f s\n" %(simulation_number,metrics["sim_duration"]))
                    nbRPB = multiple_metrics["nbRefPointBots"][-1]
                    log.write(f"Nb RefPointBots : {nbRPB}\n")
                    nbTiles = multiple_metrics["measuredTiles"][-1]
                    log.write(f"Measured Tiles : {nbTiles}\n")
                    log.close()

                
                durations.append(metrics["sim_duration"])
                avg_duration = np.mean(durations)

                # On superpose les barres de chargement de tous les threads
                fraction = simulation_number/(len(parameters)/size)
                fraction = min(1,fraction)
                nb = int(fraction*40)

                bar = str(rank)+' '*(3-len(str(rank)))+'['+'#'*nb +'-'*(40-nb)+']' + '  ' +' '*(3-len(str(min(100,int(np.ceil(fraction*100))))))+str(min(100,int(np.ceil(fraction*100))))+'%' + '  ' + time.strftime('%H:%M:%S', time.gmtime(int(max(0,len(parameters)/size*avg_duration*(1-fraction)))))

                sys.stdout.write("\033[F") # passe à la ligne précédente
                print(bar,flush=True)
                            
                simulation_number += 1
                if simulation_number % 100 == 0: # Toutes les 100 simulations, on sauvegarde les résultats dans un gros fichier.
                    # file = open(os.path.join(dirname, "./results/",str(filename[8:-7])+"-noGUI-results-"+str(file_number)+"nproc"+str(rank)+".pickle"), "wb")
                    file = open("./results/" +str(filename)+"-noGUI-results-"+str(file_number)+"nproc"+str(rank)+".pickle", "wb")
                    pickle.dump(multiple_metrics, file)
                    file.close()

                    multiple_metrics = {'sim_number'           : [],
                                        'start_pos'            : [],
                                        'start_angle'          : [],
                                        'nbRefPointBots'       : [],
                                        'nbMeasurerBots'       : [],
                                        'globalMethodRPB'      : [],
                                        'mb_exp_method'        : [],
                                        'rpb_exp_method'       : [],
                                        'rpb_sel_method'       : [],
                                        'first_loop'           : [],
                                        'measuredTiles'        : [],
                                        'surface'              : [],
                                        'pathLength'           : [],
                                        'visitsPerTile'        : [],
                                        'history'              : [],
                                        'sim_duration'         : [],
                                        'avg_iteration_duration':[],
                                        'totalPathLengthRPB'   : [],
                                        'nbMovesRPB'           : [],
                                        'averageMoveLengthRPB' : [],
                                        'maxLengthMoveRPB'     : []}
                    file_number += 1


        # file = open(os.path.join(dirname, "./results/",str(filename[8:-7])+"-noGUI-results-"+str(file_number)+"nproc"+str(rank)+".pickle"), "wb")
        if multiple_metrics is not None:
            file = open("./results/"+str(filename)+"-noGUI-results-"+str(file_number)+"nproc"+str(rank)+".pickle", "wb")
            pickle.dump(multiple_metrics, file)
            file.close()

        #------------------------------------------
        comm.Barrier() 
        #------------------------------------------

        if rank == 0:
            print("Done in %3.2f seconds\n" %(time.time()-start))
        with open("./results/" +str(filename)+"LOG-nproc"+str(rank)+".txt", "a") as log:
            log.write("Simulations were all successfull")
            log.close()


def combineAllLogs():
    pass



def combineAllResultsAfterSim(nb_cores, file_names, file_path_results = "results/", file_path=None, multithreaded = False, delete = True):
    date = datetime.now().isoformat().split(".")[0].replace(":", "-")
    csvFile = open(f"{file_path_results}combined_results_{date}.csv", 'w')
    writer = csv.writer(csvFile)
    if file_path is not None:
        file_names = []
        for filename in os.listdir(file_path):
            if filename[-6:] == "pickle":
                file_names.append(os.path.join(file_path,filename))

    header = False
    for filename in file_names:
        # cas où on lance la fonction juste après la création des fichiers pickle
        if file_path is None:
            for core in range(nb_cores):
                file_number=1
                while True:
                    name = filename
                    if multithreaded:
                        if rank == 0: # le premier thread s'occupe de faire les regroupements
                            try :
                                with open(f"{file_path_results}{name}-noGUI-results-{file_number}nproc{core}.pickle", 'rb') as pickle_file:
                                    data = pickle.load(pickle_file)
                                    pickle_file.close()
                                if delete:
                                    os.remove(f"{file_path_results}{name}-noGUI-results-{file_number}nproc{core}.pickle")
                            except FileNotFoundError:
                                break

                            data.pop("history", None) ################## ça serait sympa de les récupérer qq part qd même
                            data.pop("visitsPerTile", None)
                            n = len(data["sim_number"])
                            if not header:
                                writer.writerow(list(data.keys())+["roomName"])
                                header = True
                            for i in range(n):
                                writer.writerow([data_list[i] for data_list in list(data.values())] + [filename.split("\\")[-1]])
                            file_number+=1
                        else: # les autres threads n'ont rien à faire
                            break

                    else:
                        try :
                            with open(f"{file_path_results}{name}-noGUI-results-{file_number}.pickle", 'rb') as pickle_file:
                                data = pickle.load(pickle_file)
                                pickle_file.close()
                            if delete:
                                os.remove(f"{file_path_results}{name}-noGUI-results-{file_number}.pickle")
                        except FileNotFoundError:
                            break

                        data.pop("history", None)
                        data.pop("visitsPerTile", None)
                        n = len(data["sim_number"])
                        if not header:
                            writer.writerow(list(data.keys())+["roomName"])
                            header = True
                        for i in range(n):
                            writer.writerow([data_list[i] for data_list in list(data.values())] + [filename.split("\\")[-1]])
                        file_number+=1

        # cas où on lance la fonction dans sur des fichiers pickle existants dans le dossier file_path
        else:
            with open(filename, 'rb') as pickle_file:
                data = pickle.load(pickle_file)
                pickle_file.close()
            if delete:
                os.remove(filename)
            data.pop("history", None)
            data.pop("visitsPerTile", None)
            n = len(data["sim_number"])
            if not header:
                writer.writerow(list(data.keys())+["roomName"])
                header = True
            for i in range(n):
                writer.writerow([data_list[i] for data_list in list(data.values())] + [filename.split("\\")[-1]])


if __name__ == "__main__":
    sim()
    # combineAllResultsAfterSim(size, [], file_path="./results/simFolderTest/", multithreaded=True, delete=False)