import os
import csv
import pickle
from datetime import datetime


def combineAllResultsAfterSim(nb_cores, file_names, file_path_results = "results/", file_path=None, multithreaded = False, delete = True):
    date = datetime.now().isoformat().split(".")[0].replace(":", "-")
    csvFile = open(f"{file_path_results}combined_results_{date}.csv", 'w')
    writer = csv.writer(csvFile)
    if file_path is not None:
        file_names = []
        for filename in os.listdir(file_path):
            if filename[-6:] == "pickle":
                file_names.append(os.path.join(file_path,filename))

    print(file_names)

    header = False
    for filename in file_names:
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


combineAllResultsAfterSim(int(input("Combien de threads ? ")), [], file_path_results="./", file_path="./", multithreaded=True, delete=False)