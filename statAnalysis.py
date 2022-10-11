def getDataByRooms(data, parametersToTest, nbRobots=None, parameterY = "measuredTiles"):

    rooms = data["roomName"].unique()
    series = [[] for i in range(len(parametersToTest))]
    
    for j, parameter in enumerate(parametersToTest):
        for k, room in enumerate(rooms):
            series[j].append(data[
            (data["rpb_sel_method"] == parameter) 
            & (data["roomName"] == room)][parameterY].mean())

    return series, rooms


def getDataByNbRPB(data, parameterLabel, parametersToTest, parameterY = "measuredTiles"):

    nbRefPointBots = data["nbRefPointBots"].unique()

    series = [[] for j in range(len(parametersToTest))]
    for i, nb in enumerate(data["nbRefPointBots"].unique()):
        for j, parameter in enumerate(parametersToTest):
            series[j].append(data[(data["nbRefPointBots"] == nb) & (data[parameterLabel] == parameter) ][parameterY].mean())
    
    return series, nbRefPointBots

def getAllParams(df):
    for key in df:
        data = df[key].unique()
        if len(data) < 30:
            print(key," : ", data)
        else : print(key, "length : ", len(data))