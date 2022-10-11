import numpy as np

# https://stackoverflow.com/questions/34438313/identifying-groups-of-similar-numbers-in-a-list
def ordered_cluster(data, max_diff):
    current_group = ()
    for item in data:
        test_group = current_group + (item, )
        test_group_mean = np.mean(test_group)
        if all((abs(test_group_mean - test_item) < max_diff for test_item in test_group)):
            current_group = test_group
        else:
            yield current_group
            current_group = (item, )
    if current_group:
        yield current_group


def find_index_walls(table):
    horizontal_index = []
    vertical_index  = []

    L = len(table)      # nombre de lignes
    C = len(table[0])   # nombre de colonnes

    # balayage horizontal
    outside_a_wall = True
    for l in range(L):
        for c in range(C):
            if outside_a_wall and table[l,c] == 0:
                outside_a_wall = False
                horizontal_index.append(c)
            
            if not outside_a_wall and table[l,c] == -1:
                outside_a_wall = True

    # balayage vertical
    outside_a_wall = True
    for c in range(C):
        for l in range(L):
            if outside_a_wall and table[l,c] == 0:
                outside_a_wall = False
                vertical_index.append(l)
            
            if not outside_a_wall and table[l,c] == -1:
                outside_a_wall = True  

    horizontal_index.sort()
    vertical_index.sort()

    return horizontal_index, vertical_index


def average_index_walls(horizontal_index, vertical_index):
    horizontal_clusters = ordered_cluster(horizontal_index,10)
    vertical_clusters = ordered_cluster(vertical_index,10)

    horizontal_averages = []
    vertical_averages = []
    for cluster in horizontal_clusters:
        horizontal_averages.append(np.floor(np.mean(cluster)))
    for cluster in vertical_clusters:
        vertical_averages.append(np.floor(np.mean(cluster)))

    return horizontal_averages, vertical_averages


def closest(index,ref):
    m, m_index = abs(index-int(ref[0])), 0
    for i in range(len(ref)):
        if abs(index-int(ref[i])) < m:
            m = abs(index-int(ref[i]))
            m_index = i

    return int(ref[m_index])


def fill_small_hole(table):
    L = len(table)      # nombre de lignes
    C = len(table[0])   # nombre de colonnes
    max_error = 10

    for l in range(L):
        row = table[l,:]
        row = list(row)
        nb_pop = 0
        # on isole un vecteur qui commence et se termine par 0
        while row[0] != 0 and len(row) > 1:
            row.pop(0)
            nb_pop +=1
        while row[-1] != 0 and len(row) > 1:
            row.pop()

        if len(row) > 1:
            # on cherche tous les trous dans la ligne
            trous = []
            dans_un_trou = False
            for c in range(len(row)):
                if row[c] == -1 and dans_un_trou == False:
                    dans_un_trou = True
                    nouveau_trou = [(l, nb_pop + c)]
                elif row[c] == -1 and dans_un_trou == True:
                    nouveau_trou.append([l, nb_pop + c])
                elif row[c] == 0 and dans_un_trou == True:
                    dans_un_trou = False
                    trous.append(nouveau_trou)
                elif row[c] == 1 or row[c] == 2 or row[c] == 3:
                    nouveau_trou = []
                    nouveau_trou.append("pas un trou")
            # si les trous sont petits on les comble
            for trou in trous:
                if not "pas un trou" in trou:
                    if  len(trou) < max_error:
                        for (l,c) in trou:
                            table[l,c] = 0
            
    # même chose mais pour les colonnes
    for c in range(C):
        col = table[:,c]
        col = list(col)
        nb_pop = 0
        while col[0] != 0 and len(col) > 1:
            col.pop(0)
            nb_pop += 1
        while col[-1] != 0 and len(col) > 1:
            col.pop()
        
        if len(col) > 1:
            # on cherche tous les trous dans la ligne
            trous = []
            dans_un_trou = False
            for l in range(len(col)):
                if col[l] == -1 and dans_un_trou == False:
                    dans_un_trou = True
                    nouveau_trou = [(nb_pop + l, c)]
                elif col[l] == -1 and dans_un_trou == True:
                    nouveau_trou.append([nb_pop + l, c])
                elif col[l] == 0 and dans_un_trou == True:
                    dans_un_trou = False
                    trous.append(nouveau_trou)
                elif col[l] == 1 or col[l] == 2 or col[l] == 3:
                    nouveau_trou.append("pas un trou")
            # si les trous sont petits on les comble
            for trou in trous:
                if not "pas un trou" in trou:
                    if  len(trou) < max_error:
                        for (l,c) in trou:
                            table[l,c] = 0

    return table


def fill_bottom_right_corner(table): # rempli les coins de type "en bas à droite" qui, expérimentalement, sont les seuls non comblés
    row, col1, col2 = 1, 1, 2
    
    while row != 127 or col2 != 219:
        #  on s'éparge le traitement de lignes inutiles
        while (not (0 in table[row,:])) and row != 127:
            row += 1
            
            col1 = 1
            col2 = 2 
            
        area_index = [(row,col1),(row,col2)]
        neighbours_index = [(row-1,col1),(row-1,col2),(row,col1-1)]

        empty_area_count = 0
        neighbours_count = 0
        # on cherche à savoir si la zone étudiée est vide (zone = carré 2*2 ou rect 1*2)
        for (l,c) in area_index:
            if table[l,c] == -1:
                empty_area_count += 1
        # on cherche à savoir si les voisins du dessus et de gauche sont des morceaux de mur
        for (l,c) in neighbours_index:
            if table[l,c] == 0:
                neighbours_count += 1
        # si les 2 conditions sont remplies, on a identifié un coin inférieur droit vide, et on le remplit
        if empty_area_count == 2 and neighbours_count == 3 and (table[row-1,col1-1] == -1 or table [row-2,col1-2]):
            for (l,c) in area_index:
                table[l,c] = 0    

        # on parcourt tout le tableau en groupe de carrés 2x2
        if col2 != 219:
            col1 += 1
            col2 += 1
        else :
            row += 1
            col1 = 1
            col2 = 2 

    return table


def find_square(row,col,table):
    if table[row+1,col] == 0 and table[row+1,col+1] == 0 and table[row,col+1] == 0:
        square_indexes = [(row,col),(row+1,col),(row+1,col+1),(row,col+1)]     
    elif table[row+1,col] == 0 and table[row+1,col-1] == 0 and table[row,col-1] == 0:
        square_indexes = [(row,col),(row+1,col),(row+1,col-1),(row,col-1)]
    elif table[row-1,col] == 0 and table[row-1,col-1] == 0 and table[row,col-1] == 0:
        square_indexes = [(row,col),(row-1,col),(row-1,col-1),(row,col-1)]
    elif table[row-1,col+1] == 0 and table[row,col+1] == 0 and table[row-1,col] == 0:
        square_indexes = [(row,col),(row-1,col),(row-1,col+1),(row,col+1)]
    else:
        return []

    return square_indexes


def neighbours_pixel(row,col):
    if row <= 126 and row >= 1 and col >= 1 and col <= 218:
        return [(row-1,col),(row+1,col),(row,col-1),(row,col+1)]
    else :
        return [] # grosse flemme


def neighbours_square(square_indexes):
    if square_indexes == []:
        return []

    square_neighbours_indexes = []
    for index in square_indexes:
        row = index[0]
        col = index[1]
        temp =  neighbours_pixel(row,col)
        for index in temp:
            if not(index in square_indexes) and not(index in square_neighbours_indexes):
                square_neighbours_indexes.append(index)
    
    return square_neighbours_indexes


def find_intersections(table):
    intersection_indexes = []

    for row in range(len(table)):
        for col in range(len(table[0])):
            if table[row][col] == 0:
                square_indexes = find_square(row,col,table)
                square_neighbours_indexes = neighbours_square(square_indexes)
                if len(square_neighbours_indexes) != 0:
                    neighbours_count = 0
                    for index in square_neighbours_indexes:
                        if table[index[0],index[1]] == 0:
                            neighbours_count += 1
                    # si on trouve un carré duquel partent au moins trois branches
                    if neighbours_count >= 6:
                        if square_indexes not in intersection_indexes:
                            intersection_indexes.append(square_indexes)
    
    return intersection_indexes

def clean_intersections(table):
    max_error = 10
    intersection_indexes = find_intersections(table)
    for list_of_indexes in intersection_indexes:
        row1 = min([index[0] for index in list_of_indexes])
        row2 = max([index[0] for index in list_of_indexes])
        col1 = min([index[1] for index in list_of_indexes])
        col2 = max([index[1] for index in list_of_indexes])

        # exploration vers le bas
        k = 0
        while table[row2+k][col1] == 0:
            k+=1
        if k < max_error:
            for row in range(row2+1,row2+k):
                for col in [col1,col2]:
                    if not (row,col) in list_of_indexes:
                        table[row,col] = -1

        # exploration vers le haut
        k = 0
        while table[row1-k][col1] == 0:
            k+=1
        if k < max_error:
            for row in range(row1-2,row1-k,-1): # on met un -2 par sécurité car l'intersection peut être détectée un cran trop bas
                for col in [col1,col2]:
                    if not (row,col) in list_of_indexes:
                        table[row,col] = -1

        # exploration vers la droite
        k = 0
        while table[row1][col2+k] == 0:
            k+=1
        if k < max_error:
            for row in [row1,row2]:
                for col in range(col2+1,col2+k):
                    if not (row,col) in list_of_indexes:
                        table[row,col] = -1

        # exploration vers la gauche
        k = 0
        while table[row1][col1-k] == 0:
            k+=1
        if k < max_error:
            for row in [row1,row2]:
                for col in range(col1-1,col2-k,-1):
                    if not (row,col) in list_of_indexes:
                        table[row,col] = -1

    return table
                  

def straighten_walls(table):

    horizontal_index, vertical_index = find_index_walls(table)
    horizontal_averages, vertical_averages = average_index_walls(horizontal_index, vertical_index)

    L = len(table)      # nombre de lignes
    C = len(table[0])   # nombre de colonnes

    result = np.zeros((L,C))
    result = result -1
    
    # balayage horizontal
    outside_a_wall = True
    for l in range(L):
        for c in range(C):
            if outside_a_wall and table[l,c] == 0:
                outside_a_wall = False
                h_index = closest(c,horizontal_averages)
                result[l,h_index], result[l,h_index+1] = 0, 0
            
            if not outside_a_wall and table[l,c] == -1:
                outside_a_wall = True

    # balayage vertical
    outside_a_wall = True
    for c in range(C):
        for l in range(L):
            if outside_a_wall and table[l,c] == 0:
                outside_a_wall = False
                v_index = closest(l,vertical_averages)
                result[v_index,c], result[v_index+1,c] = 0, 0
            
            if not outside_a_wall and table[l,c] == -1:
                outside_a_wall = True  

    # recopier les autre couleurs que le noir
    for l in range(L):
        for c in range(C):
            if table[l,c] !=0 and table[l,c] != -1:
                result[l,c] = table[l,c]   

    # on comble les trous
    result = fill_small_hole(result) 
    # et les coins
    result = fill_bottom_right_corner(result)
              
    return result


def find_color_groups(color, table):
    already_visited = []
    left_to_visit = []
    for Row in range(0,len(table)-1): # on évite les bords, car voisins pas définis correctement      
        for Col in range(0,len(table[0])-1):
            if table[Row,Col] != color:
                already_visited.append((Row,Col))
            else :
                left_to_visit.append((Row,Col))

    color_groups = []
    for (Row,Col) in left_to_visit:

        if not (Row,Col) in already_visited:
            
            color_group_indexes = [(Row,Col)]
            already_visited.append((Row,Col))
            neighbours_indexes = neighbours_pixel(Row,Col)
            to_check = []

            for index in neighbours_indexes:
                if not index in already_visited:
                    to_check.append(index)

            while len(to_check) > 0:
                (row,col) = to_check.pop()
                already_visited.append((row,col))
                
                if not (row,col) in color_group_indexes: 
                    color_group_indexes.append((row,col))
                    neighbours_indexes = neighbours_pixel(row,col)
                    for index in neighbours_indexes:
                        if not index in already_visited:
                            if not index in to_check:
                                to_check.append(index)
                                        
                                    
                color_groups.append(color_group_indexes)
    
    return color_groups
    

def clean_robots(table):
    red_groups = find_color_groups(1,table)
    green_groups = find_color_groups(2,table)
    blue_groups = find_color_groups(3,table)

    for red_group_indexes in red_groups:
        rows = []
        cols = []
        for (row,col) in red_group_indexes:
            table[row,col] = -1
            rows.append(row)
            cols.append(col)
        
        table[int(np.floor(np.mean(rows))),int(np.floor(np.mean(cols)))] = 1

    for green_group_indexes in green_groups:
        rows = []
        cols = []
        for (row,col) in green_group_indexes:
            table[row,col] = -1
            rows.append(row)
            cols.append(col)
        
        table[int(np.floor(np.mean(rows))),int(np.floor(np.mean(cols)))] = 2

    for blue_group_indexes in blue_groups:
        rows = []
        cols = []
        for (row,col) in blue_group_indexes:
            table[row,col] = -1
            rows.append(row)
            cols.append(col)
        
        table[int(np.floor(np.mean(rows))),int(np.floor(np.mean(cols)))] = 3

    return table


def clean_walls_and_robots(table):
    result = clean_intersections(table)
    result = clean_robots(result)  
    
    return result