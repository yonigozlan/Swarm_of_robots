from shutil import copy
import pygame
import pygame_menu

from draw import draw_initial_config
from drawing_to_simulation import load_and_launch_exact_simulation, load_and_launch_discrete_simulation

pygame.init()

sw, sh = 1600, 900  # screenWidth, screenHeight
screen = pygame.display.set_mode((sw, sh))

# -------------------
# Global theme of the menus
# -------------------

font = pygame_menu.font.FONT_OPEN_SANS

mytheme = pygame_menu.themes.THEME_DEFAULT.copy()
mytheme.title_bar_style = pygame_menu.widgets.MENUBAR_STYLE_SIMPLE
mytheme.selection_color = (0,0,0)

# Paramètres de la simulation
defaultParameters = ["progressive", "findTargetV2", "findClosestClusterToOrigin", "findClosestClusterToMeasurerBot", "findLeastUsefulBotsDijkstra", "cluster", "aggressive"]
parameters = {}
parameters["globalMethodRPB"] = [("progressive",), ("reset",)]
parameters["targetMethod"] = [("findTargetV2",), ("findTargetV1",)]
parameters["clusterExplorationMethod"] = [("findClosestClusterToOrigin",), ("findClosestClusterToMeasurerBot",)]
parameters["visitedClusterExplorationMethod"]=[("findClosestClusterToMeasurerBot",), ("findClosestClusterToOrigin",)]
parameters["RPBSelectionMethod"]=[("findLeastUsefulBotsEuclidian",), ("findLeastUsefulBotsDijkstra",), ("findLeastUsefulBotsV2Euclidian",), ("findLeastUsefulBotsV2Dijkstra",),("findFurthestBotEuclidian",), ("findFurthestBotDijkstra",)]
parameters["changeFirst"]=[("cluster",), ("RPB",)]
parameters["antiLoopMethod"]=[("aggressive",), ("patient",)]

def updateParameters(item, index):
    defaultParameters[index] = item[0][0]

                     
# -------------------
# Main menu
# -------------------

main_menu = pygame_menu.Menu(900, 1600, 'Welcome', mouse_motion_selection=True, theme=mytheme)

main_menu.add_button('Initial configuration sketcher', draw_initial_config)
main_menu.add_vertical_margin(20)
main_menu.add_button("Load and launch 'exact' simulation", load_and_launch_exact_simulation, defaultParameters)
main_menu.add_button('Load and launch discrete simulation', load_and_launch_discrete_simulation, defaultParameters)
main_menu.add_vertical_margin(40)
main_menu.add_label("Parameters : ")
main_menu.add_selector("Select global method RPB : ",parameters["globalMethodRPB"], onchange=updateParameters, index = 0)
main_menu.add_selector("Select MeasurerBot exploration method : ",parameters["targetMethod"], onchange=updateParameters, index = 1)
main_menu.add_selector("Select cluster exploration method : ",parameters["clusterExplorationMethod"], onchange=updateParameters, index = 2)
main_menu.add_selector("Select visited cluster exploration method  : ",parameters["visitedClusterExplorationMethod"], onchange=updateParameters, index = 3)
main_menu.add_selector("Select RPB selection method : ",parameters["RPBSelectionMethod"], onchange=updateParameters, index = 4)
main_menu.add_selector("Select what to loop through first : ",parameters["changeFirst"], onchange=updateParameters, index = 5)
main_menu.add_selector("Select anti infinite loop method : ",parameters["antiLoopMethod"], onchange=updateParameters, index = 6)


main_menu.add_vertical_margin(40)
main_menu.add_button('Quit', pygame_menu.events.EXIT)
main_menu.add_vertical_margin(20)
main_menu.mainloop(screen)
