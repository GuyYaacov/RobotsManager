import copy
import math
import pandas as pd
from matplotlib import pyplot as plt
import matplotlib.animation as animation
# Here you may choose any map.csv file wich contains 0's for empty space, 1 for target point, 2's for obstacles and 3 for starting point
class WavePlanner:

    def __init__(self, imap):
        self.START_POS = ()
        self.POS = ()
        self.GOAL = ()
        self.MAP = imap  # matrix
        self.ROW = len(self.MAP) - 1  # int
        self.COL = len(self.MAP[0]) - 1  # int
        self.FindStartAndGoal()
        self.VISITED = [[False for j in range(self.COL + 1)] for i in range(self.ROW + 1)]  # boolean matrix
        self.WAVE_MAP = [[0 for j in range(self.COL + 1)] for i in range(self.ROW + 1)]
        self.PATH = []
        self.UNREACHABLE = False
        self.FindStartAndGoal()
        self.CreateWaveMap()
        self.FillTheWavesIn()
        if not self.UNREACHABLE:
            self.BuildPath()

    def FindStartAndGoal(self):
        goal_reached = 0
        start_reached = 0
        for i in range(self.ROW + 1):
            for j in range(self.COL + 1):
                if self.MAP[i][j] == 1 and goal_reached != 1:
                    self.GOAL = (j, i)
                    goal_reached = 1
                if self.MAP[i][j] == 3 and start_reached != 1:
                    self.POS = (j, i)
                    start_reached = 1
                if start_reached == 1 and goal_reached == 1:
                    break
        self.START_POS = self.POS

    def CreateWaveMap(self):
        for i in range(self.ROW + 1):
            for j in range(self.COL + 1):
                if self.MAP[i][j] == 1:
                    self.WAVE_MAP[i][j] = 2
                elif self.MAP[i][j] == 2:
                    self.WAVE_MAP[i][j] = 1

    def isSafe(self, i, j):
        # row number is in range, column number
        # is in range and value is 1
        # and not yet visited
        return (i >= 0 and i < self.ROW+1 and j >= 0 and j < self.COL+1)


    def FillTheWavesIn(self):
        rowNbr = [-1, -1, -1, 0, 0, 1, 1, 1]
        colNbr = [-1, 0, 1, -1, 1, -1, 0, 1]
        wave_strength = 3
        prv_wave_strength = 2
        done = False
        while not done:
            deploys = 0
            for i in range(self.ROW + 1):
                for j in range(self.COL + 1):
                    if self.WAVE_MAP[i][j] == prv_wave_strength:
                        for k in range(8):
                            if self.isSafe(i + rowNbr[k], j + colNbr[k]):
                                if self.WAVE_MAP[i + rowNbr[k]][j + colNbr[k]] == 0:
                                    self.WAVE_MAP[i + rowNbr[k]][j + colNbr[k]] = wave_strength
                                    deploys = deploys + 1
            if deploys == 0:
                done = True
                if self.WAVE_MAP[self.POS[1]][self.POS[0]] == 0:
                    self.UNREACHABLE = True
            prv_wave_strength = wave_strength
            wave_strength = wave_strength + 1

    def BuildPath(self):
        curr_val = self.WAVE_MAP[self.POS[1]][self.POS[0]]
        rowNbr = [-1, -1, -1, 0, 0, 1, 1, 1]
        colNbr = [-1, 0, 1, -1, 1, -1, 0, 1]
        done = False
        oohhh = False
        while not done:
            for k in range(8):
                if self.isSafe(self.POS[1] + rowNbr[k], self.POS[0] + colNbr[k]) and self.WAVE_MAP[self.POS[1] + rowNbr[k]][self.POS[0] + colNbr[k]] != 1:
                    if self.WAVE_MAP[self.POS[1] + rowNbr[k]][self.POS[0] + colNbr[k]] == 2:
                        self.POS = self.GOAL
                        self.PATH.append(self.POS)
                        done = True
                        break
                    if self.WAVE_MAP[self.POS[1] + rowNbr[k]][self.POS[0] + colNbr[k]] < curr_val and not oohhh:
                        self.POS = (self.POS[0] + colNbr[k], self.POS[1] + rowNbr[k])
                        self.PATH.append(self.POS)
                        curr_val = self.WAVE_MAP[self.POS[1]][self.POS[0]]
                        break
                    elif self.WAVE_MAP[self.POS[1] + rowNbr[k]][self.POS[0] + colNbr[k]] == curr_val and oohhh:
                        self.POS = (self.POS[0] + colNbr[k], self.POS[1] + rowNbr[k])
                        self.PATH.append(self.POS)
                        oohhh = False
                        break
                    if k == 7:
                        oohhh = True

class DefineObjects:

    def __init__(self, imap, idx):
        self.MAP = imap
        self.IDX = idx
        self.OBJECTS = []
        self.ROW = len(self.MAP) # int
        self.COL = len(self.MAP[0]) # int
        self.COUNT = 0
        self.countObjects()

    def isSafe(self, i, j, visited):
        # row number is in range, column number
        # is in range and value is 1
        # and not yet visited
        return (i >= 0 and i < self.ROW and j >= 0 and j < self.COL and not visited[i][j] and self.MAP[i][j] == self.IDX)

    # A utility function to do DFS for a 2D
    # boolean matrix. It only considers
    # the 8 neighbours as adjacent vertices
    def DFS(self, i, j, visited):
        # These arrays are used to get row and
        # column numbers of 8 neighbours
        # of a given cell
        rowNbr = [-1, -1, -1, 0, 0, 1, 1, 1]
        colNbr = [-1, 0, 1, -1, 1, -1, 0, 1]
        # Mark this cell as visited
        visited[i][j] = True
        # Recur for all connected neighbours
        for k in range(8):
            if self.isSafe(i + rowNbr[k], j + colNbr[k], visited):
                coordinate = (j + colNbr[k],i + rowNbr[k])
                if coordinate not in self.OBJECTS[self.COUNT-1]:
                    self.OBJECTS[self.COUNT-1].append(coordinate)
                self.DFS(i + rowNbr[k], j + colNbr[k], visited)

        # The main function that returns
        # count of islands in a given boolean
        # 2D matrix

    def countObjects(self):
        # Make a bool array to mark visited cells.
        # Initially all cells are unvisited
        visited = [[False for j in range(self.COL)] for i in range(self.ROW)]

        # Initialize count as 0 and traverse
        # through the all cells of
        # given matrix
        for i in range(self.ROW):
            for j in range(self.COL):
                # If a cell with value 1 is not visited yet,
                # then new island found
                if visited[i][j] == False and self.MAP[i][j] == self.IDX:
                    # Visit all cells in this island
                    # and increment island count
                    self.OBJECTS.append([(j,i)])
                    self.COUNT += 1
                    self.DFS(i, j, visited)

class ScanMap: #imap: 0-empty 1-robot 2,3,4-obstacle
            #irobot = [[(x,y)],[(x1,y1),(x2,y2),(x3,y3)......],[],[]....]
    def __init__(self, imap, irobots):
        self.MAP = imap
        self.ROBOTS = irobots
        self.ROW = len(self.MAP) - 1  # int
        self.COL = len(self.MAP[0]) - 1  # int
        self.KNOWN_MAP = [[-1 for j in range(self.COL + 1)] for i in range(self.ROW + 1)]
        self.SUS_POINTS = [[False for j in range(self.COL + 1)] for i in range(self.ROW + 1)]
        self.FINISHED_SCANING = False
        self.FINISHED_SUS_SCANING = False
        self.LAST_KNOWN_MAP = []
        self.FillKnownMap()
        self.PATHS = []
        self.LAST_DEST = []
        self.LAST_MOVE = []
        self.FORBIDDEN_DEST_TO_ROBOT = []


    def FillKnownMap(self):
        self.LAST_KNOWN_MAP.clear()
        self.LAST_KNOWN_MAP = copy.deepcopy(self.KNOWN_MAP)

        for y in range(self.ROW+1):
            for x in range(self.COL+1):
             if self.KNOWN_MAP[y][x] == 1:
                 self.KNOWN_MAP[y][x] = -1

        for robot in self.ROBOTS:
            for coordonate in robot:
                self.KNOWN_MAP[coordonate[1]][coordonate[0]] = 1

        for robot in self.ROBOTS:
            rowNbr = [-1, -1, -1, 0, 0, 1, 1, 1]
            colNbr = [-1, 0, 1, -1, 1, -1, 0, 1]
            for a in range(8):
                direction = (colNbr[a],rowNbr[a])
                self.ScanRobotView(robot, direction)

        self.TrackSusPoints()


    def ScanRobotView(self, robot, direction):
        for coordinate in robot:
            x = coordinate[0] + direction[0]
            y = coordinate[1] + direction[1]
            while (y >= 0 and y < self.ROW+1 and x >= 0 and x < self.COL+1):
                if self.MAP[y][x] == 0:
                    self.KNOWN_MAP[y][x] = 0
                elif self.MAP[y][x] == 1:
                    self.KNOWN_MAP[y][x] = 1
                    break
                else:
                    self.KNOWN_MAP[y][x] = 2
                    break
                x = x + direction[0]
                y = y + direction[1]

    def SusPointNearRobot(self, x , y):
        for robot in self.ROBOTS:
            for coo in robot:
                x_r = coo[0]
                y_r = coo[1]
                rowNbr = [-1, -1, -1, 0, 0, 1, 1, 1]
                colNbr = [-1, 0, 1, -1, 1, -1, 0, 1]
                for a in range(8):
                    direction = (colNbr[a], rowNbr[a])
                    if x == x_r + direction[0] and y == y_r + direction[1]:
                        return True
        return False

    def TrackSusPoints(self):
        for y in range(self.ROW + 1):
            for x in range(self.COL + 1):
                if self.KNOWN_MAP[y][x] == 0 and self.LAST_KNOWN_MAP[y][x] == 2:
                    add_it_to_sus = not self.SusPointNearRobot(x , y)
                    if add_it_to_sus:
                        self.SUS_POINTS[y][x] = not self.SUS_POINTS[y][x]
                elif self.KNOWN_MAP[y][x] == 2 and self.LAST_KNOWN_MAP[y][x] == 0:
                    add_it_to_sus = not self.SusPointNearRobot(x, y)
                    if add_it_to_sus:
                        self.SUS_POINTS[y][x] = not self.SUS_POINTS[y][x]
                if self.SUS_POINTS[y][x] and self.KNOWN_MAP[y][x] == 1:
                    self.SUS_POINTS[y][x] = not self.SUS_POINTS[y][x]
                elif self.SUS_POINTS[y][x] and self.SusPointNearRobot(x , y):
                    self.SUS_POINTS[y][x] = not self.SUS_POINTS[y][x]

    def RobotAdvance(self):
        if len(self.LAST_DEST) != 0:
            for move in self.LAST_MOVE:
                if not move[1]:
                    for dest in self.LAST_DEST:
                        if dest[0] == move[0]:
                            self.FORBIDDEN_DEST_TO_ROBOT.append((move[0],dest[1]))
                            break
        if not self.FINISHED_SCANING: # static obs
            occ_robots = []
            dest_to_robot = []
            idx_and_distance_arr = []
            for y in range(self.ROW + 1):
                for x in range(self.COL + 1):
                    if self.KNOWN_MAP[y][x] == -1:
                        dist = []
                        for robot in self.ROBOTS:
                            dist.append(math.dist([x,y],[robot[0][0],robot[0][1]]))
                        i=0
                        for distance in dist:
                            idx_and_distance_arr.append((i , distance))
                            i = i + 1
                        idx_and_distance_sorted = sorted(idx_and_distance_arr, key=lambda x: x[1])
                        pass_this_dest = False
                        min_dist_between_dests = (max([self.ROW,self.COL]) / (max([self.ROW,self.COL])/10)) * 2
                        for dest in dest_to_robot:
                            if math.dist([x,y],[dest[1][0],dest[1][1]]) < min_dist_between_dests:
                                pass_this_dest = True
                        if not pass_this_dest:
                            for robot_and_distance in idx_and_distance_sorted:
                                if robot_and_distance[0] not in occ_robots:
                                    if (robot_and_distance[0],(x,y)) not in self.FORBIDDEN_DEST_TO_ROBOT:
                                        occ_robots.append(robot_and_distance[0])
                                        dest_to_robot.append((robot_and_distance[0],(x,y)))
                                        break
                    if len(occ_robots) == len(self.ROBOTS):
                        return dest_to_robot
            return dest_to_robot

        else: # Go and track what is going on the sus points
            self.FINISHED_SUS_SCANING = self.FinishedSusScaning()
            occ_robots = []
            dest_to_robot = []
            idx_and_distance_arr = []
            for y in range(self.ROW + 1):
                for x in range(self.COL + 1):
                    if self.SUS_POINTS[y][x]:
                        dist = []
                        for robot in self.ROBOTS:
                            dist.append(math.dist([x, y], [robot[0][0], robot[0][1]]))
                        i = 0
                        for distance in dist:
                            idx_and_distance_arr.append((i, distance))
                            i = i + 1
                        idx_and_distance_sorted = sorted(idx_and_distance_arr, key=lambda x: x[1])
                        pass_this_dest = False
                        min_dist_between_dests = 5 #(max([self.ROW, self.COL]) / (max([self.ROW, self.COL]) / 10))
                        for dest in dest_to_robot:
                            if math.dist([x, y], [dest[1][0], dest[1][1]]) < min_dist_between_dests:
                                pass_this_dest = True
                        if not pass_this_dest:
                            for robot_and_distance in idx_and_distance_sorted:
                                if robot_and_distance[0] not in occ_robots:
                                    if (robot_and_distance[0], (x, y)) not in self.FORBIDDEN_DEST_TO_ROBOT:
                                        occ_robots.append(robot_and_distance[0])
                                        dest_to_robot.append((robot_and_distance[0], (x, y)))
                                        break
                    if len(occ_robots) == len(self.ROBOTS):
                        return dest_to_robot
            return dest_to_robot

    def BustAMove(self):  # Infected Mushroom - Classical Mushroom - Song #1
        if not self.FINISHED_SCANING:
            self.FINISHED_SCANING = self.FinishedYet()
        dest_arr = self.RobotAdvance()
        self.LAST_DEST.clear()
        self.LAST_DEST = sorted(dest_arr)
        self.PATHS.clear()
        for robot_dest in dest_arr:
            # Here you may choose any map.csv file wich contains:
            # 0's for empty space, 1 for target point, 2's for obstacles
            # and 3 for starting point
            map_for_waveplanner = []
            for i in range(self.ROW + 1):
                line = []
                for j in range(self.COL + 1):
                    line.append(0)
                map_for_waveplanner.append(line)
            for y in range(self.ROW + 1):
                for x in range(self.COL + 1):
                    if self.KNOWN_MAP[y][x] == 2 or self.KNOWN_MAP[y][x] == 1:
                        map_for_waveplanner[y][x] = 2
            for coo in self.ROBOTS[robot_dest[0]]:
                map_for_waveplanner[coo[1]][coo[0]] = 0
            coo = self.ROBOTS[robot_dest[0]]
            x = coo[0][0]
            y = coo[0][1]
            map_for_waveplanner[y][x] = 3
            if robot_dest[1][0] != x:
                map_for_waveplanner[robot_dest[1][1]][robot_dest[1][0]] = 1
            else:
                if robot_dest[1][0] + 1 <= self.COL:
                    map_for_waveplanner[robot_dest[1][1]][robot_dest[1][0] + 1] = 1
                else:
                    map_for_waveplanner[robot_dest[1][1]][robot_dest[1][0] - 1] = 1
            b2 = WavePlanner(map_for_waveplanner)
            if not b2.UNREACHABLE:
                path_arr = []
                last_pos = (x, y)
                for pos in b2.PATH:
                    tmoora = (pos[0] - last_pos[0], pos[1] - last_pos[1])
                    last_pos = pos
                    path_arr.append(tmoora)
                self.PATHS.append([robot_dest[0], path_arr])
            else:
                self.KNOWN_MAP[robot_dest[1][1]][robot_dest[1][0]] = 2

    def FinishedYet(self):
        for y in range(self.ROW + 1):
            for x in range(self.COL + 1):
                if self.KNOWN_MAP[y][x] == -1:
                    return False
        return True

    def FinishedSusScaning(self):
        for y in range(self.ROW + 1):
            for x in range(self.COL + 1):
                if self.SUS_POINTS[y][x]:
                    return not self.SUS_POINTS[y][x]
        return True

class WorldUpdater:

    def __init__(self, imap, irobots, ipaths, cycle):
        self.MAP = imap
        self.ROW = len(self.MAP) - 1  # int
        self.COL = len(self.MAP[0]) - 1  # int
        self.ROBOTS = irobots
        self.ROBOTS_DID_MOVE = []
        self.CYCLE = cycle
        self.PATHS = ipaths # [[idx , [(x0 , y0),(x1 , y1),...] ],...]
        self.TYPE3DIRECTION = 1
        self.TYPE4DIRECTION = -1

    def MoveEnsurance(self, x_move, y_move, robot_coo):
        for coo in robot_coo:
            if coo[1] + y_move < 0 or coo[1] + y_move > self.ROW:
                return False
            elif coo[0] + x_move < 0 or coo[0] + x_move > self.COL:
                return False
            elif self.MAP[coo[1] + y_move][coo[0] + x_move] == 2 or self.MAP[coo[1] + y_move][coo[0] + x_move] == 3 or self.MAP[coo[1] + y_move][coo[0] + x_move] == 4:
                return False
            elif self.MAP[coo[1] + y_move][coo[0] + x_move] == 1 and (coo[0] + x_move , coo[1] + y_move) not in robot_coo:
                return False

        return True

    def GoRobotsGo(self):
        self.ROBOTS_DID_MOVE.clear()
        for path in self.PATHS:
            robot = self.ROBOTS[path[0]]
            y_move = path[1][0][1]
            x_move = path[1][0][0]
            if self.MoveEnsurance(x_move, y_move, robot):
                self.ROBOTS_DID_MOVE.append((path[0], True))
                prv_coo = []
                for coo in robot:
                    self.MAP[coo[1]][coo[0]] = 0
                    prv_coo.append(coo)
                for coo in prv_coo:
                    self.ROBOTS[path[0]].remove(coo)
                    new_x = coo[0] + x_move
                    new_y = coo[1] + y_move
                    self.ROBOTS[path[0]].append((new_x,new_y))
                    self.MAP[new_y][new_x] = 1

            else:
                self.ROBOTS_DID_MOVE.append((path[0], False))

    def ObsMovementEnsure(self, obs_coo, type):
        x = obs_coo[0]
        y = obs_coo[1]
        if type == 3:
            if x + self.TYPE3DIRECTION > self.COL or x + self.TYPE3DIRECTION < 0:
                return False
            if self.MAP[y][x+self.TYPE3DIRECTION] == 1 or self.MAP[y][x+self.TYPE3DIRECTION] == 2 or self.MAP[y][x+self.TYPE3DIRECTION] == 4:
                return False
            return True
        elif type ==4:
            if y+self.TYPE4DIRECTION > self.ROW or y+self.TYPE4DIRECTION<0:
                return False
            if self.MAP[y+self.TYPE4DIRECTION][x] == 1 or self.MAP[y+self.TYPE4DIRECTION][x] == 2 or self.MAP[y+self.TYPE4DIRECTION][x] == 3:
                return False
            return True

    def MovingObs(self):
        type3 = DefineObjects(self.MAP, 3)
        type4 = DefineObjects(self.MAP, 4)
        dont_move = False
        if self.CYCLE % 2 == 0:
            for type3obj in type3.OBJECTS:
                for coo in type3obj:
                    if coo[0] + self.TYPE3DIRECTION < 0 or coo[0] + self.TYPE3DIRECTION > self.COL \
                            or not (self.MAP[coo[1]][coo[0] + self.TYPE3DIRECTION] == 0
                                    or self.MAP[coo[1]][coo[0] + self.TYPE3DIRECTION] == 3):
                        self.TYPE3DIRECTION = self.TYPE3DIRECTION * -1
                        break
                for coo in type3obj:
                    if not self.ObsMovementEnsure(coo,3):
                        dont_move = True
                        break
                if not dont_move:
                    for coo in type3obj:
                        self.MAP[coo[1]][coo[0]] = 0
                        self.MAP[coo[1]][coo[0] + self.TYPE3DIRECTION] = 3

        dont_move = False
        if self.CYCLE % 4 == 0:
            for type4obj in type4.OBJECTS:
                for coo in type4obj:
                    if coo[1] + self.TYPE4DIRECTION < 0 or coo[1] + self.TYPE4DIRECTION > self.ROW \
                            or not (self.MAP[coo[1] + self.TYPE4DIRECTION][coo[0]] == 0
                                    or self.MAP[coo[1] + self.TYPE4DIRECTION][coo[0]] == 4):
                        self.TYPE4DIRECTION = self.TYPE4DIRECTION * -1
                        break
                for coo in type4obj:
                    if not self.ObsMovementEnsure(coo, 4):
                        dont_move = True
                        break
                if not dont_move:
                    for coo in type4obj:
                        self.MAP[coo[1]][coo[0]] = 0
                        self.MAP[coo[1] + self.TYPE4DIRECTION][coo[0]] = 4

mapa = pd.read_csv("Map.csv", header=None).values
r = DefineObjects(mapa, 1)
scanner = ScanMap(mapa , r.OBJECTS)
plt.style.use('seaborn-pastel')
fig = plt.figure()
ax = plt.axes(xlim=(0, (scanner.COL + 1) * 10), ylim=(0, (scanner.ROW + 1) * 10))
counter = 0
im = plt.imshow(scanner.KNOWN_MAP, zorder=0, extent=[0.0, (scanner.COL + 1) * 10, 0.0, (scanner.ROW + 1) * 10], origin='upper', animated = True)
update = WorldUpdater(mapa, scanner.ROBOTS, scanner.PATHS, counter)
def updatefig(*args):
    global counter
    global scanner
    if counter % 20 == 0:
        scanner.FORBIDDEN_DEST_TO_ROBOT.clear()
    counter = counter + 1
    scanner.FillKnownMap()
    scanner.BustAMove()
    update.PATHS = scanner.PATHS
    update.CYCLE = counter
    update.ROBOTS = scanner.ROBOTS
    update.GoRobotsGo()
    scanner.LAST_MOVE = sorted(update.ROBOTS_DID_MOVE)
    scanner.ROBOTS = update.ROBOTS
    update.MovingObs()
    scanner.MAP = update.MAP
    im.set_array(scanner.KNOWN_MAP)
    return im

ani = animation.FuncAnimation(fig, updatefig, interval=50)
plt.show()
