import cv2
from os.path import exists

# IDEA
# 1. FIND BOUNDRY OF OBSTACLES FIRST. FOR EACH OBSTACLES, PUT THEM INTO DIFFERENT LIST.
# 2. ITERATION FOR EACH LIST, FIND THE POINT CLOSEST TO THIS POINT WHICH IS NOT IN ITS OWN LIST, FIND THE MIDPOINT AND PUT IT INTO A GVG MAP LIST
# 3. PLOT THE GVG MAP
# 4. DEFINE START POINT AND END POINT, TRY TO FIND THE PATH

def main():
    # read the image
    pathName = input("Please Enter the path of the image(Image name if the code and image are in the same folder. Otherwise, please specify the absolute path): ")
    while not exists(pathName):
        pathName = input("Cannot Locate the file. Please Enter the path of the image again(Image name if the code and image are in the same folder. Otherwise, please specify the absolute path): ")
    image = cv2.imread(pathName)
    height, width, channels = image.shape
    obstaclePoints = [] 
    imageDomain = []
    for i in range(height):
        for j in range(width):
            r = image[i,j][0]
            g = image[i,j][1]
            b = image[i,j][2]
            imageDomain.append((i,j))
            if r == 0 and g == 0 and b == 0:
                obstaclePoints.append((i,j))

    print("The Dimension of your file is: " + str(width) + " X " + str(height) + " pixels.")
    print("Let's set up your Start Point.")
    inObstacle = True
    initial = True
    startX = 0
    startY = 0 
    while (inObstacle):
        if not initial:
            print("Your Start Point is In Obstacle.")
            print("Let's do this again.")
        startX = int(input("Please enter X coordinate of your start point. The range of your X coordinate is (" + "0," + str(width) + "): "))
        while startX < 0 or startX >= width:
            startX = int(input("Coordinate X out of the image. Please enter X coordinate of your start point. The range of your X coordinate is (" + "0," + str(width) + "): "))
        
        startY = int(input("Please enter Y coordinate of your start point. The range of your Y coordinate is (" + "0," + str(height) + "): "))
        while startY < 0 or startY >= height:
            startY = int(input("Coordinate Y out of the image. Please enter Y coordinate of your start point. The range of your Y coordinate is (" + "0," + str(height) + "): "))
        
        if (startY, startX) not in obstaclePoints:
            inObstacle = False
        initial = False
    startPoint = [startX, startY]
    print("Start Point: " + str(startPoint))
    
    inObstacle = True
    initial = True
    print("Let's set up your Goal Point.")
    while (inObstacle):
        if not initial:
            print("Your Goal Point is In Obstacle.")
            print("Let's do this again.")
        goalX = int(input("Please enter X coordinate of your goal point. The range of your X coordinate is (" + "0," + str(width) + "): "))
        while goalX < 0 or goalX >= width:
            goalX = int(input("Coordinate X out of the image. Please enter X coordinate of your goal point. The range of your X coordinate is (" + "0," + str(width) + "): "))
        goalY = int(input("Please enter Y coordinate of your goal point. The range of your Y coordinate is (" + "0," + str(height) + "): "))
        while goalY < 0 or goalY >= height:
            goalY = int(input("Coordinate Y out of the image. Please enter Y coordinate of your goal point. The range of your Y coordinate is (" + "0," + str(height) + "): "))
        if (goalY, goalX) not in obstaclePoints:
            inObstacle = False
        initial = False
    goalPoint = [goalX, goalY]
    print("Goal Point: " + str(goalPoint))

    _, _, red = cv2.split(image)
    logPoints = [] 

    print("Looking for vertices of the contours")
    # NOW TRY TO DETECT ALL VERTEX OF THOSE OBSTACLES. THESE VERTICES WILL BE USED TO CALCULATE THE DISTANCE FROM POINTS IN FREE SPACE TO THEM 
    vertexList = []
    # REF: https://www.geeksforgeeks.org/find-co-ordinates-of-contours-using-opencv-python/
    contours, _ = cv2.findContours(image=red, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    for cnt in contours :
        contourVertices = []
        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
        n = approx.ravel() 
        i = 0
        for j in n :
            if(i % 2 == 0):
                x = n[i]
                y = n[i + 1]
                vertexCoordinate = (x,y)
                contourVertices.append(vertexCoordinate)
            i = i + 1
        # ADD LAST VERTEX AS WELL SINCE LINE FROM LAST VERTEX TO FIRST VERTEX ALSO NEED TO BE COUNTED
        contourVertices.append(contourVertices[0])
        vertexList.append(contourVertices)
    # NOW WE TRY TO DETECT THE DISTANCE FROM EACH PIXEL TO EDGES OF OBSTACLES

    diagonolDistance = (width ** 2 + height ** 2) ** 0.5
    print("Found All Vertices on the Contours. Calculating distance from freespace pixels to line segments.")
    gvgPoints = []
    count = 0 
    for i in range(height):
        for j in range(width):
            currentPosition = (j,i)
            # print("Processing point " + str(count))
            if ((i,j) in obstaclePoints):
                continue
            lstDistance = []
            for onePolygonVertices in vertexList:
                tempMinimumD = getDistanceToPolygon(currentPosition, onePolygonVertices, diagonolDistance)
                lstDistance.append(tempMinimumD)
            if midPointOfObject(lstDistance):
                gvgPoints.append((i,j))
                logPoints.append(((i,j), lstDistance))
            count += 1
    image_copy = image.copy()
    cannotFindPath = False
    print("FOUND ALL POINTS ON GVG MAP. NOW FINDING PATH TO GVG MAP FROM POINTS.")
    closestPointToStartPoint = findClosestPoint(startPoint, gvgPoints, diagonolDistance)
    closestPointToGoalPoint = findClosestPoint(goalPoint, gvgPoints, diagonolDistance)
    print("FOUND CLOSEST POINT ON GVG MAP TO START & GOAL POINT")
    
    if closestPointToStartPoint == None:
        cannotFindPath = True
    elif closestPointToGoalPoint == None:
        cannotFindPath = True
    gvgPathList = []
    startPathList = []
    endPathList = []
    print("LOOKING FOR PATH FROM START POINT TO GVG MAP")
    startPathList = findPath(startPoint, closestPointToStartPoint, imageDomain,  obstaclePoints)
    print("LOOKING FOR PATH FROM GOAL POINT TO GVG MAP")
    endPathList = findPath(goalPoint, closestPointToGoalPoint, imageDomain, obstaclePoints)
    print("LOOKING FOR PATH ON GVG MAP")
    gvgPathList = findPath(closestPointToStartPoint, closestPointToGoalPoint, gvgPoints, obstaclePoints)

    if startPathList == None or endPathList == None or gvgPathList == None:
        cannotFindPath = True
    if cannotFindPath:
        print("CANNOT FIND ANY PATH")

    if cannotFindPath:
        print("NO PATH FOUND. PLOTTING GVG POINTS ONLY.")
    else:
        print("PATH FOUND. NOW PLOTING GVG MAP AND PATH.")
    # PLOTING GVG MAP
    for i in range(height):
        for j in range(width):
            # GVG MAP - PINK COLOR
            if (i,j) in gvgPoints:
                image_copy[i,j] = (102, 178, 255)
            # PLANNED PATH - BLUE COLOR
            if startPathList != None and (i,j) in startPathList:
                image_copy[i,j] = (255, 0, 0)
            if endPathList != None and (i,j) in endPathList:
                image_copy[i,j] = (255, 0, 0)
            if gvgPathList != None and (i,j) in gvgPathList:
                image_copy[i,j] = (255, 0, 0)
    if cannotFindPath:
        cv2.imshow('GVG MAP', image_copy)
    else:
        cv2.imshow('GVG MAP AND PATH FOUND', image_copy)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def findPath(startPoint, endPoint, coordinatDomain, obstacleList):
    path = astar(startPoint, endPoint, coordinatDomain, obstacleList)
    return path

def findClosestPoint(startPoint, gvgMapPoints, defaultLogestDistance):
    distance = defaultLogestDistance
    resultPoint = None
    for (y,x) in gvgMapPoints:
        tempD = ((y - startPoint[1]) ** 2 + (x - startPoint[0]) ** 2) ** 0.5
        if tempD < distance:
            distance = tempD
            resultPoint = (x,y)
    return resultPoint

def getDistanceToPolygon(currentPoint, polygonVerticies, diagonolDistance):
    minDSquare = diagonolDistance
    index = 0
    # LOOP FROM START VERTEX TO LAST VERTEX
    while index < len(polygonVerticies) -1:
        vertexCurrent = polygonVerticies[index]
        vertexNext = polygonVerticies[index+1]
        tempD = findDistanceFromPointToLine(currentPoint[0], currentPoint[1], vertexCurrent[0], vertexCurrent[1], vertexNext[0], vertexNext[1])
        if (tempD < minDSquare):
            minDSquare = tempD
        index += 1
    return minDSquare

def midPointOfObject(list):
    my_formatted_list = [ round(elem) for elem in list ]
    listWithNoise = []
    for current in my_formatted_list:
        previous = current - 1
        next = current + 1
        if previous < 0:
            listWithNoise.append([current, next])
        else:
            listWithNoise.append([previous, current, next]) 

    existDic = {}
    for tempList in listWithNoise:
        for element in tempList:
            if element in existDic:
                existDic[element] += 1
            else:
                existDic[element] = 1
    listkeys= existDic.keys()
    for key in listkeys:
        # MID POINTs
        if existDic[key] > 1:
            if key == min(listkeys) or key == (min(listkeys) + 1) or key == (min(listkeys) + 2):
                return True
    return False

def findDistanceFromPointToLine (px, py, x1, y1, x2, y2):
    # REF: https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
    hypotenuse = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))
    u = u1 / (hypotenuse * hypotenuse)
    # WHEN THIS IS OBTUSE TRIANGLE
    if (u < 0) or (u > 1):
        side1 = ((px - x1) ** 2 + (py - y1) ** 2 ) ** 0.5
        side2 = ((px - x2) ** 2 + (py - y2) ** 2 ) ** 0.5
        if side1 > side2:
            DistancePointLine = side2
        else:
            DistancePointLine = side1
    # ACUTE TRIANGLE
    else:
        side1 = x1 + u * (x2 - x1)
        side2 = y1 + u * (y2 - y1)
        DistancePointLine = ((px - side1) ** 2 + (py - side2) ** 2) ** 0.5
    return DistancePointLine

# NODE CLASS USED FOR A* PATH PLANNING
class Node():
    # REF: https://towardsdatascience.com/a-star-a-search-algorithm-eb495fb156bb
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def astar(start, end, pointsDomain, obstacleList):
    startTranspose = (start[1], start[0])
    # TRANSPOSE OF START AND END NODE SINCE WE HAVE (Y-AXIS, X-AXIS)
    start_node = Node(None, startTranspose)
    start_node.g = start_node.h = start_node.f = 0
    endTranspose = (end[1], end[0])
    end_node = Node(None, endTranspose)
    end_node.g = end_node.h = end_node.f = 0
    # OPEN LIST, CLOSE LIST, AND VISITED LIST.
    open_list = []
    closed_list = []
    visited_list = []
    # ADDED START NODE TO THE OPEN LIST
    open_list.append(start_node)
    # MAX STEPS THAT WILL TERMINATE THE PROGRAM - THE AMOUNT OF THE DOMAIN POINTS
    maxStep = len(pointsDomain)
    step = 0
    while len(open_list) > 0:
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        open_list.pop(current_index)
        closed_list.append(current_node)
        # FOUND THE PATH
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            # RETURN THE PATH
            return path[::-1]
        children = []
        # SCAN THE PIXEL CAN BE ACCESSES FROM CURRENT NODE 
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            # NODE MUST BE IN THE DOMAIN LIST
            if node_position not in pointsDomain:
                continue
            # NODE SHOULD NOT HIT OBSTACLES
            if node_position in obstacleList:
                continue
            # NODE SHOULD NOT BE VISITED. IN CASE THE LOCAL PITFALL HAPPENS
            if node_position in visited_list:
                continue
            visited_list.append(node_position)
            new_node = Node(current_node, node_position)
            children.append(new_node)
        # Loop through children
        for child in children:
            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue
            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h
            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue
            # Add the child to the open list
            open_list.append(child)
        step += 1
        # WE CAN'T FIND THE PATH
        if step >= maxStep:
            return None

main()

