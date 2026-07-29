'''
In this method , we are trying to find offline path first using A* algorithm, then we use Artificial potential method
to reach local goals on A* planned path.
Suppose, a1, a2, a3, .......  an be path we get from A*, then to potential field we give following as consecutive goals:
a(i), a(i+4), a(i+8), a(i+12), ..... a(i+n)

Hence this way we are using power of both A-star and potential field  :)
'''

import cv2
import numpy as np
from time import sleep
import copy
import glob
import math
import time
import queue as Q


def printx(x):
    #print x
    pass
'''
function definition from A-star
start
'''
class pixel1(object):
    def __init__(self, penalty, pointx, pointy, parent, h): # parent is that pixel from which this current pixel is generated
        self.penalty = penalty
        self.pointx = int(pointx)
        self.pointy = int(pointy)
        self.parent = parent
        self.h = h #heuristic

    def __lt__(self, other): # comparable which will return self.penalty<other.penalty
        return (self.penalty + self.h) < (other.penalty + other.h)

def feasibility(nx, ny, img):  # function to check if pixel lies in obstacle
    if img[nx, ny, 0] == 255:
        return False
    else:
        return True

def penalty1(clearance):
   alpha = 10000
   sigma_sqr = 1000
   return alpha*math.exp((-1)*clearance*clearance/sigma_sqr)

def cost(ox, oy, nx, ny, penalty, clearance): #ox, oy:- old points  nx, ny :- new points
    return penalty + math.sqrt((ox-nx)*(ox-nx)+ (oy-ny)*(oy-ny))*(1+penalty1(clearance))

def heuristic(nx, ny,dx, dy): #ox, oy:- old points  nx, ny :- new points
    return math.sqrt((nx-dx)*(nx-dx)+ (ny-dy)*(ny-dy))

def check_boundaries1(ex, ey, nx, ny): #ex, ey :- end points of frame
    if nx > -1 and ny > -1 and nx < ex and ny < ey:
        return True
    else:
        return False

def bfs(arr, sx, sy, dx, dy, final_contours): # sx, sy :- source coordinates  dx, dy :- destination coordinates
    q = Q.PriorityQueue()
    temp1 = True
    temp2 = True

    for cnt in final_contours:
        if cv2.pointPolygonTest(cnt, (sx, sy), False) > -1:
            temp1 = False

    for cnt in final_contours:
        if cv2.pointPolygonTest(cnt, (dx, dy), False) > -1:
            temp2 = False

    if temp1 == False or temp2 == False:
        return []

    actions = [[0, 1], [0, -1], [1, 0], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]]
    solution = []
    ex, ey, ez = arr.shape
    #visit = [[False for x in range(ey)] for x in range(ex)]
    dist = [[10000 for x in range(ey)] for x in range(ex)]
    distplusHeuristic = [[10000 for x in range(ey)] for x in range(ex)]

    q.put(pixel1(0, sx, sy, None, heuristic(sx, sy, dx, dy)))
    dist[sx][sy] = 0
    distplusHeuristic[sx][sy] = dist[sx][sy]+heuristic(sx, sy, dx, dy)
    s = time.perf_counter()
    cnt = 0
    cntq = 0
    while not q.empty():
        p = q.get()
        x = int(p.pointx)
        y = int(p.pointy)
        pen = p.penalty
        h = p.h
        cnt = cnt+1
        if dist[x][y] < pen:
            continue
        if x == dx and y == dy:
            while p is not None:
                solution.append([p.pointx, p.pointy])
                p = p.parent
            #print 'time : ', time.perf_counter()-s
            #print cnt, cntq
            return solution

        for i in range(len(actions)):
            nx = int(actions[i][0] + x)
            ny = int(actions[i][1] + y)
            if check_boundaries1(ex, ey, nx, ny) == True:
                #if arr.item(nx, ny, 0) == 0 and arr.item(nx, ny, 1) == 0 and arr.item(nx, ny, 2) == 0:
                    pen = dist[x][y]
                    pen_new = cost(x, y, nx, ny, pen, int(arr[nx][ny][0]))
                    h_new = heuristic(nx, ny, dx, dy)
                    if dist[nx][ny] > pen_new :
                        dist[nx][ny]  = pen_new
                        nx = int(nx)
                        ny = int(ny)
                    if distplusHeuristic[nx][ny] > dist[nx][ny]+h_new :
                        distplusHeuristic[nx][ny] = dist[nx][ny] + h_new
                        cntq = cntq+1
                        q.put(pixel1(pen_new, nx, ny, p, h_new))
    #print 'time : ', time.perf_counter()-s
    return []

'''
function definition from Clearance-feasibility
end
'''

'''
function definition from Clearance-feasibility
start
'''

class pixel(object):
    def __init__(self, penalty, pointx, pointy): # parent is that pixel from which this current pixel is generated
        self.penalty = penalty
        self.pointx = int(pointx)
        self.pointy = int(pointy)

    def __lt__(self, other): # comparable which will return self.penalty<other.penalty
        return self.penalty < other.penalty

images = glob.glob('*.jpg')

def penalty(ox, oy, nx, ny, penalty): #ox, oy:- old points  nx, ny :- new points
    return penalty + math.sqrt((ox-nx)*(ox-nx)+ (oy-ny)*(oy-ny))

def check_boundaries(ex, ey, nx, ny): #ex, ey :- end points of frame
    if nx > -1 and ny > -1 and nx < ex and ny < ey:
        return True
    else:
        return False

def fill_clearance(arr,cmax,  final_contours): # sx, sy :- source coordinates  dx, dy :- destination coordinates
    q = Q.PriorityQueue()

    actions = [[0, 1], [0, -1], [1, 0], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]]
    ex, ey, ez = arr.shape
    #print ex, ey, ez
    min_cost = [[100000 for x in range(ey)] for x in range(ex)]
    for cnt in final_contours:
        for pts in cnt:
            q.put(pixel(0, pts[0, 1], pts[0, 0]))
    cnt = 0
    cntq = 0
    while not q.empty():
        p = q.get()
        x = int(p.pointx)
        y = int(p.pointy)
        pen = p.penalty
        if p.penalty > cmax:
            continue
        if min_cost[x][y] <= p.penalty:
            continue
        min_cost[x][y] = p.penalty

        for i in range(len(actions)):
            nx = int(actions[i][0] + x)
            ny = int(actions[i][1] + y)
            if check_boundaries(ex, ey, nx, ny) == True:
                if arr.item(nx, ny, 0) == 0 and arr.item(nx, ny, 1) == 0 and arr.item(nx, ny, 2) == 0:
                    if min_cost[nx][ny] > penalty(x, y, nx, ny, pen):
                        q.put(pixel(penalty(x,y,nx,ny,pen), nx, ny))
    return min_cost
'''
function definition from Clearance-feasibility
end
'''



def check_obstacles(arr, ansx, ansy):  #function to check whether a given point is on obstacle or not
    if arr[int(ansx)][int(ansy)][0] == 255:
        return True
    else:
        return False

def feasible(arr, x, y):  #function to check if a point is feasible or not
    ex, ey, ez = arr.shape
    x = int(x)
    y = int(y)

    if check_boundaries(ex, ey, x, y):
        return not check_obstacles(arr, x, y)
    else:
        return False

def dist(sx, sy, x, y, theta, arr, q_star):  #distance of obstacle in direction theta in radians
    ansx = sx
    ansy = sy
    flag = True
    count = 1
    while True:
        if count > q_star:
            return (-1, -1)
        ansx = sx + count*math.sin(theta)
        ansy = sy + count*math.cos(theta)

        if check_boundaries(x, y, ansx, ansy) == False:
            break
        else:
            if check_obstacles(arr, ansx, ansy) == True:
                break
        count += 1

    return (ansx-sx,ansy- sy)


def obstacle_force(arr, sx, sy, q_star): #sx,sy :- source    dx, dy:- destination    q-star:- threshold distance of obstacles
    forcex = 0
    forcey = 0
    neta = 3000
    x, y , z= arr.shape
    for i in range(8):
        (ox,oy) = dist(sx, sy, x, y, i*math.pi/4, arr, q_star)
        theta = i*math.pi/4
        ox = math.fabs(ox)
        oy = math.fabs(oy)

        d = math.hypot(ox,oy)
        fx = 0
        fy = 0
        if ox == -1 or oy == -1:
            fx = 0
            fy = 0
        else:
            if d == 0:
                d = 1
            f = (neta*(1.0/q_star- 1.0/d))/(d*d)
            fx = f*math.sin(theta)
            fy = f*math.cos(theta)

        forcex += fx
        forcey += fy

    return (forcex, forcey)

def goal_force(arr, sx, sy, dx, dy, d_star): # sx, sy :- source  dx, dy:- destination   d_star:- threshold distance from goal
    forcex = 0
    forcey = 0
    tau = 1  #constant
    #printx('10')
    d = math.sqrt((dx-sx)*(dx-sx) + (dy-sy)*(dy-sy))
    if d > d_star:
        forcex += ((d_star*tau*math.sin(math.atan2(dx-sx, dy-sy))))
        forcey += ((d_star*tau*math.cos(math.atan2(dx-sx, dy-sy))))

    else:
        forcex += ((dx-sx)*tau)
        forcey += ((dy-sy)*tau)

    #printx('11')
    return (forcex, forcey)


def path_planning(arr, sx1, sy1, dx, dy, theta):
    '''

    :param arr: input map
    :param sx1: source x
    :param sy1: source y
    :param dx: destination x
    :param dy: destination y
    :return: path
    '''

    #Parameters Declaration

    flx = 10000  #maximum total force in x
    fly = 10000  #maximum total force in y
    v = 4 #velocity magnitude
    t = 1 #time lapse
    #theta = 0 #initial angle
    x,y,z = arr.shape
    theta_const = math.pi*60/180  #maximum allowed turn angle
    q_star = 50000
    d_star = 20000

    if arr[sx1][sy1][0] == 255 or arr[dx][dy][0] == 255:
        return []
    sx = sx1
    sy = sy1

    sol = []
    sol.append((sx, sy))


    sx += int(v*math.sin(theta))
    sy += int(v*math.cos(theta))
    sol.append((sx, sy))

    '''
        if Q and P are two vectors and @ is angle between them

        resultant ,R = (P^2 + R^2 + 2*P*Q cos @)^(1/2)

        resultant, theta = atan((Q*sin @)/(P+Q*cos @))
    '''

    #count  = 0
    while True:
        #count += 1
        (fx, fy) = obstacle_force(arr, sx, sy, q_star)
        (gx, gy) = goal_force(arr, sx, sy, dx, dy, d_star)

        tx = gx+fx
        ty = gy+fy
        if(tx < 0):
            tx = max(tx, -flx)
        else:
            tx = min(tx, flx)
        if(ty < 0):
            ty = max(ty, -fly)
        else:
            ty = min(ty, fly)
        theta1 = math.atan2(tx, ty)

        if arr[sx][sy][0] == 255:
            print(gx, gy, fx, fy)
            print('tx ', tx, ' ty ', ty, 'sx ', sx, ' sy ', sy)
            print(theta1*180/math.pi, theta*180/math.pi)
            sleep(10)

        P = v
        angle = theta1-theta  #angle between velocity and force vector

        Q = math.sqrt(tx*tx + ty*ty)

        theta2 = math.atan2((Q*math.sin(angle)),((P + Q*math.cos(angle))))   #resultant angle with velocity

        if theta2 < 0:
            theta2 = max(theta2, -theta_const)
        else:
            theta2 = min(theta2, theta_const)

        theta += theta2

        theta = (theta + 2*math.pi)%(2*math.pi)

        sx = sx + v*math.sin(theta)
        sy = sy + v*math.cos(theta)
        sx = int(sx)
        sy = int(sy)

        if not check_boundaries(x, y, sx, sy):
            print('out of boundaries' , sx, sy)
            return sol

        sol.append((sx, sy))

        if sx < dx+ 2 and sx > dx - 2 and sy < dy+2 and sy > dy-2:
            break

    return sol


def final_path(sol, arr):
    l = len(sol)
    print(l)
    div = 250

    start = 0
    end = div
    solution = []
    theta = 0

    while start < l-1:
        print(sol[start][0], sol[start][1], sol[end][0], sol[end][1])
        ret = path_planning(arr, sol[start][0], sol[start][1], sol[end][0], sol[end][1], theta)
        for i in ret:
            solution.append(i)
        l1 = len(ret)
        if l1 > 2:
            x1 = ret[l1-1][0]
            x2 = ret[l1-2][0]
            y1 = ret[l1-1][1]
            y2 = ret[l1-2][1]
            theta = math.atan2(x1-x2, y1-y2)
        start = end
        end += div
        if end > l-1:
            end = l-1

    return solution


def main():
    counter = 1
    for im in images:

        img = cv2.imread(im)

        cimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        img2 = cv2.medianBlur(cimg,13)

        ret,thresh1 = cv2.threshold(cimg,100,120,cv2.THRESH_BINARY)
        t2 = copy.copy(thresh1)

        x, y  = thresh1.shape
        arr = np.zeros((x, y, 3), np.uint8)
        arr1 = np.zeros((x, y, 3), np.uint8)
        final_contours= []
        _res = cv2.findContours(t2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours = _res[0] if len(_res) == 2 else _res[1]
        for i in range(len(contours)):
            cnt = contours[i]
            if cv2.contourArea(cnt) > 1000 and cv2.contourArea(cnt) < 15000 :
                cv2.drawContours(img, [cnt],-1, [0, 255, 255])
                cv2.fillConvexPoly(arr, cnt, [255, 255, 255])
                cv2.fillConvexPoly(arr1, cnt, [255, 255, 255])
                final_contours.append(cnt)
        cmax = 50
        start = time.perf_counter()
        min_cost = fill_clearance(arr,cmax, final_contours)
        print('time: ',  time.perf_counter()-start)
        '''
        for i in range(x):
            for j in range(y):
                if min_cost[i][j] == 100000:
                    min_cost[i][j] = 0;
        '''

        for i in range(x):
            for j in range(y):
                pix_val = int(5*min_cost[i][j])
                if(min_cost[i][j] > 10000):
                    pix_val = 255
                arr[i, j] = (pix_val, pix_val, pix_val)
        for cnt in final_contours:
            cv2.fillConvexPoly(arr, cnt, [0, 0, 0])

        '''
        Code from A-star.py
        '''
        sx = 20 # input("Enter source and destination Coordinates")
        sy = 20  # input()
        dx = 500   # input()
        dy = 1000  # input()

        sol = bfs(arr, sx, sy, dx, dy, final_contours)
        solution = final_path(sol, arr1)

        if len(solution) == 0:
            print('No solution from source to destination')
        else:
            for i in range(len(solution)):
                start = (solution[i][1], solution[i][0])
                cv2.circle(arr,start, 1, [255, 255, 255])
                cv2.circle(img, start, 1, [255, 255, 255])

            for i in range(len(sol)):
                start = (sol[i][1], sol[i][0])
                cv2.circle(arr,start, 1, [255, 0, 0])
                cv2.circle(img, start, 1, [255, 0, 0])

        cv2.circle(arr, (sy, sx), 2, [0, 255, 0])
        cv2.circle(arr, (dy, dx), 2, [0, 255, 0])
        cv2.circle(img, (sy, sx), 2, [0, 255, 0])
        cv2.circle(img, (dy, dx), 2, [0, 255, 0])
        output = "output1/"+str(counter)
        output += ".jpg"
        cv2.imwrite(output, img)
        counter += 1
        cv2.imshow('image', img)
        cv2.imshow('arr', arr)
        cv2.waitKey(1)
        cv2.destroyAllWindows()
main()
