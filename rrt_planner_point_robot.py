import time
import random
import drawSample
import math
import _tkinter
import sys
import imageToRects

#display = drawSample.SelectRect(imfile=im2Small,keepcontrol=0,quitLabel="")

visualize = 1
prompt_before_next=1  # ask before re-running sonce solved
SMALLSTEP = 3 # what our "local planner" can handle.

XMAX=1800
YMAX=1000
G = [  [ 0 ]  , [] ]   # nodes, edges

s,obstacles = imageToRects.imageToRects(sys.argv[1])
sys.stdout.flush() #show the done message

XMAX = s[0]
YMAX = s[1]


# goal/target
tx = 900
ty = 300
# start
start_x = 10
start_y = 270

vertices = [ [start_x,start_y] ]

sigmax_for_randgen = XMAX/2.0
sigmay_for_randgen = YMAX/2.0

nodes=0
edges=1
maxvertex = 0

def drawGraph(G):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
        if len(vertices)!=1:
            canvas.polyline(  [vertices[i[0]], vertices[i[1]] ],  )


def genPoint(gauss):
    if gauss:
        x = random.gauss(tx, sigmax_for_randgen)
        y = random.gauss(ty, sigmay_for_randgen)
    else:
        x = random.randrange(XMAX)
        y = random.randrange(YMAX)
    return [x,y]

def genvertex():
    vertices.append( genPoint() )
    return len(vertices)-1

def pointToVertex(p):
    vertices.append( p )
    return len(vertices)-1

def pickvertex():
    return random.choice( range(len(vertices) ))

def lineFromPoints(p1,p2):
    line = []
    llsq = 0.0 # line length squared
    for i in range(len(p1)):  # each dimension
        h = p2[i] - p1[i] 
        line.append( h )
        llsq += h*h
    ll = math.sqrt(llsq)  # length
    # normalize line
    if ll <=0: return [0,0]
    for i in range(len(p1)):  # each dimension
        line[i] = line[i]/ll
    return line

def pointPointDistance(p1,p2):
    """ Return the distance between a pair of points (L2 norm). """
    llsq = 0.0 # line length squared
    # faster, only for 2D
    h = p2[0] - p1[0] 
    llsq = llsq + (h*h)
    h = p2[1] - p1[1] 
    llsq = llsq + (h*h)
    return math.sqrt(llsq)

    for i in range(len(p1)):  # each dimension, general case
        h = p2[i] - p1[i] 
        llsq = llsq + (h*h)
    return math.sqrt(llsq)

def closestPointToPoint(G,p2):
    dmin = 999999999
    for v in G[nodes]:
        p1 = vertices [ v ]
        d = pointPointDistance(p1,p2)
        if d <= dmin:
            dmin = d
            close = v
    return close

def returnParent(k):
    """ Return parent note for input node k. """
    for e in G[edges]:
        if e[1]==k: 
            canvas.polyline(  [vertices[e[0]], vertices[e[1]] ], style=3  )
            return e[0]

def pickGvertex():
    try: edge = random.choice( G[edges] )
    except: return pickvertex()
    v = random.choice( edge )
    return v

def redraw():
    canvas.clear()
    canvas.markit(start_x, start_y, r=SMALLSTEP)
    canvas.markit( tx, ty, r=2*SMALLSTEP )
    drawGraph(G)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")

def ccw(A,B,C):
    """ Determine if three points are listed in a counterclockwise order.
    For three points A, B and C. If the slope of the line AB is less than 
    the slope of the line AC then the three points are in counterclockwise order.
    See:  http://compgeom.cs.uiuc.edu/~jeffe/teaching/373/notes/x06-sweepline.pdf
    """
    return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

def intersect(A,B,C,D):
        """ do lines AB and CD intersect? """
        i = ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
        #if i: 
        #    canvas.polyline(  [ A,B ], style=4  , tags = ("debug"))
        #    canvas.polyline(  [ C,D ], style=4  , tags = ("debug"))
        #else: 
        #    canvas.polyline(  [ A,B ], style=1  , tags = ("debug")) # green
        #    canvas.polyline(  [ C,D ], style=1  , tags = ("debug"))
        return i
        

def lineHitsRect(p1,p2,r):
   rline = ( (r[0],r[1]), (r[0],r[3]) )
   if intersect( p1, p2, rline[0], rline[1] ): return 1
   rline = ( (r[0],r[1]), (r[2],r[1]) )
   if intersect( p1, p2, rline[0], rline[1] ): return 1
   rline = ( (r[0],r[3]), (r[2],r[3]) )
   if intersect( p1, p2, rline[0], rline[1] ): return 1
   rline = ( (r[2],r[1]), (r[2],r[3]) )
   if intersect( p1, p2, rline[0], rline[1] ): return 1

   return 0

def inRect(p,rect,dilation):
   """ Return 1 if p is inside rect, dilated by dilation (for edge cases). """
   if p[0]<rect[0]-dilation: return 0
   if p[1]<rect[1]-dilation: return 0
   if p[0]>rect[2]+dilation: return 0
   if p[1]>rect[3]+dilation: return 0
   return 1

#TODO: Implement the rrt_search algorithm in this function.
def rrt_search(G, tx, ty):
    # Implement the rrt_algorithm in this section of the code.
    # You should call genPoint() within this function to 
    #get samples from different distributions.
    numIt = 0
    while 1:
        numIt += 1
        randPoint = genPoint(True)
        closestOnTreeIndex = closestPointToPoint(G, randPoint)
        closestPointOnTree = vertices[closestOnTreeIndex]
        newLine = lineFromPoints(closestPointOnTree, randPoint)
        
        # create the new line and new point
        newLine[0] *= SMALLSTEP
        newLine[1] *= SMALLSTEP
        newPoint = []
        newPoint.append(closestPointOnTree[0] + newLine[0])
        newPoint.append(closestPointOnTree[1] + newLine[1])
        
        #verify new point in window
        if inRect(newPoint, (0, 0, XMAX, YMAX), 0)==0:
            continue
        
        #verify closest point to new point is original closest point on tree
        if closestPointToPoint(G, newPoint) != closestOnTreeIndex:
            continue
        
        #verify new point not in any obstacles (buffer = 3 to avoid edge cases)
        cont = 0
        for o in obstacles:
            if inRect(newPoint, o, 3)==1:
                cont = 1
        if cont==1:
            continue
        
        #verify new line does not cross any obstacle boundaries
        for o in obstacles:
            if lineHitsRect(closestPointOnTree, newPoint, o)==1:
                cont = 1
        if cont==1:
            continue
        
        #new point is valid, add it
        newNode = len(vertices)
        G[nodes].append(len(vertices)) #new node
        vertices.append(newPoint) #coords of new node
        G[edges].append((closestOnTreeIndex, len(vertices)-1)) #new edge connecting
        #redraw()
        #canvas.events()
        
        #check for win condition
        if pointPointDistance(newPoint, (tx, ty)) < 2*SMALLSTEP:
            k = newNode
            n = 0
            while 1:
                k=returnParent(k)
                n += 1
                if k == 0:
                    break
            return numIt, n

#ENTRY POINT
print('RRT by Mike Harvey')
sys.stdout.flush()
if visualize:
    canvas = drawSample.SelectRect(xmin=0,ymin=0,xmax=XMAX ,ymax=YMAX, nrects=0, keepcontrol=0)#, rescale=800/1800.)
    redraw()

if 0:  # line intersection testing
        obstacles.append( [ 75,60,125,500 ] )  # tall vertical
        for o in obstacles: canvas.showRect(o, outline='red', fill='blue')
        lines = [
           ( (70,50), (150,150) ),
           ( (50,50), (150,20) ),
           ( (20,20), (200,200) ),
           ( (300,300), (20, 200)  ),
           ( (300,300), (280, 90)  ),
           ]
        for l in lines:
           for o in obstacles:
              lineHitsRect(l[0],l[1],o)
        canvas.mainloop()
    

if 0:
    # random obstacle field
    for nobst in range(0,6000):
        wall_discretization=SMALLSTEP*2  # walls are on a regular grid.
        wall_lengthmax=10.  # fraction of total (1/lengthmax)
        x = wall_discretization*int(random.random()*XMAX/wall_discretization)
        y = wall_discretization*int(random.random()*YMAX/wall_discretization)
        #length = YMAX/wall_lengthmax
        length = SMALLSTEP*2
        if random.choice([0,1]) >0:
            obstacles.append( [ x,y,x+SMALLSTEP,y+10+length ] )  # vertical
        else:
            obstacles.append( [ x,y,x+10+length,y+SMALLSTEP ] )  # horizontal
else:
  if 0:
    # hardcoded simple obstacles
    obstacles.append( [ 300,0,400,95 ] )  # tall vertical
    # slightly hard
    obstacles.append( [ 300,805,400,YMAX ] )  # tall vertical
    #obstacles.append( [ 300,400,1300,430 ] )
    # hard
    obstacles.append( [ 820,220,900,940 ] )
    obstacles.append( [ 300,0,  400,95 ] )  # tall vertical
    obstacles.append( [ 300,100,400,YMAX ] )  # tall vertical
    obstacles.append( [ 200,300,800,400 ] )  # middle horizontal
    obstacles.append( [ 380,500,700,550 ] )
    # very hard
    obstacles.append( [ 705,500,XMAX,550 ] )




if visualize:
    for o in obstacles: canvas.showRect(o, outline='red', fill='blue')


maxvertex += 1
stepIteration = 0
print "step, numIt, numEdges, pathLen, ms"
while 1:
    # graph G
    G = [  [ 0 ]  , [] ]   # nodes, edges
    vertices = [ [10,270], [20,280]   ]
    #redraw()
    
    G[edges].append( (0,1) )
    G[nodes].append(1)
    #if visualize: canvas.markit( tx, ty, r=2*SMALLSTEP ) #draw goal

    #canvas.events()
    
    startTime = time.time()
    numIt, numEdges = rrt_search(G, tx, ty)
    endTime = time.time()
    pathLen = SMALLSTEP*(numEdges-1) + 14 #first edge, sqrt(200)
    #canvas.events()
    #redraw()
    #canvas.mainloop()
    print "%i," %SMALLSTEP, "%i," %numIt, "%i," %numEdges, "%i," %pathLen, "%i" %(1000.0*(endTime - startTime)) 
    sys.stdout.flush()
    
    stepIteration += 1
    if stepIteration >= 10:
        stepIteration = 0
        SMALLSTEP += 3
        if SMALLSTEP > 30:
            break
    #canvas = drawSample.SelectRect(xmin=0,ymin=0,xmax=XMAX ,ymax=YMAX, nrects=0, keepcontrol=0)

#canvas.showRect(rect,fill='red')

if visualize:
    canvas.mainloop()
