import math
import rospy

# ======================= README =======================
#
# Version 2.2 (Pedram Fathollahzadeh, Joakim Lilja, Caro Heidenreich)
# Direction based collision avoidance
# If crazyflies are within a ball of radius r and approaching each other (angle between directions between -5pi/4 and -3pi/4)
# a new direction for the crazyflie in question is calculated based on the cross product of their directions. If directions are parallell the direction is rotated around the axis with the smallest component, e.g. arg_min[direction].
# ======================================================

tolerance=1E-7

#INPUT: vectors
#OUTPUT: dot product
def dot_product(a, b):
    c = 0
    for i in range(0,len(a)):
        c = c + a[i]*b[i]
    return c

#INPUT: a=vector b=scalar
#OUTPUT: division between vector and scalar
def div(a, b):
    for i in range(0,len(a)):
        a[i]=a[i]/b
    return a

# INPUT: vectors
# OUTPUT: cross product
def cross_product(a, b):
    c = [ a[1]*b[2] - a[2]*b[1], 
    a[2]*b[0] - a[0]*b[2],
    a[0]*b[1] - a[1]*b[0] ]

    return c


# INPUT: vectors
# OUTPUT: sum of the vectors
def sum(a, b):
    c = [a[0]+b[0], a[1]+b[1], a[2]+b[2]]
    return c
    

# INPUT: vectors
# OUTPUT: norm of the sum of the vectors
def norm(a):
    s = math.sqrt( (a[0])**2 + (a[1])**2 + (a[2])**2 )
    return s


# INPUT: future position of the drones
# OUTPUT: absolute distance between the drones
def absolute_distance(fpd1, fpd2):
    
    r = math.sqrt( (fpd1[0]-fpd2[0])**2 + (fpd1[1]-fpd2[1])**2 + (fpd1[2]-fpd2[2])**2 )
    return r


# INPUT: position of the drones, their direction and their absolute distance
# OUTPUT (flag):
# 0 - not approaching at all
# 1 - approaching each other
# 2 - the drone that is behind needs to slow down
def approaching_drones(pd1, delta1, pd2, delta2, r):

    cf2_vec = [pd2[0]-pd1[0], pd2[1]-pd1[1], pd2[2] - pd1[2]]
    isApproaching = dot_product(delta1, cf2_vec) >= 0
    # check if the drones are going towards or away from each other
    if isApproaching:
            print("isApproaching")
            # the distance is shrinking, they are approaching each other

            if not norm(delta1) == 0:
                delta1 = div(delta1,norm(delta1))
            if not norm(delta2) == 0:
                delta2 = div(delta2, norm(delta2))

            dot = dot_product(delta1, delta2)
            print "Dot between vectors"
            print dot
            theta_threshold = math.pi/4
            if (dot < math.cos(math.pi + theta_threshold) and dot >= -1) or norm(delta1)==0 or norm(delta2)==0:

                # the drones are approaching each other
                return 1

            # if the drones are getting closer to each other, but are not going towards each other (one is following the other)
            else:

                # the drone following needs to slow down
                return 2

    # they are not approaching each other		
    return 0



# INPUT: vectors
# OUTPUT: new goal direction for a
def NewGoalDirection(a, b):
    c = cross_product(a, b)

    theta = math.pi/4

    if norm(c)==0 :
        print("Vectors are parallell")
        arg = 0
        for i in range(0,len(a)):
            if math.fabs(a[i]) < math.fabs(a[arg]):
                arg = i
        if arg == 0:
            print("Rotate around x")
            return [a[0], a[1]*math.cos(theta)-a[2]*math.sin(theta), a[1]*math.sin(theta)+a[2]*math.cos(theta)]
        if arg == 1:
            print("Rotate around y")
            return [a[0]*math.cos(theta)+a[2]*math.sin(theta), a[1], -a[0]*math.sin(theta)+a[2]*math.cos(theta)]
        print "Rotate around z"
        return [a[0]*math.cos(theta)-a[1]*math.sin(theta), a[0]*math.sin(theta)+a[1]*math.cos(theta), a[2]]


    c = div(c,norm(c))
    
    ab_sum = sum(a,b)
    ab_sum = div(ab_sum, norm(ab_sum))

    # Weigh c less than a and b
    c = [c[0], c[1], c[2]]

    d = sum(c,a)
    d = div(d, norm(d))
    return d


# INPUT: goal position and direction for drone 1 and 2
# OUTPUT: [direction1, direction2, flag]
# flag = 0 indicates that the drones will not crash, and the original direction is returned
# flag = 1 indicates that the drones are too close to each other and are going towards each other, new direction returned
# flag = 2 indicates that the drones are too close to each other but are not going towards each other, the original direction is returned, but the drone behind needs to slow down
def Algorithm(pos0, dir0, pos1, dir1):

    r_threshold = 0.7
    r = absolute_distance(pos0, pos1)

    if norm(dir0)<=tolerance and norm(dir1)<=tolerance:
        #do nothing
        return [dir0, dir1, 0]

    # the drones are too close to each other
    elif r<r_threshold:


        # the drones are approaching each other
        # return different goal point
        mode = approaching_drones(pos0, dir0, pos1, dir1, r)
        if mode==1:
            dir0 = NewGoalDirection(dir0, dir1)
            #direction1 = NewGoalDirection(dir1, dir0)


            return [ dir0, dir1, 1 ]

        # the drone that is behind needs to slow down
        # return the original goal direction
        elif mode==2:

            return [ dir0, dir1, 2 ]

        # the drones might be too close to each other, but they are not approaching each other
        # return the original goal direction
        else:

            return [ dir0, dir1, 0 ]	


    # the drones will not collide 
    # return the original goal direction
    else:
        return [ dir0, dir1, 0 ]





#print(Algorithm([0.0,0.0,0], [0.0,0,0], [0.2,0.0,0], [1.0, 0.0,0.0]))
