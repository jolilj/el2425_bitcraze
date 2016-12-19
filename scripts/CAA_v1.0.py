import math



# Here we have the algorithm... 
def collision_avoidance_algorithm(cpd1, cpd2, tr1, tr2, delta):

    r_d = 0.5
    r_c = r_d/2

    # ***** define new path (half circle) *****

    # The new path will be 4 points in space. This here is just an initialization with the already planned path.
    new_path1 = [ tr1[0], tr1[1], tr1[2], tr1[3] ]
    new_path2 = [ tr2[0], tr2[1], tr2[2], tr2[3] ]
    
    # find which direction they are mostly approaching (0 -> x, 1 -> y, 2 -> z)
    d = delta.index(max(delta))
    #print(d)

    # draw half circle on x axis
    if d==0:
            
            # determine which drone is on the right side
            if cpd1[0]<cpd2[0]:
            
                # drone2 is on the right side

                # new path for drone2
                new_path2[0][0] = cpd2[0] - 0.3*r_c
                new_path2[0][1] = cpd2[1] + 0.7*r_c 

                new_path2[1][0] = cpd2[0] - r_c
                new_path2[1][1] = cpd2[1] + r_c

                new_path2[2][0] = cpd2[0] - 1.3*r_c
                new_path2[2][1] = cpd2[1] + 0.7*r_c

                new_path2[3][0] = cpd2[0] - 2*r_c
                new_path2[3][1] = cpd2[1]

                # new path for drone1
                new_path1[0][0] = cpd1[0] + 0.3*r_c
                new_path1[0][1] = cpd1[1] - 0.7*r_c

                new_path1[1][0] = cpd1[0] + r_c
                new_path1[1][1] = cpd1[1] - r_c

                new_path1[2][0] = cpd1[0] + 1.7*r_c
                new_path1[2][1] = cpd1[1] - 0.7*r_c

                new_path1[3][0] = cpd1[0] + 2*r_c
                new_path1[3][1] = cpd1[1]

            else:

                # drone1 is on the right side

                # new path for drone1
                new_path1[0][0] = cpd1[0] - 0.3*r_c
                new_path1[0][1] = cpd1[1] + 0.7*r_c 

                new_path1[1][0] = cpd1[0] - r_c
                new_path1[1][1] = cpd1[1] + r_c

                new_path1[2][0] = cpd1[0] - 1.3*r_c
                new_path1[2][1] = cpd1[1] + 0.7*r_c

                new_path1[3][0] = cpd1[0] - 2*r_c
                new_path1[3][1] = cpd1[1]

                # new path for drone2
                new_path2[0][0] = cpd2[0] + 0.3*r_c
                new_path2[0][1] = cpd2[1] - 0.7*r_c

                new_path2[1][0] = cpd2[0] + r_c
                new_path2[1][1] = cpd2[1] - r_c

                new_path2[2][0] = cpd2[0] + 1.7*r_c
                new_path2[2][1] = cpd2[1] - 0.7*r_c

                new_path2[3][0] = cpd2[0] + 2*r_c
                new_path2[3][1] = cpd2[1]
                

    # draw half circle on y axis
    elif d==1:

            # determine which drone is on the lower side
            if cpd1[1]<cpd2[1]:

                # drone1 is on the lower side (y-dir)

                # new path for drone1
                new_path1[0][0] = cpd1[0] + 0.7*r_c
                new_path1[0][1] = cpd1[1] + 0.3*r_c 

                new_path1[1][0] = cpd1[0] + r_c
                new_path1[1][1] = cpd1[1] + r_c

                new_path1[2][0] = cpd1[0] + 0.7*r_c
                new_path1[2][1] = cpd1[1] + 1.7*r_c

                new_path1[3][0] = cpd1[0]
                new_path1[3][1] = cpd1[1] + 2*r_c

                # new path for drone2
                new_path2[0][0] = cpd2[0] - 0.7*r_c
                new_path2[0][1] = cpd2[1] - 0.3*r_c 

                new_path2[1][0] = cpd2[0] - r_c
                new_path2[1][1] = cpd2[1] - r_c

                new_path2[2][0] = cpd2[0] - 0.7*r_c
                new_path2[2][1] = cpd2[1] - 1.7*r_c

                new_path2[3][0] = cpd2[0]
                new_path2[3][1] = cpd2[1] - 2*r_c
                
            else:

                # drone2 is on the lower side (y-dir)

                # new path for drone2
                new_path2[0][0] = cpd2[0] + 0.7*r_c
                new_path2[0][1] = cpd2[1] + 0.3*r_c 

                new_path2[1][0] = cpd2[0] + r_c
                new_path2[1][1] = cpd2[1] + r_c

                new_path2[2][0] = cpd2[0] + 0.7*r_c
                new_path2[2][1] = cpd2[1] + 1.7*r_c

                new_path2[3][0] = cpd2[0]
                new_path2[3][1] = cpd2[1] + 2*r_c

                # new path for drone1
                new_path1[0][0] = cpd1[0] - 0.7*r_c
                new_path1[0][1] = cpd1[1] - 0.3*r_c 

                new_path1[1][0] = cpd1[0] - r_c
                new_path1[1][1] = cpd1[1] - r_c

                new_path1[2][0] = cpd1[0] - 0.7*r_c
                new_path1[2][1] = cpd1[1] - 1.7*r_c

                new_path1[3][0] = cpd1[0]
                new_path1[3][1] = cpd1[1] - 2*r_c

    # draw half circle on z axis
    elif d==2:

            # determine which drone is on the lower side
            if cpd1[2]<cpd2[2]:

                # drone1 is on the lower side (z-dir)

                # new path for drone1
                new_path1[0][0] = cpd1[0] + 0.7*r_c
                new_path1[0][2] = cpd1[2] + 0.3*r_c 

                new_path1[1][0] = cpd1[0] + r_c
                new_path1[1][2] = cpd1[2] + r_c

                new_path1[2][0] = cpd1[0] + 0.7*r_c
                new_path1[2][2] = cpd1[2] + 1.7*r_c

                new_path1[3][0] = cpd1[0]
                new_path1[3][2] = cpd1[2] + 2*r_c

                # new path for drone2
                new_path2[0][0] = cpd2[0] - 0.7*r_c
                new_path2[0][2] = cpd2[2] - 0.3*r_c 

                new_path2[1][0] = cpd2[0] - r_c
                new_path2[1][2] = cpd2[2] - r_c

                new_path2[2][0] = cpd2[0] - 0.7*r_c
                new_path2[2][2] = cpd2[2] - 1.7*r_c

                new_path2[3][0] = cpd2[0]
                new_path2[3][2] = cpd2[2] - 2*r_c

            else:

                # drone2 is on the lower side (z-dir)

                # new path for drone2
                new_path2[0][0] = cpd2[0] + 0.7*r_c
                new_path2[0][2] = cpd2[2] + 0.3*r_c 

                new_path2[1][0] = cpd2[0] + r_c
                new_path2[1][2] = cpd2[2] + r_c

                new_path2[2][0] = cpd2[0] + 0.7*r_c
                new_path2[2][2] = cpd2[2] + 1.7*r_c

                new_path2[3][0] = cpd2[0]
                new_path2[3][2] = cpd2[2] + 2*r_c

                # new path for drone1
                new_path1[0][0] = cpd1[0] - 0.7*r_c
                new_path1[0][2] = cpd1[2] - 0.3*r_c 

                new_path1[1][0] = cpd1[0] - r_c
                new_path1[1][2] = cpd1[2] - r_c

                new_path1[2][0] = cpd1[0] - 0.7*r_c
                new_path1[2][2] = cpd1[2] - 1.7*r_c

                new_path1[3][0] = cpd1[0]
                new_path1[3][2] = cpd1[2] - 2*r_c
               

           

            

    print(new_path1)
    print(new_path2)

# This will return the absolute distance between the drones.
def absolute_distance(cpd1,cpd2):
	
    r = math.sqrt( (cpd1[0]-cpd2[0])**2 + (cpd1[1]-cpd2[1])**2 + (cpd1[2]-cpd2[2])**2 )
    return r

def approaching_direction(distance_current, distance_future):
    dx = distance_current[0] - distance_future[0] # >0 indicates distance shrinking in x direction
    dy = distance_current[1] - distance_future[1] # >0 indicates distance shrinking in y direction
    dz = distance_current[2] - distance_future[2] # >0 indicates distance shrinking in z direction

    return [dx,dy,dz]

# This function will return True if the drones are approaching each other by looking at the current distance
# and comparing it to the future distance given by the trajectory paths.
def approaching_drones(cpd1, cpd2, fpd1, fpd2):

    r = absolute_distance(cpd1,cpd2)
    if r<=0.5:

            r_future = absolute_distance(fpd1, fpd2)

            if r_future<r:

                # the drones are approaching each other
                return True

    # they are not approaching each other		
    return False



def main(trajectory_1, trajectory_2, time):

    r_d = 0.5

    # current and future position of drone 1/2. should be the same as trajectory[time]
    cpd1 = trajectory_1[time]
    cpd2 = trajectory_2[time]
    
    fpd1 = trajectory_1[time+1]
    fpd2 = trajectory_2[time+1]

    # current absolute distance between drones
    #r = math.sqrt( (cpd1[0]-cpd2[0])**2 + (cpd1[1]-cpd2[1])**2 + (cpd1[2]-cpd2[2])**2 )

	
    # compare the current absolute distance between the drones with threshold
    if approaching_drones(cpd1, cpd2, fpd1, fpd2)==True:

        distance_current = [ abs(cpd1[0]-cpd2[0]), abs(cpd1[1]-cpd2[1]), abs(cpd1[2]-cpd2[2]) ]
        distance_future = [ abs(fpd1[0] - fpd2[0]), abs(fpd1[1] - fpd2[1]), abs(fpd1[2] - fpd2[2]) ]

        delta = approaching_direction(distance_current, distance_future) # [dx, dy, dz]

        tr1 = trajectory_1[time+1:time+5]
        tr2 = trajectory_2[time+1:time+5]
        collision_avoidance_algorithm(cpd1, cpd2, tr1, tr2, delta)





# the only thing we actually need is the current position and the four future position of the drones
# these trajectories are to test the code

#trajectory_1 = [ [0, 0.7, 1], [0.1, 0.7, 1], [0.2, 0.7, 1], [0.3, 0.7, 1], [0.4, 0.7, 1], [0.5, 0.7, 1], [0.6, 0.7, 1], [0.7, 0.7, 1], [0.8, 0.7, 1], [0.9, 0.7, 1], [1, 0.7, 1], [1.1, 0.7, 1] ]
#trajectory_2 = [ [1.5, 0.7, 1], [1.4, 0.7, 1], [1.3, 0.7, 1], [1.2, 0.7, 1], [1.1, 0.7, 1], [1, 0.7, 1], [0.9, 0.7, 1], [0.8, 0.7, 1], [0.7, 0.7, 1], [0.6, 0.7, 1], [0.5, 0.7, 1], [0.4, 0.7, 1] ]
#time = 5 # here they are 0.5 m from each other

trajectory_1 = [ [1, 0.1, 1], [1, 0.2, 1], [1, 0.3, 1], [1, 0.4, 1], [1, 0.5, 1], [1, 0.6, 1], [1, 0.7, 1], [1, 0.8, 1], [1, 0.9, 1], [1, 1, 1], [1, 1.1, 1], [1, 1.2, 1] ]
trajectory_2 = [ [1, 1.2, 1], [1, 1.1, 1], [1, 1, 1], [1, 0.9, 1], [1, 0.8, 1], [1, 0.7, 1], [1, 0.6, 1], [1, 0.5, 1], [1, 0.4, 1], [1, 0.3, 1], [1, 0.2, 1], [1, 0.1, 1] ]
time = 3

main(trajectory_1, trajectory_2, time)
