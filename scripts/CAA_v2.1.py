import math

# ======================= README =======================
#
# Version 2.1 (Pedram Fathollahzadeh)
#
# NOTE: Three things that have been changed from version 2.0:
#
# 1. An improved method to see if the drones are approaching each other have been
# implemented here in the approaching_drones(fpd1, delta1, fpd2, delta2, r) function.
# This one looks at the angle in x/y plane and the difference in z direction. 
#
# 2. The __init__ constructor has been removed, and all the inputs should be sent directly
# to the Algorithm(pos0, dir0, pos1, dir1) function instead. Now, only one instance/object
# of this class is needed, and different inputs can be sent to Algorithm function using 
# the same instance/object. This should be more computationally efficient.
#
# 3. This Collision Avoidance Algorithm now deals with the case when one drone is 
# following the other and they are getting too close. The Algorithm function will
# then return a flag (2) in the last element of the array.  
#
# NOTE: Algorithm() is the main function that should be called and will return either
# a new direction or the same that was decided outside this class, together with a
# flag (0, 1, 2) that tells if a new direction has been set or not, or if one drone
# needs to slow down (check the text above the Algorithm function for more info).
#
# ======================================================


class CAA:


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


	# INPUT: future position of the drones, their direction and their absolute distance
	# OUTPUT (flag):
	# 0 - not approaching at all
	# 1 - approaching each other
	# 2 - the drone that is behind needs to slow down
	def approaching_drones(fpd1, delta1, fpd2, delta2, r):

	    k = 0.05
	    # next goal positions given current direction and goal point
	    ngp1 = [fpd1[0]+k*delta1[0], fpd1[1]+k*delta1[1], fpd1[2]+k*delta1[2]] 
	    ngp2 = [fpd2[0]+k*delta2[0], fpd2[1]+k*delta2[1], fpd2[2]+k*delta2[2]]
	    r_nextGoalPoint = absolute_distance(ngp1, ngp2)

	    # check if the drones are going towards or away from each other
	    if r>r_nextGoalPoint:
	    		# the distance is shrinking, they are approaching each other

	    		# check if they are going towards the same direction
	    		angle_drone1 = math.atan2(delta1[1], delta1[0]) #returns arctan(y/x) in radians
	    		if angle_drone1<0:
	    			angle_drone1 = angle_drone1 + 6.28
	    		angle_drone2 = math.atan2(delta2[1], delta1[0])
	    		if angle_drone2<0:
	    			angle_drone2 = angle_drone2 + 6.28
	    		angle_diff = abs(angle_drone1 - angle_drone2) - 3.14

	    		# check if they are pointing too much towards each other in the x/y plane and in z direction
	            if abs(angle_diff)<0.35 and abs(delta1[2]-delta2[2])<0.10:  #20 [degrees] and 0.10 [meters]

	                # the drones are approaching each other
	                return 1

	            # if the drones are getting closer to each other, but are not going towards each other (one is following the other)
	        	elif abs(angle_diff)>170 and abs(angle_diff)<190:

	        		# the drone following needs to slow down
	        		return 2

	    # they are not approaching each other		
	    return 0


	# INPUT: vectors
	# OUTPUT: new goal direction for a
	def NewGoalDirection(a, b):

		ab_sum = sum(a,b)

		c = cross_product(a, b) / norm(ab_sum)
		d = ab_sum / norm(ab_sum)
		cd_sum = sum(c,d)

		e = cd_sum / norm(cd_sum)

		return e


	# INPUT: goal position and direction for drone 1 and 2
	# OUTPUT: [direction1, direction2, flag]
	# flag = 0 indicates that the drones will not crash, and the original direction is returned
	# flag = 1 indicates that the drones are too close to each other and are going towards each other, new direction returned
	# flag = 2 indicates that the drones are too close to each other but are not going towards each other, the original direction is returned, but the drone behind needs to slow down
	def Algorithm(pos0, dir0, pos1, dir1):

		r_threshold = 0.7
		r = absolute_distance(pos0, pos1)

		# the drones are too close to each other
		if r<r_threshold:


			# the drones are approaching each other
			# return different goal point
			if approaching_drones(pos0, dir0, pos1, dir1, r)==1:

				direction1 = NewGoalDirection(dir0, dir1)
				direction2 = NewGoalDirection(dir1, dir0)

				return [ direction1, direction2, 1 ]

			# the drone that is behind needs to slow down
			# return the original goal direction
			elif approaching_drones(pos0, dir0, pos1, dir1, r)==2:

				return [ direction1, direction2, 2 ]

			# the drones might be too close to each other, but they are not approaching each other
			# return the original goal direction
			else:

				return [ direction1, direction2, 0 ]	


		# the drones will not collide 
		# return the original goal direction
		else:
			return [ direction1, direction2, 0 ]





#collision_avoidance = CAA([0.2,0.3,0], [0.8,0.1,0.1], [0.7,0.3,0], [-0.7,0.1,0.2])
#print(collision_avoidance.Algorithm)

