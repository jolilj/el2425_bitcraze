import math

# ======================= README =======================
#
# NOTE: Two things that can and maybe should be changed:
# 1. Improve the method that checks if the drones are going towards each other. This
# can be done by, for example, using the cross product instead of using sum & norm in the
# last if statement inside the approaching_drones function. 
# 2. Remove the __init__ constructor and send inputs directly to Algorithm() function
# instead. This so that an instance/object of this class only needs to be created once,
# and different inputs can be sent to Algorithm() function using the same instance/object.
# This should be more computationally efficient.
# 
# NOTE: Here we do not deal with the case when one drone is following the other. That is,
# if they are too close to each other, but not approaching each other, this version of the
# Collision Avoidance Algorithm will not return a different point from what was initially 
# decided outside this class.
# 
# NOTE: Algorithm() is the main function that should be called and will return either
# a new direction or the same that was decided outside this class.
#
# ======================================================


class CAA:

	def __init__(self, pos0, dir0, pos1, dir1):
		self.pos0 = pos0 #future position of drone 1 (given direction)
		self.dir0 = dir0 #current direction of drone 1
		self.pos1 = pos1 #future position of drone 2 (given direction)
		self.dir1 = dir1 #current direction of drone 2


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
	# OUTPUT: True if approaching, False otherwise
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
	            if norm(sum(delta1, delta2))<=0.25: #NOTE: improve this (use cross product instead?)

	                # the drones are approaching each other
	                return True

	            # can add an else statement here to make sure one of the drones
	            # slows down

	    # they are not approaching each other		
	    return False


	# INPUT: vectors
	# OUTPUT: new goal direction for a
	def NewGoalDirection(a, b):

		ab_sum = sum(a,b)

		c = cross_product(a, b) / norm(ab_sum)
		d = ab_sum / norm(ab_sum)
		cd_sum = sum(c,d)

		e = cd_sum / norm(cd_sum)

		return e


	def Algorithm():

		fpd1 = self.pos0
		fpd2 = self.pos1
		direction1 = self.dir0
		direction2 = self.dir1

		r_threshold = 0.7
		r = absolute_distance(fpd1, fpd2)

		# the drones are too close to each other
		if r<r_threshold:

			# the drones are approaching each other, return different goal point
			if approaching_drones(fpd1, self.dir0, fpd2, self.dir1, r):

				direction1 = NewGoalDirection(direction1, direction2)
				direction2 = NewGoalDirection(direction2, direction1)

				return [ direction1, direction2 ]

		# the drones will not collide, return the original goal direction
		else:
			return [ direction1, direction2]





#collision_avoidance = CAA([0.2,0.3,0], [0.8,0.1,0.1], [0.7,0.3,0], [-0.7,0.1,0.2])
#print(collision_avoidance.Algorithm)

