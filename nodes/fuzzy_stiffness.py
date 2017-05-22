#!/usr/bin/env python
# This code is used to estimate the relative velocity between the human and the robot
import roslib; roslib.load_manifest('kcl_stiff')
import rospy, math, time
from std_msgs.msg import Float64MultiArray
import dynamic_reconfigure.client

class stiffness(object):
  def __init__(self):
    # Initialize the ROS node
    rospy.init_node('fuzzy_stiffness')
    # Get the stiffness value parameter
    self.stiff_max_input = rospy.get_param('/stiffness_max')
    # Initialize the human-robot distance vector
    self.dist_vec = [1.0, 1.0, 1.0];
    # Initialize the human-robot relative velocity vector
    self.vel_vec = [0.0, 0.0, 0.0];
    # Initialize Stiffness vector
    self.stiffness_vec = [50.0, 50.0, 50.0]
    # Initialize distance magnitude
    self.dist = 1.0
    # Initialize speed
    self.vel = 0.0
    # Define the stiffness
    self.stiff_send = Float64MultiArray()

    # Initialize maximum distance and maximum velocity for fuzzy rule
    self.dist_max = 1.0
    self.vel_max = 2.0
    # Initialize maximum stiffness
    self.stiff_max = float(self.stiff_max_input)

    # Initialize distance offset
    self.distance_offset = 0.08

    # Initialize the ROS Publisher
    # Used to publish the stiffness vector
    pub = rospy.Publisher('/stiffness_vector', Float64MultiArray, queue_size=10)

    # Initialize the ROS Subscriber
    # Used to get the human-robot closest distance in scalar
    sub_dist = rospy.Subscriber('/fourbythree_topics/stiffness/closest_distance/scalar', Float64MultiArray, self.get_dist)
    # Used to get the human-robot closest distance in vector
    sub_dist_vec = rospy.Subscriber('/fourbythree_topics/stiffness/closest_distance/vector', Float64MultiArray, self.get_dist_vec)
    # Used to get the human-robot's relative velocity in scalar
    sub_vel = rospy.Subscriber('/fourbythree_topics/stiffness/velocity', Float64MultiArray, self.get_vel)
    # Used to get the human-robot's relative velocity in vector
    sub_vel_vec = rospy.Subscriber('/fourbythree_topics/stiffness/velocity_vector', Float64MultiArray, self.get_vel_vec)
    # Set the rate to be 40 Hz
    rate = rospy.Rate(40.0)
    while (not rospy.is_shutdown()):
      # Check whether the stiffness adjustment is set
      self.stiff_on = rospy.get_param('/stiffness_on')

      # Produce the stiffness in task space using fuzzy rule
      self.fuzzy_stiff_task()
      # Prepare the stiffness data before being sent
      self.get_stiffness_task()
      # Publish the stiffness vector
      pub.publish(self.stiff_send)

      # Sleep until 1/40 s
      rate.sleep()

  # Get distance scalar
  def get_dist(self,msg):
    self.dist = msg.data[0]

  # Get distance vector
  def get_dist_vec(self,msg):
    self.dist_vec[0] = msg.data[0]
    self.dist_vec[1] = msg.data[1]
    self.dist_vec[2] = msg.data[2]

  # Get velocity magnitude
  def get_vel(self,msg):
    self.vel = msg.data[0]

  # Get velocity vector
  def get_vel_vec(self,msg):
    self.vel_vec[0] = msg.data[0]
    self.vel_vec[1] = msg.data[1]
    self.vel_vec[2] = msg.data[2]

  # Prepare the stiffness data before being sent
  def get_stiffness_task(self):
    # Empty the array
    self.stiff_send.data = []
    for i in range(0,3):
      # Put the vector component into array
      self.stiff_send.data.append(self.stiffness_vec[i])

  # Deciding which Fuzzy Rule is Active
  def fuzzy_rule_on(self, distance, velocity):
    # Set the difference in distance and velocity among the members of fuzzy rules
    self.dist_delta = self.dist_max / 5.0
    self.vel_delta = self.vel_max / 4.0

    # Determine which rule is active according to the value of distance and velocity
    if((distance>=0) and (distance<self.dist_delta)):
      self.ON_dist[0] = 1
    if((distance>0) and (distance<2*self.dist_delta)):
      self.ON_dist[1] = 1
    if((distance>self.dist_delta) and (distance<3*self.dist_delta)):
      self.ON_dist[2] = 1
    if((distance>2*self.dist_delta) and (distance<4*self.dist_delta)):
      self.ON_dist[3] = 1
    if((distance>3*self.dist_delta) and (distance<5*self.dist_delta)):
      self.ON_dist[4] = 1
    if(distance>4*self.dist_delta):
      self.ON_dist[5] = 1
    if(velocity<-3*self.vel_delta):
      self.ON_vel[0] = 1
    if((velocity>-4*self.vel_delta) and (velocity<-2*self.vel_delta)):
      self.ON_vel[1] = 1
    if((velocity>-3*self.vel_delta) and (velocity<-1*self.vel_delta)):
      self.ON_vel[2] = 1
    if((velocity>-2*self.vel_delta) and (velocity<0*self.vel_delta)):
      self.ON_vel[3] = 1
    if((velocity>-1*self.vel_delta) and (velocity<1*self.vel_delta)):
      self.ON_vel[4] = 1
    if((velocity>0*self.vel_delta) and (velocity<2*self.vel_delta)):
      self.ON_vel[5] = 1
    if((velocity>1*self.vel_delta) and (velocity<3*self.vel_delta)):
      self.ON_vel[6] = 1
    if((velocity>2*self.vel_delta) and (velocity<4*self.vel_delta)):
      self.ON_vel[7] = 1
    if(velocity>3*self.vel_delta):
      self.ON_vel[8] = 1

  # Used as a membership function of the fuzzy rule
  def membership_function(self, distance, velocity):
    # Empty the array used to keep the distance and velocity membership value
    self.miu_dist = []
    self.miu_vel = []

    # Determine how many rules are active for distance and velocity
    self.dist_rule = len(self.ON_dist)
    self.vel_rule = len(self.ON_vel)

    # Determine the membership value for distance input
    for i in range (1, self.dist_rule+1):
      # Check whether the distance rule is active
      if(self.ON_dist[i-1]==1):
        # Set 3 points used to draw a triangle membership function
        a = -1*self.dist_delta+(i-1)*self.dist_delta
        c = self.dist_delta+(i-1)*self.dist_delta
        b = (a+c)/2

        # Check in which part of triangle function is the distance input value
        if((distance>=a) and (distance<b)):
            # Executed for the case of distance between initial point and middle point of triangle
            u = (distance-a)/(b-a)
        else:
          # Executed for the case of distance between middle point and final point of triangle
    	  if(i!=6):
            # Executed for distance less than maximum distance
    	    u = (distance-c)/(b-c)
    	  else:
            # Executed for distance bigger than maximum distance
    	    u = 1
        # Put the membership value in the array
        self.miu_dist.append(u)

    # Determine the membership value for velocity input
    for i in range(1, self.vel_rule+1):
      # Check whether the velocity rule is active
      if(self.ON_vel[i-1]==1):
        # Set 3 points used to draw a triangle membership function
        a = -5*self.vel_delta+(i-1)*self.vel_delta
        c = -3*self.vel_delta+(i-1)*self.vel_delta
        b = (a+c)/2

        # Check in which part of triangle function is the velocity input value
        if((velocity>=a) and (velocity<b)):
            # Executed for the case of velocity between initial point and middle point of triangle
            u = (velocity-a)/(b-a)
        elif((velocity>=b) and (velocity<c)):
            # Executed for the case of velocity between middle point and final point of triangle
            u = (velocity-c)/(b-c)
    	else:
            # Executed for the case of velocity less than initial point of the first triangle rule or bigger than final point of the last triangle rule
            u = 1.0
        # Put the membership value in the array
        self.miu_vel.append(u)

  # Used as a fuzzy decision making
  def fuzzy_decision(self):
    # Set the number of the active fuzzy rule for distance and velocity
    self.rule_dist_size = len(self.miu_dist)
    self.rule_vel_size = len(self.miu_vel)

    # Reset the index for distance
    dist_index = 0
    # Empty all the arrays used for fuzzy decision making
    self.fuzzy_out = []
    self.fuzzy_dist = []
    self.fuzzy_vel = []

    # Decision making process to produce stiffness output membership value for all possible distance and velocity input
    for ind_d in range(1, self.rule_dist_size+1):
      # Initialize the array used to keep which distance rule is active
      ON_dist_el = [0]*self.dist_rule
      # Reset the velocity index
      vel_index = 0

      # Loop for all the active distance rule
      for i in range(1,self.dist_rule+1):
        # Check whether the distance rule is active
    	if((self.ON_dist[i-1] == 1) and (i!=dist_index)):
          # Keep the active rule inside the array
    	  ON_dist_el[i-1] = 1
          # Save the index of the distance in which the rule is active
    	  dist_index = i
          # Exit the loop
    	  break

      # Loop for all the active velocity rule
      for ind_v in range(1, self.rule_vel_size+1):
        # Initialize the array used to keep which velocity rule is active
        ON_vel_el = [0]*self.vel_rule
        # Loop for all the active velocity rule
        for i in range(1, self.vel_rule+1):
          # Check whether the velocity rule is active
    	  if((self.ON_vel[i-1] == 1) and (i!=vel_index)):
            # Keep the active rule inside the array
    	    ON_vel_el[i-1] = 1;
            # Save the index of the velocity in which the rule is active
    	    vel_index = i;
            # Exit the loop
    	    break

        # Initialize the stiffness membership array
        ON_stiff = [0, 0, 0, 0, 0, 0];
        # Produce the stiffness membership array according to all distance-velocity possibilities
        if((ON_dist_el[0]==1)):
            ON_stiff[0] = 1;
        if((ON_dist_el[1]==1) and ((ON_vel_el[0]==1) or (ON_vel_el[1]==1) or (ON_vel_el[2]==1) or (ON_vel_el[3]==1))):
            ON_stiff[0] = 1;
        if((ON_dist_el[1]==1) and ((ON_vel_el[4]==1) or (ON_vel_el[5]==1) or (ON_vel_el[6]==1) or (ON_vel_el[7]==1) or (ON_vel_el[8]==1))):
            ON_stiff[1] = 1;
        if((ON_dist_el[2]==1)):
          if((ON_vel_el[0]==1) or (ON_vel_el[1]==1)):
              ON_stiff[0] = 1;
          if((ON_vel_el[2]==1) or (ON_vel_el[3]==1)):
              ON_stiff[1] = 1;
          if((ON_vel_el[4]==1)):
              ON_stiff[2] = 1;
          if((ON_vel_el[5]==1) or (ON_vel_el[6]==1)):
              ON_stiff[4] = 1;
          if((ON_vel_el[7]==1) or (ON_vel_el[8]==1)):
              ON_stiff[5] = 1;

        if((ON_dist_el[3]==1)):
          if(ON_vel_el[0]==1):
              ON_stiff[1] = 1;
          if(ON_vel_el[1]==1):
              ON_stiff[2] = 1;
          if(ON_vel_el[2]==1):
              ON_stiff[3] = 1;
          if(ON_vel_el[3]==1):
              ON_stiff[4] = 1;
          if(ON_vel_el[4]==1 or ON_vel_el[5]==1 or ON_vel_el[6]==1 or ON_vel_el[7]==1 or ON_vel_el[8]==1):
              ON_stiff[5] = 1;

        if((ON_dist_el[4]==1) and (ON_vel_el[0]==1)):
            ON_stiff[3] = 1;
        if((ON_dist_el[4]==1) and (ON_vel_el[1]==1 or ON_vel_el[2]==1)):
            ON_stiff[4] = 1;
        if((ON_dist_el[4]==1) and (ON_vel_el[3]==1 or ON_vel_el[4]==1 or ON_vel_el[5]==1 or ON_vel_el[6]==1 or ON_vel_el[7]==1 or ON_vel_el[8]==1)):
            ON_stiff[5] = 1;
        if(ON_dist_el[5]==1):
            ON_stiff[5] = 1;

        # Put the stiffness's, distance's and velocity's membership array into arrays
        self.fuzzy_out.append(ON_stiff)
        self.fuzzy_dist.append(ON_dist_el)
        self.fuzzy_vel.append(ON_vel_el)

  # Used to do defuzzification
  def defuzzification(self):
    # Inference Process
    # 'Product' is used in this case

    # Get the width and length of the stiffness fuzzy membership array
    width = len(self.fuzzy_out)
    length = len(self.fuzzy_out[0])

    # Initialize variable k used as an index for fuzzy membership array
    k = 1;
    # Empty the array used to keep the membership product of the inputs
    miu_prem = [];
    # Empty the array used to keep the membership of the output
    miu_out = [];
    # Loop for all input possibilities
    for ind_d in range(1, self.rule_dist_size+1):
      for ind_v in range(1, self.rule_vel_size+1):
        # Add the product between input membership function into array
    	miu_prem.append(self.miu_dist[ind_d-1]*self.miu_vel[ind_v-1])
        # Calculate the membership value of output
    	for ind_o in range(1, length+1):
    	  if(self.fuzzy_out[k-1][ind_o-1] == 1):
    	    miu_out.append(self.stiff_max/5*ind_o)
    	k = k+1;

    # Defuzzification
    # Centre average is used

    # Reset the summation of numerator and denominator
    sum_num = 0;
    sum_den = 0;
    # Loop for all input possibilities
    for ind in range(1, (self.rule_dist_size*self.rule_vel_size)+1):
      # Determine the center point
      centre = (miu_out[ind-1]-self.stiff_max/5);
      # Calculate the sum of weighted membership value
      sum_num = sum_num + centre*miu_prem[ind-1];
      # Calculate the sum of membership value
      sum_den = sum_den + miu_prem[ind-1];

    # Calculate the ratio to get the centre average
    return sum_num/sum_den;

  # Used to calculate the final stiffness vector
  def fuzzy_stiff_task(self):
    # Initialize the arrays used to keep the status of the distance / velocity rules
    self.ON_dist = [0, 0, 0, 0, 0, 0]
    self.ON_vel = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    # Determine which rules is on for distance and velocity input
    self.fuzzy_rule_on(abs(self.dist-self.distance_offset), self.vel)
    # Built the membership function for distance and velocity input
    self.membership_function(abs(self.dist-self.distance_offset), self.vel)
    # Fuzzy decision making to decide the membership value of the output
    self.fuzzy_decision()
    # Calculate the stiffness magnitude
    stiffness_mag = self.stiff_max - self.defuzzification()

    # Check whether fuzzy-based stiffness adjustment status is active
    if(self.stiff_on==1):
        # If so, calculate the stiffness vector in x,y,z directions
        for i in range(0,3):
          self.stiffness_vec[i] = self.stiff_max - stiffness_mag*abs(self.dist_vec[i]/self.dist)
    # Otherwise, put the maximum stiffness in x,y,z directions
    else:
        for i in range(0,3):
          self.stiffness_vec[i] = self.stiff_max

if __name__ == '__main__':
    try:
        stiffness()
    except rospy.ROSInterruptException: pass
