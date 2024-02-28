def forward_kinematics(theta1, theta2, theta3, segment_length): 

 # Convert degrees to radians  

	theta1_rad = math.radians(theta1)  
	theta2_rad = math.radians(theta2)  
	theta3_rad = math.radians(theta3)  

	# Calculate positions of each joint  

	x1 = segment_length * math.cos(theta1_rad)  
	y1 = segment_length * math.sin(theta1_rad)  
	x2 = x1 + segment_length * math.cos(theta1_rad + theta2_rad)  
	y2 = y1 + segment_length * math.sin(theta1_rad + theta2_rad)  
	x3 = x2 + segment_length * math.cos(theta1_rad + theta2_rad + theta3_rad)  
	y3 = y2 + segment_length * math.sin(theta1_rad + theta2_rad + theta3_rad)  

return x3, y3 