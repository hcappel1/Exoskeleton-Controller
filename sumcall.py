
import numpy as np
import pickle
from sklearn.ensemble import RandomForestRegressor
from sklearn.externals import joblib
import rospy
from std_msgs.msg import String



def load():
	filename = 'forest_full_v4.pkl'
	dt = pickle.load(open(filename, 'rb'))
	return dt

def add(a,dt):
	
	a = np.reshape(a, (1, 7))
	# # dt = joblib.load('my_model.h5')
	# # print (a.shape)
	y_pred =  dt.predict(a)
	# print (y_pred)
	return y_pred[0]

	# print (a)
	# return a

def DataLog(data):
	return data


def communicate():
	rospy.init_node('stiffness_estimator', anonymous = True)
	rate = rospy.Rate(10)

	sensor_vals = rospy.Subscriber("sensor_input", Float32MultiArray, DataLog)

	dt = load()
	stiffness = add(sensor_vals,dt)	

	pub = rospy.Publisher('stiffness_value', Float32)
	while not rospy.is_shutdown():
		rospy.loginfo(stiffness)
		pub.Publish(stiffness)
		rate.sleep()

if __name__ == "__main__":
	
	try:
		communicate()
	except rospy.ROSInterruptException:
		pass





