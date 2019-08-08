
import numpy as np
import pickle
from sklearn.ensemble import RandomForestRegressor
from sklearn.externals import joblib
def load():
	dt = RandomForestRegressor(n_estimators=100, random_state=False, verbose=False)
	filename = 'forest_full_v4.pkl'
	dt = pickle.load(open(filename, 'rb'))
	return dt

def add(a,dt):
	
	a= np.reshape(a, (1, 7))
	# # dt = joblib.load('my_model.h5')
	# # print (a.shape)
	y_pred =  dt.predict(a)
	# print (y_pred)
	return y_pred[0]

	# print (a)
	# return a

if __name__ == "__main__":
	a=[1,2,3,4,5,6,7]
	dt = load()
	b=add(a,dt)
	print(b)

