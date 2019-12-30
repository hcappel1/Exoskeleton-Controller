import numpy as np
import matplotlib.pyplot as plt
import itertools as IT
import sympy as sym
import csv
import pandas as pd
import scipy.optimize as opt




def data(reader, tf,fq):
#tf: number of data to generate
#fq: num of total data ponts/num of data to generate
#0:time 2:actuator force 3:joint torque 4:Tc cuff torque 5:joint position
#6:motor current 7:actuator position 8:cmd torque
#9:gravity cmd 10:amplification cmd 11:chirp cmd 12:chirp power


    T  = [] #relative time
    Th = [] #joint position
    Tc = [] #cuff torque
    Ts = [] #actuator torque
    V  = [] #absolute time

    i  = 0

    t  = 0.
    th = 0.
    tc = 0.
    ts = 0.
    v  = 0.

    for row in reader:

        if i != 0 and t <= tf:

            t = t + 1/fq

            # joint angle
            if np.isnan(float(row[5])):
                th = th * 1.

            else:
                th = float(row[5])

            # cuff torque
            if np.isnan(float(row[4])):
                tc = tc * 1.

            else:
                tc = float(row[4])

            # spring torque
            if np.isnan(float(row[3])):
                ts = ts * 1.

            else:
                ts = float(row[3])

            if np.isnan(float(row[0])):
                v = v * 1.

            else:
                v = float(row[0])


            T .append([t ])
            Th.append([th])
            Tc.append([tc])
            Ts.append([ts])
            V .append([v])

        i = i + 1

        # display the percentage of the process
        if i % int(fq * tf / 10.) == 0.:

            print ('data:', int((t / tf) * 10.) * 10)

        # collect data points into array
    T  = np.array(T )
    T .resize(T .shape[0])
    Th = np.array(Th)
    Th.resize(Th.shape[0])
    Tc = np.array(Tc)
    Tc.resize(Tc.shape[0])
    Ts = np.array(Ts)
    Ts.resize(Ts.shape[0])
    V = np.array(V)
    V.resize(V.shape[0])

    return T, Th, Tc, Ts, V

#open file and read data
file   = open('log10L00B00A2(2).csv', 'rt')
reader = csv.reader(file)

tf = 240.
fq = 1000.

T , Th, Tc, Ts, V = data(reader, tf, fq)



#simulation data
def simdata(time,a,b,x0,v0):
	noise = np.random.normal(0,0.2,2000)
	torque = np.zeros(2000)
	torque[0] = 5
	return np.exp(-a*time)*(((v0+a*x0)/b)*np.sin(b*time)+x0*np.cos(b*time))+noise, torque

#section off time and position data into separate arrays for each impulse response
def separate(Ts,Th,T):
	time = []
	pos = []
	v0 = []
	x0 = []
	i = 0
	while i < len(Ts):
		if Ts[i] > 4:
			v0.append(1)
			x0.append(Th[i])
			T_sample = []
			Th_sample = []
			l = i
			while abs(Th[l]) >= 0.01:
				T_sample.append(T[l])
				Th_sample.append(Th[l])
				l = l + 1
			n = 0
			T_sampnorm = []
			while n < len(T_sample):
				T_samp = T_sample[n] - T_sample[0]
				T_sampnorm.append(T_samp)
				n = n+1
			time.append(T_sampnorm)
			pos.append(Th_sample)
		i = i+1
	return time, pos, v0, x0


#create function to be estimated via least squares
def function(T,a,b,x0,v0):
    return np.exp(-a*T)*(((v0+a*x0)/b)*np.sin(b*T)+x0*np.cos(b*T))


#least squares regression
def regression(time,pos,v0,x0,function,i):
	p0 = [0.1,0.1,x0[i],v0[i]]
	w, _= opt.curve_fit(function,time[i],pos[i], p0=p0)
	print("Estimated Parameters", w)
	return w

#based on regression determine damping and stiffness values into array
def params(params):
	damping = []
	stiffness = []
	m = 0.5
	i = 0
	while i < len(params):
		damp = 2*m*params[0]
		stiff = m*(params[1]**2 + params[0])
		damping.append(damp)
		stiffness.append(stiff)
		i = i+1
	return stiffness, damping

#return position,velocity,stiffness,damping,torque of same size vectors
def organize(stiff,damp,times,pos):
	tstiffness = []
	tdamping = []
	i = 0
	while i < len(times):
		kval = stiff[i][0]
		bval = damp[i][1]
		k = np.full(len(times[i]),kval)
		b = np.full(len(times[i]),bval)
		tstiffness.append(k)
		tdamping.append(b)
		i = i + 1
	return tstiffness, tdamping, times, pos



if __name__ == '__main__':
	time = np.linspace(0,8,num=2000)
	a = 0.5
	b = -0.5
	x0 = 3
	v0 = 1

	simdata1, torque1 = simdata(time,a,b,x0,v0)

	simdata2, torque2 = simdata(time,a,b,x0,v0)

	simdata3, torque3 = simdata(time,a,b,x0,v0)

	simdata = np.concatenate((simdata1,simdata2,simdata3))
	torque = np.concatenate((torque1,torque2,torque3))


	time1 = np.linspace(0,8,num=2000)
	time2 = np.linspace(8,16,num=2000)
	time3 = np.linspace(16,24,num=2000)

	t = np.concatenate((time1,time2,time3))

	
	times, pos, v0, x0 = separate(torque,simdata,t)
	i = 0
	parameters = []
	while i < len(times):
		p = regression(times,pos,v0,x0,function,i)
		parameters.append(p)
		i = i + 1

	stiffness, damping = params(parameters)

	fstiffness,fdamping,ftime,fposition = organize(stiffness,damping,times,pos)

	
	t1 = times[1]
	pos1 = pos[1]
	v01 = v0[0]
	x01 = x0[0]

	plt.plot(t,simdata)
	plt.plot(t1,pos1)
	plt.show()





	
