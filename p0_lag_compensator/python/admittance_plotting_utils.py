# Gray Thomas, NSTRF15AQ33H at NASA JSC August 2018
# Works in python 3
# import h5py
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from admittance_plotting_utils import *
from collections import namedtuple

class DataSet(object):
    def __init__(self, filename):
        self.get_data(filename)
        self.inferr_additional_data()

    def get_data(self, filename):
        with open('/home/apptronik/log/'+filename+".csv", 'r') as f:
            labels = [st.strip() for st in next(f).split(",")]
            data = np.genfromtxt(f, delimiter=",", skip_header=10)
        print (labels)

        # assert labels == [
        # 'time', 
        # '/Axon_proto_v1/miso/diag__faults__x', 
        # '/Axon_proto_v1/miso/actuator__force__N', 
        # '/Axon_proto_v1/miso/js__joint__effort__Nm', 
        # '/Axon_proto_v1/miso/cuff__ati__torque__Nm', 
        # '/Axon_proto_v1/miso/js__joint__position__rad', 
        # '/Axon_proto_v1/miso/motor__current__A', 
        # '/Axon_proto_v1/miso/actuator__position__m', 
        # '/Axon_proto_v1/miso/motor__position__Rad', 
        # '/Axon_proto_v1/miso/motor__velocity__Radps', 
        # '/Axon_proto_v1/mosi/cmd__joint__effort__Nm']
        for i in range(1, len(labels)):
            x = np.array(list(data[:,i]))
            xi = np.array(np.arange(len(x)))
            mask = np.isfinite(x)
            xfiltered = np.interp(xi, xi[mask], x[mask])
            data[:,i] = xfiltered

        self.force = np.array(data[:, [0, 2]]) # Newtons (spring)
        self.torque = np.array(data[:, [0, 3]]) # Newton meters (spring)
        self.loadtorque = np.array(data[:, [0, 4]]) # Newton meters (cell)
        self.joint = np.array(data[:, [0, 5]]) # rad
        self.current = np.array(data[:, [0, 6]]) # amps
        self.actuator = np.array(data[:, [0, 7]]) # meters
        self.motor = np.array(data[:, [0, 8]]) # Rad (motor-side)
        self.motor_vel = np.array(data[:, [0, 9]]) # Rad / s (motor-side)
        self.cmd = np.array(data[:, [0, 10]]) # Newton meters (desired)

    def inferr_additional_data(self):
        self.infjac = np.array(self.force)
        self.infloadforce = np.array(self.force)

        for i in range(len(self.force)):
            jacobian = self.torque[i,1]/self.force[i,1]
            self.infjac[i,1] = jacobian
            self.infloadforce[i,1] = -self.loadtorque[i,1]/jacobian
        

# f = h5py.File('2018_08_06_183828_test_logger[-0.0,-0.0,-0.0,-0.0].hdf5')
# open('/home/gray/hdf5_logs/2018_08_06_184256_test_logger.hdf5', 'r')
# f = h5py.File('/home/gray/hdf5_logs/2018_08_06_184256_test_logger.hdf5')
sample_rate = 1000.0 # both at nasa and UT (apptronik p0)
def fft_plot(signal):
    sample_rate = 1000.0
    A = np.fft.rfft(signal[:,1])/(sample_rate)
    f = np.fft.rfftfreq(len(signal), d=1./sample_rate)
    return A, f

def angle(n):
    ang = [a if a<=1e-6 else a-2*np.pi for a in np.angle(n)]
    return np.array(ang)


def plot_discrete(data):
    print(data)
    time = [0.0]
    value = [data[0,1]]
    time0 = data[0,0]
    for t, v in data[1:]:
        time.append(t-time0)
        time.append(t-time0)
        value.append(value[-1])
        value.append(v)
    print(len(time), len(value))
    return time, value

def average_tf(ins, n=10):
    # this doesn't seem to help
    kern = np.ones((n,))/n
    outs = []
    for ith_input in ins:
        outs.append(np.convolve(ith_input[1:], kern))
    return outs

def get_cross_and_power(signalA, signalB):
    fftA, f = fft_plot(signalA)
    fftB, fB = fft_plot(signalB)
    assert len(f)==len(fB)
    return fftA*fftB.conjugate(), (fftB*fftB.conjugate()), f

def tf_A_over_B(signalA, signalB, eps=(1e4, 1e8), ax=plt, **kwargs):
    # fftA, f = fft_plot(signalA)
    # fftB, fB = fft_plot(signalB)
    # assert len(f)==len(fB)
    # transfer = fftA*fftB.conjugate() / (fftB*fftB.conjugate())
    cross, power, f = get_cross_and_power(signalA, signalB)
    transfer = cross / power
    scatter_bode_mag(f, transfer, power, eps=eps, ax=ax, **kwargs)

def scatter_bode_mag(f, transfer, power, eps=(1e4, 1e8), ax=plt, **kwargs):
    normalizer = colors.Normalize(vmin=np.log(eps[0]), vmax=np.log(eps[1]), clip=False)
    input_power = normalizer(np.log(np.abs(power)))
    # transfer = np.abs(fftA) / np.abs(fftB)
    ax.loglog(f, abs(transfer), alpha=0.5, **kwargs)
    ax.scatter(f, abs(transfer), marker='.', c=input_power, cmap="binary")

def scatter_bode_phase(f, transfer, power, eps=(1e4, 1e8), ax=plt, **kwargs):
    normalizer = colors.Normalize(vmin=np.log(eps[0]), vmax=np.log(eps[1]), clip=False)
    input_power = normalizer(np.log(np.abs(power)))
    # transfer = np.abs(fftA) / np.abs(fftB)
    ax.semilogx(f, 180./np.pi*angle(transfer), alpha=.5, **kwargs)
    ax.scatter(f, 180./np.pi*angle(transfer), marker='.', c=input_power, cmap="binary")

    if ax is plt:
        ax.yticks([-360,-270,-180, -90, 0])
        ax.gca().yaxis.set_minor_locator(matplotlib.ticker.MultipleLocator(30))
    else:
        ax.set_yticks([-360,-270,-180, -90, 0])
        ax.yaxis.set_minor_locator(matplotlib.ticker.MultipleLocator(30))

def tf_A_over_Bf_split(signalA, signalB, split=500, eps=(1e4, 1e8)):
    n_splits = int(len(signalA)/split)
    cross, power = 0., 0.
    for i in range(n_splits):
        fftA, f = fft_plot(signalA[i*split:(i+1)*split])
        fftB, fB = fft_plot(signalB[i*split:(i+1)*split])
        cross = (fftA*fftB.conjugate())[1:] + cross
        power = (fftB*fftB.conjugate())[1:] + power
        
        assert len(f)==len(fB)
    transfer = cross/power
    normalizer = colors.Normalize(vmin=np.log(eps[0]), vmax=np.log(eps[1]), clip=False)
    input_power = normalizer(np.log(np.abs(power)))
    # transfer = np.abs(fftA) / np.abs(fftB)
    plt.loglog(f[1:], abs(transfer), lw=3, alpha=.8)
    # plt.scatter(f[1:], abs(transfer), marker='.', c=input_power, cmap="bone")

def tf_A_over_B_phase(signalA, signalB, eps=(1e4, 1e8), ax=plt, **kwargs):
    cross, power, f = get_cross_and_power(signalA, signalB)
    transfer = cross / power
    scatter_bode_phase(f, transfer, power, eps=eps, ax=ax, **kwargs)

def tf_A_over_Bf_split_phase(signalA, signalB, split=500, eps=(1e4, 1e8)):
    n_splits = int(len(signalA)/split)
    cross, power = 0., 0.
    for i in range(n_splits):
        fftA, f = fft_plot(signalA[i*split:(i+1)*split])
        fftB, fB = fft_plot(signalB[i*split:(i+1)*split])
        cross = fftA*fftB.conjugate() + cross
        power = fftB*fftB.conjugate() + power
        
        assert len(f)==len(fB)
    transfer = cross/power
    normalizer = colors.Normalize(vmin=np.log(eps[0]), vmax=np.log(eps[1]), clip=False)
    input_power = normalizer(np.log(np.abs(power)))
    # transfer = np.abs(fftA) / np.abs(fftB)
    plt.semilogx(f[1:], 180./np.pi*angle(transfer[1:]), lw=3, alpha=.8)
    # plt.scatter(f, 180./np.pi*angle(transfer), marker='.', c=input_power, cmap="bone")
    plt.yticks([-360,-270,-180, -90, 0])
    plt.gca().yaxis.set_minor_locator(matplotlib.ticker.MultipleLocator(30))





# my_data = genfromtxt('my_file.csv', delimiter=',')


# print(dir(g["JointAPS_Angle_Rad"]))
# print(ang[0])
# print(np.array(ang))
# plt.plot(np.array(ang)[:,0]-np.array(ang)[0,0], np.array(ang)[:,1])

def test_fft():
    N = 500000
    x = np.cos(np.linspace(0, 2*np.pi, N+1)[:-1])
    A = np.fft.rfft(x)/(N/2.)
    print (A[1])
    # This demonstrates the need for scaling by 1/(N/2) to preserve intuition about fft magnitudes.
    # But is normally a measure of the integral, so I'm adjusting it to be in units of time*signalunits
    # this also hands the 1/2 factor, since the integral of a squared sinusoid is 1/2 t.


def exp_noise_power_plot(exp, noise, title):
    plt.figure()
    A, f = fft_plot(exp)
    plt.loglog(f, np.abs(A)**2, label="Experiment")
    A, f = fft_plot(noise)
    plt.loglog(f, np.abs(A)**2, label="Noise")
    plt.legend()
    plt.title(title)

def two_label_power_plot(exp, noise, title, label1, label2):
    plt.figure()
    A, f = fft_plot(exp)
    plt.loglog(f, np.abs(A)**2, label=label1)
    A, f = fft_plot(noise)
    plt.loglog(f, np.abs(A)**2, label=label2)
    plt.legend()
    plt.title(title)



def fft_power_info_plots(exp_file, noise_file):
    node, ang, vel, torque, load, current, _ = get_data(exp_file)
    node1, ang1, vel1, torque1, load1, current1, _ = get_data(noise_file)
    
    exp_noise_power_plot(torque, torque1, "Torque (Spring) Power Spectrum")
    exp_noise_power_plot(load, load1, "Torque (Load Cell) Power Spectrum")
    exp_noise_power_plot(ang, ang1, "Position Power Spectrum")

def empty_admittance_plots(suptitle=""):
    figure_1 = plt.figure()
    eps = (1e-1, 1e4)
    plt.title("Position (Admittance) Magnitude")
    plt.suptitle(suptitle)
    plt.grid(True)

    figure_2 = plt.figure()

    # transfer = np.abs(fftA) / np.abs(fftB)

    plt.yticks([-360,-270,-180, -90, 0])
    plt.gca().yaxis.set_minor_locator(matplotlib.ticker.MultipleLocator(30))

    plt.title("Position (Admittance) Phase")
    plt.suptitle(suptitle)
    plt.grid(True)
    return figure_1, figure_2

def empty_bode_plot(suptitle=""):
    fig, (ax_mag, ax_phase) = plt.subplots(2, 1, sharex=True)
    ax_mag.set_title("Position-Admittance Magnitude")
    plt.suptitle(suptitle)
    ax_mag.grid(True)

    # transfer = np.abs(fftA) / np.abs(fftB)

    ax_phase.set_yticks([-360,-270,-180, -90, 0])
    ax_phase.yaxis.set_minor_locator(matplotlib.ticker.MultipleLocator(30))
    ax_phase.set_xlabel("Frequency (Hz)")

    ax_phase.set_title("Position-Admittance Phase")
    ax_phase.grid(True)
    return ax_mag, ax_phase

def plot_admittance(ang, load, suptitle=""):

    figure_1 = plt.figure()
    eps = (1e-1, 1e4)
    tf_A_over_B(ang, load, eps=eps)
    # tf_A_over_Bf_split(ang, load, split=3000, eps=eps)
    # tf_A_over_Bf_split(ang, load, split=1000, eps=eps)
    # tf_A_over_Bf_split(ang, load, split=300, eps=eps)
    # tf_A_over_Bf_split(ang, load, split=100, eps=eps)
    # tf_A_over_Bf_split(ang, load, split=30, eps=eps)
    plt.title("Position (Admittance) Magnitude")
    plt.suptitle(suptitle)
    plt.grid(True)

    figure_2 = plt.figure()
    tf_A_over_B_phase(ang, load, eps=eps)
    # tf_A_over_Bf_split_phase(ang, load, split=3000, eps=eps)
    # tf_A_over_Bf_split_phase(ang, load, split=1000, eps=eps)
    # tf_A_over_Bf_split_phase(ang, load, split=300, eps=eps)
    # tf_A_over_Bf_split_phase(ang, load, split=100, eps=eps)
    # tf_A_over_Bf_split_phase(ang, load, split=30, eps=eps)
    plt.title("Position (Admittance) Phase")
    plt.suptitle(suptitle)
    plt.grid(True)
    return figure_1, figure_2


def plot_spring_admittance(torque, load, suptitle=""):
    plt.figure()
    eps = (1e-1, 1e4)
    tf_A_over_B(torque, load, eps=eps)
    plt.title("Spring (Position Admittance) Magnitude")
    plt.suptitle(suptitle)

    plt.figure()

    tf_A_over_B_phase(torque, load, eps=eps)
    plt.title("Spring (Position Admittance) Phase")
    plt.suptitle(suptitle)

def comparison_of_test_quality(file1, file2, name1, name2):
    _, ang, _, torque, load, current, _ = get_data(file1)
    _, ang1, _, torque1, load1, current1, _ = get_data(file2)
    two_label_power_plot(torque, torque1,"Torque (Spring) Power Spectrum", name1, name2)
    two_label_power_plot(load, load1,"Torque (Load Cell) Power Spectrum", name1, name2)
    two_label_power_plot(ang, ang1,"Position Power Spectrum", name1, name2)
    two_label_power_plot(current, current1,"Current Power Spectrum", name1, name2)

def n_label_power_plot(title, list_of_data):
    plt.figure()
    for data, label in list_of_data:
        A, f = fft_plot(data)
        plt.loglog(f, np.abs(A)**2, label=label)
    plt.legend()
    plt.title(title)

def n_way_comparison_of_test_quality(list_in):
    files, names = zip(*list_in)
    _, angs, _, torques, loads, currents, _ = zip(*[get_data(file) for file in files])

    n_label_power_plot("Torque (Spring) Power Spectrum", zip(torques, names))
    n_label_power_plot("Torque (Load Cell) Power Spectrum", zip(loads, names))
    n_label_power_plot("Position Power Spectrum", zip(angs, names))
    n_label_power_plot("Current Power Spectrum", zip(currents, names))


def saturation_noise(file, gains, axmag, axphase):
    m_o = .29

    # work well on the no control tests
    m_m = .88/2
    m_s = 0.0
    b_s = 0.0

    k_s = 1180.
    b_m = 15.9+2 # pretty non linear
    deflection, ang, inf_ang, torque, load, current, motor = get_data(file)
    sat_error = saturation_error(file, gains, threshold=8)
    cross, power, f = get_cross_and_power(sat_error, load)
    w = np.pi*2*f
    s = complex(0, 1)*w
    lam = -np.log(.5)*5000
    lp_s = s/(s/lam+1)
    delay = np.exp(-s*0.0002)
    Zm_prime = m_m*s*s+b_m*s-mgain*(kd*lp_s+kp)
    Zsensitivity = mgain/Zm_prime
    transfer = cross / power * Zsensitivity

    scatter_bode_mag(f, transfer, power, ax=axmag)
    scatter_bode_phase(f, transfer, power, ax=axphase)

def filter_lp_diff(data, alpha=.5, kd=1e3): # default to the ones used on the chip
    # on the chip: .5, 5e3, 5000Hz
    # through the system we only get data at 1000Hz
    res = np.array(data)
    filter_x = data[0,1]
    for i in range(1, len(data)):
        y = data[i, 1]
        update = (1-alpha)*(y-filter_x)
        filter_x = alpha*filter_x+(1-alpha)*y
        res[i,1]= kd*update
    return res

def combine_data(datalist, gainlist):
    res = np.array(datalist[0])
    res[:,1]*=0.
    for data, gain in zip(datalist, gainlist):
        res[:,1]+=gain*data[:,1]
    return res

def saturate_data(data, lim=1):
    ndata = np.array(data)
    for i in range(len(data)):
        ndata[i,1]= -lim if data[i,1]<-lim else lim if data[i,1]>lim else data[i,1]
    return ndata

def op_data(data, op):
    ndata = np.array(data)
    for i in range(len(data)):
        ndata[i,1]= op(data[i,1])
    return ndata


def saturation_error(test, gains, threshold=8):
    deflection1, ang1, inf_ang1, torque1, load1, current1, motor1 = get_data(test)
    motordot = filter_lp_diff(motor1)
    deflectiondot = filter_lp_diff(deflection1)
    current_expected = combine_data([motor1, deflection1, motordot, deflectiondot],gains)
    satcurrent = saturate_data(current_expected, lim=threshold)
    saturation_error = combine_data([satcurrent, current_expected],[1, -1])
    return saturation_error


def ez_saturation_plot(test, gains, threshold=8):
    name = test[test.find("["):]
    deflection1, ang1, inf_ang1, torque1, load1, current1, motor1 = get_data(test)
    motordot = filter_lp_diff(motor1)
    deflectiondot = filter_lp_diff(deflection1)
    current_expected = combine_data([motor1, deflection1, motordot, deflectiondot],gains)
    satcurrent = saturate_data(current_expected, lim=threshold)
    plt.figure()
    cross, power, f = get_cross_and_power(satcurrent, current_expected)
    transfer = cross / power
    scatter_bode_mag(f, transfer, power)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")
    plt.title("Saturation Describing Function")
    plt.grid(True)
    plt.suptitle("Gains: [%.1f,%.1f,%.1f,%.1f]"%tuple(gains))

