# Gray Thomas, NSTRF15AQ33H at NASA JSC August 2018
# Works in python 3
import h5py
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.colors as colors
# f = h5py.File('2018_08_06_183828_test_logger[-0.0,-0.0,-0.0,-0.0].hdf5')
# open('/home/gray/hdf5_logs/2018_08_06_184256_test_logger.hdf5', 'r')
# f = h5py.File('/home/gray/hdf5_logs/2018_08_06_184256_test_logger.hdf5')
sample_rate = 1000.0
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

def tf_A_over_B(signalA, signalB, eps=(1e4, 1e8)):
    # fftA, f = fft_plot(signalA)
    # fftB, fB = fft_plot(signalB)
    # assert len(f)==len(fB)
    # transfer = fftA*fftB.conjugate() / (fftB*fftB.conjugate())
    cross, power, f = get_cross_and_power(signalA, signalB)
    transfer = cross / power
    scatter_bode_mag(f, transfer, power, eps=eps)

def scatter_bode_mag(f, transfer, power, eps=(1e4, 1e8), ax=plt):
    normalizer = colors.Normalize(vmin=np.log(eps[0]), vmax=np.log(eps[1]), clip=False)
    input_power = normalizer(np.log(np.abs(power)))
    # transfer = np.abs(fftA) / np.abs(fftB)
    ax.loglog(f, abs(transfer), alpha=0.5)
    ax.scatter(f, abs(transfer), marker='.', c=input_power, cmap="binary")

def scatter_bode_phase(f, transfer, power, eps=(1e4, 1e8), ax=plt):
    normalizer = colors.Normalize(vmin=np.log(eps[0]), vmax=np.log(eps[1]), clip=False)
    input_power = normalizer(np.log(np.abs(power)))
    # transfer = np.abs(fftA) / np.abs(fftB)
    ax.semilogx(f, 180./np.pi*angle(transfer), alpha=.5)
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

def tf_A_over_B_phase(signalA, signalB, eps=(1e4, 1e8)):
    cross, power, f = get_cross_and_power(signalA, signalB)
    transfer = cross / power
    scatter_bode_phase(f, transfer, power, eps=eps)

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

def get_data(filename):
    with h5py.File('/home/gray/hdf5_logs/%s.hdf5'%filename) as f:
    # with h5py.File('/home/gray/hdf5_logs/test.hdf5') as f:
        assert len(f)>0 # check for wrong file name.

        print(f)
        print(dir(f))
        print(f.id)
        print(f.update())
        print(list(f.keys()))
        print(len(f))
        print([h for h in f.items()])
        print(f["testbench/j0"])
        node = f["testbench/j0"]
        ang = np.array(node["JointAPS_Angle_Rad"])
        vel = np.array(node["JointAPS_Vel_Radps"])
        torque = np.array(node["JointTorque_Meas_Nm"])
        deflection = np.array(node["SpringDef_Meas_Rad"])
        motor = np.array(node["IncEnc_Angle_Rad"])
        load = np.array(node["LoadCell_Nm"])
        try:
            current = np.array(node["Current_Abs_Amps"])
        except KeyError:
            current = np.array(node["CurrentQ_Des_Amps"])
        inf_ang = motor - deflection # in theory, this is a measurement of actuator angle.
        inf_ang[:,0] = motor[:, 0]
        # heart = np.array(node["Proc_HeartBeat"])
    return deflection, ang, inf_ang, torque, load, current, motor

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

# if __name__ == '__main__':
#     test_fft()
#     exit()

def basic_plots(file):
    deflection, ang, vel, torque, load, current, motor = get_data(file)
    plt.figure()
    plt.plot(*plot_discrete(np.array(ang)))
    plt.title("Angle")

    plt.figure()
    plt.plot(*plot_discrete(np.array(vel)))
    plt.title("Velocity")

    plt.figure()
    plt.plot(*plot_discrete(np.array(deflection)))
    plt.title("Spring Deflection")

    plt.figure()
    plt.plot(*plot_discrete(np.array(torque)))
    plt.title("Torque (Spring)")

    plt.figure()
    plt.plot(*plot_discrete(np.array(load)))
    plt.title("Torque (Load Cell)")

    plt.figure()
    plt.plot(*plot_discrete(np.array(current)))
    plt.title("Current")

    plt.figure()
    plt.plot(*plot_discrete(np.array(motor)))
    plt.title("Motor Angle")

def fft_power_plots(title, file):
    deflection, ang, vel, torque, load, current, motor = get_data(file)
    plt.figure()
    A, f = fft_plot(torque)
    plt.loglog(f, np.abs(A)**2*())
    plt.title("Torque (Spring) Power Spectrum")
    plt.suptitle(title)

    plt.figure()
    A, f = fft_plot(load)
    plt.loglog(f, np.abs(A)**2)
    plt.title("Torque (Load Cell) Power Spectrum")
    plt.suptitle(title)

    plt.figure()
    A, f = fft_plot(ang)
    plt.loglog(f, np.abs(A)**2)
    plt.title("Position Power Spectrum")
    plt.suptitle(title)

    plt.figure()
    A, f = fft_plot(vel)
    plt.loglog(f, np.abs(A)**2)
    plt.title("Velocity Power Spectrum")
    plt.suptitle(title)

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


def initial_testing():
    # fft_power_plots("Noise Baseline", "noise_test")
    # fft_power_plots("Experiment", "test")
    fft_power_info_plots("test", "noise_test")

    # introduces a random test I had previously done:
    _, ang, _, torque, load, current, _ = get_data("2018_08_07_145945_test_logger[-0.0,-25000.0,-10.0,-50.0]")
    # fft_power_info_plots("2018_08_07_145945_test_logger[-0.0,-25000.0,-10.0,-50.0]", "noise")
    # plot_admittance(ang, load, suptitle="Recent Controller")

    _, ang, _, torque, load, current, _ = get_data("test")
    plot_admittance(ang, load, suptitle="No Current")
    # plot_spring_admittance(torque, load, suptitle="recent")

    eps = (1e-2, 1e1)
    # tf_A_over_B(ang, load, eps=eps)
    # tf_A_over_Bf_split(ang, load, split=1000, eps=eps)
    # _, ang1, _, torque1, load1, current1, _ = get_data("test")
    # two_label_power_plot(torque, torque1,"Torque (Spring) Power Spectrum", "Recent Exp", "No Current")
    # two_label_power_plot(load, load1,"Torque (Load Cell) Power Spectrum", "Recent Exp", "No Current")
    # two_label_power_plot(ang, ang1,"Position Power Spectrum", "Recent Exp", "No Current")
    # two_label_power_plot(current, current1,"Current Power Spectrum", "Recent Exp", "No Current")
    plt.show()

def test_almost_unstable():
    # This test is designed to demonstrate a boarderline stable positive velocity feedback
    # which should help up identify the motor damping (times the input gain?)
    no_control_pushing = "test"
    pushing = "2018_08_07_163110_test_logger[-0.0,0.0,3.05,0]" # pushing
    tapping = "2018_08_07_163138_test_logger[-0.0,0.0,3.05,0]" # tapping
    free_oscillating = "2018_08_07_173352_test_logger[-20,0.0,3.05,0]"
    forced_oscillating = "2018_08_07_185649_test_logger[-20,0.0,2.8,0]"
    # forced_oscillating = "2018_08_07_185649_test_logger[-20,0.0,2.8,0]"
    forced_oscillating = "2018_08_07_185922_test_logger[-20,0.0,2.95,0]"
    stiff_motor_side = "2018_08_07_190838_test_logger[-2000,0.0,-30.0,0]"

    # _, ang, _, torque, load, current, _ = get_data(tapping)
    # plot_admittance(ang, load, suptitle="Boarderline Controller (Tapping)")
    # _, ang, _, torque, load, current, _ = get_data(pushing)
    # plot_admittance(ang, load, suptitle="Boarderline Controller (Pushing)")
    # _, ang, _, torque, load, current, _ = get_data(forced_oscillating)
    # comparison_of_test_quality(pushing, forced_oscillating, "Pushing", "Forcing")
    # plot_admittance(ang, load, suptitle="Oscillating Controller (Forced!)")
    # _, ang, _, torque, load, current, _ = get_data("test")
    # plot_admittance(ang, load, suptitle="No Current")
    comparison_of_test_quality(pushing, tapping, "Pushing", "Tapping")

    plt.show()



def test_stiff_motor():
    # This test should help us identify the natural spring dynamics
    # Since these spring dynamics should dominate the admittanc plot.
    no_control_pushing = "test"
    stiff_motor_side = "2018_08_07_190838_test_logger[-2000,0.0,-30.0,0]"
    _, ang, _, torque, load, current, _ = get_data(stiff_motor_side)
    # comparison_of_test_quality(no_control_pushing, stiff_motor_side, "No Control", "Stiff Motor")
    plot_admittance(ang, load, suptitle="Stiff Motor Controller (Forced!)")

    plt.show()

def stiffer_motor():
    # Our last test was a little too close to the identified damping
    # so this one should separate the two out.
    forced = "2018_08_08_123754_test_logger[-20000,0.0,-30.0,0]" # forced
    crowbar = "2018_08_08_123828_test_logger[-20000,0.0,-30.0,0]" # crowbar
    _ = "2018_08_08_123911_test_logger[-20000,0.0,-30.0,0]" # squib
    ruler = "2018_08_08_123937_test_logger[-20000,0.0,-30.0,0]" # ruler
    extra_data = "2018_08_08_131528_test_logger[-20000,0.0,-30.0,0]"
    extra_extra_data = "2018_08_08_132606_test_logger[-20000,0.0,-30.0,0]"
    extra_extra_data = "2018_08_08_133040_test_logger[-20000,0.0,-30.0,0]" # hopefully more informative
    extra_extra_data = "2018_08_08_150852_test_logger[-20000,0.0,-30.0,0]" # forgot to fix lp filter
    extra_extra_data = "2018_08_08_151412_test_logger[-20000,0.0,-30.0,0]" # Fixed!
    extra_extra_data = "2018_08_08_152745_test_logger[-20000,0.0,-30.0,0]" # some ruler
    # n_way_comparison_of_test_quality([(forced, "Forced"), (crowbar, "Crowbar"), (ruler, "Ruler")])
    # deflection, ang, _, torque, load, current, _ = get_data(extra_data)
    deflection, ang, inf_ang, torque, load, current, motor = get_data(extra_extra_data)
    plot_admittance(ang, load, suptitle="Actuator Admittance")
    plot_admittance(inf_ang, load, suptitle="(Inferred) Actuator Admittance")
    plot_admittance(motor, load, suptitle="Motor Admittance")
    plot_admittance(deflection, load, suptitle="Spring Admittance")
    plt.show()

def boosted_admittance():
    # Seeing no mass line for the spring, we can't reduce admittance.
    # But we can boost it above the default and treat it as low mass.
    boost1 = "2018_08_08_160039_test_logger[-20000,-20000.0,-30.0,-30]"
    boost2 = "2018_08_08_161257_test_logger[-20000,-40000.0,-30.0,-60]"
    boost3 = "2018_08_08_161542_test_logger[-20000,-60000.0,-30.0,-90]"
    baseline_negative_damping = "2018_08_08_162226_test_logger[0.0,0.0,2.95,0]"
    deflection, ang, inf_ang, torque, load, current, motor = get_data(boost3)
    # plot_admittance(ang, load, suptitle="Actuator Admittance")
    plot_admittance(inf_ang, load, suptitle="(Inferred) Actuator Admittance")
    # plot_admittance(deflection, load, suptitle="Spring Admittance")
    # plot_admittance(motor, load, suptitle="Motor Admittance")
    # deflection, ang, inf_ang, torque, load, current, motor = get_data(baseline_negative_damping)
    # plot_admittance(ang, load, suptitle="Baseline (Negative damping)")
    plt.show()

def finding_the_added_inertia():
    # we have added intertia to the outside of the force sensor. This should enable
    # more agressive testing, but it requires identification of this inerita first.
    first_test = "2018_08_09_101626_test_logger[-20,0.0,2.95,0]"
    test_2 = "2018_08_09_102417_test_logger[-20000,0.0,-30.0,0]"
    deflection, ang, inf_ang, torque, load, current, motor = get_data(first_test)
    plot_admittance(ang, load, suptitle="First Output Inertia Test")
    deflection, ang, inf_ang, torque, load, current, motor = get_data(test_2)
    plot_admittance(ang, load, suptitle="Second Output Inertia Test")
    # n_way_comparison_of_test_quality([(first_test, "First Test"), (test_2, "Second Test")])
    plt.show()
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
def model_checking():
    tests = [
        # ("2018_08_08_162226_test_logger[0.0,0.0,2.95,0]", [0.0, 0.0, 2.95, 0.0]),
        # ("2018_08_08_150852_test_logger[-20000,0.0,-30.0,0]", [-20000,0.0,-30.0,0]),
        # ("2018_08_10_085301_test_logger[-20000.0,0.0,0.0,0.0]", [-20000.0,0.0,0.0,0.0]), # sanity check on input behavior 1
        # ("2018_08_10_085457_test_logger[0.0,-60000.0,0.0,0.0]" ,[0.0,-60000.0,0.0,0.0] ),
        # ("2018_08_10_085541_test_logger[0.0,0.0,-30.0,0.0]", [0.0,0.0,-30.0,0.0]),
        # ("2018_08_10_085640_test_logger[0.0,0.0,0.0,-90]",[0.0,0.0,0.0,-90]),
        # ("2018_08_10_091426_test_logger[-2000.0,0.0,0.0,0.0]", [-2000.0,0.0,0.0,0.0]), # just checks to see if discrepency is due to saturation
        # ("2018_08_10_094006_test_logger[-2000.0,0.0,0.0,0.0]", [-2000.0,0.0,0.0,0.0]), # I push until it current saturates
        # ("2018_08_09_143715_test_logger[0.0,0.0,2.95,0]", [0.0,0.0,2.95,0]),
        # ("2018_08_08_151412_test_logger[-20000,0.0,-30.0,0]", [-20000,0.0,-30.0,0]),
        # ("2018_08_08_160039_test_logger[-20000,-20000.0,-30.0,-30]", [-20000,-20000.0,-30.0,-30]),
        # ("2018_08_08_161257_test_logger[-20000,-40000.0,-30.0,-60]", [-20000,-40000.0,-30.0,-60]),
        # ("2018_08_08_161542_test_logger[-20000,-60000.0,-30.0,-90]", [-20000,-60000.0,-30.0,-90]),
        # ("2018_08_09_144205_test_logger[-20000,-60000.0,-30.0,-90]", [-20000,-60000.0,-30.0,-90]), # repeated hammering with weights
        ("2018_08_09_144904_test_logger[-20000,-60000.0,-30.0,-90]", [-20000,-60000.0,-30.0,-90]), # more hammering
        # ("2018_08_09_151715_test_logger[-20000,-60000.0,-30.0,-90]", [-20000,-60000.0,-30.0,-90]), # with good saturation model
        # ("2018_08_10_094832_test_logger[-20000,-60000.0,-30.0,-90]", [-20000,-60000.0,-30.0,-90]), # hopefully less saturation due to gentle test
        ]

    # m_m = .88
    m_o = .29

    # work well on the no control tests
    m_m = .88/2
    m_s = 0.0
    b_s = 0.0

    k_s = 1180.
    b_m = 15.9+2 # pretty non linear






    f = np.logspace(-1, 3, 1000)
    w = np.pi*2*f
    s = complex(0, 1)*w
    lam = -np.log(.5)*5000 # 551.5 Hz, 3466 rad/s
    lp_s = s/(s/lam+1)
    delay = np.exp(-s*0.0002) # .2 ms control period
    # delay = np.exp(-s*0.0002)*(2+w*w/10000)/(100+2.5*w+.1*w*w) # adds saturation effects
    # delay = np.exp(-s*0.0002)*(3+w*w/10000)/(100+.05*w*w)
    mgain = 5.2
    # mgain = 1/5.2
    Zs = k_s + s*b_s + s*s*m_s



    for file, gains in tests:
        kp, kdel, kd, kdeldot = gains
        Zm_prime = m_m*s*s+b_m*s-mgain*delay*(kd*lp_s+kp)
        Zs_prime = (m_m*s*s+b_m*s-mgain*delay*(kd*lp_s+kp+(kdeldot*lp_s+kdel)))
        Zmotor = 1/Zm_prime - mgain*delay*(kdeldot*lp_s+kdel)/Zs/Zm_prime
        
        Zcurrent = -delay*(kdeldot*lp_s+kdel)/Zs-delay*(kd*lp_s+kp)/Zm_prime
        z = (1/Zm_prime+Zs_prime/(Zs*Zm_prime))
        deflection, ang, inf_ang, torque, load, current, motor = get_data(file)
        namestrip = file[file.find("["):]

        
        # Zsensitivity scales sat_error to get the saturation noise floor!

        fig_mag, fig_phase = plot_admittance(ang, load, suptitle="Total: "+namestrip)
        fig_mag.axes[0].loglog(f, np.abs(z), label="Theory")
        fig_phase.axes[0].semilogx(f, 180./np.pi*angle(z))

        fig_mag, fig_phase = plot_admittance(motor, load, suptitle="Motor: "+namestrip)
        fig_mag.axes[0].loglog(f, np.abs(Zmotor), label="Theory")
        fig_phase.axes[0].semilogx(f, 180./np.pi*angle(Zmotor))
        ez_saturation_plot(file, gains, threshold=8)
        # compare_expected_sat_current(file, gains, threshold=8)
        # saturation_noise(file, gains, fig_mag.axes[0], fig_phase.axes[0])
        

        # fig_mag, fig_phase = plot_admittance(-deflection, load, suptitle="Spring: "+namestrip)
        # fig_mag.axes[0].loglog(f, np.abs(1/Zs), label="Theory")
        # fig_phase.axes[0].semilogx(f, 180./np.pi*angle(1/Zs))

        # fig_mag, fig_phase = plot_admittance(current, load, suptitle="Current: "+namestrip)
        # fig_mag.axes[0].loglog(f, np.abs(Zcurrent), label="Theory")
        # fig_phase.axes[0].semilogx(f, 180./np.pi*angle(Zcurrent))
    plt.show()



def compare_for_better_mass_line():
    test1="2018_08_09_144205_test_logger[-20000,-60000.0,-30.0,-90]" # repeated hammering with weights
    test2="2018_08_09_144904_test_logger[-20000,-60000.0,-30.0,-90]" # more hammering
    deflection1, ang1, inf_ang1, torque1, load1, current1, motor1 = get_data(test1)
    deflection2, ang2, inf_ang2, torque2, load2, current2, motor2 = get_data(test2)
    cross1, power1, f = get_cross_and_power(current1, load1)
    cross2, power2, f = get_cross_and_power(current2, load2)
    transfer = (cross1+cross2) / (power1+power2)
    scatter_bode_mag(f, transfer, power1+power2)
    plt.show()

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

def compare_expected_sat_current(test, gains, threshold=8):
    name = test[test.find("["):]
    deflection1, ang1, inf_ang1, torque1, load1, current1, motor1 = get_data(test)
    motordot = filter_lp_diff(np.array(motor1))
    deflectiondot = filter_lp_diff(np.array(deflection1))
    # gains[0]*=1./160.
    # gains[2]*=1./160.
    current_expected = combine_data([motor1, deflection1, motordot, deflectiondot], gains)
    satcurrent = saturate_data(current_expected, lim=threshold)
    abs_sat_current = op_data(satcurrent, lambda x: abs(x))
    plt.figure()
    plt.semilogy(*plot_discrete(abs_sat_current), label="Theory")
    plt.semilogy(*plot_discrete(current1), label="Experiment")
    plt.axis([0, 20, .001, 100])

    plt.xlabel("Time")
    plt.ylabel("Current (Amps)")
    plt.title("Prediction versus measurement")
    plt.suptitle("Gains: "+name)
    plt.legend()

def check_for_input_saturation():
    test1, gains = ("2018_08_08_162226_test_logger[0.0,0.0,2.95,0]", [0.0, 0.0, 2.95, 0.0])
    # test1="2018_08_09_151715_test_logger[-20000,-60000.0,-30.0,-90]"
    # gains = [-20000,-60000.0,-30.0,-90]
    deflection1, ang1, inf_ang1, torque1, load1, current1, motor1 = get_data(test1)
    motordot = filter_lp_diff(motor1)
    deflectiondot = filter_lp_diff(deflection1)
    current_expected = combine_data([motor1, deflection1, motordot, deflectiondot],gains)
    satcurrent = saturate_data(current_expected, 8) # I don't remember what the saturation really is
    # plt.plot(*plot_discrete(current_expected))
    # plt.plot(*plot_discrete(satcurrent))
    # plt.xlabel("Time")
    # plt.ylabel("Current")

    # fft_plot(current_expected)
    # fft_plot(satcurrent)
    plt.figure()
    # tf_A_over_B(satcurrent, current_expected)
    cross, power, f = get_cross_and_power(satcurrent, current_expected)
    transfer = cross / power
    scatter_bode_mag(f, transfer, power)
    w = np.pi*2*f
    s = complex(0,1)*w
    z = -(1)/(100.+2.5*s+.1 *s*s)
    z = (4+w*w/10000)/(100+.05*w*w)
    plt.loglog(f, np.abs(z))
    plt.title("Saturation describing function")
    # plt.figure()
    # scatter_bode_phase(f, transfer, power)
    # plt.semilogx(f, 180/np.pi*angle(z))
    plt.show()

def design_controller():
    # m_m = .88
    m_o = .29

    # work well on the no control tests
    m_m = .88/2
    m_s = 0.0
    b_s = 0.0

    k_s = 1180.
    b_m = 15.9+2 # pretty non linear


    f = np.logspace(-1, 3, 1000)
    w = np.pi*2*f
    s = complex(0, 1)*w
    lam = -np.log(.5)*5000 # 551.5 Hz, 3466 rad/s
    lp_s = s/(s/lam+1)
    delay = np.exp(-s*0.0002) # .2 ms control period
    # delay = np.exp(-s*0.0002)*(2+w*w/10000)/(100+2.5*w+.1*w*w) # adds saturation effects
    # delay = np.exp(-s*0.0002)*(3+w*w/10000)/(100+.05*w*w)
    mgain = 5.2
    # mgain = 1/5.2
    Zs = k_s + s*b_s + s*s*m_s


    # Params
    omega_p = 2*np.pi*20 # 2*pi for conversion from Hz
    omega_p_split_squared = (omega_p*.5)**2 # real for real poles, negative for complex poles
    omega_z = omega_p*.75 # convert from Hz
    omega_z_split_squared = +(omega_p*.1)**2 # negative, for complex

    # Try parameterizing by lowest freq (pole, real)
    low_p = 2*np.pi*8
    high_p = 2*np.pi*500
    omega_p = (low_p+high_p)/2 # 2*pi for conversion from Hz
    omega_p_split_squared = .25*(high_p-low_p)**2 * np.sign(high_p-low_p)
    low_z = 2*np.pi*40
    high_z = 2*np.pi*40
    omega_z = (high_z + low_z)/2
    omega_z_split_squared = .25*(high_z-low_z)**2 * np.sign(high_z-low_z)

    # m_m*s*s+b_m*s-mgain*delay*(kd*lp_s+kp) == m_m * (s^2+2*omega_p*s+omega_p^2-omega_p_split^2)
    kp = -m_m/mgain*(omega_p**2 - omega_p_split_squared)
    kd = -m_m/mgain*(omega_p*2)+b_m/mgain


    # (m_m*s*s+b_m*s-mgain*delay*(kd*lp_s+kp+(kdeldot*lp_s+kdel))) ==
    #       m_m*(s^2 + 2*omega_z*s + omega_z**2 - omega_z_split_squared )
    kdeldot = -m_m/mgain*2*omega_z+b_m/mgain-kd
    kdel = -m_m/mgain*(omega_z**2 - omega_z_split_squared)-kp

    print([kp, kdel, kd, kdeldot])


    # gains = [-2000, 0, -30, 0]
    # kp, kdel, kd, kdeldot = gains


    Zm_prime = m_m*s*s+b_m*s-mgain*delay*(kd*lp_s+kp)
    Zs_prime = (m_m*s*s+b_m*s-mgain*delay*(kd*lp_s+kp+(kdeldot*lp_s+kdel)))
    Zmotor = 1/Zm_prime - mgain*delay*(kdeldot*lp_s+kdel)/Zs/Zm_prime
    
    Zcurrent = -delay*(kdeldot*lp_s+kdel)/Zs-delay*(kd*lp_s+kp)/Zm_prime
    z = (1/Zm_prime+Zs_prime/(Zs*Zm_prime))
    Zsystem = 1/(1/z+(s*s*m_o))
    Zoutput = 1/(s*s*m_o)

    
    # Zsensitivity scales sat_error to get the saturation noise floor!
    mag_ax, phase_ax = empty_bode_plot(suptitle="Design: [%.1f,%.1f,%.1f,%.1f]"%(kp, kdel, kd, kdeldot))
    # fig_mag, fig_phase = empty_admittance_plots(suptitle="Actuator Admittance")
    mag_ax.loglog(f, np.abs(z), label="actuator")
    phase_ax.semilogx(f, 180./np.pi*angle(z), label="actuator")

    # mag_ax.loglog(f, np.abs(Zmotor), label="motor")
    # phase_ax.semilogx(f, 180./np.pi*angle(Zmotor), label="motor")

    mag_ax.loglog(f, np.abs(Zsystem), label="system")
    phase_ax.semilogx(f, 180./np.pi*angle(Zsystem), label="system")

    mag_ax.loglog(f, np.abs(1/Zm_prime*Zs_prime), lw=1.5, label="spring comp")
    phase_ax.semilogx(f, 180./np.pi*angle(complex(1,0.0)/Zm_prime*Zs_prime), lw=1.5, label="spring comp")


    mag_ax.loglog(f, np.abs(1/Zm_prime), ":",lw=2, label="\"motor\"")
    phase_ax.semilogx(f, 180./np.pi*angle(1/Zm_prime), ":", lw=2, label="\"motor\"")

    mag_ax.loglog(f, np.abs(1/Zm_prime*Zs_prime/Zs), ":",lw=2, label="\"spring\"")
    phase_ax.semilogx(f, 180./np.pi*angle(complex(1,0.0)/Zm_prime*Zs_prime/Zs), ":", lw=2, label="\"spring\"")

    plt.legend()
    mag_ax.loglog(f, np.abs(Zoutput), '--', c='k', zorder=-2)
    phase_ax.semilogx(f, 180./np.pi*angle(Zoutput), '--', c='k', zorder=-2)

    mag_ax.loglog(f, np.abs(1/(m_m*s*s+b_m*s)), '--', c='k', zorder=-2)
    phase_ax.semilogx(f, 180./np.pi*angle(1/(m_m*s*s+b_m*s)), '--', c='k', zorder=-2)

    mag_ax.loglog(f, np.abs(1/(k_s+m_s*s*s)), '--', c='k', zorder=-2)
    phase_ax.semilogx(f, 180./np.pi*angle((1/(k_s+m_s*s*s))), '--', c='k', zorder=-2)

    plt.show()

def get_theory_res(gains):
    m_o = .29

    # work well on the no control tests
    m_m = .88/2
    m_s = 0.0
    b_s = 0.0

    k_s = 1180.
    b_m = 15.9+2 # pretty non linear

    f = np.logspace(-1, 3, 1000)
    w = np.pi*2*f
    s = complex(0, 1)*w
    lam = -np.log(.5)*5000 # 551.5 Hz, 3466 rad/s
    lp_s = s/(s/lam+1)
    delay = np.exp(-s*0.0002) # .2 ms control period
    # delay = np.exp(-s*0.0002)*(2+w*w/10000)/(100+2.5*w+.1*w*w) # adds saturation effects
    # delay = np.exp(-s*0.0002)*(3+w*w/10000)/(100+.05*w*w)
    mgain = 5.2
    # mgain = 1/5.2
    Zs = k_s + s*b_s + s*s*m_s

    kp, kdel, kd, kdeldot = gains
    Zm_prime = m_m*s*s+b_m*s-mgain*delay*(kd*lp_s+kp)
    Zs_prime = (m_m*s*s+b_m*s-mgain*delay*(kd*lp_s+kp+(kdeldot*lp_s+kdel)))
    zmotor = 1/Zm_prime - mgain*delay*(kdeldot*lp_s+kdel)/Zs/Zm_prime
    
    Zcurrent = -delay*(kdeldot*lp_s+kdel)/Zs-delay*(kd*lp_s+kp)/Zm_prime
    z = (1/Zm_prime+Zs_prime/(Zs*Zm_prime))
    return f, z, zmotor

def display_results():
    # displays the admittance result of the feedback, testing for saturation and 
    # hopefully demonstrating improved stiffness
    file = "2018_08_10_165131_test_logger[-13361.925958397833,8017.155575038673,-266.6379961270732,227.5479725092422]"
    gains = [-13361.925958397833,8017.155575038673,-266.6379961270732,227.5479725092422]

    
    deflection, ang, inf_ang, torque, load, current, motor = get_data(file)
    namestrip = file[file.find("["):]
    m_o = .29

    cross, power, f = get_cross_and_power(load, inf_ang)
    transfer = cross / power
    # fftload, f = fft_plot(load)
    s = complex(0,2*np.pi)*f
    # inf_load = fft_load + m_o*s*s
    new_tf = 1/(transfer + m_o*s*s)
    mag_ax, phase_ax = empty_bode_plot(suptitle="Indirect Measurement of Output Admittance")
    
    scatter_bode_mag(f, new_tf, power, ax=mag_ax)
    scatter_bode_phase(f, new_tf, power, ax=phase_ax)

    # Zsensitivity scales sat_error to get the saturation noise floor!
    # f, z, zmotor = get_theory_res(gains)
    # fig_mag, fig_phase = plot_admittance(ang, load, suptitle="Total: [%.1f,%.1f,%.1f,%.1f]"%tuple(gains))
    # fig_mag.axes[0].loglog(f, np.abs(z), label="Theory")
    # fig_phase.axes[0].semilogx(f, 180./np.pi*angle(z))

    # fig_mag, fig_phase = plot_admittance(motor, load, suptitle="Motor: [%.1f,%.1f,%.1f,%.1f]"%tuple(gains))
    # fig_mag.axes[0].loglog(f, np.abs(zmotor), label="Theory")
    # fig_phase.axes[0].semilogx(f, 180./np.pi*angle(zmotor))
    # ez_saturation_plot(file, gains, threshold=8)
    plt.show()
    
if __name__ == '__main__':
    # initial_testing()
    # test_almost_unstable()
    # test_stiff_motor()
    # stiffer_motor()
    # boosted_admittance()
    # finding_the_added_inertia()
    # model_checking()
    # compare_for_better_mass_line()
    # check_for_input_saturation()
    # design_controller()
    display_results()