# Gray Thomas, NSTRF15AQ33H at NASA JSC August 2018
# Works in python 3

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from admittance_plotting_utils import *


def test_new():
    # data=DataSet("log_20180811T155347")
    data_on=DataSet("log_20180811T161859")
    data_off = DataSet("log_20180811T163628")
    # print(dir(data))
    mag_ax, phase_ax = empty_bode_plot(suptitle="Actuator Admittance")
    tf_A_over_B_phase(data_on.actuator, data_on.infloadforce, ax=phase_ax, label="w.r.t. Human")
    tf_A_over_B_phase(data_off.actuator, data_off.infloadforce, ax=phase_ax, label="w.r.t. Environment")
    tf_A_over_B(data_on.actuator, data_on.infloadforce, ax=mag_ax, label="w.r.t. Human")
    tf_A_over_B(data_off.actuator, data_off.infloadforce, ax=mag_ax, label="w.r.t. Environment")
    mag_ax.legend()

    # plot_admittance(data.force, data.infloadforce, suptitle="Spring")
    # plot_admittance(data.motor, data.infloadforce, suptitle="Motor")
    # plot_admittance(data.actuator, data.infloadforce, suptitle="Actuator")
    plt.show()

def model_checking():
    tests = [
        ("2018_08_09_144904_test_logger[-20000,-60000.0,-30.0,-90]", [-20000,-60000.0,-30.0,-90]), # more hammering
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

    
if __name__ == '__main__':
    test_new()