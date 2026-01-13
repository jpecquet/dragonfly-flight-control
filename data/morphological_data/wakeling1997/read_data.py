import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
from scipy.optimize import curve_fit

plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"] = ["Times New Roman"]
plt.rcParams['mathtext.fontset'] = 'stix'
plt.rcParams['font.size'] = 12

class DragonflySpecimen:
    """
    """
    def __init__(self, fore, hind, body):
        """
        """
        self.species = body['species'].iloc[0]
        self.sex = body['sex'].iloc[0]
        self.ID = body['ID'].iloc[0]
        self.m = body['m'].iloc[0]
        self.L = body['L'].iloc[0]

        self.R_f = fore['R'].iloc[0]
        self.R_h = hind['R'].iloc[0]
        self.S_f = fore['S'].iloc[0]
        self.S_h = hind['S'].iloc[0]

        self.unitsToSI()
        self.getAdimParams()

    def unitsToSI(self):
        """
        """
        self.m = self.m * 1e-6
        self.L = self.L * 1e-3
        self.R_f = self.R_f * 1e-3
        self.R_h = self.R_h * 1e-3
        self.S_f = self.S_f * 1e-6
        self.S_h = self.S_h * 1e-6

    def getAdimParams(self):
        """
        """
        self.lambda0_f = self.R_f / self.L
        self.lambda0_h = self.R_h / self.L
        Sh_f = self.S_f / (self.S_f + self.S_h)
        Sh_h = self.S_h / (self.S_f + self.S_h)
        self.lambda0 = Sh_f * self.lambda0_f + Sh_h * self.lambda0_h
        
        self.T = np.sqrt(self.L / 9.80665)
        coeff = 1.205 * (self.L / self.T)**2 / (self.m * 9.80665)

        self.mu0_f = coeff * self.S_f * self.lambda0_f
        self.mu0_h = coeff * self.S_h * self.lambda0_h
        self.mu0 = self.mu0_f + self.mu0_h

def readWakeling1997(forewing_data, hindwing_data, body_data):
    """
    Extract morphological data of dragonfly specimens from
    J.M. Wakeling, "Odonatan Wing and Body Morphologies", 1997.
    """
    fd = pd.read_csv(forewing_data)
    hd = pd.read_csv(hindwing_data)
    bd = pd.read_csv(body_data)
    
    # only keep specimens with both body and wing morphology data
    IDs = list(set(fd['ID'].values.tolist()).intersection(set(bd['ID'].values.tolist())))
    IDs.sort()
    
    specimens = []
    for ID in IDs:
        fd_i = fd.loc[fd['ID'] == ID]
        hd_i = hd.loc[hd['ID'] == ID]
        bd_i = bd.loc[bd['ID'] == ID]
        if ID[:2] != 'CS':
            specimens.append(DragonflySpecimen(fd_i, hd_i, bd_i))

    return specimens

def plotAdimParams(specimens):
    """
    """
    marker = {
            'Aeshna cyanea':'s',
            'Libellula quadrimaculata':'p',
            'Libellula depressa':'h',
            'Orthetrum cancellatum':'D',
            'Sympetrum striolatum':'^',
            'Sympetrum sanguineum':'v',
            'Calopteryx splendens':'o'
            }

    marker = {
            'Aeshna cyanea': MarkerStyle('s', fillstyle='none'),
            'Libellula quadrimaculata': MarkerStyle('o', fillstyle='none'),
            'Libellula depressa': MarkerStyle('o'),
            'Orthetrum cancellatum': MarkerStyle('x'),
            'Sympetrum striolatum': MarkerStyle('D', fillstyle='none'),
            'Sympetrum sanguineum': MarkerStyle('D'),
            'Calopteryx splendens': MarkerStyle('^')
            }

    fig, ax = plt.subplots(1, 2, figsize=(6.5,3))

    for sp in specimens:
        ax[0].scatter(sp.lambda0_f, sp.mu0_f, marker=marker[sp.species], color='k')
        ax[1].scatter(sp.lambda0_h, sp.mu0_h, marker=marker[sp.species], color='k', label=sp.species)
    # Get current handles and labels
    handles, labels = ax[1].get_legend_handles_labels()

    # Create a dictionary to store unique labels and their corresponding handles
    # The dictionary automatically handles the uniqueness of keys (labels)
    unique_labels = dict(zip(labels, handles))

    # Generate the legend using the unique handles and labels
    fig.legend(unique_labels.values(), unique_labels.keys(), loc='upper center', bbox_to_anchor=(0.5, 0), fontsize=12, ncol=2)
    ax[0].set_xlabel(r'$R_\mathrm{wing}/L_\mathrm{body}$')
    ax[0].set_ylabel(r'$\mu_0$')
    ax[0].set_xlabel(r'$\lambda_0$')
    ax[1].set_xlabel(r'$R_\mathrm{wing}/L_\mathrm{body}$')
    ax[1].set_xlabel(r'$\lambda_0$')
    ax[1].set_ylabel(r'$\mu_0$')
    ax[0].set_xlim(0.6, 0.9)
    ax[0].set_ylim(0, 0.2)
    ax[1].set_xlim(0.6, 0.9)
    ax[1].set_ylim(0, 0.2)
    ax[0].set_title('Forewings', fontsize=12)
    ax[1].set_title('Hindwings', fontsize=12)
    
    plt.tight_layout()
    
    fig.savefig('wakeling1997_data.pdf', dpi=600, bbox_inches='tight')

def mair(m, mu0):
    return mu0*m

def R(Lb, ld0):
    return ld0*Lb

def plotAdimParamsNew(specimens):
    """
    """
    marker1 = MarkerStyle('o')
    marker2 = MarkerStyle('o', fillstyle='none')

    fig, ax = plt.subplots(2, 1, figsize=(6.5,5.5))

    m, Lb = [], []
    R_f, R_h = [], []
    mair_f, mair_h = [], []

    for sp in specimens:
        if sp.m > 0.2e-3:
            ax[0].scatter(sp.m*1e3, 1.205*sp.R_f*sp.S_f*1e3, marker=marker1, color='k')
            p1 = ax[1].scatter(sp.L*1e3, sp.R_f*1e3, marker=marker1, color='k', label='Forewings')
            ax[0].scatter(sp.m*1e3, 1.205*sp.R_h*sp.S_h*1e3, marker=marker2, color='k')
            p3 = ax[1].scatter(sp.L*1e3, sp.R_h*1e3, marker=marker2, color='k', label='Hindwings')
        else:
            print(sp.species)
        
        if np.isfinite(sp.m) and np.isfinite(sp.L) and np.isfinite(sp.R_f) and np.isfinite(sp.S_f):
            m.append(sp.m)
            Lb.append(sp.L)
            R_f.append(sp.R_f)
            R_h.append(sp.R_h)
            mair_f.append(1.205*sp.R_f*sp.S_f)
            mair_h.append(1.205*sp.R_h*sp.S_h)

    m = np.array(m)
    Lb = np.array(Lb)
    R_f = np.array(R_f)
    R_h = np.array(R_h)
    mair_f = np.array(mair_f)
    mair_h = np.array(mair_h)

    popt, pcov = curve_fit(mair, m, mair_f)
    m_mod = np.linspace(0, 1.2, 11)
    ax[0].plot(m_mod, mair(m_mod, *popt), '--k', label='Forewings')
    ax[0].text(0.15, 0.075, r'$\mu_0^\text{fore} = %.3f$' % popt[0])

    popt, pcov = curve_fit(mair, m, mair_h)
    m_mod = np.linspace(0, 1.2, 11)
    ax[0].plot(m_mod, mair(m_mod, *popt), ':k', label='Hindwings')
    ax[0].text(0.15, 0.06, r'$\mu_0^\text{hind} = %.3f$' % popt[0])

    popt, pcov = curve_fit(R, Lb, R_f)
    Lb_mod = np.linspace(0, 100, 11)
    p2 = ax[1].plot(Lb_mod, mair(Lb_mod, *popt), '--k', label='Forewings')
    ax[1].text(45, 52, r'$\lambda_0^\text{fore} = %.2f$' % popt[0])

    popt, pcov = curve_fit(R, Lb, R_h)
    Lb_mod = np.linspace(0, 100, 11)
    p4 = ax[1].plot(Lb_mod, mair(Lb_mod, *popt), ':k', label='Hindwings')
    ax[1].text(45, 47.5, r'$\lambda_0^\text{hind} = %.2f$' % popt[0])
    
    handles, labels = ax[1].get_legend_handles_labels()

    p1 = handles[0]
    p2 = handles[-2]
    p3 = handles[31]
    p4 = handles[-1]

    fig.legend([(p1, p2), (p3, p4)], ['Forewings', 'Hindwings'], loc='upper center', bbox_to_anchor=(0.5, 0), fontsize=12, ncol=2)

    ax[0].set_xlabel(r'$m$ (g)')
    ax[0].set_ylabel(r'$\rho_\text{air}SR$ (g)')
    ax[1].set_xlabel(r'$L_b$ (mm)')
    ax[1].set_ylabel(r'$R$ (mm)')
    ax[0].set_xlim(0, 1.25)
    ax[0].set_ylim(0, 0.1)
    ax[1].set_xlim(40, 85)
    ax[1].set_ylim(30, 60)
    plt.tight_layout()
    
    fig.savefig('wakeling1997_data_new.pdf', dpi=600, bbox_inches='tight')

specimens = readWakeling1997('forewing_data.csv',
                             'hindwing_data.csv',
                             'body_data.csv')

plotAdimParams(specimens)
plotAdimParamsNew(specimens)
