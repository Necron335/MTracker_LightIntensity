import numpy as np

def T(theta):
    T = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0,              0,             1]
    ])
    return T

def h(p1, p2, omega, t):
    h = np.array([
        p1 * np.cos(omega * t),
        p2 * np.sin(omega * t),
        0
    ])
    return h

def dh(p1, p2, omega, t):
    dh = np.array([
        -omega * p1 * np.sin(omega * t),
         omega * p2 * np.cos(omega * t),
         0
    ])
    return dh

def f_kreska(epsilon1, epsilon2, epsilon3, alpha):
    f_kreska = np.array([
        epsilon1*np.sin(alpha),
        epsilon1*epsilon2*epsilon3*np.sin(2*alpha),
        epsilon2*np.cos(alpha)
    ])
    return f_kreska


#F zwykłe to jest to zwykłe f = h*f_kreska

# ALfa prim to jest dodatkowa zmienna stanu wirtualna mm i jest na sprzężeniu zwrotnym i wchodzi na u z kreską, alfa jest całkowana

#Stałe 

C = np.array([
    [1, 0, 0],
    [0, 0, 1]
])
# Macierz C z równań stanu
