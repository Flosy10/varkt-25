from math import *
import matplotlib.pyplot as plt

def g(h):
    G = 6.67408 * 10 ** -11
    M = 5.2915793 * 10 ** 22
    R = 600_000
    g = G * M / (R + h) ** 2
    return g


def F_sopr(h, v):
    if h >= 70_000: return 0
    global cf
    global p
    global s
    return (cf * (p * s) * v ** 2) / 2


Mr = 33165
M0 = 6500 * 3
T1 = 42.2
k = (Mr - M0) / T1

s = 2.1 ** 2 * pi
cf = 0.48
m0 = 33165

alpha = 90
h = 0
vx = 0
vy = 0
v = (vx ** 2 + vy ** 2) ** 0.5

Ft = 3 * 275_000

t = 0

T = 75
pa = 1.225
p = pa - t * pa / T
v_res = []
h_res = []
m_res = []
t_res = []

while h < 2800:
    t += 1
    p = pa - t * pa / T

    ay = abs((Ft - F_sopr(h, v) ) * 1 - m0 * g(h))/ m0
    vy += ay
    h = h + vy + ay / 2

    v = (vx ** 2 + vy ** 2) ** 0.5
    m0 = m0 - k
    v_res.append(v)
    h_res.append(h)
    m_res.append(m0)
    t_res.append(t)

b = 0
alpha0 = alpha
alpha1 = 85
tpov = 20
t_nachalo_povorota = 0
if 2800 <= h <= 2995:
    t_nachalo_povorota = t

while 2800 <= h < 16_500:

    b = (alpha0 - alpha1)/tpov
    alpha = alpha0 - (b * (t - t_nachalo_povorota))
    t += 1
    p = pa - t * pa / T
    ax = abs((Ft - F_sopr(h, v)) * cos(alpha / 180 * pi) / m0)
    ay = abs((Ft - F_sopr(h, v) - m0 * g(h)) * sin(alpha / 180 * pi) / m0)
    vx += ax
    vy += ay
    h = h + vy + ay / 2
    v = (vx ** 2 + vy ** 2) ** 0.5
    m0 = m0 - k
    v_res.append(v)
    h_res.append(h)
    m_res.append(m0)
    t_res.append(t)

m0 = 33165 - 7650 * 3
M0 = 2000
T2 = 75
k = (m0 - M0) / T2
s = 0.21 ** 2 * pi
Ft = 253_500

b = 0

if 16_500 <= h <= 16_800:
    t_nachalo_povorota = t
    tpov = 34

alpha0 = alpha
alpha1 = 0
while 16_500 <= h < 50_000:
    b = (alpha0 - alpha1) / tpov
    alpha = alpha0 - (b * (t - t_nachalo_povorota))
    t += 1
    p = pa - t * pa / T
    ax = abs((Ft * cos(alpha / 180 * pi) - F_sopr(h, v))  / m0)
    ay = abs((Ft - F_sopr(h, v) - m0 * g(h)) * sin(alpha / 180 * pi) / m0)
    vx += ax
    vy += ay
    h = h + vy + ay / 2

    v = (vx ** 2 + vy ** 2) ** 0.5
    m0 = m0 - k
    v_res.append(v)
    h_res.append(h)
    m_res.append(m0)
    t_res.append(t)

while h >= 50_000 and t < 293:
    t += 1
    b = 90 * (1 - h / 150_000)
    p = pa - t * pa / T
    ax = 0
    ay = -g(h)
    if t >= 75 and t <= 268:
        ax = 0
        ay = -g(h)
    if t > 268:
        ax = Ft / m0
        ay = -g(h)
    vx += ax
    vy += ay
    if t >= 75 and t <= 268:
        h = h + vy  + ay / 2
    else:
        h = h + vy + ay / 2

    v = (vx ** 2 + vy ** 2) ** 0.5
    if t >= 75 and t <= 268:
        m0 = m0
    else:
        m0 = m0 - k
    v_res.append(v)
    h_res.append(h)
    m_res.append(m0)
    t_res.append(t)
