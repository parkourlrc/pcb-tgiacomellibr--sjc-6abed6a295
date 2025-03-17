# Importação das bibliotecas necessárias
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Definição dos parâmetros do sistema
Vcc = 15  # Tensão do driver (V)
Vf_diode = 0.55  # Queda de tensão do diodo bootstrap (V)
fsw = 20e3  # Frequência de comutação (Hz)
Tdt = 400e-9  # Tempo morto (s)

# Parâmetros do MOSFET
Qg = 54.8e-9  # Carga total do gate (C)

# Correntes de fuga e quiescentes
Ilkcap = 0  # Corrente de fuga do capacitor de bootstrap (cerâmico)
Ilkgs = 100e-9  # Corrente de fuga do MOSFET (A)
Iqbs = 170e-6  # Corrente quiescente do bootstrap (A)
Ilk = 50e-6  # Corrente de fuga geral (A)
Ilkdiode = 10e-6  # Corrente de fuga do diodo (A)
Qls = 3e-9  # Carga do level shifter interno (C)

# Duty cycle máximo e mínimo
Dmax = 0.99
Dmin = 0.01

# Cálculo dos tempos de condução do MOSFET de baixa
Th_min = ((1 - Dmax) / fsw) - Tdt
Tl_max = (Dmax / fsw) + Tdt

# Cálculo da carga total necessária (Q_TOTAL)
Qtotal = Qg + (Ilkcap + Ilkgs + Iqbs + Ilk + Ilkdiode) * Tl_max + Qls

# Ripple máximo permitido (6% de VCC, conforme MATLAB)
delta_Vcb = 0.06 * Vcc

# Cálculo do capacitor de bootstrap (CB) em nF
Cb = Qtotal / delta_Vcb
Cb_nF = Cb * 1e9  # Convertendo para nanofarads

# Aplicar margem de segurança de 20%
Cb_nF_adjusted = Cb_nF * 1.2  # Aumento de 20%

# Cálculo do resistor de bootstrap (RB)
Rb = Th_min / (4 * Cb)  # Seguindo MATLAB (4x no denominador)

# Cálculo da corrente média de carga (I_avg)
I_avg = Qtotal / Th_min

# Cálculo da corrente de pico do diodo (I_pk)
I_pk = (Vcc - Vf_diode) / Rb

# Cálculo da potência dissipada no driver (P_D,GATE)
C_load = Qg / Vcc  # Capacitância de entrada do MOSFET
P_DGATE = 2 * C_load * fsw * Vcc**2

# Cálculo da potência dissipada no diodo de bootstrap (P_D,BOOT)
I_charge = Qtotal / Th_min
P_DBOOT = I_charge * Vf_diode

# Cálculo da dissipação total de potência (P_TOTAL)
P_TOTAL = P_DGATE + P_DBOOT

# Cálculo dos resistores de gate-on (Rg_on) e gate-off (Rg_off)
R_driver = Vcc / 2.5  # Resistência do driver (Ω)
t_sw = 100e-9  # Tempo de chaveamento desejado (100 ns)

Ig_avg = Qg / t_sw  # Corrente média durante a comutação
R_total = (Vcc - 2.2) / (251e-12 * 0.9e9)  # Resistência total baseada na capacitância de Miller

Rg_on = R_total - R_driver
Rg_off = (2.2 / (251e-12 * 0.9e9)) - R_driver  # MATLAB usa VGS_th_min

# Exibir resultados numéricos
print("\n=== Resultados da Rede de Bootstrap ===")
print(f"Capacitor de Bootstrap sem margem: {Cb_nF:.2f} nF")
print(f"Capacitor de Bootstrap com +20%: {Cb_nF_adjusted:.2f} nF")
print(f"Resistor de Bootstrap (Ω): {Rb:.2f}")
print(f"Resistor de Gate-On (Ω): {Rg_on:.2f}")
print(f"Resistor de Gate-Off (Ω): {Rg_off:.2f}")
print(f"Corrente Média de Carga (A): {I_avg:.6f}")
print(f"Corrente de Pico do Diodo (A): {I_pk:.2f}")
print(f"Potência Dissipada no Driver (W): {P_DGATE:.6f}")
print(f"Potência Dissipada no Diodo Bootstrap (W): {P_DBOOT:.6f}")
print(f"Potência Dissipada Total (W): {P_TOTAL:.6f}")

# Comparação com MATLAB
Cb_nF_matlab = 82  # MATLAB bootstrap capacitor
Rb_matlab = 2  # MATLAB bootstrap resistor

comparacao_resultados = pd.DataFrame({
    "Parâmetro": [
        "Capacitor de Bootstrap (nF)", 
        "Resistor de Bootstrap (Ω)", 
        "Corrente Média de Carga (A)", 
        "Corrente de Pico do Diodo (A)"
    ],
    "Valor (Python)": [
        Cb_nF_adjusted, Rb, I_avg, I_pk
    ],
    "Valor (MATLAB)": [
        Cb_nF_matlab, Rb_matlab, I_avg, (Vcc - Vf_diode) / Rb_matlab
    ]
})

print("\n=== Comparação Python vs MATLAB ===")
print(comparacao_resultados)

# Gráficos de potência
frequencias = np.linspace(1e3, 100e3, 100)

plt.figure()
plt.plot(frequencias, P_DGATE * np.ones_like(frequencias), label="Driver")
plt.plot(frequencias, P_DBOOT * np.ones_like(frequencias), label="Diodo Bootstrap")
plt.plot(frequencias, P_TOTAL * np.ones_like(frequencias), label="Total")
plt.xlabel("Frequência (Hz)")
plt.ylabel("Potência Dissipada (W)")
plt.title("Potências Dissipadas nos Componentes")
plt.legend()
plt.grid(True)
plt.show()
