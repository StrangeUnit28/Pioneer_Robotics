import pandas as pd
import matplotlib.pyplot as plt

# Caminho do arquivo CSV
arquivo_csv = "Pioneer_primitivas.csv"  # Ex: "trajetoria_pioneer.csv"

# Lê o CSV
df = pd.read_csv(arquivo_csv)

# Extrai as colunas de posição
x = df["x_out"]
y = df["y_out"]

# Plota a trajetória
plt.figure(figsize=(8, 6))
plt.plot(x, y, marker='o', linestyle='-', color='blue', linewidth=2, markersize=4)
plt.title("Trajetória do Robô Pioneer")
plt.xlabel("Posição X (m)")
plt.ylabel("Posição Y (m)")
plt.grid(True)
plt.axis("equal")  # Escala igual nos eixos
plt.plot(-2, 1.5, 'ko', label='Bola 1')  # verde
plt.plot(-0.5, -0.5, 'mo', label='Bola 2')   # vermelho
plt.plot(1, 1.5, 'yo', label='Bola 3')  # verde
plt.plot(1.5, -1.5, 'co', label='Bola 4')   # vermelho
plt.plot(x.iloc[0], y.iloc[0], 'go', label='Início')  # verde
plt.plot(x.iloc[-1], y.iloc[-1], 'ro', label='Fim')   # vermelho
plt.legend()
plt.savefig("trajetoria_pioneer.png", dpi=300)  # Salva a imagem
