import struct
import sys
import os

# -------- CONFIGURAÇÕES --------
input_file = "example_file/northridge.dat"  # Substitui pelo nome do teu ficheiro do SeismoSignal
output_file = "example_file/sismo.bin"
dt = 0.01  # Amostragem do sismo (Delta t). Ex: 100Hz = 0.01 segundos
# -------------------------------

posicoes = []

if not os.path.exists(input_file):
    print(f"Erro: O ficheiro '{input_file}' não foi encontrado na pasta atual.")
    sys.exit(1)

# Ler o ficheiro de texto
with open(input_file, 'r') as f:
    for linha in f:
        linha = linha.strip()
        
        # Ignorar linhas vazias ou comentários/cabeçalhos
        if not linha or linha.startswith(('%', '#', 'Time', 'time')):
            continue
        
        partes = linha.split()
        
        # Assumindo que a coluna 1 é o Tempo e a coluna 2 é o Deslocamento
        if len(partes) >= 2:
            try:
                # Se o deslocamento estiver na primeira coluna, usa partes[0] em vez de partes[1]
                deslocamento = float(partes[1])
                posicoes.append(deslocamento)
            except ValueError:
                continue # Ignora linhas que não consiga converter para número

num_pontos = len(posicoes)

if num_pontos == 0:
    print("Erro: Nenhum ponto válido encontrado no ficheiro.")
    sys.exit(1)

# Escrever no formato binário esperado pelo ESP32
with open(output_file, 'wb') as f:
    # 1. Cabeçalho (8 bytes): Número de pontos (uint32) + Amostragem (float)
    f.write(struct.pack('<I', num_pontos))
    f.write(struct.pack('<f', dt))
    
    # 2. Dados: Cada posição é escrita como um float (4 bytes)
    for pos in posicoes:
        f.write(struct.pack('<f', pos))

tamanho_final = 8 + (num_pontos * 4)
print(f"Sucesso! Convertidos {num_pontos} pontos para o ficheiro '{output_file}'.")
print(f"Tamanho do ficheiro otimizado: {tamanho_final} bytes.")
