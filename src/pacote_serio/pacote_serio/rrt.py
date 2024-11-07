from matplotlib import pyplot as plt
import math
import random

INICIO =(56,56)
OBJETIVO =(384,264)   

MAPA = open('/home/bibo/aprendendo_ros2/src/mapola.pgm', 'rb')

class RRT:

    def __init__(self):
        print('Iniciando algoritmo RRT...')
        self.arvore = [INICIO]
        self.caminho = []
        self.pais = []
        self.filhos = []
        self.max_interacoes = 10000
        self.f_crescimento = 10
        

    def plot_map(self, matrix, path=None, final_path=None):
        plt.plot(INICIO[1], INICIO[0], 'ro')  
        plt.plot(OBJETIVO[1], OBJETIVO[0], 'go')  

        if path:
            for (p1, p2) in path:
                plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b',linewidth=2) 

        if final_path:
            for (p1, p2) in final_path:
                plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'm', linewidth=2)  

        plt.imshow(matrix, interpolation='nearest', cmap='gray')
        plt.colorbar()

    def achar_no_proximo(self, ponto_aleatorio):
        menor_dist = float('inf')
        no_mais_prox = self.arvore[0]
        for n in self.arvore:
            dist = math.dist(n, ponto_aleatorio)
            if dist < menor_dist:
                menor_dist = dist
                no_mais_prox = n
        return no_mais_prox

    def gerar_novo_no(self, no_mais_prox, ponto_aleatorio):
        vet_direcao = (ponto_aleatorio[0] - no_mais_prox[0], ponto_aleatorio[1] - no_mais_prox[1])
        modulo_vet = math.sqrt(vet_direcao[0]**2 + vet_direcao[1]**2)

        if modulo_vet < self.f_crescimento:
            return ponto_aleatorio

        fator = self.f_crescimento / modulo_vet
        novo_no = (int(no_mais_prox[0] + vet_direcao[0] * fator), int(no_mais_prox[1] + vet_direcao[1] * fator))
        return novo_no

    def run(self):

        self.mapa = plt.imread(MAPA)
        self.mapa = 1.0 * (self.mapa > 250) 
        self.mapa[OBJETIVO[0]][OBJETIVO[1]] = 0  
        self.mapa[INICIO[0]][INICIO[1]] = 0  

        for i in range(self.max_interacoes):
            ponto_aleatorio = (random.randint(0, self.mapa.shape[0] - 1), random.randint(0, self.mapa.shape[1] - 1))

            no_mais_proximo = self.achar_no_proximo(ponto_aleatorio)
            novo_no = self.gerar_novo_no(no_mais_proximo, ponto_aleatorio)

            if 0 <= novo_no[0] < self.mapa.shape[0] and 0 <= novo_no[1] < self.mapa.shape[1] and self.mapa[novo_no[0]][novo_no[1]] == 1.0:
                self.arvore.append(novo_no)
                self.caminho.append((no_mais_proximo, novo_no))
                self.pais.append(no_mais_proximo)
                self.filhos.append(novo_no)

                if math.dist(novo_no, OBJETIVO) < self.f_crescimento:
                    print("Objetivo alcançado!")
                    break

        self.tracar_caminho()

    def tracar_caminho(self):
        caminho_correto = []
        filho = self.arvore[-1]
        while filho != INICIO:
            ind = self.filhos.index(filho)
            pai = self.pais[ind]
            caminho_correto.append((pai, filho))
            filho = pai

        fig = plt.figure()
        self.plot_map(self.mapa, self.caminho, caminho_correto)
        plt.show()


if __name__ == '__main__':
    try:
        RRT().run()
    except KeyboardInterrupt:
        pass
