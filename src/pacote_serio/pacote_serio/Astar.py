from matplotlib import pyplot as plt
import math 
import time

INICIO =(56,56)
OBJETIVO =(200,264)   

DIRECTIONS = [(-1, -1), (-1, 1), (1, -1), (1, 1),  
    (-1, 0), (1, 0), (0, -1), (0, 1)]

DIAGONALS = [(-1,0), (1,0), (0,1), (0,-1)]
STRAIGHTS = [(-1,-1), (-1,1), (1,-1), (1,1)]

MAPA = open('/home/bibo/aprendendo_ros2/src/mapola.pgm', 'rb')


class Astar:

    def __init__(self):
        print('Iniciando Astar Algorithm...')
        self.g = {}
        self.g[tuple(INICIO)]=0
        self.f = {}
        self.path = []

    def plot_map(self, matrix, path=None):
        plt.plot(INICIO[0], INICIO[1], 'ro')  
        plt.plot(OBJETIVO[0], OBJETIVO[1], 'go') 

        if path:
            for point in path:
                plt.plot(point[0], point[1], 'bo')  
        plt.imshow(matrix, interpolation='nearest', cmap='viridis')


    # CÁLCULOS E FUNÇÔES 

    def cost(self, previous, current):
        if previous not in self.g:
            self.g[previous] = 0  

        if current not in self.g:
            g_cost = self.g[previous]
            
            for direction in DIAGONALS:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                if neighbor == previous:
                    g_cost += 1.4
                    break
            for direction in STRAIGHTS:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                if neighbor == previous:
                    g_cost += 1.0
                    break

            self.g[current] = g_cost
        
        return self.g[current]


    def heuristica(self,current):

        h_cost = math.sqrt((current[0]-OBJETIVO[0])**2 + (current[1]-OBJETIVO[1])**2)


        return h_cost

    def funcao_avaliativa(self,previous, current):
        if current not in self.f:
            
            f_cost = self.cost(previous,current) + self.heuristica(current)
            
            self.f[current] = f_cost

        return self.f[current]

    def search(self,cell):
        next_cell = cell
        best_cost = float('inf')

        for direction in DIRECTIONS:
            neighbor = (cell[0] + direction[0], cell[1] + direction[1])

            if 0 <= neighbor[0] < len(self.mapa) and 0 <= neighbor[1] < len(self.mapa[0]):
                if self.mapa[neighbor[0]][neighbor[1]] != 0:
                    
                    if (neighbor in self.path) or (neighbor in self.f) :
                        continue

                    f = self.funcao_avaliativa(cell,neighbor)
                    #print(f'Checking {neighbor} with f {f}')
                    if f == best_cost:
                        neighbor_h = self.heuristica(neighbor)
                        current_best_h = self.heuristica(next_cell)
                        #print(f'COMPARING H VALUES: {neighbor}: {neighbor_h}, {next_cell}:{current_best_h}')
                        if(neighbor_h<current_best_h):
                            best_cost = f
                            next_cell = neighbor
                        
                    if f < best_cost:
                        best_cost = f
                        next_cell = neighbor

        
        #print(f'Heading to {next_cell} with f {best_cost}\n\n')
        return next_cell


                               
     
    def run(self):
        self.mapa = plt.imread(MAPA)
        self.mapa = 1.0 * (self.mapa > 250) # PRETO E BRANCO: preto = 0.0, branco = 1.0

        current = INICIO
        previous = None
        while current != OBJETIVO:
            self.path.append(current) 
            next_cell = self.search(current)
            previous = current
            current = next_cell
            if next_cell == previous:
                print('SOU BURRO')
                break


        self.path.append(current)
        print(f'OBJETIVO REACHED') if current == OBJETIVO else None
        self.plot_map(self.mapa,self.path)
        plt.show()



    
if __name__ == '__main__':
    try:
        Astar().run()
    except KeyboardInterrupt:
        pass