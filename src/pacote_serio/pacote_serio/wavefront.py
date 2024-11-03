from matplotlib import pyplot as plt

INICIO = [56, 56]
OBJETIVO =[384 , 264]  
DIRECTIONS = [(-1, -1), (-1, 1), (1, -1), (1, 1),  # Diagonals
    (-1, 0), (1, 0), (0, -1), (0, 1)]
MAPA = open('/home/bibo/aprendendo_ros2/src/mapola.pgm', 'rb')

class Wavefront:
    
    def __init__(self):
        print('Iniciando Wavefront Algorithm')

    def plot_map(self, matrix, path=None):
        plt.plot(INICIO[0], INICIO[1], 'ro')  
        plt.plot(OBJETIVO[0], OBJETIVO[1], 'go') 
        if path:
            for point in path:
                plt.plot(point[0], point[1], 'bo')  
        plt.imshow(matrix, interpolation='nearest', cmap='viridis')
        plt.colorbar()


    def wavefront(self, matrix):
        matrix_new = [OBJETIVO]
        matrix[OBJETIVO[0]][OBJETIVO[1]] = 2 

        while matrix_new:
            current = matrix_new.pop(0)
            current_val = matrix[current[0]][current[1]] 

            for direction in DIRECTIONS:
                neighbor = (current[0] + direction[0], current[1] + direction[1])

                if 0 <= neighbor[0] < len(matrix) and 0 <= neighbor[1] < len(matrix[0]):
                    if matrix[neighbor[0]][neighbor[1]] == 1:  
                        matrix[neighbor[0]][neighbor[1]] = current_val + 1
                        matrix_new.append(neighbor)
        return matrix


    def find_path(self, matrix):
        path = []
        current = INICIO
        path.append(current)
        best_now = matrix[current[0]][current[1]]

        while current != OBJETIVO:
            for direction in DIRECTIONS:
                neighbor = (current[0] + direction[0], current[1] + direction[1]) # NOVO VIZINHO
                #print(f'Checking {current} at {neighbor}:')

                if neighbor[0] < len(matrix) and neighbor[1] < len(matrix[0]): #Dentro da matriz 
                    neighbor_val = matrix[neighbor[0]][neighbor[1]]
                    #print(f'Current: {neighbor_val}, best:{best_now} ')
                    if neighbor_val < best_now and neighbor_val != 0:
                        best_now = neighbor_val
                        current = neighbor
                        path.append(current)
                    if neighbor_val == 2:
                        path.append(neighbor)
                        current = neighbor
                        #print("Path:", path)

                        return path
        

    def run(self):
        #mapa = MAPA_TESTE
        mapa = plt.imread(MAPA)
        mapa = 1.0 * (mapa > 250) # PRETO E BRANCO: preto = 0.0, branco = 1.0
        
        matrix = self.wavefront(mapa)

        path = self.find_path(matrix)

        self.plot_map(mapa,path)

        plt.show()



if __name__ == '__main__':
    try:
        Wavefront().run()
    except KeyboardInterrupt:
        pass
