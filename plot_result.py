import os
import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':
    result_path = 'data'
    file_list = ['result-fusion.txt', 'result-lidar.txt', 'result-radar.txt', 'ground_truth.txt']
    legends = ['fusion', 'lidar', 'radar', 'ground truth']
    file_list = [os.path.join(result_path, filename) for filename in file_list]
    plt.figure(figsize=(10, 5), dpi=500)
    for file in file_list:
        try:
            data = np.loadtxt(file)
        except:
            data = np.loadtxt(file, usecols=[1, 2, 3, 4, 5 ])
        plt.plot(data[:, 1], data[:, 2])
    plt.legend(legends)
    plt.savefig('data/result.png', bbox_inches='tight')
    plt.show()
