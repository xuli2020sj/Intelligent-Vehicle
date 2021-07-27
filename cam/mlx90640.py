import time
from ctypes import *
import numpy as np
mlx90640 = cdll.LoadLibrary('./libmlx90640.so')

np.set_printoptions(threshold=np.inf)

# mlx90640 will output 32*24 temperature array with chess mode
while True:
    temp = (c_float * 768)()
    ptemp = pointer(temp)
    mlx90640.get_mlx90640_temp(ptemp)
    my_nparray = np.frombuffer(temp, dtype=np.float32)

    t = my_nparray.reshape((32, 24))

    print(t.max)
    print(t.index(max(t)))
    time.sleep(1/2)
# for i in range(len(temp)):
#     if i % 32 == 0 and i != 0:
#         print("\r\n")
#     print("%.2f " % (temp[i]), end='')
