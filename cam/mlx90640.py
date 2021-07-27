from ctypes import *
import numpy as np
mlx90640 = cdll.LoadLibrary('./libmlx90640.so')


# mlx90640 will output 32*24 temperature array with chess mode

temp = (c_float * 768)()
ptemp = pointer(temp)
mlx90640.get_mlx90640_temp(ptemp)
print(temp)
# for i in range(len(temp)):
#     if i % 32 == 0 and i != 0:
#         print("\r\n")
#     print("%.2f " % (temp[i]), end='')
