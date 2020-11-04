import numpy as np
import matplotlib.pyplot as plt


from mpl_toolkits import mplot3d

for i in range(21):
	file = open('vals'+str(i+1)+'.txt','r')
	array = []
	i = 0
	x_array = []
	y_array = []
	z_array = []
	for line in file:
		float_arr = []
		splitarr = line.split()
		j = 0
		for x in splitarr:
			float_arr.append(float(x))
			x_array.append(i)
			y_array.append(j)
			z_array.append(float(x))
			j += 1
		array.append(float_arr)
		i+= 1
	file.close()

	numpy_arr = np.asarray(array)

	plt.clf()
	plt.imshow(numpy_arr)
	plt.show()

# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.contour3D(x_array, y_array, z_array)
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# plt.show()

