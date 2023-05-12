'''
    Note to self
        - When working on this portion of the project you HAVE to 'open' the python folder in a new instance of VSCode
'''

# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np

# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')


# # reading the CSV file
# df = pd.read_csv('IMU_Data.csv')

# # displaying the contents of the CSV file
# # print(df)

# ax.scatter3D(df.aX, df.aY, df.aZ)
# # ax.scatter(df.aX, c='r')
# # ax.scatter(df.aY, c='b')
# # ax.scatter(df.aZ, c='g')


# ax.set_xlabel('X Label')
# ax.set_ylabel('Y Label')
# ax.set_zlabel('Z Label')

# plt.show()

import plotly.express as px
import pandas as pd

# # reading the CSV file
df = pd.read_csv('IMU_Data.csv')

fig = px.scatter_3d(df, x='aX', y='aY', z='aZ',color='aX')

fig.show()
