import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

M = pd.read_csv("../dat/MANIPULABILITY_3cm.csv")
M = M.clip(lower=[-np.inf, -np.inf, 0, 0, 0], axis=1)

import matplotlib.cm as cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.path import Path
import matplotlib.patches as patches
codes = [
		Path.MOVETO,
		Path.LINETO,
		Path.LINETO,
		Path.LINETO,
		Path.CLOSEPOLY,
	]

base_extents = np.array([0.3, 0.4])
base_bl = np.array([0.78, -0.5])
base_bl = base_bl - base_extents

# fig = plt.figure(figsize=(10, 15))
# ax_y = fig.add_subplot(1, 2, 1, projection='3d')
# surf_y = ax_y.plot_trisurf(M['x'], M['y'], M['yoshikawa'], cmap='viridis')

# base_rect = patches.Rectangle(
# 					(base_bl[0], base_bl[1]),
# 					2 * base_extents[0], 2 * base_extents[1],
# 					linewidth=2, edgecolor='k', facecolor='none',
# 					alpha=1, zorder=2)
# ax_y.add_artist(base_rect)

# # ax_y.axis('equal')

# ax_t = fig.add_subplot(1, 2, 2, projection='3d')
# surf_t = ax_t.plot_trisurf(M['x'], M['y'], M['togai'], cmap='viridis')

# base_rect = patches.Rectangle(
# 					(base_bl[0], base_bl[1]),
# 					2 * base_extents[0], 2 * base_extents[1],
# 					linewidth=2, edgecolor='k', facecolor='none',
# 					alpha=1, zorder=2)
# ax_t.add_artist(base_rect)

# # ax_t.axis('equal')

# plt.tight_layout()
# plt.show()

P = pd.read_csv("../dat/PUSHES_IK_2s.csv")
start_x_min = np.min(P['x0'])
start_x_max = np.max(P['x0'])
start_y_min = np.min(P['y0'])
start_y_max = np.max(P['y0'])

FIG, AX = plt.subplots(figsize=(13, 13))
cmap = cm.get_cmap('Dark2')

push_legend = []
failure_modes = {
	1: 'ik success',
	2: 'ik did not reach end',
	3: 'ik obstacle collision',
	4: 'ik joint limits',
	5: 'ik inv vel fail',
	6: 'start invalid'
}
for i in failure_modes:
	push_legend.append(patches.Patch(color=cmap(i), label=failure_modes[i]))

for x in np.arange(start_x_min, start_x_max, step=0.01):
	for y in np.arange(start_y_min, start_y_max, step=0.01):
		base_rect = patches.Rectangle(
							(base_bl[0], base_bl[1]),
							2 * base_extents[0], 2 * base_extents[1],
							linewidth=2, edgecolor='k', facecolor='none',
							alpha=1, zorder=2)
		AX.add_artist(base_rect)
		AX.legend(handles=push_legend)
		AX.axis('equal')

		relevant = P.loc[(P['x0'] == np.round(x, decimals=2)) & (P['y0'] == np.round(y, decimals=2))]
		if (relevant.shape[0] == 1):
			continue
		yaw0 = relevant.iloc[0]['yaw0']
		yaw1 = relevant.iloc[0]['yaw1']
		AX.scatter(x, y, s=50, c='gold', zorder=3, marker='*')
		AX.arrow(x, y, np.cos(yaw0) * 0.05, np.sin(yaw0) * 0.05, head_width=0.02, fc='r', ec='r')
		AX.arrow(x, y, np.cos(yaw1) * 0.05, np.sin(yaw1) * 0.05, head_width=0.02, fc='g', ec='g')
		for idx, row in relevant.iterrows():
			AX.scatter(row['x1'], row['y1'], c=[cmap(int(row['result']+1))], zorder=1)

		plt.savefig('../dat/push_data/ik2/({},{}).png'.format(int(x*100), int(y*100)), bbox_inches='tight')
		# plt.show()
		plt.cla()
		AX = plt.gca()
		FIG = plt.gcf()
		FIG.set_size_inches(13, 13)
	# 	break
	# break
