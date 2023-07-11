import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib.cm as cm

import os
import sys
import collections

def onclick(event):
	print('(%f %f)' % (event.xdata, event.ydata))

FIG, AX = plt.subplots(figsize=(13, 13))
cid = FIG.canvas.mpl_connect('button_press_event', onclick)
FRIDGE = True
RES = 0.01

def ParseFile(filepath, saving=True):
	objs = {}
	trajs = {}
	invalids = {}
	ngr = []
	goals = []
	pushes = []
	with open(filepath, 'r') as f:
		done = False
		while not done:
			line = f.readline()[:-1]

			if len(line) == 0:
				done = True

			if line == 'O':
				line = f.readline()[:-1]
				num_objs = int(line) - 3 * saving
				for i in range(num_objs):
					line = f.readline()[:-1]
					obj = [float(val) for val in line.split(',')[:-1]]
					# line = f.readline()[:-1]
					# objs[obj[0]] += [line == 'True']
					objs[obj[0]] = obj[1:]
					objs[obj[0]] += [line.split(',')[-1] == 'True']

				objs = collections.OrderedDict(sorted(objs.items()))

			if line == 'G':
				line = f.readline()[:-1]
				if len(line) == 1:
					num_pts = int(line)
					for i in range(num_pts):
						line = f.readline()[:-1]
						pt = [float(val) for val in line.split(',')]
						goals.append(pt)

			if line == 'T':
				line = f.readline()[:-1]
				num_moving = int(line)
				for i in range(num_moving):
					line = f.readline()[:-1]
					obj_id = int(line)

					line = f.readline()[:-1]
					num_wps = int(line)

					traj = []
					for i in range(num_wps):
						line = f.readline()[:-1]
						wp = [float(val) for val in line.split(',')]
						traj.append(wp)
					trajs[obj_id] = traj

			if line == 'NGR':
				line = f.readline()[:-1]
				num_pts = int(line)
				for i in range(num_pts):
					line = f.readline()[:-1]
					pt = [float(val) for val in line.split(',')]
					ngr.append(pt)

			if line == 'PUSHES':
				line = f.readline()[:-1]
				num_pushes = int(line)
				for i in range(num_pushes):
					line = f.readline()[:-1]
					push = [float(val) for val in line.split(',')]
					pushes.append(push)

			if line == 'INVALID_L':
				line = f.readline()[:-1]
				num_objs = int(line)
				for i in range(num_objs):
					line = f.readline()[:-1]
					obj_id = int(line)

					line = f.readline()[:-1]
					num_invalids = int(line)

					invalid = []
					for i in range(num_invalids):
						line = f.readline()[:-1]
						invalid_g = [float(val) for val in line.split(',')]
						invalid.append(invalid_g)
					invalids[obj_id] = invalid

	return objs, trajs, invalids, ngr, goals, pushes

def DrawScene(filepath, objs, trajs, invalids, ngr, goals, pushes, alpha=1.0):
	codes = [
		Path.MOVETO,
		Path.LINETO,
		Path.LINETO,
		Path.LINETO,
		Path.CLOSEPOLY,
	]

	base = objs[1]
	base_bl = np.array(base[2:4])

	base_extents = np.array(base[8:10])
	base_bl = base_bl - base_extents

	base_rect = patches.Rectangle(
						(base_bl[0], base_bl[1]),
						2 * base_extents[0], 2 * base_extents[1],
						linewidth=2, edgecolor='k', facecolor='none',
						alpha=alpha, zorder=1)
	AX.add_artist(base_rect)

	for obj_id in objs:
		if (FRIDGE and obj_id <= 5):
			continue
		if (not FRIDGE and obj_id <= 1):
			continue

		obj = objs[obj_id]

		obj_shape = obj[0]
		obj_movable = obj[-1]
		ec = 'b' if obj_movable else 'r'
		fc = 'b' if obj_movable else 'r'
		tc = 'w'
		if obj_id >= 100 and obj_id <= 102:
			ec = 'gray'
			fc = 'gray'
			tc = 'k'
		if obj_id == 999:
			ec = 'gold'
			fc = 'gold'
			tc = 'k'

		obj_cent = np.array(obj[2:4])

		if (obj_shape == 0): # rectangle
			obj_extents = np.array(obj[8:10])
			obj_pts = np.vstack([	obj_cent - (obj_extents * [1, 1]),
									obj_cent - (obj_extents * [-1, 1]),
									obj_cent - (obj_extents * [-1, -1]),
									obj_cent - (obj_extents * [1, -1]),
									obj_cent - (obj_extents * [1, 1])]) # axis-aligned

			R = np.array([
					[np.cos(obj[7]), -np.sin(obj[7])],
					[np.sin(obj[7]), np.cos(obj[7])]]) # rotation matrix
			obj_pts = obj_pts - obj_cent # axis-aligned, at origin
			obj_pts = np.dot(obj_pts, R.T) # rotate at origin
			obj_pts = obj_pts + obj_cent # translate back

			path = Path(obj_pts, codes)
			obj_rect = patches.PathPatch(path, ec=ec, fc=fc, lw=1,
								alpha=alpha, zorder=2)

			AX.add_artist(obj_rect)

		elif (obj_shape == 2): # circle
			assert obj[8] == obj[9]

			obj_rad = obj[8]
			obj_circ = patches.Circle(
						obj_cent, radius=obj_rad,
						ec=ec, fc=fc, lw=1,
						alpha=alpha, zorder=2)

			AX.add_artist(obj_circ)

		AX.text(obj_cent[0], obj_cent[1], str(int(obj_id)), color=tc, zorder=3)

	if (trajs):
		cmap = cm.get_cmap('cool')
		for agent, key in enumerate(trajs):
			if key == 99:
				lc = 'lightgray'
			elif key == 999:
				lc = 'orange'
			else:
				# lc = 'cyan'
				lc = cmap(float(agent/len(trajs)))

			traj = np.asarray(trajs[key])
			if (traj.shape[0] > 0):
				AX.plot(traj[:, 0], traj[:, 1], c=lc, alpha=1.0, zorder=25)

			for wp in traj:
				obj = objs[key]

				obj_shape = obj[0]
				obj_movable = obj[-1]
				ec = lc
				fc = lc

				obj_cent = np.array(wp)

				if (obj_shape == 0): # rectangle
					obj_extents = np.array(obj[8:10])
					obj_pts = np.vstack([	obj_cent - (obj_extents * [1, 1]),
											obj_cent - (obj_extents * [-1, 1]),
											obj_cent - (obj_extents * [-1, -1]),
											obj_cent - (obj_extents * [1, -1]),
											obj_cent - (obj_extents * [1, 1])]) # axis-aligned

					R = np.array([
							[np.cos(obj[7]), -np.sin(obj[7])],
							[np.sin(obj[7]), np.cos(obj[7])]]) # rotation matrix
					obj_pts = obj_pts - obj_cent # axis-aligned, at origin
					obj_pts = np.dot(obj_pts, R.T) # rotate at origin
					obj_pts = obj_pts + obj_cent # translate back

					path = Path(obj_pts, codes)
					obj_rect = patches.PathPatch(path, ec=ec, fc=fc, lw=1,
										alpha=0.2, zorder=2)

					AX.add_artist(obj_rect)

				elif (obj_shape == 2): # circle
					assert obj[8] == obj[9]

					obj_rad = obj[8]
					obj_circ = patches.Circle(
								obj_cent, radius=obj_rad,
								ec=ec, fc=fc, lw=1,
								alpha=0.2, zorder=2)

					AX.add_artist(obj_circ)

	if (ngr):
		NGR = np.asarray(ngr)
		NGR = np.unique(NGR, axis=0)
		NGR *= RES
		AX.scatter(NGR[:, 0], NGR[:, 1], c='gray', alpha=0.2, zorder=2)

	# if (goals):
	# 	GOALS = np.asarray(goals)
	# 	AX.scatter(GOALS[:, 1], GOALS[:, 2], c='violet', zorder=11, marker='*')
	# 	for i, oid in enumerate(GOALS[:, 0]):
	# 		AX.text(GOALS[i, 1], GOALS[i, 2], str(int(oid)), color='violet', zorder=3)

	if (pushes):
		cmap = cm.get_cmap('tab10')
		push_debug_legend = []
		failure_modes = {
			0: 'sim fail',
			1: 'no push-object collision',
			2: 'sim success',
			3: 'ik did not reach end',
			4: 'ik obstacle collision',
			5: 'ik joint limits',
			6: 'ik inv vel fail',
			7: 'push start unreachable',
			8: 'push start in object'
		}
		for i in failure_modes:
			push_debug_legend.append(patches.Patch(color=cmap(i), label=failure_modes[i]))

		for p in pushes:
			f = int(p[4]) + 2
			if (p[2] == -99 and p[3] == -99):
				if (f == 8):
					AX.scatter(p[0], p[1], c=[cmap(f)], zorder=24, marker='*', alpha=1)
				elif (f == 7):
					AX.scatter(p[0], p[1], c=[cmap(f)], zorder=21, marker='p', alpha=1)
				else: # backwards compatibility
					AX.scatter(p[0], p[1], c=[cmap(f)], zorder=24, alpha=1)
			else:
				AX.plot([p[0], p[2]], [p[1], p[3]], c=cmap(f), ls='-.', zorder=22, alpha=1)
				AX.scatter(p[0], p[1], c=[cmap(f)], zorder=23, marker='P', alpha=1)
				AX.scatter(p[2], p[3], c=[cmap(f)], zorder=23, marker='X', alpha=1)
				AX.text(p[2], p[3], str(f - 2), color=cmap(f), zorder=33)
		AX.legend(handles=push_debug_legend)

	if (invalids):
		cmap = cm.get_cmap('cool')
		for i, oid in enumerate(invalids):
			invalid_gs = np.asarray(invalids[oid])
			if (invalid_gs.shape[0] > 0):
				AX.scatter(invalid_gs[:, 0], invalid_gs[:, 1], c=[cmap(float(i/len(invalids)))], zorder=31, marker='D')

	AX.axis('equal')
	AX.set_xlim([0.0, 1.2])
	AX.set_ylim([-1.1, 0.0])
	plt.savefig(filepath.replace('txt', 'png'), bbox_inches='tight')
	# plt.savefig(filepath.replace('.txt', '.svg').replace('txt', 'png'), format='svg', dpi=1200, bbox_inches='tight')
	# plt.savefig(filepath.replace('.txt', '.eps').replace('txt', 'png'), format='eps', bbox_inches='tight')
	# plt.show()
	plt.cla()

def main():
	datafolder = os.path.dirname(os.path.abspath(__file__)) + '/../dat/txt/'
	for (dirpath, dirnames, filenames) in os.walk(datafolder):
		for filename in filenames:
			if '.txt' not in filename or 'SOLUTION' in filename:
				continue

			# if int(filename.split('.')[0]) % 10 != 0:
			# 	continue

			filepath = os.path.join(dirpath, filename)
			imgfile = filepath.replace('txt', 'png')
			if os.path.isfile(imgfile):
				continue
			objs, trajs, invalids, ngr, goals, pushes = ParseFile(filepath, False)
			DrawScene(filepath, objs, trajs, invalids, ngr, goals, pushes)

def click_push(scene_id):
	filename = '../../../../simplan/src/simplan/data/clutter_scenes/'

	level = ''
	if (scene_id < 100000):
		level = '0'
	elif (scene_id < 200000):
		level = '5'
	elif (scene_id < 300000):
		level = '10'
	else:
		level = '15'

	filename += level + '/plan_' + str(scene_id) + '_SCENE.txt'
	objs, trajs, invalids, ngr, goals, pushes = ParseFile(filename, False)
	DrawScene(filename, objs, trajs, invalids, ngr, goals, pushes)

if __name__ == '__main__':
	main()

	# scene_id = int(sys.argv[1])
	# click_push(scene_id)
