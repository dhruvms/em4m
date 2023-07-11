import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib.cm as cm

import os
import sys
import collections

DATADIR = '/home/dhruv/work/code/ros/simplan/src/simplan/data/clutter_scenes/'
SAVEDIR = DATADIR + 'nb/'

def GetOrigPlanFilepath(scene_id):
	sid = int(scene_id)
	level = ''
	if (sid < 100000):
		level = '0/'
	elif (sid < 200000):
		level = '5/'
	elif (sid < 300000):
		level = '10/'
	else:
		level = '15/'

	filepath = DATADIR + level + 'plan_{}_SCENE.txt'.format(scene_id)
	return filepath

def CreateNewSceneFile(filepath, new_scene_id):
	old_scene_id = filepath.split('_')[2]

	num_objs = 0
	ooid = None
	objs = {}
	pushes = []
	with open(filepath, 'r') as f:
		done = False
		while not done:
			line = f.readline()[:-1]

			if len(line) == 0:
				done = True

			if line == 'O':
				line = f.readline()[:-1]
				num_objs = int(line)
				for i in range(num_objs):
					line = f.readline()[:-1]
					obj = [float(val) for val in line.split(',')[:-1]]
					# line = f.readline()[:-1]
					# objs[obj[0]] += [line == 'True']
					objs[obj[0]] = obj[1:]
					objs[obj[0]] += [line.split(',')[-1] == 'True']

				ooid = np.setxor1d(list(objs.keys()), np.arange(1, num_objs+1).astype('float'))
				ooid = ooid[ooid != 999][0]
				objs[ooid] = objs.pop(999)
				objs = collections.OrderedDict(sorted(objs.items()))

			if line == 'PUSHES':
				line = f.readline()[:-1]
				num_pushes = int(line)
				for i in range(num_pushes):
					line = f.readline()[:-1]
					push = [float(val) for val in line.split(',')]
					pushes.append(push[-1])

	if pushes and all(np.array(pushes) != -1):
		return False
	else:
		newfilename = SAVEDIR + 'plan_{}_SCENE.txt'.format(new_scene_id)
		with open(newfilename, 'w') as f:
			f.write('O\n')
			f.write(str(num_objs) + '\n')
			for oid in objs:
				o = objs[oid]
				o_str = str(int(oid)) + ','
				o_str += ','.join([str(int(x)) for x in o[:2]]) + ','
				o_str += ','.join([str(x) for x in o[2:-1]]) + ','
				o_str += 'True\n' if o[-1] == 1 else 'False\n'
				f.write(o_str)

			f.write('R\n')
			f.write('0.0,0.0,0.0,0.0,0.0,0.0,1.0\n')

			f.write('S\n')
			f.write('8\n')
			f.write('torso_lift_joint,0.1\n')
			f.write('r_shoulder_pan_joint,-1.68651\n')
			f.write('r_shoulder_lift_joint,-0.184315\n')
			f.write('r_upper_arm_roll_joint,-1.67611\n')
			f.write('r_elbow_flex_joint,-1.66215\n')
			f.write('r_forearm_roll_joint,0.285096\n')
			f.write('r_wrist_flex_joint,-0.658798\n')
			f.write('r_wrist_roll_joint,1.41563\n')

			f.write('ooi\n')
			f.write(str(int(ooid)) + '\n')

			old_scene_file = GetOrigPlanFilepath(old_scene_id)
			with open(old_scene_file, 'r') as of:
				done = False
				while not done:
					line = of.readline()[:-1]

					if len(line) == 0:
						done = True

					if line == 'G':
						line = of.readline()[:-1]
						f.write('G\n')
						f.write(line + '\n')
						break

		return True

def main():
	new_scene_ids = np.arange(900000, 1000000)

	i = 0
	datafolder = os.path.dirname(os.path.abspath(__file__)) + '/../dat/results/EM4M-ICAPS23/Naive_Bayes/txt/'
	for (dirpath, dirnames, filenames) in os.walk(datafolder):
		for filename in filenames:
			if '.txt' not in filename or 'SOLUTION' in filename:
				continue

			filepath = os.path.join(dirpath, filename)
			saved = CreateNewSceneFile(filepath, new_scene_ids[i])
			if (saved):
				i += 1

if __name__ == '__main__':
	main()
