#!usr/bin/env python
import sys
sys.path.insert(0, "../..")
import planning_python.environment_interface.env_2d as env_2d
import os
envfile = os.path.abspath("../../databases/random_bugtrap_small/0.png")
params = {'x_lims': [0, 100], 'y_lims': [0,100]}
print(params)
e = env_2d.Env2D()
e.initialize(envfile, params)
print('Initialized')
print('Origin %f,%f'%(e.orig_pix))

print('Checking collision')
# print(e.collision_free((50,50)))
print("Collision:0, Free: 1, Result: %d"%(e.collision_free((0, 0))))
print('Checking state valid')
print("Invalid:0, Valid: 1, Result: %d"%(e.is_state_valid((50.1, 50.1))))
print('Checking in bounds')
print("Out of bounds: 0, In bounds: 1", e.in_limits((50.1, 50.1)))
