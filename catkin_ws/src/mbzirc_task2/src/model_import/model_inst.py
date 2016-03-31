#!/usr/bin/python

import shutil
import os
import errno

home = os.environ['HOME'];
print home

model_dir = os.path.dirname(home + '/.gazebo/models/')
print model_dir	
new_model_resource = os.path.split(os.path.split(os.path.dirname\
	(os.path.abspath(__file__)))[0])[0] + '/models/'

def copy(src, dest):
    try:
        shutil.copytree(src, dest)
    except OSError as e:
        # If the error was caused because the source wasn't a directory
        if e.errno == errno.ENOTDIR:
            shutil.copy(src, dest)
        else:
            print('Directory not copied. Error: %s' % e)
    	return False
    return True

def add_model(name):
	if not os.path.isdir(model_dir + '/' + name):
	 	if (copy(new_model_resource + '/' + name, model_dir + '/' + name)):
	 		print '\tmodel ' + name + ' added'
	 	else:
	 		print '\tERROR copying model ' + name + 'to gazebo models folder'
	else:
	 	print '\tmodel ' + name + ' exists'


def copy_models():
	print 'Copying models from',
	print os.path.abspath(new_model_resource)
	for model in os.listdir(new_model_resource):
		add_model(model)