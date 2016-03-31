#!/bin/bash

rosrun xacro xacro.py model.urdf.xacro > tmp.urdf
gzsdf print tmp.urdf > model.sdf
rm tmp.urdf