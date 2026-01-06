#!/usr/bin/env python
#
# Copyright (c) Diligent Robotics, 2018
# Copyright (c) Adam Allevato, 2018
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
################################################################################
#
# Introduction
##############
# This script generates gazebo models for black and white checkerboards for use
# in visual detection or calibration. The checkerboards are pure black and white
# with a white border, which allows them to be used with systems such as
# OpenCV's chessboard detectors.
# 
# This script could also be used as a jumping-off point to create an automatic
# AR tag generator.
#
# Usage
#######
# The following command will generate a new folder and populate it with a Gazebo
# model.
#
# generate_checkerboard.py rows columns [square_size_in_meters] [border_width]
#
# This folder can be placed anywhere in your Gazebo models path, such as the 
# default location: $HOME/.gazebo/models.
################################################################################

import sys
import os

# basic argument checking
# this script is not complicated enough to merit using a system such as argparse
if len(sys.argv) < 3:
    print("usage is: generate_checkerboard.py rows columns "
          "[square_size_in_meters] [border_width]")
    print("Note: border_width is optional and defaults to half square size")
    exit()

# in meters
DEFAULT_SQUARE_SIZE = 0.0254

# set up shop
rows = int(sys.argv[1])  # Changed to match typical checkerboard convention
cols = int(sys.argv[2])
if len(sys.argv) > 3:
    sq_size = float(sys.argv[3])
else:
    sq_size = DEFAULT_SQUARE_SIZE

if len(sys.argv) > 4:
    border_width = float(sys.argv[4])
else:
    border_width = sq_size / 2.0

# Create descriptive names
name = "_".join(["checkerboard", f"{rows}x{cols}",
                 f"square{str(sq_size).replace('.', '_')}",
                 f"border{str(border_width).replace('.', '_')}"])
pretty_name = f"Checkerboard {rows}x{cols}, square {sq_size}m, border {border_width}m"

# create directory for the model if it doesn't exist.
# If it does, don't overwrite it!
directory = name
if not os.path.exists(directory):
    os.makedirs(directory)
else:
    print(f"directory {name} already exists. Aborting.")
    sys.exit(2)

# parts of the SDF file that don't depend on the size
sdf_preamble = """<?xml version="1.0"?>
<sdf version="1.4">
  <model name="{name}">
  <static>true</static>
  <link name="{name}_body">\n""".format(name=name)

sdf_postamble = """    </link>
  </model>
</sdf>\n"""

# generate the SDF code for the checkerboard itself
half_width = (cols * sq_size) / 2.0
half_height = (rows * sq_size) / 2.0
visual_elements = []

for i in range(cols):
    for j in range(rows):
        # Create RGBA color: black (0,0,0,1) or white (1,1,1,1)
        rgba = " ".join([str((i + j) % 2)] * 3) + " 1"
        
        # Calculate position - using first script's coordinate system (more intuitive)
        x = i * sq_size + sq_size / 2 - half_width
        y = j * sq_size + sq_size / 2 - half_height
        
        element = """      <visual name="checker_{i}_{j}">
        <pose>{x} {y} 0 0 0 0</pose>
        <geometry>
          <box>
            <size>{size} {size} 0.0002</size>
          </box>
        </geometry>
        <material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
          <specular>{rgba}</specular>
          <emissive>{rgba}</emissive>
        </material>
      </visual>""".format(i=i, j=j, x=x, y=y, rgba=rgba, size=sq_size)
        visual_elements.append(element)

# Create the white backdrop with custom border width
xsize = cols * sq_size + 2 * border_width
ysize = rows * sq_size + 2 * border_width

backdrop = """      <visual name="checker_backdrop">
        <pose>0.0 0.0 -0.0001 0 0 0</pose>
        <geometry>
          <box>
            <size>{xsize} {ysize} 0.0001</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>1 1 1 1</specular>
          <emissive>1 1 1 1</emissive>
        </material>
      </visual>""".format(xsize=xsize, ysize=ysize)
visual_elements.append(backdrop)

# Add collision element (from second script)
collision = """    <collision name="collision">
        <pose>0.0 0.0 -0.0001 0 0 0</pose>
        <geometry>
          <box>
            <size>{xsize} {ysize} 0.0001</size>
          </box>
        </geometry>
    </collision>
""".format(xsize=xsize, ysize=ysize)

# create the final SDF contents
sdf_visuals = "\n".join(visual_elements)
sdf_string = sdf_preamble + sdf_visuals + collision + sdf_postamble

# save the SDF
sdf_path = os.path.join(name, "checkerboard.sdf")
with open(sdf_path, "w") as f:
    f.write(sdf_string)
print(f"wrote checkerboard to {sdf_path}")

# generate the config file to go with the sdf
config_string = """<?xml version='1.0'?>
<model>
  <name>{pretty_name}</name>
  <version>1.0.0</version>
  <sdf version='1.4'>checkerboard.sdf</sdf>
  <author>
    <name>checkerboard_generator.py</name>
    <email>N/A</email>
  </author>
</model>
""".format(pretty_name=pretty_name)

# save the config file
model_filepath = os.path.join(name, "model.config")
with open(model_filepath, "w") as f:
    f.write(config_string)
print(f"wrote config file to {model_filepath}")