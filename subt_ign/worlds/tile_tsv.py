#!/usr/bin/env python
from __future__ import print_function
import argparse
import csv
import math
import os
import sys
import yaml

tunnel_tile_name_counter = 0
artifact_name_counter = {}
plugin_artifacts = ''
# Maps from tuple representing the center of level bounding box, to Level
plugin_levels = {}
levels_counter = 0

class Level:

    def __init__(self, name, x, y, z, sx, sy, sz, buf=10):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.sx = sx
        self.sy = sy
        self.sz = sz
        self.buffer = buf

        # List of strings of model names
        self.refs = []

    def add_ref(self, ref_name):
        self.refs.append(ref_name)
 
    def str(self):
        s = """
      <level name="%s">""" % self.name

        for ref in self.refs:
            s += """
        <ref>%s</ref>""" % ref

        s += """
        <pose>%f %f %f 0 0 0</pose>
        <geometry><box><size>%f %f %f</size></box></geometry>
        <buffer>%f</buffer>
      </level>""" % (self.x, self.y, self.z, self.sx, self.sy, self.sz, self.buffer)

        return s

# Calculate the center of the level to put a model, based on the model center
# and bounding box size of levels. Assumes all levels have the same size.
def calculate_level_center(model_x, model_y, model_z, levels_sx, levels_sy, levels_sz):

    lx = math.trunc(model_x / levels_sx) * levels_sx + 0.5 * levels_sx
    ly = math.trunc(model_y / levels_sy) * levels_sy + 0.5 * levels_sy
    lz = math.trunc(model_z / levels_sz) * levels_sz + 0.5 * levels_sz

    return (lx, ly, lz)

def expand_levels(levels_dictionary):
    s = ''
    for level in levels_dictionary.values():
        s += level.str()
    return s

def model_include_string(tileNamePrefix, modelType,
                         pose_x, pose_y, pose_z, pose_yaw,
                         levels_sx, levels_sy, levels_sz, levels_buf,
                         types_to_paths=None):
    if types_to_paths is None:
        types_to_paths_f = lambda t : t
    else:
        types_to_paths_f = lambda t : types_to_paths[t]

    if 'tunnel_tile_' in modelType or 'Urban' in modelType:
        global tunnel_tile_name_counter
        modelName = tileNamePrefix + "_" + str(tunnel_tile_name_counter)
        tunnel_tile_name_counter += 1
    else:
        global artifact_name_counter
        if not modelType in artifact_name_counter:
            artifact_name_counter[modelType] = 0
        artifact_name_counter[modelType] += 1
        model_type = modelType.lower().replace(' ', '_')
        modelName = model_type + '_' + str(artifact_name_counter[modelType])
        global plugin_artifacts
        plugin_artifacts += """
      <artifact>
        <name>%s</name>
        <type>TYPE_%s</type>
      </artifact>""" % (modelName, model_type.upper())

    # Levels plugin for this model
    global plugin_levels
    global levels_counter
    lx, ly, lz = calculate_level_center(
        float(pose_x), float(pose_y), float(pose_z),
        levels_sx, levels_sy, levels_sz)
    if (lx, ly, lz) not in plugin_levels.keys():
        plugin_levels[(lx, ly, lz)] = Level("level" + str(levels_counter),
            lx, ly, lz, levels_sx, levels_sy, levels_sz, levels_buf)
        levels_counter += 1
    plugin_levels[(lx, ly, lz)].add_ref(modelName)

    return """    <include>
      <name>%s</name>
      <uri>%s</uri>
      <pose>%f %f %f 0 0 %f</pose>
    </include>
""" % (modelName, types_to_paths_f(modelType),
                     float(pose_x), float(pose_y), float(pose_z),
                     float(pose_yaw))

def print_tsv_model_includes(args):
    types_to_paths = None
    if (os.path.exists(args.map_file)):
        with open(args.map_file, 'rb') as mapf:
            types_to_paths = yaml.load(mapf)

    with open(args.file_name, 'rb') as tsvfile:
        spamreader = csv.reader(tsvfile, delimiter='\t')
        for iy, row in enumerate(spamreader):
            for ix, cell in enumerate(row):
                if (len(cell) > 0):
                    for parts in csv.reader([cell]):
                        modelType = parts[0]
                        yawDegrees = float(parts[1])
                        z_level = float(parts[2])
                        print(model_include_string("tile", modelType,
                                         args.x0 + ix*args.scale_x,
                                         args.y0 - iy*args.scale_y,
                                         args.z0 + z_level*args.scale_z,
                                         yawDegrees * math.pi / 180,
                                         args.levels_sx, args.levels_sy, args.levels_sz, args.levels_buf,
                                         types_to_paths))

def parse_args(argv):
    parser = argparse.ArgumentParser('Generate tiled world file from tsv.')
    parser.add_argument('file_name', help='name of tsv file to read')
    parser.add_argument('--world-name', dest='world_name', type=str, default='default', help='world name')
    parser.add_argument('--x0', dest='x0', type=float, default=0, help='origin X coordinate')
    parser.add_argument('--y0', dest='y0', type=float, default=0, help='origin Y coordinate')
    parser.add_argument('--z0', dest='z0', type=float, default=0, help='origin Z coordinate')
    parser.add_argument('--scale_x', dest='scale_x', type=float, default=20, help='tile scale in X')
    parser.add_argument('--scale_y', dest='scale_y', type=float, default=20, help='tile scale in Y')
    parser.add_argument('--scale_z', dest='scale_z', type=float, default=5,  help='tile scale in Z')
    parser.add_argument('--wind_x', dest='wind_x', type=float, default=0, help='global wind velocity in X')
    parser.add_argument('--wind_y', dest='wind_y', type=float, default=0, help='global wind velocity in Y')
    parser.add_argument('--wind_z', dest='wind_z', type=float, default=0, help='global wind velocity in Z')
    parser.add_argument('--levels_sx', dest='levels_sx', type=float, default=200, help='Levels box size in X')
    parser.add_argument('--levels_sy', dest='levels_sy', type=float, default=200, help='Levels box size in Y')
    parser.add_argument('--levels_sz', dest='levels_sz', type=float, default=200, help='Levels box size in Z')
    parser.add_argument('--levels_buf', dest='levels_buf', type=float, default=20, help='Levels box buffer size')
    parser.add_argument('--map-file', type=str,
        default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "type_to_path.yaml"),
        help='YAML file mapping model types in tsv to file paths')
    args = parser.parse_args()
    return args

def check_main():
    args = parse_args(sys.argv)
    print("""<?xml version="1.0" ?>
<!--
  Generated with the tile_tsv.py script:
    %s
-->
<sdf version="1.6">
  <world name="%s">

    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>-6.3 -4.2 3.6 0 0.268 0.304</pose>
      </camera>
    </gui>

    <scene>
      <ambient>0.2 0.2 0.2 1.0</ambient>
      <background>0.34 0.39 0.43 1.0</background>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
    </scene>

    <!-- The base station / staging area -->
    <!-- Important: Do not rename this model! -->
    <include>
      <static>true</static>
      <name>BaseStation</name>
      <pose>0 0 0 0 0 0</pose>
      <uri>model://tunnel_staging_area</uri>
    </include>

    <!-- Fiducial marking the origin for artifacts reports -->
    <include>
      <name>artifact_origin</name>
      <pose>2 4 0.5 0 0 0</pose>
      <uri>model://fiducial</uri>
    </include>


    <!-- Tunnel tiles and artifacts -->""" %
  (' '.join(sys.argv).replace('--', '-\-'), args.world_name))
    print_tsv_model_includes(args)

    global plugin_artifacts
    global plugin_levels
    print("""
    <!-- The SubT challenge logic plugin -->
    <plugin name="game_logic_plugin" filename="libGameLogicPlugin.so">
      <logging>
        <filename_prefix>subt_%s</filename_prefix>
      </logging>
      <!-- The collection of artifacts to locate -->
%s
    </plugin>

    <!-- The SubT comms broker plugin -->
    <plugin name="comms_broker_plugin" filename="libCommsBrokerPlugin.so">
      <comms_model>
        <neighbor_distance_min>0.0</neighbor_distance_min>
        <neighbor_distance_max>100.0</neighbor_distance_max>
        <comms_distance_min>0.0</comms_distance_min>
        <comms_distance_max>100.0</comms_distance_max>
        <comms_drop_probability_min>0.0</comms_drop_probability_min>
        <comms_drop_probability_max>0.0</comms_drop_probability_max>
        <comms_outage_probability>0.0</comms_outage_probability>
        <comms_outage_duration_min>0.0</comms_outage_duration_min>
        <comms_outage_duration_max>10.0</comms_outage_duration_max>
      </comms_model>
    </plugin>

    <!-- rotors_gazebo support -->
    <plugin name="ros_interface_plugin"
            filename="librotors_gazebo_ros_interface_plugin.so"/>

    <wind>
      <linear_velocity>%f %f %f</linear_velocity>
    </wind>

    <!-- Load the plugin for the wind -->
    <plugin name="wind" filename="libWindPlugin.so">
      <horizontal>
        <magnitude>
          <time_for_rise>10</time_for_rise>
          <sin>
            <amplitude_percent>0.05</amplitude_percent>
            <period>60</period>
          </sin>
          <noise type="gaussian">
           <mean>0</mean>
           <stddev>0.0002</stddev>
          </noise>
        </magnitude>
        <direction>
          <time_for_rise>30</time_for_rise>
          <sin>
            <amplitude>5</amplitude>
            <period>20</period>
          </sin>
          <noise type="gaussian">
           <mean>0</mean>
           <stddev>0.03</stddev>
          </noise>
        </direction>
      </horizontal>
      <vertical>
        <noise type="gaussian">
         <mean>0</mean>
         <stddev>0.03</stddev>
        </noise>
      </vertical>
    </plugin>

    <!-- Levels plugin -->
    <plugin name="ignition::gazebo" filename="dummy">
      <performer name="perf_x2">
        <ref>X2</ref>
        <geometry><box><size>2 2 2</size></box></geometry>
      </performer>

      <performer name="perf_x1">
        <ref>X1</ref>
        <geometry><box><size>2 2 2</size></box></geometry>
      </performer>

%s
    </plugin>

  </world>
</sdf>""" %
(args.world_name, plugin_artifacts, args.wind_x, args.wind_y, args.wind_z, expand_levels(plugin_levels)))
        
if __name__ == '__main__':
    check_main()
