#!/usr/bin/env python3

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
                         pose_roll='0', pose_pitch='0',
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
      <pose>%f %f %f %s %s %f</pose>
    </include>
""" % (modelName, types_to_paths_f(modelType),
                     float(pose_x), float(pose_y), float(pose_z),
                     pose_roll, pose_pitch, float(pose_yaw))

def parse_args(argv):
    parser = argparse.ArgumentParser('Generate tiled world file from tsv.')
    parser.add_argument('tsv_name', help='name of tsv file to read')
    parser.add_argument('--world-name', dest='world_name', type=str, default='default', help='world name')
    parser.add_argument('--world-file', dest='world_file', type=str, default='', help='world output file')
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

def print_world_top(args, world_file):
    print("""<?xml version="1.0" ?>
<!--
  Generated with the {file_name} script:
    {command}
-->
<sdf version="1.6">
  <world name="{world_name}">

    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin
      filename="libignition-gazebo-physics-system.so"
      name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin
      filename="libignition-gazebo-user-commands-system.so"
      name="ignition::gazebo::systems::UserCommands">
    </plugin>
    <plugin
      filename="libignition-gazebo-scene-broadcaster-system.so"
      name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>

    <gui fullscreen="0">

      <!-- 3D scene -->
      <plugin filename="Scene3D" name="3D View">
        <ignition-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </ignition-gui>

        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <camera_pose>-6 0 6 0 0.5 0</camera_pose>
        <service>/world/{world_name}/scene/info</service>
        <pose_topic>/world/{world_name}/pose/info</pose_topic>
        <scene_topic>/world/{world_name}/scene/info</scene_topic>
        <deletion_topic>/world/{world_name}/scene/deletion</deletion_topic>
      </plugin>

      <!-- World control -->
      <plugin filename="WorldControl" name="World control">
        <ignition-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="width">121</property>
          <property type="double" key="z">1</property>

          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </ignition-gui>

        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>true</start_paused>
        <service>/world/{world_name}/control</service>
        <stats_topic>/world/{world_name}/stats</stats_topic>

      </plugin>

      <!-- World statistics -->
      <plugin filename="WorldStats" name="World stats">
        <ignition-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>

          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </ignition-gui>

        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
        <topic>/world/{world_name}/stats</topic>

      </plugin>

    </gui>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>


    <!-- The base station / staging area -->
    <!-- Important: Do not rename this model! -->
    <include>
      <static>true</static>
      <name>BaseStation</name>
      <pose>0 0 0 0 0 0</pose>
      <uri>https://fuel.ignitionrobotics.org/1.0/openrobotics/models/subt_tunnel_staging_area/2</uri>
    </include>

    <!-- Fiducial marking the origin for artifacts reports -->
    <include>
      <name>artifact_origin</name>
      <pose>2 4 0.5 0 0 0</pose>
      <uri>https://fuel.ignitionrobotics.org/1.0/openrobotics/models/Fiducial</uri>
    </include>


    <!-- Tunnel tiles and artifacts -->""".format(
    file_name=__file__, command=' '.join(sys.argv).replace('--', '-\-'), world_name=args.world_name), file=world_file)

def check_main():
    args = parse_args(sys.argv)

    if len(args.world_file) > 0:
        world_file = open(args.world_file, 'w')
    else:
        world_file = sys.stdout

    print_world_top(args, world_file=world_file)

    types_to_paths = None
    if (os.path.exists(args.map_file)):
        with open(args.map_file, 'rb') as mapf:
            types_to_paths = yaml.load(mapf)

    with open(args.tsv_name, 'rt') as tsvfile:
        spamreader = csv.reader(tsvfile, delimiter='\t')
        for iy, row in enumerate(spamreader):
            for ix, cell in enumerate(row):
                if (len(cell) > 0):
                    for parts in csv.reader([cell]):
                        # each cell of spreadsheet contains comma-separated
                        # value strings with at least 3 fields
                        # modelType,yawDegrees,z_level eg. 'tunnel_tile_1,180,-1'
                        # it can have more fields, which are described below
                        modelType = parts[0]
                        yawDegrees = float(parts[1])
                        z_level = float(parts[2])
                        pose_x = args.x0 + ix*args.scale_x
                        pose_y = args.y0 - iy*args.scale_y
                        pose_z = args.z0 + z_level*args.scale_z
                        print(model_include_string("tile", modelType,
                                         pose_x, pose_y, pose_z,
                                         yawDegrees * math.pi / 180,
                                         args.levels_sx, args.levels_sy, args.levels_sz, args.levels_buf,
                                         types_to_paths=types_to_paths),
                                         file=world_file)
                        # the 4th field is another string that contains a list
                        # of submodels and a relative pose where they are to be
                        # placed within the tile
                        # submodels are separated by `;` eg. 'Phone;tunnel_tile_blocker'
                        # a relative pose can be specified with @
                        # pose is specified like in sdformat but with angles in degrees
                        # eg. 'Phone@0 0 0.004 90 0 0;tunnel_tile_blocker@11 0 0 0 0 0'
                        if len(parts) > 3:
                            submodels = parts[3]
                            for submodel in submodels.split(';'):
                                pose_xi = pose_x
                                pose_yi = pose_y
                                pose_zi = pose_z
                                # pose_roll and pose_pitch are printed as strings for now
                                # to minimize the diff for tunnel_qual.world
                                # which has a bunch of poses with no trailing zeros for roll and pitch
                                # like '100.0000 200.0000 0.0000 0 0 -1.57079'
                                # so let pose_roll and pose_pitch default to '0'
                                pose_roll = '0'
                                pose_pitch = '0'
                                pose_yaw = 0.0
                                # separate name from pose string by splitting at `@`
                                submodelType = ''
                                poseStr = ''
                                submodelType_poseStr = submodel.split('@')
                                if len(submodelType_poseStr) == 0:
                                    print("ERROR: invalid submodel specification %s" % submodel)
                                    continue
                                submodelType = submodelType_poseStr[0]
                                # pose is optional
                                if len(submodelType_poseStr) >= 2:
                                    poseStr = submodelType_poseStr[1]
                                pose = poseStr.split(' ')
                                # set position if only 3 pose values are given
                                if len(pose) >= 3:
                                    pose_xi += float(pose[0])
                                    pose_yi += float(pose[1])
                                    pose_zi += float(pose[2])
                                # additionally set roll, pitch, yaw if 6 values are given
                                if len(pose) == 6:
                                    # print pose_roll and pose_pitch as %f if
                                    # they aren't exactly '0'
                                    if pose_roll != pose[3]:
                                        pose_roll = '%f' % (float(pose[3]) * math.pi / 180)
                                    if pose_pitch != pose[4]:
                                        pose_pitch = '%f' % (float(pose[4]) * math.pi / 180)
                                    pose_yaw = float(pose[5]) * math.pi / 180
                                print(model_include_string("tile", submodelType,
                                                 pose_xi, pose_yi, pose_zi,
                                                 pose_yaw * math.pi / 180,
                                                 args.levels_sx, args.levels_sy, args.levels_sz, args.levels_buf,
                                                 pose_roll=pose_roll,
                                                 pose_pitch=pose_pitch,
                                                 types_to_paths=types_to_paths),
                                                 file=world_file)

    print_world_bottom(args, world_file=world_file)

    if len(args.world_file) > 0:
        world_file.close()

def print_world_bottom(args, world_file=sys.stdout):
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
        <comms_model_type>visibility_range</comms_model_type>

        <range_config>
          <max_range>500.0</max_range>
          <fading_exponent>2.5</fading_exponent>
          <L0>40</L0>
          <sigma>10.0</sigma>
        </range_config>

        <visibility_config>
          <visibility_cost_to_fading_exponent>0.2</visibility_cost_to_fading_exponent>
          <comms_cost_max>15</comms_cost_max>
        </visibility_config>

        <radio_config>
          <capacity>1000000</capacity>
          <tx_power>20</tx_power>
          <noise_floor>-90</noise_floor>
          <modulation>QPSK</modulation>
        </radio_config>
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
    (args.world_name, plugin_artifacts, args.wind_x, args.wind_y, args.wind_z, expand_levels(plugin_levels)), file=world_file)

if __name__ == '__main__':
    check_main()
