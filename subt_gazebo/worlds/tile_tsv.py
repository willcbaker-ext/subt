#!/usr/bin/env python3

import argparse
import csv
import math
import os
import sys

tunnel_tile_name_counter = 0
artifact_name_counter = {}
plugin_artifacts = ''

def model_include_string(tileNamePrefix, modelType,
                         pose_x, pose_y, pose_z, pose_yaw,
                         pose_roll='0', pose_pitch='0'):
    if 'tunnel_tile_' in modelType:
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
    return """    <include>
      <name>%s</name>
      <uri>model://%s</uri>
      <pose>%f %f %f %s %s %f</pose>
    </include>
""" % (modelName, modelType,
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
    args = parser.parse_args()
    return args

def print_world_top(args, world_file):
    print("""<?xml version="1.0" ?>
<!--
  Generated with the %s script:
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
    (__file__, ' '.join(sys.argv).replace('--', '-\-'), args.world_name), file=world_file)

def check_main():
    args = parse_args(sys.argv)

    if len(args.world_file) > 0:
        world_file = open(args.world_file, 'w')
    else:
        world_file = sys.stdout

    print_world_top(args, world_file=world_file)

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
                                         yawDegrees * math.pi / 180),
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
                                                 pose_roll=pose_roll,
                                                 pose_pitch=pose_pitch),
                                                 file=world_file)

    print_world_bottom(args, world_file=world_file)

    if len(args.world_file) > 0:
        world_file.close()

def print_world_bottom(args, world_file=sys.stdout):
    global plugin_artifacts
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

  </world>
</sdf>""" %
    (args.world_name, plugin_artifacts, args.wind_x, args.wind_y, args.wind_z), file=world_file)

if __name__ == '__main__':
    check_main()


