#!/usr/bin/env python3

import argparse
import csv
import math
import numpy as np
import os
import sys

tunnel_tile_name_counter = 0
artifact_name_counter = {}
plugin_artifacts = ''

def generate_model_name(tileNamePrefix, modelType):
    if 'tunnel_tile_' in modelType:
        global tunnel_tile_name_counter
        modelName = tileNamePrefix + "_" + str(tunnel_tile_name_counter)
        counter = tunnel_tile_name_counter
        tunnel_tile_name_counter += 1
    else:
        global artifact_name_counter
        if not modelType in artifact_name_counter:
            artifact_name_counter[modelType] = 0
        artifact_name_counter[modelType] += 1
        model_type = modelType.lower().replace(' ', '_')
        modelName = model_type + '_' + str(artifact_name_counter[modelType])
        counter = artifact_name_counter[modelType]
        global plugin_artifacts
        plugin_artifacts += """
      <artifact>
        <name>%s</name>
        <type>TYPE_%s</type>
      </artifact>""" % (modelName, model_type.upper())
    return (modelName, counter)

def model_include_string(modelName, modelType,
                         pose_x, pose_y, pose_z, pose_yaw):
    return """    <include>
      <name>%s</name>
      <uri>model://%s</uri>
      <pose>%f %f %f 0 0 %f</pose>
    </include>
""" % (modelName, modelType,
                     float(pose_x), float(pose_y), float(pose_z),
                     float(pose_yaw))

class GraphRules:

    # For computing edge cost
    STRAIGHT = 0
    TURN = 1
    # Key: tile mesh name
    tile_straightness = {
        'base_station': STRAIGHT,
        'tunnel_tile_1': TURN,
        'tunnel_tile_2': TURN,
        'tunnel_tile_3': TURN,
        'tunnel_tile_4': TURN,
        'tunnel_tile_5': STRAIGHT,
        'tunnel_tile_6': STRAIGHT,
        'tunnel_tile_7': TURN,
        'tunnel_tile_blocker': STRAIGHT,
        'constrained_tunnel_tile_tall': STRAIGHT,
        'constrained_tunnel_tile_short': STRAIGHT}

    # For comments in .dot
    intersections = ['tunnel_tile_1', 'tunnel_tile_4']

    # Assumption to constraints: tsv file is a valid tunnel system.
    #     Currently only checking ambiguous cases in a valid tsv specification.
    # Constraint rule 1:
    # Yaw of 90-degree corner tile resolves ambiguous edges e.g. when
    #     all cells in a 2 x 2 block in tsv are occupied
    CORNER_TILE = 'tunnel_tile_2'
    # Constraint rule 2: consecutive blockers cannot be connected
    BLOCKER_TILE = 'tunnel_tile_blocker'
    # Constraint rule 3: parallel non-intersecting (not on same line) linear
    #     tiles cannot be connected. Check yaw to determine connection.
    LINEAR_TILES = ['tunnel_tile_5', 'tunnel_tile_6', 'tunnel_tile_7',
        'constrained_tunnel_tile_tall', 'constrained_tunnel_tile_short']

    # Ignored in scene graph
    artifacts = ['Backpack', 'Electrical Box', 'Extinguisher', 'Phone',
        'Radio', 'Survivor Female', 'Survivor Male', 'Toolbox', 'Valve',
        BLOCKER_TILE]


    @classmethod
    def calc_edge_cost(self, mesh1, mesh2):
        try:
            # Heuristic: if both tiles are straight, cost 1;
            #   if both are turns, cost 6;
            #   otherwise (one is straight, one is a turn), cost 3.
            if self.tile_straightness[mesh1] == self.STRAIGHT and \
                self.tile_straightness[mesh2] == self.STRAIGHT:
                return 1
            elif self.tile_straightness[mesh1] == self.TURN and \
                self.tile_straightness[mesh2] == self.TURN:
                return 6
            else:
                return 3
        except KeyError:
            if mesh1 in self.artifacts or mesh2 in self.artifacts:
                return 0
            else:
                raise


    # Constraint rule 1: corner tile yaw degrees
    #     cdy, cdx: current dy and dx, of cell indices in tsv, with respect to
    #         corner tile.
    @classmethod
    def check_corner_tile_connection(self, cdy, cdx, yaw):

        is_connected = True

        # Hardcoded based on corner tile mesh
        # Yaw degrees are with reference to the corner tile
        # yaw 0, neighbors are necessarily in cells (y=0, x=+1) or (+1, 0)
        #     (right, below)
        if abs (yaw - 0) < 1e-6:
            if not ((cdy == 0 and cdx == 1) or (cdy == 1 and cdx == 0)):
                is_connected = False
        # yaw 90, neighbors are necessarily in cells (0, +1) or (-1, 0)
        #     (right, above)
        elif abs (yaw - 90) < 1e-6:
            if not ((cdy == 0 and cdx == 1) or (cdy == -1 and cdx == 0)):
                is_connected = False
        # yaw 180, neighbors are necessarily in cells (0, -1) or (-1, 0)
        #     (left, above)
        elif abs (yaw - 180) < 1e-6:
            if not ((cdy == 0 and cdx == -1) or (cdy == -1 and cdx == 0)):
                is_connected = False
        # yaw 270 (or -90), neighbors are necessarily in cells (0, -1) or
        #   (+1, 0) (left, below)
        elif abs (yaw - 270) < 1e-6 or abs (yaw + 90) < 1e-6:
            if not ((cdy == 0 and cdx == -1) or (cdy == 1 and cdx == 0)):
                is_connected = False

        return is_connected


    @classmethod
    def check_linear_tile_connection(self, y1, x1, yaw1, y2, x2, yaw2):

        # Hardcoded assumption on meshes:
        # Linear tile meshes are vertical (parallel to world y) at yaw 0,
        #     horizontal (parallel to world x) at yaw 90.

        offset = abs(yaw2 - yaw1)
        # Require consecutive linear tiles to be parallel
        if not (abs(offset - 0) < 1e-6 or abs(offset - 180) < 1e-6):
            return False

        # Require consecutive parallel linear tiles to be on same line.
        #     yaw 0: x1 == x2 (same tsv column) required.
        #     yaw 90: y1 == y2 (same tsv row) required.
        if abs(yaw1 - 0) < 1e-6 or abs(yaw1 - 180) < 1e-6:
            if x1 != x2:
                return False
        elif abs(yaw1 - 90) < 1e-6 or abs(yaw1 - 270) < 1e-6:
            if y1 != y2:
                return False

        return True


def parse_args(argv):
    parser = argparse.ArgumentParser(
        'Generate tiled world and connectivity graph files from tsv. '
        'The graph file is not written if the --graph-file argument is not specified.')
    parser.add_argument('tsv_name', help='name of tsv file to read')
    parser.add_argument('--world-name', dest='world_name', type=str, default='default', help='world name')
    parser.add_argument('--world-file', dest='world_file', type=str, default='', help='world output file')
    parser.add_argument('--graph-file', dest='graph_file', type=str, default='', help='dot graph output file')
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

def print_graph_top(args, graph_file):
    print('''/* Visibility graph for %s
   Generated with the %s script:
     %s */''' % (
        args.tsv_name, __file__, ' '.join(sys.argv).replace('--', '-\-')),
        file=graph_file)

def check_main():
    args = parse_args(sys.argv)

    if len(args.world_file) > 0:
        world_file = open(args.world_file, 'w')
    else:
        world_file = sys.stdout

    if len(args.graph_file) > 0:
        graph_file = open(args.graph_file, 'w')
    else:
        graph_file = open(os.devnull, 'w')

    print_graph_top(args, graph_file=graph_file)
    print_world_top(args, world_file=world_file)

    # vert_id, vert_id, tile_type, vert_id
    vert_fmt_base = '  %s   [label="%s::%s::BaseStation"];'
    vert_fmt = '  %-3d [label="%d::%s::%s"];'
    # vert1_id, vert2_id, edge_cost
    edge_fmt = '  %-2s -- %-3d [label=%d];%s'

    # (iy, ix): iv
    cell_to_iv = dict()
    cell_to_mesh = dict()
    cell_to_yaw = dict()

    # Keep a sorted list of vertex indices. This makes output prettier.
    # [(iy, ix), ...]
    iyx = []


    BASE_MESH = 'base_station'
    base_symbol = 'S'

    print('''
graph {
  /* ==== Vertices ==== */

  /* Base station / Staging area */''', file=graph_file)

    # First vertex is base station, not in tsv
    print(vert_fmt_base % (base_symbol, base_symbol, BASE_MESH), file=graph_file)
    print('', file=graph_file)

    # read from tsv spreadsheet file
    with open(args.tsv_name, 'rt') as tsvfile:
        spamreader = csv.reader(tsvfile, delimiter='\t')
        for iy, row in enumerate(spamreader):
            for ix, cell in enumerate(row):
                if (len(cell) > 0):
                    for parts in csv.reader([cell]):
                        modelType = parts[0]
                        yawDegrees = float(parts[1])
                        z_level = float(parts[2])
                        (modelName, iv) = generate_model_name("tile", modelType)
                        print(model_include_string(modelName, modelType,
                                         args.x0 + ix*args.scale_x,
                                         args.y0 - iy*args.scale_y,
                                         args.z0 + z_level*args.scale_z,
                                         yawDegrees * math.pi / 180),
                                         file=world_file)

                        # Ignore artifacts in topology
                        if modelType not in GraphRules.artifacts:
                            print(vert_fmt % (iv, iv, modelType, modelName),
                                file=graph_file)

                            iyx.append ((iy, ix))

                            # Yaw resolves ambiguous connected vertices
                            cell_to_yaw[(iy, ix)] = yawDegrees
                            cell_to_iv[(iy, ix)] = iv
                            cell_to_mesh[(iy, ix)] = modelType
                         
                            # Assumption: Leftmost column has only one cell,
                            #   the start cell connected to base station
                            if ix == 0:
                                iv_start_tile = iv
                                iv_start_type = modelType

    print('''
  /* ==== Edges ==== */

  /* Base station */''', file=graph_file)
    # Base station (vertex base_symbol) to start tunnel tile
    print(edge_fmt % (base_symbol, iv_start_tile, GraphRules.calc_edge_cost(BASE_MESH,
        iv_start_type), ''), file=graph_file)

    y, x = zip(*iyx)

    # Tile the array to n x n, then use vectorized subtraction
    yt = np.tile(y, (len(y), 1))
    xt = np.tile(x, (len(x), 1))
    dy = np.abs (yt - yt.T)
    dx = np.abs (xt - xt.T)

    dy = np.triu(dy)
    dx = np.triu(dx)

    # Indices of adjacent tiles r and c
    # Take upper triangle, to eliminate symmetric duplicates
    # Horizontal and vertical neighbors are adjacent, i.e. exactly one of dx
    #   and dy is 1.
    adjy1 = np.array(dy == 1)
    adjy0 = np.array(dy == 0)
    adjx1 = np.array(dx == 1)
    adjx0 = np.array(dx == 0)

    # Test (dy == 1 and dx == 0) or (dy == 0 and dx == 1)
    adj_r, adj_c = np.where(np.logical_or(np.logical_and(adjy1, adjx0),
      np.logical_and(adjy0, adjx1)))

    # For each pair of adjacent cells
    for t1, t2 in zip(adj_r, adj_c):
        # Unique vertex (tile) IDs
        iv1 = cell_to_iv[y[t1], x[t1]]
        iv2 = cell_to_iv[y[t2], x[t2]]
        mesh1 = cell_to_mesh[y[t1], x[t1]]
        mesh2 = cell_to_mesh[y[t2], x[t2]]


        # Resolve ambiguities in connectivity, e.g. 2 x 2 blocks in tsv

        is_connected = True

        # Constraint rule 1
        check_corner = False
        # Set corner tile as reference tile, subtract reference tile
        if mesh1 == GraphRules.CORNER_TILE:
            cdy = y[t2] - y[t1]
            cdx = x[t2] - x[t1]
            cy = y[t1]
            cx = x[t1]
            check_corner = True
        elif mesh2 == GraphRules.CORNER_TILE:
            cdy = y[t1] - y[t2]
            cdx = x[t1] - x[t2]
            cy = y[t2]
            cx = x[t2]
            check_corner = True
        if check_corner:
            is_connected = GraphRules.check_corner_tile_connection(
                cdy, cdx, cell_to_yaw[cy, cx])

        # Currently removing blocker from topological graph
        # Constraint rule 2: consecutive blockers can't be connected
        #if mesh1 == GraphRules.BLOCKER_TILE and \
        #    mesh2 == GraphRules.BLOCKER_TILE:
        #    is_connected = False

        # Constraint rule 3: parallel non-intersecting linear tiles aren't nbrs
        if mesh1 in GraphRules.LINEAR_TILES and \
            mesh2 in GraphRules.LINEAR_TILES:
            is_connected = GraphRules.check_linear_tile_connection(
                y[t1], x[t1], cell_to_yaw[y[t1], x[t1]],
                y[t2], x[t2], cell_to_yaw[y[t2], x[t2]])

        if is_connected:
            cmt = ''
            if mesh1 in GraphRules.intersections:
                cmt = '  /* Intersection */'
            print(edge_fmt % (iv1, iv2, GraphRules.calc_edge_cost(mesh1, mesh2),
              cmt), file=graph_file)
        else:
            print('DEBUG: Ambiguity resolved: tile %s (%d) and %s (%d) not connected' % (
                mesh1, iv1, mesh2, iv2), file=sys.stderr)

    print_graph_bottom(args, graph_file=graph_file)
    print_world_bottom(args, world_file=world_file)

    if len(args.world_file) > 0:
        world_file.close()

    if len(args.graph_file) > 0:
        graph_file.close()

def print_graph_bottom(args, graph_file):
    print('}', file=graph_file)

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

