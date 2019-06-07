#!/usr/bin/env python

from __future__ import print_function
import argparse
import sys
import csv
import os
import numpy as np


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
    parser = argparse.ArgumentParser('Generate topological dot file from tsv.')
    parser.add_argument('tsv_name', help='name of tsv file to read')
    parser.add_argument('--dot-name', dest='dot_name', type=str, default='',
        help='name of dot file to output')
    args = parser.parse_args()
    return args


def print_graph():

    args = parse_args(sys.argv)

    if not os.path.exists(args.tsv_name):
        print ('Error: Input file does not exist: %s' % args.tsv_name)
        return

    if len(args.dot_name) > 0:
        outfile = open(args.dot_name, 'w')
        usage = ' --dot-name %s' % args.dot_name
    else:
        outfile = sys.stdout
        usage = ''

    print('''/* Visibility graph for %s
   Generated with the %s script:
     python %s %s%s */''' % (
        args.tsv_name, __file__, __file__, args.tsv_name, usage),
        file=outfile)

    # vert_id, vert_id, tile_type, vert_id
    vert_fmt_base = '  %d [label="%d::%s::BaseStation"];'
    vert_fmt = '  %d [label="%d::%s::tile_%d"];'
    # vert1_id, vert2_id, edge_cost
    edge_fmt = '  %d -- %d [label=%d];%s'

    # (iy, ix): iv
    cell_to_iv = dict()
    cell_to_mesh = dict()
    cell_to_yaw = dict()

    # Keep a sorted list of vertex indices. This makes output prettier.
    #   Not needed in python3 - dictionary keys are sorted.
    # [(iy, ix), ...]
    iyx = []


    BASE_MESH = 'base_station'

    print('''
graph {
  /* ==== Vertices ==== */

  /* Base station / Staging area */''', file=outfile)

    # First vertex is base station, not in tsv
    iv = 0
    print(vert_fmt_base % (iv, iv, BASE_MESH), file=outfile)
    #print(vert_fmt % (iv, iv, BASE_MESH, iv), file=outfile)
    print('', file=outfile)
    iv += 1

    with open(args.tsv_name, 'r') as tsvfile:
        spamreader = csv.reader(tsvfile, delimiter='\t')
        for iy, row in enumerate(spamreader):
            for ix, cell in enumerate(row):

                if (len(cell) > 0):
                    for parts in csv.reader([cell]):
                        modelType = parts[0]
                        yawDegrees = float(parts[1])

                        # Ignore artifacts in topology
                        if modelType not in GraphRules.artifacts:
                            print(vert_fmt % (iv, iv, modelType, iv),
                                file=outfile)

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
                         
                            iv += 1


    print('''
  /* ==== Edges ==== */

  /* Base station */''', file=outfile)
    # Base station (vertex 0) to start tunnel tile
    print(edge_fmt % (0, iv_start_tile, GraphRules.calc_edge_cost(BASE_MESH,
        iv_start_type), ''), file=outfile)

    # This suffices for python3, only matters for pretty formatting
    #y, x = zip(*cell_to_iv.keys())
    # Get manually sorted list for Python2
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
              cmt), file=outfile)
        else:
            print('DEBUG: Ambiguity resolved: tile %s (%d) and %s (%d) not connected' % (
                mesh1, iv1, mesh2, iv2), file=sys.stderr)

    print('}', file=outfile)

    if len(args.dot_name) > 0:
        outfile.close()


if __name__ == '__main__':
    print_graph()
