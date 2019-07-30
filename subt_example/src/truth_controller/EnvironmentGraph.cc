/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "EnvironmentGraph.hh"

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include <vector>

using namespace boost;

namespace subt_example
{

struct vertex_label_t { typedef vertex_property_tag kind; };

using vertex_p = property<vertex_name_t, std::string,
                 property<vertex_label_t, std::string>>;
using edge_p = property<edge_name_t, std::string>;
using graph_t = adjacency_list<vecS, vecS, undirectedS, vertex_p, edge_p>;

void ParseGraph(const std::string &_dotFilename, Graph *_g)
{
  graph_t graph(0);
  dynamic_properties dp;

  property_map<graph_t, vertex_name_t>::type vname =
    get(vertex_name, graph);
  dp.property("node_id", vname);

  property_map<graph_t, vertex_label_t>::type vlabel =
    get(vertex_label_t(), graph);
  dp.property("label", vlabel);

  property_map<graph_t, edge_name_t>::type elabel =
    get(edge_name, graph);
  dp.property("label", elabel);

  auto ifs = std::ifstream(_dotFilename);
  bool status = read_graphviz(ifs, graph, dp, "node_id");
  ignmsg << status << std::endl;

  std::vector<Vertex> verts;

  auto vs = vertices(graph);
  std::cout << "Found vertices: " << num_vertices(graph) << std::endl;
  for (auto vit = vs.first; vit != vs.second; ++vit)
  {
    auto node_id = get("node_id", dp, *vit);
    auto label = get("label", dp, *vit);
    auto id = std::atoi(node_id.c_str());
    auto tokens = ignition::common::split(label, "::");

    if (tokens.size() != 3) {
      std::cerr << "Error splitting: " << label << std::endl;
      continue;
    }

    VertexData vd;
    vd.tileName= tokens[2];
    vd.tileType = tokens[1];
    _g->AddVertex(label, vd, id);
  }
  std::cout << "Inserted vertices: " << _g->Vertices().size() << std::endl;

  auto es = edges(graph);

  std::cout << "Found edges: " << num_edges(graph) << std::endl;
  for (auto eit = es.first; eit != es.second; ++eit)
  {
    auto s = source(*eit, graph);
    auto s_id = get("node_id", dp, s);
    auto s_id_int = std::atoi(s_id.c_str());

    auto t = target(*eit, graph);
    auto t_id = get("node_id", dp, t);
    auto t_id_int = std::atoi(t_id.c_str());

    if(!_g->AddEdge({s_id_int, t_id_int}, 0.0).Valid())
    {
      std::cout << "Ignored edge" << s << " " << t << std::endl;
    }
  }
  std::cout << "Inserted Edges: " << _g->Edges().size() << std::endl;
}

}
