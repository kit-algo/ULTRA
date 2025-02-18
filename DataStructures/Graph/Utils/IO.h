#pragma once

#include <vector>

#include "../Classes/GraphInterface.h"

#include "../Classes/DynamicGraph.h"
#include "../Classes/EdgeList.h"
#include "../Classes/StaticGraph.h"

#include "Conversion.h"

#include "../../Geometry/Point.h"

#include "../../../Helpers/Assert.h"
#include "../../../Helpers/IO/Serialization.h"

namespace Graph {

template <typename GRAPH>
inline void fromDimacs(const std::string &fileBaseName, GRAPH &graph) noexcept {
  EdgeList<typename GRAPH::ListOfVertexAttributes,
           typename GRAPH::ListOfEdgeAttributes>
      edgeList;
  edgeList.fromDimacs(fileBaseName);
  move(std::move(edgeList), graph);
}

template <typename GRAPH, typename WEIGHT_TYPE>
inline void toDimacs(const std::string &fileBaseName, const GRAPH &graph,
                     const std::vector<WEIGHT_TYPE> &weight) noexcept {
  std::ofstream grOs(fileBaseName + ".gr");
  Assert(grOs, "Cannot create output stream for " << fileBaseName << ".gr");
  Assert(grOs.is_open(),
         "Cannot open output stream for " << fileBaseName << ".gr");
  grOs << "p sp " << graph.numVertices() << " " << graph.numEdges()
       << std::endl;
  for (const auto [edge, from] : graph.edgesWithFromVertex()) {
    grOs << "a " << (from + 1) << " " << (graph.get(ToVertex, edge) + 1) << " "
         << weight[edge] << std::endl;
  }
  grOs.close();
  if constexpr (GRAPH::HasVertexAttribute(Coordinates)) {
    std::ofstream coOs(fileBaseName + ".co");
    Assert(coOs, "Cannot create output stream for " << fileBaseName << ".co");
    Assert(coOs.is_open(),
           "Cannot open output stream for " << fileBaseName << ".co");
    coOs << "p aux sp co " << graph.numVertices() << std::endl;
    for (const Vertex vertex : graph.vertices()) {
      coOs << "v " << (vertex + 1) << " " << graph.get(Coordinates, vertex).x
           << " " << graph.get(Coordinates, vertex).y << std::endl;
    }
    coOs.close();
  }
}

template <typename GRAPH>
inline void toDimacs(const std::string &fileBaseName,
                     const GRAPH &graph) noexcept {
  toDimacs(fileBaseName, graph, graph.get(Weight));
}

template <typename GRAPH>
inline void toEdgeListCSV(const std::string &fileBaseName,
                          const GRAPH &graph) noexcept {
  std::ofstream csv(fileBaseName + ".csv");
  Assert(csv, "Cannot create output stream for " << fileBaseName << ".csv");
  Assert(csv.is_open(),
         "Cannot open output stream for " << fileBaseName << ".csv");

  csv << "FromVertex,ToVertex";

  if constexpr (GRAPH::HasEdgeAttribute(TravelTime))
    csv << ",TravelTime";
  if constexpr (GRAPH::HasEdgeAttribute(Distance))
    csv << ",Distance";
  if constexpr (GRAPH::HasEdgeAttribute(ViaVertex))
    csv << ",ViaVertex";
  if constexpr (GRAPH::HasEdgeAttribute(Weight))
    csv << ",Weight";
  if constexpr (GRAPH::HasEdgeAttribute(Capacity))
    csv << ",Capacity";
  if constexpr (GRAPH::HasEdgeAttribute(ReverseEdge))
    csv << ",ReverseEdge";

  csv << "\n";

  for (const auto [edge, from] : graph.edgesWithFromVertex()) {
    csv << size_t(from) << "," << size_t(graph.get(ToVertex, edge));
    if constexpr (GRAPH::HasEdgeAttribute(TravelTime))
      csv << "," << (int)graph.get(TravelTime, edge);
    if constexpr (GRAPH::HasEdgeAttribute(Distance))
      csv << "," << (int)graph.get(Distance, edge);
    if constexpr (GRAPH::HasEdgeAttribute(ViaVertex))
      csv << "," << size_t(graph.get(ViaVertex, edge));
    if constexpr (GRAPH::HasEdgeAttribute(Weight))
      csv << "," << (int)graph.get(Weight, edge);
    if constexpr (GRAPH::HasEdgeAttribute(Capacity))
      csv << "," << (int)graph.get(Capacity, edge);
    if constexpr (GRAPH::HasEdgeAttribute(ReverseEdge))
      csv << "," << size_t(graph.get(ReverseEdge, edge));
    csv << "\n";
  }
  csv.close();
}

template <typename GRAPH>
inline void toGML(const std::string &fileBaseName,
                  const GRAPH &graph) noexcept {
  std::ofstream gml(fileBaseName + ".graphml");
  Assert(gml, "Cannot create output stream for " << fileBaseName << ".graphml");
  Assert(gml.is_open(),
         "Cannot open output stream for " << fileBaseName << ".graphml");
  gml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  gml << "<graphml xmlns=\"http://graphml.graphdrawing.org/xmlns\" "
         "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
         "xsi:schemaLocation=\"http://graphml.graphdrawing.org/xmlns "
         "http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd\">\n";
  if constexpr (GRAPH::HasVertexAttribute(Coordinates)) {
    gml << "        <key id=\"lat_n\" for=\"node\" attr.name=\"latitude\" "
           "attr.type=\"double\"/>\n";
    gml << "        <key id=\"lon_n\" for=\"node\" attr.name=\"longitude\" "
           "attr.type=\"double\"/>\n";
  }
  if constexpr (GRAPH::HasVertexAttribute(Size))
    gml << "        <key id=\"size_n\" for=\"node\" attr.name=\"size\" "
           "attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasVertexAttribute(Weight))
    gml << "        <key id=\"weight_n\" for=\"node\" attr.name=\"weight\" "
           "attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasVertexAttribute(ViaVertex))
    gml << "        <key id=\"viavertex_n\" for=\"node\" "
           "attr.name=\"viavertex\" attr.type=\"int\"/>\n";

  if constexpr (GRAPH::HasEdgeAttribute(TravelTime))
    gml << "        <key id=\"traveltime_e\" for=\"edge\" "
           "attr.name=\"traveltime\" attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(Distance))
    gml << "        <key id=\"distance_e\" for=\"edge\" attr.name=\"distance\" "
           "attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(Size))
    gml << "        <key id=\"size_e\" for=\"edge\" attr.name=\"edgesize\" "
           "attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(Weight))
    gml << "        <key id=\"weight_e\" for=\"edge\" attr.name=\"edgeweight\" "
           "attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(ReverseEdge))
    gml << "        <key id=\"reverseedge_e\" for=\"edge\" "
           "attr.name=\"reverseegde\" attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(Capacity))
    gml << "        <key id=\"capacity_e\" for=\"edge\" attr.name=\"capacity\" "
           "attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(ViaVertex))
    gml << "        <key id=\"viavertex_e\" for=\"edge\" "
           "attr.name=\"viavertex\" attr.type=\"int\"/>\n";
  gml << "    <graph id=\"G\" edgedefault=\"directed\">\n";

  for (const Vertex vertex : graph.vertices()) {
    gml << "        <node id=\"" << size_t(vertex) << "\">\n";

    if constexpr (GRAPH::HasVertexAttribute(Coordinates)) {
      gml << "            <data key=\"lat_n\">"
          << (float)graph.get(Coordinates, vertex).latitude << "</data>\n";
      gml << "            <data key=\"lon_n\">"
          << (float)graph.get(Coordinates, vertex).longitude << "</data>\n";
    }
    if constexpr (GRAPH::HasVertexAttribute(Size))
      gml << "            <data key=\"size_n\">" << (int)graph.get(Size, vertex)
          << "</data>\n";
    if constexpr (GRAPH::HasVertexAttribute(Weight))
      gml << "            <data key=\"weight_n\">"
          << (int)graph.get(Weight, vertex) << "</data>\n";
    if constexpr (GRAPH::HasVertexAttribute(ViaVertex))
      gml << "            <data key=\"viavertex_n\">"
          << (int)graph.get(ViaVertex, vertex) << "</data>\n";

    gml << "        </node>\n";
  }
  for (const auto [edge, from] : graph.edgesWithFromVertex()) {
    gml << "        <edge source=\"" << size_t(from) << "\" target=\""
        << size_t(graph.get(ToVertex, edge)) << "\">\n";
    if constexpr (GRAPH::HasEdgeAttribute(TravelTime))
      gml << "            <data key=\"traveltime_e\">"
          << (int)graph.get(TravelTime, edge) << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(Distance))
      gml << "            <data key=\"distance_e\">"
          << (int)graph.get(Distance, edge) << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(Size))
      gml << "            <data key=\"size_e\">" << (int)graph.get(Size, edge)
          << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(Weight))
      gml << "            <data key=\"weight_e\">"
          << (int)graph.get(Weight, edge) << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(ReverseEdge))
      gml << "            <data key=\"reverseedge_e\">"
          << (int)graph.get(ReverseEdge, edge) << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(Capacity))
      gml << "            <data key=\"capacity_e\">"
          << (int)graph.get(Capacity, edge) << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(ViaVertex))
      gml << "            <data key=\"viavertex_e\">"
          << size_t(graph.get(ViaVertex, edge)) << "</data>\n";
    gml << "        </edge>\n";
  }
  gml << "    </graph>\n";
  gml << "</graphml>" << std::endl;
  gml.close();
}
} // namespace Graph
