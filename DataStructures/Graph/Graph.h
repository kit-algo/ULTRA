#pragma once

#include "Classes/GraphInterface.h"

#include "Utils/Utils.h"

#include "Classes/DynamicGraph.h"
#include "Classes/StaticGraph.h"
#include "Classes/EdgeList.h"

using NoVertexAttributes = List<>;
using WithCoordinates = List<Attribute<Coordinates, Geometry::Point>>;
using WithSize = List<Attribute<Size, size_t>>;

using NoEdgeAttributes = List<>;
using WithTravelTime = List<Attribute<TravelTime, int>>;
using WithTravelTimeAndDistance = List<Attribute<TravelTime, int>, Attribute<Distance, int>>;
using WithTravelTimeAndEdgeFlags = List<Attribute<TravelTime, int>, Attribute<EdgeFlags, std::vector<bool>>>;
using WithTravelTimeAndBundleSize = List<Attribute<TravelTime, int>, Attribute<BundleSize, int>>;
using WithReverseEdges = List<Attribute<ReverseEdge, Edge>>;
using WithCapacity = List<Attribute<Capacity, int>>;
using WithWeight = List<Attribute<Weight, int>>;
using WithViaVertex = List<Attribute<ViaVertex, Vertex>>;
using WithViaVertexAndWeight = List<Attribute<ViaVertex, Vertex>, Attribute<Weight, int>>;
using WithReverseEdgesAndViaVertex = List<Attribute<ReverseEdge, Edge>, Attribute<ViaVertex, Vertex>>;
using WithReverseEdgesAndWeight = List<Attribute<ReverseEdge, Edge>, Attribute<Weight, int>>;
using WithReverseEdgesAndCapacity = List<Attribute<ReverseEdge, Edge>, Attribute<Capacity, int>>;

using StrasserGraph = StaticGraph<NoVertexAttributes, WithTravelTimeAndDistance>;
using StrasserGraphWithCoordinates = StaticGraph<WithCoordinates, WithTravelTimeAndDistance>;

using TransferGraph = StaticGraph<WithCoordinates, WithTravelTime>;
using DynamicTransferGraph = DynamicGraph<WithCoordinates, WithTravelTime>;
using TransferEdgeList = EdgeList<WithCoordinates, WithTravelTime>;
using EdgeFlagsTransferGraph = DynamicGraph<WithCoordinates, WithTravelTimeAndEdgeFlags>;

using SimpleDynamicGraph = DynamicGraph<NoVertexAttributes, NoEdgeAttributes>;
using SimpleStaticGraph = StaticGraph<NoVertexAttributes, NoEdgeAttributes>;
using SimpleEdgeList = EdgeList<NoVertexAttributes, NoEdgeAttributes>;

using DynamicFlowGraph = DynamicGraph<NoVertexAttributes, WithReverseEdgesAndCapacity>;
using StaticFlowGraph = StaticGraph<NoVertexAttributes, WithReverseEdgesAndCapacity>;

using CHConstructionGraph = EdgeList<NoVertexAttributes, WithViaVertexAndWeight>;
using CHCoreGraph = DynamicGraph<NoVertexAttributes, WithViaVertexAndWeight>;
using CHGraph = StaticGraph<NoVertexAttributes, WithViaVertexAndWeight>;

using DimacsGraph = EdgeList<NoVertexAttributes, WithTravelTime>;
using DimacsGraphWithCoordinates = EdgeList<WithCoordinates, WithTravelTime>;

using TravelTimeGraph = StaticGraph<NoVertexAttributes, WithTravelTime>;

using CondensationGraph = DynamicGraph<WithSize, WithTravelTime>;

using BundledGraph = StaticGraph<WithCoordinates, WithTravelTimeAndBundleSize>;
using DynamicBundledGraph = DynamicGraph<WithCoordinates, WithTravelTimeAndBundleSize>;

#include "Utils/Conversion.h"
#include "Utils/IO.h"
