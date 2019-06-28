/**********************************************************************************

 Copyright (c) 2019 Jonas Sauer, Tobias Zündorf

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/

#pragma once

#include "Classes/GraphInterface.h"

#include "Utils/Utils.h"

#include "Classes/DynamicGraph.h"
#include "Classes/StaticGraph.h"

using NoVertexAttributes = List<>;
using WithCoordinates = List<Attribute<Coordinates, Geometry::Point>>;

using NoEdgeAttributes = List<>;
using WithTravelTime = List<Attribute<TravelTime, int>>;
using WithViaVertexAndWeight = List<Attribute<ViaVertex, Vertex>, Attribute<Weight, int>>;

using TransferGraph = StaticGraph<WithCoordinates, WithTravelTime>;
using DynamicTransferGraph = DynamicGraph<WithCoordinates, WithTravelTime>;

using CHConstructionGraph = EdgeList<NoVertexAttributes, WithViaVertexAndWeight>;
using CHCoreGraph = DynamicGraph<NoVertexAttributes, WithViaVertexAndWeight>;
using CHGraph = StaticGraph<NoVertexAttributes, WithViaVertexAndWeight>;

using DimacsGraph = EdgeList<NoVertexAttributes, WithTravelTime>;
using DimacsGraphWithCoordinates = EdgeList<WithCoordinates, WithTravelTime>;

using TravelTimeGraph = StaticGraph<NoVertexAttributes, WithTravelTime>;

#include "Utils/Conversion.h"
