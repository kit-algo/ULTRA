#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "Data.h"
#include "DelayData.h"
#include "../RAPTOR/Data.h"
#include "../../Algorithms/RAPTOR/InitialTransfers.h"
#include "../../Helpers/Ranges/SubRange.h"
#include "../../Helpers/Vector/Permutation.h"

namespace TripBased {

struct RouteInfo {
    RouteInfo(const RouteId route, const SubRange<std::vector<StopId>>& stops) :
        route(route) {
        for (const StopId stop : stops) {
            stopSequence.emplace_back(stop);
        }
    }

    RouteId route;
    std::vector<StopId> stopSequence;

    inline bool operator<(const RouteInfo& other) const noexcept {
        return std::lexicographical_compare(stopSequence.begin(), stopSequence.end(), other.stopSequence.begin(), other.stopSequence.end());
    }
};

struct RouteGrouping {
    RouteGrouping() {}

    RouteGrouping(const RAPTOR::Data& data) :
        sequenceOfRoute(data.numberOfRoutes(), -1) {
        std::vector<RouteInfo> routeInfo;
        for (const RouteId route : data.routes()) {
            routeInfo.emplace_back(route, data.stopsOfRoute(route));
        }
        std::sort(routeInfo.begin(), routeInfo.end());
        sequenceOfRoute[routeInfo[0].route] = 0;
        routesWithSequence.emplace_back();
        routesWithSequence.back().emplace_back(routeInfo[0].route);

        for (size_t i = 1; i < routeInfo.size(); i++) {
            if (routeInfo[i].stopSequence != routeInfo[routesWithSequence.back()[0]].stopSequence) {
                routesWithSequence.emplace_back();
            }
            sequenceOfRoute[routeInfo[i].route] = routesWithSequence.size() - 1;
            routesWithSequence.back().emplace_back(routeInfo[i].route);
        }
    }

    inline const std::vector<RouteId>& getRoutesWithSameSequence(const RouteId route) const noexcept {
        return routesWithSequence[sequenceOfRoute[route]];
    }

    std::vector<size_t> sequenceOfRoute;
    std::vector<std::vector<RouteId>> routesWithSequence;
};

struct DelayUpdateData {
    DelayUpdateData(const RAPTOR::BucketCHInitialTransfers& initialTransfers) :
        initialTransfers(initialTransfers) {
    }

    inline void update(const RAPTOR::Data&& newRaptorData, const std::pair<TransferGraph, TransferGraph>&& filteredShortcuts) noexcept {
        raptorData = std::move(newRaptorData);
        const Order stopEventOrder = raptorData.rebuildRoutes();
        internalToOriginal = Permutation(stopEventOrder);
        originalToInternal = Permutation(Construct::Invert, stopEventOrder);
        reverseRaptorData = raptorData.reverseNetwork();
        tripData = Data(raptorData);
        routeGrouping = RouteGrouping(raptorData);
        std::tie(tripData.stopEventGraph, infeasibleShortcuts) = std::move(filteredShortcuts);
        tripData.stopEventGraph.applyVertexPermutation(originalToInternal);
        tripData.stopEventGraph.sortEdges(ToVertex);
        infeasibleShortcuts.applyVertexPermutation(originalToInternal);
        infeasibleShortcuts.sortEdges(ToVertex);
        reverseStopEventGraph = tripData.stopEventGraph;
        reverseStopEventGraph.revert();
    }

    const RAPTOR::BucketCHInitialTransfers initialTransfers;
    Permutation internalToOriginal;
    Permutation originalToInternal;
    Data tripData;
    RAPTOR::Data raptorData;
    RAPTOR::Data reverseRaptorData;
    TransferGraph reverseStopEventGraph;
    TransferGraph infeasibleShortcuts;
    RouteGrouping routeGrouping;
};

struct DelayQueryData {
    inline void update(RAPTOR::Data&& newRaptorData, const std::pair<TransferGraph, TransferGraph>&& filteredShortcuts) noexcept {
        const Order stopEventOrder = newRaptorData.rebuildRoutes();
        internalToOriginal = Permutation(stopEventOrder);
        const Permutation originalToInternal(Construct::Invert, stopEventOrder);
        tripData = Data(newRaptorData);
        tripData.stopEventGraph = filteredShortcuts.first;
        tripData.stopEventGraph.applyVertexPermutation(originalToInternal);
    }

    Data tripData;
    Permutation internalToOriginal;
};

}
