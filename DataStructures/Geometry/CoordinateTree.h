#pragma once

#include <cmath>
#include <vector>
#include <sstream>

#include "Point.h"
#include "Rectangle.h"
#include "Metric.h"

#include "../../Helpers/Types.h"
#include "../../Helpers/Assert.h"
#include "../../Helpers/Vector/Vector.h"
#include "../../Helpers/Vector/Permutation.h"

template<typename METRIC = Geometry::EuclideanMetric>
class CoordinateTree {

private:
    // Data for leaf nodes: Interval of points represented by the leaf
    struct LeafNodeData {
        int begin;
        int end;
    };

    // Data for inner nodes: Index of split dimension and max (respectively min) value for the splitDimension of the child nodes
    struct InnerNodeDate {
        int splitDimension;
        double minChildMax;
        double maxChildMin;
    };

    // Node label: Contains child Node indices and inner/leaf node data
    struct Node {
        inline bool isLeaf() const {return (minChild < 0) && (maxChild < 0);}
        inline void makeLeaf() {minChild = -1; maxChild = -1;}
        int minChild;
        int maxChild;
        union {
            LeafNodeData leaf;
            InnerNodeDate inner;
        };
    };

public:
    CoordinateTree(const METRIC& metric, const std::vector<Geometry::Point>& coordinates, const int maxLeafSize = 10) :
        metric(metric),
        coordinates(coordinates),
        vertexCount(coordinates.size()),
        maxLeafSize(maxLeafSize),
        order(Vector::id<Vertex>(numVertices())),
        nearestVertex(noVertex),
        nearestDistance(-1),
        minimumDistance(-1) {
        Assert(!coordinates.empty());
        nodes.reserve((coordinates.size() / maxLeafSize) * 4);
        makeBoundingBox(0, numVertices(), boundingBox);
        divideTree(newNode(), 0, numVertices(), boundingBox);
    }
    CoordinateTree(const std::vector<Geometry::Point>& coordinates, const int maxLeafSize = 10) :
        coordinates(coordinates),
        vertexCount(coordinates.size()),
        maxLeafSize(maxLeafSize),
        order(Vector::id<Vertex>(numVertices())),
        nearestVertex(noVertex),
        nearestDistance(-1),
        minimumDistance(-1) {
        Assert(!coordinates.empty());
        nodes.reserve((coordinates.size() / maxLeafSize) * 4);
        makeBoundingBox(0, numVertices(), boundingBox);
        divideTree(newNode(), 0, numVertices(), boundingBox);
    }

    inline Vertex getNearestNeighbor(const Geometry::Point& p) const noexcept {
        nearestVertex = Vertex(0);
        nearestDistance = metric.distanceSquare(p, coordinates[nearestVertex]);
        this->p = p;
        searchLevel<false>(0, boundingBox.closestPoint(p));
        return nearestVertex;
    }

    inline Vertex getNearestNeighbor(const Geometry::Point& p, const double minDistance) const noexcept {
        nearestVertex = noVertex;
        nearestDistance = metric.distanceSquare(boundingBox.min, boundingBox.max) + 1;
        this->p = p;
        this->minimumDistance = minDistance * minDistance;
        searchLevel<true>(0, boundingBox.closestPoint(p));
        return nearestVertex;
    }

    inline std::vector<Vertex> getNeighbors(const Geometry::Point& p, const double maxDistance) const noexcept {
        neighbors.clear();
        nearestDistance = maxDistance * maxDistance;
        this->p = p;
        searchNeighbors(0, boundingBox.closestPoint(p));
        return neighbors;
    }

    inline size_t numVertices() const noexcept {
        return vertexCount;
    }

    inline const Geometry::Point& getCoordinates(const Vertex vertex) const noexcept {
        return coordinates[vertex];
    }

    inline void clear() noexcept {
        order.clear();
        nodes.clear();
    }

    inline const std::vector<Vertex>& getOrder() const noexcept {
        return order;
    }

    inline size_t numNodes() const noexcept {
        return nodes.size();
    }

    inline long long byteSize() const noexcept {
        return sizeof(*this) + Vector::byteSize(order) + Vector::byteSize(nodes);
    }

    inline void check() const noexcept {
        std::vector<bool> seen(numVertices());
        check(0, boundingBox, seen);
        for (int i = 0; i < seen.size(); i++) {
            Assert(seen[i]);
        }
    }

private:
    void divideTree(const int nodeIndex, const int beginIndex, const int endIndex, const Geometry::Rectangle& boundingBox) noexcept {
        AssertMsg(isBoundingBox(beginIndex, endIndex, boundingBox), "Invalid bounding box!");
        AssertMsg(endIndex > beginIndex, "endIndex = " << endIndex << " <= beginIndex = " << beginIndex << "!");
        AssertMsg(isNode(nodeIndex), "Index = " << nodeIndex << " is not a node!");

        Node& node = nodes[nodeIndex];
        if ((endIndex - beginIndex) <= maxLeafSize || boundingBox.isPoint()) {
            node.makeLeaf();
            node.leaf.begin = beginIndex;
            node.leaf.end = endIndex;
        } else {
            if (metric.spread(boundingBox, 0) > metric.spread(boundingBox, 1)) {
                node.inner.splitDimension = 0;
            } else {
                node.inner.splitDimension = 1;
            }
            double splitValue = boundingBox.center()[node.inner.splitDimension];
            Geometry::Rectangle minBoundingBox(Geometry::Rectangle::Negative());
            Geometry::Rectangle maxBoundingBox(Geometry::Rectangle::Negative());
            int i = beginIndex;
            int j = endIndex - 1;
            while (i <= j) {
                while (i <= j && coordinates[order[i]][node.inner.splitDimension] < splitValue) {
                    minBoundingBox.extend(coordinates[order[i++]]);
                }
                while (i <= j && coordinates[order[j]][node.inner.splitDimension] >= splitValue) {
                    maxBoundingBox.extend(coordinates[order[j--]]);
                }
                if (i >= j) break;
                std::swap(order[i], order[j]);
                minBoundingBox.extend(coordinates[order[i++]]);
                maxBoundingBox.extend(coordinates[order[j--]]);
            }
            Assert(i - 1 == j);
            Assert(i > beginIndex);
            Assert(i < endIndex);
            node.inner.minChildMax = minBoundingBox.max[node.inner.splitDimension];
            node.inner.maxChildMin = maxBoundingBox.min[node.inner.splitDimension];
            const int minChild = newNode();
            divideTree(minChild, beginIndex, i, minBoundingBox);
            const int maxChild = newNode();
            divideTree(maxChild, i, endIndex, maxBoundingBox);
            nodes[nodeIndex].minChild = minChild;
            nodes[nodeIndex].maxChild = maxChild;
        }
    }

    template<bool MIN_DISTANCE = false>
    void searchLevel(const int nodeIndex, const Geometry::Point& closest) const noexcept {
        AssertMsg(isNode(nodeIndex), "Index = " << nodeIndex << " is not a node!");

        const Node& node = nodes[nodeIndex];
        if (node.isLeaf()) {
            for (int i = node.leaf.begin; i < node.leaf.end; i++) {
                const double distance = metric.distanceSquare(p, coordinates[order[i]]);
                if (nearestDistance <= distance) continue;
                if constexpr (MIN_DISTANCE) if (distance <= minimumDistance) continue;
                nearestDistance = distance;
                nearestVertex = order[i];
            }
        } else {
            Geometry::Point minClosest = closest;
            minClosest[node.inner.splitDimension] = std::min(closest[node.inner.splitDimension], node.inner.minChildMax);
            const double minChildDistance = metric.distanceSquare(p, minClosest);
            Geometry::Point maxClosest = closest;
            maxClosest[node.inner.splitDimension] = std::max(closest[node.inner.splitDimension], node.inner.maxChildMin);
            const double maxChildDistance = metric.distanceSquare(p, maxClosest);
            if (minChildDistance < maxChildDistance) {
                if (nearestDistance <= minChildDistance) return;
                searchLevel<MIN_DISTANCE>(node.minChild, minClosest);
                if (nearestDistance <= maxChildDistance) return;
                searchLevel<MIN_DISTANCE>(node.maxChild, maxClosest);
            } else {
                if (nearestDistance <= maxChildDistance) return;
                searchLevel<MIN_DISTANCE>(node.maxChild, maxClosest);
                if (nearestDistance <= minChildDistance) return;
                searchLevel<MIN_DISTANCE>(node.minChild, minClosest);
            }
        }
    }

    void searchNeighbors(const int nodeIndex, const Geometry::Point& closest) const noexcept {
        AssertMsg(isNode(nodeIndex), "Index = " << nodeIndex << " is not a node!");

        const Node& node = nodes[nodeIndex];
        if (node.isLeaf()) {
            for (int i = node.leaf.begin; i < node.leaf.end; i++) {
                const double distance = metric.distanceSquare(p, coordinates[order[i]]);
                if (nearestDistance >= distance) neighbors.emplace_back(order[i]);
            }
        } else {
            Geometry::Point minClosest = closest;
            minClosest[node.inner.splitDimension] = std::min(closest[node.inner.splitDimension], node.inner.minChildMax);
            const double minChildDistance = metric.distanceSquare(p, minClosest);
            if (nearestDistance >= minChildDistance) searchNeighbors(node.minChild, minClosest);
            Geometry::Point maxClosest = closest;
            maxClosest[node.inner.splitDimension] = std::max(closest[node.inner.splitDimension], node.inner.maxChildMin);
            const double maxChildDistance = metric.distanceSquare(p, maxClosest);
            if (nearestDistance >= maxChildDistance) searchNeighbors(node.maxChild, maxClosest);
        }
    }

    inline int newNode() noexcept {
        nodes.push_back(Node());
        return nodes.size() - 1;
    }

    inline bool isNode(const size_t index) const noexcept {
        return index < nodes.size();
    }

    inline void makeBoundingBox(const int beginIndex, const int endIndex, Geometry::Rectangle& boundingBox) noexcept {
        Assert(endIndex > beginIndex);
        boundingBox.clear(coordinates[order[beginIndex]]);
        for (int i = beginIndex + 1; i < endIndex; i++) {
            boundingBox.extend(coordinates[order[i]]);
        }
    }

    inline bool isBoundingBox(const int beginIndex, const int endIndex, const Geometry::Rectangle& boundingBox) noexcept {
        Geometry::Rectangle realBoundingBox;
        makeBoundingBox(beginIndex, endIndex, realBoundingBox);
        return (realBoundingBox == boundingBox);
    }

    void check(const int nodeIndex, const Geometry::Rectangle& bb, std::vector<bool>& seen) const noexcept {
        AssertMsg(isNode(nodeIndex), "Index = " << nodeIndex << " is not a node!");
        const Node& node = nodes[nodeIndex];
        if (node.isLeaf()) {
            for (int i = node.leaf.begin; i < node.leaf.end; i++) {
                if (!bb.contains(coordinates[order[i]])) {
                    std::cout << "nodeIndex: " << nodeIndex << std::endl << std::flush;
                    std::cout << "bb: " << bb << std::endl << std::flush;
                    std::cout << "i: " << i << std::endl << std::flush;
                    std::cout << "order[i]: " << order[i] << std::endl << std::flush;
                    std::cout << "coordinates[order[i]]: " << coordinates[order[i]] << std::endl << std::flush;
                }
                Assert(bb.contains(coordinates[order[i]]));
                Assert(!seen[order[i]]);
                seen[order[i]] = true;
            }
        } else {
            Geometry::Rectangle minBB = bb;
            minBB.max[node.inner.splitDimension] = node.inner.minChildMax;
            if (!bb.contains(minBB)) {
                std::cout << "nodeIndex: " << nodeIndex << std::endl << std::flush;
                std::cout << "bb: " << bb << std::endl << std::flush;
                std::cout << "minBB: " << minBB << std::endl << std::flush;
                std::cout << "node.splitDimension: " << node.inner.splitDimension << std::endl << std::flush;
                std::cout << "node.minChildMax: " << node.inner.minChildMax << std::endl << std::flush;
                std::cout << "node.maxChildMin: " << node.inner.maxChildMin << std::endl << std::flush;
            }
            Assert(bb.contains(minBB));
            check(node.minChild, minBB, seen);
            Geometry::Rectangle maxBB = bb;
            maxBB.min[node.inner.splitDimension] = node.inner.maxChildMin;
            Assert(bb.contains(maxBB));
            check(node.maxChild, maxBB, seen);
        }
    }

private:
    METRIC metric;

    const std::vector<Geometry::Point>& coordinates;
    const int vertexCount;
    const int maxLeafSize;

    Geometry::Rectangle boundingBox;
    std::vector<Vertex> order;
    std::vector<Node> nodes; // nodes[0] = root of tree

    mutable Vertex nearestVertex;
    mutable double nearestDistance;
    mutable double minimumDistance;
    mutable Geometry::Point p;
    mutable std::vector<Vertex> neighbors;

};
