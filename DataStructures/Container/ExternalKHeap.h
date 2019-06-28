/**********************************************************************************

 Copyright (c) 2019 Thomas Pajor, Tobias Zündorf

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

#include <vector>
#include <type_traits>

#include "../../Helpers/Helpers.h"
#include "../../Helpers/Assert.h"

class ExternalKHeapElement {

public:
    inline int getHeapPosition() const noexcept {return heapPosition;}
    inline void setHeapPosition(const int p) noexcept {heapPosition = p;}
    inline bool isOnHeap() const noexcept {return heapPosition != -1;}
    inline bool hasSmallerKey(const ExternalKHeapElement* const) const noexcept {
        AssertMsg(false, "ExternalKHeapElement keys are compared using a default implementation of hasSmallerKey (This method should be overwritten in a derived class)!");
        return true;
    }
    ExternalKHeapElement() : heapPosition(-1) {}

private:
    int heapPosition;

};


template<int logK, typename elementType>
class ExternalKHeap {

public:
    using ElementType = elementType;
    static constexpr int K = 1 << logK;

public:
    ExternalKHeap(const int initialNumberOfElements = 1000) : heap(0) {
        static_assert(std::is_base_of<ExternalKHeapElement, ElementType>::value, "Element type must inherit from ExternalKHeapElement");
        heap.reserve(initialNumberOfElements);
        initialize();
    }

    inline int size() const {return numberOfElements;}
    inline bool empty() const {return size() == 0;}

    inline ElementType* extractFront() {
        AssertMsg(!empty(), "Heap is empty!");
        ElementType* front = heap[0];
        front->setHeapPosition(-1);
        numberOfElements--;
        if (!empty()) {
            heap[0] = heap[numberOfElements];
            heap[0]->setHeapPosition(0);
            siftDown(0);
        }
        heap.resize(numberOfElements);
        return front;
    }
    inline ElementType* pop() {return extractFront();}

    inline void update(ElementType* const element) {
        AssertMsg(heap.size() == static_cast<size_t>(numberOfElements), "Heap with " << numberOfElements << " elements has a size is " << heap.size() << "!");
        if (element->getHeapPosition() == -1) {
            heap.emplace_back(element);
            element->setHeapPosition(numberOfElements);
            siftUp(numberOfElements++);
        } else if (element->getHeapPosition() == 0) {
            siftDown(0);
        } else {
            int parentIndex = parent(element->getHeapPosition());
            if (element->hasSmallerKey(heap[parentIndex])) {
                siftUp(element->getHeapPosition());
            } else {
                siftDown(element->getHeapPosition());
            }
        }
    }
    inline void update(ElementType& element) {update(&element);}
    inline void push(ElementType* const element) {update(element);}
    inline void push(ElementType& element) {update(&element);}

    inline void remove(ElementType* const element) {
        AssertMsg(element->getHeapPosition() != -1, "Element is not in heap!");
        AssertMsg(heap[element->getHeapPosition()] == element, "Heap is broken!");
        siftDownHole(element->getHeapPosition());
    }

    inline ElementType* front() const {
        AssertMsg(!empty(), "An empty heap has no front!");
        return heap[0];
    }

    inline ElementType& min() const {
        return *front();
    }

    inline void reserve(int size) {
        heap.reserve(size);
    }

    inline void reset() {
        initialize();
    }

    inline void clear() {
        for (int i = 0; i < numberOfElements; ++i) {
            heap[i]->setHeapPosition(-1);
        }
        numberOfElements = 0;
        heap.resize(0);
    }

    inline bool contains(const ElementType* const element) const {
        return element->getHeapPosition() != -1;
    }

    template<typename Range>
    inline void build(Range& range) {
        clear();
        for (ElementType& element : range) {
            element.setHeapPosition(heap.size());
            heap.push_back(&element);
        }
        numberOfElements = heap.size();
        for (int i = parent(size() - 1); i >= 0; i--) {
            siftDown(i);
        }
    }

    inline const std::vector<ElementType*>& data() const noexcept {
        return heap;
    }

protected:
    inline void initialize() {
        numberOfElements = 0;
    }

    inline int parent(const int i) const {return (i - 1) >> logK;}
    inline int fistChild(const int i) const {return (i << logK) + 1;}

    inline void siftUp(const int i) {
        AssertMsg(i < numberOfElements, "Index i is too large!");
        int currentIndex = i;
        while (currentIndex > 0) {
            int parentIndex = parent(currentIndex);
            if (heap[currentIndex]->hasSmallerKey(heap[parentIndex])) {
                swap(currentIndex, parentIndex);
            } else {
                break;
            }
            currentIndex = parentIndex;
        }
    }

    inline void siftDown(int i) {
        AssertMsg(i < numberOfElements, "Index i is too large!");
        while (true) {
            int minIndex = i;
            ElementType* minElement = heap[i];
            int childIndexLower = fistChild(i);
            int childIndexUpper = std::min(childIndexLower + K, numberOfElements);
            for (int j = childIndexLower; j < childIndexUpper; ++j) {
                if (heap[j]->hasSmallerKey(minElement)) {
                    minIndex = j;
                    minElement = heap[j];
                }
            }
            if (minIndex != i) {
                swap(i, minIndex);
                i = minIndex;
            } else {
                break;
            }
        }
    }

    inline void siftDownHole(int i) {
        AssertMsg(i < numberOfElements, "Index i is too large!");
        heap[i]->setHeapPosition(-1);
        while (true) {
            int childIndexLower = fistChild(i);
            int childIndexUpper = std::min(childIndexLower + K, numberOfElements);
            if (childIndexLower >= childIndexUpper) {
                numberOfElements--;
                if (i >= size()) {
                    heap.pop_back();
                } else {
                    heap[i] = heap.back();
                    heap[i]->setHeapPosition(i);
                    heap.pop_back();
                    siftUp(i);
                }
                return;
            } else {
                int minIndex = childIndexLower;
                ElementType* minElement = heap[childIndexLower];
                for (int j = childIndexLower + 1; j < childIndexUpper; ++j) {
                    if (heap[j]->hasSmallerKey(minElement)) {
                        minIndex = j;
                        minElement = heap[j];
                    }
                }
                heap[i] = minElement;
                heap[i]->setHeapPosition(i);
                i = minIndex;
            }
        }
    }

    inline void swap(const int i, const int j) {
        heap[i]->setHeapPosition(j);
        heap[j]->setHeapPosition(i);
        ElementType* temp = heap[i];
        heap[i] = heap[j];
        heap[j] = temp;
    }

private:
    int numberOfElements;
    std::vector<ElementType*> heap;

};
