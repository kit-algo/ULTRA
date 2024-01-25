[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# ULTRA: UnLimited TRAnsfers for Multimodal Route Planning

ULTRA is a C++ framework for efficient journey planning in multimodal networks consisting of public transit and non-schedule-based transfer modes (e.g., walking, cycling, e-scooter). It was developed at [KIT](https://www.kit.edu) in the [group of Prof. Dorothea Wagner](https://i11www.iti.kit.edu/). This repository contains code for the following publications: 

* *UnLimited TRAnsfers for Multi-Modal Route Planning: An Efficient Solution*
  Moritz Baum, Valentin Buchhold, Jonas Sauer, Dorothea Wagner, Tobias Zündorf
  In: Proceedings of the 27th Annual European Symposium on Algorithms (ESA'19), Leibniz International Proceedings in Informatics, pages 14:1–14:16, 2019
  [pdf](https://drops.dagstuhl.de/opus/volltexte/2019/11135/pdf/LIPIcs-ESA-2019-14.pdf) [arXiv](https://arxiv.org/abs/1906.04832)

* *Integrating ULTRA and Trip-Based Routing*
  Jonas Sauer, Dorothea Wagner, Tobias Zündorf
  In: Proceedings of the 20th Symposium on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems (ATMOS'20), OpenAccess Series in Informatics, pages 4:1–4:15, 2020
  [pdf](http://i11www.iti.kit.edu/extra/publications/swz-iultr-20.pdf)

* *An Efficient Solution for One-to-Many Multi-Modal Journey Planning*
  Jonas Sauer, Dorothea Wagner, Tobias Zündorf
  In: Proceedings of the 20th Symposium on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems (ATMOS'20), OpenAccess Series in Informatics, pages 1:1–1:15, 2020
  [pdf](https://i11www.iti.kit.edu/extra/publications/swz-aesom-20.pdf)

* *Fast Multimodal Journey Planning for Three Criteria*
  Moritz Potthoff, Jonas Sauer
  In: Proceedings of the 24th Workshop on Algorithm Engineering and Experiments (ALENEX'22), SIAM, pages 145–157, 2022
  [pdf](https://epubs.siam.org/doi/epdf/10.1137/1.9781611977042.12) [arXiv](https://arxiv.org/abs/2110.12954)

* *Efficient Algorithms for Fully Multimodal Journey Planning*
  Moritz Potthoff, Jonas Sauer
  In: Proceedings of the 22nd Symposium on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems (ATMOS'22), OpenAccess Series in Informatics, pages 14:1–14:15, 2022
  [pdf](https://drops.dagstuhl.de/opus/volltexte/2022/17118/pdf/OASIcs-ATMOS-2022-14.pdf)
  
* *Fast and Delay-Robust Multimodal Journey Planning*
  Dominik Bez, Jonas Sauer
  Accepted for publication at the 26th Workshop on Algorithm Engineering and Experiments (ALENEX'24)


# Useful Links to get Familirise with the ULTRA

* Explore the research work comprehensively through our dedicated YouTube playlist. This curated collection provides in-depth insights into the ULTRA project, covering its current status, future prospects, tool usage tutorials, and a holistic understanding of the research. For a detailed walkthrough and to become familiar with our work, refer to the [ULTRA Research Project Playlist](https://www.youtube.com/watch?v%253DLmKhB_EpwpI%2526list%253DPLKylawUbe40d6tInpese9KRFIDEMDcvFM%2526pp%253DiAQB). It's a valuable resource for open-source enthusiasts looking to contribute and engage with this research.🎥🔍

  
- **Video 1: [Public Transport Planning: Introduction](https://youtu.be/LmKhB_EpwpI?feature=shared)**
  - What makes a good public transport supply?
  - Exploring Pareto optimal solutions
  - Characteristics of public transport
  - Useful terminology

- **Video 2: [Public Transport Planning: Data Models](https://youtu.be/vfPZBLH9XCo?feature=shared)**
  - Public transport supply and demand
  - Building graphs for network models
  - Elements: links, stops, lines, timetables, blocks
  - Introduction to Event Activity Network

- **Video 3: [Travel Demand Modelling](https://youtu.be/6QT8M8YWVas?feature=shared)**
  - Defining travel demand
  - Demand Modelling
  - The 4-stage algorithm
  - Insights into trip balancing

- **Video 4: [Public Transport Planning: Approach in Planning practice](https://youtu.be/tgxxUlIlyac?feature=shared)**
  - Travel Demand Model demonstration
  - Overview of planning approaches
  - Line planning and timetabling in practice
  - Operating costs and Express systems

- **Video 5: [Modeling with Integer Variables](https://youtu.be/aZY41pkgthA?feature=shared)**
  - Overview of modeling problems
  - How to Model what we Understood
  - Practical insights into modeling
  - Shortest path problems
  - Flow conservation
  - Some Practical Remarks

- **Video 6: [Introduction to LinTim](https://youtu.be/bgpx1nmgff0?feature=shared)**
  - Unveiling LinTim: What is LinTim and its core functionalities
  - Benefits of LinTim Integration for public transport planning
  - Familiarizing with LinTim Software Toolbox
  - Hands-on experience with LinTim: Practical insights into tool usage

- **Video 7: [LinTim Line Planning](https://youtu.be/OJzMy7vNYR0?feature=shared)**
  - Understanding the line concept
  - What Should we determine while Planning?
  - When is a Solution feasible?
  - Cost Oriented Model – LP1 
  - Extended Cost Model – LP2 
  - Direct Travelers Model – LP3 
  - Travel Time Model – LP4 

- **Video 8: [LinTim Timetabling](https://youtu.be/jjQbSiPKcaw?feature=shared)**
  - Modeling timetables in public transportation
  - Introduction to Periodic Event Scheduling Problem (PESP)
  - Properties of PESP
  - Algorithm for timetabling with practical demonstrations

- **Video 9: [LinTim Vehicle Scheduling](https://youtu.be/4zjA9HOpWbI?feature=shared)**
  - Compatibility for trips 
  - Vehicle Scheduling in public transportation
  - Modeling vehicle scheduling as a flow problem
  - Delay management, stop location, and tariff planning
  - Scheduling Problems as integer problems 

- **Video 10: [Route Planning in Transport Network](https://youtu.be/m9l0cra-5gY?feature=shared)**
  - Basic/speed-up techniques in route planning
  - Algorithms: Dijkstra's Algorithm, Gole Directed Search
  - Multi-Level Dijkstra’s Algorithm
  - Extreme Search Space Reduction 
  - Notion Of Nested Dissection

- **Video 11: [Journey Planning in Public Transit Networks](https://youtu.be/AdArDN4E6Hg?feature=shared)**
  - Terminology
  - How to Model the Timetable (2 Approaches)
  - Time-expanded and Time-dependent approaches for Graph Modelling
  - Query types and handling
  - Working of Different variants of Dijkstra’s Algorithm 
  - Connection Scan Algorithm and Pareto Set
  - RAPTOR - Road Based Public Transit Optimizes Router
  - Route Scanning
  - McRAPTOR (More Citeria RAPTOR)
  - rRAPTOR
  - Comparison between Algorithms Performance


- **Video 12: [Multimodal Journey Planning](https://youtu.be/OsdFfOQGd6k?feature=shared)**
  - Exploring multimodal routing in Practice
  - User-Constrained CH (UCCH) Algorithm qqand Multimodal Multicriteria RAPTOR (MCR Algorithm)
  - Fuzzy Dominance and Heuristic solutions
  - Experiments: London Example
  - Unlimited Walking Effects
  - Experiments: Switzerland Network
  - Problem Setting
  - Goal and Approaches
  - Experiments: Switzerland Network (Preprocessing)
  - Comparison between Algorithms Performance
 
## Usage
Most preprocessing steps and query algorithms are provided in the console application ``ULTRA``. You can compile it with the ``Makefile`` in the ``Runnables`` folder. Type ``make ULTRARelease -B`` to compile in release mode. The following commands are available:

* Contraction Hierarchies (CH) computation:
    - ``buildCH`` performs a regular CH precomputation. The output is used by the (Mc)ULTRA query algorithms for the Bucket-CH searches.
    - ``buildCoreCH`` performs a Core-CH precomputation. The output is used by the (Mc)ULTRA shortcut computation and by the MCSA and M(C)R query algorithms.
* (Mc)ULTRA shortcut computation:
    - ``computeStopToStopShortcuts`` computes stop-to-stop ULTRA shortcuts for use with ULTRA-CSA and ULTRA-RAPTOR.
    - ``computeEventToEventShortcuts`` computes event-to-event ULTRA shortcuts for use with ULTRA-TB.
	- ``computeDelayEventToEventShortcuts`` computes delay-tolerate event-to-event ULTRA shortcuts.
    - ``computeMcStopToStopShortcuts`` computes stop-to-stop McULTRA shortcuts for use with ULTRA-McRAPTOR and UBM-RAPTOR.
    - ``computeMcEventToEventShortcuts`` computes event-to-event McULTRA shortcuts for use with ULTRA-McTB and UBM-TB.
    - ``augmentTripBasedShortcuts`` performs the shortcut augmentation step that is required for UBM-TB.
    - ``validateStopToStopShortcuts`` and ``validateEventToEventShortcuts`` test the validity of the computed shortcuts by comparing them to paths in the original transfer graph.
* Original TB transfer generation:
    - ``raptorToTripBased`` takes a network in RAPTOR format as input and runs the TB transfer generation.
    - With a transitively closed transfer graph as input, this performs the original TB preprocessing.
    - With stop-to-stop ULTRA shortcuts as input, this performs the sequential ULTRA-TB preprocessing.
    - The parameter "Route-based pruning?" enables the optimized preprocessing proposed by Lehoux and Loiodice.
* Query algorithms:

| Command                                 | Algorithm       | Transfers  | Query type       | Criteria                                                  |
|-----------------------------------------|-----------------|------------|------------------|-----------------------------------------------------------|
| ``runTransitiveCSAQueries``             | CSA             | Transitive | Stop-to-stop     | Arrival time                                              |
| ``runDijkstraCSAQueries``               | MCSA            | Unlimited  | Vertex-to-vertex | Arrival time                                              |
| ``runHLCSAQueries``                     | HL-CSA          | Unlimited  | Vertex-to-vertex | Arrival time                                              |
| ``runULTRACSAQueries``                  | ULTRA-CSA       | Unlimited  | Vertex-to-vertex | Arrival time                                              |
| ``runTransitiveRAPTORQueries``          | RAPTOR          | Transitive | Stop-to-stop     | Arrival time, number of trips                             |
| ``runDijkstraRAPTORQueries``            | MR              | Unlimited  | Vertex-to-vertex | Arrival time, number of trips                             |
| ``runHLRAPTORQueries``                  | HL-RAPTOR       | Unlimited  | Vertex-to-vertex | Arrival time, number of trips                             |
| ``runULTRARAPTORQueries``               | ULTRA-RAPTOR    | Unlimited  | Vertex-to-vertex | Arrival time, number of trips                             |
| ``runTransitiveTBQueries``              | TB              | Transitive | Stop-to-stop     | Arrival time, number of trips                             |
| ``runULTRATBQueries``                   | ULTRA-TB        | Unlimited  | Vertex-to-vertex | Arrival time, number of trips                             |
| ``runTransitiveMcRAPTORQueries``        | McRAPTOR        | Transitive | Stop-to-stop     | Arrival time, number of trips, transfer time (full)       |
| ``runMCRQueries``                       | MCR             | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (full)       |
| ``runULTRAMcRAPTORQueries``             | ULTRA-McRAPTOR  | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (full)       |
| ``runULTRAMcTBQueries``                 | ULTRA-McTB      | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (full)       |
| ``runTransitiveBoundedMcRAPTORQueries`` | BM-RAPTOR       | Transitive | Stop-to-stop     | Arrival time, number of trips, transfer time (restricted) |
| ``runUBMRAPTORQueries``                 | UBM-RAPTOR      | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (restricted) |
| ``runUBMTBQueries``                     | UBM-TB          | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (restricted) |
| ``runUBMHydRAQueries``                  | UBM-HydRA       | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (restricted) |

## Networks
We use custom data formats for loading the public transit network and the transfer graph: The Intermediate format allows for easy network manipulation, while the RAPTOR format is required by the preprocessing and all query algorithms except for CSA, which uses its own format. The Switzerland and London networks used in our experiments are available at [https://i11www.iti.kit.edu/PublicTransitData/ULTRA/](https://i11www.iti.kit.edu/PublicTransitData/ULTRA/) in the required formats. Unfortunately, we cannot provide the Germany and Stuttgart networks because they are proprietary.

The ``Network`` application provides commands for manipulating the network data and for converting public transit data to our custom format. It includes the following commands:
* ``parseGTFS`` converts GFTS data in CSV format to a binary format.
* ``gtfsToIntermediate`` converts GFTS binary data to the Intermediate network format.
* ``intermediateToCSA`` converts a network in Intermediate format to CSA format.
* ``intermediateToRAPTOR`` converts a network in Intermediate format to RAPTOR format.
* ``loadDimacsGraph`` converts a graph in the format used by the [9th DIMACS Implementation Challenge](http://diag.uniroma1.it/challenge9/download.shtml) to our custom binary graph format.
* ``duplicateTrips`` duplicates all trips in the network and shifts them by a specified time offset. This is used to extend networks that only comprise a single day to two days, in order to allow for overnight journeys.
* ``addGraph`` adds a transfer graph to a network in Intermediate format. Existing transfer edges in the network are preserved.
* ``replaceGraph`` replaces the transfer graph of a network with a specified transfer graph.
* ``reduceGraph`` contracts all vertices with degree less than 3 in the transfer graph.
* ``reduceToMaximumConnectedComponent`` reduces a network to its largest connected component.
* ``applyBoundingBox`` removes all parts of a network that lie outside a predefined bounding box.
* ``applyCustomBoundingBox`` removes all parts of a network that lie outside a specified bounding box.
* ``makeOneHopTransfers`` computes one-hop transfers for all stops whose distance is below a specified threshold. This is used to create a transitively closed network for comparison with non-multi-modal algorithms.
* ``applyMaxTransferSpeed`` applies a maximum transfer speed to all edges in the transfer graph.
* ``applyConstantTransferSpeed`` applies a constant transfer speed to all edges in the transfer graph and computes the travel times accordingly.

An example script that combines all steps necessary to load a public transit network is provided at ``Runnables/BuildNetworkExample.script``. It can be run from the ``Network`` application using ``runScript BuildNetworkExample.script``. It takes as input GFTS data in CSV format located at ``Networks/Switzerland/GTFS/`` and a road graph in DIMACS format located at ``Networks/Switzerland/OSM/dimacs``.

## Multiple Transfer Modes
The algorithms listed above support bimodal networks with public transit and a single transfer mode. Additionally, this framework provides algorithms for multimodal networks with multiple transfer modes. The required multimodal data structures can be built with the following commands in ``Network``:
* ``buildMultimodalRAPTORData`` converts unimodal RAPTOR data into multimodal RAPTOR data. The transfer graph contained in the RAPTOR data is used for the "free" transfers whose transfer time is not penalized. The transfer graphs for the non-"free" modes must be added separately with the ``addModeToMultimodalRAPTORData``.
* ``addModeToMultimodalRAPTORData`` adds a transfer graph for a specified transfer mode to the given multimodal RAPTOR data.
* ``buildMultimodalTripBasedData`` converts unimodal TB data into multimodal TB data. The transfer graph contained in the TB data is used for the "free" transfers whose transfer time is not penalized. The transfer graphs for the non-"free" modes must be added separately with the ``addModeToMultimodalTripBasedData``.
* ``addModeToMultimodalTripBasedData`` adds a shortcut graph for a specified transfer mode to the given multimodal TB data.
`
Additionally, the command ``buildFreeTransferGraph`` in ``ULTRA`` builds a "free" transfer graph by connecting all pairs of stops within a specified geographical distance and then computing the transitive closure.

ULTRA shortcuts for networks with multiple transfer modes can be computed with the following commands in ``ULTRA``:
* ``computeMultimodalMcStopToStopShortcuts`` computes multimodal stop-to-stop McULTRA shortcuts for use with ULTRA-McRAPTOR and UBM-RAPTOR.
* ``computeMultimodalMcEventToEventShortcuts`` computes multimodal event-to-event McULTRA shortcuts for use with UBM-HydRA.

The ``ULTRA`` application offers the following query algorithms. All algorithms optimize arrival time, number of trips and one transfer time criterion per transfer mode.
* ``runMultimodalMCRQueries``: MCR for full Pareto sets
* ``runMultimodalULTRAMcRAPTORQueries``: ULTRA-McRAPTOR with stop-to-stop shortcuts for full Pareto sets
* ``runMultimodalUBMRAPTORQueries``: UBM-RAPTOR with stop-to-stop shortcuts for restricted Pareto sets
* ``runMultimodalUBMHydRAQueries``: UBM-HydRA with event-to-event shortcuts for restricted Pareto sets

## One-to-Many Journey Planning
The query algorithms in the `ULTRA` application only support one-to-one queries. The `ULTRAPHAST` application provides algorithms for one-to-all and one-to-many queries:

| Command                                      | Algorithm | Target set     | Criteria                      |
| -------------------------------------------- | --------- | -------------- | ----------------------------- |
| `runOneToAllDijkstraCSAQueriesToVertices`    | MCSA      | Vertices       | Arrival time                  |
| `runOneToManyDijkstraCSAQueriesToStops`      | MCSA      | Stops          | Arrival time                  |
| `runOneToManyDijkstraCSAQueriesToBall`       | MCSA      | Ball           | Arrival time                  |
| `runUPCSAQueries`                            | UP-CSA    | Vertices/Stops | Arrival time                  |
| `runUPCSAQueriesToBall`                      | UP-CSA    | Ball           | Arrival time                  |
| `runOneToAllDijkstraRAPTORQueriesToVertices` | MR        | Vertices       | Arrival time, number of trips |
| `runOneToManyDijkstraRAPTORQueriesToStops`   | MR        | Stops          | Arrival time, number of trips |
| `runOneToManyDijkstraRAPTORQueriesToBall`    | MR        | Ball           | Arrival time, number of trips |
| `runUPRAPTORQueries`                         | UP-RAPTOR | Vertices/Stops | Arrival time, number of trips |
| `runUPRAPTORQueriesToBall`                   | UP-RAPTOR | Ball           | Arrival time, number of trips |
| `runUPTBQueries`                             | UP-TB     | Vertices/Stops | Arrival time, number of trips |

Random ball target sets can be generated with the command `createBallTargetSets`. CH and Core-CH precomputations for these target sets can be run with `buildUPCHForTargetSets` and `buildCoreCHForTargetSets`, respectively.

## Delay-Robustness
The application ``DelayExperiments`` provides commands for evaluating Delay-ULTRA, the variant of ULTRA that anticipates possible vehicle delays. The delay-robust shortcut computation itself is run with the command ``computeDelayEventToEventShortcuts`` in ``ULTRA``. All delays up to the specified limit (measured in seconds) are accounted for. ``DelayExperiments`` provides the following commands:
* ``GenerateDelayScenario`` generates a delay scenario for the given network, using a synthetic delay model.
* ``GenerateDelayQueries`` generates queries for the specified delay scenario that are answered incorrectly by an algorithm without delay information.
* ``BuildFakeDelayData`` takes as input a network with regular ULTRA shortcuts and converts it to the format used by Delay-ULTRA. This is useful for comparing Delay-ULTRA to regular ULTRA. 
* ``RunDelayUpdatesWithoutReplacement`` simulates basic delay updates for the given delay scenario.
* ``RunDelayUpdatesWithReplacement`` simulates advanced delay updates for the given delay scenario. A heuristic replacement search is performed to find missing shortcuts.
* ``MeasureDelayULTRAQueryCoverage`` measures the result quality of TB using Delay-ULTRA shortcuts.
* ``MeasureHypotheticalDelayULTRAQueryCoverage`` measures the result quality of TB using Delay-ULTRA shortcuts, assuming that updates can be performed instantly.
* ``MeasureDelayULTRAQueryPerformance`` measures the query performance of TB using Delay-ULTRA shortcuts.