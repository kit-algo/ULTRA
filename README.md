[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# ULTRA: UnLimited TRAnsfers for Multi-Modal Route Planning

ULTRA is a C++ framework for efficient journey planning in multimodal networks consisting of public transit and a non-schedule-based secondary transportation mode (e.g., walking, cycling, taxi). It was developed at [KIT](https://www.kit.edu) in the [group of Prof. Dorothea Wagner](https://i11www.iti.kit.edu/). This repository contains code for the following publications: 

* *UnLimited TRAnsfers for Multi-Modal Route Planning: An Efficient Solution*
  Moritz Baum, Valentin Buchhold, Jonas Sauer, Dorothea Wagner, Tobias Zündorf
  In: Proceedings of the 27th Annual European Symposium on Algorithms (ESA'19), Leibniz International Proceedings in Informatics, pages 14:1–14:16, 2019
  [pdf](https://drops.dagstuhl.de/opus/volltexte/2019/11135/pdf/LIPIcs-ESA-2019-14.pdf) [arXiv](https://arxiv.org/abs/1906.04832)

* *Integrating ULTRA and Trip-Based Routing*
  Jonas Sauer, Dorothea Wagner, Tobias Zündorf
  In: Proceedings of the 20th Symposium on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems (ATMOS'20), OpenAccess Series in Informatics, pages 4:1–4:15, 2020
  [pdf](http://i11www.ira.uka.de/extra/publications/swz-iultr-20.pdf)

* *Fast Multimodal Journey Planning for Three Criteria*
  Moritz Potthoff, Jonas Sauer
  In: Proceedings of the 20th Symposium on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems (ATMOS'20), OpenAccess Series in Informatics, pages 4:1–4:15, 2020
  [pdf](https://epubs.siam.org/doi/epdf/10.1137/1.9781611977042.12) [arXiv](https://arxiv.org/abs/2110.12954)

## Usage
All query algorithms and preprocessing steps are available via the console application ``ULTRA``, which can be compiled using the ``Makefile`` located in the ``Runnables`` folder. It includes the following commands:

* CH computation:
    - ``buildCH`` performs a regular CH precomputation. The output is required by the (Mc)ULTRA query algorithms, which use it to perform the Bucket-CH precomputation.
    - ``buildCoreCH`` performs a Core-CH precomputation. The output is required by the (Mc)ULTRA shortcut computation and by the MCSA and M(C)R query algorithms.
* (Mc)ULTRA shortcut computation:
    - ``computeStopToStopShortcuts`` computes stop-to-stop ULTRA shortcuts for use with ULTRA-RAPTOR.
    - ``computeEventToEventShortcuts`` computes event-to-event ULTRA shortcuts for use with ULTRA-TB.
    - ``computeMcStopToStopShortcuts`` computes stop-to-stop McULTRA shortcuts for use with ULTRA-McRAPTOR and ULTRA-BM-RAPTOR.
    - ``computeMcEventToEventShortcuts`` computes event-to-event McULTRA shortcuts for use with ULTRA-McTB and ULTRA-BM-TB.
    - ``augmentTripBasedShortcuts`` performs the shortcut augmentation step that is required for ULTRA-BM-TB.
    - ``validateStopToStopShortcuts`` and ``validateEventToEventShortcuts`` test the validity of the computed shortcuts by comparing them to paths in the original transfer graph.
* Original TB preprocessing:
    - ``raptorToTripBased`` takes a network in RAPTOR format as input and runs the TB preprocessing algorithm.
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
| ``runTransitiveMcRAPTORQueries``        | McRAPTOR        | Transitive | Stop-to-stop     | Arrival time, number of trips, transfer time (full)       |
| ``runMCRQueries``                       | MCR             | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (full)       |
| ``runULTRAMcRAPTORQueries``             | ULTRA-McRAPTOR  | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (full)       |
| ``runTransitiveBoundedMcRAPTORQueries`` | BM-RAPTOR       | Transitive | Stop-to-stop     | Arrival time, number of trips, transfer time (restricted) |
| ``RunUBMRAPTORQueries``                 | ULTRA-BM-RAPTOR | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (restricted) |
| ``runTransitiveTripBasedQueries``       | TB              | Transitive | Stop-to-stop     | Arrival time, number of trips                             |
| ``runULTRATripBasedQueries``            | ULTRA-TB        | Unlimited  | Vertex-to-vertex | Arrival time, number of trips                             |
| ``runULTRAMcTripBasedQueries``          | ULTRA-McTB      | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (full)       |
| ``runBoundedULTRAMcTripBasedQueries``   | ULTRA-BM-TB     | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (restricted) |

## Networks
We use custom data formats for loading the public transit network and the transfer graph: The Intermediate format allows for easy network manipulation, while the RAPTOR format is required by the preprocessing and all query algorithms except for CSA, which uses its own format. The Switzerland and London networks used in our experiments are available at [https://i11www.iti.kit.edu/PublicTransitData/ULTRA/](https://i11www.iti.kit.edu/PublicTransitData/ULTRA/) in the required formats. Unfortunately, we cannot provide the Germany and Stuttgart networks because they are proprietary.

The ``Network`` application provides commands for manipulating the network data and for converting public transit data to our custom format. It includes the following commands:
* ``parseGTFS`` converts GFTS data in CSV format to a binary format.
* ``gtfsToIntermediate`` converts GFTS binary data to the Intermediate network format.
* ``intermediateToCSA`` converts a network in Intermediate format to CSA format.
* ``intermediateToRAPTOR`` converts a network in Intermediate format to RAPTOR format.
* ``loadDimacsGraph`` converts a graph in the format used by the [9th DIMACS Implementation Challenge](http://users.diag.uniroma1.it/challenge9/download.shtml) to our custom binary graph format.
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
