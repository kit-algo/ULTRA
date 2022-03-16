[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# ULTRA
UnLimited TRAnsfers for Multi-Modal Route Planning

ULTRA is a C++ framework for efficient journey planning in multimodal networks consisting of public transit and a non-schedule-based secondary transportation mode (e.g., walking, cycling, taxi). It was developed at [KIT](https://www.kit.edu) in the [group of Prof. Dorothea Wagner](https://i11www.iti.kit.edu/).

## Usage

This framework contains code for the ULTRA preprocessing step (i.e., contraction hierarchies and shortcut computation) as well as two query algorithms (ULTRA-RAPTOR and ULTRA-CSA). Each of the major components of the framework can be used/tested separately using on of the programs in the ``Runnables`` folder:

* ``BuildCoreCH`` takes the public transit network and computes the core-CH required for the ULTRA preprocessing
* ``ComputeShortcuts`` takes the public transit network as well as the core-CH and computes the ULTRA shortcuts
* ``BuildBucketCH`` takes the transfer graph and computes the CH required in the query phase for initial/final transfers
* ``RunRAPTORQueries`` takes the public transit network, shortcuts, and CH and performs ULTRA-RAPTOR experiments
* ``RunCSAQueries`` takes the public transit network, shortcuts, and CH and performs ULTRA-CSA experiments

The ``Makefile`` located in the ``Runnables`` folder contains instructions for building all of the above programs. Simply edit the top part of the Makefile to adjust the compiler and flags available to you and run ``make`` afterwards.

All of the above programs use a custom binary format for loading the public transit network as well as the transfer graph. As an example we provide the public transit network of Switzerland together with a transfer graph extracted from OpenStreetMap in the appropriate binary format at [https://i11www.iti.kit.edu/PublicTransitData/Switzerland/binaryFiles/](https://i11www.iti.kit.edu/PublicTransitData/Switzerland/binaryFiles/). 

## Publications

A detailed introduction to the ULTRA approach as well as an in-depth evaluation of its performance is given in:

* *UnLimited TRAnsfers for Multi-Modal Route Planning: An Efficient Solution*  
  Moritz Baum, Valentin Buchhold, Jonas Sauer, Dorothea Wagner, Tobias Zündorf  
  In: Proceedings of the 27th Annual European Symposium on Algorithms (ESA'19), Leibniz International Proceedings in Informatics, pages 14:1–14:16, 2019  
  [pdf](https://drops.dagstuhl.de/opus/volltexte/2019/11135/pdf/LIPIcs-ESA-2019-14.pdf) [arXiv](https://arxiv.org/abs/1906.04832)
