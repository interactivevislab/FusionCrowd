# FusionCrowd

FusionCrowd is a library for multi-agent pedestrian and transport simulations.

## High-level architecture

On a high level, library consists of 6 main components:
- Simulator 
- Navigation System 
- Strategy components 
- Tactic Components 
- Operational components 
- Export 

Simulator controls the simulation loop and routes the control flow across components.

## Solution structure
https://github.com/itmo-escience/FusionCrowd

This solution requires Windows SDK 10.0.17763.0 and Visual Studio 2017 or later.

Solution consists of three main projects:
- FusionCrowd  – this is the library itself
- TestFusionCrowd  – this project contains examples with some simple infrastructure for running them.
- UnitTest – an attempt to test the library, contains a few tests using Microsoft Unit Testing Framework for C++

There also are two projects (FusionCrowdClient and FusionCrowdServer) in abandoned state.
