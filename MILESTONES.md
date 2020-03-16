# Fusion Crowd Milestones

Here we describe some work we've done so far and our future goals. Theese are not milestones at all but you know, naming is hard. Please, don't expect these plans to be future-proof, we reserve the right to change them at any time. 

### 2019
* Models
  * [x] Finite state machine as strategic-level model
* Integration capabilities
  * [x] Set agent operational model
  * [x] Enchance simulation recording capabilities
* Tools for experiments
  * [x] Framework for running simulation scenarios (ITestCase)
  * [x] A few simulation scenarios using various models
  * [x] Simple visualization tool written in python
* NavMesh runtime cutting
  * [x] Cutting convex polygons from one node
  * [x] Cutting convex polygons from multiple nodes

### 2020
* Navigation Graph
  * [x] Loading from file
  * [x] Building in code
  * [x] Tactic model
* [ ] *WiP* Traffic lights 
* [ ] *WiP* Bycicle navigation model (operatinal level)
* [ ] *WiP* Road vehicle navigation model (operatinal level)
* Tools for experiments
  * [x] More simulation scenarios using various models
  * [x] Ability to draw navmesh and navgraph with viz tool
  * [ ] Scenario documentation
  * [ ] Benchmarking and measuring tools (density, flow speed, trajectory smoothness, etc)
* [x] Non-convex polygon cutting from navmesh
* Group navigation
  * [x] Group API (creation, deletion, navigation, add/remove agent, set group goal, etc)
  * [ ] *WiP* Simple fixed-layout group
  * [ ] Flexible-layout groups
  * [ ] Large group navigation
  * [ ] Group events: group formed, agents not in position, etc
* Parallelizaton
  * [ ] Multi-threaded parallelization
  * [ ] Multiple computation nodes parallelization
