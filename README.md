# CBR 2023 - Flying Robots League

Repository to add codes for the CBR 2023. The competition consisted of 4 phases. The team reached fifth place, scoring in phase 3. The team's Software technology consisted of the use of **Ardupilot, Drone, OpenCV and YOLO v5**. Innovations brought by the team in the competition were, for example, **precision landing**, the use of a synthetic dataset created in **Blender** to detect bases.

Due to time and logistics limitations, phases 1, 2 and 3 were not able to be completed. However, the base codes that were tested in a **Gazebo** simulation are situated in this repository, along with the codes that were used during the competition. 

The repository is divided into:
* `stage_1`, `stage_2`, `stage_3`, `stage_4` : codes used to prepare for the phases and test during the competition
* `testing`: codes to test basic functions in a flying robot&#x20;
  
<figure><img src="" alt=""><figcaption><p><em>Photo taken with all the teams participating in the competition</em></p></figcaption></figure>

## Phase 1: Localization and mapping 

In this mission element, the task was to reconnaissance the arena, mapping the environment to detect mobile land bases (randomly allocated before the start of the phase) and detect suspended landing/takeoff bases. The robot was supposed to leave the takeoff base, travel around the arena while detecting the existing bases and land one time on each of the detected bases, after that returning to the takeoff base and landing. Prior to the competition an AI (...)

<figure><img src="assets/fase1.JPG" alt=""><figcaption><p><em>F450: Drone used in phase 1</em></p></figcaption></figure>

## Phase 2: Package transportation

In this mission element, teams had to locate four a priori known objects on the ground and create an ortho map with which the track of the hikers can be reconstructed. We used the Agisoft Metashape Pro software to create the map. In tests, it was possible to build maps of reasonable quality. On the day of the test, due to the low quality of the photos, the map processing was hampered. Examples can be seen in the `deliverables` folder. Furthermore, in the `fase2` folder, all mapping and post-processing codes are present. In the `missions` folder, you can see the survey planned for the phase.

<figure><img src="assets/_MG_5066.JPG" alt=""><figcaption><p><em>Condor: Drone used in phase 2</em></p></figcaption></figure>

## Phase 3: Inventory control:

In this mission element, four waypoints had to be reached in an apriori unknown waypoint sequence. The problem was that the waypoints would be given when the drone was already flying and far away, making communication difficult. It was not necessary to use an on-board computer. We only relied on Lua codes on the controller and Ardupilot parameters to avoid objects in the field and create optimized trajectories, using, for example, the Dijkstra's algorithm.&#x20;

<figure><img src="assets/_MG_5043.JPG" alt=""><figcaption><p><em>Tixa</em>: Autonomous FPV planned to be used in phase 3</p></figcaption></figure>

## Phase 4: Mobile Robot Landing:&#x20;

In this mission element, the task was to fly to a known position and identify the number and state (lying on the ground, standing, sitting) the hikers are in. In this task, a synthetic dataset was created with blender and **YOLO v5** was used to train an AI to detect blue dummies that needed to be rescued. The mission for phase 4 is available in the `missions` folder and all image processing is in the `fase4` folder, including the YOLO weights.

<figure><img src="assets/_MG_5090.JPG" alt=""><figcaption><p><em>Morcego: Drone planned to be used in phase 4 and 5</em></p></figcaption></figure>


