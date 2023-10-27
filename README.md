# CBR 2023 - Flying Robots League

Repository to add codes for the CBR 2023. The competition consisted of 4 phases. The team reached fifth place, scoring in phase 3. The team's Software technology consisted of the use of **Ardupilot, Dronekit, OpenCV and YOLO v5**. Innovations brought by the team in the competition were, for example, **precision landing**, the use of a synthetic dataset created in **Blender** to detect bases.

Due to time and logistics limitations, phases 1, 2 and 3 were not able to be completed. However, the base codes that were tested in a **Gazebo** simulation are situated in this repository, along with the codes that were used during the competition. Also, as a strategy we opted for using a Tello to accomplish phase 4, securing thirty points for the team. 

The repository is divided into:
* `stage_1`, `stage_2`, `stage_3`, `stage_4` : codes used to prepare for the phases and test during the competition
* `testing`: codes to test basic functions in a flying robot&#x20;
  
![cbr](https://github.com/SkyRats/sky_mission/assets/106029376/480cc916-04fb-4f46-853d-525aaf8dfef1)
 ><figcaption><p><em>Skyrats members that went to CBR</em></p></figcaption>

## Phase 1: Localization and mapping 

In this mission element, the task was to reconnaissance the arena, mapping the environment to detect mobile land bases (randomly allocated before the start of the phase) and detect suspended landing/takeoff bases. The robot was supposed to leave the takeoff base, travel around the arena while detecting the existing bases and land one time on each of the detected bases, after that returning to the takeoff base and landing. Prior to the competition an AI was trained using **YOLO v5** to accomplish the detection. 

![_MG_6248](https://github.com/SkyRats/sky_mission/assets/106029376/9b095ae0-3f05-4afc-98c5-8f8a441d9981)
><figcaption><p><em>Close-Up of the drone used in the competition</em></p></figcaption>

## Phase 2: Package transportation

In this mission element, teams had to to transport packages from one landing/takeoff base (either suspended or mobile) to another. Each package had a QR code at the top with the letter of the base where it should have been delivered, however none of the teams had the technology to properly colect the package and deliver it. As a consequence, our task during the competition was limited to recognizing the QR codes, which we used the **pyzbar** library for, and returning back to the takeoff base. 

![_MG_6205](https://github.com/SkyRats/sky_mission/assets/106029376/6fcfb140-ccf1-4aa2-b109-e722723f8058)
><figcaption><p><em>Primary drone used during the competition</em></p></figcaption>

## Phase 3: Inventory control:

In this mission element, 16 barcoded boxes were deposited along 4 shelves on the back of the arena. The  drone had to leave the take off base, go through each of the shelves on the shelf at a time while detecting and reading one barcode at a time, then return to the take off base and land. In order to properly distinguish the barcodes we also used the **pyzbar** library and set the drone to pause during its movement while scanning the shelf. 

![_MG_6311](https://github.com/SkyRats/sky_mission/assets/106029376/ebdbf2aa-7ab3-4a61-a1aa-be1190ccdf74)
><figcaption><p><em>Drone recognizing the barcoded boxes on the shelf</p></figcaption>

## Phase 4: Mobile Robot Landing:

In this mission element, the team's robots was required to reconnaissance the arena, mapping the environment to detect and land on a base coupled to a land mobile robot. Due to errors in the realsense connected to the drones we were originally using, we opted to utilize as a Plan B a Tello Drone from DJI. The code was based in using the detection of a abrupt height change to kill the drone, that way making it "land" on the base. 

![Screenshot from MVI_6389 MOV](https://github.com/SkyRats/sky_mission/assets/106029376/bf2e8686-fb09-4941-9ff2-68e979ec8f78)
><figcaption><p><em>Tello Drone landing on a moving base</em></p></figcaption>

## Extra: Thechnical Challenge:

Separate challenge held in the end of the competition that evaluate advances in state-of-the-art flying robot technology that were made. Each team must present a 10 minute Workshop disseminating their drone technologies. For the challenge we chose to present our base detection AI, that inovated by using a synthetic made dataset for its training. 

![WhatsApp Image 2023-10-26 at 17 22 49](https://github.com/SkyRats/sky_mission/assets/106029376/5d318c4e-41f4-477d-8be1-d06916691d91)
><figcaption><p><em>AI recognizing two bases in a photo taken during the event </em></p></figcaption>

