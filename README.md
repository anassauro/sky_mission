# CBR 2024 - Flying Robots League

## Phase 1: Localization and mapping 

In this mission element, the task was to reconnaissance the arena, mapping the environment to detect mobile land bases (randomly allocated before the start of the phase) and detect suspended landing/takeoff bases. The robot was supposed to leave the takeoff base, travel around the arena while detecting the existing bases and land one time on each of the detected bases, after that returning to the takeoff base and landing. Prior to the competition an AI was trained using **YOLO v5** to accomplish the detection. 

![_MG_6248](https://github.com/SkyRats/sky_mission/assets/106029376/9b095ae0-3f05-4afc-98c5-8f8a441d9981)
><figcaption><p><em>Close-Up of the drone used in the competition</em></p></figcaption>

## Phase 2: Package transportation
n this mission element, 16 barcoded boxes were deposited along 4 shelves on the back of the arena. The  drone had to leave the take off base, go through each of the shelves on the shelf at a time while detecting and reading one barcode at a time, then return to the take off base and land. In order to properly distinguish the barcodes we also used the **pyzbar** library and set the drone to pause during its movement while scanning the shelf. 

![_MG_6311](https://github.com/SkyRats/sky_mission/assets/106029376/ebdbf2aa-7ab3-4a61-a1aa-be1190ccdf74)
><figcaption><p><em>Drone recognizing the barcoded boxes on the shelf</p></figcaption>

## Phase 3: Inventory control:
In this mission element, teams had to to transport packages from one landing/takeoff base (either suspended or mobile) to another. Each package had a QR code at the top with the letter of the base where it should have been delivered, however none of the teams had the technology to properly colect the package and deliver it. As a consequence, our task during the competition was limited to recognizing the QR codes, which we used the **pyzbar** library for, and returning back to the takeoff base. 

![_MG_6205](https://github.com/SkyRats/sky_mission/assets/106029376/6fcfb140-ccf1-4aa2-b109-e722723f8058)
><figcaption><p><em>Primary drone used during the competition</em></p></figcaption>
## Phase 4: Mobile Robot Landing:


## Extra: Thechnical Challenge:


