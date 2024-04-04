# SUNGA 2 - Outdoor mission

Repository to add codes for the 2023 edition of "SUNGA". SUNGA is a internal project that aims at the development of team members and technology . In this edition the focus was precision, qr code detection and collecting packages on outdoor missions. The team's Software technology consisted of the use of **ROS and OpenCV**.

The repository is divided into:
* `mov`: codes to test basic movements in a flying robot
* `magnet`: codes to test the electro magnet 
* `qr_code`: codes involving reading and identifying qr codes 
* `final`: codes with final missions&#x20;

To execute the codes, it is needed to initialize mavros besides the actual code.

terminal 1:
```bash
roslaunch sky_sim apm.launch
```

terminal 2:
```bash
python3 <code>
```