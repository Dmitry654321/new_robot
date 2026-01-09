# DuckieBot Examples

Here we have examples of the following functionalitiy for the DuckieBot:
1) A ROS workspace (src) with blinker package inside. The functionality of this is that is makes the DuckieBot LEDs blink in some colored order. This works by publishing to the /led_pattern topic for the respective vehicle.
2) Same as above but the package is image_saver whose functionality is that it saves an image from the camera every 30 calls. This works by subscribing to the /image/compressed topic for the respective vehicle.


This is the structure of each workspace (shown specifically with the blinker package described in 1) but something similar holds for all the rest).
```
src                   ← ROS workspace
└── blinker/              ← ROS package
    ├── blinker/          ← Python module 
    │   └── blinker.py    ← Your actual ROS node code
    ├── resource/
    │   └── blinker       
    ├── package.xml       ← ROS metadata (we add dependencies)
    ├── setup.py          ← Python packaging + ROS entry points (manually add line for each node)
    └── setup.cfg        
```

You can clone this repo and run things to see how they work. You cannot push any changes. 

Here are the steps to do to clone:
- Open terminal
- Navigate to where you want to put your cloned repo
- (hopefully you have an SSH key from in class activity, if not follow instruction on Google)
- From Github web, click green code button and copy SSH link
- In terminal, do ``` git clone [INSERT LINK] ```

Here are the steps to run things:
- fill
-

