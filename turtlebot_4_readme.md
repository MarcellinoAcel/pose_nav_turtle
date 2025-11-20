# TurtleBot4 Setup & Run Guide

This guide explains how to set up your workspace, build the project, and run TurtleBot4 with navigation, localization, RViz visualization, and custom packages.

---

## 📁 1. Create Workspace & Build

1. Create a workspace:
```bash
mkdir -p ~/turtlebot4_delivery/src
cd ~/turtlebot4_delivery/src
```
2. Clone the GitHub repository:
```bash
git clone <your-turtlebot4-repo-link>
```
3. Build using colcon:
```bash
cd ~/turtlebot4_delivery
colcon build
```
4. Source the workspace:
```bash
source install/setup.bash
```

---

## 🖥️ 2. Tab 1 — Visualization (RViz)
### Option A: Run RViz remotely via SSH (recommended only if needed)
Connect with X forwarding:
```bash
ssh -X ubuntu@192.168.185.3
```
Launch the navigation RViz view:
```bash
ros2 launch turtlebot4_viz view_navigation.launch.py
```

### Option B: Run RViz locally (better performance)
If RViz is installed on your laptop, **no need for SSH -X**.
Simply run:
```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

---

## 📡 3. Tab 2 — Localization
Connect to the robot:
```bash
ssh ubuntu@192.168.185.3
```
Run localization with your map:
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=/path/to/map
```
In RViz:
- Use **2D Pose Estimate** to set the initial pose.

---

## 🤖 4. Tab 3 — Run Your Navigation Package
Connect again:
```bash
ssh ubuntu@192.168.185.3
```
Enter your workspace:
```bash
cd ~/turtlebot4_delivery
source install/setup.bash
```
Launch your navigation node:
```bash
ros2 launch pose_nav_turtle uts_nav.launch.py
```
Then test navigation using **Nav2 Goal** in RViz.

---

## ⚙️ 5. Tab 4 — Run Additional Nodes
Connect once more:
```bash
ssh ubuntu@192.168.185.3
```
Source workspace:
```bash
cd ~/turtlebot4_delivery
source install/setup.bash
```
Run your node:
```bash
ros2 run pose_nav_turtle pose_nav_turtle
```

---

## ✅ You’re Ready!
You now have:
- RViz visualization running (remote or local)
- Localization active
- Your navigation package launched
- Additional custom nodes running in separate terminals

Happy robot testing!

