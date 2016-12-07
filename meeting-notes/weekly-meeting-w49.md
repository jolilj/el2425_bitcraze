# Weekly Meeting W49

* Antonio asked about how we divide the work, we gave him access to our spreadsheet with the working hours on Google Drive so that he can keep himself informed about our contributions
* The trajectory planning for one agent works very well, so we should be getting started with multiple agents (same channel, different addresses according to whoening's answer on Antonios issue on GitHub)
* Demonstration of the plotter with planned/actual trajectory
* Idea: Making the crazyflie react to key presses, time might be short for that

## Tasks

* **Imran and Saber**: Writing a higher level trajectory publisher for more complex trajectories
* **Pedram**: Set up automatic logging framework (should be automatically launched in connect.launch with a parameter log=True) and investigate into deviations of the crazyflie from planned trajectory (data processing in Matlab))
* If needed: Tweaking parameters in PositionHandler.py (delT, k..)
* **Hui (and Pedram)**: Review collision avoidance
* **Robert and Joakim**: Control two crazyflies
* **Joakim**: Document and comment current code
* **Caro**: Writing second project review and starting with project report

## The upcoming weeks:

Unless we finish it this week:

* Implementing a collision avoidance algorithm

Week 50:

* Define and implement path planning tasks (position swapping, following a moving target)

Week 51:

* finish up project
* Write project report
* Record a project movie
* Create presentation slides