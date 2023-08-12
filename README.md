# Adaptive Controller for Hospital Bed

I completed this project for a graduate level control systems course I took (Design of Control Systems). Essentially, using [I came across this hospital bed](https://www.mdpi.com/2076-3417/11/18/8459) and thought that it would be cool to design my own controller for this system for the semester-long project in this course. 

The bed itself has a pendant that the patient uses to move the bed using open-loop control. In my eyes, this was a flawed method of preventing pressure sores because the patient would only be moving the bed after feeling some sort of pain (indicating that the pressure sore has already started to develop). Instead, the goal of this project was to place pressure sensors at 17 different points along the bed, and the controller would subtly (without the patient feeling) move around to distribute the patient's bodyweight around in order to prevent sores.

![image](https://github.com/nathanmarcel/adaptive-hospital-bed-controller/assets/93691232/cff13e20-5003-4afc-a33c-545e133ce86a)
