BLDC and PMSM simulator
=====================
This interactive simulator is designed to visualize the inner workings of BLDC and PMSM control algorithms and provide a test bed for prototyping of more advanced control algorithms. Parameters like rotor inertia, bus voltage, PWM timer frequency, and many more can be adjusted in real time to see their effects on the control policy.

![image](https://user-images.githubusercontent.com/58680214/90294531-ab40ef00-de54-11ea-831c-d248d98dd388.png)

Starting Up
------------
Install the bazel build system and clone this project. Run `bazel run -c opt simulator:simulator`

Quick Start to FOC Simulation
-----------------
Step 1. In the Commutation Control Mode tab, enable FOC.
![step1](https://user-images.githubusercontent.com/58680214/90319559-4d211400-df07-11ea-91c7-7832133f5197.jpg)

Step 2. Then set a desired torque.
![step2](https://user-images.githubusercontent.com/58680214/90319557-4abeba00-df07-11ea-9ff2-dd814851287e.jpg)

Step 3. Adjust the step multiplier to change the speed of simulation. 
![step3](https://user-images.githubusercontent.com/58680214/90319558-4befe700-df07-11ea-85dc-260200905ad2.jpg)

Now the rotor should start spinning up.
![rotor_viz](https://user-images.githubusercontent.com/58680214/90319684-5494ed00-df08-11ea-8150-4654c97bdbc2.jpg)



