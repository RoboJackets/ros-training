# What are we doing today

-   What is Controls?
-   The Ideal PID Equation
-   How do we make this work in practice
-   Walkthrough of the current controls code
-   Advice on Tuning


# Control Theory

-   How do we reliably get from point A (our current state) to point B (our desired state)?
-   We call the difference between desired and current state at a given time <span class="underline">error</span>
-   The desired state is also referred to as the <span class="underline">setpoint</span>


## Closed Loop Controls

-   The output of our motor, referred to as its **effort** is what we have control over. Effort is typically something measured as a PWM signal
-   We get feedback on the state of the robot from sensors. For example, an encoder gives us ticks/second or an accelerometer would tell us g-forces. These tell us the *state* of the robot.
-   We want to inform the effort of the motor based on the measured robot state.


## Why do we need to study this?

-   Our robot will be operating in a variety of conditions
    -   Differences in battery voltage or driving surface means that the same motor effort can result in different speeds. We need to account for this
    -   We want our robots motion to be accurate and consistent despite changing conditions.
-   We want our robot's behavior to be "smooth".


## A Simple Example: bang-bang controller

```C++
effort <- Fixed Output
threshold <- Margin of Error
setpoint <- desired state
while(true) {
      if (current_state < setpoint - threshold) {
	    output(effort);
      } else if (current_state > setpoint + threshold) {
	    output(-effort);
      } else {
	    output(0);
      }
      update_current_state();
}
```

<div class="NOTES">
Pros: Gets us more or less to our setpoint Can account for changing conditions Cons: Bumpy rides ahead (not differentiable)

</div>


# The PID Controller

-   Let's vary our motor effort so that the robot's motion isn't as jerky
-   We're going to build a black box that takes a function of error over time and spits out motor effort
-   More specifically:

![img](https://www.researchgate.net/profile/Vishnu_Divakar/publication/281746636/figure/fig4/AS:284649973665803@1444877250888/Figure-5-PID-Equation.png)


## The Proportional Component

-   As we get closer to the setpoint, slow down!
-   The rate at which we slow down is given by Kp
-   This is good because we don't see the same boucing back and forth as bang-bang
-   This is bad because we'll never quite hit our setpoint (SP).

<div class="NOTES">
The problem with just a P controller is called steady state error The problem isn't as bad with a threshold

</div>


## The Integral Component

-   Very low error for a long period of time
-   Add up previous errors and produce an output proportional to that sum
-   Pro: We now reach our setpoint!
-   Con: An I gain that's too high will lead to oscillation about the setpoint

<div class="NOTES">
This and the D component will have to get sketched up on a whiteboard Note that the summing we are doing here is basically LRAM

</div>


## The Derivative Component

-   Rather than carrying about the error itself, try to keep the rate at which the error is changing stable
-   This helps keep the PID controller smooth
-   It can also help offset some of the oscillation seen from the I component


# Making this work in practice

-   We can define our state in terms of speed or distance travelled, but speed makes more sense.
-   In the I component don't sum over all the old values, just the most recent ones.
-   Rather than having your motor effort being output by this PID Controller, have the PID Controller dictate the change in motor effort.
-   Often, just a PD controller does the trick

<div class="NOTES">
If you have a wide enough margin for error - i.e steering angle - just a P controller can cut it

</div>


# How do we do it right now?

-   <https://github.com/robojackets/roboracing-firmware>


# Some advice on tuning

<div class="NOTES">
This is more of an art than a science

</div>

-   When starting, set I&D to 0 and just increment P until you're happy with thethe behavior
-   You shouldn't have to recompile/redeploy software everytime you want to tweak these gains. Launchfile paramaters are your friends!
-   Rqt (specifically rqt<sub>plot</sub>) is a really useful tool to look at how your error is changing

<div class="NOTES">
Things I wish I could cover but it wouldnt be realistic: (writing these down in case we do advanced spring sessions) Motion Profiling (not enough time) Gain Scheduling (not enough time) LQR (Wut. How even) Making our "current<sub>state</sub>" estimate more realistic via Kalman Filters or something of that nature. (out of scope + not enough time)

</div>