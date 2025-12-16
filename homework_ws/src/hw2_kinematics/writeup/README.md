# Homework 2: Kinematics

## Q1.1: Kinematics Model Derivation

### Derivation for x_t

Starting with the differential equation for x:

$$\frac{\partial x}{\partial t} = v \cos \theta$$

Integrating both sides from $t$ to $t + \Delta t$:

$$\int_{x_{t-1}}^{x_t} dx = \int_{t}^{t+\Delta t} v \cos \theta \, dt$$

The left side gives us:

$$x_t - x_{t-1} = \int_{t}^{t+\Delta t} v \cos \theta \, dt$$

Now, we need to handle the fact that $\theta$ changes during the time step. From the given equation, we know:

$$\theta(t') = \theta_{t-1} + \frac{v}{L} \tan \alpha (t' - t)$$

for $t' \in [t, t+\Delta t]$.

Substituting this into our integral:

$$x_t - x_{t-1} = \int_{t}^{t+\Delta t} v \cos\left(\theta_{t-1} + \frac{v}{L} \tan \alpha (t' - t)\right) dt'$$

Let $\omega = \frac{v}{L} \tan \alpha$. Then:

$$x_t - x_{t-1} = v \int_{0}^{\Delta t} \cos(\theta_{t-1} + \omega \tau) d\tau$$

$$= v \left[\frac{1}{\omega} \sin(\theta_{t-1} + \omega \tau)\right]_0^{\Delta t}$$

$$= \frac{v}{\omega} [\sin(\theta_{t-1} + \omega \Delta t) - \sin(\theta_{t-1})]$$

Since $\theta_t = \theta_{t-1} + \omega \Delta t$:

$$x_t = x_{t-1} + \frac{v}{\omega}[\sin(\theta_t) - \sin(\theta_{t-1})]$$

$$x_t = x_{t-1} + \frac{L}{\tan \alpha}[\sin(\theta_t) - \sin(\theta_{t-1})]$$

### Derivation for y_t

Following the same process for y:

$$\frac{\partial y}{\partial t} = v \sin \theta$$

Integrating:

$$\int_{y_{t-1}}^{y_t} dy = \int_{t}^{t+\Delta t} v \sin \theta \, dt$$

$$y_t - y_{t-1} = \int_{t}^{t+\Delta t} v \sin\left(\theta_{t-1} + \frac{v}{L} \tan \alpha (t' - t)\right) dt'$$

With $\omega = \frac{v}{L} \tan \alpha$:

$$y_t - y_{t-1} = v \int_{0}^{\Delta t} \sin(\theta_{t-1} + \omega \tau) d\tau$$

$$= v \left[-\frac{1}{\omega} \cos(\theta_{t-1} + \omega \tau)\right]_0^{\Delta t}$$

$$= -\frac{v}{\omega} [\cos(\theta_{t-1} + \omega \Delta t) - \cos(\theta_{t-1})]$$

$$= \frac{v}{\omega} [\cos(\theta_{t-1}) - \cos(\theta_t)]$$

$$y_t = y_{t-1} + \frac{v}{\omega}[\cos(\theta_{t-1}) - \cos(\theta_t)]$$

$$y_t = y_{t-1} + \frac{L}{\tan \alpha}[\cos(\theta_{t-1}) - \cos(\theta_t)]$$


## Q1.3: Kinematics Model Derivation

![deterministic_rollouts](deterministic_rollouts.png)


---

## Q2.4: Kinematic Simulation Analysis

### 1. End-Effector Orientation

The orientation doesn't match because we're only controlling position. Our controller uses a 3x6 position Jacobian that only relates joint velocities to the camera's translational velocity. We have 6 joints but only 3 position constraints, so the remaining 3 DOFs (which affect orientation) are free to do whatever minimizes the joint velocity norm.

To track orientation too, we'd need to use the full 6x6 Jacobian (including the angular velocity rows) and add an orientation error term to our control law. Something like computing the rotation error between current and target orientations and turning that into a desired angular velocity.

### 2. Behavior with Different Kp Values

With Kp=0.1, the arm barely moves. It crawls toward the target really slowly and never catches up when the target is moving. The tracking error stays large.

With Kp=50.0, the arm jerks around violently. It shoots past the target, then overcorrects back the other way, and keeps bouncing around. The high gain makes tiny errors produce huge velocity commands, and the discrete time steps mean it can't settle down properly.

A moderate Kp around 1-5 works best - fast enough to actually track the target without going crazy. I can imagine if you were to set Kp more than 50 that it would oscillate widly and I actually tried it and my screen froze :).
