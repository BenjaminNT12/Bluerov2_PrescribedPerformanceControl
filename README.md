# 🐋 PPC-Rigidity-Based Formation Control for BlueROV2 Agents

This repository provides a MATLAB-based simulation framework for evaluating **Prescribed Performance Control (PPC)** in conjunction with **Rigidity Theory** for the coordinated formation control of **underwater autonomous vehicles (UAVs)** modeled using **BlueROV2** parameters.

The simulation captures the performance of a 3-agent triangular formation led by a virtual leader. The dynamics are based on real BlueROV2 specifications and allow the user to evaluate position-tracking performance, formation maintenance, and inter-agent coordination in the presence of bounded disturbances.

---

## 🧠 Key Features

- ✅ Full 3D simulation of underwater dynamics based on BlueROV2
- ✅ Multi-agent formation via **graph-based rigidity constraints**
- ✅ **Prescribed Performance Control**: ensures bounded tracking error evolution
- ✅ Vectorized simulation using custom Runge-Kutta 4 integrator
- ✅ Performance metric computation and visualization scripts
- ✅ Modular structure for scalability to more agents or alternate control laws

---

## 📁 File Structure

| Filename                                         | Description |
|--------------------------------------------------|-------------|
| **BlueROV2ModelParameters.m**                    | Contains physical and hydrodynamic parameters of the BlueROV2 model. |
| **VehicleModel.m**                               | Encapsulates the underwater vehicle dynamics (mass, drag, etc.). |
| **multiagent_underwater_model_blueroV2.m**       | Multi-agent simulation model interfacing all vehicle dynamics. |
| **modelFunction.m**                              | Used to compute state derivatives during RK4 integration. |
| **RK4step_new.m**                                | Custom Runge-Kutta 4 integrator for time propagation. |
| **qtinVector.m**                                 | Computes quaternion-to-Euler angle conversion or related vector projections. |
| **rotationMatrix.m**                             | Utility to compute body-to-inertial or inertial-to-body transformations. |
| **main.m**                                       | 🚀 Main script to run the full simulation including initialization, integration, and plotting. |
| **matrizRTriangle3AgentWithLeader.m**            | Rigid formation matrix and incidence definitions for a 3-agent triangle graph with a leader. |
| **calculateFrameworkCoords.m**                   | Computes the desired shape positions based on the rigidity graph. |
| **calculatePerformanceMetrics.m**                | Computes PPC-transformed errors and performance functions. |
| **calculateAdditionalMetrics.m**                 | Computes mean square error, steady-state error, etc. |
| **PIDController.m**                              | (Optional) Baseline controller for comparative purposes. |
| **plotPositionErrors.m**                         | Generates plots of tracking errors against time and performance bounds. |
| **Framework3Dplot.m**                            | Visualizes 3D formation framework at final or intermediate times. |
| **Framework3Dplot_Initial.m**                    | Plots the initial configuration of agents and desired shape. |

---

## 🧪 Running the Simulation

```matlab
% In MATLAB:
>> main
