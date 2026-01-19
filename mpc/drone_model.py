from casadi import *
import numpy as np
import matplotlib.pyplot as plt

MOTOR_LEVER_ARM = 0.15556 # 0.22 / sqrt(2) meters for 440mm rod
OMEGA_SQUARED_OVER_FORCE = 3337.3729 # proportionality constant between force and omega^2 (approximation from MATLAB)
OMEGA_SQUARED_OVER_Z_MOMENT = 287356.32 # proportionality constant between z moment and omega^2 (approximation from MATLAB)
MAX_OMEGA_SQUARED = 5234944.0 # approx (2288 rad/s)^2 (approximation of 2200kv motor at 10V)

#constants for converting motor speed to control input (not that good right now, only FF to get right stead state speed)
CONTROL_INPUT_OVER_OMEGA_SQUARED = 6.863e-6
CONTROL_INPUT_OVER_OMEGA = 0.0044

MASS = 1.0

MAX_VOLT = 10.5

MOMENT_OF_INERTIA_INV = np.linalg.inv(np.array([[0.005194, 0.0, 0.0], [0.0, 0.005194, 0.0], [0.0, 0.0, 0.010296]]))

class DroneModel:
    def __init__(self):
        self.state = self.empty_state()

    def update(self, dt, u):
        x = self.state
        x_dot = self.state_transfer_function(x, u)
        self.state = x + x_dot * dt

    def empty_state(self):
        #x, y, z, xdot, ydot, zdot, roll, pitch, yaw, rolldot, pitchdot, yawdot, motor0_omega, motor1_omega, motor2_omega, motor3_omega
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def moments_to_omega_squared_matrix(self):
        motor_force_no_yaw_mat = np.array([
            [MOTOR_LEVER_ARM, MOTOR_LEVER_ARM, 0.0, 1.0],
            [-MOTOR_LEVER_ARM, MOTOR_LEVER_ARM, 0.0, 1.0],
            [-MOTOR_LEVER_ARM, -MOTOR_LEVER_ARM, 0.0, 1.0],
            [MOTOR_LEVER_ARM, MOTOR_LEVER_ARM, 0.0, 1.0],
        ]) / 4.0 * OMEGA_SQUARED_OVER_FORCE
        motor_force_from_yaw_mat = np.array([
            [0.0, 0.0, -OMEGA_SQUARED_OVER_Z_MOMENT, 0.0],
            [0.0, 0.0, OMEGA_SQUARED_OVER_Z_MOMENT, 0.0],
            [0.0, 0.0, -OMEGA_SQUARED_OVER_Z_MOMENT, 0.0],
            [0.0, 0.0, OMEGA_SQUARED_OVER_Z_MOMENT, 0.0]
        ])
        omega_squared_from_moments_and_force = motor_force_from_yaw_mat + motor_force_no_yaw_mat
        return omega_squared_from_moments_and_force

    def omega_squared_to_moments_matrix(self):
        return np.linalg.inv(self.moments_to_omega_squared_matrix())
    
    def omega_squared_to_u(self, omega_squared_array):
        # Find max omega_squared and clamp it
        max_omega_squared_mag = np.max(np.abs(omega_squared_array))
        
        if max_omega_squared_mag > MAX_OMEGA_SQUARED:
            scaling_factor = MAX_OMEGA_SQUARED / max_omega_squared_mag
            omega_array = np.sign(omega_squared_array) * np.sqrt(np.abs(omega_squared_array)) * scaling_factor
        else:
            omega_array = np.sign(omega_squared_array) * np.sqrt(np.abs(omega_squared_array))
        
        control_input_array = omega_squared_array * CONTROL_INPUT_OVER_OMEGA_SQUARED + omega_array * CONTROL_INPUT_OVER_OMEGA
        return control_input_array
    
    def control_input_to_alpha(self, u, omega):
        # omega_dot = 3280.991105*u + -14.957832*omega + -0.000446*omega^2 + -0.020689*|omega|*omega + 37.643698*sgn(omega)

        # ????? need to find out more about this in matlab, right now ive only modelled steady state behavior
        return 3280.991105 * u - 14.957832 * omega - 0.000446 * omega**2 - 0.020689 * abs(omega) * omega + 37.643698 * np.sign(omega)
    
    def control_input_to_omega_casadi(self, u, omega):
        return (3280.991105 * u
            - 14.957832 * omega
            - 0.000446   * omega**2
            - 0.020689   * fabs(omega) * omega
            + 37.643698  * sign(omega))


    def R_xyz(self, roll, pitch, yaw):
        cr = np.cos(roll);  sr = np.sin(roll)
        cp = np.cos(pitch); sp = np.sin(pitch)
        cy = np.cos(yaw);   sy = np.sin(yaw)

        return np.array([
            [ cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr ],
            [ sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr ],
            [  -sp ,           cp*sr,            cp*cr    ]
        ])
    
    #takes in x and u and returns x_dot
    def state_transfer_function(self, x, u):
        w_to_m = self.omega_squared_to_moments_matrix()
        omega_squared_vector = np.array([[x[12]**2], [x[13]**2], [x[14]**2], [x[15]**2]])
        moments_vector = w_to_m @ omega_squared_vector
        local_z_force = moments_vector[3, 0]
        roll = x[6]
        pitch = x[7]
        yaw = x[8]

        global_force_vec = self.R_xyz(roll, pitch, yaw) @ np.array([[0.0], [0.0], [local_z_force]])

        global_accel_vec = global_force_vec / MASS

        moments_vector = moments_vector[0:3, :]
        rotational_rates_vec = MOMENT_OF_INERTIA_INV @ moments_vector

        state_contribution = np.array([x[3], x[4], x[5], global_accel_vec[0, 0], global_accel_vec[1, 0], global_accel_vec[2, 0] - 9.8, x[9], x[10], x[11], rotational_rates_vec[0, 0], rotational_rates_vec[1, 0], rotational_rates_vec[2, 0], self.control_input_to_alpha(u[0], x[12]), self.control_input_to_alpha(u[1], x[13]), self.control_input_to_alpha(u[2], x[14]), self.control_input_to_alpha(u[3], x[15])])

        return state_contribution
    
    def state_transfer_function_casadi(self, x, u):
        w_to_m = self.omega_squared_to_moments_matrix()
        omega_squared_vector = vertcat(x[12]**2, x[13]**2, x[14]**2, x[15]**2)
        moments_vector = w_to_m @ omega_squared_vector
        local_z_force = moments_vector[3]
        roll = x[6]
        pitch = x[7]
        yaw = x[8]

        cr = cos(roll);  sr = sin(roll)
        cp = cos(pitch); sp = sin(pitch)
        cy = cos(yaw);   sy = sin(yaw)

        R = vertcat(
            horzcat(cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr),
            horzcat(sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr),
            horzcat(-sp, cp*sr, cp*cr)
        )

        global_force_vec = R @ vertcat(0.0, 0.0, local_z_force)

        global_accel_vec = global_force_vec / MASS

        moments_vector = moments_vector[0:3]
        rotational_rates_vec = MOMENT_OF_INERTIA_INV @ moments_vector

        state_contribution = vertcat(
            x[3],
            x[4],
            x[5],
            global_accel_vec[0],
            global_accel_vec[1],
            global_accel_vec[2] - 9.8,
            x[9],
            x[10],
            x[11],
            rotational_rates_vec[0],
            rotational_rates_vec[1],
            rotational_rates_vec[2],
            self.control_input_to_omega_casadi(u[0], x[12]),
            self.control_input_to_omega_casadi(u[1], x[13]),
            self.control_input_to_omega_casadi(u[2], x[14]),
            self.control_input_to_omega_casadi(u[3], x[15])
        )

        return state_contribution
    
    
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_error = 0.0
        self.previous_error = 0.0

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral_error += error * dt
        derivative_error = (error - self.previous_error) / dt if dt > 0 else 0.0
        self.previous_error = error

        output = (self.kp * error) + (self.ki * self.integral_error) + (self.kd * derivative_error)
        return output
    

class MPCController:
    def __init__(self):
        T = 0.1 # time horizon
        N = 50 # number of control intervals

        x = MX.sym('x', 16) # state vector
        u = MX.sym('u', 4)  # control input vector

        x_dot = DroneModel().state_transfer_function_casadi(x, u)

        L = x[6]**2 + x[7]**2 + x[8]**2 # simple cost function: sum of squares of euler angles

        F = self.rk4(T/N, x, u, x_dot, L)

        self.solver, self.w0, self.lbw, self.ubw, self.lbg, self.ubg = self.make_mpc_solver(F, N)

    def rk4(self, dt, x, u, xdot, L):
        
        M = 4 # RK4 steps per interval
        DT = dt / M
        f = Function('f', [x, u], [xdot, L])
        X0 = MX.sym('X0', 16)
        U = MX.sym('U', 4)
        X = X0
        Q = 0
        for j in range(M):
            k1, k1_q = f(X, U)
            k2, k2_q = f(X + DT/2 * k1, U)
            k3, k3_q = f(X + DT/2 * k2, U)
            k4, k4_q = f(X + DT * k3, U)
            X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
            Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
        F = Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])
        return F
    
    def make_mpc_solver(self, F, N):
        """
        Build an MPC solver using single-shooting.
        
        F           : CasADi integrator with signature F(x0=..., p=...)
        N           : Prediction horizon (number of control steps)
        MAX_VOLT    : Scalar. All controls satisfy [-MAX_VOLT, MAX_VOLT]
        """

        w   = []   # Decision variables (controls only)
        w0  = []   # Initial guess
        lbw = []   # Lower bounds on decision vars
        ubw = []   # Upper bounds on decision vars

        g   = []   # Constraints
        lbg = []
        ubg = []

        J = 0      # Cost function

        # Symbolic parameter: initial state (16-dim)
        X0 = MX.sym("X0", 16)

        # Start rollout from symbolic initial state
        Xk = X0

        # Build the shooting problem
        for k in range(N):

            # Control variable: 4-dim vector
            Uk = MX.sym(f"U_{k}", 4)
            w.append(Uk)

            # Bounds: each entry in Uk ∈ [-MAX_VOLT, MAX_VOLT]
            lbw += [-MAX_VOLT] * 4
            ubw += [ MAX_VOLT] * 4

            # Initial guess: 0 control
            w0  += [0] * 4

            # Propagate dynamics
            Fk = F(x0=Xk, p=Uk)
            Xk = Fk["xf"]

            # Add stage cost
            J += Fk["qf"]

        # Build NLP
        prob = {
            'f': J,
            'x': vertcat(*w),
            'g': vertcat(*g),
            'p': X0          # <- pass current state at solve time
        }

        # Create solver (heavy one-time cost)
        solver = nlpsol("solver", "ipopt", prob)

        return solver, w0, lbw, ubw, lbg, ubg

        
def mpc_step(solver, x_current, w0, lbw, ubw, lbg, ubg):
    sol = solver(
        x0=w0,
        lbx=lbw,
        ubx=ubw,
        lbg=lbg,  # empty
        ubg=ubg,  # empty
        p=x_current
    )

    # Return the entire predicted control sequence (N x 4)
    U_opt = sol["x"].full().reshape(-1, 4)
    return U_opt

if __name__ == "__main__":
    model = DroneModel()
    model.state[6] = 0.1 # small roll angle
    model_mpc = DroneModel()
    model_mpc.state[6] = 0.1 # small roll angle
    dt = 0.01
    pid = PIDController(0.08, 0.0, 0.04)
    mpc = MPCController()
    
    # Store euler angles for plotting
    time_values = []
    roll_values = []
    pitch_values = []
    yaw_values = []

    time_values_mpc = []
    roll_values_mpc = []
    pitch_values_mpc = []
    yaw_values_mpc = []

    control_input_mpc = None
    control_input_idx = 0
    control_input_sub_idx = 0  # Track position within each 2-timestep control block
    mpc_solve_count = 0



    
    for i in range(1000):
        roll_moment = pid.update(0.0, model.state[6], dt)
        print(f"Roll Moment Command: {roll_moment}")
        moments_vector = np.array([roll_moment, 0.0, 0.0, 9.8 * MASS])
        w_to_m = model.moments_to_omega_squared_matrix()
        omega_squared_vector = w_to_m @ moments_vector.reshape((4, 1))
        u = model.omega_squared_to_u(omega_squared_vector.flatten())
        print(f"Control Inputs: {u}")
        model.update(dt, u)
        
        # Store data for plotting
        time_values.append(i * dt)
        roll_values.append(model.state[6])
        pitch_values.append(model.state[7])
        yaw_values.append(model.state[8])
        
        print(f"Time: {i*dt:.2f} s, State: {model.state}")

    for i in range(1000):
        if mpc_solve_count == 100:
            # Solve MPC every 100 timesteps (50 control intervals x 2 timesteps each)
            control_input_mpc = mpc_step(mpc.solver, model_mpc.state, mpc.w0, mpc.lbw, mpc.ubw, mpc.lbg, mpc.ubg)
            control_input_idx = 0
            control_input_sub_idx = 0
            mpc_solve_count = 0
        else:
            mpc_solve_count += 1
        
        model_mpc.update(dt, u_mpc[0:4])
        
        # Store data for plotting
        time_values_mpc.append(i * dt)
        roll_values_mpc.append(model_mpc.state[6])
        pitch_values_mpc.append(model_mpc.state[7])
        yaw_values_mpc.append(model_mpc.state[8])
        
        print(f"MPC Time: {i*dt:.2f} s, State: {model_mpc.state}")
    
    # Plot euler angles - PID
    plt.figure(figsize=(12, 8))
    plt.plot(time_values, roll_values, 'b-', linewidth=2, label='Roll')
    plt.plot(time_values, pitch_values, 'r-', linewidth=2, label='Pitch')
    plt.plot(time_values, yaw_values, 'g-', linewidth=2, label='Yaw')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (radians)')
    plt.title('Drone Euler Angles Response - PID Controller')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

    # Plot euler angles - MPC
    plt.figure(figsize=(12, 8))
    plt.plot(time_values_mpc, roll_values_mpc, 'b-', linewidth=2, label='Roll')
    plt.plot(time_values_mpc, pitch_values_mpc, 'r-', linewidth=2, label='Pitch')
    plt.plot(time_values_mpc, yaw_values_mpc, 'g-', linewidth=2, label='Yaw')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (radians)')
    plt.title('Drone Euler Angles Response - MPC Controller')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

    # Plot comparison
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    axes[0].plot(time_values, roll_values, 'b-', linewidth=2, label='PID')
    axes[0].plot(time_values_mpc, roll_values_mpc, 'r--', linewidth=2, label='MPC')
    axes[0].set_ylabel('Roll (rad)')
    axes[0].set_title('Roll Angle Comparison')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    axes[1].plot(time_values, pitch_values, 'b-', linewidth=2, label='PID')
    axes[1].plot(time_values_mpc, pitch_values_mpc, 'r--', linewidth=2, label='MPC')
    axes[1].set_ylabel('Pitch (rad)')
    axes[1].set_title('Pitch Angle Comparison')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    axes[2].plot(time_values, yaw_values, 'b-', linewidth=2, label='PID')
    axes[2].plot(time_values_mpc, yaw_values_mpc, 'r--', linewidth=2, label='MPC')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Yaw (rad)')
    axes[2].set_title('Yaw Angle Comparison')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

    