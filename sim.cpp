#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

typedef std::vector<double> Arr;
class PID {
	public:
		PID(double ki, double kp, double kd) : 
			ki(ki), kp(kp), kd(kd), prev_e{0.0}, integral{0.0} {}

		double compute(double st, double cu, double dt) {
			double err = st - cu;
		    integral += err * dt;
		    double derivative = (err - prev_e) / dt;
		    double output = kp * err + ki * integral + kd * derivative;
		    prev_e = err;
		    return output;
		}

	private:
		double ki, kp, kd;
		// integreal, proportional, derivative gain
		double prev_e = 0.0;
		double integral = 0.0;
};

class Drone {
public:
	Drone(Arr init_pos,  PID& px, PID& py, PID& pz)
		: position(init_pos), velocity({0,0,0}), px(px), py(py), pz(pz) {}
	void update(Arr tgt, double dt) {

		Arr pid_o = {
			px.compute(tgt[0], position[0], dt),
			py.compute(tgt[1], position[1], dt),
			pz.compute(tgt[2], position[2], dt)
		};

		velocity[0] += pid_o[0] * dt;
		velocity[1] += pid_o[1] * dt;
		velocity[2] += pid_o[2] * dt;

		position[0] += velocity[0] * dt;
		position[1] += velocity[1] * dt;
		position[2] += velocity[2] * dt;
	}

	Arr pos() { return position; }

private:
	Arr position = {0, 0, 0};
	Arr velocity = {0, 0, 0};

	PID &px;
	PID &py;
	PID &pz;
};

// Generate Non-Linear Waypoints (Sine Wave Path)
std::vector<std::vector<double>> generateWaypoints() {
    std::vector<std::vector<double>> waypoints;
    int num_points = 20;
    for (int i = 0; i < num_points; ++i) {
        double t = (2 * M_PI * i) / num_points;
        waypoints.push_back({t * 2, std::sin(t) * 5});
    }
    return waypoints;
}


int main() {
	PID px(1.0, 0.1, 0.5);
	PID py(1.0, 0.1, 0.5);
	PID pz(1.0, 0.1, 0.5);

	Arr st = {0,0,0};
	Drone drone = Drone(st, px, py, pz);// Drone(double *init_pos, double *velocity, double kp, double ki, double kd)

    // Generate Waypoints
    std::vector<std::vector<double>> waypoints = generateWaypoints();

    // Simulation parameters
    double dt = 0.1;
    int num_steps = 500;
    int waypoint_index = 0;
    double threshold_distance = 0.5;
    
    // Store trajectory for visualization
    std::vector<std::vector<double>> trajectory;
    trajectory.push_back(drone.pos());

    std::cout << "Simulating drone flight...\n";

    for (int step = 0; step < num_steps; ++step) {
    	if (waypoint_index < waypoints.size()) {
	        Arr target = waypoints[waypoint_index];
	        drone.update(target, dt);

	        Arr position = drone.pos();
	        trajectory.push_back(position);

	        // Print every 10 steps
	        if (step % 10 == 0) {
	            std::cout << "Step " << step << ": Position = ("
	                      << position[0] << ", " << position[1] << ")\n";
	        }

	        // Compute distance to current waypoint
	        double distance = std::sqrt(
	            std::pow(position[0] - target[0], 2) + std::pow(position[1] - target[1], 2));

	        // Switch to next waypoint if close enough
	        if (distance < threshold_distance) {
	            waypoint_index++;
	        } // if
		} // outter if
    } // for int step

    std::cout << "Simulation complete. Saving results...\n";

    // Save Trajectory to CSV
    std::ofstream file("drone_trajectory.csv");
    for (const auto& pos : trajectory) {
        file << pos[0] << "," << pos[1] << "\n";
    }
    file.close();

    std::cout << "Results saved to 'drone_trajectory.csv'.\n";
    std::cout << "Use Python to visualize the path.\n";

    return 0;

} // main