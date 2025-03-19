#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

typedef std::vector<double> Arr;
class PID {
	// i only know about PID because of sebastian thurn
	public:
		PID(double kp, double ki, double kd) : 
			ki(ki), kp(kp), kd(kd), prev_e{0.0}, integral{0.0} {}

		double compute(double st, double cu, double dt) {
	        double error = st - cu;
	        integral += error * dt;
	        double derivative = (error - prev_e) / dt;
	        double output = kp * error + ki * integral + kd * derivative;
	        prev_e = error;
	        return output;
		}

	private:
		double ki, kp, kd;
		// integreal, proportional, derivative gain
		double prev_e = 0.0;
		double integral = 0.0;
};

class Drone {
	// class Drone ties together the 3 PID controllers
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

			// these dampeners don't work
			// velocity[0] = velocity[0] > 1 ? 1 : velocity[0];
			// velocity[1] = velocity[1] > 1 ? 1 : velocity[1];
			// velocity[2] = velocity[2] > 1 ? 1 : velocity[2];

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
		// cannot initialize a member object with constructor 
		// arguments directly in the class definition 
};

// sine waves are fun
std::vector<std::vector<double>> gen_waypoints() {
    std::vector<std::vector<double>> waypoints;
    int num_points = 20;
    // 20 is a lucky number

    for (int i = 0; i < num_points; ++i) {
        double t = (2 * M_PI * i) / num_points;
        // evenly space the values of t along 2pi for a 
        // full sequence of sine vvalues

        waypoints.push_back({
        	t * 2, // change this to 5 for very fas, 0.5 for very slow
        	std::sin(t) * 5, // 5 for wide, dramatic swoops
        	std::sin(t) * 3 + 10 // 10 units in the air, and moderate swoops
        						 // up and down from the first way point
        });
    }
    return waypoints;
}


int main() {
    double dt = 0.1;
    double threshold_distance = 0.5;

    int num_steps = 500;
    int waypoint_index = 0;

	Arr st = {0,0,0};
	PID px(1.0, 0.1, 0.5);
	PID py(1.0, 0.1, 0.5);
	PID pz(1.0, 0.1, 0.5);

	Drone drone = Drone(st, px, py, pz);

    std::vector<std::vector<double>> waypoints = gen_waypoints();
    // should probably save these too
    
    std::vector<std::vector<double>> trajectory;
    trajectory.push_back(drone.pos()); // should be the origin

    for (int step = 0; step < num_steps && waypoint_index < waypoints.size(); ++step) {
	        Arr target = waypoints[waypoint_index];
	        drone.update(target, dt);

	        trajectory.push_back(drone.pos());

	        double distance = std::sqrt(
	            std::pow(position[0] - target[0], 2) + std::pow(position[1] - target[1], 2)+ std::pow(position[2] - target[2], 2));

	        // move on to next waypoint when done
	        if (distance < threshold_distance) { waypoint_index++; }

    } // for int step


    // save trajectory to CSV
    std::ofstream file("drone_trajectory.csv");
    for (const auto& pos : trajectory) { file << pos[0] << "," << pos[1] << "," << pos[2] << "\n"; }
    file.close();

    return 0;

} // main