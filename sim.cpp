#include <stdlib.h>

#include <vector>

class PID {
	public:
		PID(double ki, double kp, double kd) : 
			ki(ki), kp(kp), kd(kd), prev_e{0.0}, integral{0.0} {}
		PID() : kp(1.0), ki(0.1), kd(0.5) {}

		double* compute(double st[], double cu[], double dt) {
			double err[3] = {st[0] - cu[0], st[1] - cu[1], st[2] - cu[2]};
			double deriv[3] = {0.0};
			double *ans = new double[3];

			for (int i = 0; i<3; i++) {
				integral[i] += err[i] * dt;
			}

			for (int i = 0; i < 3; i++) { 
				deriv[i] = (err[i] - prev_e[i]) / dt;
			}

			for (int i = 0; i < 3; i++) {
				ans[i] = (kp * err[i]) + (ki * integral[i]) + (kd * deriv[i]);
			}

			for (int i = 0; i < 3; i++) {
				prev_e[i] = err[i];
			}

			return ans;
		}

	private:
		double ki, kp, kd;
		double prev_e[3] = {0.0};
		double integral[3] = {0.0};
		// integreal, proportional, derivative gain
};

class Drone {
public:
	Drone(double *init_pos, double *velocity, double kp, double ki, double kd)
		: position(init_pos), velocity(velocity), px(kp, ki, kd), py(kp, ki, kd), pz(kp, ki, kd) {}
	double update(double tgt[], double dt) {
		if (tgt == NULL) { return 0;}

        std::vector<double> pid_output = {pid_x.compute({target_position[0]}, {position[0]}, dt)[0],
                                          pid_y.compute({target_position[1]}, {position[1]}, dt)[0]};
        
        
		std::vector<double> pid_o = 
		pid_o[0] = px.compute(tgt, position, dt);
		pid_o[1] = py.compute(tgt, position, dt);
		pid_o[2] = pz.compute(tgt, position, dt);

		velocity[0] += pid_o[0] * dt;
		velocity[1] += pid_o[1] * dt;
		velocity[2] += pid_o[2] * dt;

		position[0] += velocity[0] * dt;
		position[1] += velocity[1] * dt;
		position[2] += velocity[2] * dt;
	}

private:
	double *position = new double[3];
	double *velocity = new double[3];
	PID px;
	PID py;
	PID pz;
};

int main() {
	PID px(1.0, 0.1, 0.5);
	PID py(1.0, 0.1, 0.5);
	PID pz(1.0, 0.1, 0.5);
	// PID p = PID(.1, .1, .1);
	// double st[3] = {1.0};
	// double cu[3] = {0.0};
	// double *ans;
	// ans = p.compute(st, cu, 0.01 );
	Drone d = Drone(0, 0, 0.5, 0.5, 0.1);// Drone(double *init_pos, double *velocity, double kp, double ki, double kd)
}