#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H


class CParticle {
	private:
		double x_pos;
		double y_pos;
		double heading;

	public:
		CParticle() {

		}

		void MotionUpdate(const geometry_msgs::Twist::ConstPtr& update) {
			
			if (update->linear.x != 0)
			{
				(*this).x_pos += update->linear.x * cos(heading);
				(*this).y_pos += update->linear.x * sin(heading);
			}
			else if (update->angular.z > 0) {
				double x_centre = (*this).x_pos - update->linear.z * sin((*this).heading);
				double y_centre = (*this).y_pos + update->linear.z * cos((*this).heading);
				(*this).x_pos = x_centre + update->linear.z * sin((*this).heading + update->angular.z);
				(*this).y_pos = y_centre - update->linear.z * cos((*this).heading + update->angular.z);
				(*this).heading += update->angular.z;
			} else {
				double x_centre = (*this).x_pos + update->linear.z * sin((*this).heading);
				double y_centre = (*this).y_pos - update->linear.z * cos((*this).heading);
				(*this).x_pos = x_centre - update->linear.z * sin((*this).heading - update->angular.z);
				(*this).y_pos = y_centre + update->linear.z * cos((*this).heading - update->angular.z);
				(*this).heading -= update->angular.z;
			}


		}


};

class CParticleFilter {

public:
	


private:



};

#endif //PARTICLEFILTER_H