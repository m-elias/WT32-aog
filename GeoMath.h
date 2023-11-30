/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.
  Based on Theejs development.

  This library coordinates all other libraries.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef GEOMATH_H
#define GEOMATH_H

#include <cmath>

class Vector2{
public:
	Vector2(){
		Vector2(0,0);
	}

	Vector2(double _x, double _y){
		x=_x;
		y=_y;
	}
	
	double x=0, y=0;
	
	double angle(){
		// computes the angle in radians with respect to the positive x-axis
		return std::atan2( - y, - x ) + 3.14159265;
	}

	Vector2& add(Vector2 v){
		x += v.x;
		y += v.y;
		return *this;
	}
	
	Vector2 clone(){
		return Vector2(x,y);
	}
	
	void copy(Vector2 v){
		x = v.x;
		y = v.y;
	}
	
	double distanceTo(Vector2 v){
		return std::sqrt(distanceToSquared(v));
	}
	
	double distanceToSquared(Vector2 v){
		double dx = x-v.x;
		double dy = y-v.y;
		return dx*dx+dy*dy;
	}
	
	Vector2& divideScalar(double v){
		return multiplyScalar(1/v);
	}
	
	double length(){
		return std::sqrt(x*x+y*y);
	}
	
	Vector2& lerp(Vector2 v, double a){
		x+=(v.x-x)*a;
		y+=(v.y-y)*a;
		return *this;
	}
	
	Vector2& multiplyScalar(double v){
		x *=v;
		y *=v;
		
		return *this;
	}
	
	Vector2& normalize(){
		double l = length()? length() : 1;
		return divideScalar(l);
	}
	
	Vector2& rotateAround(Vector2 center, double angle){
		double c = std::cos(angle);
		double s = std::sin(angle);
		
		double xx = x-center.x;
		double yy = y-center.y;
		
		x = xx*c-yy*s+center.x;
		y = xx*s+yy*c+center.y;
		return *this;
	}
	
  Vector2& set(double _x, double _y){
    x = _x;
    y = _y;
    return *this;
  }

	Vector2& setLength(double length){
		return normalize().multiplyScalar(length);
	}
	
	Vector2& sub(Vector2& v){
		x -= v.x;
		y -= v.y;
		return *this;
	}
};

class Quaternion{
public:
  Quaternion(double _x=0, double _y=0, double _z=0, double _w=0){
    x = _x;
    y = _y;
    z = _z;
    w = _w;
  }
  
  double x=0, y=0, z=0, w=0;
  
  Quaternion& setFromEuler(double _x, double _y, double _z, const char* _order="XYZ"){
    // http://www.mathworks.com/matlabcentral/fileexchange/
    //  20696-function-to-convert-between-dcm-euler-angles-quaternions-and-euler-vectors/
    //  content/SpinCalc.m
    
    double c1 = std::cos(_x/2);
    double c2 = std::cos(_y/2);
    double c3 = std::cos(_z/2);
    
    double s1 = std::sin(_x/2);
    double s2 = std::sin(_y/2);
    double s3 = std::sin(_z/2);
    
    if(strcmp(_order,"XYZ") == 0){
      x = s1 * c2 * c3 + c1 * s2 * s3;
      y = c1 * s2 * c3 - s1 * c2 * s3;
      z = c1 * c2 * s3 + s1 * s2 * c3;
      w = c1 * c2 * c3 - s1 * s2 * s3;
    }else if(strcmp(_order,"YXZ") == 0){
      x = s1 * c2 * c3 + c1 * s2 * s3;
      y = c1 * s2 * c3 - s1 * c2 * s3;
      z = c1 * c2 * s3 - s1 * s2 * c3;
      w = c1 * c2 * c3 + s1 * s2 * s3;
    }else if(strcmp(_order,"ZXY") == 0){
      x = s1 * c2 * c3 - c1 * s2 * s3;
      y = c1 * s2 * c3 + s1 * c2 * s3;
      z = c1 * c2 * s3 + s1 * s2 * c3;
      w = c1 * c2 * c3 - s1 * s2 * s3;
    }else if(strcmp(_order,"ZYX") == 0){
      x = s1 * c2 * c3 - c1 * s2 * s3;
      y = c1 * s2 * c3 + s1 * c2 * s3;
      z = c1 * c2 * s3 - s1 * s2 * c3;
      w = c1 * c2 * c3 + s1 * s2 * s3;
    }else if(strcmp(_order,"YZX") == 0){
      x = s1 * c2 * c3 + c1 * s2 * s3;
      y = c1 * s2 * c3 + s1 * c2 * s3;
      z = c1 * c2 * s3 - s1 * s2 * c3;
      w = c1 * c2 * c3 - s1 * s2 * s3;
    }else if(strcmp(_order,"XZY") == 0){
      x = s1 * c2 * c3 - c1 * s2 * s3;
      y = c1 * s2 * c3 - s1 * c2 * s3;
      z = c1 * c2 * s3 + s1 * s2 * c3;
      w = c1 * c2 * c3 + s1 * s2 * s3;
    }

    return *this;   
  }
};

class Vector3{
public:
	Vector3(double _x=0.0, double _y=0.0, double _z=0.0){
		x=_x;
		y=_y;
		z=_z;
	}

	double x=0, y=0, z=0;
	
	Vector3& add(Vector3 v){
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	
	Vector3& applyQuaternion(Quaternion q){
		double xx = x;
		double yy = y;
		double zz = z;

		double qx = q.x;
		double qy = q.y;
		double qz = q.z;
		double qw = q.w;
		
		//calculate quat*vector
		double ix = qw*xx+qy*zz-qz*yy;
		double iy = qw*yy+qz*xx-qx*zz;
		double iz = qw*zz+qx*yy-qy*xx;
		double iw = -qx*xx-qy*yy-qz*zz;
		
		//calculate result*inverse quat
		x = ix*qw+iw*-qx+iy*-qz-iz*-qy;
		y = iy*qw+iw*-qy+iz*-qx-ix*-qz;
		z = iz*qw+iw*-qz+ix*-qy-iy*-qx;

		return *this;
	}
	
	Vector3 clone(){
		return Vector3(x,y,z);
	}
	
	double distanceTo(Vector3 v){
		return std::sqrt(distanceToSquared(v));
	}
	
	double distanceToSquared(Vector3 v){
		double dx = x-v.x;
		double dy = y-v.y;
		double dz = z-v.z;
		return dx*dx+dy*dy+dz*dz;
	}
	
	double dot(Vector3 v){
		return x*v.x+y*v.y+z*v.z;
	}
	
	Vector3& multiply(Vector3 v){
		x *= v.x;
		y *= v.y;
		z *= v.z;
		return *this;
	}
	
	Vector3& multiplyScalar(double v){
		x *=v;
		y *=v;
		z *=v;
		
		return *this;
	}
	
  Vector3& set(double _x, double _y, double _z){
    x = _x;
    y = _y;
    z = _z;
    return *this;
  }
	
	Vector3& sub(Vector3 v){
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}
	
	Vector3& subVectors(Vector3 a, Vector3 b){
		x = a.x-b.x;
		y = a.y-b.y;
		z = a.z-b.z;
		return *this;
	}
	
};

class Line3{
public:
	Line3():start(0,0,0), end(0,0,0){
    Line3(start, end);
	}
	
	Line3(Vector3 _start, Vector3 _end):start(_start.x,_start.y,_start.z),end(_end.x,_end.y,_end.z){
		Vector3 _startP(0,0,0);
		Vector3 _startEnd(0,0,0);
	}
	
    Vector3 start, end;

	void closestPointToPoint(Vector3 point, bool clampToLine, Vector3& target){
		double t = closestPointToPointParameter(point, clampToLine);
		delta(target).multiplyScalar(t).add(start);
	}
	
	double closestPointToPointParameter(Vector3 point, bool clampToLine){
		_startP.subVectors(point, start);
		_startEnd.subVectors(end, start);
		
		double startEnd2 = _startEnd.dot(_startEnd);
		double startEnd_startP = _startEnd.dot(_startP);
		
		double t = startEnd_startP/startEnd2;
		if(clampToLine){
			t = std::max(0.0,std::min(1.0,t));
		}
		return t;
	}

	Vector3& delta(Vector3& target){
		return target.subVectors(end, start);
	}

private:
    Vector3 _startP, _startEnd;
};
#endif
