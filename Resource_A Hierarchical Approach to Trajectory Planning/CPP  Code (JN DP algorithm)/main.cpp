
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <cmath>
#include <array>
#include <random>
#include <numeric>
#include <limits>
#include <tuple>
#include <iomanip>
#include <sstream>
#include <string>
#include <algorithm>
using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// structure of vehicle geometry
struct VehicleGeometrics {
    double vehicle_wheelbase;
    double vehicle_front_hang;
    double vehicle_rear_hang;
    double vehicle_width;
    double vehicle_residual;
    double vehicle_length;
};

// structure of vehicle dynamic parameters
struct VehicleKinematics {
    double vehicle_v_ref;
    double vehicle_a_max;
    double vehicle_phy_max; // Max rotation angle of front wheel
    double vehicle_w_max;   // Max rotational velocity of  front wheel
};

// structure of DP search parameters
struct DPParams {
    int num_t_grids;
    int num_s_grids;
    int num_l_grids;
    double unit_time;
    double max_unit_s;
    double min_unit_s;
    std::vector<double> ds;
    std::vector<double> dl;

    double w_collision;
    double w_Ncollision;
    double w_Njerky;
    double w_lat_change;
    double w_lon_change;
    double w_lon_achieved;
    double w_biasd;
};

// structure of instant index
struct TimelineIndex {
    int ind1;
    int ind2;
};

// structure of vehicle position and pose
struct ReferenceLineInfo {
    double xr;
    double yr;
    double lb;
    double rb;
    double theta;
};

// structure of vehicle polygon 
struct VehiclePolygon {
    std::vector<double> x, y;
};
struct ObstacleElement {
    std::vector<double> x, y, theta, s, l;
};

struct RoadBarrier {
    std::vector<double> x;
    std::vector<double> y;
};

struct BV {
    double s0;
    double l0;
    double x0;
    double y0;
    double theta0;
    double v0;
    double phy0;
};

struct State {
    double cost;
    int parent_s;
    int parent_l;
    double s_achieved;
};

//structure of Cubic spline interpolation coefficient 
struct SplineCoefficients {

    std::vector<double> a, b, c, d;

};

class CubicSpline {
private:
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> b_, c_, d_;     
public:
    CubicSpline(const std::vector<double>& x, const std::vector<double>& y) {
        int n = x.size() - 1;
        x_ = x;
        y_ = y;
        
        b_.resize(n);
        c_.resize(n + 1);
        d_.resize(n);
        
        std::vector<double> h(n);
        std::vector<double> delta(n);
        for (int i = 0; i < n; i++) {
            h[i] = x[i + 1] - x[i];
            delta[i] = (y[i + 1] - y[i]) / h[i];
        }
        
        std::vector<double> a(n - 1);
        std::vector<double> b(n - 1);
        std::vector<double> c(n - 1);
        std::vector<double> r(n - 1);
        
        for (int i = 0; i < n - 1; i++) {
            a[i] = h[i];
            b[i] = 2 * (h[i] + h[i + 1]);
            c[i] = h[i + 1];
            r[i] = 3 * (delta[i + 1] - delta[i]);
        }
        
        std::vector<double> m = solve_tridiagonal(a, b, c, r);
        
        c_[0] = 0;
        c_[n] = 0;
        for (int i = 1; i < n; i++) {
            c_[i] = m[i - 1];
        }
        
        for (int i = 0; i < n; i++) {
            b_[i] = delta[i] - h[i] * (2 * c_[i] + c_[i + 1]) / 3;
            d_[i] = (c_[i + 1] - c_[i]) / (3 * h[i]);
        }
    }
    
    double interpolate(double x) const {
        int i = 0;
        while (i < x_.size() - 2 && x > x_[i + 1]) {
            i++;
        }
        
        double dx = x - x_[i];
        
        return y_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
    }
    
private:
    std::vector<double> solve_tridiagonal(const std::vector<double>& a, 
                                        const std::vector<double>& b, 
                                        const std::vector<double>& c, 
                                        const std::vector<double>& r) {
        int n = r.size();
        std::vector<double> u(n);
        std::vector<double> gam(n);
        double bet = b[0];
        
        u[0] = r[0] / bet;
        for (int i = 1; i < n; i++) {
            gam[i] = c[i-1] / bet;
            bet = b[i] - a[i] * gam[i];
            u[i] = (r[i] - a[i] * u[i-1]) / bet;
        }
        
        for (int i = n-2; i >= 0; i--) {
            u[i] -= gam[i+1] * u[i+1];
        }
        
        return u;
    }
};

// global variables
//extern RoadBarrier road_barriers_;  //no used in main() function
RoadBarrier road_barriers_;
BV BV_;

// declaration of global variables
extern VehicleGeometrics vehicle_geometrics_;
extern VehicleKinematics vehicle_kinematics_;
extern DPParams dp_;
extern int Nobs;
extern double agv_vel;
extern std::vector<double> precise_timeline;
extern std::vector<TimelineIndex> precise_timeline_index;
extern std::vector<ObstacleElement> obstacles_;
std::vector<std::vector<std::vector<State>>> state_space;

// Function declaration
std::vector<double> linspace(double start, double end, int num_points);
VehiclePolygon CreateVehiclePolygon(double x, double y, double theta);
bool isPointInPolygon(double px, double py, const VehiclePolygon& polygon);
bool isPolygonOverlapping(const VehiclePolygon& V1, const VehiclePolygon& V2);
bool IsCurrentVehicleValid(const ObstacleElement& elem, const std::vector<ObstacleElement>& obstacles);

// instantiation of global variables
VehicleGeometrics vehicle_geometrics_;
VehicleKinematics vehicle_kinematics_;
DPParams dp_;
int Nobs = 5;
double agv_vel = 22.0;
std::vector<double> precise_timeline;
std::vector<TimelineIndex> precise_timeline_index;

// linspace() definition
std::vector<double> linspace(double start, double end, int num_points) {
    std::vector<double> result(num_points);
    double delta = (end - start) / (num_points - 1);
    for (int i = 0; i < num_points; ++i) {
        result[i] = start + i * delta;
    }
    return result;
}

// initialization of global variables
void initializeGlobals() {
    // initialize geometry parameters
    vehicle_geometrics_.vehicle_wheelbase = 2.88;
    vehicle_geometrics_.vehicle_front_hang = 0.96;
    vehicle_geometrics_.vehicle_rear_hang = 0.929;
    vehicle_geometrics_.vehicle_width = 1.942;
    vehicle_geometrics_.vehicle_residual = 0.1;
    vehicle_geometrics_.vehicle_length = vehicle_geometrics_.vehicle_wheelbase +
        vehicle_geometrics_.vehicle_front_hang +
        vehicle_geometrics_.vehicle_rear_hang;
    // initialize dynamic parameters
    vehicle_kinematics_.vehicle_v_ref = 20.0;
    vehicle_kinematics_.vehicle_a_max = 0.5;
    vehicle_kinematics_.vehicle_phy_max = 0.7;
    vehicle_kinematics_.vehicle_w_max = 0.5;

    // initialize DP search parameters
    dp_.num_t_grids = 5;
    dp_.num_s_grids = 7;
    dp_.num_l_grids = 8;
    dp_.unit_time = 2.0;
    dp_.max_unit_s = dp_.unit_time * vehicle_kinematics_.vehicle_v_ref;
    dp_.min_unit_s = 0;
    dp_.ds = linspace(dp_.min_unit_s, dp_.max_unit_s, dp_.num_s_grids);
    dp_.dl = linspace(0, 1, dp_.num_l_grids);
    //  initialize collision weighting parameters
    dp_.w_collision = 1.0;
    dp_.w_Ncollision = 10000;
    dp_.w_Njerky = 10;  
    dp_.w_lat_change = 1.0;
    dp_.w_lon_change = 1.0;
    dp_.w_lon_achieved = 10.0;
    dp_.w_biasd = 0.5;

    // initialize timeline
    precise_timeline = linspace(0, dp_.unit_time * dp_.num_t_grids, (dp_.unit_time * dp_.num_t_grids) / 0.05 + 1);
    precise_timeline_index.resize(dp_.num_t_grids);
    int num_timeline_points = precise_timeline.size();
    int indices_step = std::round(static_cast<double>(num_timeline_points - 1) / dp_.num_t_grids);
    for (int ii = 0; ii < dp_.num_t_grids; ++ii) {
        precise_timeline_index[ii] = { ii * indices_step , (ii + 1) * indices_step };
    }
}


std::vector<ObstacleElement> obstacles_(0);
void saveObstacles(const std::vector<ObstacleElement>& obstacles, const std::string& filename);
void loadObstacles(std::vector<ObstacleElement>& obstacles, const std::string& filename);

std::vector<ObstacleElement> GenerateObstacles(double agv_vel);

ReferenceLineInfo ProvideReferenceLineInfo(double s);

RoadBarrier GenerateRoadBarrierGrids();

void ConvertFrenetToCartesian(double s, double l, double& x, double& y, double& theta);
void SearchDecisionTrajectoryViaDp(std::vector<double>& x, std::vector<double>& y, std::vector<double>& theta);
std::pair<double, double> EstimateCostZero(int jj, int kk);
std::pair<double, double> ProvideRoadBound(double s);
double EvaluateCollision(int ind_time, std::vector<double> s_list, std::vector<double> l_list);
double MeasureCollision(std::vector<double> s, std::vector<double> l, std::vector<double> s_ego, std::vector<double> l_ego);
double EvaluateLateralChange(std::vector<double> s_list, std::vector<double> l_list);
double EvaluateAchievement(std::vector<double> s_list);
double EvaluateMeanAbs(std::vector<double> l_list);
std::pair<double, double> EstimateCost(std::vector<int> cur_state_ind, std::vector<int> next_layer_state_ind);
double EvaluateLongituteChange(int ind1, int ind2);

std::tuple<double, double, int> EstimateCostZeroJP(int jj, int kk);
std::tuple<double, double, int> EstimateCostJP(std::vector<int> cur_state_ind, std::vector<int> next_layer_state_ind);
std::tuple<double, std::vector<int>, std::vector<double >> EvaluateCollisionJP(int ind_time, std::vector<double> s_list, std::vector<double> l_list);
std::tuple<double, std::vector<int>, std::vector<double >> MeasureCollisionJP(std::vector<double> s, std::vector<double> l, std::vector<double> s_ego, std::vector<double> l_ego);

std::pair<std::vector<double>, std::vector<double>> ResampleSL(const std::vector<double>& s, const std::vector<double>& l);
SplineCoefficients calculateSplineCoefficients(const std::vector<double>& x, const std::vector<double>& y);
std::vector<double> splineInterpolate(const SplineCoefficients& coeffs, const std::vector<double>& x_orig, const std::vector<double>& x_new);

void ResampleSL_BSpline(const std::vector<double>& s, 
                       const std::vector<double>& l,
                       std::vector<double>& s_full,
                       std::vector<double>& l_full);

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> 
ConvertSlToXYTheta(const std::vector<double>& s, const std::vector<double>& l) ;

int main() {
    // initialize global variables
    initializeGlobals();
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    //obstacles_ = GenerateObstacles(agv_vel);
    //saveObstacles(obstacles_, "obstacles_.txt");
    loadObstacles(obstacles_, "obstacles_.txt");

    // Call Function of Generating Road Barrier Grids
    road_barriers_ = GenerateRoadBarrierGrids();

    BV_.s0 = 0;
    BV_.l0 = 0.78;
    ConvertFrenetToCartesian(BV_.s0, BV_.l0, BV_.x0, BV_.y0, BV_.theta0);
    BV_.v0 = 20;
    BV_.phy0 = 0.18;

    std::vector<double>  x,  y,  theta;

    SearchDecisionTrajectoryViaDp(x, y, theta);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time final  t cost = " << time_used.count() << " seconds. " << endl;

    // output and result validation
    std::cout << "Vehicle Length: " << vehicle_geometrics_.vehicle_length << std::endl;
    std::cout << "Max Unit S: " << dp_.max_unit_s << std::endl;
    std::cout << "Precise Timeline Size: " << precise_timeline.size() << std::endl;
    std::cout << "precise_timeline_index: " << precise_timeline_index[1].ind1 << "  _  " << precise_timeline_index[1].ind2 << std::endl;
    std::cout << " ɹ    У  " << std::endl;

    // ... other code

    return 0;
}


std::vector<ObstacleElement> GenerateObstacles(double agv_vel) {
    std::random_device rd;  
    std::mt19937 gen(rd()); 
    std::uniform_real_distribution<> dis(0, 1);

    //int Nfe = std::size(precise_timeline); 
    int Nfe = precise_timeline.size();
    double tf = dp_.unit_time * dp_.num_t_grids;
    //std::vector<ObstacleElement> obstacles(1);
    int ii = 0;

    double start_s = 10 + 100 * dis(gen);
    double end_s = start_s + agv_vel * dis(gen) * tf;
    double ds = (end_s - start_s) / (Nfe - 1);
    double offset = dis(gen) * 5 - 4;
    
    if (offset >= -1) {

        offset = -dis(gen) * 0.1;
    }
    else {
    
        offset = 2.95 - dis(gen) * 0.1;
    }
    
    std::vector<double> x, y, t;
    for (int jj = 0; jj < Nfe; ++jj) {
        ReferenceLineInfo info = ProvideReferenceLineInfo(start_s + ds * jj);
        x.push_back(info.xr - offset * std::cos(M_PI / 2 + info.theta));
        y.push_back(info.yr - offset * std::sin(M_PI / 2 + info.theta));
        t.push_back(info.theta);
    }
    
    ObstacleElement elem;
    elem.x = x;
    elem.y = y;
    elem.theta = t;
    elem.s = linspace(start_s, end_s, Nfe);
    elem.l = std::vector<double>(Nfe, offset);
    
    //obstacles_[ii] = elem;
    obstacles_.push_back(elem);
    
    while (ii < Nobs - 1) {
        start_s = 10 + 100 * dis(gen);
        end_s = start_s + agv_vel * dis(gen) * tf;
        ds = (end_s - start_s) / (Nfe - 1);
        offset = dis(gen) * 5 - 4;
    
        if (offset >= -1) {
            offset = -dis(gen) * 0.1;
        }
        else {
            offset = 2.95 - dis(gen) * 0.1;
        }
    
        x.clear(); y.clear(); t.clear();
        for (int jj = 0; jj < Nfe; ++jj) {
            ReferenceLineInfo info = ProvideReferenceLineInfo(start_s + ds * jj );
            x.push_back(info.xr - offset * std::cos(M_PI / 2 + info.theta));
            y.push_back(info.yr - offset * std::sin(M_PI / 2 + info.theta));
            t.push_back(info.theta);
        }
    
        elem.x = x;
        elem.y = y;
        elem.theta = t;
        elem.s = linspace(start_s, end_s, Nfe);
        elem.l = std::vector<double>(Nfe, offset);
    
        if (IsCurrentVehicleValid(elem, obstacles_)) {
            ++ii;
            obstacles_.push_back(elem);
        }
    }

    return obstacles_;
}

VehiclePolygon CreateVehiclePolygon(double x, double y, double theta) {
    VehiclePolygon V;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double vehicle_half_width = vehicle_geometrics_.vehicle_width * 0.5;

    double AX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta - vehicle_half_width * sin_theta;
    double BX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta + vehicle_half_width * sin_theta;
    double CX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta + vehicle_half_width * sin_theta;
    double DX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta - vehicle_half_width * sin_theta;
    double AY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta + vehicle_half_width * cos_theta;
    double BY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta - vehicle_half_width * cos_theta;
    double CY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta - vehicle_half_width * cos_theta;
    double DY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta + vehicle_half_width * cos_theta;

    V.x = { AX, BX, CX, DX, AX };
    V.y = { AY, BY, CY, DY, AY };

    return V;
}

bool isPointInPolygon(double px, double py, const VehiclePolygon& polygon) {
    bool inside = false;
    size_t j = polygon.x.size() - 1;
    for (size_t i = 0; i < polygon.x.size(); i++) {
        if (((polygon.y[i] > py) != (polygon.y[j] > py)) &&
            (px < (polygon.x[j] - polygon.x[i]) * (py - polygon.y[i]) / (polygon.y[j] - polygon.y[i]) + polygon.x[i])) {
            inside = !inside;
        }
        j = i;
    }
    return inside;
}

bool isPolygonOverlapping(const VehiclePolygon& V1, const VehiclePolygon& V2) {
    for (size_t i = 0; i < V1.x.size(); ++i) {
        if (isPointInPolygon(V1.x[i], V1.y[i], V2)) return true;
        if (isPointInPolygon(V2.x[i], V2.y[i], V1)) return true;
    }
    return false;
}

bool IsCurrentVehicleValid(const ObstacleElement& elem, const std::vector<ObstacleElement>& obstacles) {
    size_t Nalready_stored_obstacles = obstacles.size();
    size_t Nfe = elem.x.size();

    for (size_t ii = 0; ii < Nalready_stored_obstacles; ++ii) {
        const ObstacleElement& cur_obs = obstacles[ii];

        for (size_t jj = 0; jj < Nfe; ++jj) {
            double x_ego = elem.x[jj];
            double y_ego = elem.y[jj];
            double theta_ego = elem.theta[jj];

            double x = cur_obs.x[jj];
            double y = cur_obs.y[jj];
            double theta = cur_obs.theta[jj];

            VehiclePolygon V1 = CreateVehiclePolygon(x_ego, y_ego, theta_ego);
            VehiclePolygon V2 = CreateVehiclePolygon(x, y, theta);

            if (isPolygonOverlapping(V1, V2)) {
                return false;
            }
        }
    }
    return true;
}
   
   ReferenceLineInfo ProvideReferenceLineInfo(double s) {
       ReferenceLineInfo info;
   
       info.lb = -2 - cos(0.2 * s + 2.8166138) * 0.2;   
       info.rb = 5 + cos(0.78 * s - 0.8166138) * 0.15;    
       // ˮƽֱ ߶ 
       if ((s >= -10) && (s <= 40)) {
           info.xr = s;
           info.yr = 0;
           info.theta = 0;
       }
       else if ((s > 40) && (s <= 40 + 5 * M_PI)) {
       double ds = s - 40;
       double ang = ds / 10;
       double xc = 40;
       double yc = -10;
       info.xr = xc + 10 * cos(M_PI / 2 - ang);
       info.yr = yc + 10 * sin(M_PI / 2 - ang);
       info.theta = -ang;
       }
       else if ((s > 40 + 5 * M_PI) && (s <= 50 + 5 * M_PI)) {
           double ds = s - 40 - 5 * M_PI;
           info.theta = -M_PI / 2;
           info.xr = 50;
           info.yr = -10 - ds;
           }
       else if ((s > 50 + 5 * M_PI) && (s <= 50 + 10 * M_PI)) {
               double ds = s - 50 - 5 * M_PI;
               double ang = ds / 5;
               info.theta = -M_PI / 2 + ang;
               double xc = 55;
               double yc = -20;
               info.xr = xc + 5 * cos(M_PI + ang);
               info.yr = yc + 5 * sin(M_PI + ang);
               }
       else if ((s > 50 + 10 * M_PI) && (s <= 70 + 10 * M_PI)) {
                   double ds = s - 50 - 10 * M_PI;
                   info.theta = M_PI / 2;
                   info.xr = 60;
                   info.yr = -20 + ds;
                   }
       else if ((s > 70 + 10 * M_PI) && (s <= 70 + 15 * M_PI)) {
                       double ds = s - 70 - 10 * M_PI;
                       double ang = ds / 10;
                       double xc = 50;
                       double yc = 0;
                       info.xr = xc + 10 * cos(ang);
                       info.yr = yc + 10 * sin(ang);
                       info.theta = M_PI / 2 + ang;
                       }
       else if (s > 70 + 15 * M_PI) {
                           double ds = s - 70 - 15 * M_PI;
                           info.xr = 50 - ds;
                           info.yr = 10;
                           info.theta = M_PI;
                           }
   
       return info;
   }

   //Function of Generating Road Barrier Grids
   RoadBarrier GenerateRoadBarrierGrids() {
       std::vector<double> llx, lly, uux, uuy;
       double ds = 1.0;
       double d = -10;

       while (d < 200) {      // The planned range for the road grids is [-10m, 200m] for each time.
           ReferenceLineInfo info = ProvideReferenceLineInfo(d);
           double lb = info.lb;
           double rb = info.rb;
           double xr = info.xr;
           double yr = info.yr;
           double theta = info.theta;

           // Upper boundary of the road
           llx.push_back(xr - lb * cos(M_PI / 2 + theta));
           lly.push_back(yr - lb * sin(M_PI / 2 + theta));

           // Lower boundary of the road
           uux.push_back(xr - rb * cos(M_PI / 2 + theta));
           uuy.push_back(yr - rb * sin(M_PI / 2 + theta));

           d += ds; // go forward
       }

       RoadBarrier barriers;
       barriers.x = { llx.begin(), llx.end() };
       barriers.x.insert(barriers.x.end(), uux.begin(), uux.end());
       barriers.y = { lly.begin(), lly.end() };
       barriers.y.insert(barriers.y.end(), uuy.begin(), uuy.end());

       return barriers;
   }

   void ConvertFrenetToCartesian(double s, double l, double& x, double& y, double& theta) {
       double xr = 0, yr = 0, ang = 0, ds = 0;

       if (s <= 40) {
           xr = s;
           yr = 0;
           theta = 0;
       }
       else if (s > 40 && s <= 40 + 5 * M_PI) {
           ang = (s - 40) / 10;
           xr = 40 + 10 * std::cos(M_PI / 2 - ang);
           yr = -10 + 10 * std::sin(M_PI / 2 - ang);
           theta = -ang;
       }
       else if (s > 40 + 5 * M_PI && s <= 50 + 5 * M_PI) {
           ds = s - 40 - 5 * M_PI;
           theta = -0.5 * M_PI;
           xr = 50;
           yr = -10 - ds;
       }
       else if (s > 50 + 5 * M_PI && s <= 50 + 10 * M_PI) {
           ds = s - 50 - 5 * M_PI;
           ang = ds / 5;
           theta = -0.5 * M_PI + ang;
           xr = 55 + 5 * std::cos(M_PI + ang);
           yr = -20 + 5 * std::sin(M_PI + ang);
       }
       else if (s > 50 + 10 * M_PI && s <= 70 + 10 * M_PI) {
           ds = s - 50 - 10 * M_PI;
           theta = 0.5 * M_PI;
           xr = 60;
           yr = -20 + ds;
       }
       else if (s > 70 + 10 * M_PI && s <= 70 + 15 * M_PI) {
           ds = s - 70 - 10 * M_PI;
           ang = ds / 10;
           double xc = 50, yc = 0;
           xr = xc + 10 * std::cos(ang);
           yr = yc + 10 * std::sin(ang);
           theta = 0.5 * M_PI + ang;
       }
       else if (s > 70 + 15 * M_PI) {
           ds = s - 70 - 15 * M_PI;
           xr = 50 - ds;
           yr = 10;
           theta = M_PI;
       }

       x = xr - l * std::cos(M_PI / 2 + theta);
       y = yr - l * std::sin(M_PI / 2 + theta);
   }

   void SearchDecisionTrajectoryViaDp(std::vector<double>& x, std::vector<double>& y, std::vector<double>& theta) {
       const int NT = dp_.num_t_grids;
       const int NS = dp_.num_s_grids;
       const int NL = dp_.num_l_grids;

       state_space.resize(NT, std::vector<std::vector<State>>(NS, std::vector<State>(NL)));
       State node_default_status = { std::numeric_limits<double>::infinity(), -1, -1, -std::numeric_limits<double>::infinity() };

        //initial state space 
       for (int ii = 0; ii < NT; ++ii) {
           for (int jj = 0; jj < NS; ++jj) {
               for (int kk = 0; kk < NL; ++kk) {
                   state_space[ii][jj][kk] = node_default_status;
               }
           }
       }
      // Starting with Calculating starting time
      chrono::steady_clock::time_point tt1 = chrono::steady_clock::now();

//-------------------------------------------------------------------------
// Starting with traditonal DP algrithom
/* 
      for (int jj = 0; jj < NS; ++jj) {
          for (int kk = 0; kk < NL; ++kk) {
              auto [cost_val, s_achieved] = EstimateCostZero(jj, kk);
              state_space[0][jj][kk] = { cost_val, -999, -999, s_achieved };
          }
      }
   
      for (int ii = 0; ii < NT - 1; ++ii) {
          for (int jj = 0; jj < NS; ++jj) {
              for (int kk = 0; kk < NL; ++kk) {
                  std::vector<int> cur_state_ind = { ii, jj, kk };
                  for (int mm = 0; mm < NS; ++mm) {
                      for (int nn = 0; nn < NL; ++nn) {
                          std::vector<int> next_layer_state_ind = { ii + 1, mm, nn };
                          double delta_cost, s_achieved;
                          std::tie(delta_cost, s_achieved) = EstimateCost(cur_state_ind, next_layer_state_ind);
                          double cost = state_space[ii][jj][kk].cost + delta_cost;
                          if (cost < state_space[ii + 1][mm][nn].cost) {
                              state_space[ii + 1][mm][nn] = { cost, jj, kk, s_achieved };
                          }
                          //cout << "ii=" << ii << " jj=" << jj << " kk=" << kk << " mm=" << mm << " nn=" << nn << endl;
                      }
                  }
              }
          }
      }  
 */
// End of traditonal DP algrithom 

////-------------------------------------------------------------------------
// Starting with jump-point DP algrithom 
       for (int jj = 0; jj < NS; ++jj) {
           int kk = 0;
           //double cost_val;
           //int s_achieved, jOmmit;
           while (true) {
               auto [cost_val, s_achieved, jOmmit] = EstimateCostZeroJP(jj, kk);

               if (jOmmit == 0) {
                   state_space[0][jj][kk] = { cost_val, -999, -999, s_achieved };
               }
               else {
                   for (int uu = kk; uu <= kk + jOmmit; ++uu) {
                           state_space[0][jj][uu] = { cost_val, -999, -999, s_achieved };
                   }
                   kk += jOmmit; 
               }

               kk += 1;
               if (kk > NL - 1) {
                   break;
               }
           }
       }

       for (int ii = 0; ii <= NT - 2; ++ii) {
           for (int jj = 0; jj <= NS - 1; ++jj) {
               for (int kk = 0; kk <= NL - 1; ++kk) {
                   std::vector<int> cur_state_ind = { ii, jj, kk };
                   for (int mm = 0; mm <= NS - 1 ; ++mm) {
                        int nn = 0;
                        while (true) {
                             std::vector<int> next_layer_state_ind = { ii +1, mm, nn };
                             auto [delta_cost, s_achieved, jOmmit] = EstimateCostJP(cur_state_ind, next_layer_state_ind);
                             double cost = state_space[ii][jj][kk].cost + delta_cost;
                             if (jOmmit == 0) {
                                 if(cost < state_space[ii + 1][mm][nn].cost ){
                                       state_space[ii + 1][mm][nn] = { cost, jj, kk, s_achieved };
                                 }
                             }
                             else {
                                 for (int uu = nn; uu <= nn + jOmmit; ++uu) {
                                     if(cost < state_space[ii + 1][mm][uu].cost ){
                                          state_space[ii + 1][mm][uu] = { cost, jj, kk, s_achieved };
                                     }
                                 }
                                 nn += jOmmit; 
                             }
                             nn += 1;
                             if (nn > NL - 1) {
                                 break;
                             }
                        }
                   }
               }
           }
       } 
// End of jump-point DP algrithom 
//----------------------------------------------------------------------------------
// End of  calculating starting time
       chrono::steady_clock::time_point tt2 = chrono::steady_clock::now();
       chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(tt2 - tt1);
       std::cout << "solve time total tt time = " << time_used.count() << " seconds. " << endl;
        
       // For data testing
       // Building a text file to save the data with state space
       std::ofstream fid("statedata.txt");
       //std::ofstream fid("statedataJP.txt");
       if (!fid.is_open()) {
        
           std::cerr << "Failed to open file for writing." << std::endl;
       }

       // Writing data to file
       for (int ii = 0; ii < NT; ++ii) {
           for (int jj = 0; jj < NS; ++jj) {
               for (int kk = 0; kk < NL; ++kk) {
                    if ( ii == 0)
                    {
                      fid << ii + 1 << " "  // Make the index start from 1
                          << jj + 1 << " "
                          << kk + 1 << " "
                          << std::fixed << std::setprecision(6) << state_space[ii][jj][kk].cost << " "
                          << state_space[ii][jj][kk].parent_s << " "
                          << state_space[ii][jj][kk].parent_l << " "
                         << std::fixed << std::setprecision(6) << state_space[ii][jj][kk].s_achieved << "\n";

                    }
                    else
                    {
                      fid << ii + 1 << " "  // Make the index start from 1
                          << jj + 1 << " "
                          << kk + 1 << " "
                          << std::fixed << std::setprecision(6) << state_space[ii][jj][kk].cost << " "
                          << state_space[ii][jj][kk].parent_s  + 1 << " "
                          << state_space[ii][jj][kk].parent_l  + 1 << " "
                         << std::fixed << std::setprecision(6) << state_space[ii][jj][kk].s_achieved << "\n";

                    }
                }
           }
       }

       // close file
       fid.close();
       //For data testing

       //// Find the optimal path 
       double cur_best = std::numeric_limits<double>::infinity();
       int cur_best_s_ind = -1, cur_best_l_ind = -1;
       for (int jj = 0; jj < NS; ++jj) {
          for (int kk = 0; kk < NL; ++kk) {
              if (state_space[NT - 1][jj][kk].cost < cur_best) {
                  cur_best = state_space[NT - 1][jj][kk].cost;
                  cur_best_s_ind = jj;
                  cur_best_l_ind = kk;
              }
          }
       }

       int ind_s[NT], ind_l[NT];
       int child_s = cur_best_s_ind,  child_l = cur_best_l_ind;
       ind_s[NT-1] = child_s;   ind_l[NT-1] = child_l;
       for (int ii = NT - 2; ii >= 0; --ii) {
          auto& parent = state_space[ii + 1][child_s][child_l];
          ind_s[ii] = parent.parent_s;
          ind_l[ii] = parent.parent_l;
          child_s = parent.parent_s;
          child_l = parent.parent_l;
       }
       
       std::vector<double> s(1, BV_.s0);
       std::vector<double> l(1, BV_.l0); 
       for (int ii = 0; ii < NT; ++ii) {
          s.push_back(s.back() + dp_.ds[ind_s[ii]]);
          auto [lb, rb] = ProvideRoadBound(s.back());
          lb += (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual) * 0.5;
          rb -= (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual) * 0.5;
          l.push_back(lb + (rb - lb) * dp_.dl[ind_l[ii]]);
       }

       // Convert to Cartesian coordinates
       auto [rs, rl] = ResampleSL(s, l);

    //  pick out key points
       std::vector<double> s_original = {rs[0], rs[20], rs[40], rs[60], rs[80], rs[100], rs[120], rs[140], rs[160], rs[180], rs[200]};
       std::vector<double> l_original = {rl[0], rl[20], rl[40], rl[60], rl[80], rl[100], rl[120], rl[140], rl[160], rl[180], rl[200]};
       std::vector<double> s_full, l_full;
    
       ResampleSL_BSpline(s_original, l_original, s_full, l_full);

        // // Define the original x-coordinate
        // std::vector<double> t1 = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
 
        // // Define a new x-coordinate for interpolation
        // std::vector<double> t2;
        // for (double i = 0; i < 10; i += 0.05) {
        //     t2.push_back(i);
        // }
        // t2.push_back(10.0);
  
 
        // // Calculate the coefficients of cubic spline interpolation
        // SplineCoefficients s_coeffs = calculateSplineCoefficients(t1, s_original);
        // SplineCoefficients l_coeffs = calculateSplineCoefficients(t1, l_original);
 
        // // Interpolation using spline coefficients
        // std::vector<double> s_full = splineInterpolate(s_coeffs, t1, t2);
        // std::vector<double> l_full = splineInterpolate(l_coeffs, t1, t2);

       auto [final_x, final_y, final_theta] = ConvertSlToXYTheta(s_full, l_full);

       x = final_x;
       y = final_y;
       theta = final_theta;
    }

 // EstimateCostZero    ʵ  
 std::pair<double, double> EstimateCostZero(int jj, int kk) {
     double s0 = BV_.s0;
     double l0 = BV_.l0;
     double s1 = s0 + dp_.ds[jj];

     auto [lb, rb] = ProvideRoadBound(s1);
     lb += (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual) * 0.5;
     rb -= (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual) * 0.5;
     double l1 = lb + (rb - lb) * dp_.dl[kk];

     int Nfe = precise_timeline_index[0].ind2 - precise_timeline_index[0].ind1 + 1;
     std::vector<double> s_list(Nfe);
     std::vector<double> l_list(Nfe);

     for (int i = 0; i < Nfe; ++i) {
         s_list[i] = s0 + (s1 - s0) * i / (Nfe - 1);
         l_list[i] = l0 + (l1 - l0) * i / (Nfe - 1);
     }
     double val = 0;

     val  = dp_.w_collision * EvaluateCollision(0, s_list, l_list);
     val += dp_.w_lat_change * EvaluateLateralChange(s_list, l_list);
     val += dp_.w_lon_achieved * EvaluateAchievement(s_list);
     val += dp_.w_biasd * EvaluateMeanAbs(l_list);

     return { val, s1 };
 }

   // ProvideRoadBound     
   std::pair<double, double> ProvideRoadBound(double s)
   {
       double lb, rb;
       lb = -2 - std::cos(0.2 * s + 2.8166138) * 0.2;
       rb = 5 + std::cos(0.78 * s - 0.8166138) * 0.15;
       return { lb, rb };
   }

  // EvaluateCollision
  double EvaluateCollision(int ind_time, std::vector<double> s_list, std::vector<double> l_list) {
      double cost = 0.0;
      int intd1 = precise_timeline_index[ind_time].ind1; 
      int intd2 = precise_timeline_index[ind_time].ind2;

      for (size_t ii = 0; ii < Nobs; ++ii) {
          const std::vector<double>& s = obstacles_[ii].s;
          const std::vector<double>& l = obstacles_[ii].l;

          std::vector<double> s_sub(s.begin() + intd1, s.begin() + std::min(intd2 +1, (int)s.size()));
          std::vector<double> l_sub(l.begin() + intd1, l.begin() + std::min(intd2 +1, (int)l.size()));

         cost += MeasureCollision(s_sub, l_sub, s_list, l_list);
      }

      return cost;
  }

   // MeasureCollision
   double MeasureCollision( std::vector<double> s,  std::vector<double> l, std::vector<double> s_ego,  std::vector<double> l_ego) {
       int Nfe = s.size(); 
       std::vector<double> err_s(Nfe), err_l(0), lt(0), l_egot(0);

       std::transform(s.begin(), s.end(), s_ego.begin(), err_s.begin(),
           [](double a, double b) { return std::fabs(a - b); });

       for (size_t i = 0; i < err_s.size(); ++i) {
           if (err_s[i] <= (vehicle_geometrics_.vehicle_length + vehicle_geometrics_.vehicle_residual)) {
               lt.push_back(l[i]);
               l_egot.push_back(l_ego[i]);
           }
       }

       err_l.resize(lt.size());
       std::transform(lt.begin(), lt.end(), l_egot.begin(), err_l.begin(),
           [](double a, double b) { return std::fabs(a - b); });

       int j = 0;
       for (size_t i = 0; i < err_l.size(); ++i) {
           if (err_l[i] <= (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual)) {
               ++j;
           }
       }

       if (j > 0) {
           return dp_.w_Ncollision + j / Nfe;
       }
       else {
           return 0.0;
       }
   }

  // EvaluateLateralChange
      double EvaluateLateralChange(std::vector<double> s_list, std::vector<double> l_list) {
      double cost = 0.0;
      double ds = s_list.back() - s_list.front() + 0.00000001;
      double dl = std::abs(l_list.back() - l_list.front());
      cost = dl / ds;
      return cost;
  }

  // EvaluateAchievement
     double EvaluateAchievement( std::vector<double> s_list) {
      double cost = 0.0;
      cost = 1 - (s_list.back() - s_list.front()) / dp_.max_unit_s;
      return cost;
  }

     // EvaluateMeanAbs
      double EvaluateMeanAbs(std::vector<double> l_list) {
      double sum_abs = std::accumulate(l_list.begin(), l_list.end(), 0.0,
          [](double acc, double val) {
              return acc + std::abs(val);
          });

      return sum_abs / l_list.size();

  }

  std::pair<double, double> EstimateCost(std::vector<int> cur_state_ind, std::vector<int> next_layer_state_ind) {
      double s0 = state_space[cur_state_ind[0]][cur_state_ind[1]][cur_state_ind[2]].s_achieved;
      auto [lb, rb] = ProvideRoadBound(s0);
      lb += (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual) * 0.5;
      rb -= (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual) * 0.5;
      double l0 = lb + (rb - lb) * dp_.dl[cur_state_ind[2]];
      //
      double s1 = s0 + dp_.ds[next_layer_state_ind[1]] ;
      auto [lb1, rb1] = ProvideRoadBound(s1);
      lb1 += (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual) * 0.5;
      rb1 -= (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual) * 0.5;
      double l1 = lb1 + (rb1 - lb1) * dp_.dl[next_layer_state_ind[2]];
      
      int Nfe = precise_timeline_index[next_layer_state_ind[0]].ind2 - precise_timeline_index[next_layer_state_ind[0]].ind1 + 1;
      std::vector<double> s_list = linspace(s0, s1, Nfe);
      std::vector<double> l_list = linspace(l0, l1, Nfe);

      double val = 0;

      val = dp_.w_collision * EvaluateCollision(next_layer_state_ind[0], s_list, l_list);
      val += dp_.w_lat_change * EvaluateLateralChange(s_list, l_list);
      val += dp_.w_lon_achieved * EvaluateAchievement(s_list);
      val += dp_.w_lon_change * EvaluateLongituteChange(cur_state_ind[1], next_layer_state_ind[1]);
      val += dp_.w_biasd * EvaluateMeanAbs(l_list);

      return { val, s1 };
  }

  double EvaluateLongituteChange(int ind1, int ind2) {
      if (std::abs(ind1 - ind2) >= 3) {
          return dp_.w_Njerky;
      }
      else {
          return 0.0;
      }
  }
  void saveObstacles(const std::vector<ObstacleElement>& obstacles, const std::string& filename) {
      std::ofstream file(filename);
      if (!file.is_open()) {
          std::cerr << "Failed to open file for writing." << std::endl;
          return;
      }

      for (const auto& elem : obstacles) {
          for (size_t i = 0; i < elem.x.size(); ++i) {
              file << elem.x[i] << " "
                  << elem.y[i] << " "
                  << elem.theta[i] << " "
                  << elem.s[i] << " "
                  << elem.l[i] << std::endl;
          }
          file << std::endl;
      }

      file.close();
  }

  void loadObstacles(std::vector<ObstacleElement>& obstacles, const std::string& filename) {
      std::ifstream file(filename);
      if (!file.is_open()) {
          std::cerr << "Failed to open file for reading." << std::endl;
          return;
      }

      std::string line;
      ObstacleElement currentElement;

      while (std::getline(file, line)) {
          if (line.empty()) {
              if (!currentElement.x.empty()) {
                  obstacles.push_back(currentElement);
                  currentElement = ObstacleElement(); 
              }
              continue;
          }

          std::istringstream iss(line);
          double x, y, theta, s, l;
          if (iss >> x >> y >> theta >> s >> l) {
              currentElement.x.push_back(x);
              currentElement.y.push_back(y);
              currentElement.theta.push_back(theta);
              currentElement.s.push_back(s);
              currentElement.l.push_back(l);
          }
          else {
              std::cerr << "Failed to parse line: " << line << std::endl;
          }
      }

      if (!currentElement.x.empty()) {
          obstacles.push_back(currentElement);
      }

      file.close();
  }

   // EstimateCostZero
   std::tuple<double, double, int> EstimateCostZeroJP(int jj, int kk) {
      double val = 0;
      int j = 0;
      int NL = dp_.num_l_grids;
      double s0 = BV_.s0;
      double l0 = BV_.l0;
      double s1 = s0 + dp_.ds[jj];

      auto [lb, rb] = ProvideRoadBound(s1);
      lb += (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual) * 0.5;
      rb -= (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual) * 0.5;
      double l1 = lb + (rb - lb) * dp_.dl[kk];

      int Nfe = precise_timeline_index[0].ind2 - precise_timeline_index[0].ind1 + 1;
      std::vector<double> s_list(Nfe);
      std::vector<double> l_list(Nfe);

      for (int i = 0; i < Nfe; ++i) {
          s_list[i] = s0 + (s1 - s0) * i / (Nfe - 1);
          l_list[i] = l0 + (l1 - l0) * i / (Nfe - 1);
      }
 
      auto [nCost, nCol, LLV] = EvaluateCollisionJP(0, s_list, l_list);
    
      if (!nCol.empty() && (NL -1 > kk)) {
          double val = nCost;
          int nL0 = nCol[0]; 
          int nL1 = nCol.back();

          for (int i = kk + 1; i < NL; ++i) {
              double lltemp = lb + (rb - lb) * dp_.dl[i];
              std::vector<double> l_listtemp = linspace(l0, lltemp, Nfe);
             
              double minVal = std::min(l_listtemp[nL0], l_listtemp[nL1]) -
                  (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual);
              double maxVal = std::max(l_listtemp[nL0], l_listtemp[nL1]) +
                  (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual);

              if (LLV[1] < minVal || LLV[0] > maxVal) {
                  return  { val, s1, j }; 
              }
              else {
                  ++j;
              }
          }
          if (j == (NL-1 - kk)) {
              return  { val, s1, j }; 
          }
      }

      val = dp_.w_collision * nCost;
      val += dp_.w_lat_change * EvaluateLateralChange(s_list, l_list);
      val += dp_.w_lon_achieved * EvaluateAchievement(s_list);
      val += dp_.w_biasd * EvaluateMeanAbs(l_list);

      return { val, s1, j };
  }

  // EstimateCostJP
      std::tuple<double, double, int> EstimateCostJP(std::vector<int> cur_state_ind, std::vector<int> next_layer_state_ind) {
      double val = 0;
      int j = 0;
      int NL = dp_.num_l_grids;
      double s0 = state_space[cur_state_ind[0]][cur_state_ind[1]][cur_state_ind[2]].s_achieved;
      auto [lb, rb] = ProvideRoadBound(s0);
      lb += (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual) * 0.5;
      rb -= (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual) * 0.5;
      double l0 = lb + (rb - lb) * dp_.dl[cur_state_ind[2]];
      //
      double s1 = s0 + dp_.ds[next_layer_state_ind[1]];
      auto [lb1, rb1] = ProvideRoadBound(s1);
      lb1 += (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual) * 0.5;
      rb1 -= (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual) * 0.5;
      int kk = next_layer_state_ind[2];
      double l1 = lb1 + (rb1 - lb1) * dp_.dl[kk];

      int Nfe = precise_timeline_index[next_layer_state_ind[0]].ind2 - precise_timeline_index[next_layer_state_ind[0]].ind1 + 1;
      std::vector<double> s_list = linspace(s0, s1, Nfe);
      std::vector<double> l_list = linspace(l0, l1, Nfe);

      auto [nCost, nCol, LLV] = EvaluateCollisionJP(next_layer_state_ind[0], s_list, l_list);

      if (!nCol.empty() && (NL - 1 > kk)) {
          double val = nCost;
          int nL0 = nCol[0];
          int nL1 = nCol.back();

          for (int i = kk + 1; i < NL; ++i) {
              double lltemp = lb + (rb - lb) * dp_.dl[i];
              std::vector<double> l_listtemp = linspace(l0, lltemp, Nfe);

              double minVal = std::min(l_listtemp[nL0], l_listtemp[nL1]) -
                  (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual);
              double maxVal = std::max(l_listtemp[nL0], l_listtemp[nL1]) +
                  (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual);

              if (LLV[1] < minVal || LLV[0] > maxVal) {
                  return  { val, s1, j };               }
              else {
                  ++j;
              }
          }
          if (j == (NL - 1 - kk)) {
              return  { val, s1, j };           }
      }

      val = dp_.w_collision * nCost;
      val += dp_.w_lat_change * EvaluateLateralChange(s_list, l_list);
      val += dp_.w_lon_achieved * EvaluateAchievement(s_list);
      val += dp_.w_lon_change * EvaluateLongituteChange(cur_state_ind[1], next_layer_state_ind[1]);
      val += dp_.w_biasd * EvaluateMeanAbs(l_list);

      return { val, s1, j };
  }

     
  // EvaluateCollisionJP
      std::tuple<double, std::vector<int>, std::vector<double >> EvaluateCollisionJP(int ind_time, std::vector<double> s_list, std::vector<double> l_list) {
      double cost = 0.0;
      std::vector<int> nCol(0);
      std::vector<double> LLV(0);
      int ind1 = precise_timeline_index[ind_time].ind1; 
      int ind2 = precise_timeline_index[ind_time].ind2;

      for (size_t ii = 0; ii < Nobs; ++ii) {
          const std::vector<double>& s = obstacles_[ii].s;
          const std::vector<double>& l = obstacles_[ii].l;

          std::vector<double> s_sub(s.begin() + ind1, s.begin() + std::min(ind2 +1, (int)s.size()));
          std::vector<double> l_sub(l.begin() + ind1, l.begin() + std::min(ind2 +1, (int)l.size()));

          auto [local_nCost, local_nCol, local_LLV] = MeasureCollisionJP(s_sub, l_sub, s_list, l_list);

          if (!local_nCol.empty()) {
              cost = local_nCost;
              nCol = local_nCol;
              LLV = local_LLV;
              return { cost, nCol, LLV };
          }    
      }

      return { cost, nCol, LLV };
  }

  // MeasureCollisionJP
     std::tuple<double, std::vector<int>, std::vector<double >> MeasureCollisionJP(std::vector<double> s, std::vector<double> l, std::vector<double> s_ego, std::vector<double> l_ego) {
      double nCost;
      std::vector<int> nCol(0), nNum1(0), nNum2(0);
      std::vector<double > LLV(0);
      int Nfe = s.size(); 
      std::vector<double> err_s(Nfe), err_l(0), lt(0), l_egot(0);

      std::transform(s.begin(), s.end(), s_ego.begin(), err_s.begin(),
          [](double a, double b) { return std::fabs(a - b); });

          for (size_t i = 0; i < err_s.size(); ++i) {
          if (err_s[i] <= (vehicle_geometrics_.vehicle_length + vehicle_geometrics_.vehicle_residual)) {
              //st.push_back(s[i]);
              lt.push_back(l[i]);
              //s_egot.push_back(s_ego[i]);
              l_egot.push_back(l_ego[i]);
              nNum1.push_back(i);
          }
      }

      err_l.resize(lt.size());
      std::transform(lt.begin(), lt.end(), l_egot.begin(), err_l.begin(),
          [](double a, double b) { return std::fabs(a - b); });

      int j = 0;
      for (size_t i = 0; i < err_l.size(); ++i) {
          if (err_l[i] <= (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual))  {
              nNum2.push_back(i);
              ++j;
          }
      }
      double minL = 999.0, maxL = -999.0;
      if (j > 0) {
          nCost = dp_.w_Ncollision + j / Nfe;
          for (int index : nNum2) {
                  nCol.push_back(nNum1[index]);

                  if (lt[index] < minL) {
                      minL = lt[index];
                  }
                  if (lt[index] > maxL) {
                      maxL = lt[index];
                  }
          }
          LLV.insert(LLV.end(), { minL, maxL });
      }
      else {
          nCost = 0.0;

      }

      return { nCost, nCol, LLV };
  }

  // ResampleSL 
  std::pair<std::vector<double>, std::vector<double>> ResampleSL(const std::vector<double>& s, const std::vector<double>& l) {
    std::vector<double> s_full;
    std::vector<double> l_full;
 
    s_full.push_back(s[0]);
    l_full.push_back(l[0]);
 
    for (int ii = 0; ii < dp_.num_t_grids; ++ii) {
        int ind1 = precise_timeline_index[ii].ind1;
        int ind2 = precise_timeline_index[ii].ind2;
        int len = ind2 - ind1;
 
        std::vector<double> s_temp = linspace(s[ii], s[ii + 1], len + 1);
        std::vector<double> l_temp = linspace(l[ii], l[ii + 1], len + 1);
 
        //omit the fist vector element
        for (size_t i = 1; i < s_temp.size(); ++i) {
            s_full.push_back(s_temp[i]);
            l_full.push_back(l_temp[i]);
        }
    }
 
    return std::make_pair(s_full, l_full);
  //  return { s_full, l_full };
}

// ResampleSL_BSpline
void ResampleSL_BSpline(const std::vector<double>& s_original, 
                       const std::vector<double>& l_original,
                       std::vector<double>& s_full,
                       std::vector<double>& l_full) {
    std::vector<double> t1 = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<double> t2;
    for (double t = 0; t <= 10; t += 0.05) {
        t2.push_back(t);
    }
    t2.push_back(10.0);

    s_full.resize(t2.size());
    l_full.resize(t2.size());

    for (size_t i = 0; i < t1.size(); i++) {
        size_t idx = i * 20; 
        if (idx < t2.size()) {
            s_full[idx] = s_original[i];
            l_full[idx] = l_original[i];
        }
    }

    CubicSpline spline_s(t1, s_original);
    CubicSpline spline_l(t1, l_original);
    
    for (size_t i = 0; i < t2.size(); i++) {
        if (i % 20 != 0 || i >= t1.size() * 20) {  
            s_full[i] = spline_s.interpolate(t2[i]);
            l_full[i] = spline_l.interpolate(t2[i]);
        }
    }
}

SplineCoefficients calculateSplineCoefficients(const std::vector<double>& x, const std::vector<double>& y) {
    int n = x.size() - 1;
    std::vector<double> h(n), alpha(n);
 
    for (int i = 0; i < n; ++i) {
        h[i] = x[i + 1] - x[i];
    }
 
    for (int i = 1; i < n - 1; ++i) {
        alpha[i] = (3.0 / h[i] * (y[i + 1] - y[i])) - (3.0 / h[i - 1] * (y[i] - y[i - 1]));
    }
 
    alpha[0] = 0;
    alpha[n - 1] = 0;
 
    std::vector<double> l(n + 1), mu(n + 1), z(n + 1);
    l[0] = 1;
    mu[0] = 0;
    z[0] = 0;
 
    for (int i = 1; i < n; ++i) {
        l[i] = 2 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }
 
    std::vector<double> c(n + 1), b(n), d(n), a(n + 1);

    l[n] = 1;
    z[n] = 0;
    c[n] = 0;

    for (int j = n - 1; j >= 0; --j) {
        c[j] = z[j] - mu[j] * c[j + 1];
        b[j] = ((y[j + 1] - y[j]) / h[j]) - (h[j] * (c[j + 1] + 2 * c[j]) / 3.0);
        d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        a[j] = y[j];
    }
 
    SplineCoefficients coeffs = {a, b, c, d};
    return coeffs;
}
 
std::vector<double> splineInterpolate(const SplineCoefficients& coeffs, const std::vector<double>& x_orig, const std::vector<double>& x_new) {
    std::vector<double> y_new(x_new.size());
    int n = x_orig.size() - 1;
 
    for (size_t i = 0; i < x_new.size(); ++i) {
        double x = x_new[i];
        int k = 0;
        while (k < n && x > x_orig[k + 1]) {
            ++k;
        }
 
        double dx = x - x_orig[k];
        y_new[i] = coeffs.a[k] + dx * (coeffs.b[k] + dx * (coeffs.c[k] + dx * coeffs.d[k]));
    }
 
    return y_new;
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> 
ConvertSlToXYTheta(const std::vector<double>& s, const std::vector<double>& l) {
    int Nfe = s.size();
    std::vector<double> x(Nfe);
    std::vector<double> y(Nfe);
    std::vector<double> theta(Nfe);
    std::vector<double> thetaFrenet(Nfe);

    for (int ii = 0; ii < Nfe; ii++) {
        ConvertFrenetToCartesian(s[ii], l[ii], x[ii], y[ii], thetaFrenet[ii]);
    }

    std::vector<double> diff_x(Nfe);
    std::vector<double> diff_y(Nfe);
    
    for (int i = 0; i < Nfe - 1; i++) {
        diff_x[i] = x[i + 1] - x[i];
        diff_y[i] = y[i + 1] - y[i];
    }
    
    diff_x[Nfe - 1] = diff_x[Nfe - 2];
    diff_y[Nfe - 1] = diff_y[Nfe - 2];

    for (int i = 0; i < Nfe; i++) {
        theta[i] = std::atan2(diff_y[i], diff_x[i]);
    }

    std::vector<int> ind;
    for (int i = 0; i < Nfe; i++) {
        if (theta[i] <= -2.5) {
            ind.push_back(i);
        }
    }

    if (!ind.empty()) {
        std::vector<double> theta0;
        theta0.insert(theta0.end(), theta.begin(), theta.begin() + ind[0]);

        for (int jj = ind[0]; jj < Nfe; jj++) {
            if (theta[jj] <= -2.5) {
                theta0.push_back(theta[jj] + 2 * M_PI);
            } else {
                theta0.push_back(theta[jj]);
            }
        }
        theta = theta0;
    }

    return std::make_tuple(x, y, theta);
}
