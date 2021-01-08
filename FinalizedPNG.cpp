#include <vector>
#include <math.h>
#include <fstream>
#include <iterator>
#include <string>
#include <iostream>
#include <cmath>
using namespace std; // so you don't have to type "std::" before things
// As of 1/7/2021 this code works for the example of the target taking a 7 g turn. 

struct Point {
    double x = 0; // Sane defaults in case you forget to initialize your Point
    double y = 0;
    //generic Point structure

};
struct VelocityVector {
    double missile = 0; 
    double target = 0;
    //generic Velocity structure
};

struct Values{
    double separation=0;
    double beta_t=0;
    double beta_m=0;
    double sigma=0;
    double theta_t=0;
    double theta_m=0; //actually should be 0 so that missile points at target
    double k=0;
    double time = 0;
    double accel_target=0;
    double accel_missile=0;
    double dist;
};

struct Derivatives{
    double r_dot=0;
    double beta_t_dot=0;
    double beta_m_dot=0;
    double sigma_dot=0;
    double theta_t_dot=0;
    double theta_m_dot=0;
    double a_m_dot=0;
};

struct WorldState{
    Values values;
    Point missile_location;
    VelocityVector velocities;
    Point target_location;
};
Derivatives calculate_dddt(const WorldState & previous_state)
{
    double r_dot=previous_state.velocities.target*cos(previous_state.values.beta_t) - previous_state.velocities.missile*cos(previous_state.values.beta_m);

    double sigma_dot = (previous_state.velocities.target*sin(previous_state.values.beta_t) - previous_state.velocities.missile*sin(previous_state.values.beta_m))/previous_state.values.separation;
    
    double theta_t_dot= previous_state.values.accel_target/previous_state.velocities.target;

    double theta_m_dot = previous_state.values.k*sigma_dot;

    double beta_t_dot = theta_t_dot - sigma_dot;

    double beta_m_dot=previous_state.values.k*sigma_dot;

    double term_one = previous_state.values.k*previous_state.velocities.missile*previous_state.values.accel_target*cos(previous_state.values.beta_t)/previous_state.values.separation;
    double term_two = previous_state.values.accel_missile*(2*r_dot + previous_state.values.k*previous_state.velocities.missile*cos(previous_state.values.beta_m))/previous_state.values.separation;

    double a_m_dot=term_one-term_two;

    return Derivatives{
        r_dot,
        beta_t_dot,
        beta_m_dot,
        sigma_dot,
        theta_t_dot,
        theta_m_dot,
        a_m_dot
    };
}

Point UpdateMisPos(const WorldState & previous_state)
{
    const double time_step=0.01; // s
    return Point{
        previous_state.missile_location.x+time_step*previous_state.velocities.missile*cos(previous_state.values.theta_m), // calculates new x position
        previous_state.missile_location.y+time_step*previous_state.velocities.missile*sin(previous_state.values.theta_m), // calculates new y position
    };
}

Point UpdateTargPos(const WorldState & previous_state)
{
    const double time_step=0.01; // s
    return Point{
        previous_state.target_location.x+time_step*previous_state.velocities.target*cos(previous_state.values.theta_t), // calculates new x position
        previous_state.target_location.y+time_step*previous_state.velocities.target*sin(previous_state.values.theta_t), // calculates new y position
    };
}

Values Update_Values(const Values & prev_values, const Derivatives & drvs)
{
    const double time_step=0.01; // s
    double new_r = prev_values.separation + time_step*drvs.r_dot;
    double new_sigma=prev_values.sigma+ time_step*drvs.sigma_dot;
    double new_theta_m=prev_values.theta_m+time_step*drvs.theta_m_dot;
    double new_theta_t=prev_values.theta_t+time_step*drvs.theta_t_dot; 
    double new_beta_t=prev_values.beta_t+time_step*drvs.beta_t_dot;
    double new_beta_m=prev_values.beta_m+time_step*drvs.beta_m_dot;
    double new_accel_m=prev_values.accel_missile+time_step*drvs.a_m_dot;

    return Values{
        new_r,
        new_beta_t,
        new_beta_m,
        new_sigma,
        new_theta_t,
        new_theta_m,
        prev_values.k,
        prev_values.time,
        prev_values.accel_target,
        new_accel_m,
        prev_values.dist
    };
};
double separation_checker(const WorldState & previous_state)
{
    double xdif=previous_state.missile_location.x - previous_state.target_location.x;
    double ydif=previous_state.missile_location.y-previous_state.target_location.y;
    return sqrt(pow(xdif,2)+ pow(ydif,2));
}

WorldState EulerStep(const WorldState & previous_state)
{
    const double time_step=0.01; // s
    WorldState out=previous_state;
    Derivatives derivs=calculate_dddt(previous_state);// calculate first order time derivatives
    out.values=Update_Values(previous_state.values, derivs); // update our values
    out.missile_location=UpdateMisPos(previous_state); // update missile pos
    out.target_location=UpdateTargPos(previous_state);
    out.values.time=previous_state.values.time+time_step;
    out.values.dist = separation_checker(out);

    return out;
}
double determinescore(const WorldState & final_state)
{
    //TO:DO determine how to calculate the percent of a rectangle present within the explosion radius
    // this problem might be a bit trickier than I anticipated so for now I am just leaving the score as a function of explosion radius/final sep
    // returned value is < 1 if complete miss, > 1 if good hit 
    double finalsep=final_state.values.dist; // final separation of missile and target
    double target_size=11.1*9.64*0.6; //60% box of the size of a MIG-17
    double explosion_radius = 10; //m
    return explosion_radius/finalsep;

}
// double separation_checker(const WorldState & previous_state)
// {
//     double xdif=previous_state.missile_location.x - previous_state.target_location.x;
//     double ydif=previous_state.missile_location.y-previous_state.target_location.y;
//     return sqrt(pow(xdif,2)+ pow(ydif,2));
// }
int main()
{
    double sep;
    double beta_m;
    double beta_T;
    double sigma;
    double Theta_T;
    double Theta_m;
    double propo;
    double accelM=0;
    double accelT;
    double MissileVel;
    double TargetVel;
    cout << "Welcome to the ProNavi simulation! \n You will be asked to input an intial world state. \n";

    cout << "\nInput Theta_T: ";
    cin >> Theta_T;
    cout << "\nInput Theta_M: ";
    cin >> Theta_m;
    cout << "\nInput distance to target: ";
    cin >> sep;
    cout <<"\nInput Sigma: ";
    cin >> sigma;
    // cout << "\nInput Beta_T: ";
    // cin >> beta_T;
    // cout << "\nInput Beta_M:";
    // cin >> beta_m;
    cout << "\nInput proportionality constant: ";
    cin >> propo;
    cout <<"\nInput Target Acceleration: ";
    cin >> accelT;
    // cout << "\nInput Missile Acceleration: ";
    // cin >> accelM;

    cout <<"\nInput Missile Velocity: ";
    cin >> MissileVel;
    cout << "\nInput Target Velocity: ";
    cin >> TargetVel;
    vector<WorldState> worldstates;
    const WorldState initial_state{
        Values {sep,
                Theta_T-sigma,//beta_T
                Theta_m-sigma,//beta_M
                sigma,
                Theta_T,
                Theta_m,
                propo,
                0, //time
                accelT,
                0,//missile accel, we default it to be 0 initially
                sep}, //dist between targ and missile starts being equal to r
        Point{0,0},
        VelocityVector {MissileVel, TargetVel}, // initial velocity {missile, target} m/s
        Point{initial_state.values.separation*cos(initial_state.values.sigma), initial_state.values.separation*sin(initial_state.values.sigma)},// initial target loc {X, Y}
    };
    worldstates.push_back(initial_state);
    //For the simulation, I assume that the missile has 10 seconds of fuel. 
    for (double timesteps = 0; timesteps<1000; timesteps++){
        const WorldState & prev_state = worldstates.back();
        const WorldState & new_state = EulerStep(prev_state);
        double old_sep=separation_checker(prev_state);
        double new_sep=new_state.values.dist;
        // new_state.values.dist=new_sep;
        double missile_accel = abs(new_state.values.accel_missile/9.8);
        if (missile_accel >= 30){
            cout << "Missile acceleration greater than 30 g's, failed to track given these conditions";
            cout << endl;
            break;
        }
        else if (new_sep>=10){
            worldstates.push_back(new_state);
        } 
        else {
            cout << "Explosion radius has been entered\nWill now check next 1 seconds...";
            for(double extra_steps=0; extra_steps <100; extra_steps++)
            {
                const WorldState & prev_state = worldstates.back();
                const WorldState & new_state = EulerStep(prev_state);
                double old_sep=separation_checker(prev_state);
                double new_sep=new_state.values.dist;
                // new_state.values.dist=new_sep;
                double missile_accel = abs(new_state.values.accel_missile/9.8);
                if (missile_accel >= 30){
                    cout << "Missile acceleration greater than 30 g's, failed to track given these conditions";
                    cout << endl;
                    break;
                }
                else if (new_sep < old_sep)
                {
                    worldstates.push_back(new_state);
                }
                else{
                    cout << "PoCA Reached!\nPreparing to exit sim...";
                    break;
                }
            }
            break;
        }
        cout << new_state.values.dist << endl;
    
    }
    const WorldState & final_state=worldstates.back();
    double score=determinescore(final_state);
    double intercept_time = worldstates.size()*0.01; // calculates intercept time
    ofstream OutFile;
    string filename;
    cout << "Input a file name to save data to: ";//User Inputs filename
    cin >> filename;
    cout << "Filename is: " << filename;
    OutFile.open(filename);
    int j=0;
    OutFile << "Author: Spencer Rodgers, Date 1/6/2021, Score = " << score << ", Intercept Time = " << intercept_time << '\n';
    OutFile << "Missile X, Missile Y, Target X, Target Y, AccelM, Separation, Time \n";
    for (vector<WorldState> :: iterator it = worldstates.begin(); it!=worldstates.end(); it++)
    {

        WorldState value=worldstates.at(j);
        OutFile << value.missile_location.x << ',' << value.missile_location.y << ',' << value.target_location.x << ',' << value.target_location.y << ',' << value.values.accel_missile/9.8 << ','<< value.values.dist << ','<< value.values.time << '\n';
        j+=1;

    };
    OutFile.close();
    cout << "\nTask completed";
    cout<< endl;
    return 0;

}