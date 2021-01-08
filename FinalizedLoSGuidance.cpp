#include <vector>
#include <math.h>
#include <fstream>
#include <iterator>
#include <string>
#include <iostream>
#include <cmath>
using namespace std; // so you don't have to type "std::" before every vector

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
    double theta_t=0;
    double phi=0;// angle from missile to target  = 
    double alpha_t=0;
    double Rt=0; //initial distance from missile to target
    double Rm=0; // missile initial radius from origin shoudl be 0
    double alpha_m=0;
    double theta_m=0; //actually should be 0 so that missile points at target
    double separation=0;
    double time = 0;
};
struct Accels{
    double target=0;
    double missile=0;
};
struct Derivatives{
    double theta_t_dot=0;
    double phi_dot=0;
    double alpha_t_dot=0;
    double r_t_dot=0;
    double r_m_dot=0;
    double alpha_m_dot=0;
    double theta_m_dot=0;
};

struct WorldState{
    Values values;
    Accels accels;
    Point missile_location;
    VelocityVector velocities;
    Point target_location;
};
Derivatives calculate_dddt(const WorldState & previous_state)
{
    double theta_t_dot=previous_state.accels.target/previous_state.velocities.target;

    double phi_dot = previous_state.velocities.target*sin(previous_state.values.alpha_t)/previous_state.values.Rt;

    double alpha_t_dot=theta_t_dot - phi_dot;

    double r_t_dot = previous_state.velocities.target*cos(previous_state.values.alpha_t);

    double r_m_dot= previous_state.velocities.missile*cos(previous_state.values.alpha_m);

    double alpha_m_dot = (1/(previous_state.values.Rt*r_m_dot))*(r_m_dot*previous_state.velocities.target*sin(previous_state.values.alpha_t)+ previous_state.values.Rm*r_t_dot*alpha_t_dot - r_t_dot*previous_state.velocities.missile*sin(previous_state.values.alpha_m));

    double theta_m_dot = alpha_m_dot + phi_dot;

    return Derivatives{
        theta_t_dot, //r_dot
        phi_dot,
        alpha_t_dot,
        r_t_dot,
        r_m_dot,
        alpha_m_dot,
        theta_m_dot 
    };
}
double UpdateSeparation(const WorldState & previous_state)
{
    double xdif=previous_state.missile_location.x - previous_state.target_location.x;
    double ydif=previous_state.missile_location.y-previous_state.target_location.y;
    return sqrt(pow(xdif,2)+ pow(ydif,2));
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


    double new_theta_t= prev_values.theta_t+ time_step*drvs.theta_t_dot;

    double new_phi=prev_values.phi + time_step*drvs.phi_dot;

    double new_alpha_t = prev_values.alpha_t+ time_step*drvs.alpha_t_dot;

    double new_r_t= prev_values.Rt+ time_step*drvs.r_t_dot;

    double new_r_m=prev_values.Rm+time_step*drvs.r_m_dot;

    double new_alpha_m=prev_values.alpha_m+time_step*drvs.alpha_m_dot;

    double new_theta_m=prev_values.theta_m+time_step*drvs.theta_m_dot;

    return Values{
        new_theta_t,
        new_phi,
        new_alpha_t,
        new_r_t,
        new_r_m,
        new_alpha_m,
        new_theta_m,
        prev_values.time
    };
};

WorldState EulerStep(const WorldState & previous_state)
{
    const double time_step=0.01; // s
    WorldState out=previous_state;
    Derivatives derivs=calculate_dddt(previous_state);// calculate first order time derivatives
    out.accels.missile=previous_state.velocities.missile*derivs.theta_m_dot; //update accerations
    out.accels.target=previous_state.accels.target; //+2*9.8*sin(15*previous_state.values.time)
    out.values=Update_Values(previous_state.values, derivs); // update our values
    out.missile_location=UpdateMisPos(previous_state); // update missile pos
    out.target_location=UpdateTargPos(previous_state);
    out.values.time=previous_state.values.time+time_step;
    out.values.separation=UpdateSeparation(previous_state); // update target pos
    return out;
}
double determinescore(const WorldState & final_state)
{
    //TO:DO determine how to calculate the percent of a rectangle present within the explosion radius
    // this problem might be a bit trickier than I anticipated so for now I am just leaving the score as a function of explosion radius/final sep
    // returned value is < 1 if complete miss, > 1 if good hit 
    double finalsep=final_state.values.separation; // final separation of missile and target
    double target_size=11.1*9.64*0.6; //60% box of the size of a MIG-17
    double explosion_radius = 10; //m
    return explosion_radius/finalsep;

}
int main()
{
    double Theta_T;
    double Phi;
    double TargetAccel;
    // double Alpha_T;
    double Rt;
    // double Theta_M;
    cout << "Welcome to the simulation!\n You will be asked to input an intial world state. It is formatted as follows: \n";
    cout << "\nInput Theta_T: ";
    cin >> Theta_T;
    cout << "\nInput Phi: ";
    cin >> Phi;
    // cout << "\nInput Alpha_T: ";
    // cin >> Alpha_T;
    cout << "\nInput distance to target: ";
    cin >> Rt;
    // cout <<"\nInput Theta_M: ";
    // cin >> Theta_M;
    double MissileVel;
    double TargetVel;
    cout << "\nInput Missile Velocity: ";
    cin >> MissileVel;
    cout << "\nInput Target Velocity: ";
    cin >> TargetVel;
    cout <<"\nInput Target Accel: ";
    cin >> TargetAccel;
    vector<WorldState> worldstates;
    const WorldState initial_state{
        Values {Theta_T, // theta_t
                Phi,//phi
                Theta_T-Phi, //alpha_t
                Rt,//Rt,
                0,//Rm default should be 0 duh
                0,//alpha_m set to 0 if you want theta_m to be directly to target
                Phi, //theta_m
                Rt,//separation
                0}, //time 
        Accels{TargetAccel,0}, //target and missile
        Point{0,0},
        VelocityVector {MissileVel, TargetVel}, // initial velocity {missile, target} m/s
        Point{initial_state.values.Rt*cos(initial_state.values.phi), initial_state.values.Rt*sin(initial_state.values.phi)},// initial target loc {X, Y}
    };
    worldstates.push_back(initial_state);
    //For the simulation, I assume that the missile has 10 seconds of fuel. 
    for (double timesteps = 0; timesteps<1000; timesteps++){
        const WorldState & prev_state = worldstates.back();
        const WorldState & new_state = EulerStep(prev_state);
        double missile_accel = abs(new_state.accels.missile/9.8);
        if (missile_accel >= 30){
            cout << "Missile acceleration greater than 30 g's, failed to track given these conditions";
            cout << endl;
            break;
        }
        else if (new_state.values.separation >= 10){
            worldstates.push_back(new_state);
        } 
        else {
            cout << "Explosion Radius reached running for 1 more second";
            for(double extra_steps=0; extra_steps <100; extra_steps++)
            {
                const WorldState & prev_state = worldstates.back();
                const WorldState & new_state = EulerStep(prev_state);
                double old_sep=prev_state.values.separation;
                double new_sep=new_state.values.separation;
                // new_state.values.dist=new_sep;
                double missile_accel = abs(new_state.accels.missile/9.8);
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
                    cout << "\nPoCA Reached!\nPreparing to exit sim...\n";
                    break;
                }
            }
            break;
            cout << endl;
        }
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
    OutFile << "Author: Spencer Rodgers, Date 1/8/2021, Score = " << score << ", Intercept Time = " << intercept_time << '\n';
    OutFile << "Missile X, Missile Y, Target X, Target Y,AccelM, Separation, Time \n";
    for (vector<WorldState> :: iterator it = worldstates.begin(); it!=worldstates.end(); it++)
    {

        WorldState value=worldstates.at(j);
        OutFile << value.missile_location.x << ',' << value.missile_location.y << ',' << value.target_location.x << ',' << value.target_location.y << ','  << value.accels.missile/9.8 << ','<< value.values.separation << ','<< value.values.time << '\n';
        j+=1;

    };
    OutFile.close();
    cout << "\nTask completed";
    cout<< endl;
    return 0;

}