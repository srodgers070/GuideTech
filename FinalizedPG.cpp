#include <vector>
#include <math.h>
#include <fstream>
#include <iterator>
#include <string>
#include <iostream>
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

struct Angles{
    double R=0;
    double Beta=0;
    double k=1;
    double accel_m=0;
    double separtion=0;
};
struct Derivatives{
    double r_dot=0;
    double Beta_dot=0;
};
// define the world state 
struct WorldState{
    Angles angles;
    Point missile_location;
    VelocityVector velocities;
    Point target_location;
};
double UpdateSeparation(const WorldState & previous_state)
{
    double xdif=previous_state.missile_location.x - previous_state.target_location.x;
    double ydif=previous_state.missile_location.y-previous_state.target_location.y;
    return sqrt(pow(xdif,2)+ pow(ydif,2));
}
Derivatives calculate_dddt(const WorldState & previous_state, const WorldState & initial_state)
{
    double A=pow((1+cos(previous_state.angles.Beta)/(1+cos(initial_state.angles.Beta))),initial_state.angles.k);
    double B = pow(sin(previous_state.angles.Beta),2-initial_state.angles.k)/pow(sin(initial_state.angles.Beta),1-initial_state.angles.k);
    double beta_dot=(-1*previous_state.velocities.target/initial_state.angles.R)*A*B;
    double r_dot= previous_state.velocities.target*cos(previous_state.angles.Beta) - previous_state.velocities.missile;

    return Derivatives{
        r_dot, //r_dot
        beta_dot, //phi_dot
    };
}
Point UpdateMisPos(const Point & previous_loc, const Angles prev_angles, const VelocityVector & velocity)
{
    const double time_step=0.01; // s
    return Point{
        previous_loc.x+velocity.missile*cos(prev_angles.Beta)*time_step, // calculates new x position
        previous_loc.y+velocity.missile*sin(prev_angles.Beta)*time_step, // calculates new y position
    };
}
Point UpdateTargPos(const Point & previous_loc, const Angles prev_angles, const VelocityVector & velocity)
{
    const double time_step=0.01; // s
    return Point{
        previous_loc.x+velocity.target*time_step, // calculates new x position
        previous_loc.y, // calculates new y position
    };
}

Angles Update_Angles(const Angles & prev_angles, const Derivatives & drvs, const WorldState initial_state)
{
    const double time_step=0.01; // s
    double new_beta=prev_angles.Beta+drvs.Beta_dot*time_step;
    double A= pow((1+cos(initial_state.angles.Beta))/(1+cos(new_beta)),initial_state.angles.k);
    double B= pow(sin(new_beta)/sin(initial_state.angles.Beta),initial_state.angles.k-1);
    double new_R= initial_state.angles.R*A*B;
    double T1= -initial_state.velocities.missile*initial_state.velocities.target*pow((1+cos(prev_angles.Beta)/(1+cos(initial_state.angles.Beta))),prev_angles.k)/initial_state.angles.R;
    double T2= (pow(sin(prev_angles.Beta),2-prev_angles.k)/(pow(sin(initial_state.angles.Beta),1-prev_angles.k)));
    double new_a = T1*T2;
    return Angles{
        new_R,
        new_beta,
        prev_angles.k,
        new_a,
        prev_angles.separtion
    };
};
WorldState EulerStep(const WorldState & previous_state, const WorldState & initial_state)
{
    const double time_step=0.01; // s
    WorldState out = previous_state;
    Derivatives derivs=calculate_dddt(previous_state,initial_state);// calculate first order time derivatives
    out.angles=Update_Angles(previous_state.angles, derivs, initial_state); // update our angles and separation
    out.missile_location=UpdateMisPos(previous_state.missile_location, previous_state.angles, previous_state.velocities); // update missile pos
    out.target_location=UpdateTargPos(previous_state.target_location, previous_state.angles,previous_state.velocities); // update target pos
    out.angles.separtion=UpdateSeparation(previous_state);
    return out;
}
double SeparationCheck(const WorldState & previous_state, const WorldState & current_state)
{
    double currentR=current_state.angles.R;
    double oldR=previous_state.angles.R;
    return currentR-oldR;
}

int main()
{
    vector<WorldState> worldstates;
    const WorldState initial_state{
        Angles {2000, // distance
                1.107, // Beta 
                1,// k
                0},
        Point{0,0},
        VelocityVector{600, 300}, // initial velocity {missile, target} m/s
        Point{initial_state.angles.R*cos(initial_state.angles.Beta), initial_state.angles.R*sin(initial_state.angles.Beta)}// initial target loc {X, Y}
    };
    worldstates.push_back(initial_state);
    //lets change the for loop to a while loop
    const WorldState & prev_state = worldstates.back();
    for (double timesteps = 0; timesteps<1000; timesteps++){
        const WorldState & prev_state = worldstates.back();
        const WorldState & new_state = EulerStep(prev_state,initial_state);
        double missile_accel = abs(new_state.angles.accel_m/9.8);
        if (missile_accel >= 30){
            cout << "Missile acceleration greater than 30 g's, failed to track given these conditions";
            cout << endl;
            break;
        }
        else if (new_state.angles.separtion >= 50){
            worldstates.push_back(new_state);
        } 
        else {
            cout << "Explosion Radius reached running for 1 more second";
            for(double extra_steps=0; extra_steps <100; extra_steps++)
            {
                const WorldState & prev_state = worldstates.back();
                const WorldState & new_state = EulerStep(prev_state,initial_state);
                double old_sep=prev_state.angles.separtion;
                double new_sep=new_state.angles.separtion;
                // new_state.values.dist=new_sep;
                double missile_accel = abs(new_state.angles.accel_m/9.8);
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
    ofstream OutFile;
    OutFile.open("PG_NMT-K1.csv");
    int j=0;
    OutFile << "Author: Spencer Rodgers, Date 1/8/2021\n";
    OutFile << "Missile X, Missile Y, Target X, Target Y,AccelM, Separation, Time \n";
    for (vector<WorldState> :: iterator it = worldstates.begin(); it!=worldstates.end(); it++)
    {

        WorldState value=worldstates.at(j);
        double timeis=j*0.01; //s
        OutFile << value.missile_location.x << ',' << value.missile_location.y << ',' << value.target_location.x << ',' << value.target_location.y << ',' << value.angles.accel_m/9.8  << ',' << value.angles.separtion << ',' << timeis << '\n';
        j+=1;

    };
    OutFile.close();
    cout << "Task completed";
    return 0;
// To Do: let's try a tail chase
}