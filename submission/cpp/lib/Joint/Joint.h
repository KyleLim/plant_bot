enum JOINT_TYPES {
    REVOLUTE,
    PRISMATIC
};

class Joint {
private:
    int i;              // Joint number
    JOINT_TYPES type;   // 0: Revolute, 1: Prismatic
    double theta;        // Angle between axis x_i-1 and x_i about axis z_i-1 (sign according to RHR)
    double alpha;        // Angle between axis z_i-1 and z_i about axis x_i (sign according to RHR)
    double d;            // Coordinate of origin O_i' along z_i-1
    double a;            // Displacement between origins O_i and O_i'

    Joint* next = nullptr;  // Next joint 
    Joint* prev = nullptr;  // Previous joint

public:
    Joint(int _i, JOINT_TYPES _type, double _theta, double _alpha, double _d, double _a) : i(_i), type(_type), theta(_theta), alpha(_alpha), d(_d), a(_a) { };

    friend class Manipulator;
};