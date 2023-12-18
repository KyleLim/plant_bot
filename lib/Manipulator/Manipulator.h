#include <Joint.h>
#include <Matrix.h>
#include <stdio.h>
#include <math.h>

class Manipulator {
    // Joint Linked-List; Note: joints are 1-indexed!
    //      0: nullptr state
    //      1: base == end
    //      2: base->next == end
    //      etc...
    int n_DOF = 0;
    Joint* base = nullptr;
    Joint* end = nullptr;


    // Matrix* Translation = nullptr;
    // Matrix* Jacobian = nullptr;
public:
    Manipulator() : n_DOF(0), base(nullptr), end(nullptr) { };
    // Manipulator(int _n_DOF) : n_DOF(_n_DOF) {


        // Translation = new Matrix(4, 4, 0.0);    // 4x4 translation matrix
        // Jacobian = new Matrix(6, n_DOF, 0.0);   // 6xn jacobian matrix

    // }
    ~Manipulator() {
        // Traverse joints (linked list) and delete them
        Joint* cur = base;
        Joint* temp = nullptr;
        while(cur != nullptr) {
            temp = cur;
            cur = cur->next;
            delete temp;
        }

        // // Delete allocated matrices if created
        // if (Translation != nullptr)
        //     delete Translation;
        // if (Jacobian != nullptr)
        //     delete Jacobian;
    }

    void add_Joint(JOINT_TYPES type, double theta, double alpha, double d, double a) {
        if (n_DOF == 0) {
            n_DOF += 1; 
            base = new Joint(n_DOF, type, theta, alpha, d, a);
            end = base;
        }
        else {
            n_DOF += 1;
            end->next = new Joint(n_DOF, type, theta, alpha, d, a);
            end->next->prev = end;
            end = end->next;
        }
    }

    Joint* getJoint(int i) {
        Joint* cur = base;
        while(cur != nullptr && cur->i != i) {
            cur = cur->next;
        }
        return cur;
    }

    void print(void) {
        printf("Manipulator: %d DOF\n", n_DOF);
        printf("- [  i, theta,    d,    a, alpha]\n");
        // Traverse joints (linked list) and print them
        Joint* cur = base;
        while(cur != nullptr) {
            // float printing is disabled for embedded devices ;-;
            printf("- [ %2d, %5d, %4d, %4d, %5d]\n", cur->i, static_cast<int>(cur->theta), static_cast<int>(cur->d), static_cast<int>(cur->a), static_cast<int>(cur->alpha));
            cur = cur->next;
        }
    }

    Matrix<double> getDHMatrix(int i) {
        if (i <= 0 || i > n_DOF) {
            printf("Error: DH Translation Matrix does not exist from frame %d!", i);
            Matrix<double> m_result(4, 4, 0.0);
            return m_result;
        }
        Joint* j = getJoint(i);
        double theta = j->theta;
        double alpha = j->alpha;
        double d = j->d;
        double a = j->a;

        Matrix<double> m_result(4, 4, {
            cos(theta), -1.0 * cos(alpha) * sin(theta),        sin(alpha) * sin(theta), a * cos(theta), 
            sin(theta),        cos(alpha) * cos(theta), -1.0 * sin(alpha) * cos(theta), a * sin(theta),
                   0.0,                     sin(alpha),                     cos(alpha),              d,
                   0.0,                            0.0,                            0.0,            1.0,
        });

        return m_result;
    }


    // Returns the Translation Matrix from desired end frame to base frame
    Matrix<double> getTranslation(int i_base, int i_end) {
        // Invalid input
        if (i_end <= i_base || i_end > n_DOF || i_base < 0) {
            printf("Error: Translation from frame %d to frame %d is invalid!", i_end, i_base);
            Matrix<double> m_result(4, 4, 0.0);
            return m_result;
        }

        Matrix<double> m_result = getDHMatrix(i_base+1);

        for(int i = i_base + 1; i < i_end; i++) {
            m_result.multiply(getDHMatrix(i + 1));
        }

        return m_result;
    }
    
};