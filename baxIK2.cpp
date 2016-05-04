#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast

/*
Set which IKFast version you are using
The API calls are slightly different for versions > 54
*/

#define IK_VERSION 61
#include "baxter_right_arm_ikfast_solver.cpp"

//#define IK_VERSION 56
//#include "ikfast56.Transform6D.0_1_2_3_4_5.cpp"

//#define IK_VERSION 54
//#include "output_ikfast54.cpp"


//----------------------------------------------------------------------------//

#include <stdio.h>
#include <stdlib.h>
#include <time.h> // for clock_gettime()

float SIGN(float x);
float NORM(float a, float b, float c, float d);

#if IK_VERSION > 54
#define IKREAL_TYPE IkReal // for IKFast 56,61
#else
#define IKREAL_TYPE IKReal // for IKFast 54
#endif

int main(int argc, char** argv)
{
cout<<"HELLO\n";
}

extern "C"
{
float* compute(float x, float y, float z, float q1, float q2, float q3, float q4);
}


float* compute(float x, float y, float z, float q1, float q2, float q3, float q4)
{
    IKREAL_TYPE eerot[9],eetrans[3];

#if IK_VERSION > 54
    // for IKFast 56,61
    unsigned int num_of_joints = GetNumJoints();
    unsigned int num_free_parameters = GetNumFreeParameters();
#endif

#if IK_VERSION > 54
            // for IKFast 56,61
            IkSolutionList<IKREAL_TYPE> solutions;
#endif
IKREAL_TYPE vfree = -1.571;

            eetrans[0] = x;
            eetrans[1] = y;
            eetrans[2] = z;

            // Convert input effector pose, in w x y z quaternion notation, to rotation matrix. 
            // Must use doubles, else lose precision compared to directly inputting the rotation matrix.
            double qw = q1;
            double qx = q2;
            double qy = q3;
            double qz = q4;
            const double n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
            qw *= n;
            qx *= n;
            qy *= n;
            qz *= n;
            eerot[0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;  eerot[1] = 2.0f*qx*qy - 2.0f*qz*qw;         eerot[2] = 2.0f*qx*qz + 2.0f*qy*qw;
            eerot[3] = 2.0f*qx*qy + 2.0f*qz*qw;         eerot[4] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;  eerot[5] = 2.0f*qy*qz - 2.0f*qx*qw;
            eerot[6] = 2.0f*qx*qz - 2.0f*qy*qw;         eerot[7] = 2.0f*qy*qz + 2.0f*qx*qw;         eerot[8] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;
float* sols = (float *)malloc(100000 * sizeof(float));
bool bSuccess = false;
int count = 0;
do
{
            // for IKFast 56,61
            bSuccess = ComputeIk(eetrans, eerot, &vfree, solutions);
            std::vector<IKREAL_TYPE> solvalues(num_of_joints);
for(std::size_t i = 0; i < (int)solutions.GetNumSolutions(); i++)
{
const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
int this_sol_free_params = (int)sol.GetFree().size(); 
std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);

                for( std::size_t j = 0; j < solvalues.size(); ++j)
{
		*(sols + (count++)) = solvalues[j];
}
}

            vfree += 0.01;
        
       
}while(vfree < 2.094);
*(sols + 999) = count;
cout << count << endl;
         
return sols;
}

float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}

