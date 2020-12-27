#define CPPAD
#define CPPADCG
#include <ct/optcon/optcon.h>
namespace ct {
namespace optcon {

// the following path will be set automatically during building (do not copy this file manually!)
static const std::string exampleDir = "/home/chengdingan/ros_ws/src/control-toolbox-3.0.2/ct_optcon/examples";

}
}
// #include "plotResultsOscillator.h"

using namespace ct::core;
using namespace ct::optcon;


int main(int argc, char** argv)
{
    /*get the state and control input dimension of the oscillator. Since we're dealing with a simple oscillator,
	 the state and control dimensions will be state_dim = 2, and control_dim = 1. */
    const size_t state_dim = ct::core::SecondOrderSystem::STATE_DIM;
    const size_t control_dim = ct::core::SecondOrderSystem::CONTROL_DIM;

    typedef ct::core::ADCGScalar Scalar;
    


    Scalar w_n(50.0);
    std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim, Scalar>> oscillatorDynamics(
        new ct::core::tpl::SecondOrderSystem<Scalar>(w_n));
    

    std::shared_ptr<ct::core::ADCodegenLinearizer<state_dim, control_dim, Scalar>> adLinearizer(
        new ct::core::ADCodegenLinearizer<state_dim, control_dim, Scalar>(oscillatorDynamics)
    );
    
    
    adLinearizer->compileJIT();

    // std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost(
    //     new ct::optcon::TermQuadratic<state_dim, control_dim>());
    // std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> finalCost(
    //     new ct::optcon::TermQuadratic<state_dim, control_dim>());

    // bool verbose = true;
    // intermediateCost->loadConfigFile(ct::optcon::exampleDir + "/nlocCost.info", "intermediateCost", verbose);
    // finalCost->loadConfigFile(ct::optcon::exampleDir + "/nlocCost.info", "finalCost", verbose);


    // std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(
    //     new CostFunctionAnalytical<state_dim, control_dim>());
    // costFunction->addIntermediateTerm(intermediateCost);
    // costFunction->addFinalTerm(finalCost);


    // StateVector<state_dim, Scalar> x0;
    // x0.setZero();  // in this example, we choose a random initial state x0
    

    // ct::core::ControlledSystem<state_dim, control_dim, Scalar>::time_t tf(1.0);  // and a final time horizon in [sec]
    // // double tf(1.0);

    // // STEP 1-E: create and initialize an "optimal control problem"
    // ContinuousOptConProblem<state_dim, control_dim, Scalar> optConProblem(
    //     tf, x0, oscillatorDynamics, costFunction, adLinearizer);

}
