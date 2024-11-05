#include <mpc_critics/scoring_model.h>

namespace mpc_critics
{

class CollisionMinMaxModel: public ScoringModel{

  public:
    
    CollisionMinMaxModel();
    virtual double scoreTrajectory(base_trajectory::Trajectory &traj);

  protected:

    virtual void onInitialize();
};

}//end of name space
