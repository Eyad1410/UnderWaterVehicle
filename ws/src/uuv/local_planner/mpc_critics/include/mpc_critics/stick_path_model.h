
#include <mpc_critics/scoring_model.h>

namespace mpc_critics
{

class StickPathModel: public ScoringModel{

  public:
    
    StickPathModel();
    virtual double scoreTrajectory(base_trajectory::Trajectory &traj);

  protected:

    virtual void onInitialize();
};

}//end of name space
