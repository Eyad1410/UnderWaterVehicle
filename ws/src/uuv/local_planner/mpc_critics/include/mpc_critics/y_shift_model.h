
#include <mpc_critics/scoring_model.h>

namespace mpc_critics
{

class YShiftModel: public ScoringModel{

  public:
    
    YShiftModel();
    virtual double scoreTrajectory(base_trajectory::Trajectory &traj);

  protected:

    virtual void onInitialize();

  private:

};

}//end of name space
