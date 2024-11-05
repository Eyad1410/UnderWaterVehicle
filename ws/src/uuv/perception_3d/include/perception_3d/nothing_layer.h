#ifndef PERCEPTION_3D_NOTHING_LAYER_H_
#define PERCEPTION_3D_NOTHING_LAYER_H_

#include <perception_3d/sensor.h>

namespace perception_3d
{

class NothingLayer: public Sensor{

  public:
    NothingLayer();
    ~NothingLayer();
    virtual void onInitialize();
    virtual void selfClear();
    virtual void selfMark();
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getObservation();
    virtual void resetdGraph();
    virtual double get_dGraphValue(const unsigned int index);
    virtual bool isCurrent();

  private:
    
};

}//end of name space

#endif