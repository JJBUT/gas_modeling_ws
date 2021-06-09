#include <nrg_gas_source_localization/nrg_gas_source_localization.h>
#include <nrg_gas_utilities/nrg_gas_utilities.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <memory>

namespace nrg_gas
{


using nrg_gas_utilities::AnemometerMsg;
using nrg_gas_utilities::GasConcentration;

typedef message_filters::sync_policies::ApproximateTime<GasConcentration, AnemometerMsg> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

std::unique_ptr<NRGGasSourceLocalization> source_localization_node;

//Forward declaration
void measurementCallback( const nrg_gas_utilities::GasConcentrationConstPtr& gas, 
                          const nrg_gas_utilities::AnemometerMsgConstPtr& wind )
{
    GasConcentration non_constptr_gas;
    non_constptr_gas.concentration = gas->concentration;
    non_constptr_gas.header = gas->header;

    AnemometerMsg non_constptr_wind;
    non_constptr_wind.speed = wind->speed;
    non_constptr_wind.azimuth = wind->azimuth;
    non_constptr_wind.header = wind->header;

    source_localization_node->update( non_constptr_gas, non_constptr_wind);
} 

} // namespace nrg_gas



////////////////Entry Point/////////////////
int main(int argc, char** argv)
{
    ros::init(argc, argv, "nrg_gas_source_localization_node");
    ros::NodeHandle nh;
    message_filters::Subscriber<nrg_gas_utilities::GasConcentration> sub_gas; 
    message_filters::Subscriber<nrg_gas_utilities::AnemometerMsg> sub_wind;
    sub_wind.subscribe( nh, "/anemometer_data", 20 );
    sub_gas.subscribe( nh, "/gas_sensor_data",  20 );

    boost::shared_ptr<nrg_gas::Sync> sync;
    sync.reset(new nrg_gas::Sync(nrg_gas::MySyncPolicy(100), sub_gas, sub_wind));   
    sync->registerCallback(boost::bind(&nrg_gas::measurementCallback, _1, _2));

    nrg_gas::source_localization_node.reset(new nrg_gas::NRGGasSourceLocalization());
    
    ros::spin();
  
    return(0);
}
