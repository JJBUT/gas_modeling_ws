#include <nrg_gas_utilities/nrg_gas.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>

namespace nrg_gas
{


NRGGas::NRGGas()
: private_nh_("~"),
  tfListener(tfBuffer_)
{
    private_nh_.param( "horizontal_dispersion_x", wp_.horizontal.x, 0.1 ); //TODO give these  guys default values 
    private_nh_.param( "horizontal_dispersion_y", wp_.horizontal.y, 0.1 ); 
    private_nh_.param( "horizontal_dispersion_z", wp_.horizontal.z, 0.1 ); 
    private_nh_.param( "vertical_dispersion_x", wp_.vertical.x, 0.1 );
    private_nh_.param( "vertical_dispersion_y", wp_.vertical.y, 0.1 );
    private_nh_.param( "vertical_dispersion_z", wp_.vertical.z, 0.1 );
}

double NRGGas::calculateConcentration( const GasSource &gs, const AnemometerMsg &wind, TransformStamped map_to_anemometer ) const
{  
    Point source_local_point;
    source_local_point = calculateSourceTransform(gs.position, wind, map_to_anemometer);

    if( source_local_point.x<0 )
    {
        //Downwind concentration is zero if the measurement point is not downwind of source 
        return 0.0;
    }else{
        std::cout<<"Downwind"<<std::endl;
        const double sy = wp_.horizontal.x*source_local_point.x*std::pow( 1.0+wp_.horizontal.y*source_local_point.x, -wp_.horizontal.z );
        const double sz = wp_.vertical.x*source_local_point.x*std::pow( 1.0+wp_.vertical.y*source_local_point.x, -wp_.vertical.z );
        const double expy = std::exp(-( source_local_point.y*source_local_point.y )/( 2*sy*sy ));  
        const double expz = std::exp(-( source_local_point.z*source_local_point.z )/( 2*sz*sz ));
        const double norm = ( gs.rate/wind.speed )/( 2*M_PI*sy*sz );
        return norm*expy*expz;
    }
}

Point NRGGas::calculateSourceTransform(const PointStamped& source, const AnemometerMsg& wind, const TransformStamped& map_to_anemometer) const
{
    Point t_in;
    // Source local translation
    t_in.x = map_to_anemometer.transform.translation.x - source.point.x;
    t_in.y = map_to_anemometer.transform.translation.y - source.point.y;
    t_in.z = map_to_anemometer.transform.translation.z - source.point.z;

    tf2::Quaternion q_wind_azimuth, q_base;
    
    q_wind_azimuth.setRPY( 0, 
                           0,
                           M_PI - wind.azimuth);       // Wind azimuth in anemometer frame:
    tf2::convert( map_to_anemometer.transform.rotation,
                  q_base );                                             // Anemometer rotation in map frame
    TransformStamped rot;
    rot.transform.rotation = tf2::toMsg( q_wind_azimuth*q_base );       // Wind azimuth transformed to map frame

    Point t_out;
    // Source local rotation
    tf2::doTransform(t_in, t_out, rot); 

    return t_out;
}

} // namespace nrg_gas
