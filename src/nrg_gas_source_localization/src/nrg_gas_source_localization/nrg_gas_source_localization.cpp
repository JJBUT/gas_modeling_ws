#include <nrg_gas_source_localization/nrg_gas_source_localization.h>
#include <nrg_gas_utilities/nrg_gas_utilities.h>

using visualization_msgs::MarkerArray;


namespace nrg_gas
{

NRGGasSourceLocalization::NRGGasSourceLocalization()
: NRGGas(),
  state_space_( 4, std::vector<double>(2) )
{   
    visualization_pub_ = nh_.advertise<MarkerArray>("/particle_set_visualization", 10, true);

    private_nh_.getParam( "state_space_bounds_x", state_space_[0] ); 
    private_nh_.getParam( "state_space_bounds_y", state_space_[1] ); 
    private_nh_.getParam( "state_space_bounds_z", state_space_[2] ); 
    private_nh_.getParam( "state_space_bounds_rate", state_space_[3] ); 
    
    int np;
    private_nh_.param( "number_of_particles", np, 2000 ); 
    private_nh_.param( "minimum_number_of_particles", np_min_, 500 );
    ROS_DEBUG_COND_NAMED( np_min_> np, "NRGGasSourceLocalization::NRGGasSourceLocalization()", 
                          "Number of particles is less than the minimum number of particles, resampling will be triggered by default" ); 
    private_nh_.param( "measurement_noise", R_, 1.0 ); 
    private_nh_.param( "process_noise", Q_, 1.0 ); 

    initialize(np);
}

void NRGGasSourceLocalization::update( const GasConcentration& gas_measurement, 
                                       const AnemometerMsg& wind_measurement )
{
    visualization_pub_.publish(createParticleSetVisualization());

    reweight(gas_measurement, wind_measurement);
    if( isDegenerate() )
    {
        resample();
    }
    return;
}

void NRGGasSourceLocalization::initialize( const int& np )
{
    particle_set_.resize( np );
    for( auto& particle: particle_set_ )
    {
        particle.weight = 1.0/np;
        particle.source.position.header.frame_id = "map";
        auto rnv = uniform_rn(4);
        particle.source.position.point.x = state_space_[0][0] + rnv[0]*(state_space_[0][1] - state_space_[0][0]);
        particle.source.position.point.y = state_space_[1][0] + rnv[1]*(state_space_[1][1] - state_space_[1][0]);
        particle.source.position.point.z = state_space_[2][0] + rnv[2]*(state_space_[2][1] - state_space_[2][0]);
        particle.source.rate = state_space_[3][0] + rnv[3]*(state_space_[3][1] - state_space_[3][0]);
    }
    return;
}

void NRGGasSourceLocalization::reweight( const GasConcentration& gas_measurement, 
                                         const AnemometerMsg& wind_measurement )
{   
    TransformStamped map_to_anemometer_tf;
    try{
      map_to_anemometer_tf = tfBuffer_.lookupTransform( "map", 
                                                        wind_measurement.header.frame_id,
                                                        ros::Time(0), 
                                                        ros::Duration(0.1) );
    }
    catch ( tf2::TransformException &ex ) {
      ROS_WARN_NAMED( "NRGGasSourceLocalization::reweight()","%s",ex.what() );
      ros::Duration( 1.0 ).sleep();
      return;
    }

    double max_weight = 0.0;
    // Reweight particles based on similarity between measured concentration and theoretical concentration
    for( auto& particle: particle_set_ )
    {
        particle.weight *= gaussian( calculateConcentration( particle.source, 
                                                             wind_measurement, 
                                                             map_to_anemometer_tf ), 
                                     gas_measurement.concentration,
                                     R_ );   
        if( particle.weight > max_weight ) max_weight = particle.weight;
    }
    
    // Log-likelihood reweight to prevent numerical underflow
    double weight_sum = 0;
    for( auto& particle: particle_set_ )
    {
        particle.weight = std::exp( std::log(particle.weight/max_weight) );
        weight_sum += particle.weight; 
    }
    
    // Normalize the cdf to 1
    for( auto& particle: particle_set_ )
    {
        particle.weight /= weight_sum;
    }
    return;
}

void NRGGasSourceLocalization::resample()
{
    // See Multinomial Resampling in: "Particle filters and resampling techniques: Importance in computational complexity analysis" IEEE 2013
    const int np = particle_set_.size();
    std::vector<Particle> new_particle_set;
    new_particle_set.resize( np );
    
    // Find incremental sum of weight vector
    std::vector<double> weight_sum{0.0};    //Final size will be np+1 elements
    for( const auto& particle: particle_set_ )
    {
        weight_sum.push_back( weight_sum.back() + particle.weight ); 
    }
    
    // Pick a new particles from old particle set 
    for( auto& new_particle: new_particle_set )
    {        
        const double pick = uniform_rn();
        for(int i = 0; i < np; i++)
        {
            if( pick > weight_sum[i] &&
                pick < weight_sum[i+1] )
            {
                new_particle = particle_set_[i];
                new_particle.weight = 1.0/np;
                new_particle.source.position.point.x += uniform_rn()*Q_;
                new_particle.source.position.point.y += uniform_rn()*Q_;
                new_particle.source.position.point.z += uniform_rn()*Q_;
            }
        }
    }
    particle_set_ = std::move( new_particle_set );
    return;
}

bool NRGGasSourceLocalization::isDegenerate()
{
    double sum = 0;
    for( auto& particle: particle_set_)
    {
        sum += particle.weight*particle.weight;
    }
    
    if( 1.0/sum < np_min_ )
    {    
        // Degenerate
        return 1;
    }
    // Not degenerate
    return 0;
}

visualization_msgs::MarkerArray NRGGasSourceLocalization::createParticleSetVisualization()
{
    int id{0};
    MarkerArray particle_set_visualization;
    for( const auto& particle: particle_set_)
    {
        particle_set_visualization.markers.push_back( createSourceMarker(particle.source) );
        particle_set_visualization.markers.back().id = id;  //Each marker must have unique ID
        particle_set_visualization.markers.back().ns = "particles"; // Unique namespace 
        ++id;
    }
    return particle_set_visualization;
}

} // namespace nrg_gas