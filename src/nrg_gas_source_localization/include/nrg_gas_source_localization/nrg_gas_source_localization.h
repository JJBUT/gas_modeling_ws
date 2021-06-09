#include <nrg_gas_utilities/nrg_gas.h>
#include <nrg_gas_utilities/GasSource.h>

#include <visualization_msgs/MarkerArray.h>

#include <vector>

namespace nrg_gas
{

struct Particle
{
    nrg_gas_utilities::GasSource source;
    double weight;
};

class NRGGasSourceLocalization: protected NRGGas
{
public:
    NRGGasSourceLocalization();
    void update( const GasConcentration& gas_measurement, const AnemometerMsg& wind_measurement );
private:
    void initialize(const int& np);
    void reweight( const GasConcentration& gas_measurement, const AnemometerMsg& wind_measurement );
    void resample();
    bool isDegenerate();

    std::vector<Particle> particle_set_;
    std::vector<std::vector<double>> state_space_;
    int np_min_;

    double R_; 
    double Q_;

    ros::NodeHandle nh_;

    ros::Publisher visualization_pub_;

    visualization_msgs::MarkerArray createParticleSetVisualization();

};

} // namespace nrg_gas
