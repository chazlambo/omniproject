#include <iostream>
#include "omnimagnet.hpp"

// No return type for constructor
OmniMagnet::OmniMagnet(double wirewidth, double wirelenin, double wirelenmid,	double wirelenout, double coresize, int pinin, int pinmid, int pinout,bool estimate)
{
	// wire_guage = wireguage;
	// wire_len_in = wirelenin;
	// wire_len_mid = wirelenmid;
	// wire_len_out = wirelenout;
	// core_size = coresize;
	// current_ << 0.0, 0.0 ,0.0;
 	SetProp(wirewidth, wirelenin, wirelenmid, wirelenout, coresize, pinin, pinout, pinout, estimate);

};

OmniMagnet::OmniMagnet(){
};

void OmniMagnet::SetProp(double wirewidth, double wirelenin, double wirelenmid,	double wirelenout, double coresize, int pinin, int pinmid, int pinout, bool estimate) 
{
	wire_width = wirewidth;
	wire_len_in = wirelenin;
	wire_len_mid = wirelenmid;
	wire_len_out = wirelenout;
	core_size = coresize;
	orientation_ = 0.0;
	current_ << 0.0, 0.0 ,0.0;
	mapping_ << 0.0, 0.0 ,0.0,
				0.0, 0.0 ,0.0,
				0.0, 0.0 ,0.0;
	D2A_pin_number << pinin, pinmid, pinout;
	estimate_ = estimate;
	// SetCurrent(current_);
};

void OmniMagnet::UpdateMapping() /* Generates the mapping (3X3) matrix*/ 
{
	if (estimate_) {
		// std::cout<<"here";

		// Generates mapping of dipole (Am^2) to current density (A/m^2)
		// Equation (13) from Omnimagnet Paper
		mapping_ = Eigen::MatrixXd::Identity(3,3)*((51.45 * pow (10.0, -3.0) * pow(0.115,4)));
		// mapping_ = Eigen::MatrixXd::Identity(3,3)*((51.45 * pow (10.0, -3.0) * (0.115))/(wire_width*wire_width));
		axis_rot_Z = (Eigen::AngleAxisd(orientation_*M_PI/180.0, Eigen::Vector3d::UnitZ()));
		// std::cout<<"Res_rot:    \n"<<axis_rot_Z * mapping_<<std::endl;
		axis_rot_Z.normalize();
		// std::cout<<"Res_rot:    \n"<<axis_rot_Z * mapping_<<std::endl;
		mapping_ = (axis_rot_Z * mapping_);
		
		//Eigen::Vector3d max_dipole = mapping_ *  
		//std::cout<<mapping_<<std::endl;
	}
	else{
		std::cout<<"No method, use the estimate method!!!!!";
	}
};


MAP OmniMagnet::GetMapping()
{
	return mapping_;	
};

double OmniMagnet::GetCoreSize()
{
	return core_size;	
};


// void OmniMagnet::SetCurrent(Eigen::Vector3d current)
// {
// 	current_= current;
// 	int retval;
// 	// std::cout<<"----------------------------------------------\n";
// 	// std::cout<<"set :    "<<current_[0]<<" , "<<current_[1]<<" , "<<current_[2]<<" \n";
// 	// std::cout<<"set :    "<<D2A_pin_number[0]<<" , "<<D2A_pin_number[1]<<" , "<<D2A_pin_number[2]<<" \n";
// 	retval = comedi_data_write(card_, subdev, D2A_pin_number[0],0, AREF_GROUND, CurrentD2A(current_[0]));
// 	// std::cout<<retval<<std::endl;
// 	retval = comedi_data_write(card_, subdev, D2A_pin_number[1],0, AREF_GROUND, CurrentD2A(current_[1]));
// 	//std::cout<<current_[1]<<std::endl;
// 	retval = comedi_data_write(card_, subdev, D2A_pin_number[2],0, AREF_GROUND, CurrentD2A(current_[2]));
// 	// std::cout<<retval<<std::endl;
// };



CUR OmniMagnet::GetCurrent()
{
	return current_;
};


double OmniMagnet::GetOrientation()
{
	return orientation_;	
};


void OmniMagnet::SetOrientation(double orientation)
{
	orientation_ = orientation;
	UpdateMapping();
};



//CUR OmniMagnet::Dipole2Current(DIP dipole) 
/* 
For a given dipole (3X1), calculates the needed current (3X1). The mapping matrix needs to get updated the Omni moves*/
/* 
{
	CUR result;
	MAPPING = mapping_.completeOrthogonalDecomposition().solve(dipole);	// calculates the needed current density for desired dipole 
	result = result*((wire_width*wire_width));							// calculates current needed
	// std::cout<< "The requested dipole:     \n" << dipole << std::endl<< "The generated dipole:     \n"<<  mapping_ * result<< std::endl;	
	// std::cout <<"result :     \n"<< result << std::endl;
	return result;
};
*/


POS OmniMagnet::GetPosition(){
	return position_;
};

//TEMP OmniMagnet::GetTemperature(){};
//POW OmniMagnet::GetPower(){};
void OmniMagnet::SetTemperature(TEMP temperature_){};
void OmniMagnet::SetPower(POW power_){};
void OmniMagnet::SetPosition(POS position){
	position_ = position;
};
// void OmniMagnet::SetOrientation( Eigen::Matrix3f orientation_ ){};
//DIP OmniMagnet::GetDipole(TARGET_LOC target_loc){};




void OmniMagnet::RotatingDipole(DIP init_dipole, AXIS_ROT axis_rot, double freq_, int dur)
{
	std::chrono::duration<double> test_duration = std::chrono::milliseconds(dur);
	current_time = std::chrono::high_resolution_clock::now();
	ref_time = current_time; 
	std::cout<<"Start_rot!!!!!\n";
	while ( current_time - ref_time <test_duration){
		current_time = std::chrono::high_resolution_clock::now();
		// std::cout<<"------\n";
		// std::cout<< ((Eigen::AngleAxisd(( 2.0 * M_PI * ((current_time - ref_time).count()/1000000000.0) * freq_), axis_rot)) * init_dipole) <<"\n";
		// SetCurrent(Dipole2Current((Eigen::AngleAxisd(( 2.0 * M_PI * ((current_time - ref_time).count()/1000000000.0) * freq_), axis_rot)) * init_dipole));
	};
	std::cout<<"End rot!!!!!\n";
	// std::cout<<"here";
};
