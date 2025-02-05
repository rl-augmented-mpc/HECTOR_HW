#include "GaitGenerator.h"


Gait::Gait(int mpc_horizon, Vec2<int> dsp_durations, Vec2<int> ssp_durations)
: _mpc_horizon(mpc_horizon), _dsp_durations(dsp_durations.array()), _ssp_durations(ssp_durations.array())
{

    //********Gait Cycle Design for Bipeds********//
    // All the unit is control timestep (1 control step ticks 1 control_dt)
    // L:  |    Stance     |    Swing   |St |
    // R:  |   Swing   |       Stance       |
    //Time:|   SSP,L   |DSP|   SSP,R    |DSP|
    // Gait Cycle Length = 2*DSP + SSP,L + SSP,R
    //********Gait Cycle Design for Bipeds********//

    //For standing, set DSP = 0.5*MPC_horizon (actially it can be arbitrary value), SSP,L = 0, SSP,R = 0
    // Phase is within 0 to 1, you can control the how fast each gait ticks with stepping_frequency (1.0 means no speedup)

    // _mpc_table = std::make_shared<int[]>(2*_mpc_horizon); // NOT supported in CXX14
    _mpc_table = std::make_unique<int[]>(2*_mpc_horizon); 
    _gait_cycle_length = _dsp_durations.sum() + _ssp_durations.sum();

    _stance_durations << ssp_durations[0]+dsp_durations.sum(), ssp_durations[1]+dsp_durations.sum(); // left foot, right foot
    _stance_durations_phase = _stance_durations.cast<double>() / (double)_gait_cycle_length; 

    _swing_durations << ssp_durations[1], ssp_durations[0]; // left foot, right foot
    _swing_durations_phase = _swing_durations.cast<double>() / (double)_gait_cycle_length;

    _ssp_durations_phase = _ssp_durations.cast<double>() / (double)_gait_cycle_length;
    _dsp_durations_phase = _dsp_durations.cast<double>() / (double)_gait_cycle_length;

    _swing = _swing_durations; 
    _stance = _stance_durations;

}

Gait::~Gait() = default;


void Gait::update_parameter(Vec2<int> dsp_durations, Vec2<int> ssp_durations)
{
    _dsp_durations = dsp_durations.array();
    _ssp_durations = ssp_durations.array();
    _gait_cycle_length = _dsp_durations.sum() + _ssp_durations.sum();

    _stance_durations << ssp_durations[0]+dsp_durations.sum(), ssp_durations[1]+dsp_durations.sum(); // left foot, right foot
    _stance_durations_phase = _stance_durations.cast<double>() / (double)_gait_cycle_length; 

    _swing_durations << ssp_durations[1], ssp_durations[0]; // left foot, right foot
    _swing_durations_phase = _swing_durations.cast<double>() / (double)_gait_cycle_length;

    _ssp_durations_phase = _ssp_durations.cast<double>() / (double)_gait_cycle_length;
    _dsp_durations_phase = _dsp_durations.cast<double>() / (double)_gait_cycle_length;

    _swing = _swing_durations;
    _stance = _stance_durations;

    reset();
}

void Gait::updatePhase(float stepping_frequency)
{
    // update gait phase based on stepping frequency (default=1)
    int delta_t = 1; 
    _gait_phase += stepping_frequency * delta_t / _gait_cycle_length;
    if (_gait_phase > 1.0)
    {
        _gait_phase -= 1.0;
    }
}

// Subphase is phase of each foot in contact/swing duration

Vec2<double> Gait::getContactSubPhase() 
{

  Array2d contact_sub_phase = {0, 0};

  //For Left Foot: assumes that stance of the left foot starts from (SSP,L + DSP + SSP,R) timestep in the gait cycle
  if (_gait_phase < _ssp_durations_phase[0] + _dsp_durations_phase[0])
  {
    contact_sub_phase[0] = (double)(_gait_phase)/(double)(_ssp_durations_phase[0] + _dsp_durations_phase[0]);
  }
  else if (_gait_phase < _ssp_durations_phase[0] + _dsp_durations_phase[0] + _ssp_durations_phase[1])
  {
    contact_sub_phase[0] = 0; // swing
  }
  else
  {
    contact_sub_phase[0] = (double)(_gait_phase - (_ssp_durations_phase[0] + _dsp_durations_phase[0] + _ssp_durations_phase[1]))/(double)(_dsp_durations_phase[1]);
  }

  //For Right Foot: assumes that stance of the right foot starts from (SSP,L) timestep in the gait cycle
  if (_gait_phase < _ssp_durations_phase[0])
  {
    contact_sub_phase[1] = 0; //swing
  }
  else
  {
    contact_sub_phase[1] = (double)(_gait_phase - _ssp_durations_phase[0])/(double)(_dsp_durations_phase[0]+_ssp_durations_phase[1]+_dsp_durations_phase[1]);
  }

  return contact_sub_phase.matrix();
}

Vec2<double> Gait::getSwingSubPhase(){

  Array2d swing_sub_phase = {0, 0};

  //For Left Foot
  
  if (_ssp_durations_phase[0] + _dsp_durations_phase[0] <= _gait_phase && _gait_phase < _ssp_durations_phase[0] + _dsp_durations_phase[0] + _ssp_durations_phase[1])
  {
    swing_sub_phase[0] = (double)(_gait_phase - (_ssp_durations_phase[0] + _dsp_durations_phase[0])) / (double)(_ssp_durations_phase[1]);
  }
  else
  {
    swing_sub_phase[0] = 0; // stance
  }


  //For Right Foot
  if (_gait_phase < _ssp_durations_phase[0])
  {
    swing_sub_phase[1] = (double)(_gait_phase) / (double)(_ssp_durations_phase[0]);
  }
  else
  {
    swing_sub_phase[1] = 0; // stance
  }
  
  return swing_sub_phase.matrix();
    
}


int *Gait::mpc_gait(int iterations_between_mpc, float stepping_frequency){
  // compute contact sequence during mpc horizon
  for (int tMPC = 0; tMPC < _mpc_horizon; tMPC++)
  {
    int gait_time_step_from_phase = (int)(_gait_phase * (double)_gait_cycle_length);
    int gait_time_step_mpc_forward = (gait_time_step_from_phase + (int)(tMPC*iterations_between_mpc*stepping_frequency)) % _gait_cycle_length;

    if (gait_time_step_mpc_forward < _ssp_durations[0])
    {
      _mpc_table.get()[tMPC * 2 + 0] = 1; // left foot: stance
      _mpc_table.get()[tMPC * 2 + 1] = 0; // right foot: swing
    }
    else if (_ssp_durations[0] <= gait_time_step_mpc_forward && gait_time_step_mpc_forward < _ssp_durations[0] + _dsp_durations[0])
    {
      _mpc_table.get()[tMPC * 2 + 0] = 1; // left foot: stance
      _mpc_table.get()[tMPC * 2 + 1] = 1; // right foot: stance
    }
    else if (_ssp_durations[0] + _dsp_durations[0] <= gait_time_step_mpc_forward && gait_time_step_mpc_forward < _ssp_durations[0] + _dsp_durations[0] + _ssp_durations[1])
    {
      _mpc_table.get()[tMPC * 2 + 0] = 0; // left foot: swing
      _mpc_table.get()[tMPC * 2 + 1] = 1; // right foot: stance
    }
    else{
      _mpc_table.get()[tMPC * 2 + 0] = 1; // left foot: stance
      _mpc_table.get()[tMPC * 2 + 1] = 1; // right foot: stance
    }

  }

  return _mpc_table.get();
}